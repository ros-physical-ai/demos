# Copyright (C) 2026 Franco Cipollone
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Topic-driven Cartesian differential IK servo.

Owns the entire robot-facing pipeline: builds a Pinocchio model from the latched
robot description, seeds the DifferentialIKSolver from ``/joint_states``, and at
a fixed rate solves IK toward the latest target pose received on ``target_pose``,
streaming joint positions to a JointGroupPositionController. The gripper setpoint
is received as a scalar on ``gripper_command`` and clamped to URDF limits. The
current commanded tool pose is republished on ``ee_pose`` so input adapters can
initialize/anchor without any Pinocchio code.

Frame handling
--------------
An incoming ``target_pose`` is interpreted in its ``header.frame_id`` and
transformed into the solver's universe frame (the URDF root link) using the
Pinocchio model frame placements, cached per ``frame_id``. For a fixed-base arm
these transforms are constant. Unknown frames are treated as ``root_frame`` with
a one-time warning. ``ee_pose`` is published in ``root_frame`` so adapters can
publish targets back in the same frame they receive.

Topics
------
Subscribes:
  ``/joint_states``        sensor_msgs/JointState
  ``/robot_description``   std_msgs/String  (transient-local / latched)
  ``target_pose``          geometry_msgs/PoseStamped  (Cartesian target)
  ``gripper_command``      std_msgs/Float64           (gripper setpoint, rad)

Publishes:
  ``ee_pose``              geometry_msgs/PoseStamped  (commanded tool pose)
  ``<command_topic>``      std_msgs/Float64MultiArray -> forward_position_controller
"""

from __future__ import annotations

import threading

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from pai_teleop_ik.ik_solver import DifferentialIKSolver
from pai_teleop_ik.ros_utils import wait_for_robot_description
from pai_teleop_ik.se3_filter import se3_lowpass


def _pose_to_se3(pose: Pose) -> "pin.SE3":
    """Convert a geometry_msgs/Pose to a Pinocchio SE3."""
    quat = pin.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
    translation = np.array([pose.position.x, pose.position.y, pose.position.z])
    return pin.SE3(quat.normalized().toRotationMatrix(), translation)


def _se3_to_pose(transform: "pin.SE3") -> Pose:
    """Convert a Pinocchio SE3 to a geometry_msgs/Pose."""
    quat = pin.Quaternion(transform.rotation)
    pose = Pose()
    pose.position.x = float(transform.translation[0])
    pose.position.y = float(transform.translation[1])
    pose.position.z = float(transform.translation[2])
    pose.orientation.x = float(quat.x)
    pose.orientation.y = float(quat.y)
    pose.orientation.z = float(quat.z)
    pose.orientation.w = float(quat.w)
    return pose


class IKServoNode(Node):
    """Cartesian IK servo driven by a target pose topic."""

    def __init__(self) -> None:
        """Declare parameters, build the solver, and start the control loop."""
        super().__init__("ik_servo")

        # Robot / IK configuration. ``arm_joints`` is robot-specific and required:
        # the servo is arm-agnostic, so the caller must list the joints to
        # actuate (every other joint is locked at neutral). Declared with dynamic
        # typing so it can default to an empty list without a particular-robot
        # default while still accepting a string array.
        self.declare_parameter(
            "arm_joints",
            [],
            ParameterDescriptor(
                dynamic_typing=True,
                description="Ordered joint names the IK actuates; all others are locked. Required.",
            ),
        )
        self.declare_parameter("gripper_joint", "gripper_joint")
        self.declare_parameter("ee_frame", "gripper_frame_link")
        self.declare_parameter("root_frame", "world")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("position_cost", 0.5)
        self.declare_parameter("orientation_cost", 0.1)
        self.declare_parameter("posture_cost", 1e-3)
        self.declare_parameter("lm_damping", 1e-2)
        self.declare_parameter("max_joint_velocity", 2.0)
        self.declare_parameter("max_joint_acceleration", 10.0)
        self.declare_parameter("qp_solver", "quadprog")
        self.declare_parameter("inactivity_timeout", 0.3)
        self.declare_parameter("target_lowpass_alpha", 0.5)
        # Topics.
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("target_pose_topic", "ik_servo/target_pose")
        self.declare_parameter("gripper_command_topic", "ik_servo/gripper_command")
        self.declare_parameter("ee_pose_topic", "ik_servo/ee_pose")
        self.declare_parameter("command_topic", "/forward_position_controller/commands")

        self._ee_frame = self.get_parameter("ee_frame").value
        self._root_frame = self.get_parameter("root_frame").value
        self._joint_states_topic = self.get_parameter("joint_states_topic").value
        self._control_rate = float(self.get_parameter("control_rate").value)
        self._inactivity_timeout = float(self.get_parameter("inactivity_timeout").value)
        self._lowpass_alpha = float(np.clip(self.get_parameter("target_lowpass_alpha").value, 1e-3, 1.0))
        arm_joints = list(self.get_parameter("arm_joints").value or [])
        if not arm_joints:
            raise RuntimeError(
                "The 'arm_joints' parameter is required: list the ordered joint names the IK "
                "should actuate (for example via the launch file). The servo is arm-agnostic and "
                "has no default arm."
            )

        # Build the FK/IK solver from the latched robot description.
        urdf_xml = wait_for_robot_description(self, self.get_parameter("robot_description_topic").value)
        self._solver = DifferentialIKSolver(
            urdf_xml,
            arm_joints,
            self._ee_frame,
            position_cost=float(self.get_parameter("position_cost").value),
            orientation_cost=float(self.get_parameter("orientation_cost").value),
            posture_cost=float(self.get_parameter("posture_cost").value),
            lm_damping=float(self.get_parameter("lm_damping").value),
            max_joint_velocity=float(self.get_parameter("max_joint_velocity").value),
            max_joint_acceleration=float(self.get_parameter("max_joint_acceleration").value),
            qp_solver=self.get_parameter("qp_solver").value,
        )

        # Gripper detection + command joint ordering (arm joints, then gripper).
        gripper_joint_param = self.get_parameter("gripper_joint").value
        self._gripper_joint: str | None = gripper_joint_param if gripper_joint_param else None
        self._has_gripper = self._gripper_joint is not None and self._solver.full_model.existJointName(
            self._gripper_joint
        )
        if self._gripper_joint and not self._has_gripper:
            self.get_logger().warn(
                f"gripper_joint '{self._gripper_joint}' not found in URDF; gripper omitted from commands."
            )
        self._command_joints = list(self._solver.joint_names)
        if self._has_gripper:
            self._command_joints.append(self._gripper_joint)
            self._gripper_min, self._gripper_max = self._solver.joint_position_limits(self._gripper_joint)
        else:
            self._gripper_min = self._gripper_max = 0.0

        # Precompute the universe->root transform and cache per-frame_id
        # universe->frame transforms used to resolve incoming target frames.
        self._warned_frames: set[str] = set()
        self._frame_cache: dict[str, pin.SE3] = {}
        self._T_world_root = self._frame_to_universe(self._root_frame)

        # Shared state (guarded by _lock).
        self._lock = threading.Lock()
        self._measured_q: np.ndarray | None = None
        self._gripper_position: float = 0.0
        self._gripper_setpoint: float | None = None
        self._ik_initialized = False
        self._target_se3: pin.SE3 | None = None  # in universe frame
        self._target_filtered: pin.SE3 | None = None
        self._last_target_time: rclpy.time.Time | None = None
        self._last_q_cmd: np.ndarray | None = None
        self._warned_missing: set[str] = set()

        # ROS I/O.
        self._command_pub = self.create_publisher(Float64MultiArray, self.get_parameter("command_topic").value, 10)
        self._ee_pose_pub = self.create_publisher(PoseStamped, self.get_parameter("ee_pose_topic").value, 10)
        self.create_subscription(JointState, self._joint_states_topic, self._joint_state_cb, 10)
        self.create_subscription(PoseStamped, self.get_parameter("target_pose_topic").value, self._target_cb, 10)
        self.create_subscription(Float64, self.get_parameter("gripper_command_topic").value, self._gripper_cb, 10)

        # Fixed-rate IK solve and command publishing on its own callback group so
        # it is not starved by subscription callbacks.
        self._control_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / self._control_rate, self._control_loop, callback_group=self._control_group)
        self.get_logger().info(
            f"IK servo ready. ee_frame='{self._ee_frame}', root_frame='{self._root_frame}', "
            f"control_rate={self._control_rate:.0f} Hz."
        )

    # ------------------------------------------------------------------
    # Frame resolution
    # ------------------------------------------------------------------
    def _frame_to_universe(self, frame: str) -> "pin.SE3":
        """Return the constant SE3 from Pinocchio's universe frame to ``frame``.

        Cached per frame name. Falls back to identity (the universe frame) with a
        one-time warning if the frame is unknown.
        """
        if frame in self._frame_cache:
            return self._frame_cache[frame]
        if not self._solver.model.existFrame(frame):
            if frame not in self._warned_frames:
                self.get_logger().warn(f"frame '{frame}' not found in model; treating target as root frame.")
                self._warned_frames.add(frame)
            self._frame_cache[frame] = pin.SE3.Identity()
            return self._frame_cache[frame]
        data = self._solver.model.createData()
        pin.forwardKinematics(self._solver.model, data, pin.neutral(self._solver.model))
        pin.updateFramePlacements(self._solver.model, data)
        fid = self._solver.model.getFrameId(frame)
        transform = data.oMf[fid].copy()
        self._frame_cache[frame] = transform
        return transform

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState) -> None:
        """Cache the latest measured arm joint positions and gripper angle."""
        positions = dict(zip(msg.name, msg.position, strict=False))
        with self._lock:
            q = self._solver.configuration_from_dict(positions)
            if q is None:
                for joint in self._solver.joint_names:
                    if joint not in positions and joint not in self._warned_missing:
                        self.get_logger().warn(
                            f"Arm joint '{joint}' missing from {self._joint_states_topic}; "
                            "arm joints not available yet."
                        )
                        self._warned_missing.add(joint)
                return
            self._measured_q = q
            if self._has_gripper and self._gripper_joint in positions:
                self._gripper_position = positions[self._gripper_joint]

    def _target_cb(self, msg: PoseStamped) -> None:
        """Store the latest Cartesian target, resolved into the universe frame."""
        frame = msg.header.frame_id or self._root_frame
        transform_world_frame = self._frame_to_universe(frame)
        target_world = transform_world_frame * _pose_to_se3(msg.pose)
        with self._lock:
            self._target_se3 = target_world
            self._last_target_time = self.get_clock().now()

    def _gripper_cb(self, msg: Float64) -> None:
        """Store the latest gripper setpoint (clamped in the control loop)."""
        with self._lock:
            self._gripper_setpoint = float(msg.data)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        """Solve differential IK and stream joint commands at ``control_rate`` Hz."""
        dt = 1.0 / self._control_rate
        now = self.get_clock().now()
        q_cmd: np.ndarray | None = None
        gripper: float | None = None

        with self._lock:
            if self._measured_q is None:
                return  # no joint data yet

            # Seed the solver once; let IK integrate forward thereafter so the
            # AccelerationLimit can smooth velocity transitions between ticks.
            if not self._ik_initialized:
                self._solver.reset(self._measured_q)
                self._ik_initialized = True
                self.get_logger().info("IK solver initialized from measured joint states.")

            # Publish the current commanded tool pose in root_frame so adapters
            # can spawn/anchor their UI without any Pinocchio code.
            transform_world_ee = self._solver.forward_kinematics()
            transform_root_ee = self._T_world_root.inverse() * transform_world_ee
            ee_msg = PoseStamped()
            ee_msg.header.stamp = now.to_msg()
            ee_msg.header.frame_id = self._root_frame
            ee_msg.pose = _se3_to_pose(transform_root_ee)
            self._ee_pose_pub.publish(ee_msg)

            # Gripper: clamp the latest setpoint (if any) to the URDF limits.
            if self._has_gripper and self._gripper_setpoint is not None:
                self._gripper_position = float(np.clip(self._gripper_setpoint, self._gripper_min, self._gripper_max))

            # Inactivity gate: stop solving once the target has been at rest, so
            # residual error is not re-commanded every tick (steady-state jitter).
            active = self._last_target_time is not None and (
                (now - self._last_target_time).nanoseconds * 1e-9 < self._inactivity_timeout
            )
            if active and self._target_se3 is not None:
                # SE3 low-pass: step the filtered target toward the latest raw one.
                if self._target_filtered is None:
                    self._target_filtered = self._target_se3
                else:
                    self._target_filtered = se3_lowpass(self._target_filtered, self._target_se3, self._lowpass_alpha)
                try:
                    q_cmd = self._solver.solve(self._target_filtered, dt)
                    self._last_q_cmd = q_cmd
                except Exception as exc:
                    self.get_logger().warn(f"IK solve failed: {exc}", throttle_duration_sec=2.0)
            else:
                # Idle: command no arm motion. Tell the solver it is stationary so
                # the AccelerationLimit ramps from rest when tracking resumes, and
                # drop the filter so it re-seeds on the next target.
                self._solver.set_zero_velocity()
                self._target_filtered = None
                # Re-use the last commanded configuration so a gripper-only update
                # can still be published while the arm holds.
                q_cmd = self._last_q_cmd

            gripper = self._gripper_position if self._has_gripper else None

        if q_cmd is not None:
            self._publish_command(q_cmd, gripper)

    def _publish_command(self, q: np.ndarray, gripper: float | None) -> None:
        """Publish a Float64MultiArray to the forward_position_controller."""
        q_by_name = dict(zip(self._solver.joint_names, q, strict=True))
        if self._has_gripper and gripper is not None:
            q_by_name[self._gripper_joint] = gripper
        cmd = Float64MultiArray()
        cmd.data = [float(q_by_name[name]) for name in self._command_joints]
        self._command_pub.publish(cmd)


def main(args: list[str] | None = None) -> None:
    """Entry point for the IK servo node."""
    rclpy.init(args=args)
    node = IKServoNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

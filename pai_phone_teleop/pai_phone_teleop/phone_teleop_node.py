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

"""Integrated phone-teleop node: FK + teleop.Teleop WebXR bridge + differential IK.

Architecture
------------
* Main thread — ``executor.spin()`` handles all ROS callbacks.
* Daemon thread — ``teleop.Teleop.run()`` (blocking uvicorn/WebSocket server).
* ``_joint_state_cb`` (executor thread) — caches the latest measured joint
  positions and gripper angle under ``_lock``.  Does not touch the IK solver.
* ``_teleop_cb`` (uvicorn/asyncio thread) — called on every WebSocket frame.
  When the phone is moving, stores the commanded target (a 4×4 matrix in
  ``root_frame``) for the control loop and publishes visualization topics.
* ``_control_loop`` (executor timer, ``MutuallyExclusiveCallbackGroup``) —
  runs at ``control_rate`` Hz.  Seeds ``teleop.set_pose()`` with the current
  commanded EE pose expressed in ``root_frame``, then solves differential IK
  toward the latest phone target (converted back to Pinocchio's universe frame)
  and publishes joint commands to the ``forward_position_controller``.

Frame handling
--------------
Pinocchio FK always returns poses in its universe frame (= the URDF ``world``
root link).  The arm's ``base_link`` is mounted with a non-trivial offset and
yaw=π relative to ``world``, so without conversion the phone's translational
deltas would feel inverted.  ``_T_world_root`` (precomputed at init) is the
fixed SE3 from the universe frame to ``root_frame`` (default ``base_link``).
The control loop converts EE pose → root_frame for teleop seeding and converts
the phone target → universe frame before the IK solve.

Topics
------
Subscribes:
  ``/joint_states``        sensor_msgs/JointState
  ``/robot_description``   std_msgs/String  (transient-local / latched)

Publishes:
  ``teleop_target``        geometry_msgs/PoseStamped (phone target in root_frame)
  ``/tf``                  TransformBroadcaster (root_frame → teleop_target)
  ``<command_topic>``      std_msgs/Float64MultiArray → forward_position_controller
"""

from __future__ import annotations

import threading

import numpy as np
import pinocchio as pin
import rclpy
import transforms3d as t3d
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from teleop import Teleop
from tf2_ros import TransformBroadcaster

from pai_rviz_teleop.ik_solver import DifferentialIKSolver

# Default arm joints for the SO-ARM101, in order.  The gripper joint is
# intentionally excluded — it does not move the tool frame.
DEFAULT_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
]


def transient_local_qos(depth: int = 1) -> QoSProfile:
    """QoS matching latched publishers such as ``/robot_description``."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def _se3_to_mat(se3: pin.SE3) -> np.ndarray:
    """Convert a Pinocchio SE3 to a 4×4 homogeneous matrix."""
    T = np.eye(4)
    T[:3, :3] = se3.rotation
    T[:3, 3] = se3.translation
    return T


class PhoneTeleopNode(Node):
    """Single node that runs FK and the teleop.Teleop WebXR bridge together."""

    def __init__(self) -> None:
        """Declare parameters, build solver, start teleop server thread."""
        super().__init__("phone_teleop")

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("teleop_target_topic", "teleop_target")
        self.declare_parameter("ee_frame", "gripper_frame_link")
        # root_frame sets the reference frame for the teleop bridge and the
        # published PoseStamped/TF.  FK is always computed in Pinocchio's
        # universe frame; the node converts to/from root_frame so that phone
        # translational deltas feel natural relative to the arm's base.
        self.declare_parameter("root_frame", "base_link")
        self.declare_parameter("arm_joints", DEFAULT_ARM_JOINTS)
        # teleop.Teleop server settings
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 4443)
        self.declare_parameter(
            "natural_orientation",
            [0.0, 0.0, 0.0],
        )
        self.declare_parameter("natural_position", [0.0, 0.0, 0.0])
        # IK solver weights forwarded directly to DifferentialIKSolver.
        self.declare_parameter("position_cost", 0.5)
        self.declare_parameter("orientation_cost", 0.1)
        self.declare_parameter("posture_cost", 1e-3)
        self.declare_parameter("lm_damping", 1e-2)
        self.declare_parameter("max_joint_velocity", 2.0)
        self.declare_parameter("max_joint_acceleration", 10.0)
        self.declare_parameter("qp_solver", "quadprog")
        # IK control loop settings.
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("command_topic", "/forward_position_controller/commands")
        self.declare_parameter("gripper_joint", "gripper_joint")
        self.declare_parameter("inactivity_timeout", 0.3)
        self.declare_parameter("target_lowpass_alpha", 0.5)

        self._joint_states_topic = self.get_parameter("joint_states_topic").value
        self._robot_description_topic = self.get_parameter("robot_description_topic").value
        self._teleop_target_topic = self.get_parameter("teleop_target_topic").value
        self._ee_frame = self.get_parameter("ee_frame").value
        self._root_frame = self.get_parameter("root_frame").value
        arm_joints = list(self.get_parameter("arm_joints").value)

        # Build the FK/IK solver.
        urdf_xml = self._wait_for_robot_description()
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

        # Detect gripper and build the ordered command joint list.
        gripper_joint_param = self.get_parameter("gripper_joint").value
        self._gripper_joint: str | None = gripper_joint_param if gripper_joint_param else None
        self._has_gripper = (
            self._gripper_joint is not None
            and self._solver.full_model.existJointName(self._gripper_joint)
        )
        if self._gripper_joint and not self._has_gripper:
            self.get_logger().warn(
                f"gripper_joint '{self._gripper_joint}' not found in URDF; "
                "gripper will be omitted from commands."
            )
        # Arm joints first, then gripper (matches forward_position_controller order).
        self._command_joints = list(self._solver.joint_names)
        if self._has_gripper:
            self._command_joints.append(self._gripper_joint)

        # Precompute the fixed SE3 from Pinocchio universe ("world" URDF root)
        # to root_frame.  For a fixed-base arm this is constant — base_link is
        # connected to world via a fixed joint so its placement never changes.
        self._T_world_root: pin.SE3 = self._precompute_T_world_root()

        # teleop.Teleop expects the natural_orientation in radians.
        natural_orientation_deg = list(self.get_parameter("natural_orientation").value)
        natural_orientation_rad = [np.deg2rad(d) for d in natural_orientation_deg]
        natural_position = list(self.get_parameter("natural_position").value)

        self._teleop = Teleop(
            host=self.get_parameter("host").value,
            port=int(self.get_parameter("port").value),
            natural_phone_orientation_euler=natural_orientation_rad,
            natural_phone_position=natural_position,
        )
        self._teleop.subscribe(self._teleop_cb)

        # ROS I/O
        self._target_pub = self.create_publisher(PoseStamped, self._teleop_target_topic, 1)
        self._tf_broadcaster = TransformBroadcaster(self)
        command_topic = self.get_parameter("command_topic").value
        self._command_pub = self.create_publisher(Float64MultiArray, command_topic, 10)

        # Shared mutable state accessed by joint-state CB, teleop CB, and control loop.
        # All writes and reads must be under _lock.
        self._lock = threading.Lock()
        self._measured_q: np.ndarray | None = None
        self._gripper_position: float = 0.0
        self._ik_initialized = False
        self._target_se3: pin.SE3 | None = None
        self._target_filtered_se3: pin.SE3 | None = None
        self._last_move_time: rclpy.time.Time | None = None
        self._warned_missing: set[str] = set()

        self._joint_state_sub = self.create_subscription(
            JointState, self._joint_states_topic, self._joint_state_cb, 10
        )

        # Fixed-rate IK solve and command publishing, isolated on its own
        # callback group so it is not starved by subscription callbacks.
        self._control_rate = float(self.get_parameter("control_rate").value)
        self._inactivity_timeout = float(self.get_parameter("inactivity_timeout").value)
        self._lowpass_alpha = float(
            np.clip(self.get_parameter("target_lowpass_alpha").value, 1e-3, 1.0)
        )
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._control_timer = self.create_timer(
            1.0 / self._control_rate,
            self._control_loop,
            callback_group=self._control_group,
        )

        # Start the blocking uvicorn server in a daemon thread so it dies
        # automatically when the process exits.
        self._teleop_thread = threading.Thread(
            target=self._teleop.run, daemon=True, name="teleop_server"
        )
        self._teleop_thread.start()
        self.get_logger().info(
            f"Phone teleop node ready. "
            f"ee_frame='{self._ee_frame}', root_frame='{self._root_frame}', "
            f"command_topic='{command_topic}', control_rate={self._control_rate:.0f} Hz."
        )

    # ------------------------------------------------------------------
    # URDF wait (identical pattern to ee_pose_publisher)
    # ------------------------------------------------------------------
    def _wait_for_robot_description(self) -> str:
        """Block until the latched URDF is received."""
        topic = self._robot_description_topic
        result: dict[str, str] = {}
        event = threading.Event()

        def _cb(msg: String) -> None:
            result["urdf"] = msg.data
            event.set()

        sub = self.create_subscription(String, topic, _cb, transient_local_qos())
        self.get_logger().info(f"Waiting for {topic} ...")
        while rclpy.ok() and not event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        if "urdf" not in result:
            raise RuntimeError(f"Did not receive {topic}")
        self.get_logger().info(f"Received robot description from {topic}.")
        return result["urdf"]

    # ------------------------------------------------------------------
    # Joint state callback (executor thread)
    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState) -> None:
        """Cache the latest measured joint positions and gripper angle.

        The solver is NOT reset here — the IK control loop seeds it once at
        initialization and integrates forward thereafter so the
        AccelerationLimit can smooth velocity transitions between ticks.
        """
        positions = dict(zip(msg.name, msg.position, strict=False))
        with self._lock:
            q = self._solver.configuration_from_dict(positions)
            if q is None:
                for joint in self._solver.joint_names:
                    if joint not in positions and joint not in self._warned_missing:
                        self.get_logger().warn(
                            f"Arm joint '{joint}' missing from {self._joint_states_topic}; "
                            "arm joints not available yet. (Subsequent misses will not be logged.)"
                        )
                        self._warned_missing.add(joint)
                return
            self._measured_q = q
            if self._has_gripper and self._gripper_joint in positions:
                self._gripper_position = positions[self._gripper_joint]

    # ------------------------------------------------------------------
    # Teleop callback (uvicorn/asyncio thread)
    # ------------------------------------------------------------------
    def _teleop_cb(self, pose: np.ndarray, params: dict) -> None:
        """Store the phone-commanded target and publish visualization topics.

        Called by ``teleop.Teleop`` on every WebSocket frame.  ``pose`` is a
        4×4 homogeneous matrix in ``root_frame`` coordinates.

        When ``params["move"]`` is ``True`` the target is stored for the IK
        control loop and the ``teleop_target`` PoseStamped + TF are published
        for visualization.  When ``move`` is ``False`` the phone is at rest;
        we skip publishing so the arm holds its last commanded position.
        """
        if not params.get("move", False):
            return

        # Store target for the IK control loop (under lock).
        target_se3 = pin.SE3(pose[:3, :3].copy(), pose[:3, 3].copy())
        with self._lock:
            self._target_se3 = target_se3
            self._last_move_time = self.get_clock().now()

        stamp = self.get_clock().now().to_msg()

        # --- PoseStamped on teleop_target topic ---
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._root_frame
        msg.pose.position.x = float(pose[0, 3])
        msg.pose.position.y = float(pose[1, 3])
        msg.pose.position.z = float(pose[2, 3])
        quat = t3d.quaternions.mat2quat(pose[:3, :3])  # [w, x, y, z]
        msg.pose.orientation.w = float(quat[0])
        msg.pose.orientation.x = float(quat[1])
        msg.pose.orientation.y = float(quat[2])
        msg.pose.orientation.z = float(quat[3])
        self._target_pub.publish(msg)

        # --- TF: root_frame → teleop_target (for RViz visualization) ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._root_frame
        tf_msg.child_frame_id = "teleop_target"
        tf_msg.transform.translation.x = float(pose[0, 3])
        tf_msg.transform.translation.y = float(pose[1, 3])
        tf_msg.transform.translation.z = float(pose[2, 3])
        tf_msg.transform.rotation.w = float(quat[0])
        tf_msg.transform.rotation.x = float(quat[1])
        tf_msg.transform.rotation.y = float(quat[2])
        tf_msg.transform.rotation.z = float(quat[3])
        self._tf_broadcaster.sendTransform(tf_msg)

    def _precompute_T_world_root(self) -> pin.SE3:
        """Return the constant SE3 from Pinocchio's universe frame to ``root_frame``.

        For a fixed-base arm ``root_frame`` (typically ``base_link``) is
        connected to the URDF root (``world``) via a fixed joint, so this
        transform is the same for every joint configuration.  It is used to
        express FK results and IK targets in ``root_frame`` coordinates rather
        than in Pinocchio's universe frame, which may have an arbitrary rotation
        relative to the arm's base.
        """
        if not self._solver.model.existFrame(self._root_frame):
            self.get_logger().warn(
                f"root_frame '{self._root_frame}' not found in the Pinocchio model; "
                "falling back to the universe frame.  Teleop axes may feel inverted."
            )
            return pin.SE3.Identity()
        tmp_data = self._solver.model.createData()
        pin.forwardKinematics(self._solver.model, tmp_data, pin.neutral(self._solver.model))
        pin.updateFramePlacements(self._solver.model, tmp_data)
        fid = self._solver.model.getFrameId(self._root_frame)
        T = tmp_data.oMf[fid].copy()
        self.get_logger().info(
            f"root_frame '{self._root_frame}': translation={T.translation}, "
            f"rpy={pin.rpy.matrixToRpy(T.rotation)}."
        )
        return T

    # ------------------------------------------------------------------
    # IK control loop (executor timer, MutuallyExclusiveCallbackGroup)
    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        """Solve differential IK and stream joint commands at ``control_rate`` Hz.

        The solver is seeded once from measured joint states at initialization.
        Thereafter the IK integrates the configuration forward so the
        AccelerationLimit smooths velocity transitions between ticks.

        ``teleop.set_pose()`` is updated from the commanded EE pose (not the
        measured one) so the phone bridge's delta is relative to where the arm
        is being commanded to be, not where it happens to be measured.
        """
        dt = 1.0 / self._control_rate
        now = self.get_clock().now()
        q_cmd: np.ndarray | None = None
        gripper: float | None = None

        with self._lock:
            if self._measured_q is None:
                return  # no joint data yet

            # Seed the solver once; let IK integrate thereafter.
            if not self._ik_initialized:
                self._solver.reset(self._measured_q)
                self._ik_initialized = True
                self.get_logger().info("IK solver initialized from measured joint states.")

            # Seed teleop with the commanded EE pose expressed in root_frame
            # coordinates so the phone's translational deltas are in the arm's
            # natural coordinate frame rather than the URDF universe frame.
            T_world_ee = self._solver.forward_kinematics()
            T_root_ee = self._T_world_root.inverse() * T_world_ee
            self._teleop.set_pose(_se3_to_mat(T_root_ee))

            # Inactivity gate: stop solving when the phone has been at rest.
            arm_active = self._last_move_time is not None and (
                (now - self._last_move_time).nanoseconds * 1e-9 < self._inactivity_timeout
            )

            if not arm_active or self._target_se3 is None:
                self._solver.set_zero_velocity()
                return

            # SE3 low-pass filter: geodesic step from the filtered target
            # toward the latest raw target.
            if self._target_filtered_se3 is None:
                self._target_filtered_se3 = self._target_se3
            else:
                delta = pin.log6(self._target_filtered_se3.actInv(self._target_se3)).vector
                self._target_filtered_se3 = self._target_filtered_se3 * pin.exp6(
                    self._lowpass_alpha * delta
                )

            try:
                # Convert the filtered target from root_frame back to world
                # (Pinocchio universe) frame before passing it to the IK solver.
                T_world_target = self._T_world_root * self._target_filtered_se3
                q_cmd = self._solver.solve(T_world_target, dt)
            except Exception as exc:
                self.get_logger().warn(f"IK solve failed: {exc}")
                return

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
    """Entry point for the phone teleop node."""
    rclpy.init(args=args)
    node = PhoneTeleopNode()
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

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

"""Integrated phone-teleop node: FK + teleop.Teleop WebXR bridge.

Replaces the two-process design (``ee_pose_publisher`` node +
``python3 -m teleop.ros2`` subprocess) with a single rclpy node that embeds
the ``teleop.Teleop`` WebXR server in a background thread.

Architecture
------------
* Main thread — ``executor.spin()`` handles all ROS callbacks.
* Daemon thread — ``teleop.Teleop.run()`` (blocking uvicorn server).
* ``_joint_state_cb`` (executor thread) — runs FK, calls
  ``teleop.set_pose(T)`` so the bridge always has the current EE pose as its
  internal reference.  ``Teleop.set_pose`` is a simple attribute assignment,
  safe to call from any thread.
* ``_teleop_cb`` (uvicorn/asyncio thread) — called by the bridge on every
  WebSocket frame.  When the phone is moving it publishes a
  ``geometry_msgs/PoseStamped`` on ``teleop_target`` and a TF transform
  ``root_frame → teleop_target``.  rclpy publisher ``publish()`` and
  ``TransformBroadcaster.sendTransform()`` are thread-safe.

Topics
------
Subscribes:
  ``/joint_states``        sensor_msgs/JointState
  ``/robot_description``   std_msgs/String  (transient-local / latched)

Publishes:
  ``teleop_target``         geometry_msgs/PoseStamped  (EE target in root_frame)
  ``/tf``                  via TransformBroadcaster (root_frame → teleop_target)
"""

from __future__ import annotations

import threading

import numpy as np
import pinocchio as pin
import rclpy
import transforms3d as t3d
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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
        # root_frame must match Pinocchio's universe frame, which for the
        # SO-ARM101 URDF is "world" (the URDF root link).  Changing this only
        # relabels the published frame_id; the FK coordinates do not change.
        self.declare_parameter("root_frame", "world")
        self.declare_parameter("arm_joints", DEFAULT_ARM_JOINTS)
        # teleop.Teleop server settings
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 4443)
        self.declare_parameter(
            "natural_orientation",
            [0.0, -45.0, 0.0],
        )
        self.declare_parameter("natural_position", [0.0, 0.0, 0.0])
        # IK solver weights (forwarded to DifferentialIKSolver; unused in FK
        # path but kept here so a future IK consumer can share the same node).
        self.declare_parameter("position_cost", 0.5)
        self.declare_parameter("orientation_cost", 0.1)
        self.declare_parameter("posture_cost", 1e-3)
        self.declare_parameter("lm_damping", 1e-2)
        self.declare_parameter("max_joint_velocity", 2.0)
        self.declare_parameter("max_joint_acceleration", 10.0)
        self.declare_parameter("qp_solver", "quadprog")

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

        self._warned_missing: set[str] = set()

        self._joint_state_sub = self.create_subscription(
            JointState, self._joint_states_topic, self._joint_state_cb, 10
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
            f"teleop_target_topic='{self._teleop_target_topic}'."
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
        """Run FK and update teleop's internal reference pose."""
        positions = dict(zip(msg.name, msg.position, strict=False))

        q = self._solver.configuration_from_dict(positions)
        if q is None:
            for joint in self._solver.joint_names:
                if joint not in positions and joint not in self._warned_missing:
                    self.get_logger().warn(
                        f"Arm joint '{joint}' missing from {self._joint_states_topic}; "
                        "skipping this message. (Subsequent misses will not be logged.)"
                    )
                    self._warned_missing.add(joint)
            return

        self._solver.reset(q)
        pose_se3 = self._solver.forward_kinematics()
        # Keep teleop seeded with the current EE pose so the bridge can
        # compute phone-relative deltas from the actual arm position.
        self._teleop.set_pose(_se3_to_mat(pose_se3))

    # ------------------------------------------------------------------
    # Teleop callback (uvicorn/asyncio thread)
    # ------------------------------------------------------------------
    def _teleop_cb(self, pose: np.ndarray, params: dict) -> None:
        """Publish the phone-commanded EE target pose.

        Called by ``teleop.Teleop`` on every WebSocket frame.  ``pose`` is a
        4×4 homogeneous matrix in ``root_frame`` coordinates.

        When ``params["move"]`` is ``False`` the phone is at rest; teleop
        emits the current (unchanged) reference pose.  We skip publishing in
        that state — the arm should hold its position until a new target
        arrives.
        """
        if not params.get("move", False):
            return

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

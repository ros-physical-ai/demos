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

"""Phone teleop adapter: teleop.Teleop WebXR bridge -> Cartesian target topic.

A thin frontend for the ``pai_teleop_ik`` IK servo. It owns no IK or kinematics:
it embeds the upstream ``teleop`` WebXR bridge and translates the phone's 6-DoF
pose into a Cartesian target for the servo to track.

Architecture
------------
* Main thread -- ``executor.spin()`` handles all ROS callbacks.
* Daemon thread -- ``teleop.Teleop.run()`` (blocking uvicorn/WebSocket server).
* ``_ee_pose_cb`` (executor thread) -- caches the servo's latest commanded tool
  pose (in ``root_frame``) as a 4x4 matrix used to seed the teleop bridge.
* ``_teleop_cb`` (uvicorn/asyncio thread) -- on every WebSocket frame: captures
  button states and, while the phone is moving, publishes the commanded pose to
  ``target_pose`` (plus a ``root_frame -> phone_teleop_target`` TF for RViz).
* ``_control_loop`` (executor timer) -- seeds ``teleop.set_pose()`` with the
  latest ``ee_pose`` so phone deltas are relative to where the arm is, and ramps
  + publishes the gripper setpoint on ``gripper_command``.

Frame handling lives entirely in the servo: the teleop bridge already produces
poses in ``root_frame``, so this adapter just stamps ``target_pose`` with
``root_frame`` and lets the servo resolve it.

Topics
------
Subscribes:
  ``ee_pose``              geometry_msgs/PoseStamped  (servo commanded tool pose)

Publishes:
  ``target_pose``          geometry_msgs/PoseStamped  (phone target in root_frame)
  ``gripper_command``      std_msgs/Float64           (gripper setpoint, rad)
  ``/tf``                  TransformBroadcaster (root_frame -> phone_teleop_target)

The ``target_pose`` topic doubles as the RViz visualization source (it is a
``PoseStamped`` in ``root_frame``); the ``root_frame -> phone_teleop_target`` TF
offers the same target as a frame for the TF display.
"""

from __future__ import annotations

import threading

import numpy as np
import rclpy
import transforms3d as t3d
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float64
from teleop import Teleop
from tf2_ros import TransformBroadcaster


def _pose_to_mat(pose) -> np.ndarray:
    """Convert a geometry_msgs/Pose to a 4x4 homogeneous matrix."""
    quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    mat = np.eye(4)
    mat[:3, :3] = t3d.quaternions.quat2mat(quat)
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return mat


class PhoneTeleopNode(Node):
    """Publishes the phone's 6-DoF pose as a Cartesian target for the IK servo."""

    def __init__(self) -> None:
        """Declare parameters and start the teleop server thread."""
        super().__init__("phone_teleop")

        # root_frame must match the servo's root_frame; the teleop bridge and the
        # published target/visualization are all expressed in it.
        self.declare_parameter("root_frame", "base_link")
        self.declare_parameter("control_rate", 50.0)
        # teleop.Teleop server settings.
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 4443)
        self.declare_parameter("natural_orientation", [0.0, 0.0, 0.0])
        self.declare_parameter("natural_position", [0.0, 0.0, 0.0])
        # Topics shared with the servo.
        self.declare_parameter("target_pose_topic", "ik_servo/target_pose")
        self.declare_parameter("gripper_command_topic", "ik_servo/gripper_command")
        self.declare_parameter("ee_pose_topic", "ik_servo/ee_pose")
        # Gripper: whether this arm has a gripper, the open/closed positions (rad),
        # and the ramp speed for the A/B buttons.
        # This adapter does not read the URDF, so the range is a parameter.
        self.declare_parameter("has_gripper", True)
        self.declare_parameter("gripper_open_position", 1.7)
        self.declare_parameter("gripper_closed_position", 0.0)
        self.declare_parameter("gripper_speed", 5.0)

        self._root_frame = self.get_parameter("root_frame").value
        self._control_rate = float(self.get_parameter("control_rate").value)
        self._has_gripper = bool(self.get_parameter("has_gripper").value)
        self._gripper_open = float(self.get_parameter("gripper_open_position").value)
        self._gripper_closed = float(self.get_parameter("gripper_closed_position").value)
        self._gripper_speed = float(self.get_parameter("gripper_speed").value)
        self._gripper_dir = 1.0 if self._gripper_open >= self._gripper_closed else -1.0

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

        # ROS I/O.
        self._target_pub = self.create_publisher(PoseStamped, self.get_parameter("target_pose_topic").value, 10)
        self._gripper_pub = self.create_publisher(Float64, self.get_parameter("gripper_command_topic").value, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, self.get_parameter("ee_pose_topic").value, self._ee_pose_cb, 10)

        # Shared mutable state accessed by ee_pose CB, teleop CB, and control loop.
        self._lock = threading.Lock()
        self._ee_mat: np.ndarray | None = None  # latest servo ee_pose as 4x4
        self._gripper_position: float = self._gripper_closed
        # Button A -> open gripper slowly; B -> close slowly (held = ramping).
        # Gripper button (engaged) -> lock at the closed position (apply pressure).
        self._button_a_held = False
        self._button_b_held = False
        self._gripper_engaged = False

        # Fixed-rate loop: seed teleop with the latest ee_pose and ramp + publish
        # the gripper. On its own callback group so it is not starved.
        self._control_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / self._control_rate, self._control_loop, callback_group=self._control_group)

        # Start the blocking uvicorn server in a daemon thread so it dies
        # automatically when the process exits.
        self._teleop_thread = threading.Thread(target=self._teleop.run, daemon=True, name="teleop_server")
        self._teleop_thread.start()
        self.get_logger().info(
            f"Phone teleop adapter ready. root_frame='{self._root_frame}', "
            f"publishing targets on '{self.get_parameter('target_pose_topic').value}'."
        )

    # ------------------------------------------------------------------
    # ee_pose callback (executor thread)
    # ------------------------------------------------------------------
    def _ee_pose_cb(self, msg: PoseStamped) -> None:
        """Cache the servo's commanded tool pose as a 4x4 matrix for teleop seeding."""
        mat = _pose_to_mat(msg.pose)
        with self._lock:
            self._ee_mat = mat

    # ------------------------------------------------------------------
    # Teleop callback (uvicorn/asyncio thread)
    # ------------------------------------------------------------------
    def _teleop_cb(self, pose: np.ndarray, params: dict) -> None:
        """Publish the phone-commanded target and visualization topics.

        ``pose`` is a 4x4 homogeneous matrix already expressed in ``root_frame``.
        Button states are captured regardless of ``move`` so the gripper can be
        operated while the arm is stationary.
        """
        with self._lock:
            self._button_a_held = bool(params.get("reservedButtonA", False))
            self._button_b_held = bool(params.get("reservedButtonB", False))
            self._gripper_engaged = params.get("gripper", "open") == "close"

        if not params.get("move", False):
            return

        stamp = self.get_clock().now().to_msg()
        quat = t3d.quaternions.mat2quat(pose[:3, :3])  # [w, x, y, z]

        # --- target_pose for the servo (also the RViz visualization source) ---
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._root_frame
        msg.pose.position.x = float(pose[0, 3])
        msg.pose.position.y = float(pose[1, 3])
        msg.pose.position.z = float(pose[2, 3])
        msg.pose.orientation.w = float(quat[0])
        msg.pose.orientation.x = float(quat[1])
        msg.pose.orientation.y = float(quat[2])
        msg.pose.orientation.z = float(quat[3])
        self._target_pub.publish(msg)

        # --- TF: root_frame -> phone_teleop_target (for RViz visualization) ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._root_frame
        tf_msg.child_frame_id = "phone_teleop_target"
        tf_msg.transform.translation.x = float(pose[0, 3])
        tf_msg.transform.translation.y = float(pose[1, 3])
        tf_msg.transform.translation.z = float(pose[2, 3])
        tf_msg.transform.rotation.w = float(quat[0])
        tf_msg.transform.rotation.x = float(quat[1])
        tf_msg.transform.rotation.y = float(quat[2])
        tf_msg.transform.rotation.z = float(quat[3])
        self._tf_broadcaster.sendTransform(tf_msg)

    # ------------------------------------------------------------------
    # Control loop (executor timer)
    # ------------------------------------------------------------------
    def _control_loop(self) -> None:
        """Seed teleop with the latest ee_pose and ramp + publish the gripper."""
        dt = 1.0 / self._control_rate
        with self._lock:
            # Seed teleop with the commanded EE pose so the phone's deltas are
            # relative to where the arm is being commanded to be.
            if self._ee_mat is not None:
                self._teleop.set_pose(self._ee_mat)

            # Gripper policy: engaged -> lock at the closed position (apply
            # pressure); else A ramps open and B ramps closed.
            if not self._has_gripper:
                gripper_value = None
            else:
                if self._gripper_engaged:
                    self._gripper_position = self._gripper_closed
                elif self._button_a_held:
                    lo = min(self._gripper_open, self._gripper_closed)
                    hi = max(self._gripper_open, self._gripper_closed)
                    self._gripper_position = max(
                        lo,
                        min(hi, self._gripper_position + self._gripper_dir * self._gripper_speed * dt),
                    )
                elif self._button_b_held:
                    lo = min(self._gripper_open, self._gripper_closed)
                    hi = max(self._gripper_open, self._gripper_closed)
                    self._gripper_position = max(
                        lo,
                        min(hi, self._gripper_position - self._gripper_dir * self._gripper_speed * dt),
                    )
                gripper_value = self._gripper_position

        if gripper_value is not None:
            self._gripper_pub.publish(Float64(data=gripper_value))


def main(args: list[str] | None = None) -> None:
    """Entry point for the phone teleop adapter."""
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

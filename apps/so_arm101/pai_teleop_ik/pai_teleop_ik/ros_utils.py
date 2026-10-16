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

"""Small ROS helpers shared across teleop nodes."""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


def transient_local_qos(depth: int = 1) -> QoSProfile:
    """QoS matching latched publishers such as ``/robot_description``."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def wait_for_robot_description(node: Node, topic: str = "/robot_description") -> str:
    """Block until the latched robot description URDF is received on ``topic``."""
    result: dict[str, str] = {}
    event = threading.Event()

    def _cb(msg: String) -> None:
        result["urdf"] = msg.data
        event.set()

    sub = node.create_subscription(String, topic, _cb, transient_local_qos())
    node.get_logger().info(f"Waiting for {topic} ...")
    while rclpy.ok() and not event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if "urdf" not in result:
        raise RuntimeError(f"Did not receive {topic}")
    node.get_logger().info(f"Received robot description from {topic}.")
    return result["urdf"]

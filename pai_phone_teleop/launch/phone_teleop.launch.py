#!/usr/bin/env python3

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

"""Phone teleop launch: integrated PhoneTeleopNode (FK + teleop.Teleop WebXR bridge).

Starts a single ``phone_teleop_node`` that:

1. Subscribes to ``/joint_states`` and runs forward kinematics to track the
   current end-effector pose.
2. Embeds ``teleop.Teleop`` (WebXR WebSocket server) in a background thread,
   keeping it seeded with the live EE pose expressed in ``root_frame``.
3. On every phone move, solves differential IK toward the commanded target and
   publishes joint positions to the ``forward_position_controller``.
4. Publishes the raw phone target as a ``geometry_msgs/PoseStamped`` on
   ``teleop_target`` and broadcasts ``root_frame → teleop_target`` TF for
   RViz visualization.

The user is expected to have launched a ``pai_bringup`` bringup (real,
mujoco, or gz) in a separate terminal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the phone teleop launch description."""
    declared_arguments = [
        DeclareLaunchArgument(
            "host",
            default_value="0.0.0.0",
            description="Bind address for the teleop.Teleop WebXR server.",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="4443",
            description="Port for the teleop.Teleop WebXR server.",
        ),
        DeclareLaunchArgument(
            "ee_frame",
            default_value="gripper_frame_link",
            description="End-effector frame whose pose is tracked and published.",
        ),
        DeclareLaunchArgument(
            "root_frame",
            default_value="base_link",
            description=(
                "Reference frame for the teleop target PoseStamped and TF broadcast.  "
                "The IK solver always operates in Pinocchio's universe frame; this node "
                "converts to/from root_frame so phone deltas feel natural relative to "
                "the arm base.  Defaults to 'base_link' (arm's natural base frame)."
            ),
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="phone_teleop",
            description="Name of the phone_teleop_node.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="/forward_position_controller/commands",
            description="Float64MultiArray topic consumed by the forward_position_controller.",
        ),
        DeclareLaunchArgument(
            "control_rate",
            default_value="50.0",
            description="IK control loop rate in Hz.",
        ),
        DeclareLaunchArgument(
            "inactivity_timeout",
            default_value="0.3",
            description="Seconds after the phone stops moving before commands halt.",
        ),
    ]

    phone_teleop_node = Node(
        package="pai_phone_teleop",
        executable="phone_teleop_node",
        name=LaunchConfiguration("node_name"),
        output="both",
        parameters=[
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "root_frame": LaunchConfiguration("root_frame"),
                "command_topic": LaunchConfiguration("command_topic"),
                "control_rate": LaunchConfiguration("control_rate"),
                "inactivity_timeout": LaunchConfiguration("inactivity_timeout"),
            }
        ],
    )

    return LaunchDescription(
        [
            *declared_arguments,
            phone_teleop_node,
        ]
    )

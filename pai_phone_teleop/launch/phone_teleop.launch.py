#!/usr/bin/env python3

# Copyright (C) 2026 Sebastian Castro
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
   keeping it seeded with the live EE pose.
3. Publishes the phone-commanded target on ``target_frame``
   (``geometry_msgs/PoseStamped``) and broadcasts a TF
   ``root_frame → teleop_target`` for RViz visualization.

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
            default_value="world",
            description=(
                "Reference frame for the published target_frame PoseStamped "
                "and TF broadcast.  Must match Pinocchio's universe frame "
                "(i.e. the URDF root link, 'world' for the SO-ARM101)."
            ),
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="phone_teleop",
            description="Name of the phone_teleop_node.",
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
            }
        ],
    )

    return LaunchDescription(
        [
            *declared_arguments,
            phone_teleop_node,
        ]
    )

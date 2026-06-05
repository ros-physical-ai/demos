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

"""Interactive marker IK node for the SO-ARM101.

Starts the Pink-based differential IK node that drives the arm's tool frame
toward the interactive marker. The simulation and RViz are expected to be
launched separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the interactive IK demo launch description."""
    declared_arguments = [
        DeclareLaunchArgument(
            "control_rate",
            default_value="50.0",
            description="Differential IK / command streaming rate in Hz.",
        ),
        DeclareLaunchArgument(
            "ee_frame",
            default_value="gripper_frame_link",
            description="End-effector (tool) frame driven toward the marker.",
        ),
    ]

    interactive_ik_node = Node(
        package="pai_rviz_teleop",
        executable="interactive_ik_node",
        output="both",
        parameters=[
            {
                "use_sim_time": True,
                "control_rate": LaunchConfiguration("control_rate"),
                "ee_frame": LaunchConfiguration("ee_frame"),
            }
        ],
    )

    return LaunchDescription([*declared_arguments, interactive_ik_node])

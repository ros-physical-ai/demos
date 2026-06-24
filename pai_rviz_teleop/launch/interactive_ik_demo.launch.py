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

"""Interactive marker IK demo.

Launches the ``pai_teleop_ik`` IK servo together with the interactive marker
adapter. Drag the marker in RViz and the arm's tool frame follows it. The
simulation and RViz are expected to be launched separately.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# SO-ARM101 arm joints, in order. The gripper joint is excluded (it does not move
# the tool frame); the servo appends it separately via its gripper_joint param.
SO_ARM101_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
]


def generate_launch_description():
    """Generate the interactive IK demo launch description."""
    declared_arguments = [
        DeclareLaunchArgument(
            "arm_joints",
            default_value=",".join(SO_ARM101_ARM_JOINTS),
            description="Comma-separated ordered joint names the IK actuates.",
        ),
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
        DeclareLaunchArgument(
            "root_frame",
            default_value="world",
            description="Frame the marker is anchored in; must match the servo root_frame.",
        ),
    ]

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("pai_teleop_ik"), "launch", "ik_servo.launch.py")
        ),
        launch_arguments={
            "arm_joints": LaunchConfiguration("arm_joints"),
            "root_frame": LaunchConfiguration("root_frame"),
            "ee_frame": LaunchConfiguration("ee_frame"),
            "control_rate": LaunchConfiguration("control_rate"),
            "target_pose_topic": "ik_servo/target_pose",
            "gripper_command_topic": "ik_servo/gripper_command",
            "ee_pose_topic": "ik_servo/ee_pose",
        }.items(),
    )

    interactive_ik_node = Node(
        package="pai_rviz_teleop",
        executable="interactive_ik_node",
        output="both",
        parameters=[
            {
                "use_sim_time": True,
                "control_rate": LaunchConfiguration("control_rate"),
                "root_frame": LaunchConfiguration("root_frame"),
                "target_pose_topic": "ik_servo/target_pose",
                "gripper_command_topic": "ik_servo/gripper_command",
                "ee_pose_topic": "ik_servo/ee_pose",
            }
        ],
    )

    return LaunchDescription([*declared_arguments, servo_launch, interactive_ik_node])

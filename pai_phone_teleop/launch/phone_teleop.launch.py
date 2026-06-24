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

"""Phone teleop demo launch.

Launches the ``pai_teleop_ik`` IK servo together with the phone teleop adapter
(the teleop.Teleop WebXR bridge). The phone's 6-DoF pose is published as a
Cartesian target and the servo tracks it. The user is expected to have launched
a ``pai_bringup`` bringup (real, mujoco, or gz) in a separate terminal.
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
    """Generate the phone teleop launch description."""
    declared_arguments = [
        DeclareLaunchArgument(
            "arm_joints",
            default_value=",".join(SO_ARM101_ARM_JOINTS),
            description="Comma-separated ordered joint names the IK actuates.",
        ),
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
            description="End-effector frame the servo drives toward the phone target.",
        ),
        DeclareLaunchArgument(
            "root_frame",
            default_value="base_link",
            description="Reference frame for the phone target; must match the servo root_frame.",
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
            "command_topic": LaunchConfiguration("command_topic"),
            "target_pose_topic": "ik_servo/target_pose",
            "gripper_command_topic": "ik_servo/gripper_command",
            "ee_pose_topic": "ik_servo/ee_pose",
        }.items(),
    )

    phone_teleop_node = Node(
        package="pai_phone_teleop",
        executable="phone_teleop_node",
        name="phone_teleop",
        output="both",
        parameters=[
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "root_frame": LaunchConfiguration("root_frame"),
                "control_rate": LaunchConfiguration("control_rate"),
                "target_pose_topic": "ik_servo/target_pose",
                "gripper_command_topic": "ik_servo/gripper_command",
                "ee_pose_topic": "ik_servo/ee_pose",
            }
        ],
    )

    return LaunchDescription([*declared_arguments, servo_launch, phone_teleop_node])

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

"""Standalone launch for the topic-driven IK servo node.

Included by the teleop frontend demos, or run on its own and driven by any
publisher of ``target_pose`` (PoseStamped) and ``gripper_command`` (Float64).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    # arm_joints is passed as a comma-separated string so it can cross the
    # IncludeLaunchDescription boundary (launch arguments are strings); split it
    # back into a list for the node parameter.
    arm_joints = [j.strip() for j in LaunchConfiguration("arm_joints").perform(context).split(",") if j.strip()]

    servo = Node(
        package="pai_teleop_ik",
        executable="ik_servo_node",
        name="ik_servo",
        output="both",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "arm_joints": arm_joints,
                "ee_frame": LaunchConfiguration("ee_frame"),
                "root_frame": LaunchConfiguration("root_frame"),
                "control_rate": LaunchConfiguration("control_rate"),
                "command_topic": LaunchConfiguration("command_topic"),
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
                "ee_pose_topic": LaunchConfiguration("ee_pose_topic"),
            }
        ],
    )
    return [servo]


def generate_launch_description():
    """Generate the IK servo launch description."""
    args = [
        DeclareLaunchArgument(
            "arm_joints",
            default_value="",
            description="Comma-separated ordered joint names the IK actuates (required).",
        ),
        DeclareLaunchArgument("ee_frame", default_value="gripper_frame_link"),
        DeclareLaunchArgument("root_frame", default_value="world"),
        DeclareLaunchArgument("control_rate", default_value="50.0"),
        DeclareLaunchArgument("command_topic", default_value="/forward_position_controller/commands"),
        DeclareLaunchArgument("target_pose_topic", default_value="ik_servo/target_pose"),
        DeclareLaunchArgument("gripper_command_topic", default_value="ik_servo/gripper_command"),
        DeclareLaunchArgument("ee_pose_topic", default_value="ik_servo/ee_pose"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]
    return LaunchDescription([*args, OpaqueFunction(function=_launch_setup)])

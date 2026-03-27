#!/usr/bin/env python3

# Copyright 2026 Franco Cipollone
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

"""Common SO ARM bringup launch file.

This launch file provides shared functionality for all SO ARM bringup
variants (real, mujoco, gazebo).  It is meant to be included via
``IncludeLaunchDescription`` rather than launched directly.

Nodes launched:
  - robot_state_publisher
  - joint_state_broadcaster spawner
  - initial joint controller spawner (optionally in stopped state)
  - gripper_controller spawner (only when using joint_trajectory_controller)
  - rviz2 (optional, delayed until joint_state_broadcaster is ready)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    """Set up nodes for the SO ARM bringup."""
    description_file = LaunchConfiguration("description_file").perform(context)
    description_xacro_args = LaunchConfiguration("description_xacro_args").perform(context)
    ros2_control_file = LaunchConfiguration("ros2_control_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    initial_joint_controller = LaunchConfiguration("initial_joint_controller").perform(context)
    activate_joint_controller = LaunchConfiguration("activate_joint_controller").perform(context).lower() == "true"
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context).lower() == "true"
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)

    # Build robot description via xacro
    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        description_file,
    ]
    if ros2_control_file:
        xacro_cmd += [" ", f"ros2_control_file:={ros2_control_file}"]
    if description_xacro_args:
        xacro_cmd += [" ", description_xacro_args]

    robot_description_content = Command(xacro_cmd)
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="both",
    )

    # Initial joint controller - started or stopped depending on argument
    controller_args = [initial_joint_controller, "-c", "/controller_manager"]
    if not activate_joint_controller:
        controller_args.append("--stopped")
    initial_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=controller_args,
        output="both",
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner,
    ]

    # When using joint_trajectory_controller, also spawn the gripper_controller
    # since that controller does not include the gripper joint.
    # With forward_position_controller (default), the gripper joint is already included.
    if initial_joint_controller == "joint_trajectory_controller":
        gripper_controller_args = ["gripper_controller", "-c", "/controller_manager"]
        if not activate_joint_controller:
            gripper_controller_args.append("--stopped")
        gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=gripper_controller_args,
            output="both",
        )
        nodes.append(gripper_controller_spawner)

    if launch_rviz:
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": use_sim_time}],
        )
        delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
            ),
        )
        nodes.append(delay_rviz_after_joint_state_broadcaster)

    return nodes


def generate_launch_description():
    """Generate launch description with declared arguments."""
    declared_arguments = [
        DeclareLaunchArgument(
            "description_file",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "description_xacro_args",
            default_value="",
            description="Extra arguments to pass to the xacro command.",
        ),
        DeclareLaunchArgument(
            "ros2_control_file",
            default_value="",
            description="Path to ros2_control xacro file. When provided, it is passed as "
            "'ros2_control_file:=<path>' to the xacro command, overriding the default in the "
            "description file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="forward_position_controller",
            description="Robot controller to start. "
            "Use 'forward_position_controller' (default) for single-topic control of all 6 joints "
            "(including gripper) for inference/rosetta, or 'joint_trajectory_controller' for "
            "MoveIt-style control (gripper_controller is automatically spawned alongside it).",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate the initial joint controller on start.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            description="RViz config file to use.",
        ),
    ]

    return LaunchDescription([*declared_arguments, OpaqueFunction(function=launch_setup)])

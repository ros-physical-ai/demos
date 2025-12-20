#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pai_mujoco"),
                    "urdf",
                    "so_arm101.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

    controller_parameters_file = PathJoinSubstitution([FindPackageShare("pai_mujoco"), "config", "ros2_controllers.yaml"])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            controller_parameters_file,
        ],
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            controller_parameters_file,
        ],
        output="both",
    )

    spawn_jtc = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_trajectory_controller",
        arguments=[
            "joint_trajectory_controller",
            "--param-file",
            controller_parameters_file,
        ],
        output="both",
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_gripper_controller",
        arguments=[
            "gripper_controller",
            "--param-file",
            controller_parameters_file,
        ],
        output="both",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("pai_mujoco"), "rviz", "so_arm.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            control_node,
            spawn_joint_state_broadcaster,
            spawn_jtc,
            spawn_gripper_controller,
            rviz_node,
        ]
    )

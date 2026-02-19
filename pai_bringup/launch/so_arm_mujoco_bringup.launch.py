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
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pai_bringup"),
                    "urdf",
                    "so_arm101_mujoco.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

    ros2_controllers_file = PathJoinSubstitution(
        [FindPackageShare("pai_bringup"), "config", "control", "ros2_controllers.yaml"]
    )
    ros2_controllers_file = ReplaceString(
        source_file=ros2_controllers_file,
        replacements={"<robot_namespace>": ("")},
    )
    controller_parameters = ParameterFile(
        RewrittenYaml(
            source_file=ros2_controllers_file,
            root_key="",
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

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
            controller_parameters,
        ],
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
        ],
        output="both",
    )

    spawn_forward_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_forward_position_controller",
        arguments=[
            "forward_position_controller",
        ],
        output="both",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("pai_bringup"), "config", "rviz", "so_arm_mujoco.rviz"]
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
            spawn_forward_position_controller,
            rviz_node,
        ]
    )

#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Process controller parameters for mujoco control node
    ros2_controllers_file = PathJoinSubstitution(
        [FindPackageShare("pai_bringup"), "config", "control", "ros2_controllers.yaml"]
    )
    controllers_file_replaced = ReplaceString(
        source_file=ros2_controllers_file,
        replacements={"<robot_namespace>": ""},
    )
    controller_parameters = ParameterFile(
        RewrittenYaml(
            source_file=controllers_file_replaced,
            root_key="",
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
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

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pai_bringup"), "launch", "include", "so_arm_common.launch.py"]
            )
        ),
        launch_arguments={
            "description_file": PathJoinSubstitution(
                [FindPackageShare("pai_bringup"), "urdf", "so_arm101_mujoco.urdf.xacro"]
            ),
            "use_sim_time": "true",
            "rviz_config_file": PathJoinSubstitution(
                [FindPackageShare("pai_bringup"), "config", "rviz", "so_arm_mujoco.rviz"]
            ),
        }.items(),
    )

    return LaunchDescription([common, control_node])

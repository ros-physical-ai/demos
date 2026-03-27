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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def launch_setup(context, *args, **kwargs):
    """Set up nodes for the SO ARM real hardware bringup."""
    prefix = LaunchConfiguration("prefix").perform(context)
    usb_port = LaunchConfiguration("usb_port").perform(context)
    controllers_file = LaunchConfiguration("controllers_file")
    joint_config_file = LaunchConfiguration("joint_config_file").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    ros2_control_file = LaunchConfiguration("ros2_control_file").perform(context)
    initial_joint_controller = LaunchConfiguration("initial_joint_controller").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)

    # Process controller parameters for ros2_control_node
    controllers_file_replaced = ReplaceString(
        source_file=controllers_file,
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

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_parameters],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("pai_bringup"),
                    "launch",
                    "include",
                    "so_arm_common.launch.py",
                ]
            )
        ),
        launch_arguments={
            "description_file": description_file,
            "ros2_control_file": ros2_control_file,
            "description_xacro_args": (
                f"ros2_control_hardware_type:=real"
                f" prefix:={prefix}"
                f" usb_port:={usb_port}"
                f" joint_config_file:={joint_config_file}"
            ),
            "use_sim_time": "false",
            "initial_joint_controller": initial_joint_controller,
            "launch_rviz": launch_rviz,
            "rviz_config_file": rviz_config_file,
        }.items(),
    )

    return [common, ros2_control_node]


def generate_launch_description():
    """Generate launch description with declared arguments."""
    declared_arguments = [
        DeclareLaunchArgument(
            "usb_port",
            default_value="/dev/ttyACM0",
            description="USB port for the Feetech servo bus.",
        ),
        DeclareLaunchArgument(
            "joint_config_file",
            default_value="",
            description="Path to YAML file with per-robot joint calibration "
            "(homing offsets, PID gains, etc.). "
            "Each robot requires its own calibration file. "
            "See config/hardware/follower.yaml for an example. "
            "If not set, only URDF settings are used.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("pai_bringup"),
                    "config",
                    "control",
                    "ros2_controllers.yaml",
                ]
            ),
            description="Absolute path to YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([FindPackageShare("pai_bringup"), "urdf", "so_arm_real.urdf.xacro"]),
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "ros2_control_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("pai_bringup"),
                    "config",
                    "control",
                    "so_arm101.ros2_control.xacro",
                ]
            ),
            description="Path to a custom ros2_control xacro file to override the default in the description file.",
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
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([FindPackageShare("pai_bringup"), "config", "rviz", "so_arm_gz.rviz"]),
            description="RViz config file to use.",
        ),
    ]

    return LaunchDescription([*declared_arguments, OpaqueFunction(function=launch_setup)])

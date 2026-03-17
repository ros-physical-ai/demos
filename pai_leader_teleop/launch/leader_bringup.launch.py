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

"""Bringup launch file for the SO-ARM101 leader arm with teleop relay.

Launches the leader arm hardware stack under the /leader namespace and
a teleop relay node that forwards leader joint states to the follower's
position controller.

The leader arm has no command interfaces — torque is disabled on all
joints so the human can move the arm freely.  Joint states are
published to /leader/joint_states.

Usage:
    ros2 launch pai_leader_teleop leader_bringup.launch.py usb_port:=/dev/ttyACM1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    usb_port = LaunchConfiguration("usb_port").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    follower_commands_topic = LaunchConfiguration("follower_commands_topic").perform(context)

    description_file = LaunchConfiguration("description_file").perform(context)
    ros2_control_file = LaunchConfiguration("ros2_control_file").perform(context)
    controllers_file = LaunchConfiguration("controllers_file")

    # Build robot description via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            f"ros2_control_file:={ros2_control_file}",
            " ",
            f"ros2_control_hardware_type:=real",
            " ",
            f"prefix:={prefix}",
            " ",
            f"usb_port:={usb_port}",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controller_parameters = ParameterFile(
        RewrittenYaml(
            source_file=controllers_file,
            root_key="",
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # ros2_control_node — reads the URDF from the namespaced robot_description topic
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_parameters, {"use_sim_time": use_sim_time}],
        remappings=[("~/robot_description", "robot_description")],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/tf", f"/{namespace}/tf"),
            ("/tf_static", f"/{namespace}/tf_static"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            f"/{namespace}/controller_manager",
        ],
        output="both",
    )

    # Wrap hardware nodes under the leader namespace
    namespaced_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            ros2_control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
        ]
    )

    # Teleop relay node — runs in the default namespace so it can
    # subscribe to /leader/joint_states and publish to the follower's
    # command topic without extra remapping.
    teleop_node = Node(
        package="pai_leader_teleop",
        executable="leader_teleop_node",
        name="leader_teleop_node",
        parameters=[
            {
                "leader_joint_states_topic": f"/{namespace}/joint_states",
                "follower_commands_topic": follower_commands_topic,
            }
        ],
        output="both",
    )

    return [namespaced_nodes, teleop_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "usb_port",
            default_value="/dev/ttyACM1",
            description="USB port for the leader arm Feetech servo bus.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="leader",
            description="ROS namespace for the leader arm nodes.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (set to true when the follower is simulated).",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("so_arm101_description"), "urdf", "so_arm101.urdf.xacro"]
            ),
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "ros2_control_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("pai_leader_teleop"),
                    "config",
                    "control",
                    "so_arm101_leader.ros2_control.xacro",
                ]
            ),
            description="Path to the leader ros2_control xacro (state-only interfaces).",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("pai_leader_teleop"),
                    "config",
                    "control",
                    "ros2_controllers_leader.yaml",
                ]
            ),
            description="Path to the leader controllers YAML.",
        ),
        DeclareLaunchArgument(
            "follower_commands_topic",
            default_value="/forward_position_controller/commands",
            description="Topic for the follower's forward position controller commands.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

# Copyright 2025 Yadunund Vijay.
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
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
from ros_gz_sim.actions import GzServer


def launch_setup(context, *args, **kwargs):
    """Set up nodes for the SO ARM Gazebo bringup."""
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    activate_joint_controller = LaunchConfiguration("activate_joint_controller").perform(context)
    initial_joint_controller = LaunchConfiguration("initial_joint_controller").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context)
    world_file = LaunchConfiguration("world_file")
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("roll").perform(context)
    pitch = LaunchConfiguration("pitch").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    # Process controllers file for xacro
    controllers_file_replaced = ReplaceString(
        source_file=controllers_file,
        replacements={"<robot_namespace>": ""},
    )
    controllers_file_str = controllers_file_replaced.perform(context)

    # Build xacro args
    description_xacro_args = (
        f"simulation_controllers:={controllers_file_str}"
        f" prefix:={prefix}"
        f" x:={x} y:={y} z:={z}"
        f" roll:={roll} pitch:={pitch} yaw:={yaw}"
    )

    # Build robot_description_content for gz_spawn_entity
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            description_xacro_args,
        ]
    )

    # Include common launch for RSP, spawners, and RViz
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
            "description_xacro_args": description_xacro_args,
            "use_sim_time": "true",
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "launch_rviz": launch_rviz,
            "rviz_config_file": rviz_config_file,
        }.items(),
    )

    # GZ-specific nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "so_arm",
            "-allow_renaming",
            "true",
        ],
    )

    gzserver = GzServer(
        world_sdf_file=world_file,
        container_name="ros_gz_container",
        create_own_container="True",
        use_composition="True",
    )

    # Make the /clock topic available in ROS
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        output="screen",
    )

    nodes_to_start = [
        common,
        gz_spawn_entity,
        gzserver,
        gz_sim_bridge,
    ]

    if gazebo_gui.lower() == "true":
        gzgui = ExecuteProcess(
            cmd=["gz", "sim", "-g"],
            output="screen",
        )
        nodes_to_start.append(gzgui)

    return nodes_to_start


def generate_launch_description():
    """Generate launch description with declared arguments."""
    declared_arguments = [
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
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
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
            "description_file",
            default_value=PathJoinSubstitution([FindPackageShare("pai_bringup"), "urdf", "so_arm_gz.urdf.xacro"]),
            description="URDF/XACRO description file (absolute path) with the robot.",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([FindPackageShare("pai_bringup"), "config", "rviz", "so_arm_gz.rviz"]),
            description="Rviz config file (absolute path) to use when launching rviz.",
        ),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start gazebo with GUI?"),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("pai_description"), "world", "so_arm_in_lightbox.sdf"]
            ),
            description="Gazebo world file (absolute path or filename from the "
            "gazebosim worlds collection) containing a custom world.",
        ),
        DeclareLaunchArgument("x", default_value="0.0", description="Robot spawn X position"),
        DeclareLaunchArgument("y", default_value="-0.488", description="Robot spawn Y position"),
        DeclareLaunchArgument("z", default_value="0.845", description="Robot spawn Z position"),
        DeclareLaunchArgument(
            "roll",
            default_value="0.0",
            description="Robot spawn roll orientation (radians)",
        ),
        DeclareLaunchArgument(
            "pitch",
            default_value="0.0",
            description="Robot spawn pitch orientation (radians)",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="1.5708",
            description="Robot spawn yaw orientation (radians)",
        ),
    ]

    return LaunchDescription([*declared_arguments, OpaqueFunction(function=launch_setup)])

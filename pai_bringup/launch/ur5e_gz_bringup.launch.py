# Copyright 2026 Franco Cipollone.
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

"""UR5e Gazebo bringup (Phase A: forward_position_controller, no sensors)."""

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
from ros_gz_sim.actions import GzServer


def launch_setup(context, *args, **kwargs):
    """Set up nodes for the UR5e Gazebo bringup."""
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    ur_type = LaunchConfiguration("ur_type").perform(context)
    activate_joint_controller = LaunchConfiguration("activate_joint_controller").perform(context)
    initial_joint_controller = LaunchConfiguration("initial_joint_controller").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    gazebo_gui = LaunchConfiguration("gazebo_gui").perform(context)
    enable_wrist_cameras = LaunchConfiguration("enable_wrist_cameras").perform(context)
    world_file = LaunchConfiguration("world_file")
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("roll").perform(context)
    pitch = LaunchConfiguration("pitch").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    description_xacro_args = (
        f"simulation_controllers:={controllers_file}"
        f" name:=ur"
        f" ur_type:={ur_type}"
        f" enable_wrist_cameras:={enable_wrist_cameras}"
        f" x:={x} y:={y} z:={z}"
        f" roll:={roll} pitch:={pitch} yaw:={yaw}"
    )

    # Robot description used both by robot_state_publisher (via the
    # common include below) and by the gz spawn entity.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            description_xacro_args,
        ]
    )

    # robot_state_publisher + spawners + optional RViz.
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

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur5e",
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

    # Bridge /clock so use_sim_time works on the ROS side, plus the
    # three Phase B wrist cameras (image + camera_info).
    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    if enable_wrist_cameras.lower() == "true":
        for cam in ("wrist_left_camera", "wrist_center_camera", "wrist_right_camera"):
            bridge_args.append(f"/{cam}/image_raw@sensor_msgs/msg/Image[gz.msgs.Image")
            bridge_args.append(f"/{cam}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo")
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
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
            "ur_type",
            default_value="ur5e",
            description="Type/series of UR robot.",
            choices=[
                "ur3", "ur5", "ur10",
                "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e",
                "ur8long", "ur15", "ur20", "ur30",
            ],
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("pai_bringup"),
                    "config",
                    "control",
                    "ur5e_controllers.yaml",
                ]
            ),
            description="Absolute path to YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate the initial joint controller on start.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="forward_position_controller",
            description="ros2_control controller to spawn after the robot is in Gazebo.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("pai_bringup"), "urdf", "ur5e_gz.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
            ),
            description="Rviz config file (absolute path) to use when launching rviz.",
        ),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start gazebo with GUI?"),
        DeclareLaunchArgument(
            "enable_wrist_cameras",
            default_value="true",
            description="Spawn the three wrist-mounted cameras and bridge their topics (Phase B).",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("pai_description"), "world", "empty_ground.sdf"]
            ),
            description="SDF world file (absolute path) to load in Gazebo.",
        ),
        # Robot spawn pose (UR base on the ground).
        DeclareLaunchArgument("x", default_value="0.0", description="Robot spawn X position"),
        DeclareLaunchArgument("y", default_value="0.0", description="Robot spawn Y position"),
        DeclareLaunchArgument("z", default_value="0.0", description="Robot spawn Z position"),
        DeclareLaunchArgument("roll", default_value="0.0", description="Robot spawn roll (rad)"),
        DeclareLaunchArgument("pitch", default_value="0.0", description="Robot spawn pitch (rad)"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Robot spawn yaw (rad)"),
    ]

    return LaunchDescription([*declared_arguments, OpaqueFunction(function=launch_setup)])

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

"""MoveIt launch file for the SO ARM.

Launches move_group and (optionally) MoveIt-configured RViz using the
upstream ``so_arm101_moveit_config`` package.

Meant to be launched alongside one of the existing bringup files
(Gazebo, MuJoCo, or real hardware) with the ``joint_trajectory_controller``
selected and RViz disabled, e.g.::

    ros2 launch pai_bringup so_arm_gz_bringup.launch.py \
        initial_joint_controller:=joint_trajectory_controller launch_rviz:=false

    ros2 launch pai_bringup so_arm_moveit.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for MoveIt integration."""
    moveit_config_pkg = FindPackageShare("so_arm101_moveit_config")

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, "launch", "move_group.launch.py"]),
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, "launch", "moveit_rviz.launch.py"]),
        ),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz with MoveIt configuration.",
            ),
            move_group_launch,
            moveit_rviz_launch,
        ],
    )

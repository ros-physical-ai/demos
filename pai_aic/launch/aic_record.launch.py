# Copyright 2026 demos contributors
# Apache 2.0

"""Wrap rosetta's episode_recorder_launch with the AIC contract.

Usage:
    ros2 launch pai_aic aic_record.launch.py \
        bag_base_dir:=$HOME/datasets/aic/bags
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    contract_path = PathJoinSubstitution([
        FindPackageShare("pai_data_collection"),
        "config", "rosetta", "aic.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_base_dir",
            default_value="~/datasets/aic/bags",
            description="Where to write recorded rosbag episodes.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use sim time from /clock (Gazebo).",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("rosetta"),
                    "launch", "episode_recorder_launch.py",
                ])
            ),
            launch_arguments={
                "contract_path": contract_path,
                "bag_base_dir": LaunchConfiguration("bag_base_dir"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
    ])
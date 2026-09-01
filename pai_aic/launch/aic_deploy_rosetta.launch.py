# Copyright 2026 demos contributors
# Apache 2.0

"""Wrap rosetta's rosetta_client_launch with the AIC contract (Deploy Path B).

Usage:
    ros2 launch pai_aic aic_deploy_rosetta.launch.py \
        pretrained_name_or_path:=outputs/train/aic_act/checkpoints/last/pretrained_model
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
            "pretrained_name_or_path",
            default_value="",
            description="Path or HF repo ID of the trained LeRobot checkpoint.",
        ),
        DeclareLaunchArgument(
            "policy_type",
            default_value="act",
            description="Policy architecture (act, diffusion, ...).",
        ),
        DeclareLaunchArgument(
            "policy_device",
            default_value="cuda",
            description="Inference device (cuda or cpu).",
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
                    "launch", "rosetta_client_launch.py",
                ])
            ),
            launch_arguments={
                "contract_path": contract_path,
                "pretrained_name_or_path": LaunchConfiguration("pretrained_name_or_path"),
                "policy_type": LaunchConfiguration("policy_type"),
                "policy_device": LaunchConfiguration("policy_device"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
    ])
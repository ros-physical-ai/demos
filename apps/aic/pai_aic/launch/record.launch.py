"""Rosetta's episode recorder with the AIC contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Wrap episode_recorder_launch.py with the contract and simulated time."""
    contract = PathJoinSubstitution([FindPackageShare("pai_aic"), "config", "rosetta", "aic.yaml"])
    recorder = PathJoinSubstitution([FindPackageShare("rosetta"), "launch", "episode_recorder_launch.py"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_base_dir", default_value="datasets/aic/bags"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(recorder),
                launch_arguments={
                    "contract_path": contract,
                    "bag_base_dir": LaunchConfiguration("bag_base_dir"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )

"""Rosetta's policy runner with the AIC contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Wrap policy_runner_launch.py with the contract, runner parameters and simulated time."""
    share = FindPackageShare("pai_aic")
    contract = PathJoinSubstitution([share, "config", "rosetta", "aic.yaml"])
    params = PathJoinSubstitution([share, "config", "rosetta", "policy_runner.yaml"])
    runner = PathJoinSubstitution([FindPackageShare("rosetta"), "launch", "policy_runner_launch.py"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("pretrained_name_or_path", description="Checkpoint directory or Hub id"),
            DeclareLaunchArgument("policy_type", default_value="act"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(runner),
                launch_arguments={
                    "contract_path": contract,
                    "params_file": params,
                    "pretrained_name_or_path": LaunchConfiguration("pretrained_name_or_path"),
                    "policy_type": LaunchConfiguration("policy_type"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )

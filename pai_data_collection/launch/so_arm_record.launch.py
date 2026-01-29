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

"""
Launch file for recording SO-ARM100 episodes using rosetta.

This launch file starts the rosetta episode_recorder node which records
rosbags based on the so_arm100 contract specification.

Usage:
    ros2 launch pai_data_collection so_arm_record.launch.py

    # With custom parameters:
    ros2 launch pai_data_collection so_arm_record.launch.py \
        bag_base_dir:=/path/to/bags \
        episode_seconds:=30
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pai_data_collection_share = get_package_share_directory('pai_data_collection')
    
    # Default contract path.
    default_contract = os.path.join(pai_data_collection_share, 'config', 'rosetta', 'so_arm100.yaml')
    # Default bag directory.
    default_bag_dir = os.path.expanduser('~/datasets/so_arm100/bags')

    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'contract_path',
            default_value=default_contract,
            description='Path to the rosetta contract YAML file.'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'bag_base_dir',
            default_value=default_bag_dir,
            description='Base directory where rosbags will be saved.'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'episode_seconds',
            default_value='60',
            description='Maximum duration of each episode in seconds.'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from /clock topic.'
        )
    )

    # Rosetta episode recorder node
    episode_recorder_node = Node(
        package='rosetta',
        executable='episode_recorder',
        name='episode_recorder',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'contract_path': LaunchConfiguration('contract_path')},
            {'bag_base_dir': LaunchConfiguration('bag_base_dir')},
            {'episode_seconds': LaunchConfiguration('episode_seconds')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription(
        declared_arguments + [episode_recorder_node]
    )

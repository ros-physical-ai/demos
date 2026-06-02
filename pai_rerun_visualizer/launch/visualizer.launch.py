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

"""Launch the pai_rerun_visualizer node with parameters from a YAML file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for the pai_rerun_visualizer node."""
    pkg_share = get_package_share_directory("pai_rerun_visualizer")
    default_params = os.path.join(pkg_share, "config", "visualizer.yaml")

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML parameter file for the visualizer node.",
            ),
            Node(
                package="pai_rerun_visualizer",
                executable="visualizer_node",
                name="pai_rerun_visualizer",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )

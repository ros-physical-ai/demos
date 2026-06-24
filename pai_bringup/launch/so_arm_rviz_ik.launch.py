#!/usr/bin/env python3

# Copyright (C) 2026 Franco Cipollone
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

"""SO-ARM101 interactive-marker IK teleop demo.

Thin wrapper that launches the generic ``pai_rviz_teleop`` interactive IK launch
with the SO-ARM101 teleop configuration. A simulation or hardware bringup and
RViz are expected to be launched separately.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate the SO-ARM101 interactive IK teleop launch description."""
    config_file = os.path.join(
        get_package_share_directory("pai_bringup"), "config", "teleop", "so_arm101_interactive_ik.yaml"
    )

    interactive_ik = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("pai_rviz_teleop"), "launch", "interactive_ik.launch.py")
        ),
        launch_arguments={"config_file": config_file}.items(),
    )

    return LaunchDescription([interactive_ik])

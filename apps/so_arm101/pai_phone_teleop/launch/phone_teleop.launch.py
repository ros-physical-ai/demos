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

"""Generic phone (WebXR) IK teleop launch.

Starts the ``pai_teleop_ik`` IK servo together with the phone teleop adapter
(the teleop.Teleop WebXR bridge), both configured from a single YAML
``config_file``. The default config is a generic template with placeholder joint
names; pass ``config_file:=`` (e.g. a robot-specific file from another package)
to drive a real arm. The user is expected to have launched a ``pai_bringup``
bringup (real, mujoco, or gz) in a separate terminal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the phone teleop launch description."""
    default_config = os.path.join(get_package_share_directory("pai_phone_teleop"), "config", "phone_teleop.yaml")

    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YAML file configuring both the ik_servo and the phone adapter.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock.",
        ),
    ]

    # The same config file feeds the servo (via its standalone launch) and the
    # adapter; the `/**` wildcard in the file applies to both node names.
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("pai_teleop_ik"), "launch", "ik_servo.launch.py")
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("config_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    phone_teleop_node = Node(
        package="pai_phone_teleop",
        executable="phone_teleop_node",
        name="phone_teleop",
        output="both",
        parameters=[
            LaunchConfiguration("config_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([*declared_arguments, servo_launch, phone_teleop_node])

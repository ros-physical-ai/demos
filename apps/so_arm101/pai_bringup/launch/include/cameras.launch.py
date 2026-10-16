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

"""YAML-driven camera launcher for pai_bringup.

Reads a camera registry YAML (default: config/cameras/cameras.yaml) and
spawns one usb_cam node per camera entry.

Published topics per camera (under its namespace):
  <namespace>/image_raw
  <namespace>/camera_info
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_cameras(context):
    pkg = LaunchConfiguration("bringup_pkg").perform(context)
    cameras_cfg = LaunchConfiguration("cameras_config").perform(context)

    pkg_share = get_package_share_directory(pkg)

    with open(cameras_cfg, "r") as f:
        cfg = yaml.safe_load(f) or {}

    nodes = []
    for cam in cfg.get("cameras", []):
        name = cam["name"]
        ns = cam.get("namespace", "")
        param_path = cam["param_path"]

        param_file = (
            param_path if os.path.isabs(param_path) else os.path.join(pkg_share, "config", "cameras", param_path)
        )

        nodes.append(
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name=name,
                namespace=ns,
                parameters=[param_file, {"use_sim_time": False}],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    """Generate launch description with camera nodes based on a YAML registry."""
    bringup_pkg = "pai_bringup"

    return LaunchDescription(
        [
            DeclareLaunchArgument("bringup_pkg", default_value=bringup_pkg),
            DeclareLaunchArgument(
                "cameras_config",
                default_value=os.path.join(
                    get_package_share_directory(bringup_pkg),
                    "config",
                    "cameras",
                    "cameras.yaml",
                ),
                description="Path to camera registry YAML file.",
            ),
            OpaqueFunction(function=_spawn_cameras),
        ]
    )

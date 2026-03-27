#!/usr/bin/env python3

# Copyright (C) 2026 Sebastian Castro, Julia Jia, Franco Cipollone
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import tempfile
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def _generate_mjcf_at_launch(pkg_share):
    """Run xacro on scene and so_arm101 xacros, write to temp dir. Poses from poses_args.xacro."""
    mjcf_dir = Path(pkg_share) / "mjcf"
    scene_xacro = mjcf_dir / "scene.xml.xacro"
    so_arm101_xacro = mjcf_dir / "so_arm101.xml.xacro"
    if not scene_xacro.exists() or not so_arm101_xacro.exists():
        raise FileNotFoundError(
            f"MJCF xacro sources not found. Ensure mjcf/ is installed. Looked for {scene_xacro} and {so_arm101_xacro}"
        )

    out_dir = Path(tempfile.mkdtemp(prefix="pai_bringup_mjcf_"))
    so_arm101_out = out_dir / "so_arm101.xml"
    scene_out = out_dir / "scene.xml"

    meshdir = Path(get_package_share_directory("so_arm101_description")) / "meshes"
    doc = xacro.process_file(str(so_arm101_xacro), mappings={"meshdir": str(meshdir)})
    so_arm101_out.write_text(doc.toxml())

    doc = xacro.process_file(str(scene_xacro))
    scene_out.write_text(doc.toxml())

    return str(scene_out)


def launch_setup(context, *args, **kwargs):
    """Set up nodes for the SO ARM MuJoCo bringup."""
    pkg_share = PathJoinSubstitution([FindPackageShare("pai_bringup")]).perform(context)
    mujoco_model = _generate_mjcf_at_launch(pkg_share)
    description_xacro_args = f"mujoco_model:={mujoco_model}"

    ros2_controllers_file = PathJoinSubstitution(
        [FindPackageShare("pai_bringup"), "config", "control", "ros2_controllers.yaml"]
    )
    controllers_file_replaced = ReplaceString(
        source_file=ros2_controllers_file,
        replacements={"<robot_namespace>": ""},
    )
    controller_parameters = ParameterFile(
        RewrittenYaml(
            source_file=controllers_file_replaced,
            root_key="",
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            controller_parameters,
        ],
    )

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("pai_bringup"),
                    "launch",
                    "include",
                    "so_arm_common.launch.py",
                ]
            )
        ),
        launch_arguments={
            "description_file": PathJoinSubstitution(
                [FindPackageShare("pai_bringup"), "urdf", "so_arm101_mujoco.urdf.xacro"]
            ),
            "description_xacro_args": description_xacro_args,
            "use_sim_time": "true",
            "rviz_config_file": PathJoinSubstitution(
                [
                    FindPackageShare("pai_bringup"),
                    "config",
                    "rviz",
                    "so_arm_mujoco.rviz",
                ]
            ),
        }.items(),
    )

    return [common, control_node]


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription([OpaqueFunction(function=launch_setup)])

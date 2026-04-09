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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def _normalize_world_mjcf_eulerseq(mjcf_path):
    """Neutralize the eulerseq directive emitted by sdformat_mjcf.

    sdformat_mjcf (gz-mujoco) emits ``<compiler eulerseq="XYZ"/>`` in the
    converted MJCF.  Uppercase ``XYZ`` denotes *extrinsic* rotations
    (R = Rz · Ry · Rx), whereas MuJoCo's default lowercase ``xyz`` is
    *intrinsic* (R = Rx · Ry · Rz).

    Because MuJoCo's ``<compiler>`` settings are global — the last one
    parsed wins and applies to the *entire* model — including the world
    via ``<include>`` causes the world's ``eulerseq="XYZ"`` to silently
    override the robot's euler attributes (which were authored for the
    default intrinsic ``xyz``).  Same numbers, different rotation order →
    cameras / bodies end up misoriented.

    Neither placing a ``<compiler eulerseq="xyz"/>`` in the scene template
    nor in the robot file is a robust fix: it would re-interpret any
    non-identity euler values that sdformat_mjcf emitted under ``XYZ``.

    The correct workaround is to convert every euler attribute in the
    world MJCF to a quaternion (which is convention-independent) and
    then strip the ``eulerseq`` directive so it cannot pollute the rest of
    the model.

    See:
        https://github.com/gazebosim/gz-mujoco (sdformat_mjcf)
        https://mujoco.readthedocs.io/en/latest/XMLreference.html#compiler
    """
    import math
    import xml.etree.ElementTree as ET

    def _euler_xyz_extrinsic_to_quat(ex, ey, ez):
        """Convert extrinsic XYZ euler (radians) to wxyz quaternion."""
        cx, sx = math.cos(ex / 2), math.sin(ex / 2)
        cy, sy = math.cos(ey / 2), math.sin(ey / 2)
        cz, sz = math.cos(ez / 2), math.sin(ez / 2)
        # Extrinsic XYZ = intrinsic ZYX: q = qz * qy * qx
        w = cz * cy * cx + sz * sy * sx
        x = cz * cy * sx - sz * sy * cx
        y = cz * sy * cx + sz * cy * sx
        z = sz * cy * cx - cz * sy * sx
        return w, x, y, z

    tree = ET.parse(mjcf_path)
    for elem in tree.iter():
        euler_str = elem.get("euler")
        if euler_str is not None:
            ex, ey, ez = (float(v) for v in euler_str.split())
            w, x, y, z = _euler_xyz_extrinsic_to_quat(ex, ey, ez)
            elem.set("quat", f"{w} {x} {y} {z}")
            del elem.attrib["euler"]
    for comp in tree.findall("compiler"):
        if "eulerseq" in comp.attrib:
            del comp.attrib["eulerseq"]
    tree.write(mjcf_path, xml_declaration=True)


def _generate_mjcf_at_launch(pkg_share, world_sdf_path, arm_base_xyz, arm_base_rpy):
    """Convert an SDF world to MJCF, process robot xacro, and compose the scene.

    Pipeline:
      1. Convert SDF world → MJCF world XML (via sdformat_mjcf)
      2. Process so_arm101.xml.xacro → so_arm101.xml (hand-tuned robot, unchanged)
      3. Process scene_template.xml.xacro → scene.xml (composes world + robot)
    """
    # sdformat_mjcf expects unversioned modules (`sdformat`, `gz.math`), but
    # robostack-kilted ships versioned ones (`sdformat15`, `gz.math8`).
    import sys

    import gz.math8
    import sdformat15

    sys.modules.setdefault("sdformat", sdformat15)
    sys.modules.setdefault("gz.math", gz.math8)

    from sdformat_mjcf.sdformat_to_mjcf.sdformat_to_mjcf import sdformat_file_to_mjcf

    mjcf_dir = Path(pkg_share) / "mjcf"
    scene_template_xacro = mjcf_dir / "scene_template.xml.xacro"
    so_arm101_xacro = mjcf_dir / "so_arm101.xml.xacro"
    if not scene_template_xacro.exists() or not so_arm101_xacro.exists():
        raise FileNotFoundError(
            f"MJCF xacro sources not found. Ensure mjcf/ is installed. "
            f"Looked for {scene_template_xacro} and {so_arm101_xacro}"
        )
    sdf_path = Path(world_sdf_path)
    if not sdf_path.exists():
        raise FileNotFoundError(f"SDF world file not found: {sdf_path}")

    out_dir = Path(tempfile.mkdtemp(prefix="pai_bringup_mjcf_"))

    # Step 1: Convert SDF world → MJCF world XML.
    world_mjcf_out = out_dir / "world.xml"
    ret = sdformat_file_to_mjcf(str(sdf_path), str(world_mjcf_out))
    if ret:
        raise RuntimeError(f"sdformat_mjcf conversion failed (exit code {ret}) for {sdf_path}")

    _normalize_world_mjcf_eulerseq(world_mjcf_out)

    # Step 2: Process robot xacro → MJCF.
    so_arm101_out = out_dir / "so_arm101.xml"
    meshdir = Path(get_package_share_directory("so_arm101_description")) / "meshes"
    doc = xacro.process_file(
        str(so_arm101_xacro),
        mappings={
            "meshdir": str(meshdir),
            "arm_base_x": arm_base_xyz[0],
            "arm_base_y": arm_base_xyz[1],
            "arm_base_z": arm_base_xyz[2],
            "arm_base_roll": arm_base_rpy[0],
            "arm_base_pitch": arm_base_rpy[1],
            "arm_base_yaw": arm_base_rpy[2],
        },
    )
    so_arm101_out.write_text(doc.toxml())

    # Step 3: Compose scene from template.
    scene_out = out_dir / "scene.xml"
    doc = xacro.process_file(
        str(scene_template_xacro),
        mappings={
            "world_mjcf_path": str(world_mjcf_out),
            "robot_mjcf_path": str(so_arm101_out),
        },
    )
    scene_out.write_text(doc.toxml())

    return str(scene_out)


def launch_setup(context, *args, **kwargs):
    """Set up nodes for the SO ARM MuJoCo bringup."""
    pkg_share = PathJoinSubstitution([FindPackageShare("pai_bringup")]).perform(context)
    world_sdf_path = LaunchConfiguration("world_file").perform(context)
    arm_base_xyz = (
        LaunchConfiguration("x").perform(context),
        LaunchConfiguration("y").perform(context),
        LaunchConfiguration("z").perform(context),
    )
    arm_base_rpy = (
        LaunchConfiguration("roll").perform(context),
        LaunchConfiguration("pitch").perform(context),
        LaunchConfiguration("yaw").perform(context),
    )
    mujoco_model = _generate_mjcf_at_launch(pkg_share, world_sdf_path, arm_base_xyz, arm_base_rpy)
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
                    "so_arm_101.rviz",
                ]
            ),
        }.items(),
    )

    return [common, control_node]


def generate_launch_description():
    """Generate launch description."""
    declared_arguments = [
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([FindPackageShare("pai_description"), "world", "so_arm_table.sdf"]),
            description="SDF world file to convert and load in MuJoCo.",
        ),
        DeclareLaunchArgument("x", default_value="0.38", description="Robot arm base X position"),
        DeclareLaunchArgument("y", default_value="0.0", description="Robot arm base Y position"),
        DeclareLaunchArgument("z", default_value="0.4", description="Robot arm base Z position"),
        DeclareLaunchArgument("roll", default_value="0.0", description="Robot arm base roll orientation (radians)"),
        DeclareLaunchArgument("pitch", default_value="0.0", description="Robot arm base pitch orientation (radians)"),
        DeclareLaunchArgument("yaw", default_value="3.14159", description="Robot arm base yaw orientation (radians)"),
    ]
    return LaunchDescription([*declared_arguments, OpaqueFunction(function=launch_setup)])

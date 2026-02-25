#!/usr/bin/env python3
# Copyright (C) 2026 Julia Jia
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

"""Refresh scene (MuJoCo) and world (Gazebo) files from config/scene/poses.yaml.

Regenerates MuJoCo scene, robot model, and URDF. Gazebo world generation
is planned. Run after editing poses. Usage:

  ./scripts/refresh_scene.py
  # or from package root:
  python scripts/refresh_scene.py
"""

import argparse
import re
from pathlib import Path

import yaml


def _package_root():
    """Package root (parent of scripts/ when run from source)."""
    return Path(__file__).resolve().parent.parent


def load_poses(config_path):
    with open(config_path) as f:
        return yaml.safe_load(f)


def pos_str(pose):
    return f"{pose['x']} {pose['y']} {pose['z']}"


def tray_body_attrs(pose):
    """Tray body pos and euler (MuJoCo euler order zyx, radians)."""
    pos = pos_str(pose)
    r, p, y = pose["roll"], pose["pitch"], pose["yaw"]
    if r == 0 and p == 0 and y == 0:
        return f'pos="{pos}"'
    euler = f"{y} {p} {r}"
    return f'pos="{pos}" euler="{euler}"'


def generate_scene_xml(poses, template_path, output_path):
    table = pos_str(poses["table"])
    tray_attrs = tray_body_attrs(poses["tray"])
    cube_small = pos_str(poses["cube_small"])
    cube_medium = pos_str(poses["cube_medium"])
    cube_large = pos_str(poses["cube_large"])

    with open(template_path) as f:
        xml = f.read()

    xml = xml.replace("{table}", table)
    xml = xml.replace("{tray_attrs}", tray_attrs)
    xml = xml.replace("{cube_small}", cube_small)
    xml = xml.replace("{cube_medium}", cube_medium)
    xml = xml.replace("{cube_large}", cube_large)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(xml)


def patch_arm_base_in_mjcf(so_arm101_path, poses):
    arm = poses["arm_base"]
    pos = f'{arm["x"]} {arm["y"]} {arm["z"]}'
    with open(so_arm101_path) as f:
        content = f.read()
    content = re.sub(
        r'(<body name="base_link" )pos="[^"]*"',
        rf'\1pos="{pos}"',
        content,
        count=1,
    )
    with open(so_arm101_path, "w") as f:
        f.write(content)


def _get_paths(pkg_root):
    """Paths for scene refresh outputs. Add gazebo_world when Gazebo support is added."""
    return {
        "config": pkg_root / "config" / "scene" / "poses.yaml",
        "scene_template": pkg_root / "mjcf" / "scene.xml.template",
        "scene_xml": pkg_root / "mjcf" / "scene.xml",
        "so_arm101": pkg_root / "mjcf" / "so_arm101.xml",
        "urdf": pkg_root / "urdf" / "so_arm101_mujoco.urdf.xacro",
        # "gazebo_world": pkg_root / "..." / "so_arm_table_scene.sdf",  # planned
    }


def _run_updates(poses, paths):
    """Run all scene refresh steps. Extend with Gazebo when ready."""
    generate_scene_xml(poses, paths["scene_template"], paths["scene_xml"])
    print(f"Updated {paths['scene_xml']}")

    patch_arm_base_in_mjcf(paths["so_arm101"], poses)
    print(f"Updated {paths['so_arm101']}")

    update_urdf_defaults(paths["urdf"], poses)
    print(f"Updated {paths['urdf']}")

    # generate_gazebo_world(poses, paths["gazebo_world"])  # planned


def update_urdf_defaults(urdf_path, poses):
    with open(urdf_path) as f:
        content = f.read()

    replacements = [
        ("table_x", poses["table"]["x"]),
        ("table_y", poses["table"]["y"]),
        ("table_z", poses["table"]["z"]),
        ("tray_x", poses["tray"]["x"]),
        ("tray_y", poses["tray"]["y"]),
        ("tray_z", poses["tray"]["z"]),
        ("tray_roll", poses["tray"]["roll"]),
        ("tray_pitch", poses["tray"]["pitch"]),
        ("tray_yaw", poses["tray"]["yaw"]),
        ("cube_small_x", poses["cube_small"]["x"]),
        ("cube_small_y", poses["cube_small"]["y"]),
        ("cube_small_z", poses["cube_small"]["z"]),
        ("cube_medium_x", poses["cube_medium"]["x"]),
        ("cube_medium_y", poses["cube_medium"]["y"]),
        ("cube_medium_z", poses["cube_medium"]["z"]),
        ("cube_large_x", poses["cube_large"]["x"]),
        ("cube_large_y", poses["cube_large"]["y"]),
        ("cube_large_z", poses["cube_large"]["z"]),
        ("arm_base_x", poses["arm_base"]["x"]),
        ("arm_base_y", poses["arm_base"]["y"]),
        ("arm_base_z", poses["arm_base"]["z"]),
        ("arm_base_roll", poses["arm_base"]["roll"]),
        ("arm_base_pitch", poses["arm_base"]["pitch"]),
        ("arm_base_yaw", poses["arm_base"]["yaw"]),
    ]

    for name, value in replacements:
        content = re.sub(
            rf'(name="{name}" default=")[^"]*(")',
            rf'\g<1>{value}\2',
            content,
            count=1,
        )

    with open(urdf_path, "w") as f:
        f.write(content)


def main():
    parser = argparse.ArgumentParser(
        description="Refresh scene.xml, so_arm101.xml, and URDF from config/scene/poses.yaml"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to poses.yaml (default: config/scene/poses.yaml in package)",
    )
    parser.add_argument(
        "--package-root",
        type=Path,
        default=None,
        help="Package root (default: parent of scripts/)",
    )
    args = parser.parse_args()

    pkg_root = args.package_root or _package_root()
    paths = _get_paths(pkg_root)
    config_path = args.config or paths["config"]

    if not config_path.exists():
        raise SystemExit(f"Config not found: {config_path}")
    if not paths["scene_template"].exists():
        raise SystemExit(f"Template not found: {paths['scene_template']}")

    poses = load_poses(config_path)
    _run_updates(poses, paths)


if __name__ == "__main__":
    main()

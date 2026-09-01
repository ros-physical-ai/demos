#!/usr/bin/env python3

#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

"""Spawn a full AIC scene from a YAML definition, without the aic_engine.

This mirrors the scene-setup the ``aic_engine`` performs at the start of a
trial (task board with its rails/mounts/cards, plus gripper-attached cables),
but as a standalone command driven by a YAML scene definition. It lets you
compose rich scenes (NIC card mounts, SC ports, LC/SFP/SC mounts) for
teleoperation and dataset recording.

Scene YAML format (identical to the ``scene:`` block of the aic_engine config):

    # optional, overrides the default rail translation clamp limits
    task_board_limits:
      nic_rail:   {min_translation: -0.048,   max_translation: 0.036}
      sc_rail:    {min_translation: -0.055,   max_translation: 0.055}
      mount_rail: {min_translation: -0.09625, max_translation: 0.09625}

    scene:
      task_board:
        pose: {x: 0.15, y: -0.2, z: 1.14, roll: 0.0, pitch: 0.0, yaw: 3.1415}
        nic_rail_0:
          entity_present: true
          entity_pose: {translation: 0.036, roll: 0.0, pitch: 0.0, yaw: 0.0}
        sc_rail_0:
          entity_present: true
          entity_pose: {translation: 0.042, roll: 0.0, pitch: 0.0, yaw: 0.1}
        lc_mount_rail_0:
          entity_present: true
          entity_pose: {translation: 0.02, roll: 0.0, pitch: 0.0, yaw: 0.0}
        # ... nic_rail_1..4, sc_rail_1, sfp_mount_rail_0/1, sc_mount_rail_0/1,
        #     lc_mount_rail_1 (each defaults to entity_present: false)
      cables:
        cable_0:
          pose:
            gripper_offset: {x: 0.0, y: 0.015385, z: 0.04245}
            roll: 0.4432
            pitch: -0.4838
            yaw: 1.3303
          attach_cable_to_gripper: true
          cable_type: sfp_sc_cable   # or sfp_sc_cable_reversed

The same file used by the engine works directly: pass the engine config and
select a trial with ``--trial trial_1``.

The script installs to ``share/pai_aic/scripts`` (ament_python), so run it by
path, e.g. via ``pixi run`` or directly:

Examples:
    # Spawn the default example scene shipped with pai_aic.
    python3 $(ros2 pkg prefix pai_aic)/share/pai_aic/scripts/aic_spawn_scene.py

    # Spawn trial_2 straight from the engine's sample config.
    python3 $(ros2 pkg prefix pai_aic)/share/pai_aic/scripts/aic_spawn_scene.py \
        --scene-file $(ros2 pkg prefix aic_engine)/share/aic_engine/config/sample_config.yaml \
        --trial trial_2

    # Clear any existing task board / cables first, then spawn.
    python3 $(ros2 pkg prefix pai_aic)/share/pai_aic/scripts/aic_spawn_scene.py --clear
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import DeleteEntity, SpawnEntity
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

# Default rail translation clamp limits. Keep in sync with the defaults used by
# aic_engine (see Engine::spawn_entity in aic_engine.cpp).
DEFAULT_LIMITS = {
    "nic_rail": {"min_translation": -0.048, "max_translation": 0.036},
    "sc_rail": {"min_translation": -0.055, "max_translation": 0.055},
    "mount_rail": {"min_translation": -0.09625, "max_translation": 0.09625},
}

# NIC rails map to nic_card_mount_{i}; SC rails map to sc_port_{i}. The
# type-specific mount rails use their own key as the xacro-argument prefix.
NIC_RAILS = [f"nic_rail_{i}" for i in range(5)]
SC_RAILS = [f"sc_rail_{i}" for i in range(2)]
MOUNT_RAILS = [
    "lc_mount_rail_0",
    "sfp_mount_rail_0",
    "sc_mount_rail_0",
    "lc_mount_rail_1",
    "sfp_mount_rail_1",
    "sc_mount_rail_1",
]

SERVICE_TIMEOUT_SEC = 10.0


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll/pitch/yaw (radians) to a geometry_msgs Quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def load_scene(path: str, trial: str | None):
    """Load a scene definition and its clamp limits from a YAML file."""
    with open(path, "r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle)
    if not isinstance(doc, dict):
        raise ValueError(f"Scene file '{path}' did not parse to a mapping.")

    limits = doc.get("task_board_limits", DEFAULT_LIMITS)

    if "trials" in doc:
        if trial is None:
            available = ", ".join(doc["trials"].keys())
            raise ValueError(
                f"'{path}' contains trials; select one with --trial "
                f"(available: {available})."
            )
        if trial not in doc["trials"]:
            available = ", ".join(doc["trials"].keys())
            raise ValueError(
                f"Trial '{trial}' not found in '{path}' (available: {available})."
            )
        scene = doc["trials"][trial]["scene"]
    elif "scene" in doc:
        scene = doc["scene"]
    else:
        scene = doc

    return scene, limits


class AICSpawnSceneNode(Node):
    """One-shot node that spawns a full AIC scene from a YAML definition."""

    def __init__(self, scene: dict, limits: dict, args: argparse.Namespace):
        super().__init__("aic_spawn_scene")
        self.scene = scene
        self.limits = limits
        self.args = args

        self.spawn_client = self.create_client(SpawnEntity, "/gz_server/spawn_entity")
        self.delete_client = self.create_client(
            DeleteEntity, "/gz_server/delete_entity"
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # -- service helpers --------------------------------------------------------

    def _call(self, client, request, description: str):
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            self.get_logger().error(
                f"Service '{client.srv_name}' not available; skipping {description}."
            )
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT_SEC)
        if not future.done():
            self.get_logger().error(f"{description} timed out.")
            return None
        return future.result()

    def _run_xacro(self, xacro_file: str, xacro_args: list[str]) -> str | None:
        try:
            result = subprocess.run(
                ["xacro", xacro_file, *xacro_args],
                check=True,
                capture_output=True,
                text=True,
            )
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            self.get_logger().error(f"xacro expansion failed: {exc}")
            return None
        if not result.stdout.strip():
            self.get_logger().error("Expanded description is empty.")
            return None
        return result.stdout

    def _spawn(
        self,
        name: str,
        resource_string: str,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> bool:
        request = SpawnEntity.Request()
        request.name = name
        request.allow_renaming = True
        request.uri = ""
        request.resource_string = resource_string
        request.entity_namespace = ""
        request.initial_pose.header.frame_id = "world"
        request.initial_pose.pose.position.x = x
        request.initial_pose.pose.position.y = y
        request.initial_pose.pose.position.z = z
        request.initial_pose.pose.orientation = rpy_to_quaternion(roll, pitch, yaw)

        response = self._call(self.spawn_client, request, f"spawn '{name}'")
        if response is None or response.result.result != Result.RESULT_OK:
            message = (
                response.result.error_message if response is not None else "no response"
            )
            self.get_logger().error(f"Failed to spawn '{name}': {message}")
            return False
        self.get_logger().info(f"Spawned '{response.entity_name}'.")
        return True

    def _delete(self, name: str) -> None:
        request = DeleteEntity.Request()
        request.entity = name
        response = self._call(self.delete_client, request, f"delete '{name}'")
        if response is None:
            return
        if response.result.result == Result.RESULT_OK:
            self.get_logger().info(f"Deleted '{name}'.")
        elif response.result.result != Result.RESULT_NOT_FOUND:
            self.get_logger().warning(
                f"Could not delete '{name}': {response.result.error_message}"
            )

    # -- task board -------------------------------------------------------------

    def _rail_args(self, prefix: str, rail_cfg: dict, limit_key: str) -> list[str]:
        """Build the xacro args for one rail slot (present + pose, clamped)."""
        present = bool(rail_cfg.get("entity_present", False))
        if not present:
            return [f"{prefix}_present:=false"]

        args = [f"{prefix}_present:=true"]
        pose = rail_cfg.get("entity_pose")
        if pose is not None:
            limit = self.limits.get(limit_key, DEFAULT_LIMITS[limit_key])
            translation = clamp(
                float(pose.get("translation", 0.0)),
                float(limit["min_translation"]),
                float(limit["max_translation"]),
            )
            args += [
                f"{prefix}_translation:={translation}",
                f"{prefix}_roll:={float(pose.get('roll', 0.0))}",
                f"{prefix}_pitch:={float(pose.get('pitch', 0.0))}",
                f"{prefix}_yaw:={float(pose.get('yaw', 0.0))}",
            ]
        return args

    def spawn_task_board(self) -> bool:
        board = self.scene.get("task_board")
        if board is None:
            self.get_logger().info("No task_board in scene; skipping.")
            return True

        xacro_args: list[str] = []
        for i, rail in enumerate(NIC_RAILS):
            xacro_args += self._rail_args(
                f"nic_card_mount_{i}", board.get(rail, {}), "nic_rail"
            )
        for i, rail in enumerate(SC_RAILS):
            xacro_args += self._rail_args(
                f"sc_port_{i}", board.get(rail, {}), "sc_rail"
            )
        for rail in MOUNT_RAILS:
            xacro_args += self._rail_args(rail, board.get(rail, {}), "mount_rail")
        xacro_args.append(
            f"ground_truth:={'true' if self.args.ground_truth else 'false'}"
        )

        share = get_package_share_directory("aic_description")
        resource_string = self._run_xacro(
            f"{share}/urdf/task_board.urdf.xacro", xacro_args
        )
        if resource_string is None:
            return False

        pose = board.get("pose", {})
        return self._spawn(
            "task_board",
            resource_string,
            float(pose.get("x", 0.0)),
            float(pose.get("y", 0.0)),
            float(pose.get("z", 0.0)),
            float(pose.get("roll", 0.0)),
            float(pose.get("pitch", 0.0)),
            float(pose.get("yaw", 0.0)),
        )

    # -- cables -----------------------------------------------------------------

    def spawn_cables(self) -> bool:
        cables = self.scene.get("cables")
        if not cables:
            self.get_logger().info("No cables in scene; skipping.")
            return True

        ok = True
        gripper_frame = self.args.gripper_frame
        for cable_name, cable_cfg in cables.items():
            attach = bool(cable_cfg.get("attach_cable_to_gripper", False))
            cable_type = cable_cfg.get("cable_type", "sfp_sc_cable")
            pose = cable_cfg.get("pose", {})
            offset = pose.get("gripper_offset", {})

            share = get_package_share_directory("aic_description")
            resource_string = self._run_xacro(
                f"{share}/urdf/cable.sdf.xacro",
                [
                    f"attach_cable_to_gripper:={'true' if attach else 'false'}",
                    f"cable_type:={cable_type}",
                ],
            )
            if resource_string is None:
                ok = False
                continue

            # Cables are posed relative to the current gripper pose.
            try:
                transform = self.tf_buffer.lookup_transform(
                    "world", gripper_frame, rclpy.time.Time()
                )
            except (
                LookupException,
                ConnectivityException,
                ExtrapolationException,
            ) as exc:
                self.get_logger().error(
                    f"Could not look up 'world' -> '{gripper_frame}' for "
                    f"'{cable_name}': {exc}"
                )
                ok = False
                continue

            ok &= self._spawn(
                cable_name,
                resource_string,
                transform.transform.translation.x + float(offset.get("x", 0.0)),
                transform.transform.translation.y + float(offset.get("y", 0.0)),
                transform.transform.translation.z + float(offset.get("z", 0.0)),
                float(pose.get("roll", 0.0)),
                float(pose.get("pitch", 0.0)),
                float(pose.get("yaw", 0.0)),
            )
        return ok

    # -- orchestration ----------------------------------------------------------

    def run(self) -> bool:
        if self.args.clear:
            self._delete("task_board")
            for cable_name in self.scene.get("cables", {}):
                self._delete(cable_name)

        ok = True
        if not self.args.no_task_board:
            ok &= self.spawn_task_board()
        if not self.args.no_cables:
            ok &= self.spawn_cables()
        return ok


def parse_args(argv):
    default_scene = (
        f"{get_package_share_directory('pai_aic')}/config/scene_example.yaml"
    )
    parser = argparse.ArgumentParser(
        description="Spawn a full AIC scene from a YAML definition."
    )
    parser.add_argument(
        "--scene-file",
        default=default_scene,
        help="Path to the scene YAML (default: aic_bringup scene_example.yaml).",
    )
    parser.add_argument(
        "--trial",
        default=None,
        help="If the file has a 'trials' block (e.g. the engine sample config), "
        "select this trial.",
    )
    parser.add_argument(
        "--gripper-frame",
        default="gripper/tcp",
        help="TF frame cables are spawned relative to (default: gripper/tcp).",
    )
    parser.add_argument(
        "--ground-truth",
        action="store_true",
        help="Expand the task board with ground-truth TF frames.",
    )
    parser.add_argument(
        "--clear",
        action="store_true",
        help="Delete the task board and scene cables before spawning.",
    )
    parser.add_argument(
        "--no-task-board",
        action="store_true",
        help="Skip spawning the task board.",
    )
    parser.add_argument(
        "--no-cables",
        action="store_true",
        help="Skip spawning cables.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    args = parse_args(argv)

    try:
        scene, limits = load_scene(args.scene_file, args.trial)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"Failed to load scene: {exc}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = AICSpawnSceneNode(scene, limits, args)
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not success:
        print("Scene spawn finished with errors.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

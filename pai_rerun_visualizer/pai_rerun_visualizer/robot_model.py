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

"""Pinocchio-backed kinematic model built from a live URDF string.

This module isolates everything that depends on ``pinocchio`` so the ROS node
stays focused on message plumbing and Rerun logging. It provides:

* forward kinematics for arbitrary URDF frames given a joint name -> value map,
* parsing of the URDF ``<visual>`` meshes (path, origin, scale) so the meshes
  can be logged to Rerun as ``rr.Asset3D`` and posed each frame.
"""

from __future__ import annotations

import logging
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pinocchio as pin
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

logger = logging.getLogger(__name__)


@dataclass
class VisualMesh:
    """A single URDF visual mesh attached to a link."""

    link: str  # URDF link (frame) the visual is rigidly attached to
    mesh_path: str  # absolute filesystem path to the mesh file
    origin: np.ndarray  # 4x4 link_T_visual transform
    scale: np.ndarray  # (3,) per-axis mesh scale
    color: Optional[np.ndarray] = None  # RGBA in [0, 1] from URDF <material>, or None if unset


def _xyz_rpy_to_matrix(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """Build a 4x4 homogeneous transform from URDF xyz + rpy (radians)."""
    rot = pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = np.asarray(xyz, dtype=float)
    return mat


def _resolve_mesh_path(filename: str) -> Optional[str]:
    """Resolve a URDF mesh ``filename`` (package:// or file://) to a real path."""
    if filename.startswith("package://"):
        rest = filename[len("package://") :]
        pkg, _, rel = rest.partition("/")
        try:
            share = get_package_share_directory(pkg)
        except PackageNotFoundError:
            return None
        return os.path.join(share, rel)
    if filename.startswith("file://"):
        return filename[len("file://") :]
    return filename if os.path.isabs(filename) else None


class RobotModel:
    """Forward kinematics + visual geometry parsed from a URDF XML string."""

    def __init__(self, urdf_xml: str) -> None:
        """Build the pinocchio model from a URDF XML string and parse visual meshes."""
        self._model = pin.buildModelFromXML(urdf_xml)
        self._data = self._model.createData()
        # Map every actuated joint name to its configuration index in ``q``.
        self._name_to_qidx: dict[str, int] = {}
        for jid in range(1, self._model.njoints):  # joint 0 is the universe
            joint = self._model.joints[jid]
            if joint.nq == 1:
                self._name_to_qidx[self._model.names[jid]] = joint.idx_q
        self._q = pin.neutral(self._model)
        self.visuals: list[VisualMesh] = self._parse_visuals(urdf_xml)

    # -- configuration -------------------------------------------------
    def set_joint_positions(self, name_to_value: dict[str, float]) -> None:
        """Update the internal configuration from a joint name -> radians map."""
        for name, value in name_to_value.items():
            qidx = self._name_to_qidx.get(name)
            if qidx is not None:
                self._q[qidx] = value
        pin.forwardKinematics(self._model, self._data, self._q)
        pin.updateFramePlacements(self._model, self._data)

    def frame_pose(self, frame: str) -> Optional[np.ndarray]:
        """Return the 4x4 world_T_frame transform, or ``None`` if unknown."""
        if not self._model.existFrame(frame):
            return None
        fid = self._model.getFrameId(frame)
        placement = self._data.oMf[fid]
        mat = np.eye(4)
        mat[:3, :3] = placement.rotation
        mat[:3, 3] = placement.translation
        return mat

    def has_frame(self, frame: str) -> bool:
        """Return ``True`` if *frame* exists in the pinocchio model."""
        return self._model.existFrame(frame)

    # -- URDF visual parsing -------------------------------------------
    @staticmethod
    def _parse_visuals(urdf_xml: str) -> list[VisualMesh]:
        """Extract all mesh-backed visual elements from the URDF XML.

        Walks every ``<link>/<visual>/<geometry>/<mesh>`` element, resolves the
        mesh file path, reads the ``<origin>`` transform and per-axis scale, and
        returns a list of :class:`VisualMesh` instances ready for Rerun logging.
        Primitive geometries (box / cylinder / sphere) are silently skipped.
        """
        visuals: list[VisualMesh] = []
        try:
            root = ET.fromstring(urdf_xml)
        except ET.ParseError:
            return visuals

        # Pre-parse robot-level named materials: <material name="..."><color rgba="..."/>.
        named_materials: dict[str, np.ndarray] = {}
        for mat in root.findall("material"):
            mat_name = mat.get("name")
            color_el = mat.find("color")
            if mat_name and color_el is not None:
                rgba_str = color_el.get("rgba")
                if rgba_str:
                    named_materials[mat_name] = np.array([float(v) for v in rgba_str.split()], dtype=float)

        for link in root.findall("link"):
            link_name = link.get("name")
            if not link_name:
                continue
            for visual in link.findall("visual"):
                geometry = visual.find("geometry")
                if geometry is None:
                    continue
                mesh = geometry.find("mesh")
                if mesh is None:
                    logging.warning(
                        "URDF visual geometry for link '%s' is not a mesh, skipping (only <mesh> is supported)",
                        link_name,
                    )
                    continue
                filename = mesh.get("filename")
                if not filename:
                    continue
                mesh_path = _resolve_mesh_path(filename)
                if not mesh_path or not os.path.exists(mesh_path):
                    continue

                origin_el = visual.find("origin")
                xyz = [0.0, 0.0, 0.0]
                rpy = [0.0, 0.0, 0.0]
                if origin_el is not None:
                    if origin_el.get("xyz"):
                        xyz = [float(v) for v in origin_el.get("xyz").split()]
                    if origin_el.get("rpy"):
                        rpy = [float(v) for v in origin_el.get("rpy").split()]
                origin = _xyz_rpy_to_matrix(xyz, rpy)

                scale = np.ones(3)
                if mesh.get("scale"):
                    scale = np.asarray([float(v) for v in mesh.get("scale").split()])

                # Resolve material color: prefer inline <color rgba="...">,
                # fall back to a robot-level named material.
                color: Optional[np.ndarray] = None
                material_el = visual.find("material")
                if material_el is not None:
                    color_el = material_el.find("color")
                    if color_el is not None:
                        rgba_str = color_el.get("rgba")
                        if rgba_str:
                            color = np.array([float(v) for v in rgba_str.split()], dtype=float)
                    if color is None:
                        ref_name = material_el.get("name")
                        if ref_name:
                            color = named_materials.get(ref_name)

                visuals.append(
                    VisualMesh(
                        link=link_name,
                        mesh_path=mesh_path,
                        origin=origin,
                        scale=scale,
                        color=color,
                    )
                )
        return visuals

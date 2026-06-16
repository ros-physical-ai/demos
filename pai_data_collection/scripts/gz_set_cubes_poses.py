#!/usr/bin/env python3
r"""Reset or randomize the pose of cubes in the pai Gazebo world.

It shells out to ``gz service`` to set the pose of the three cubes defined in
``pai_description/world/so_arm_table.sdf`` inside ``/world/pai_world``.

Usage:
    # Reset to the hardcoded nominal pose.
    python pai_data_collection/scripts/gz_set_cubes_poses.py

    # Randomize around the nominal pose.
    python pai_data_collection/scripts/gz_set_cubes_poses.py --random

    # Custom randomization range and seed.
    python pai_data_collection/scripts/gz_set_cubes_poses.py --random \\
        --radius 0.05 --angle-range 180 --seed 0

    # Override the nominal pose of one cube.
    python pai_data_collection/scripts/gz_set_cubes_poses.py \\
        --pose cube_small=0.16,-0.11,0.41,0,0,0,1

    # Print the gz commands without executing them.
    python pai_data_collection/scripts/gz_set_cubes_poses.py --dry-run
"""

from __future__ import annotations

import argparse
import math
import random
import subprocess
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class Cube:
    """A cube in ``/world/pai_world`` with its nominal pose.

    The orientation is a unit quaternion ``(qx, qy, qz, qw)``. Authoritative
    values are defined in ``pai_description/world/so_arm_table.sdf`` (Euler
    roll/pitch/yaw on the ``<pose>`` element); the quaternion below is the
    same pose converted via the z-axis-only yaw formula
    ``(0, 0, sin(yaw/2), cos(yaw/2))``.
    """

    name: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


# Nominal poses — keep in sync with pai_description/world/so_arm_table.sdf.
#   cube_small:  pose="0.16 -0.11 0.41  0 0 0.06"  (yaw 0.06 rad)
#   cube_medium: pose="0.17  0.05 0.41  0 0 0"     (identity)
#   cube_large:  pose="0.12  0.20 0.41  0 0 -0.73" (yaw -0.73 rad)
DEFAULT_CUBES: list[Cube] = [
    Cube("cube_small", 0.16, -0.11, 0.41, 0.0, 0.0, 0.0299955, 0.99955),
    Cube("cube_medium", 0.17, 0.05, 0.41, 0.0, 0.0, 0.0, 1.0),
    Cube("cube_large", 0.12, 0.20, 0.41, 0.0, 0.0, -0.3569493, 0.9341238),
]


def parse_pose_arg(value: str, known_names: set[str]) -> Cube:
    """Parse a ``--pose NAME=x,y,z,qx,qy,qz,qw`` argument into a Cube.

    Raises ``argparse.ArgumentTypeError`` for any malformed input.
    """
    if "=" not in value:
        raise argparse.ArgumentTypeError(f"--pose {value!r}: expected NAME=x,y,z,qx,qy,qz,qw")
    name, _, raw = value.partition("=")
    name = name.strip()
    if name not in known_names:
        raise argparse.ArgumentTypeError(f"--pose: unknown cube {name!r}; valid names are {sorted(known_names)}")
    parts = [p.strip() for p in raw.split(",")]
    if len(parts) != 7:  # noqa: PLR2004
        raise argparse.ArgumentTypeError(f"--pose {value!r}: expected 7 comma-separated values, got {len(parts)}")
    try:
        floats = [float(p) for p in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"--pose {value!r}: {exc}") from exc
    x, y, z, qx, qy, qz, qw = floats
    return Cube(name=name, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw)


def apply_pose_overrides(cubes: list[Cube], pose_args: list[str]) -> list[Cube]:
    """Return a new list of cubes with any ``--pose`` overrides applied.

    The last override for a given cube name wins.
    """
    by_name = {c.name: c for c in cubes}
    known = set(by_name)
    for raw in pose_args:
        override = parse_pose_arg(raw, known)
        by_name[override.name] = override
    return [by_name[c.name] for c in cubes]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse and validate the command-line arguments."""
    parser = argparse.ArgumentParser(
        description=("Reset or randomize the pose of cubes in the pai Gazebo world."),
    )
    parser.add_argument(
        "--random",
        action="store_true",
        help="Randomize pose around the (possibly overridden) nominal.",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=0.05,
        metavar="R",
        help="Radius (m) for x,y perturbation when --random is set (default: 0.05).",
    )
    parser.add_argument(
        "--angle-range",
        type=float,
        default=180.0,
        metavar="DEG",
        help=("Total sweep of the z-axis rotation (deg) when --random is set (default: 180)."),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Seed for reproducible randomization (only used with --random).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print gz service commands instead of executing them.",
    )
    parser.add_argument(
        "--pose",
        action="append",
        default=[],
        metavar="NAME=x,y,z,qx,qy,qz,qw",
        help=("Override the nominal pose of cube NAME. Repeatable; the last occurrence wins."),
    )
    return parser.parse_args(argv)


def randomize_position(cube: Cube, radius: float, rng: random.Random) -> tuple[float, float]:
    """Sample a new (x, y) uniformly inside a disk of `radius` around the cube.

    The disk is sampled with ``r = R * sqrt(u)`` (not ``R * u``) so the points
    are uniformly distributed in area, not clustered near the center.
    """
    r = radius * math.sqrt(rng.random())
    theta = 2.0 * math.pi * rng.random()
    return cube.x + r * math.cos(theta), cube.y + r * math.sin(theta)


def random_orientation_z(angle_range_deg: float, rng: random.Random) -> tuple[float, float, float, float]:
    """Sample a fresh z-axis quaternion with sweep ``angle_range_deg``.

    `angle_range_deg` is the total sweep centered on 0 (e.g. 180 means
    uniform on ``[-90 deg, +90 deg]``).
    """
    phi = math.radians(rng.uniform(-angle_range_deg / 2.0, angle_range_deg / 2.0))
    half = phi / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def build_pose_request(cube: Cube) -> str:
    """Build the ``--req`` payload matching the legacy bash script exactly."""
    return (
        f"name: '{cube.name}', "
        f"position: {{x: {cube.x}, y: {cube.y}, z: {cube.z}}}, "
        f"orientation: {{x: {cube.qx}, y: {cube.qy}, z: {cube.qz}, w: {cube.qw}}}"
    )


def format_gz_command(req: str) -> str:
    """Format the full ``gz service ... --req '...'`` command for one cube.

    Returns a single shell-quoted line suitable for printing in --dry-run.
    """
    return f"gz service -s /world/pai_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req '{req}'"


def run_gz_invocations(cubes: list[Cube], dry_run: bool) -> int:
    """Run the gz service call for every cube in dry-run or live mode.

    In dry-run mode, prints the gz service commands. In live mode, invokes
    them in parallel via ``subprocess.Popen``. Returns 0 on success, or
    raises ``RuntimeError`` naming the failing cubes if any live ``gz``
    invocation exits non-zero.
    """
    if dry_run:
        for cube in cubes:
            print(format_gz_command(build_pose_request(cube)))
        return 0

    procs: list[tuple[Cube, subprocess.Popen]] = []
    for cube in cubes:
        req = build_pose_request(cube)
        cmd = [
            "gz",
            "service",
            "-s",
            "/world/pai_world/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--req",
            req,
        ]
        procs.append((cube, subprocess.Popen(cmd)))

    failures: list[tuple[str, int]] = []
    for cube, proc in procs:
        rc = proc.wait()
        if rc != 0:
            failures.append((cube.name, rc))
    if failures:
        details = ", ".join(f"{name} (exit {rc})" for name, rc in failures)
        raise RuntimeError(f"gz service failed for: {details}")
    return 0


def main(argv: list[str] | None = None) -> int:
    """Entry point. Returns a process exit code."""
    args = parse_args(argv)
    resolved = apply_pose_overrides(DEFAULT_CUBES, args.pose)

    rng: random.Random | None = None
    if args.random:
        rng = random.Random(args.seed)

    cubes_to_set: list[Cube] = []
    for cube in resolved:
        if args.random:
            assert rng is not None
            x, y = randomize_position(cube, args.radius, rng)
            qx, qy, qz, qw = random_orientation_z(args.angle_range, rng)
            z = cube.z
        else:
            x, y, z = cube.x, cube.y, cube.z
            qx, qy, qz, qw = cube.qx, cube.qy, cube.qz, cube.qw
        cubes_to_set.append(Cube(name=cube.name, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw))

    try:
        return run_gz_invocations(cubes_to_set, args.dry_run)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Derive joint limits in radians from LeRobot calibration JSON.

This script reads the per-robot calibration file (follower_arm.json) and
computes joint limits in radians using the same conversion as the
feetech_ros2_driver.  The output can be used to patch URDF and MJCF files
so that all representations agree on the hardware's actual range of motion.

Encoder-to-radian formula (from feetech_ros2_driver/common.hpp):
    joint_rad = (raw_encoder - offset) * 2π / 4096

where *offset* is the ros2_control ``<param name="offset">`` value
(defaults to 2048 — the STS3215 servo centre).  The driver subtracts the
offset from the raw reading before converting.

range_min / range_max in the calibration JSON are raw servo register limits.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import yaml


ENCODER_RESOLUTION = 4096
DEFAULT_OFFSET = 2048  # STS3215 centre position

# Ordered list of bare joint names (LeRobot native format).
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Bare name ↔ ROS joint name mapping.
BARE_TO_ROS = {name: f"{name}_joint" for name in JOINT_NAMES}
ROS_TO_BARE = {v: k for k, v in BARE_TO_ROS.items()}

DEFAULT_SERVO_DEFAULTS_PATH = (
    Path(__file__).resolve().parent.parent / "config" / "servo_defaults" / "so_arm101.yaml"
)


def load_servo_defaults(path: Path = DEFAULT_SERVO_DEFAULTS_PATH) -> dict:
    """Load servo defaults YAML and flatten all_joints + per-joint overrides.

    Returns {bare_name: {param: value, ...}} ready for merging into generated YAML.
    """
    with open(path) as f:
        raw = yaml.safe_load(f)

    all_joints = raw.get("all_joints", {})
    per_joint = raw.get("joints", {})

    defaults = {}
    for name in JOINT_NAMES:
        merged = dict(all_joints)
        merged.update(per_joint.get(name, {}))
        defaults[name] = merged
    return defaults


def encoder_to_rad(raw: int, offset: int = DEFAULT_OFFSET) -> float:
    """Convert a raw encoder value to radians."""
    return (raw - offset) * 2.0 * math.pi / ENCODER_RESOLUTION


def normalize_calibration(data: dict) -> dict:
    """Accept calibration with either naming convention, return bare-name dict."""
    normalized = {}
    for name, params in data.items():
        bare = ROS_TO_BARE.get(name, name)
        if bare not in BARE_TO_ROS:
            print(f"Warning: unknown joint name '{name}', passing through", file=sys.stderr)
        normalized[bare] = params
    return normalized


def load_calibration(path: Path) -> dict:
    with open(path) as f:
        data = json.load(f)
    return normalize_calibration(data)


def resolve_robot_calibration(robot_id: str, arm: str = "follower") -> Path:
    """Resolve robot_id to a calibration JSON path relative to this script."""
    config_dir = Path(__file__).resolve().parent.parent / "config" / "lerobots" / robot_id
    path = config_dir / f"{arm}_arm.json"
    if not path.exists():
        raise FileNotFoundError(
            f"Calibration file not found: {path}\n"
            f"Run LeRobot calibration and copy the JSON to config/lerobots/{robot_id}/")
    return path


# URDF reference limits (vendor xacro: so_arm101_macro.xacro).
# Used for comparison / validation, not as source of truth.
# Keyed by bare name.
URDF_LIMITS = {
    "shoulder_pan":  (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex":    (-1.69,    1.54),
    "wrist_flex":    (-1.60,    1.60),
    "wrist_roll":    (-2.30,    2.30),
    "gripper":       ( 0.00,    1.00),
}


def derive_limits(calibration: dict, offset: int = DEFAULT_OFFSET) -> dict:
    """Return {bare_name: (lower_rad, upper_rad)} from calibration data."""
    limits = {}
    for name, params in calibration.items():
        lower = encoder_to_rad(params["range_min"], offset)
        upper = encoder_to_rad(params["range_max"], offset)
        if lower > upper:
            lower, upper = upper, lower
        limits[name] = (lower, upper)
    return limits


def print_comparison(derived: dict) -> None:
    print(f"{'Joint':<25} {'Derived lower':>14} {'Derived upper':>14}"
          f"  {'URDF lower':>11} {'URDF upper':>11}  {'Δ lower':>8} {'Δ upper':>8}")
    print("-" * 105)
    for name, (d_lo, d_hi) in derived.items():
        ros_name = BARE_TO_ROS.get(name, name)
        u_lo, u_hi = URDF_LIMITS.get(name, (float("nan"), float("nan")))
        print(f"{ros_name:<25} {d_lo:>14.5f} {d_hi:>14.5f}"
              f"  {u_lo:>11.5f} {u_hi:>11.5f}"
              f"  {d_lo - u_lo:>+8.4f} {d_hi - u_hi:>+8.4f}")


def print_mjcf_patch(limits: dict) -> None:
    """Print MJCF joint range and actuator ctrlrange values."""
    print("\n--- MJCF joint range values ---")
    for name, (lo, hi) in limits.items():
        ros_name = BARE_TO_ROS.get(name, name)
        print(f'  range="{lo:.5f} {hi:.5f}"   <!-- {ros_name} -->')
    print("\n--- MJCF actuator ctrlrange values ---")
    for name, (lo, hi) in limits.items():
        ros_name = BARE_TO_ROS.get(name, name)
        print(f'  ctrlrange="{lo:.5f} {hi:.5f}"   <!-- {ros_name} -->')


def generate_yaml(
    calibration: dict,
    servo_defaults: dict,
    output_path: Path,
    source_desc: str = "<calibration.json>",
    defaults_path: Path = DEFAULT_SERVO_DEFAULTS_PATH,
) -> None:
    """Generate a follower.yaml for the feetech_ros2_driver.

    Per-robot calibration fields (id, homing_offset, range_min, range_max) come
    from the calibration JSON.  Servo-default fields (PID, torque) are merged
    from config/servo_defaults/so_arm101.yaml.
    Output uses ROS joint names (_joint suffix).
    """
    lines = [
        "# Generated — do not edit by hand.",
        f"# Sources:",
        f"#   calibration: {source_desc}",
        f"#   servo defaults: {defaults_path.name}",
        f"# Re-generate with: pixi run python3 pai_bringup/scripts/generate_limits.py \\",
        f"#     {source_desc} --generate-yaml {output_path}",
        "joints:",
    ]
    for bare_name, params in calibration.items():
        ros_name = BARE_TO_ROS.get(bare_name, bare_name)
        defaults = servo_defaults.get(bare_name, {})
        lines.append(f"  {ros_name}:")
        lines.append(f"    id: {params['id']}")
        lines.append(f"    homing_offset: {params['homing_offset']}")
        lines.append(f"    range_min: {params['range_min']}")
        lines.append(f"    range_max: {params['range_max']}")
        for key, value in defaults.items():
            lines.append(f"    {key}: {value}")
        lines.append("")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n")
    print(f"Wrote {output_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("calibration_json", nargs="?", default=None,
                        help="Path to calibration JSON (LeRobot format). "
                             "Optional if --robot-id is given.")
    parser.add_argument("--robot-id", default=None,
                        help="Robot identifier (e.g. arm-001). Resolves to "
                             "config/lerobots/<robot_id>/<arm>_arm.json")
    parser.add_argument("--arm", default="follower", choices=["follower", "leader"],
                        help="Which arm to process (default: follower)")
    parser.add_argument("--offset", type=int, default=DEFAULT_OFFSET,
                        help=f"Encoder offset (default: {DEFAULT_OFFSET})")
    parser.add_argument("--use-urdf", action="store_true",
                        help="Use URDF vendor limits instead of deriving from calibration "
                             "(for initial alignment when the formula is unverified)")
    parser.add_argument("--servo-defaults", type=Path, default=DEFAULT_SERVO_DEFAULTS_PATH,
                        metavar="PATH",
                        help="Path to servo defaults YAML "
                             f"(default: {DEFAULT_SERVO_DEFAULTS_PATH.name})")
    parser.add_argument("--generate-yaml", type=Path, default=None, metavar="PATH",
                        help="Generate a YAML config at PATH from calibration JSON "
                             "(merges servo-default PID/torque fields). "
                             "If PATH is omitted when --robot-id is set, writes to "
                             "config/lerobots/<robot_id>/<arm>.yaml")
    args = parser.parse_args()

    # Resolve calibration source.
    if args.robot_id and args.calibration_json:
        parser.error("Specify either --robot-id or a calibration_json path, not both.")
    if args.robot_id:
        cal_path = resolve_robot_calibration(args.robot_id, arm=args.arm)
        source_desc = f"--robot-id {args.robot_id} --arm {args.arm}"
    elif args.calibration_json:
        cal_path = Path(args.calibration_json)
        source_desc = str(cal_path)
    else:
        parser.error("Provide either --robot-id or a calibration_json path.")
        return 1  # unreachable

    calibration = load_calibration(cal_path)
    derived = derive_limits(calibration, offset=args.offset)

    print("=== Calibration-derived joint limits (radians) ===\n")
    print_comparison(derived)

    if args.use_urdf:
        print("\n--use-urdf: using URDF vendor limits for MJCF patch")
        print_mjcf_patch(URDF_LIMITS)
    else:
        print_mjcf_patch(derived)

    if args.generate_yaml is not None:
        yaml_path = args.generate_yaml
        servo_defaults = load_servo_defaults(args.servo_defaults)
        print()
        generate_yaml(calibration, servo_defaults, yaml_path, source_desc, args.servo_defaults)
    elif args.robot_id:
        # Hint: user can generate YAML with --generate-yaml
        robot_yaml = resolve_robot_calibration(args.robot_id, arm=args.arm).parent / f"{args.arm}.yaml"
        print(f"\nTo generate {args.arm}.yaml for this robot, add:")
        print(f"  --generate-yaml {robot_yaml}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

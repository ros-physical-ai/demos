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


ENCODER_RESOLUTION = 4096
DEFAULT_OFFSET = 2048  # STS3215 centre position

# Model-default parameters merged into generated YAML.
# These are not per-robot — they apply to all SO-ARM101 units.
MODEL_DEFAULTS = {
    "shoulder_pan_joint":  {"p_coefficient": 8,  "i_coefficient": 0, "d_coefficient": 32,
                            "return_delay_time": 0, "acceleration": 254},
    "shoulder_lift_joint": {"p_coefficient": 16, "i_coefficient": 0, "d_coefficient": 32,
                            "return_delay_time": 0, "acceleration": 254},
    "elbow_flex_joint":    {"p_coefficient": 16, "i_coefficient": 0, "d_coefficient": 32,
                            "return_delay_time": 0, "acceleration": 254},
    "wrist_flex_joint":    {"p_coefficient": 16, "i_coefficient": 0, "d_coefficient": 32,
                            "return_delay_time": 0, "acceleration": 254},
    "wrist_roll_joint":    {"p_coefficient": 16, "i_coefficient": 0, "d_coefficient": 32,
                            "return_delay_time": 0, "acceleration": 254},
    "gripper_joint":       {"p_coefficient": 16, "i_coefficient": 0, "d_coefficient": 32,
                            "return_delay_time": 0, "acceleration": 254,
                            "torque_limit": 680, "overload_torque": 680},
}


def encoder_to_rad(raw: int, offset: int = DEFAULT_OFFSET) -> float:
    """Convert a raw encoder value to radians."""
    return (raw - offset) * 2.0 * math.pi / ENCODER_RESOLUTION


def load_calibration(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)


# URDF reference limits (vendor xacro: so_arm101_macro.xacro).
# Used for comparison / validation, not as source of truth.
URDF_LIMITS = {
    "shoulder_pan_joint":  (-1.91986, 1.91986),
    "shoulder_lift_joint": (-1.74533, 1.74533),
    "elbow_flex_joint":    (-1.69,    1.54),
    "wrist_flex_joint":    (-1.60,    1.60),
    "wrist_roll_joint":    (-2.30,    2.30),
    "gripper_joint":       ( 0.00,    1.00),
}


def derive_limits(calibration: dict, offset: int = DEFAULT_OFFSET) -> dict:
    """Return {joint_name: (lower_rad, upper_rad)} from calibration data."""
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
        u_lo, u_hi = URDF_LIMITS.get(name, (float("nan"), float("nan")))
        print(f"{name:<25} {d_lo:>14.5f} {d_hi:>14.5f}"
              f"  {u_lo:>11.5f} {u_hi:>11.5f}"
              f"  {d_lo - u_lo:>+8.4f} {d_hi - u_hi:>+8.4f}")


def print_mjcf_patch(limits: dict) -> None:
    """Print MJCF joint range and actuator ctrlrange values."""
    print("\n--- MJCF joint range values ---")
    for name, (lo, hi) in limits.items():
        print(f'  range="{lo:.5f} {hi:.5f}"   <!-- {name} -->')
    print("\n--- MJCF actuator ctrlrange values ---")
    for name, (lo, hi) in limits.items():
        print(f'  ctrlrange="{lo:.5f} {hi:.5f}"   <!-- {name} -->')


def generate_yaml(calibration: dict, output_path: Path) -> None:
    """Generate a follower.yaml for the feetech_ros2_driver.

    Per-robot calibration fields (id, homing_offset, range_min, range_max) come
    from the calibration JSON.  Model-default fields (PID, torque) are merged in.
    """
    lines = [
        "# Generated from LeRobot calibration JSON — do not edit by hand.",
        "# Re-generate with: pixi run python3 pai_bringup/scripts/generate_limits.py \\",
        f"#     {calibration_path_placeholder()} --generate-yaml {output_path}",
        "joints:",
    ]
    for name, params in calibration.items():
        defaults = MODEL_DEFAULTS.get(name, {})
        lines.append(f"  {name}:")
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


def calibration_path_placeholder() -> str:
    return "<calibration.json>"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("calibration_json",
                        help="Path to follower_arm.json (LeRobot calibration)")
    parser.add_argument("--offset", type=int, default=DEFAULT_OFFSET,
                        help=f"Encoder offset (default: {DEFAULT_OFFSET})")
    parser.add_argument("--use-urdf", action="store_true",
                        help="Use URDF vendor limits instead of deriving from calibration "
                             "(for initial alignment when the formula is unverified)")
    parser.add_argument("--generate-yaml", type=Path, default=None, metavar="PATH",
                        help="Generate a follower.yaml at PATH from calibration JSON "
                             "(merges model-default PID/torque fields)")
    args = parser.parse_args()

    calibration = load_calibration(Path(args.calibration_json))
    derived = derive_limits(calibration, offset=args.offset)

    print("=== Calibration-derived joint limits (radians) ===\n")
    print_comparison(derived)

    if args.use_urdf:
        print("\n--use-urdf: using URDF vendor limits for MJCF patch")
        print_mjcf_patch(URDF_LIMITS)
    else:
        print_mjcf_patch(derived)

    if args.generate_yaml:
        print()
        generate_yaml(calibration, args.generate_yaml)

    return 0


if __name__ == "__main__":
    sys.exit(main())

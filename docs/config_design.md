# Configuration Design

## Scenarios

**New arm bring-up.**
A developer calibrates a new unit with `lerobot-calibrate`, copies the JSON into `config/lerobots/<robot_id>/`, generates the YAML, and launches. No knowledge of PID gains or torque limits is required.

**Servo tuning across units.**
A change to torque protection or PID gains applies to all robots by editing a single field in `config/servo_defaults/so_arm101.yaml` and re-generating.

**Multi-developer workflow.**
Each developer's robot has its own directory under `config/lerobots/<robot_id>/`. Per-unit calibration files are gitignored, preventing conflicts between different physical units.

These scenarios drive the separation of config into three layers, each with a different owner and change cadence:

| Layer | Example fields | Changes when... | Owner |
|-------|---------------|-----------------|-------|
| Robot calibration | `homing_offset`, `range_min`, `range_max` | Recalibration | Developer running `lerobot-calibrate` |
| Servo defaults | PID gains, `acceleration`, torque protection | Arm design revision or app tuning | Arm designer |
| Generated output | All of the above, merged | Either input changes | `generate_limits.py` (never hand-edited) |

## Motivation: consolidated joint limits

Joint limits appear in URDF, MJCF, calibration JSON, generated YAML, and training dataset metadata. When these diverge, failures are silent:

- **Training:** A policy trained against wrong sim limits learns to command positions that hardware clips, or never explores the full range.
- **Inference:** Mismatched limits cause controllers or drivers to clamp commands, producing motion that doesn't match the policy's intent.
- **Hardware:** Wrong `range_min`/`range_max` or `homing_offset` in servo EEPROM causes overtravel or shifts the zero reference.

The layered config design ensures all consumers derive limits from a single per-robot calibration source, eliminating hand-maintained copies that drift independently.

## Design: layered config with merge-at-build

We separate the three concerns into distinct files, then merge them into a single generated artifact for the driver.

```
config/
├── lerobots/                          # Per-robot calibration (LeRobot native)
│   ├── nominal/                       # Default / CI (tracked in git)
│   │   ├── follower_arm.json
│   │   └── leader_arm.json
│   └── arm-001/                       # Developer X's robot (gitignored)
│       ├── follower_arm.json
│       ├── leader_arm.json
│       └── follower.yaml              # GENERATED — never hand-edit
│
├── servo_defaults/                    # Shared across all units
│   └── so_arm101.yaml                 # PID, acceleration, torque protection
└── ...
```

### Layer 1 — Robot calibration (`lerobots/<robot_id>/follower_arm.json`)

Written by LeRobot's calibration tool. Contains only per-unit data: servo IDs, homing offsets, encoder range limits. Uses LeRobot's native bare joint names (`shoulder_pan`, not `shoulder_pan_joint`).

Each physical robot gets its own directory. `nominal/` is the checked-in default for CI and developers without hardware. Per-robot directories (e.g., `arm-001/`) are gitignored.

### Layer 2 — Servo defaults (`servo_defaults/so_arm101.yaml`)

Shared across all SO-ARM101 units. Contains STS3215 servo parameters written to EEPROM by the feetech_ros2_driver: PID gains, acceleration, torque protection.

Structure uses `all_joints` (applied to every joint) + per-joint overrides:

```yaml
all_joints:
  i_coefficient: 0
  d_coefficient: 32
  return_delay_time: 0
  acceleration: 254

joints:
  shoulder_pan:
    p_coefficient: 8      # lower gain for base joint (higher inertia)
  gripper:
    p_coefficient: 16
    max_torque_limit: 500  # gripper protection
```

To change gains or protection for ALL robots, edit this file. One place, one review.

### Layer 3 — Generated output (`lerobots/<robot_id>/follower.yaml`)

Produced by `generate_limits.py`. Merges Layer 1 (per-robot calibration) + Layer 2 (servo defaults) into the format expected by the feetech_ros2_driver. Maps bare joint names to ROS names (`_joint` suffix).

The header shows provenance:

```yaml
# Generated — do not edit by hand.
# Sources:
#   calibration: --robot-id arm-001
#   servo defaults: so_arm101.yaml
```

## Developer mental model

- **Calibrated your robot?** Copy the JSON into `config/lerobots/<robot_id>/`. Done.
- **Tuning servo gains?** Edit `config/servo_defaults/so_arm101.yaml`. Affects all robots.
- **Never edit the generated YAML.** Re-run `generate_limits.py` instead.

## Generation flow

```
                    ┌───────────────────────────────┐
                    │  lerobot-calibrate            │
                    │  (writes to ~/.cache/...)     │
                    └──────────────┬────────────────┘
                                   │ copy
                                   ▼
┌──────────────────────────────────────────────────────┐
│  config/lerobots/<robot_id>/follower_arm.json        │  Layer 1: per-robot
│  config/lerobots/<robot_id>/leader_arm.json          │  (one per arm)
└──────────────────────────┬───────────────────────────┘
                           │
                           │  generate_limits.py --robot-id <id> --arm <arm> --generate-yaml
                           │
┌──────────────────────────┴───────────────────────────┐
│  config/servo_defaults/so_arm101.yaml                │  Layer 2: shared
└──────────────────────────┬───────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────┐
│  config/lerobots/<robot_id>/follower.yaml            │  Layer 3: generated
│  config/lerobots/<robot_id>/leader.yaml              │  (one per arm)
└──────────────────────────┬───────────────────────────┘
                           │
              ┌────────────┴────────────┐
              ▼                         ▼
     feetech_ros2_driver         MuJoCo / Gazebo
     (joint_config_file)         (joint limits via
                                  launch robot_id)
```

## Launch integration

All launch files accept a `robot_id` argument (default: `nominal`):

```bash
# Real hardware — auto-resolves follower.yaml from robot_id
ros2 launch pai_bringup so_arm_real_bringup.launch.py robot_id:=arm-001

# MuJoCo — derives joint limits from calibration JSON, passes to MJCF xacro
ros2 launch pai_bringup so_arm_mujoco_bringup.launch.py robot_id:=arm-001

# Gazebo — robot_id accepted but URDF limit parameterization is future work
ros2 launch pai_bringup so_arm_gz_bringup.launch.py robot_id:=arm-001

# Override with a specific joint_config_file (bypasses robot_id resolution)
ros2 launch pai_bringup so_arm_real_bringup.launch.py \
    joint_config_file:=/absolute/path/to/follower.yaml
```

## Open questions

### Gazebo per-robot joint limits

MuJoCo joint limits are parameterized via xacro args in `mjcf/so_arm101.xml.xacro` — the launch file loads the calibration JSON, derives radian limits, and passes them through. Gazebo does not yet support per-robot limits. The `robot_id` arg is accepted but has no effect on joint limits.

Two approaches are under consideration:

**Option A — Parameterize the vendor URDF xacro**

Add 12 xacro args (6 joints x lower/upper) to `external/SO-ARM100/.../so_arm101_macro.xacro` with defaults matching current hardcoded values. The Gazebo launch file computes per-robot limits and passes them as xacro args, same pattern as MuJoCo.

| Pros | Cons |
|------|------|
| Gazebo physics joint stops match per-robot limits exactly | Modifies external/vendor code; needs rebasing on upstream updates |
| Correct for sim-to-real policy training in Gazebo | 12 additional xacro args in vendor macro |
| Consistent with MuJoCo approach | |

**Option B — `joint_limits.yaml` overlay at the controller layer**

Use ros2_control's `joint_limits` interface to load per-robot limit overrides. Controllers clamp commands before they reach Gazebo. The URDF physics joint stops remain at vendor defaults as a secondary backstop.

| Pros | Cons |
|------|------|
| No vendor code modification | Gazebo physics stops don't match per-robot limits — sim joint can travel beyond (or be tighter than) the real robot's range |
| Clean separation — limits stay in the ROS control layer | If training policies in Gazebo, the sim-real limit mismatch could affect transfer |
| Works with existing ros2_control infrastructure | Two limit enforcement layers with different values (URDF stops vs controller clamp) |

**Recommendation depends on use case:** If Gazebo will be used for policy training, Option A is necessary to avoid sim-real limit mismatch. If Gazebo is only used for software validation, Option B is simpler and avoids patching vendor code.

Feedback welcome — please comment on this section or open an issue.

---

## Why this pattern

This is the standard **layered configuration** pattern used in:

- **Kubernetes**: base manifests + per-env overlays, merged by kustomize
- **UR/Fanuc/KUKA**: factory calibration file + controller config, merged at boot
- **Embedded firmware**: board support package + app config, linked at build

The key principle: **each file has exactly one reason to change**. Calibration changes when you recalibrate. Servo tuning changes when you revise the arm design. The generated output changes whenever either input changes — and nothing else.

# SO-ARM101 Calibration Guide

## Overview

Before using the SO-ARM101 with the ROS 2 stack, each robot must be calibrated using **LeRobot's calibration procedure**. This step writes a `homing_offset` to the EEPROM of each Feetech servo, ensuring that the encoder mid-point (tick 2048) corresponds to the physical mid-range of every joint.

The ROS 2 `ros2_control` configuration ([so_arm101.ros2_control.xacro](../pai_bringup/config/control/so_arm101.ros2_control.xacro)) sets a software `offset` of **2048** for every joint — the exact center of the 0–4095 encoder range. Combined with the per-servo `homing_offset` written during calibration, this means that sending **0 rad** to any joint controller moves it to its calibrated middle position.

## Prerequisites

- SO-ARM101 follower arm (and optionally a leader arm) connected via USB.
- LeRobot installed (`pip install lerobot` or via the pixi environment).
- Identify the serial ports for each arm (e.g. `/dev/ttyACM0`, `/dev/ttyACM1`).

## Step 1 — Run LeRobot Calibration

Follow the official LeRobot SO-101 calibration instructions:
**<https://huggingface.co/docs/lerobot/en/so101#calibrate>**

### Follower arm

```bash
lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower_arm
```

### Leader arm (if using teleoperation)

```bash
lerobot-calibrate \
    --robot.type=so101_leader \
    --robot.port=/dev/ttyACM1 \
    --robot.id=leader_arm
```

> [!IMPORTANT] Important: — getting the mid-range position right:**
> During calibration you will be prompted with: *"Move follower_arm SOFollower to the middle of its range of motion and press ENTER...."*. Take extra care at this step: the calibration routine computes `homing_offset = encoder_position − 2047` and writes it to the servo's EEPROM. If a joint is not truly centered when you confirm, every subsequent position command will be shifted by that error.
>
> **Gripper exception:** Leave the **gripper fully closed** instead of centering it. This aligns the zero-point between LeRobot and ROS 2 so that "zero" means closed in both stacks and positive values open the gripper (see [Known Limitations](#gripper-normalization-differs-between-lerobot-and-ros-2) for details).

## Step 2 — Verify calibration files

After calibration, LeRobot writes a JSON file per arm under:

```
~/.cache/huggingface/lerobot/calibration/robots/so101_follower/follower_arm.json
~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/leader_arm.json
```

Open the file and verify each joint has sensible values. Example (follower):

```json
{
    "shoulder_pan_joint": {
        "id": 1,
        "drive_mode": 0,
        "homing_offset": -1480,
        "range_min": 748,
        "range_max": 3410
    },
    ...
}
```

- `homing_offset` — should be in the range of roughly ±2048; large outliers may indicate the motor was not centered correctly.
- `range_min` / `range_max` — the observed physical limits for the joint, in encoder ticks.


## How It Works

### Servo firmware (hardware level)

During calibration, LeRobot writes a `homing_offset` to each servo's EEPROM (register 31). The servo firmware automatically applies it on every position read:

```
Present_Position = Raw_Encoder − Homing_Offset
```

This makes tick **2047** correspond to the physical center of the joint.

### ros2_control xacro (software level)

Every joint in the [ros2_control xacro](../pai_bringup/config/control/so_arm101.ros2_control.xacro) has:

```xml
<param name="offset">2048</param>
```

The `feetech_ros2_driver` uses this software offset when converting between radians and encoder ticks:

```
read :  θ = (2π / 4096) × (Present_Position − offset)
write:  ticks = (θ × 4096 / 2π) + offset
```

Since the servo firmware has already centered `Present_Position` around 2047 via the homing offset, and the software offset is 2048 (≈ 2047), sending **θ = 0** results in the joint moving to its calibrated middle position.

### Summary

| Layer | What it does | Value |
|-------|-------------|-------|
| Servo EEPROM (`homing_offset`) | Per-robot correction so tick 2047 = physical center | Set by `lerobot.calibrate` |
| Xacro software `offset` | Nominal center of the 12-bit encoder range | 2048 (fixed) |

The two layers work together: the EEPROM homing offset handles per-robot mechanical variation, and the fixed software offset maps the centered tick value to 0 radians.

## Known Limitations

### Gripper normalization differs between LeRobot and ROS 2

The two stacks use **different conventions** for the gripper joint:

| Stack | Gripper mode | "Zero" means | Units |
|-------|-------------|--------------|-------|
| **LeRobot** | `RANGE_0_100` (hardcoded) | `range_min` — one physical limit | 0–100 % |
| **ROS 2** (`feetech_ros2_driver`) | Same as all joints | Calibrated mid-range (encoder 2048) | radians |

In LeRobot's `so_follower.py`, the gripper is always created with `MotorNormMode.RANGE_0_100`(See [here](https://github.com/huggingface/lerobot/blob/0b067df57d21d3a02d6c511f1609172fa39ac29b/src/lerobot/robots/so_follower/so_follower.py#L60)) regardless of the `use_degrees` setting. This maps 0 → `range_min` (fully closed) and 100 → `range_max` (fully open), treating the gripper as a percentage-open actuator. Body joints, on the other hand, use `RANGE_M100_100` (or `DEGREES`), where 0 maps to the midpoint of the range.

The ROS 2 `feetech_ros2_driver` makes **no distinction** between the gripper and other joints — all positions are in radians with a software offset of 2048. Sending 0 rad to the gripper moves it to its calibrated center.

**Practical impact:** When sending a "zero" action through LeRobot, the gripper moves to a physical limit instead of the center. To command the gripper to its midpoint in LeRobot, send **50** (not 0). In ROS 2, send **0 rad** as with any other joint.

**Workaround — calibrate with the gripper closed:**
During the calibration step where you are asked to move joints to their mid-range, leave the **gripper fully closed** instead of centering it. This way the `homing_offset` is computed at the closed position, and both stacks will agree that their respective "zero" corresponds to a closed gripper:

- **LeRobot:** 0 (in `RANGE_0_100`) → `range_min` → closed.
- **ROS 2:** 0 rad → encoder 2048 → homing-offset-adjusted closed position.

Positive values then open the gripper in both stacks. This is an approximation — the actual numeric ranges still differ (0–100 % vs. radians), so the values are not directly interchangeable — but it ensures that the zero-point and direction of motion are consistent between the two systems.

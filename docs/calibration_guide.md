# SO-ARM101 Calibration Guide

## Overview

Before using the SO-ARM101 with the ROS 2 stack, each robot must be calibrated using **LeRobot's calibration procedure**. This step writes a `homing_offset` to the EEPROM of each Feetech servo so that the servo reports the desired physical center at the STS midpoint tick value (**2048**).

In the current `feetech_ros2_driver`, positions are converted relative to the fixed STS midpoint (**2048**), while `homing_offset` in the motor EEPROM provides the per-robot centering. In practice, this means that after calibration, sending **0 rad** to a joint controller moves the joint to its calibrated middle position.

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

> [!IMPORTANT]
> Getting the reference pose right:
> During calibration you will be prompted with: *"Move follower_arm SOFollower to the middle of its range of motion and press ENTER...."*. Take extra care at this step: the calibration routine writes a `homing_offset` to the servo's EEPROM so that the pose you confirm becomes the joint's zero/mid reference used later by ROS 2.
>
> **Gripper exception:** Leave the **gripper fully closed** instead of centering it. This aligns the zero-point between LeRobot and ROS 2 so that "zero" means closed in both stacks and positive values open the gripper (see [Known Limitations](#gripper-normalization-differs-between-lerobot-and-ros-2) for details).

## Step 2 — Optional: use a ROS 2 joint config file

After LeRobot calibration, the arm is calibrated so that the joint zero reference matches the centered position expected by the Feetech-based ROS 2 setup. In the common case, no extra ROS 2 calibration file is required.

LeRobot writes a JSON file per arm under:

```
~/.cache/huggingface/lerobot/calibration/robots/so101_follower/follower_arm.json
~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/leader_arm.json
```

If you want to make those values explicit in this project, copy the relevant fields into a `joint_config_file`, for example:

- [`../pai_bringup/config/hardware/my_follower.yaml`](../pai_bringup/config/hardware/my_follower.yaml)
- [`../pai_bringup/config/hardware/my_leader.yaml`](../pai_bringup/config/hardware/my_leader.yaml)

Use a `joint_config_file` only if you want to keep a versioned per-robot configuration in the repo, override existing motor settings, or set additional driver parameters.

### Parameter precedence

**Precedence:** `joint_config_file` overrides `URDF/Xacro` defaults.

On initialization, the driver writes the resulting values for those parameters to the motor EEPROM, replacing the older stored values.

If you use a `joint_config_file`, each joint must:

- include the correct `id`
- match a joint defined in the xacro/ros2_control description

Common parameters:

- `id`: motor ID on the bus
- `homing_offset`: joint zero alignment, written to the servo EEPROM
- `range_min` / `range_max`: joint travel limits, written to the servo EEPROM
- `p_coefficient` / `i_coefficient` / `d_coefficient`, `return_delay_time`, `max_torque_limit`, `protection_current`, `overload_torque`: optional tuning and protection settings written to the servo EEPROM
- `acceleration`: optional motion parameter written by the driver, but not persistently to EEPROM

For the **follower gripper**, this project sets these protection values by default in [`../pai_bringup/config/control/so_arm101.ros2_control.xacro`](../pai_bringup/config/control/so_arm101.ros2_control.xacro) to reduce the risk of overloading or damaging the motor:

- `max_torque_limit: 500`
- `protection_current: 250`
- `overload_torque: 25`

LeRobot calibration does not produce these safety values in its calibration output. If needed, you can still override them per robot through `joint_config_file`.

For full parameter details and memory behavior, see the [`feetech_ros2_driver` user guide](https://github.com/legalaspro/feetech_ros2_driver/blob/feat/joint-config-and-calibration/doc/user.md#ros2_control-urdf-tag).


## How It Works

### Servo firmware (hardware level)

During calibration, LeRobot writes a `homing_offset` to each servo's EEPROM (register 31). The servo firmware applies that offset internally when reporting position:

```
Present_Position = Raw_Encoder − Homing_Offset
```

With correct calibration, this makes the desired physical center of the joint appear at the midpoint used by the ROS 2 driver (**2048**).

### `feetech_ros2_driver` (ROS 2 hardware interface)

The driver converts between radians and encoder ticks relative to the fixed STS midpoint (**2048**):

```
read :  θ = (2π / 4096) × (Present_Position − 2048)
write:  ticks = (θ × 4096 / 2π) + 2048
```

Because the servo firmware already applies `homing_offset` in hardware, a correctly calibrated joint reports its physical center at tick **2048**. As a result, sending **θ = 0** through ROS 2 moves the joint to its calibrated middle position.

If a `joint_config_file` is provided, the driver merges those parameters over the URDF joint parameters and writes any provided values such as `homing_offset`, `range_min`, `range_max`, and tuning/protection settings to the motor EEPROM during initialization.

### Summary

| Layer | What it does | Value |
|-------|-------------|-------|
| Servo EEPROM (`homing_offset`) | Per-robot correction so the desired physical center is reported at the midpoint tick | Set by `lerobot.calibrate` or by the driver if provided in URDF/YAML |
| `feetech_ros2_driver` midpoint | Fixed tick reference used for radian/tick conversion | 2048 |

The two layers work together: the EEPROM `homing_offset` handles per-robot mechanical variation in hardware, and the driver always maps the centered midpoint tick value to **0 rad**.

## Known Limitations

### Gripper normalization differs between LeRobot and ROS 2

The two stacks use **different conventions** for the gripper joint:

| Stack | Gripper mode | "Zero" means | Units |
|-------|-------------|--------------|-------|
| **LeRobot** | `RANGE_0_100` (hardcoded) | `range_min` — one physical limit | 0–100 % |
| **ROS 2** (`feetech_ros2_driver`) | Same as all joints | Calibrated mid-range (fixed midpoint tick 2048) | radians |

In LeRobot's `so_follower.py`, the gripper is always created with `MotorNormMode.RANGE_0_100`(See [here](https://github.com/huggingface/lerobot/blob/0b067df57d21d3a02d6c511f1609172fa39ac29b/src/lerobot/robots/so_follower/so_follower.py#L60)) regardless of the `use_degrees` setting. This maps 0 → `range_min` (fully closed) and 100 → `range_max` (fully open), treating the gripper as a percentage-open actuator. Body joints, on the other hand, use `RANGE_M100_100` (or `DEGREES`), where 0 maps to the midpoint of the range.

The ROS 2 `feetech_ros2_driver` makes **no distinction** between the gripper and other joints — all positions are in radians and are converted relative to the fixed midpoint tick value **2048**. Sending 0 rad to the gripper moves it to its calibrated center.

**Practical impact:** When sending a "zero" action through LeRobot, the gripper moves to a physical limit instead of the center. To command the gripper to its midpoint in LeRobot, send **50** (not 0). In ROS 2, send **0 rad** as with any other joint.

**Workaround — calibrate with the gripper closed:**
During the calibration step where you are asked to move joints to their mid-range, leave the **gripper fully closed** instead of centering it. This way the `homing_offset` is computed at the closed position, and both stacks will agree that their respective "zero" corresponds to a closed gripper:

- **LeRobot:** 0 (in `RANGE_0_100`) → `range_min` → closed.
- **ROS 2:** 0 rad → encoder 2048 → homing-offset-adjusted closed position.

Positive values then open the gripper in both stacks. This is an approximation — the actual numeric ranges still differ (0–100 % vs. radians), so the values are not directly interchangeable — but it ensures that the zero-point and direction of motion are consistent between the two systems.

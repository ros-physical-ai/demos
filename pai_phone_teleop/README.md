# pai_phone_teleop

Phone-based 6-DoF pose teleoperation **adapter** for the [`pai_teleop_ik`](../pai_teleop_ik) servo.

This package embeds the upstream [`teleop`](https://github.com/SpesRobotics/teleop)
WebXR bridge and translates the phone's 6-DoF pose into a Cartesian target for
the IK servo. It is a thin frontend: it owns no IK or kinematics.

> [!IMPORTANT]
> Your phone has to support the [WebXR API](https://developer.mozilla.org/en-US/docs/Web/API/WebXR_Device_API). Unfortunately, iPhone doesn't support the WebXR API.

## How it works

```
 phone (WebXR) ──target_pose (PoseStamped)──► ik_servo_node ──► forward_position_controller ──► sim/hardware
  (this adapter)──gripper_command (Float64)──►  (Pink IK)     ◄── /joint_states
                ◄──ee_pose (PoseStamped)────────┘
```

The adapter seeds the WebXR bridge with the servo's `ee_pose` so the phone's
deltas are relative to where the arm is, publishes the phone pose to
`target_pose` (in `root_frame`) while "move" is held, and publishes the gripper
setpoint on `gripper_command`. The Pink solver, Pinocchio model, frame handling,
and joint streaming all live in `pai_teleop_ik` — see its
[README](../pai_teleop_ik/README.md) for the IK parameters and topic contract.

## How to run

1. Launch any `pai_bringup` bringup (real / mujoco / gz), e.g.:

   ```bash
   pixi run so-arm-gz
   ```

2. Launch the phone teleop (this also starts the IK servo):

   ```bash
   pixi run so-arm-phone-ik
   ```

3. Open the HTTPS URL logged to stdout (default `https://<host>:4443/`) on a
   phone with a WebXR-capable browser (e.g. Chrome on Android).

4. Press `START` to begin, then hold `HOLD TO MOVE` while moving the phone.

> [!NOTE]
> Recommendation: Keep the phone vertical and steady, then press `START`.

> [!IMPORTANT]
> When you press start, XR localization uses the orientation registered at
> initialization. Pressing START in a weird orientation makes it tricky to
> teleoperate correctly.

## Controls

| Control                     | Action                                                                           |
| --------------------------- | -------------------------------------------------------------------------------- |
| **Move** button (hold)      | Streams phone pose; arm tracks it via the servo's differential IK                |
| **A** button (hold)         | Gripper opens slowly                                                             |
| **B** button (hold)         | Gripper closes slowly                                                            |
| **Gripper** button (toggle) | Engage: locks gripper at `gripper_min` (apply pressure). Disengage: releases A/B |

The gripper engage button takes priority over A/B — A and B are ignored while
the gripper is engaged.

## Topics

| Topic                      | Direction  | Type                        | Notes                                                 |
| -------------------------- | ---------- | --------------------------- | ----------------------------------------------------- |
| `ik_servo/ee_pose`         | Subscribes | `geometry_msgs/PoseStamped` | Servo's commanded tool pose; seeds the teleop bridge. |
| `ik_servo/target_pose`     | Publishes  | `geometry_msgs/PoseStamped` | Phone-commanded EE target in `root_frame`.            |
| `ik_servo/gripper_command` | Publishes  | `std_msgs/Float64`          | Gripper setpoint (rad).                               |
| `teleop_target`            | Publishes  | `geometry_msgs/PoseStamped` | Phone target, for RViz visualization.                 |
| `/tf`                      | Publishes  | `tf2_msgs/TFMessage`        | `root_frame → teleop_target` transform for RViz.      |

## Key parameters

These are the adapter's parameters. IK parameters live on the servo — see the
[`pai_teleop_ik` README](../pai_teleop_ik/README.md).

| Parameter               | Default                    | Description                                             |
| ----------------------- | -------------------------- | ------------------------------------------------------- |
| `host`                  | `0.0.0.0`                  | Bind address for the WebXR server.                      |
| `port`                  | `4443`                     | Port for the WebXR server.                              |
| `root_frame`            | `base_link`                | Reference frame for the phone target; must match servo. |
| `control_rate`          | `50.0`                     | Teleop seeding + gripper publish rate (Hz).             |
| `gripper_min`           | `0.0`                      | Gripper "closed" / engage-lock position (rad).          |
| `gripper_max`           | `1.7`                      | Gripper "open" position (rad).                          |
| `gripper_open_speed`    | `5.0`                      | Gripper ramping speed while A/B is held (rad/s).        |
| `target_pose_topic`     | `ik_servo/target_pose`     | Topic the phone pose is published on.                   |
| `gripper_command_topic` | `ik_servo/gripper_command` | Topic the gripper setpoint is published on.             |
| `ee_pose_topic`         | `ik_servo/ee_pose`         | Servo tool-pose topic used to seed the bridge.          |

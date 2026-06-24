# pai_rviz_teleop

Interactive marker teleoperation **adapter** for the [`pai_teleop_ik`](../pai_teleop_ik) servo.

Drag a 6-DOF [interactive marker](https://wiki.ros.org/interactive_markers) in
RViz and the arm's tool frame follows it in real time. This package is a thin
frontend: it owns no IK or kinematics. It publishes the marker pose as a
Cartesian target and lets the servo do the differential IK and joint command
streaming.

## How it works

```
 RViz interactive marker ──target_pose (PoseStamped)──► ik_servo_node ──► forward_position_controller ──► sim/hardware
   (this adapter)         ──gripper_command (Float64)──►  (Pink IK)     ◄── /joint_states
                          ◄──ee_pose (PoseStamped)────────┘
```

1. **Spawn** — the adapter subscribes to the servo's `ee_pose` (the current
   commanded tool pose). On the first message it spawns a 6-DOF interactive
   marker at that pose, so the marker starts exactly where the arm is.
2. **Drag** — every time the marker moves past `marker_deadband`, the adapter
   publishes its pose to `target_pose` (a `PoseStamped` in `root_frame`). The
   servo resolves the frame, runs differential IK, and streams joint commands.
3. **Gripper** — a right-click menu toggles the gripper; an animation timer
   ramps a setpoint between open/closed and publishes it on `gripper_command`
   (`std_msgs/Float64`).

The Pink solver, Pinocchio model, frame handling, and joint command streaming
all live in `pai_teleop_ik`. See its [README](../pai_teleop_ik/README.md) for
the IK parameters and the full topic contract.

## Marker menu

Right- or left-click the marker sphere for a context menu:

- **Toggle gripper (open/close)** — flips the gripper setpoint and animates
  toward it at `gripper_speed` rad/s.
- **Reset to current end-effector pose** — snaps the marker back to the latest
  `ee_pose` from the servo.

## Running

The Zenoh router must be running first (see the repository README). Then launch
your hardware/sim bringup and the SO-ARM101 demo (which also starts the IK servo):

```bash
pixi run start_zenoh        # terminal 1
pixi run so-arm-mujoco      # terminal 2 (MuJoCo + controllers; or another bringup)
pixi run so-arm-rviz-ik     # terminal 3 (IK servo + marker adapter)
```

`pixi run so-arm-rviz-ik` launches `pai_bringup`'s `so_arm_rviz_ik.launch.py`,
which wraps the generic launch below with the SO-ARM101 config.

### Generic launch and configuration

This package ships a **generic, config-driven** launch and a template config
([config/interactive_ik.yaml](config/interactive_ik.yaml)) with placeholder
joint names. Run it directly with a robot-specific config:

```bash
ros2 launch pai_rviz_teleop interactive_ik.launch.py \
  config_file:=/path/to/your_robot_interactive_ik.yaml
```

The SO-ARM101 config lives in `pai_bringup`
(`config/teleop/so_arm101_interactive_ik.yaml`). A single config file configures
both the `ik_servo` and the marker adapter (the `/**` wildcard applies to both),
so the shared frames and topics stay consistent.

In your RViz, add an `InteractiveMarkers` display on namespace `/interactive_ik`
with fixed frame `world`, then drag the marker. To confirm the arm is tracking
it, watch the joint states in another terminal:

```bash
ros2 topic echo /joint_states
```

## Parameters

The adapter's parameters are set through the launch's `config_file` (see
[config/interactive_ik.yaml](config/interactive_ik.yaml)). IK/servo parameters
live in the same file and are documented in the
[`pai_teleop_ik` README](../pai_teleop_ik/README.md).

| Parameter                 | Default                    | Description                                                          |
| ------------------------- | -------------------------- | -------------------------------------------------------------------- |
| `root_frame`              | `world`                    | Frame the marker is anchored in; must match the servo `root_frame`.  |
| `control_rate`            | `50.0`                     | Gripper animation rate (Hz).                                         |
| `marker_deadband`         | `2e-3`                     | Min marker pose change (m + rad) for a drag to publish a new target. |
| `has_gripper`             | `true`                     | Whether to show the gripper menu and publish `gripper_command`.      |
| `gripper_open_position`   | `1.7`                      | Gripper "open" setpoint (rad).                                       |
| `gripper_closed_position` | `0.0`                      | Gripper "closed" setpoint (rad).                                     |
| `gripper_speed`           | `1.0`                      | Gripper toggle animation speed (rad/s).                              |
| `target_pose_topic`       | `ik_servo/target_pose`     | Topic the marker pose is published on.                               |
| `gripper_command_topic`   | `ik_servo/gripper_command` | Topic the gripper setpoint is published on.                          |
| `ee_pose_topic`           | `ik_servo/ee_pose`         | Servo tool-pose topic the marker spawns/resets from.                 |

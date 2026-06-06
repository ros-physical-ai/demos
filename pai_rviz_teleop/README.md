# pai_rviz_teleop

Interactive marker differential inverse kinematics demo for the SO-ARM101.

Drag a 6-DOF [interactive marker](https://wiki.ros.org/interactive_markers) in RViz and the arm's tool frame (`gripper_frame_link`) follows it in real time.
The motion is solved with the [Pink](https://github.com/stephane-caron/pink) differential IK solver.
The output of the solve is streamed to the `forward_position_controller` (`position_controllers/JointGroupPositionController`) running against the MuJoCo simulation.

## How it works

```
 RViz interactive marker ──pose──► interactive_ik_node ──Float64MultiArray──► forward_position_controller ──► MuJoCo
                                        │  (Pink differential IK)                                                │
                                        └────────────────────── /joint_states ◄──────────────────────────────────┘
```

1. **Model** — the node subscribes to the latched `/robot_description` and hands
   it to a `DifferentialIKSolver`, which builds a Pinocchio model and locks every
   joint except the configured arm joints (by default the SO-ARM101's
   `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`).
   The gripper joint is a side branch that does not affect the tool frame, so it
   is excluded from the IK.
2. **Marker** — once the first `/joint_states` message arrives, a 6-DOF
   interactive marker is spawned at the current tool pose (so it starts exactly
   where the arm is) in the `world` frame.
3. **IK loop** — at `control_rate` Hz, the raw marker pose is first passed
   through an SE3 low-pass filter (a geodesic step using Pinocchio's
   `log6`/`exp6` exponential map) for smoother motion, then handed to the solver.
   The integrated configuration is published as a `Float64MultiArray`
   (the solved arm joints plus the gripper) in the order the `forward_position_controller` expects.

## Marker menu

Right- or left-click the marker sphere for a context menu:

- **Toggle gripper (open/close)** — flips the gripper setpoint and animates the
  gripper toward it at `gripper_speed` rad/s (open/closed positions are clamped
  to the URDF joint limits).
- **Reset to current joint states** — re-seeds the IK and snaps the marker back
  to the arm's current measured tool pose, cancelling any accumulated target
  offset and stopping gripper motion.

## Running

The Zenoh router must be running first (see the repository README).
Then, you can launch your hardware bringup and this RViz IK demo.

```bash
pixi run start_zenoh        # terminal 1
pixi run so-arm-mujoco      # terminal 2 (MuJoCo + controllers) (can switch for another simulator or hardware bringup)
pixi run so-arm-rviz-ik     # terminal 3 (IK node)
```

or directly:

```bash
ros2 launch pai_rviz_teleop interactive_ik_demo.launch.py
```

In your separate RViz, if you don't have it loaded, add an `InteractiveMarkers` display on namespace `/interactive_ik` with fixed frame `world`.
Drag the interactive marker.
To confirm the arm is tracking it, watch the joint states stream in another terminal:

```bash
ros2 topic echo /joint_states
```

## Parameters

| Parameter                 | Default                                 | Description                                                                                                |
| ------------------------- | --------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `arm_joints`              | SO-ARM101 arm joints                    | Ordered joints the IK actuates; all others are locked. Override to drive a different arm.                  |
| `gripper_joint`           | `gripper_joint`                         | Joint driven by the gripper menu toggle (not part of the IK). Empty string for arms without a gripper.     |
| `ee_frame`                | `gripper_frame_link`                    | Tool frame driven toward the marker.                                                                       |
| `root_frame`              | `world`                                 | Model root frame; the marker is anchored here.                                                             |
| `control_rate`            | `50.0`                                  | IK / command streaming rate (Hz).                                                                          |
| `position_cost`           | `0.5`                                   | FrameTask position task weight.                                                                            |
| `orientation_cost`        | `0.1`                                   | FrameTask orientation task weight.                                                                         |
| `posture_cost`            | `1e-3`                                  | PostureTask regularization weight.                                                                         |
| `lm_damping`              | `1e-2`                                  | FrameTask Levenberg-Marquardt damping; raise to reduce near-singular oscillation.                          |
| `max_joint_velocity`      | `2.0`                                   | Per-joint velocity cap (rad/s) enforced as a QP constraint; lower for smoother motion.                     |
| `inactivity_timeout`      | `0.3`                                   | Seconds of marker inactivity after which the arm holds and stops republishing (kills steady-state jitter). |
| `marker_deadband`         | `2e-3`                                  | Min marker pose change (6D log norm) for feedback to count; ignores mouse-down-without-moving.             |
| `qp_solver`               | `quadprog`                              | QP backend used by Pink.                                                                                   |
| `command_topic`           | `/forward_position_controller/commands` | JointGroupPositionController command topic.                                                                |
| `gripper_open_position`   | `1.7`                                   | Gripper "open" setpoint, rad (clamped to URDF limits).                                                     |
| `gripper_closed_position` | `0.0`                                   | Gripper "closed" setpoint, rad (clamped to URDF limits).                                                   |
| `gripper_speed`           | `1.0`                                   | Gripper toggle animation speed (rad/s).                                                                    |
| `target_lowpass_alpha`    | `0.5`                                   | SE3 low-pass per-tick geodesic fraction, `(0, 1]`; `1.0` disables filtering.                               |

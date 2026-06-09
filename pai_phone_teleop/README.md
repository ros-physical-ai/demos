# pai_phone_teleop

Phone-based 6-DoF pose teleoperation for the SO-ARM101.

This package wraps the upstream [`teleop`](https://github.com/SpesRobotics/teleop)
`python3 -m teleop.ros2` WebXR bridge and supplies it with the end-effector's
current pose via the `ee_pose_publisher` node.

## Status

This iteration publishes the EE current pose and runs the phone bridge.
Driving the arm from the phone-published target is **out of scope** and
will be added in a future iteration.

## How to run

1. Launch any `pai_bringup` bringup (real / mujoco / gz), e.g.:

   ```bash
   ros2 launch pai_bringup so_arm_mujoco_bringup.launch.py
   ```

2. Launch the phone teleop:

   ```bash
   ros2 launch pai_phone_teleop phone_teleop.launch.py
   ```

3. Open the URL logged to stdout (default `http://<host>:4443/`) on a
   phone with a WebXR-capable browser (e.g. Chrome on Android). Hold the
   phone in the WebXR "natural orientation" (screen up, camera toward you)
   and press the **Move** button on screen to begin streaming.

## Topics

| Topic | Direction | Type | Notes |
|---|---|---|---|
| `/current_pose` | Publishes | `geometry_msgs/PoseStamped` | EE pose in `base_link` frame, computed from `/joint_states` FK. Consumed by `teleop.ros2`. |
| `/target_frame` | Subscribes (deferred) | `geometry_msgs/PoseStamped` | Published by `teleop.ros2`. Will be consumed in a future iteration to drive the arm. |
| `/tf` | Publishes (`teleop_target`) | `tf2_msgs/TFMessage` | `base_link -> teleop_target`, published by `teleop.ros2` for RViz visualization. |

## Known limitations

- `teleop.ros2` hard-codes the published `target_frame` `frame_id` to
  `"link_base"` (likely a typo in the upstream library). The TF broadcaster
  uses the correct `base_link`. Downstream consumers should rely on the
  TF or remap/override the frame_id.
- The `omit_current_pose` launch argument is declared but not yet wired
  into the `teleop.ros2` command; we always run without the flag, which
  is the documented correct default (use `/current_pose` as the seed).
- Gripper mapping is not supported by upstream `teleop.ros2`. Use
  `teleop.ros2_ik` for that, or wait for the future iteration.

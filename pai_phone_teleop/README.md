# pai_phone_teleop

Phone-based 6-DoF pose teleoperation for the SO-ARM101.

A single ROS 2 node embeds the upstream [`teleop`](https://github.com/SpesRobotics/teleop)
WebXR bridge together with Pinocchio forward kinematics and Pink differential
IK. The arm tracks the phone's 6-DoF pose in real time by streaming joint
positions to the `forward_position_controller`.

> [!IMPORTANT]
> Your phone has to support the [WebXR API](https://developer.mozilla.org/en-US/docs/Web/API/WebXR_Device_API). Unfortunately, iPhone doesn't support the WebXR API.

## How to run

1. Launch any `pai_bringup` bringup (real / mujoco / gz), e.g.:

   ```bash
   pixi run so-arm-gz
   ```

2. Launch the phone teleop:

   ```bash
   pixi run so-arm-phone-ik
   ```

3. Open the HTTPS URL logged to stdout (default `https://<host>:4443/`) on a
   phone with a WebXR-capable browser (e.g. Chrome on Android).

4. Press `START` button for initiating the teleoperation and keep press the `HOLD TO MOVE` button while moving the phone to teleoperate.

> [!NOTE]
> Recommendation: Keep the phone >vertical and steady and then press `START`.

> [!IMPORTANT]
> When you press the start button the XR localization starts using the orientation registered at initialization. If you press START while holding the phone in a weird orientation it might be tricky to start teleoperating correctly as it is easy to get confused.

## Controls

| Control                     | Action                                                                                |
| --------------------------- | ------------------------------------------------------------------------------------- |
| **Move** button (hold)      | Streams phone pose; arm tracks via differential IK                                    |
| **A** button (hold)         | Gripper opens slowly                                                                  |
| **B** button (hold)         | Gripper closes slowly                                                                 |
| **Gripper** button (toggle) | Engage: locks gripper at position 0 (apply pressure). Disengage: releases A/B control |

The gripper engage button takes priority over A/B — A and B are ignored while
the gripper is engaged.

## Topics

| Topic                                   | Direction  | Type                         | Notes                                                          |
| --------------------------------------- | ---------- | ---------------------------- | -------------------------------------------------------------- |
| `/joint_states`                         | Subscribes | `sensor_msgs/JointState`     | Arm and gripper positions used to seed the IK solver.          |
| `/robot_description`                    | Subscribes | `std_msgs/String`            | Latched URDF, used to build the Pinocchio model at startup.    |
| `teleop_target`                         | Publishes  | `geometry_msgs/PoseStamped`  | Phone-commanded EE target in `root_frame`, for RViz.           |
| `/tf`                                   | Publishes  | `tf2_msgs/TFMessage`         | `root_frame → teleop_target` transform for RViz visualization. |
| `/forward_position_controller/commands` | Publishes  | `std_msgs/Float64MultiArray` | Arm + gripper joint positions at `control_rate` Hz.            |

## Key parameters

| Parameter            | Default                                 | Description                                          |
| -------------------- | --------------------------------------- | ---------------------------------------------------- |
| `host`               | `0.0.0.0`                               | Bind address for the WebXR server.                   |
| `port`               | `4443`                                  | Port for the WebXR server.                           |
| `ee_frame`           | `gripper_frame_link`                    | End-effector frame for FK and IK.                    |
| `root_frame`         | `base_link`                             | Reference frame for teleop deltas and visualization. |
| `control_rate`       | `50.0`                                  | IK solve and command publish rate (Hz).              |
| `inactivity_timeout` | `0.3`                                   | Seconds of phone stillness before IK halts.          |
| `gripper_open_speed` | `5.0`                                   | Gripper ramping speed while A/B is held (rad/s).     |
| `command_topic`      | `/forward_position_controller/commands` | Joint command topic.                                 |

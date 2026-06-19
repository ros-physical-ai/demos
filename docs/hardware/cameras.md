# Cameras

The real-robot bringup launches a **wrist camera** and a **static camera** by default using [usb_cam](https://github.com/ros-drivers/usb_cam).
Both publish at 640×480 @ 30 fps:

| Topic                      | Frame                | Device            |
| -------------------------- | -------------------- | ----------------- |
| `/wrist_camera/image_raw`  | `wrist_camera_link`  | `/dev/cam_wrist`  |
| `/static_camera/image_raw` | `static_camera_link` | `/dev/cam_static` |

> [!IMPORTANT]
> [Udev rules](udev-rules.md) must be configured before using cameras on real hardware. Without them, `/dev/cam_wrist` and `/dev/cam_static` will not exist and the camera nodes will fail to start.

Each camera has its own driver-parameter file (`usb_cam_wrist.yaml`, `usb_cam_static.yaml`) so you can tune resolution, framerate, or pixel format independently — useful when the two cameras are different models.

> [!NOTE]
> A default camera-calibration file ([`pai_bringup/config/cameras/default_640x480.yaml`](../../pai_bringup/config/cameras/default_640x480.yaml)) is shipped so that RViz Camera displays work without errors.
> It is **NOT** required for policy training or inference — the policy consumes raw pixel observations and joint-position actions, so camera intrinsics (focal length, principal point, distortion coefficients) never enter the learning or inference pipeline.
> We publish `camera_info` with placeholder intrinsics for standard ROS tooling (e.g. RViz, image_proc), not for LeRobot.
> If you need accurate intrinsics (e.g. for 3D reconstruction), replace the default file with a proper calibration via `ros2 run camera_calibration cameracalibrator` or your tool of preference.

Camera frames are defined in [`pai_bringup/urdf/cameras.xacro`](../../pai_bringup/urdf/cameras.xacro) and published to TF by `robot_state_publisher`.
The wrist camera moves with the gripper; the static camera is fixed relative to `base_link`.

## Disabling cameras

```bash
ros2 launch pai_bringup so_arm_real_bringup.launch.py usb_port:=/dev/so101_follower use_cameras:=false
```

## Overriding the static camera pose

The static camera position and orientation can be overridden at launch time to match your physical mounting:

```bash
ros2 launch pai_bringup so_arm_real_bringup.launch.py \
    usb_port:=/dev/so101_follower \
    cam_static_xyz:="0.0 0.0 0.50" \
    cam_static_rpy:="3.6652 0.0 -1.5708"
```

> [!NOTE]
> In simulation (Gazebo / MuJoCo), the same two cameras are rendered by the simulator and bridged to the same ROS topics.
> Camera TF frames come from the same `cameras.xacro`.

> [!NOTE]
> [gscam](https://github.com/ros-drivers/gscam) (GStreamer) offers better timestamp fidelity but is not available from robostack and must be compiled from source.

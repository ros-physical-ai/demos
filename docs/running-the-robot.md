# Running the Robot

This guide covers launching the SO-ARM101 in simulation (Gazebo or MuJoCo) or on real hardware.

> [!IMPORTANT]
> This project uses `rmw_zenoh` as the ROS 2 middleware. The Zenoh router must be
> running before launching any demo. Start it in a dedicated terminal and leave it
> running for the duration of your session:
>
> ```bash
> pixi run start_zenoh
> ```
>
> Then open a second terminal for the launch commands below. When using a manual
> (non-Pixi) install, see [Installation](installation.md#middleware-rmw_zenoh).

## Gazebo

![](media/so_arm_gz.png)

```bash
ros2 launch pai_bringup so_arm_gz_bringup.launch.py
```

With Pixi:

```bash
pixi run so-arm-gz
```

## MuJoCo

![](media/so_arm_mujoco.png)

```bash
ros2 launch pai_bringup so_arm_mujoco_bringup.launch.py
```

With Pixi:

```bash
pixi run so-arm-mujoco
```

## Real hardware

```bash
ros2 launch pai_bringup so_arm_real_bringup.launch.py usb_port:=/dev/so101_follower
```

With Pixi:

```bash
pixi run so-arm-real usb_port:=/dev/so101_follower
```

Before running on real hardware, complete the one-time hardware setup:

- [Servo calibration](hardware/calibration.md) — required so encoder zero aligns with the expected physical pose.
- [Udev rules](hardware/udev-rules.md) — stable device symlinks for arms and cameras.
- [Cameras](hardware/cameras.md) — wrist/static camera configuration and overrides.

## Leader arm teleoperation

You can use a leader SO-ARM101 to teleoperate the follower arm (sim or real):

```bash
ros2 launch pai_leader_teleop leader_bringup.launch.py usb_port:=/dev/so101_leader
```

With Pixi:

```bash
pixi run so-arm-leader usb_port:=/dev/so101_leader
```

See the [Teleoperation Overview](teleoperation.md) for all available teleoperation methods.

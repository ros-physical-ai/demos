# Udev Rules

Stable device symlinks (`/dev/cam_wrist`, `/dev/cam_static`, and the arm serial ports) prevent arms and cameras from swapping after a reboot.

See [`pai_bringup/config/hardware/99-so-arm101.rules.example`](../../pai_bringup/config/hardware/99-so-arm101.rules.example) for the rules template and setup instructions.

> [!IMPORTANT]
> Udev rules must be configured before using cameras on real hardware. Without them, `/dev/cam_wrist` and `/dev/cam_static` will not exist and the camera nodes will fail to start. See [Cameras](cameras.md).

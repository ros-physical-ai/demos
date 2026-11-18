# Udev Rules

Stable device symlinks (`/dev/so101_leader`, `/dev/so101_follower`, `/dev/cam_wrist`, `/dev/cam_static`) prevent arms and cameras from swapping after a reboot or replug.
The rules template is [`pai_bringup/config/hardware/99-so-arm101.rules.example`](../../pai_bringup/config/hardware/99-so-arm101.rules.example).

## Serial port permissions

The arm serial ports belong to the `dialout` group. Add your user to it, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

## Find device properties

```bash
pixi run list-hw-devices
```

Example output:

```
=== Serial ports (arms) ===
/dev/ttyACM0
  ID_VENDOR_ID=1a86
  ID_MODEL_ID=55d3
  ID_SERIAL_SHORT=5A7C121431

=== Cameras ===
/dev/video2  [video stream]
  name=USB2.0_CAM1: USB2.0_CAM1
  ID_PATH=pci-0000:80:14.0-usb-0:6.1:1.0
/dev/video3  [metadata, not usable as a camera]
  name=USB2.0_CAM1: USB2.0_CAM1
  ID_PATH=pci-0000:80:14.0-usb-0:6.1:1.0

=== Symlinks from udev rules ===
/dev/so101_leader  (missing)
...
```

### Arms

Both arms use the same USB serial adapter, so they report the same `ID_VENDOR_ID` and `ID_MODEL_ID`.
`ID_SERIAL_SHORT` tells them apart. Plug in one arm at a time to see which serial belongs to the leader and which to the follower.

### Cameras

Each USB camera exposes two `/dev/video*` nodes: a video stream and a metadata node. The rules select the video stream with `ATTR{index}=="0"`.
Cameras match by `ID_PATH`, the physical USB port they are plugged into, so keep each camera in the same port. Unplug one camera to see which `ID_PATH` is which.
See [Cameras](cameras.md) for configuration. If you have no cameras yet, comment out the camera lines in the rules file.

> [!IMPORTANT]
> Without camera rules, `/dev/cam_wrist` and `/dev/cam_static` do not exist and the camera nodes fail to start.

## Install the rules

```bash
sudo cp pai_bringup/config/hardware/99-so-arm101.rules.example /etc/udev/rules.d/99-so-arm101.rules
sudoedit /etc/udev/rules.d/99-so-arm101.rules   # replace the <...> placeholders
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify by running `pixi run list-hw-devices` again: the symlinks section shows each link and the device it points to.

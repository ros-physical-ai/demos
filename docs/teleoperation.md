# Teleoperation

The SO-ARM101 follower arm can be teleoperated with several input methods. All of
them publish to the same `/forward_position_controller/commands` topic, so they
work against any backend — Gazebo, MuJoCo, or real hardware.

| Method                      | Input device                          | Guide                                                 |
| --------------------------- | ------------------------------------- | ----------------------------------------------------- |
| **Leader arm**              | A second physical SO-ARM101 (by hand) | [`pai_leader_teleop`](../pai_leader_teleop/README.md) |
| **RViz interactive marker** | 6-DoF marker dragged in RViz          | [`pai_rviz_teleop`](../pai_rviz_teleop/README.md)     |
| **Phone (WebXR)**           | A WebXR-capable phone                 | [`pai_phone_teleop`](../pai_phone_teleop/README.md)   |

## Leader arm

A human moves a torque-free **leader** arm by hand while the **follower** mirrors
the motion in real time. This is the recommended input method for collecting
real-world demonstrations, because the recorder captures the leader motion
transparently. See [`pai_leader_teleop`](../pai_leader_teleop/README.md).

```bash
ros2 launch pai_leader_teleop leader_bringup.launch.py usb_port:=/dev/so101_leader
# or: pixi run so-arm-leader usb_port:=/dev/so101_leader
```

## RViz interactive marker (differential IK)

Drag a 6-DOF interactive marker in RViz and the arm's tool frame follows it,
solved with the [Pink](https://github.com/stephane-caron/pink) differential IK
solver. See [`pai_rviz_teleop`](../pai_rviz_teleop/README.md).

```bash
pixi run so-arm-rviz-ik
```

## Phone (WebXR)

Track a phone's 6-DoF pose to teleoperate the arm via differential IK. Requires a
[WebXR](https://developer.mozilla.org/en-US/docs/Web/API/WebXR_Device_API)-capable
browser (e.g. Chrome on Android; iPhone is not supported). See
[`pai_phone_teleop`](../pai_phone_teleop/README.md).

```bash
pixi run so-arm-phone-ik
```

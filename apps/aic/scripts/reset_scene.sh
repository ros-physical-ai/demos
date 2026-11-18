#!/bin/bash
# Scene reset, run inside the simulator container through `pixi run reset-scene`.
#
# Removes the cable, then homes the arm the way aic_engine does: controller
# off, joints teleported, controller on. Under effort control the arm falls
# while the controller is off, so the three calls run back to back. The
# controller then re-captures its Cartesian reference through a target-mode
# round trip, which makes it hold the home pose. With the arm still, a fresh
# cable is spawned where aic_engine spawns it: at the gripper's live pose from
# TF plus the gripper offset, with the orientation from the engine's sample
# config. The cable plugin then fixes the cable to the gripper. A final
# re-capture lets the controller hold the pose under the cable's load.

set -eo pipefail

# shellcheck disable=SC1091
source /ws_aic/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'

CABLE_TYPE="${AIC_CABLE_TYPE:-sfp_sc_cable}"
CABLE_NAME=cable_0
WORLD=aic_world
GRIPPER_FRAME=gripper/tcp
# aic_control_interfaces/msg/TargetMode
MODE_CARTESIAN=1
MODE_JOINT=2

# Home joints: aic_engine's sample config, equal to the robot's initial positions.
JOINT_NAMES='[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]'
HOME_POSITIONS='[-0.1597, -1.3542, -1.6648, -1.6933, 1.5710, 1.4110]'

# Cable spawn pose relative to the gripper: aic_engine's sample config
# (scene.cables.cable_0.pose). Offset in world axes, orientation absolute.
CABLE_OFFSET_X=0.0
CABLE_OFFSET_Y=0.015385
CABLE_OFFSET_Z=0.04245
CABLE_ROLL=0.4432
CABLE_PITCH=-0.4838
CABLE_YAW=1.3303

switch_controller() {
  ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: [$1], deactivate_controllers: [$2], strictness: 2}" | grep -q 'ok=True'
}

change_target_mode() {
  ros2 service call /aic_controller/change_target_mode aic_control_interfaces/srv/ChangeTargetMode \
    "{target_mode: {mode: $1}}" | grep -q 'success=True'
}

recapture_reference() {
  change_target_mode "$MODE_JOINT"
  change_target_mode "$MODE_CARTESIAN"
}

# Prints "x y z" of the gripper frame in the world frame.
gripper_position() {
  python3 - "$GRIPPER_FRAME" <<'PY'
import sys, time
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time

frame = sys.argv[1]
rclpy.init()
node = Node("reset_scene_tf")
buffer = tf2_ros.Buffer()
tf2_ros.TransformListener(buffer, node)
deadline = time.time() + 10.0
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if buffer.can_transform("world", frame, Time()):
        t = buffer.lookup_transform("world", frame, Time()).transform.translation
        print(f"{t.x:.6f} {t.y:.6f} {t.z:.6f}")
        break
else:
    sys.exit(f"reset_scene: no transform world -> {frame}")
PY
}

echo "reset_scene: removing $CABLE_NAME"
gz service -s "/world/$WORLD/remove" --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 5000 --req "name: \"$CABLE_NAME\", type: MODEL" | grep -q 'data: true'

echo "reset_scene: homing the arm"
switch_controller '' 'aic_controller'
ros2 service call /scoring/reset_joints aic_engine_interfaces/srv/ResetJoints \
  "{joint_names: $JOINT_NAMES, initial_positions: $HOME_POSITIONS}" | grep -q 'success=True'
switch_controller 'aic_controller' ''
sleep 0.5
recapture_reference

echo "reset_scene: taring the force/torque sensor"
ros2 service call /aic_controller/tare_force_torque_sensor std_srvs/srv/Trigger | grep -q 'success=True'

# Let the arm come to rest at home before placing the cable in the gripper.
sleep 1.5
read -r tcp_x tcp_y tcp_z <<< "$(gripper_position)"
cable_x="$(python3 -c "print($tcp_x + $CABLE_OFFSET_X)")"
cable_y="$(python3 -c "print($tcp_y + $CABLE_OFFSET_Y)")"
cable_z="$(python3 -c "print($tcp_z + $CABLE_OFFSET_Z)")"

echo "reset_scene: spawning a $CABLE_TYPE in the gripper at ($cable_x, $cable_y, $cable_z)"
ros2 launch aic_bringup spawn_cable.launch.py attach_cable_to_gripper:=true "cable_type:=$CABLE_TYPE" \
  "cable_x:=$cable_x" "cable_y:=$cable_y" "cable_z:=$cable_z" \
  "cable_roll:=$CABLE_ROLL" "cable_pitch:=$CABLE_PITCH" "cable_yaw:=$CABLE_YAW" \
  2>&1 | grep -i 'error\|finished' || true

# The attached cable loads the gripper; let it settle, then make the
# controller re-capture its Cartesian reference so the arm holds this pose.
echo "reset_scene: re-capturing the controller reference"
sleep 1.5
recapture_reference

echo "reset_scene: done"

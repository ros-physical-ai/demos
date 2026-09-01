# Copyright 2026 demos contributors
# Apache 2.0

"""
Custom Rosetta converters for the AIC cable-insertion scenario.

These functions are referenced from the contract YAML
(pai_data_collection/config/rosetta/aic.yaml) via:

    decoder: pai_aic.converters:decode_aic_observation
    encoder: pai_aic.converters:encode_aic_motion_update
    decoder: pai_aic.converters:decode_aic_motion_update

They run at three sites with the same code path:
    1. episode_recorder_node  — live decode at record time
    2. port_bags              — offline decode at convert time
    3. rosetta_client_node    — live decode/encode at deploy time
       (and encode_aic_motion_update is also called directly from
       LerobotPolicy, which does not use the contract's encoder field)

The functions are ALSO registered in rosetta's global registry via
@register_decoder / @register_encoder. The contract's decoder/encoder
fields remain the source of truth for the actual message<->array
conversion; the decorators exist only so rosetta's registry-based dtype
lookups (port_bags._build_features / _sample_frame, which read
DTYPES[msg_type]) resolve without a KeyError. rosetta imports this module
while validating the contract's converter paths (contract.py
_validate_converter_path), so the decorators fire before port_bags builds
its features — no wrapper script or upstream patch is needed.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
from rosetta.common.converters import register_decoder, register_encoder

if TYPE_CHECKING:
    from aic_model_interfaces.msg import Observation
    from aic_control_interfaces.msg import MotionUpdate


# Joint order on the AIC UR5 (matches aic_controller config —
# aic_ros2_controllers.yaml → joints: [...]). Index 6 is the gripper,
# which aic_adapter publishes on /observations.joint_states under the
# name "gripper" (renamed from gripper/left_finger_joint and converted
# to finger-separation distance). This is pai_aic's choice; AIC itself
# does not mandate a specific layout.
_AIC_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "gripper",
)

# Documented 26-dim state layout. This is pai_aic's contract — anyone
# training a policy against this contract must produce features
# matching this layout.
_STATE_LAYOUT_DOC = """
[ 0: 7]  tcp_pose            — x, y, z (3) + qx, qy, qz, qw (4)
[ 7:10]  tcp_linear_velocity — x, y, z
[10:13]  tcp_angular_velocity — x, y, z
[13:19]  tcp_error           — x, y, z, rx, ry, rz
[19:26]  joint_positions     — 6 arm joints + 1 gripper (URDF order)
"""


@register_decoder("aic_model_interfaces/msg/Observation", dtype="float64")
def decode_aic_observation(msg: Any, spec: Any) -> np.ndarray:
    """Decode aic_model_interfaces/msg/Observation → 26-dim state vector.

    Layout (see _STATE_LAYOUT_DOC for the canonical ordering):

        [ 0: 7]  tcp_pose            — x,y,z (3) + qx,qy,qz,qw (4)
        [ 7:10]  tcp_linear_velocity — x,y,z
        [10:13]  tcp_angular_velocity — x,y,z
        [13:19]  tcp_error           — x,y,z,rx,ry,rz
        [19:26]  joint_positions     — 6 arm joints + 1 gripper

    Units are raw SI (radians, meters). Unit conversion (rad2deg, if any)
    is applied downstream by rosetta's decode_value() — leave this decoder
    in raw units.
    """
    cs = msg.controller_state
    tcp = cs.tcp_pose
    tcv = cs.tcp_velocity

    pose = np.array(
        [tcp.position.x, tcp.position.y, tcp.position.z,
         tcp.orientation.x, tcp.orientation.y, tcp.orientation.z, tcp.orientation.w],
        dtype=np.float64,
    )
    vel = np.array(
        [tcv.linear.x, tcv.linear.y, tcv.linear.z,
         tcv.angular.x, tcv.angular.y, tcv.angular.z],
        dtype=np.float64,
    )
    err = np.asarray(cs.tcp_error, dtype=np.float64)
    if err.shape != (6,):
        raise ValueError(f"tcp_error must be length 6, got shape {err.shape}")

    name_to_idx = {n: i for i, n in enumerate(msg.joint_states.name)}
    try:
        jp = np.array(
            [float(msg.joint_states.position[name_to_idx[n]]) for n in _AIC_JOINT_NAMES],
            dtype=np.float64,
        )
    except KeyError as e:
        raise ValueError(
            f"Joint {e} missing from /observations.joint_states. "
            f"Expected {_AIC_JOINT_NAMES}, got {list(msg.joint_states.name)}"
        ) from None

    return np.concatenate([pose, vel, err, jp])


@register_encoder("aic_control_interfaces/msg/MotionUpdate")
def encode_aic_motion_update(
    action_vec: np.ndarray, spec: Any, stamp_ns: int | None = None
) -> Any:
    """Encode 6- or 7-dim action → aic_control_interfaces/msg/MotionUpdate.

    Layout (must mirror decode_aic_motion_update's reverse order):

        [0:3] linear  — linear twist (m/s) in frame_id (default: base_link)
        [3:6] angular — angular twist (rad/s)
        [ 6 ] gripper — dropped here; see spec.

    The 7-dim shape is accepted for forward-compatibility with future
    gripper support, but only the first 6 dims are written into
    MotionUpdate. MotionUpdate has no gripper field; the impedance
    controller drives the 6 arm joints only.
    """
    from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Header

    arr = np.asarray(action_vec, dtype=np.float64).flatten()
    if arr.shape not in ((6,), (7,)):
        raise ValueError(f"action must be 6- or 7-dim, got shape {arr.shape}")

    msg = MotionUpdate()
    if stamp_ns is not None:
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    msg.header.frame_id = "base_link"

    msg.velocity = Twist(
        linear=Vector3(x=float(arr[0]), y=float(arr[1]), z=float(arr[2])),
        angular=Vector3(x=float(arr[3]), y=float(arr[4]), z=float(arr[5])),
    )
    msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY
    return msg


@register_decoder("aic_control_interfaces/msg/MotionUpdate", dtype="float64")
def decode_aic_motion_update(msg: Any, spec: Any) -> np.ndarray:
    """MotionUpdate → 6-dim action vector (inverse of encode_aic_motion_update).

    Layout:
        [0:3] linear  twist (m/s)
        [3:6] angular twist (rad/s)

    Used by port_bags at convert time to read recorded MotionUpdate msgs
    back into the LeRobot action vector. The pose and impedance fields
    of MotionUpdate are not used (we only record velocity-mode twists).
    """
    return np.array(
        [
            msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z,
            msg.velocity.angular.x, msg.velocity.angular.y, msg.velocity.angular.z,
        ],
        dtype=np.float64,
    )
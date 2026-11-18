"""Rosetta codecs for the twist action.

The contract's action channel carries `aic_control_interfaces/msg/MotionUpdate`.
These codecs map it to and from the 6-dimensional twist action: linear x, y, z
in m/s, then angular x, y, z in rad/s, expressed in the tool frame.
"""

import numpy as np
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode

TOOL_FRAME = "gripper/tcp"
ACTION_DIM = 6

# Impedance every command carries, at recording and at deployment. The values
# AIC's reference policy (aic_example_policies RunACT) uses: translation then
# rotation.
STIFFNESS_DIAGONAL = (100.0, 100.0, 100.0, 50.0, 50.0, 50.0)
DAMPING_DIAGONAL = (40.0, 40.0, 40.0, 15.0, 15.0, 15.0)


def decode_motion_update(msg: MotionUpdate, spec) -> np.ndarray:
    """Read the twist target out of a MotionUpdate."""
    v = msg.velocity
    return np.array(
        [v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z],
        dtype=np.float64,
    )


def encode_motion_update(action, spec, stamp_ns: int | None = None) -> MotionUpdate:
    """Build a velocity-mode MotionUpdate in the tool frame from a twist action."""
    action = np.asarray(action, dtype=np.float64).reshape(-1)
    if action.shape[0] != ACTION_DIM:
        raise ValueError(f"twist action must have {ACTION_DIM} values, got {action.shape[0]}")

    msg = MotionUpdate()
    msg.header.frame_id = TOOL_FRAME
    if stamp_ns is not None:
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)

    msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z = action[0:3]
    msg.velocity.angular.x, msg.velocity.angular.y, msg.velocity.angular.z = action[3:6]

    msg.target_stiffness = np.diag(STIFFNESS_DIAGONAL).flatten().tolist()
    msg.target_damping = np.diag(DAMPING_DIAGONAL).flatten().tolist()
    msg.wrench_feedback_gains_at_tip = [0.0] * 6
    msg.trajectory_generation_mode.mode = TrajectoryGenerationMode.MODE_VELOCITY
    return msg

"""Twist action codecs: the contract's action channel round-trips through MotionUpdate."""

import numpy as np
import pytest
from aic_control_interfaces.msg import MotionUpdate, TrajectoryGenerationMode

from pai_aic.codecs import decode_motion_update, encode_motion_update

TWIST = np.array([0.01, -0.02, 0.03, 0.1, -0.2, 0.3])


def test_encode_then_decode_reproduces_the_twist():
    """Encoding a twist and decoding the message gives the twist back."""
    msg = encode_motion_update(TWIST, spec=None)
    np.testing.assert_allclose(decode_motion_update(msg, spec=None), TWIST)


def test_encoded_command_is_a_velocity_target_in_the_tool_frame():
    """Encoded commands are velocity targets in the tool frame."""
    msg = encode_motion_update(TWIST, spec=None)
    assert isinstance(msg, MotionUpdate)
    assert msg.header.frame_id == "gripper/tcp"
    assert msg.trajectory_generation_mode.mode == TrajectoryGenerationMode.MODE_VELOCITY


def test_encoded_command_carries_the_stamp():
    """The given stamp lands in the header."""
    msg = encode_motion_update(TWIST, spec=None, stamp_ns=1_500_000_000)
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1, 500_000_000)


def test_encoded_command_uses_the_recording_impedance():
    """Encoded commands carry the impedance the demonstrations were recorded under."""
    msg = encode_motion_update(TWIST, spec=None)
    np.testing.assert_allclose(np.asarray(msg.target_stiffness).reshape(6, 6), np.diag([100.0] * 3 + [50.0] * 3))
    np.testing.assert_allclose(np.asarray(msg.target_damping).reshape(6, 6), np.diag([40.0] * 3 + [15.0] * 3))
    assert list(msg.wrench_feedback_gains_at_tip) == [0.0] * 6
    w = msg.feedforward_wrench_at_tip
    assert (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z) == (0.0,) * 6


def test_decode_reads_linear_then_angular():
    """Decoding orders linear before angular."""
    msg = MotionUpdate()
    msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z = 1.0, 2.0, 3.0
    msg.velocity.angular.x, msg.velocity.angular.y, msg.velocity.angular.z = 4.0, 5.0, 6.0
    np.testing.assert_array_equal(decode_motion_update(msg, spec=None), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])


def test_encode_rejects_wrong_width():
    """A vector with a width other than six is refused."""
    with pytest.raises(ValueError, match="6"):
        encode_motion_update(np.zeros(7), spec=None)

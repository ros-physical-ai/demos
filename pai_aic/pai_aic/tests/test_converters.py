# Copyright 2026 demos contributors
# Apache 2.0

"""Unit tests for pai_aic.converters."""

from types import SimpleNamespace

import numpy as np
import pytest


# Synthetic message factories ------------------------------------------------

def _make_observation(tcp_pose, tcp_vel, tcp_error, joint_names, joint_positions):
    """Build a duck-typed Observation.msg substitute.

    We avoid importing aic_model_interfaces at module-load time so the tests
    can be collected even before colcon has built the AIC packages. The
    converters only access msg.controller_state / msg.joint_states fields,
    so a SimpleNamespace with the right attributes is enough.
    """
    cs = SimpleNamespace(
        tcp_pose=SimpleNamespace(
            position=SimpleNamespace(x=tcp_pose[0], y=tcp_pose[1], z=tcp_pose[2]),
            orientation=SimpleNamespace(
                x=tcp_pose[3], y=tcp_pose[4], z=tcp_pose[5], w=tcp_pose[6]
            ),
        ),
        tcp_velocity=SimpleNamespace(
            linear=SimpleNamespace(x=tcp_vel[0], y=tcp_vel[1], z=tcp_vel[2]),
            angular=SimpleNamespace(x=tcp_vel[3], y=tcp_vel[4], z=tcp_vel[5]),
        ),
        tcp_error=list(tcp_error),
    )
    js = SimpleNamespace(
        name=list(joint_names),
        position=list(joint_positions),
    )
    return SimpleNamespace(controller_state=cs, joint_states=js)


# Tests ----------------------------------------------------------------------

class TestDecodeAicObservation:
    def test_returns_26_dim_vector(self):
        from pai_aic.converters import decode_aic_observation

        obs = _make_observation(
            tcp_pose=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],   # pos + quat
            tcp_vel=[0.01, 0.02, 0.03, 0.04, 0.05, 0.06],  # lin + ang
            tcp_error=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            joint_names=[
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
                "gripper_joint",
            ],
            joint_positions=[0.0, -0.5, 0.3, -1.2, 1.5, 0.0, 0.05],
        )
        out = decode_aic_observation(obs, spec=None)
        assert isinstance(out, np.ndarray)
        assert out.shape == (26,)
        assert out.dtype == np.float64

    def test_layout_matches_documented_order(self):
        from pai_aic.converters import decode_aic_observation

        tcp_pose = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        tcp_vel = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]
        tcp_err = [20.0, 21.0, 22.0, 23.0, 24.0, 25.0]
        joint_pos = [30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0]

        obs = _make_observation(
            tcp_pose=tcp_pose, tcp_vel=tcp_vel, tcp_error=tcp_err,
            joint_names=[
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
                "gripper_joint",
            ],
            joint_positions=joint_pos,
        )
        out = decode_aic_observation(obs, spec=None)
        np.testing.assert_allclose(out[0:7], tcp_pose)
        np.testing.assert_allclose(out[7:13], tcp_vel)
        np.testing.assert_allclose(out[13:19], tcp_err)
        np.testing.assert_allclose(out[19:26], joint_pos)

    def test_joint_order_resilient_to_msg_reordering(self):
        """joint_states.name[] order may differ from URDF; decoder must
        look up by name, not by index."""
        from pai_aic.converters import decode_aic_observation

        # Reverse the order in the message; the decoder should still produce
        # the canonical URDF-ordered output.
        names_reversed = [
            "gripper_joint", "wrist_3_joint", "wrist_2_joint", "wrist_1_joint",
            "elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
        ]
        positions_reversed = [99.0, 88.0, 77.0, 66.0, 55.0, 44.0, 33.0]
        # Canonical URDF order:
        expected = [33.0, 44.0, 55.0, 66.0, 77.0, 88.0, 99.0]

        obs = _make_observation(
            tcp_pose=[0]*7, tcp_vel=[0]*6, tcp_error=[0]*6,
            joint_names=names_reversed,
            joint_positions=positions_reversed,
        )
        out = decode_aic_observation(obs, spec=None)
        np.testing.assert_allclose(out[19:26], expected)

    def test_missing_joint_raises(self):
        from pai_aic.converters import decode_aic_observation

        obs = _make_observation(
            tcp_pose=[0]*7, tcp_vel=[0]*6, tcp_error=[0]*6,
            joint_names=["shoulder_pan_joint"],  # missing the rest
            joint_positions=[0.0],
        )
        with pytest.raises(ValueError, match="gripper_joint"):
            decode_aic_observation(obs, spec=None)

    def test_bad_tcp_error_shape_raises(self):
        from pai_aic.converters import decode_aic_observation

        obs = _make_observation(
            tcp_pose=[0]*7, tcp_vel=[0]*6, tcp_error=[1.0, 2.0, 3.0],  # wrong length
            joint_names=[
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
                "gripper_joint",
            ],
            joint_positions=[0.0]*7,
        )
        with pytest.raises(ValueError, match="tcp_error"):
            decode_aic_observation(obs, spec=None)


class TestEncodeAicMotionUpdate:
    def test_six_dim_action_encodes_to_twist(self):
        from pai_aic.converters import encode_aic_motion_update

        action = np.array([0.1, 0.2, 0.3, 0.01, 0.02, 0.03], dtype=np.float64)
        msg = encode_aic_motion_update(action, spec=None)
        assert msg.header.frame_id == "base_link"
        # Linear twist
        assert abs(msg.velocity.linear.x - 0.1) < 1e-9
        assert abs(msg.velocity.linear.y - 0.2) < 1e-9
        assert abs(msg.velocity.linear.z - 0.3) < 1e-9
        # Angular twist
        assert abs(msg.velocity.angular.x - 0.01) < 1e-9
        assert abs(msg.velocity.angular.y - 0.02) < 1e-9
        assert abs(msg.velocity.angular.z - 0.03) < 1e-9

    def test_stamp_ns_propagates_to_header(self):
        from pai_aic.converters import encode_aic_motion_update

        action = np.zeros(6, dtype=np.float64)
        # 1.5 seconds past epoch
        msg = encode_aic_motion_update(action, spec=None, stamp_ns=1_500_000_000)
        assert msg.header.stamp.sec == 1
        assert msg.header.stamp.nanosec == 500_000_000

    def test_no_stamp_leaves_header_zero(self):
        from pai_aic.converters import encode_aic_motion_update

        action = np.zeros(6, dtype=np.float64)
        msg = encode_aic_motion_update(action, spec=None)
        assert msg.header.stamp.sec == 0
        assert msg.header.stamp.nanosec == 0

    def test_trajectory_generation_mode_is_velocity(self):
        from pai_aic.converters import encode_aic_motion_update
        from aic_control_interfaces.msg import TrajectoryGenerationMode

        action = np.zeros(6, dtype=np.float64)
        msg = encode_aic_motion_update(action, spec=None)
        assert msg.trajectory_generation_mode.mode == (
            TrajectoryGenerationMode.MODE_VELOCITY
        )

    def test_rejects_bad_shape(self):
        from pai_aic.converters import encode_aic_motion_update

        with pytest.raises(ValueError, match="6- or 7-dim"):
            encode_aic_motion_update(np.zeros(5), spec=None)
        with pytest.raises(ValueError, match="6- or 7-dim"):
            encode_aic_motion_update(np.zeros(8), spec=None)

    def test_seven_dim_action_drops_gripper(self):
        """7-dim actions are accepted (gripper dim) but only the first 6
        dims go into MotionUpdate. Gripper control is not in the
        initial integration; see spec."""
        from pai_aic.converters import encode_aic_motion_update

        action = np.array([0.1, 0.2, 0.3, 0.01, 0.02, 0.03, 0.5], dtype=np.float64)
        msg = encode_aic_motion_update(action, spec=None)
        assert abs(msg.velocity.linear.x - 0.1) < 1e-9
        # The 7th dim is silently dropped from MotionUpdate


class TestDecodeAicMotionUpdate:
    def test_returns_six_dim_vector(self):
        from pai_aic.converters import decode_aic_motion_update

        msg = SimpleNamespace(
            velocity=SimpleNamespace(
                linear=SimpleNamespace(x=0.1, y=0.2, z=0.3),
                angular=SimpleNamespace(x=0.01, y=0.02, z=0.03),
            )
        )
        out = decode_aic_motion_update(msg, spec=None)
        assert isinstance(out, np.ndarray)
        assert out.shape == (6,)
        assert out.dtype == np.float64

    def test_layout_matches_encode(self):
        """Round-trip: encode then decode returns the same vector."""
        from pai_aic.converters import (
            decode_aic_motion_update,
            encode_aic_motion_update,
        )

        action = np.array([0.5, -0.5, 0.25, 0.1, -0.1, 0.0], dtype=np.float64)
        encoded = encode_aic_motion_update(action, spec=None, stamp_ns=None)
        out = decode_aic_motion_update(encoded, spec=None)
        np.testing.assert_allclose(out, action)
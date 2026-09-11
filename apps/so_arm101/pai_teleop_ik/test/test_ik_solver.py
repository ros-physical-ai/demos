# Copyright (C) 2026 Franco Cipollone
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the DifferentialIKSolver built from a minimal URDF."""

import numpy as np
import pinocchio as pin
import pytest

from pai_teleop_ik.ik_solver import DifferentialIKSolver

# Minimal 2-revolute arm + a fixed tool frame. "j1" and "j2" rotate about Z;
# "tool" is a fixed frame at the tip. "gripper" is an extra locked joint so we
# can exercise joint_position_limits on a non-IK joint.
URDF = """
<robot name="test_arm">
  <link name="world"/>
  <link name="l1"/>
  <link name="l2"/>
  <link name="tool_link"/>
  <link name="grip"/>
  <joint name="j1" type="revolute">
    <parent link="world"/><child link="l1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="10"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="10"/>
  </joint>
  <joint name="tool" type="fixed">
    <parent link="l2"/><child link="tool_link"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="gripper" type="revolute">
    <parent link="l2"/><child link="grip"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-0.5" upper="1.7" effort="1" velocity="10"/>
  </joint>
</robot>
"""


def _solver():
    return DifferentialIKSolver(URDF, ["j1", "j2"], "tool_link")


def test_joint_names_are_arm_joints():
    """The reduced model exposes only the requested arm joints, in order."""
    s = _solver()
    assert s.joint_names == ["j1", "j2"]


def test_configuration_from_dict_roundtrips():
    """A name->position mapping maps to the arm-joint configuration vector."""
    s = _solver()
    q = s.configuration_from_dict({"j1": 0.1, "j2": -0.2, "gripper": 0.0})
    assert np.allclose(q, [0.1, -0.2])


def test_configuration_from_dict_missing_returns_none():
    """A missing arm joint yields None instead of a partial configuration."""
    s = _solver()
    assert s.configuration_from_dict({"j1": 0.1}) is None


def test_forward_kinematics_returns_se3():
    """FK returns the tool SE3 at the expected location for the neutral pose."""
    s = _solver()
    s.reset(np.array([0.0, 0.0]))
    pose = s.forward_kinematics()
    assert isinstance(pose, pin.SE3)
    # Tool sits at x = 0.1 (j2 origin) + 0.1 (tool origin) with zero angles.
    assert np.allclose(pose.translation, [0.2, 0.0, 0.0], atol=1e-9)


def test_gripper_joint_limits_from_full_model():
    """Limits of a locked joint remain queryable via the full model."""
    s = _solver()
    lo, hi = s.joint_position_limits("gripper")
    assert (round(lo, 2), round(hi, 2)) == (-0.5, 1.7)


def test_unknown_ee_frame_raises():
    """An unknown end-effector frame is rejected at construction."""
    with pytest.raises(ValueError):
        DifferentialIKSolver(URDF, ["j1", "j2"], "nope_link")


def test_missing_arm_joint_raises():
    """An arm joint absent from the URDF is rejected at construction."""
    with pytest.raises(ValueError):
        DifferentialIKSolver(URDF, ["j1", "does_not_exist"], "tool_link")


def test_solve_moves_tool_toward_target():
    """Iterating the solve drives the tool frame closer to the target."""
    s = _solver()
    s.reset(np.array([0.0, 0.0]))
    start = s.forward_kinematics().translation.copy()
    target = pin.SE3(np.eye(3), np.array([0.15, 0.1, 0.0]))
    for _ in range(200):
        s.solve(target, 0.02)
    end = s.forward_kinematics().translation
    assert np.linalg.norm(end - target.translation) < np.linalg.norm(start - target.translation)

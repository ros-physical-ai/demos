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

"""Unit tests for the SE3 geodesic low-pass filter."""

import numpy as np
import pinocchio as pin

from pai_teleop_ik.se3_filter import se3_lowpass


def _se3(xyz):
    return pin.SE3(np.eye(3), np.array(xyz, dtype=float))


def test_alpha_one_is_passthrough():
    """alpha=1.0 should land exactly on the raw target."""
    filtered = _se3([0.0, 0.0, 0.0])
    raw = _se3([1.0, 2.0, 3.0])
    out = se3_lowpass(filtered, raw, 1.0)
    assert np.allclose(out.translation, raw.translation)


def test_step_moves_toward_target():
    """A partial step moves between the filtered and raw poses."""
    filtered = _se3([0.0, 0.0, 0.0])
    raw = _se3([1.0, 0.0, 0.0])
    out = se3_lowpass(filtered, raw, 0.5)
    # A half geodesic step in pure translation lands halfway.
    assert 0.0 < out.translation[0] < 1.0


def test_iteration_converges():
    """Repeated steps converge onto the raw target."""
    filtered = _se3([0.0, 0.0, 0.0])
    raw = _se3([1.0, -2.0, 0.5])
    for _ in range(200):
        filtered = se3_lowpass(filtered, raw, 0.5)
    assert np.allclose(filtered.translation, raw.translation, atol=1e-6)

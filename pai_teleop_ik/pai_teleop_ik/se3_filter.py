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

"""SE3 geodesic low-pass filter shared by the teleop frontends.

Steps a filtered pose a fraction ``alpha`` along the geodesic toward a raw
target pose using Pinocchio's log6/exp6 exponential map. ``alpha = 1.0`` is a
passthrough (no filtering); smaller values are smoother.
"""

from __future__ import annotations

import pinocchio as pin


def se3_lowpass(filtered: "pin.SE3", raw: "pin.SE3", alpha: float) -> "pin.SE3":
    """Return ``filtered`` stepped a fraction ``alpha`` toward ``raw``."""
    twist = pin.log6(filtered.actInv(raw)).vector
    return filtered * pin.exp6(alpha * twist)

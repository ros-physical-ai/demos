# Copyright (C) 2026 Sebastian Castro
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

"""Standalone differential inverse kinematics solver.

This module deliberately knows nothing about ROS, RViz, or any specific robot.
It wraps a Pinocchio model and the `Pink <https://github.com/stephane-caron/pink>`_
differential IK QP so that *any* teleoperation frontend (an RViz interactive
marker, a phone app, a leader arm, a scripted trajectory, ...) can reuse the
same solve by simply feeding it a target tool pose.

The robot is described entirely by its inputs:

  * ``urdf_xml`` -- the robot description.
  * ``arm_joints`` -- the ordered list of joints the IK is allowed to actuate.
    Every other joint in the URDF is locked at its neutral configuration, so the
    solver is agnostic to the particular arm (the SO-ARM101 is just the default
    used by the demo node).
  * ``ee_frame`` -- the tool frame driven toward the target.

Typical usage::

    solver = DifferentialIKSolver(urdf_xml, arm_joints, ee_frame)
    solver.reset(solver.configuration_from_dict(joint_positions))
    pose = solver.forward_kinematics()        # current tool pose (SE3)
    q = solver.solve(target_pose, dt)         # one differential IK step
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin
from pink import solve_ik
from pink.configuration import Configuration
from pink.limits import ConfigurationLimit, VelocityLimit
from pink.tasks import FrameTask, PostureTask


class DifferentialIKSolver:
    """Robot- and frontend-agnostic differential IK solver.

    The solver holds the current arm configuration internally. Frontends seed it
    with :meth:`reset` (typically from measured joint states) and then call
    :meth:`solve` once per control tick with the desired tool pose.
    """

    def __init__(
        self,
        urdf_xml: str,
        arm_joints: list[str],
        ee_frame: str,
        *,
        position_cost: float = 0.5,
        orientation_cost: float = 0.1,
        posture_cost: float = 1e-3,
        lm_damping: float = 1e-2,
        max_joint_velocity: float = 2.0,
        qp_solver: str = "quadprog",
    ):
        """Build the reduced Pinocchio model and the differential IK tasks.

        Args:
            urdf_xml: The robot description (URDF XML string).
            arm_joints: Ordered list of joint names the IK may actuate. All
                other joints are locked at their neutral configuration.
            ee_frame: Name of the tool frame driven toward the target pose.
            position_cost: FrameTask position weight.
            orientation_cost: FrameTask orientation weight.
            posture_cost: PostureTask regularization weight (rest-pose pull).
            lm_damping: FrameTask Levenberg-Marquardt damping; regularizes the
                QP in near-singular directions so commanded velocities stay
                bounded.
            max_joint_velocity: Per-joint velocity cap (rad/s) enforced as a QP
                constraint.
            qp_solver: QP backend used by Pink (e.g. ``"quadprog"``).

        """
        self.arm_joints = list(arm_joints)
        self.ee_frame = ee_frame
        self.qp_solver = qp_solver

        # Keep the full model so callers can query joint limits of locked joints
        # (e.g. a gripper) that are not part of the reduced IK model.
        self.full_model = pin.buildModelFromXML(urdf_xml)

        missing = [j for j in self.arm_joints if not self.full_model.existJointName(j)]
        if missing:
            available = [self.full_model.names[jid] for jid in range(1, self.full_model.njoints)]
            raise ValueError(f"Arm joints {missing} not found in URDF. Available joints: {available}")

        # Lock every joint that is not one of the arm joints so the IK only
        # actuates ``arm_joints``. Locked joints are pinned at neutral.
        locked_joint_ids = [
            jid
            for jid in range(1, self.full_model.njoints)  # joint 0 is the universe
            if self.full_model.names[jid] not in self.arm_joints
        ]
        q_ref = pin.neutral(self.full_model)
        self.model = pin.buildReducedModel(self.full_model, locked_joint_ids, q_ref)
        self.data = self.model.createData()

        if not self.model.existFrame(self.ee_frame):
            available = [f.name for f in self.model.frames]
            raise ValueError(f"End-effector frame '{self.ee_frame}' not found in model. Available frames: {available}")

        # Joint names of the reduced model, in configuration-vector order. This
        # is what frontends use to map joint states into the configuration.
        self.joint_names = [self.model.names[jid] for jid in range(1, self.model.njoints)]

        # Differential IK tasks. The frame task target is retargeted every solve.
        self._frame_task = FrameTask(
            self.ee_frame,
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            lm_damping=lm_damping,
        )
        self._posture_task = PostureTask(cost=posture_cost)

        # Limits enforced by the QP solve. The ConfigurationLimit keeps the
        # integrated configuration inside the URDF joint position bounds as a
        # proper QP constraint (no post-hoc clamping). The VelocityLimit reads
        # the model's velocityLimit, so override it with the tunable per-joint
        # cap (the URDF limits are effectively unbounded) before constructing it.
        self.model.velocityLimit[:] = max_joint_velocity
        self._limits = [ConfigurationLimit(self.model), VelocityLimit(self.model)]

        # Start at the neutral configuration until a frontend seeds it.
        self._q = pin.neutral(self.model)
        self._configuration = Configuration(self.model, self.data, self._q)
        # The posture task is a fixed rest-pose regularizer, not retargeted to
        # the live configuration.
        self._posture_task.set_target(self._q)

    # ------------------------------------------------------------------
    # Configuration access
    # ------------------------------------------------------------------
    @property
    def q(self) -> np.ndarray:
        """The current configuration vector (in ``joint_names`` order)."""
        return self._q

    def configuration_from_dict(self, positions: dict) -> np.ndarray | None:
        """Build a configuration vector from a name -> position mapping.

        Returns ``None`` if any arm joint is missing from ``positions``.
        """
        if not all(j in positions for j in self.joint_names):
            return None
        return np.array([positions[j] for j in self.joint_names])

    def reset(self, q: np.ndarray) -> None:
        """Seed the internal configuration (e.g. from measured joint states)."""
        self._q = np.asarray(q, dtype=float)
        self._configuration.update(self._q)

    def forward_kinematics(self, q: np.ndarray | None = None) -> "pin.SE3":
        """Return the tool-frame pose for ``q`` (defaults to the current one)."""
        if q is None:
            q = self._q
        else:
            self._configuration.update(np.asarray(q, dtype=float))
        return self._configuration.get_transform_frame_to_world(self.ee_frame)

    def joint_position_limits(self, joint_name: str) -> tuple[float, float]:
        """Return the ``(lower, upper)`` position limits of a URDF joint.

        Looks up the *full* model so limits of locked joints (e.g. a gripper)
        remain queryable by frontends.
        """
        if not self.full_model.existJointName(joint_name):
            raise ValueError(f"Joint '{joint_name}' not found in URDF.")
        jid = self.full_model.getJointId(joint_name)
        qidx = self.full_model.joints[jid].idx_q
        return (
            float(self.full_model.lowerPositionLimit[qidx]),
            float(self.full_model.upperPositionLimit[qidx]),
        )

    # ------------------------------------------------------------------
    # Solve
    # ------------------------------------------------------------------
    def solve(self, target_pose: "pin.SE3", dt: float) -> np.ndarray:
        """Run one differential IK step toward ``target_pose``.

        Solves the Pink QP for a joint velocity that drives the tool frame
        toward ``target_pose`` (subject to configuration and velocity limit
        constraints), integrates it over ``dt``, and updates the internal
        configuration.

        Args:
            target_pose: Desired tool pose (SE3) in the model root frame.
            dt: Integration timestep in seconds.

        Returns:
            The new configuration vector (in ``joint_names`` order).

        """
        q = self._q
        self._configuration.update(q)
        self._frame_task.set_target(target_pose)
        velocity = solve_ik(
            self._configuration,
            [self._frame_task, self._posture_task],
            dt,
            solver=self.qp_solver,
            limits=self._limits,
        )
        q_next = pin.integrate(self.model, q, velocity * dt)
        self._q = q_next
        return q_next

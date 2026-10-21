"""The arm model, loaded exactly the way the C++ Dynamics loads it.

Same reduction as src/dynamics.cpp: keep the joints on the chain root -> ee_frame,
lock every other joint at q = 0 (an articulated gripper stays lumped at the
wrist), and pack continuous joints as (cos, sin). Anything else here would fit
a different model than the one the driver runs.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin

NUM_JOINTS = 7
# Static gravity only touches the first four of Pinocchio's ten dynamic parameters
# per body: (m, m*c_x, m*c_y, m*c_z), in the body's joint frame.
PARAMS_PER_BODY = 4


@dataclass
class ArmModel:
    model: pin.Model
    data: pin.Data
    joint_names: list[str]
    lower: np.ndarray
    upper: np.ndarray
    continuous: np.ndarray
    ee_frame_id: int

    @classmethod
    def load(cls, urdf_path: str, ee_frame: str) -> "ArmModel":
        return cls.from_model(pin.buildModelFromUrdf(urdf_path), ee_frame)

    @classmethod
    def from_model(cls, full: pin.Model, ee_frame: str) -> "ArmModel":
        if not full.existFrame(ee_frame):
            raise ValueError(f"EE frame '{ee_frame}' not in URDF")
        chain = list(full.supports[full.frames[full.getFrameId(ee_frame)].parentJoint])
        lock = [j for j in range(1, full.njoints) if j not in chain]
        m = pin.buildReducedModel(full, lock, pin.neutral(full)) if lock else full
        if m.nv != NUM_JOINTS or m.njoints != NUM_JOINTS + 1:
            raise ValueError(f"reduced chain has nv={m.nv}, expected {NUM_JOINTS} (wrong URDF or EE frame)")
        for j in range(1, m.njoints):
            if m.nvs[j] != 1 or not m.joints[j].shortname().startswith("JointModelR"):
                raise ValueError(f"joint '{m.names[j]}' is {m.joints[j].shortname()}; must be 1-DOF revolute")
        continuous = np.array([m.nqs[j] == 2 for j in range(1, m.njoints)])
        lower = np.full(NUM_JOINTS, -np.inf)
        upper = np.full(NUM_JOINTS, np.inf)
        for i in range(NUM_JOINTS):
            if not continuous[i]:
                k = m.idx_qs[i + 1]
                lower[i], upper[i] = m.lowerPositionLimit[k], m.upperPositionLimit[k]
        return cls(m, m.createData(), list(m.names)[1:], lower, upper, continuous, m.getFrameId(ee_frame))

    def pack(self, q: np.ndarray) -> np.ndarray:
        m = self.model
        cfg = pin.neutral(m)
        for i in range(NUM_JOINTS):
            k = m.idx_qs[i + 1]
            if self.continuous[i]:
                cfg[k], cfg[k + 1] = np.cos(q[i]), np.sin(q[i])
            else:
                cfg[k] = q[i]
        return cfg

    def gravity(self, q: np.ndarray) -> np.ndarray:
        return pin.computeGeneralizedGravity(self.model, self.data, self.pack(q)).copy()

    def regressor(self, q: np.ndarray) -> np.ndarray:
        """7 x 28 static regressor: regressor(q) @ params() == gravity(q)."""
        zero = np.zeros(NUM_JOINTS)
        Y = pin.computeJointTorqueRegressor(self.model, self.data, self.pack(q), zero, zero)
        cols = [10 * b + k for b in range(NUM_JOINTS) for k in range(PARAMS_PER_BODY)]
        return Y[:, cols].copy()

    def params(self) -> np.ndarray:
        """The URDF's own (m, m*c) per body 1..7, flattened (28,)."""
        return np.concatenate(
            [self.model.inertias[j].toDynamicParameters()[:PARAMS_PER_BODY] for j in range(1, NUM_JOINTS + 1)]
        )

    def up(self) -> np.ndarray:
        g = self.model.gravity.linear
        return -g / np.linalg.norm(g)

    def frame_heights(self, q: np.ndarray) -> np.ndarray:
        """Height of every joint origin and the EE along the model's up direction [m]."""
        pin.forwardKinematics(self.model, self.data, self.pack(q))
        pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        pts = [self.data.oMi[j].translation for j in range(1, self.model.njoints)]
        pts.append(self.data.oMf[self.ee_frame_id].translation)
        return np.array(pts) @ self.up()

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, self.pack(q))
        return pin.updateFramePlacement(self.model, self.data, self.ee_frame_id).translation.copy()

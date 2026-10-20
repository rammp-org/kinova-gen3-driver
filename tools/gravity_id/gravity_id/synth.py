"""Synthetic arm data: the URDF plus known injected errors, for proving the fit."""

from __future__ import annotations

import numpy as np

from gravity_id.model import NUM_JOINTS, PARAMS_PER_BODY, ArmModel
from gravity_id.posegrid import Pose
from gravity_id.transcript import Hold


def point_mass(body: int, mass: float, com: np.ndarray) -> np.ndarray:
    """(28,) correction adding `mass` kg at `com` (joint-`body` frame, m)."""
    dpi = np.zeros(PARAMS_PER_BODY * NUM_JOINTS)
    k = PARAMS_PER_BODY * (body - 1)
    dpi[k] = mass
    dpi[k + 1 : k + 4] = mass * np.asarray(com, dtype=float)
    return dpi


def scale_body(arm: ArmModel, body: int, factor: float) -> np.ndarray:
    """(28,) correction scaling a body's mass (and first moment with it) by `factor`."""
    dpi = np.zeros(PARAMS_PER_BODY * NUM_JOINTS)
    k = PARAMS_PER_BODY * (body - 1)
    dpi[k : k + 4] = (factor - 1.0) * arm.params()[k : k + 4]
    return dpi


def true_torque(arm: ArmModel, q: np.ndarray, dpi: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    return arm.gravity(q) + arm.regressor(q) @ dpi + offsets


def make_holds(
    arm: ArmModel,
    poses: list[Pose],
    dpi: np.ndarray | None = None,
    offsets: np.ndarray | None = None,
    noise: float = 0.05,
    seed: int = 0,
) -> list[Hold]:
    rng = np.random.default_rng(seed)
    dpi = np.zeros(PARAMS_PER_BODY * NUM_JOINTS) if dpi is None else dpi
    offsets = np.zeros(NUM_JOINTS) if offsets is None else np.asarray(offsets, dtype=float)
    return [
        Hold(p.q, true_torque(arm, p.q, dpi, offsets) + rng.normal(0, noise, NUM_JOINTS), np.full(NUM_JOINTS, noise), 8, p.id, p.set)
        for p in poses
    ]

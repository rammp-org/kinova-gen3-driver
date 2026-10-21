"""Prior scales and the scaled design matrix shared by the pose grid and the fit.

Unknowns are the correction to each body's (m, m*c) plus a per-joint torque offset,
each divided by its prior std so every column is in "prior units". Values from the
spec (Phase I): sigma_m = max(0.2 kg, 0.3 m), sigma_mc = 0.1 m * sigma_m, sigma_o = 2 N*m.
"""

from __future__ import annotations

import numpy as np

from gravity_id.model import NUM_JOINTS, PARAMS_PER_BODY, ArmModel

SIGMA_OFFSET = 2.0  # N*m
SIGMA_TAU = 0.1  # N*m, measurement noise
ALL_BODIES = tuple(range(1, NUM_JOINTS + 1))


def prior_std(arm: ArmModel) -> np.ndarray:
    """(28,) prior std of the (m, m*c) correction per body 1..7."""
    out = []
    for j in ALL_BODIES:
        s_m = max(0.2, 0.3 * arm.model.inertias[j].mass)
        out += [s_m, 0.1 * s_m, 0.1 * s_m, 0.1 * s_m]
    return np.array(out)


def body_columns(bodies: tuple[int, ...]) -> np.ndarray:
    return np.array([PARAMS_PER_BODY * (j - 1) + k for j in bodies for k in range(PARAMS_PER_BODY)], dtype=int)


def design(arm: ArmModel, q: np.ndarray, bodies: tuple[int, ...], offsets: bool) -> np.ndarray:
    """7 x n scaled design for one pose: chosen body columns, then 7 offset columns."""
    cols = body_columns(bodies)
    blocks = [arm.regressor(q)[:, cols] * prior_std(arm)[cols]]
    if offsets:
        blocks.append(np.eye(NUM_JOINTS) * SIGMA_OFFSET)
    return np.hstack(blocks)

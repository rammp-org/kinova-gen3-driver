"""Nested-family gravity fit with held-out selection (spec, Phase I).

Each family fits a correction to the URDF as a Bayesian ridge: the unknowns are
scaled by their prior std (design.py), so directions the data pins down are set
by the data and directions it cannot see stay at the URDF instead of wandering.
Physical checks after the fit reject a family; they never clip it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from gravity_id.design import ALL_BODIES, SIGMA_OFFSET, SIGMA_TAU, body_columns, design, prior_std
from gravity_id.model import NUM_JOINTS, PARAMS_PER_BODY, ArmModel
from gravity_id.transcript import Hold

# name -> (bodies whose (m, m*c) are free, offsets free)
FAMILIES: dict[str, tuple[tuple[int, ...], bool]] = {
    "F0": ((), False),
    "F1": ((), True),
    "F2": ((7,), True),
    "F3": ((5, 6, 7), True),
    "F4": (ALL_BODIES, True),
}
SELECT_TOLERANCE = 1.10  # smallest family within 10% of the best held-out RMS
MAX_COM_M = 0.5


@dataclass
class FamilyFit:
    family: str
    dpi: np.ndarray  # (28,) correction to (m, m*c) per body, zero outside the family
    offsets: np.ndarray  # (7,)
    std_dpi: np.ndarray
    std_offsets: np.ndarray
    rejected: str | None = None


@dataclass
class Selection:
    fits: dict[str, FamilyFit]
    val_rms: dict[str, np.ndarray] = field(default_factory=dict)  # per joint
    val_max: dict[str, np.ndarray] = field(default_factory=dict)
    # What corrected.urdf alone delivers: the same residuals WITHOUT the fitted offsets,
    # which the URDF cannot carry. The static gate is judged on these.
    urdf_rms: dict[str, np.ndarray] = field(default_factory=dict)
    urdf_max: dict[str, np.ndarray] = field(default_factory=dict)
    chosen: str = ""


def n_unknowns(family: str) -> int:
    bodies, offsets = FAMILIES[family]
    return PARAMS_PER_BODY * len(bodies) + (NUM_JOINTS if offsets else 0)


def fit_family(arm: ArmModel, holds: list[Hold], family: str) -> FamilyFit:
    bodies, offsets = FAMILIES[family]
    zeros = np.zeros(PARAMS_PER_BODY * NUM_JOINTS)
    if n_unknowns(family) == 0:
        return FamilyFit(family, zeros, np.zeros(NUM_JOINTS), zeros.copy(), np.zeros(NUM_JOINTS))
    A = np.vstack([design(arm, h.q, bodies, offsets) for h in holds]) / SIGMA_TAU
    r = np.concatenate([h.tau - arm.gravity(h.q) for h in holds]) / SIGMA_TAU
    n = A.shape[1]
    x = np.linalg.lstsq(np.vstack([A, np.eye(n)]), np.concatenate([r, np.zeros(n)]), rcond=None)[0]
    std_x = np.sqrt(np.diag(np.linalg.inv(A.T @ A + np.eye(n))))

    cols = body_columns(bodies)
    scale = prior_std(arm)[cols]
    nb = len(cols)
    dpi, std_dpi = zeros.copy(), zeros.copy()
    dpi[cols], std_dpi[cols] = x[:nb] * scale, std_x[:nb] * scale
    off, std_off = np.zeros(NUM_JOINTS), np.zeros(NUM_JOINTS)
    if offsets:
        off, std_off = x[nb:] * SIGMA_OFFSET, std_x[nb:] * SIGMA_OFFSET
    fit = FamilyFit(family, dpi, off, std_dpi, std_off)
    fit.rejected = physical_check(arm, dpi, bodies)
    return fit


def physical_check(arm: ArmModel, dpi: np.ndarray, bodies: tuple[int, ...]) -> str | None:
    pi = arm.params() + dpi
    for j in bodies:
        m, h = pi[4 * (j - 1)], pi[4 * (j - 1) + 1 : 4 * j]
        if m <= 0:
            return f"body {j} mass {m:.3f} kg <= 0"
        if np.linalg.norm(h / m) > MAX_COM_M:
            return f"body {j} COM {np.linalg.norm(h / m):.3f} m from its joint (> {MAX_COM_M} m)"
    return None


def predict(arm: ArmModel, fit: FamilyFit, q: np.ndarray) -> np.ndarray:
    return arm.gravity(q) + arm.regressor(q) @ fit.dpi + fit.offsets


def select(arm: ArmModel, holds: list[Hold]) -> Selection:
    fit_h = [h for h in holds if h.set == "fit"]
    val_h = [h for h in holds if h.set == "val"]
    if not val_h:
        raise ValueError("no 'val' holds: selection needs held-out poses (spec: validation never reuses fit poses)")
    need = 2 * max(n_unknowns(f) for f in FAMILIES)
    if NUM_JOINTS * len(fit_h) < need:
        raise ValueError(
            f"{len(fit_h)} fit holds give {NUM_JOINTS * len(fit_h)} torque readings; need >= {need} "
            "(2x the largest family's unknowns)"
        )
    sel = Selection({f: fit_family(arm, fit_h, f) for f in FAMILIES})
    for f, fit in sel.fits.items():
        err = np.array([predict(arm, fit, h.q) - h.tau for h in val_h])
        sel.val_rms[f] = np.sqrt(np.mean(err**2, axis=0))
        sel.val_max[f] = np.abs(err).max(axis=0)
        err_urdf = err - fit.offsets
        sel.urdf_rms[f] = np.sqrt(np.mean(err_urdf**2, axis=0))
        sel.urdf_max[f] = np.abs(err_urdf).max(axis=0)
    ok = [f for f in FAMILIES if sel.fits[f].rejected is None]  # F0 is never rejected
    total = {f: float(np.sqrt(np.mean(sel.val_rms[f] ** 2))) for f in ok}
    best = min(total.values())
    sel.chosen = next(f for f in ok if total[f] <= SELECT_TOLERANCE * best)
    return sel

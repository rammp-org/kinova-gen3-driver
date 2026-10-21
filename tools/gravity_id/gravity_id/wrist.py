"""Fit a payload added at the wrist (a camera, fingertips) from j7 sweeps.

The mass comes from a scale. On j5-j7 a payload's mass and its offset along the tool
axis move the torques almost identically (a fit of both leaves ~3 kg of uncertainty),
and the proximal joints that would separate them carry ~0.5 N*m of friction. Given the
mass, j7 turned through a full turn at two or more j6 angles fixes m*c to ~1 g*m.

Unknowns: m*c (joint-7 frame), the j5-j7 torque offsets, and j7's gearbox ripple as
harmonics of q7. dojo's ripple is 0.17 N*m at 3 per turn. At one j6 angle a 1-per-turn
part looks exactly like an off-axis mass on j7; across j6 angles the mass's torque
scales with the tilt of j7's axis and the ripple's does not, which separates them.

Each j6 angle must be approached from both sides (onarm/wrist_sweep.sh does): j6
friction that differs between sweeps reads as m*c_z, and the printed uncertainty
cannot see it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from gravity_id.model import NUM_JOINTS, PARAMS_PER_BODY, ArmModel
from gravity_id.transcript import Hold

WRIST = (4, 5, 6)  # j5, j6, j7
HARMONICS = (1, 2, 3)  # of q7, in j7's ripple
_M = PARAMS_PER_BODY * (NUM_JOINTS - 1)  # body 7's mass column; m*c follows it
MAX_COND = 100.0  # column-scaled; j6 30/90 deg -> 6, 30/50 -> 64, 30/35 -> 1e3


@dataclass
class WristFit:
    com: np.ndarray  # m, joint-7 frame
    std_com: np.ndarray
    offsets: np.ndarray  # j5, j6, j7, N*m
    ripple: np.ndarray  # amplitude per harmonic, N*m
    rms: float  # N*m
    n_holds: int


def _rows(arm: ArmModel, h: Hold, mass: float) -> tuple[np.ndarray, np.ndarray]:
    Y = arm.regressor(h.q)
    y = (h.tau - arm.gravity(h.q) - mass * Y[:, _M])[list(WRIST)]
    A = np.zeros((len(WRIST), 3 + len(WRIST) + 2 * len(HARMONICS)))
    A[:, :3] = Y[list(WRIST), _M + 1 : _M + 4]
    A[:, 3 : 3 + len(WRIST)] = np.eye(len(WRIST))
    A[-1, 3 + len(WRIST) :] = np.ravel([(np.sin(n * h.q[6]), np.cos(n * h.q[6])) for n in HARMONICS])
    return A, y


def fit_wrist(arm: ArmModel, holds: list[Hold], mass: float) -> WristFit:
    """Least squares over every hold; `mass` [kg] is the payload weighed on a scale."""
    if mass <= 0.0:
        raise ValueError(f"payload mass must be > 0 kg (weigh it), got {mass}")
    rows = [_rows(arm, h, mass) for h in holds]
    A, y = np.vstack([r[0] for r in rows]), np.concatenate([r[1] for r in rows])
    s = np.linalg.svd(A / np.linalg.norm(A, axis=0), compute_uv=False)
    if len(y) <= A.shape[1] or s[-1] * MAX_COND < s[0]:
        j6 = sorted({round(float(np.degrees(h.q[5]))) for h in holds})
        raise ValueError(f"{len(holds)} holds, j6 at {j6} deg: these do not fix the payload's centre of mass. "
                         "Turn j7 through a full turn (30 deg steps) at two or more j6 angles, "
                         "at least 20 deg apart (60 or more is better)")
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = A @ x - y
    cov = np.linalg.inv(A.T @ A) * (res @ res) / (len(y) - A.shape[1])
    h = x[3 + len(WRIST) :].reshape(-1, 2)
    return WristFit(x[:3] / mass, np.sqrt(np.diag(cov)[:3]) / mass, x[3 : 3 + len(WRIST)],
                    np.hypot(h[:, 0], h[:, 1]), float(np.sqrt(np.mean(res**2))), len(holds))

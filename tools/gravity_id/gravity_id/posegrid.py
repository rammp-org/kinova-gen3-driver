"""Pose grid for the read-only diagnostic session (spec, Phase D1).

Candidates are sampled within the joint limits (minus a margin) and a j1 sector,
kept only if every joint origin and the EE are at least z_min above the base along
the model's up direction. Fit poses are chosen greedily for D-optimality on the
largest family's identifiable subspace; validation poses are random survivors so
they are not cherry-picked to be easy.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass

import numpy as np

from gravity_id.design import ALL_BODIES, design
from gravity_id.model import NUM_JOINTS, ArmModel


@dataclass
class Pose:
    id: str
    set: str  # "fit" | "val"
    q: np.ndarray  # rad


def wrap(a: np.ndarray) -> np.ndarray:
    """Wrap angles to (-pi, pi]."""
    return np.pi - np.mod(np.pi - a, 2 * np.pi)


def sample_candidates(arm, n, z_min, j1_range_deg, margin_deg, rng) -> list[np.ndarray]:
    margin = np.radians(margin_deg)
    lo = np.where(arm.continuous, -np.pi, arm.lower + margin)
    hi = np.where(arm.continuous, np.pi, arm.upper - margin)
    lo[0], hi[0] = max(lo[0], np.radians(j1_range_deg[0])), min(hi[0], np.radians(j1_range_deg[1]))
    out = []
    while len(out) < n:
        q = rng.uniform(lo, hi)
        if arm.frame_heights(q).min() >= z_min:
            out.append(q)
    return out


def make_grid(
    arm: ArmModel,
    n_fit: int = 40,
    n_val: int = 10,
    z_min: float = 0.10,
    j1_range_deg: tuple[float, float] = (-180.0, 180.0),
    margin_deg: float = 10.0,
    n_candidates: int = 3000,
    seed: int = 0,
) -> list[Pose]:
    rng = np.random.default_rng(seed)
    cand = sample_candidates(arm, n_candidates, z_min, j1_range_deg, margin_deg, rng)
    D = np.array([design(arm, q, ALL_BODIES, True) for q in cand])  # (n, 7, p)
    _, s, Vt = np.linalg.svd(D.reshape(-1, D.shape[2]), full_matrices=False)
    V = Vt[s > 1e-6 * s[0]].T
    P = D @ V  # projected onto the identifiable subspace, (n, 7, r)
    info = 1e-6 * np.eye(V.shape[1])
    chosen: list[int] = []
    for _ in range(n_fit):
        inv = np.linalg.inv(info)
        # Matrix determinant lemma: logdet(info + P_i^T P_i) - logdet(info) = logdet(I + P_i inv P_i^T)
        gain = np.linalg.slogdet(np.eye(NUM_JOINTS) + P @ inv @ P.transpose(0, 2, 1))[1]
        gain[chosen] = -np.inf
        i = int(np.argmax(gain))
        chosen.append(i)
        info += P[i].T @ P[i]
    rest = np.setdiff1d(np.arange(len(cand)), chosen)
    val = list(rng.choice(rest, n_val, replace=False))
    labelled = [(cand[i], "fit") for i in chosen] + [(cand[i], "val") for i in val]

    # Nearest-neighbour tour from q = 0 to cut travel between poses.
    order, here, left = [], np.zeros(NUM_JOINTS), list(range(len(labelled)))
    while left:
        k = min(left, key=lambda k: np.linalg.norm(wrap(labelled[k][0] - here)))
        order.append(k)
        left.remove(k)
        here = labelled[k][0]
    return [Pose(f"P{n:02d}", labelled[k][1], labelled[k][0]) for n, k in enumerate(order)]


def write_poses_csv(path, poses: list[Pose]) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "set"] + [f"j{i}_deg" for i in range(1, NUM_JOINTS + 1)])
        for p in poses:
            w.writerow([p.id, p.set] + [f"{d:.2f}" for d in np.degrees(wrap(p.q))])


def read_poses_csv(path) -> list[Pose]:
    with open(path, newline="") as f:
        return [
            Pose(r["id"], r["set"], np.radians([float(r[f"j{i}_deg"]) for i in range(1, NUM_JOINTS + 1)]))
            for r in csv.DictReader(f)
        ]

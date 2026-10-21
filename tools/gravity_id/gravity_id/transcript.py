"""Parse `benchmark_grav_comp --dry-run` stdout into held poses.

The dry-run prints, every 500 ms, a table in exactly this format
(apps/benchmark_grav_comp.cpp):

      j |   q[deg] | meas[Nm] | grav[Nm] |  resid[Nm]
      1 |     0.00 |     0.00 |    -0.00 |    +0.000
      ... seven rows ...

Everything else in the transcript is ignored. Parsing stdout (instead of adding a
CSV flag) lets the session run on the binary already deployed on the Jetson.
"""

from __future__ import annotations

import csv
import re
from dataclasses import dataclass

import numpy as np

from gravity_id.model import NUM_JOINTS, ArmModel
from gravity_id.posegrid import Pose, wrap

HEADER = "  j |   q[deg] | meas[Nm] | grav[Nm] |  resid[Nm]"
ROW = re.compile(r"^\s+(\d) \|\s*(\S+) \|\s*(\S+) \|\s*(\S+) \|\s*(\S+)\s*$")


@dataclass
class Table:
    q: np.ndarray  # rad
    tau: np.ndarray  # N*m, command convention (~ +g at free hold)
    grav: np.ndarray  # N*m, the driver's own g(q)


@dataclass
class Hold:
    q: np.ndarray
    tau: np.ndarray
    tau_std: np.ndarray
    n: int
    pose_id: str = ""
    set: str = ""


def format_table(q: np.ndarray, tau: np.ndarray, grav: np.ndarray) -> str:
    """The dry-run's own printf, for tests and synthetic transcripts."""
    lines = [HEADER]
    for i in range(NUM_JOINTS):
        lines.append(
            "  %d | %8.2f | %8.2f | %8.2f | %+9.3f" % (i + 1, np.degrees(q[i]), tau[i], grav[i], tau[i] - grav[i])
        )
    return "\n".join(lines) + "\n"


def parse_tables(text: str) -> list[Table]:
    lines = text.splitlines()
    tables = []
    for k, line in enumerate(lines):
        if line.rstrip() != HEADER.rstrip():
            continue
        rows = [ROW.match(x) for x in lines[k + 1 : k + 1 + NUM_JOINTS]]
        if len(rows) < NUM_JOINTS or not all(rows) or [int(r[1]) for r in rows] != list(range(1, NUM_JOINTS + 1)):
            continue  # truncated table (e.g. Ctrl-C mid-print)
        v = np.array([[float(r[c]) for c in (2, 3, 4)] for r in rows])
        tables.append(Table(np.radians(v[:, 0]), v[:, 1], v[:, 2]))
    return tables


def check_against_model(tables: list[Table], arm: ArmModel, tol: float = 0.02) -> None:
    """Refuse a transcript taken with a different model, or with the pre-#74 sign."""
    worst = (0.0, -1, -1, 0.0, 0.0)
    for t, tab in enumerate(tables):
        g = arm.gravity(tab.q)
        j = int(np.argmax(np.abs(tab.grav - g)))
        if abs(tab.grav[j] - g[j]) > worst[0]:
            worst = (abs(tab.grav[j] - g[j]), t, j, tab.grav[j], g[j])
    if worst[0] > tol:
        _, t, j, printed, ours = worst
        raise ValueError(
            f"table {t} j{j + 1}: printed grav {printed:+.3f} vs this URDF's {ours:+.3f} N*m -- "
            "the transcript was taken with a different URDF/EE frame than the one passed here"
        )
    grav = np.concatenate([t.grav for t in tables])
    tau = np.concatenate([t.tau for t in tables])
    loaded = np.abs(grav) > 1.0
    if loaded.any() and np.dot(tau[loaded], grav[loaded]) < 0:
        raise ValueError(
            "measured torque anti-correlates with gravity: reaction-signed torque, "
            "the binary predates #74 (sign normalized at the Transport boundary)"
        )


def segment(tables: list[Table], tol_deg: float = 0.05, min_tables: int = 6, settle: int = 4) -> list[Hold]:
    """Group consecutive tables whose angles stay within tol of the run's first table."""
    tol = np.radians(tol_deg)
    runs: list[list[Table]] = []
    for tab in tables:
        if runs and np.abs(wrap(tab.q - runs[-1][0].q)).max() <= tol:
            runs[-1].append(tab)
        else:
            runs.append([tab])
    holds = []
    for run in runs:
        kept = run[settle:]
        if len(kept) < min_tables:
            continue
        ref = kept[0].q
        q = ref + np.median([wrap(t.q - ref) for t in kept], axis=0)
        tau = np.array([t.tau for t in kept])
        holds.append(Hold(wrap(q), np.median(tau, axis=0), tau.std(axis=0), len(kept)))
    return holds


def match_holds(holds: list[Hold], poses: list[Pose], tol_deg: float = 1.0) -> tuple[list[Hold], list[Hold]]:
    """Label each hold with the grid pose it sits at; return (matched, unmatched)."""
    matched, unmatched = [], []
    for h in holds:
        err = [np.abs(wrap(h.q - p.q)).max() for p in poses]
        k = int(np.argmin(err))
        if err[k] <= np.radians(tol_deg):
            matched.append(Hold(h.q, h.tau, h.tau_std, h.n, poses[k].id, poses[k].set))
        else:
            unmatched.append(h)
    return matched, unmatched


_COLS = ["pose_id", "set", "n"] + [f"{p}{i}" for p in ("q", "tau", "std") for i in range(1, NUM_JOINTS + 1)]


def write_holds_csv(path, holds: list[Hold]) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(_COLS)
        for h in holds:
            w.writerow([h.pose_id, h.set, h.n] + [f"{x:.6f}" for x in np.concatenate([h.q, h.tau, h.tau_std])])


def read_holds_csv(path) -> list[Hold]:
    def vec(r, p):
        return np.array([float(r[f"{p}{i}"]) for i in range(1, NUM_JOINTS + 1)])

    with open(path, newline="") as f:
        return [
            Hold(vec(r, "q"), vec(r, "tau"), vec(r, "std"), int(r["n"]), r["pose_id"], r["set"])
            for r in csv.DictReader(f)
        ]

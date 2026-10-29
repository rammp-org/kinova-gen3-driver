"""Parity check: the C++ observer (contact_replay output) against the Python reference.

Runs the reference observer.py over the same recording and prints, per joint, the
largest |r_cpp - r_py| after the first 5/gain seconds. Exits 1 if any joint exceeds
--tol (default 0.05 N·m).

    uv run -q --no-project --with pin --with numpy python -I tools/contact/parity.py \
        --observer-py path/to/observer.py --csv in.csv --urdf arm.urdf --cpp r.csv [--gain 25] [--tau-sign -1]

in.csv is the recording contact_replay read (columns t, q1..q7, qd1..qd7, tau1..tau7,
tau in command convention, or raw core v1.2.0 with --tau-sign -1,
matching contact_replay); r.csv is contact_replay's --out.
"""

import argparse
import importlib.util
import sys

import numpy as np


def load_observer(path):
    spec = importlib.util.spec_from_file_location("observer_ref", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def columns(data, prefix):
    return np.stack([data[f"{prefix}{i}"] for i in range(1, 8)], axis=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--observer-py", required=True)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--cpp", required=True)
    ap.add_argument("--gain", type=float, default=25.0)
    ap.add_argument("--tau-sign", type=float, default=1.0)
    ap.add_argument("--tol", type=float, default=0.05)
    a = ap.parse_args()

    obs = load_observer(a.observer_py)
    rec = np.genfromtxt(a.csv, delimiter=",", names=True)
    cpp = np.genfromtxt(a.cpp, delimiter=",", names=True)
    if len(rec) != len(cpp):
        sys.exit(f"row count differs: {a.csv} has {len(rec)}, {a.cpp} has {len(cpp)}")

    t = rec["t"]
    r_py = obs.run_observer(
        obs.ArmModel(a.urdf), t, columns(rec, "q"), columns(rec, "qd"), a.tau_sign * columns(rec, "tau"), a.gain
    )
    r_cpp = columns(cpp, "r")

    keep = t - t[0] >= 5.0 / a.gain
    worst = np.abs(r_cpp - r_py)[keep].max(axis=0)
    print("max |r_cpp - r_py| [N·m] after %.2f s:" % (5.0 / a.gain))
    for j, w in enumerate(worst, 1):
        print(f"  joint {j}: {w:.4f}")
    if worst.max() > a.tol:
        print(f"FAIL: over {a.tol} N·m")
        return 1
    print(f"OK: all within {a.tol} N·m")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""gravity_id CLI. Offline only: nothing here connects to the arm.

  posegrid  URDF -> poses.csv (40 fit + 10 val, D-optimal, ordered for travel)
  parse     dry-run transcript(s) + poses.csv -> holds.csv
  fit       holds.csv -> report.md, params.json, corrected.urdf
  drift     two dry-run transcripts (before/after a torque run) -> joint and EE drift
  synth     poses.csv -> a fake dry-run transcript with injected errors (rehearsal)
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from gravity_id.fit import FAMILIES, select
from gravity_id.model import NUM_JOINTS, ArmModel
from gravity_id.posegrid import make_grid, read_poses_csv, wrap, write_poses_csv
from gravity_id.report import render
from gravity_id.synth import point_mass, true_torque
from gravity_id.transcript import (
    check_against_model,
    format_table,
    match_holds,
    parse_tables,
    read_holds_csv,
    segment,
    write_holds_csv,
)
from gravity_id.urdf_write import write_corrected_urdf


def _floats(s: str, n: int) -> list[float]:
    v = [float(x) for x in s.split(",")]
    if len(v) != n:
        raise argparse.ArgumentTypeError(f"expected {n} comma-separated numbers, got {s!r}")
    return v


def cmd_posegrid(a, arm):
    poses = make_grid(arm, a.n_fit, a.n_val, a.z_min, tuple(_floats(a.j1_range, 2)), seed=a.seed)
    write_poses_csv(a.out, poses)
    print(f"wrote {len(poses)} poses to {a.out}")


def cmd_parse(a, arm):
    tables = [t for path in a.transcripts for t in parse_tables(Path(path).read_text())]
    if not tables:
        raise SystemExit("no dry-run tables found")
    check_against_model(tables, arm)
    holds = segment(tables)
    matched, unmatched = match_holds(holds, read_poses_csv(a.poses))
    write_holds_csv(a.out, matched)
    print(f"{len(tables)} tables -> {len(holds)} holds: {len(matched)} matched grid poses, {len(unmatched)} not")
    # Repeatability: any configuration visited more than once (home at start/middle/end).
    groups: list[list] = []
    for h in holds:
        g = next((g for g in groups if np.abs(wrap(h.q - g[0].q)).max() <= np.radians(1.0)), None)
        (g.append(h) if g else groups.append([h]))
    for g in (g for g in groups if len(g) > 1):
        spread = np.ptp([h.tau for h in g], axis=0)
        print(f"repeat x{len(g)} at {np.degrees(g[0].q).round(1).tolist()} deg: "
              f"tau spread [N*m] {spread.round(3).tolist()}")


def cmd_fit(a, arm):
    holds = read_holds_csv(a.holds)
    sel = select(arm, holds)
    family = a.family or sel.chosen
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    changes = write_corrected_urdf(a.urdf, a.ee_frame, sel.fits[family].dpi, str(out / "corrected.urdf"))
    n_fit = sum(h.set == "fit" for h in holds)
    (out / "report.md").write_text(render(arm, sel, family, changes, n_fit, len(holds) - n_fit))
    fit = sel.fits[family]
    (out / "params.json").write_text(json.dumps({
        "chosen": sel.chosen, "family": family, "dpi": fit.dpi.tolist(), "std_dpi": fit.std_dpi.tolist(),
        "offsets": fit.offsets.tolist(), "std_offsets": fit.std_offsets.tolist(),
        "val_rms": {f: sel.val_rms[f].tolist() for f in FAMILIES},
    }, indent=1))
    print(f"chosen {sel.chosen}; wrote {out}/report.md, params.json, corrected.urdf")


def cmd_drift(a, arm):
    before = parse_tables(Path(a.before).read_text())[-1].q
    after = parse_tables(Path(a.after).read_text())[0].q
    dq = np.degrees(wrap(after - before))
    print("dq [deg]: " + " ".join(f"j{i + 1} {d:+.2f}" for i, d in enumerate(dq)))
    print(f"EE displacement: {np.linalg.norm(arm.ee_position(after) - arm.ee_position(before)) * 1e3:.1f} mm")


def cmd_synth(a, arm):
    rng = np.random.default_rng(a.seed)
    dpi = point_mass(7, a.payload[0], np.array(a.payload[1:])) if a.payload else None
    offsets = np.array(a.offsets) if a.offsets else np.zeros(NUM_JOINTS)
    dpi = np.zeros(4 * NUM_JOINTS) if dpi is None else dpi
    poses, here, text = read_poses_csv(a.poses), np.zeros(NUM_JOINTS), []
    for p in poses:
        for s in (0.25, 0.5, 0.75):  # moving between poses
            q = here + s * wrap(p.q - here)
            text.append(format_table(q, true_torque(arm, q, dpi, offsets), arm.gravity(q)))
        for _ in range(10):  # a 5 s hold at 2 Hz
            tau = true_torque(arm, p.q, dpi, offsets) + rng.normal(0, a.noise, NUM_JOINTS)
            text.append(format_table(p.q, tau, arm.gravity(p.q)))
        here = p.q
    Path(a.out).write_text("[dry-run] synthetic transcript (gravity_id synth)\n" + "\n".join(text))
    print(f"wrote {a.out}")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="gravity_id", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add(name, fn):
        p = sub.add_parser(name)
        p.add_argument("--urdf", required=True, help="the URDF the dry-run used (production: rendered robot.urdf)")
        p.add_argument("--ee-frame", required=True)
        p.set_defaults(fn=fn)
        return p

    p = add("posegrid", cmd_posegrid)
    p.add_argument("--out", required=True)
    p.add_argument("--n-fit", type=int, default=40)
    p.add_argument("--n-val", type=int, default=10)
    p.add_argument("--z-min", type=float, default=0.10, help="min height of every joint and the EE [m]")
    p.add_argument("--j1-range", default="-180,180", help="allowed j1 sector, deg")
    p.add_argument("--seed", type=int, default=0)

    p = add("parse", cmd_parse)
    p.add_argument("--poses", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("transcripts", nargs="+")

    p = add("fit", cmd_fit)
    p.add_argument("--holds", required=True)
    p.add_argument("--out-dir", required=True)
    p.add_argument("--family", choices=list(FAMILIES), help="force a family instead of the selected one")

    p = add("drift", cmd_drift)
    p.add_argument("--before", required=True)
    p.add_argument("--after", required=True)

    p = add("synth", cmd_synth)
    p.add_argument("--poses", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--payload", type=lambda s: _floats(s, 4), help="M,X,Y,Z: kg at m in the joint-7 frame")
    p.add_argument("--offsets", type=lambda s: _floats(s, NUM_JOINTS))
    p.add_argument("--noise", type=float, default=0.05)
    p.add_argument("--seed", type=int, default=0)

    a = ap.parse_args(argv)
    a.fn(a, ArmModel.load(a.urdf, a.ee_frame))


if __name__ == "__main__":
    main()

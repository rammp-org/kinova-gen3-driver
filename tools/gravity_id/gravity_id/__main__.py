"""gravity_id CLI. Offline only: nothing here connects to the arm.

  posegrid  URDF -> poses.csv (40 fit + 10 val, D-optimal, ordered for travel)
  parse     dry-run transcript(s) + poses.csv -> holds.csv
  fit       holds.csv -> report.md, params.json, corrected.urdf
  drift     two dry-run transcripts (before/after a torque run) -> joint and EE drift
  candle    dry-run transcript(s) of two-sided returns to candle -> torque-sensor offsets
  synth     poses.csv -> a fake dry-run transcript with injected errors (rehearsal)
  wrist     dry-run transcript(s) of j7 sweeps + the payload's weighed mass -> its COM
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
from gravity_id.wrist import HARMONICS, WRIST, fit_wrist, plan_std, sweep_poses


def _floats(s: str, n: int) -> list[float]:
    v = [float(x) for x in s.split(",")]
    if len(v) != n:
        raise argparse.ArgumentTypeError(f"expected {n} comma-separated numbers, got {s!r}")
    return v


def cmd_posegrid(a, arm):
    poses = make_grid(arm, a.n_fit, a.n_val, a.z_min, tuple(_floats(a.j1_range, 2)), a.margin, seed=a.seed)
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


def candle_holds(tables) -> tuple[list, list]:
    """Holds AT candle in one transcript, each with the side it was approached from.

    The side comes from the excursion since the previous candle stay: its furthest
    table (the +/-10 deg turnaround; the table just before arrival is often caught
    mid-return). It counts only if every joint is >= 3 deg out, all on one side. A run
    with no such excursion (the logger started at candle, or the arm arrived from
    somewhere arbitrary) has no known side and is skipped: friction biases each side
    oppositely.
    """
    at = [np.abs(wrap(t.q)).max() <= np.radians(0.5) for t in tables]
    holds, sides = [], []
    k, prev_end = 0, 0
    while k < len(tables):
        if not at[k]:
            k += 1
            continue
        j = k
        while j < len(tables) and at[j]:
            j += 1
        excursion = [wrap(t.q) for t in tables[prev_end:k]]
        if excursion:
            far = max(excursion, key=lambda q: np.abs(q).mean())
            if np.abs(far).min() >= np.radians(3.0) and abs(np.sign(far).sum()) == len(far):
                for h in segment(tables[k:j]):
                    holds.append(h)
                    sides.append(np.sign(far[0]))
        k = prev_end = j
    return holds, sides


def cmd_candle(a, arm):
    holds, sides = [], []
    for path in a.transcripts:  # per transcript: a file must not borrow another's side
        tables = parse_tables(Path(path).read_text())
        check_against_model(tables, arm)
        h, s = candle_holds(tables)
        holds += h
        sides += s
    if not (+1.0 in sides and -1.0 in sides):
        raise SystemExit(f"found {len(holds)} candle hold(s) approached from "
                         f"{sorted({'+' if s > 0 else '-' for s in sides}) or 'no known side'}; "
                         "need returns from both + and - sides")
    resid = np.array([h.tau - arm.gravity(h.q) for h in holds])  # g(candle) ~ 0: this is the offset
    sides = np.array(sides)
    plus, minus = resid[sides > 0].mean(axis=0), resid[sides < 0].mean(axis=0)
    # Balanced per side, so uneven counts (two + returns, one -) cannot tilt the offset.
    offset, friction = (plus + minus) / 2, np.abs(plus - minus) / 2
    print(f"{len(holds)} candle holds ({int((sides > 0).sum())} from +, {int((sides < 0).sum())} from -)")
    print("friction (half-spread) [N*m]: " + " ".join(f"j{i + 1} {f:.2f}" for i, f in enumerate(friction)))
    print("--torque-offset " + ",".join(f"{o:.3f}" for o in offset))


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


def cmd_wrist(a, arm):
    if a.plan:
        if not a.j6 or a.transcripts:
            raise SystemExit("--plan takes --j6 A,B,... and no transcripts")
        poses = sweep_poses(_floats(a.plan, 5), [float(x) for x in a.j6.split(",")])
        try:
            std = plan_std(arm, poses, a.mass)
        except ValueError as e:
            raise SystemExit(str(e))
        print(f"plan OK: {len(poses)} holds; expected COM std at 0.02 N*m [mm]: "
              + " ".join(f"{ax} {s * 1e3:.1f}" for ax, s in zip("xyz", std)))
        return
    if not a.transcripts:
        raise SystemExit("give dry-run transcript(s), or --plan J1,..,J5 --j6 A,B to check a sweep first")
    tables = [t for path in a.transcripts for t in parse_tables(Path(path).read_text())]
    if not tables:
        raise SystemExit("no dry-run tables found")
    check_against_model(tables, arm)
    try:
        fit = fit_wrist(arm, segment(tables), a.mass)
    except ValueError as e:
        raise SystemExit(str(e))
    print(f"{fit.n_holds} holds; residual rms {fit.rms:.3f} N*m")
    print(f"payload {a.mass:.3f} kg, COM [mm, joint-7 frame]: "
          + "  ".join(f"{ax} {c * 1e3:+.1f} +- {s * 1e3:.1f}" for ax, c, s in zip("xyz", fit.com, fit.std_com)))
    print("offsets [N*m]: " + " ".join(f"j{j + 1} {o:+.3f}" for j, o in zip(WRIST, fit.offsets)))
    print("j7 ripple [N*m]: " + " ".join(f"{n}/turn {r:.3f}" for n, r in zip(HARMONICS, fit.ripple)))
    if a.out_urdf:
        write_corrected_urdf(a.urdf, a.ee_frame, point_mass(7, a.mass, fit.com), a.out_urdf)
        print(f"wrote {a.out_urdf}")


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
    p.add_argument("--margin", type=float, default=10.0, help="stay this far inside revolute limits, deg")
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

    p = add("candle", cmd_candle)
    p.add_argument("transcripts", nargs="+")

    p = add("synth", cmd_synth)
    p.add_argument("--poses", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--payload", type=lambda s: _floats(s, 4), help="M,X,Y,Z: kg at m in the joint-7 frame")
    p.add_argument("--offsets", type=lambda s: _floats(s, NUM_JOINTS))
    p.add_argument("--noise", type=float, default=0.05)
    p.add_argument("--seed", type=int, default=0)

    p = add("wrist", cmd_wrist)
    p.add_argument("--mass", type=float, required=True, help="the added payload's mass from a scale, kg")
    p.add_argument("--out-urdf", help="write the URDF with the payload added to body 7")
    p.add_argument("--plan", metavar="J1,..,J5", help="check a wrist_sweep.sh plan at this pose (deg); moves nothing")
    p.add_argument("--j6", default="", metavar="A,B,...", help="with --plan: the sweep's j6 angles (deg; --j6=-30,30 if the first is negative)")
    p.add_argument("transcripts", nargs="*")

    a = ap.parse_args(argv)
    a.fn(a, ArmModel.load(a.urdf, a.ee_frame))


if __name__ == "__main__":
    main()

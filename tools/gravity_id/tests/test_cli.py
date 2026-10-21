import re
import subprocess
import sys

import numpy as np

from conftest import FIXTURE_EE, FIXTURE_URDF
from gravity_id.model import ArmModel
from gravity_id.transcript import format_table


def run(*args):
    cmd = [sys.executable, "-m", "gravity_id", args[0], "--urdf", FIXTURE_URDF, "--ee-frame", FIXTURE_EE, *args[1:]]
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


def test_end_to_end_synthetic(tmp_path):
    poses, tr, holds, out = (tmp_path / n for n in ("poses.csv", "t.txt", "holds.csv", "fit"))
    run("posegrid", "--out", str(poses))
    run("synth", "--poses", str(poses), "--out", str(tr), "--payload", "0.25,0.02,0,0.12")
    assert "50 matched" in run("parse", "--poses", str(poses), "--out", str(holds), str(tr))
    run("fit", "--holds", str(holds), "--out-dir", str(out))
    report = (out / "report.md").read_text()
    assert "Chosen family: F2" in report
    assert "Per-joint limits: PASS" in report
    ArmModel.load(str(out / "corrected.urdf"), FIXTURE_EE)


def test_drift_readout(arm, tmp_path):
    q = np.radians([0.0, 20.0, 0.0, 90.0, 0.0, 40.0, 0.0])
    q2 = q + np.radians([0, 0.5, 0, 0, 0, 0, 0])
    (tmp_path / "a.txt").write_text(format_table(q, arm.gravity(q), arm.gravity(q)))
    (tmp_path / "b.txt").write_text(format_table(q2, arm.gravity(q2), arm.gravity(q2)))
    out = run("drift", "--before", str(tmp_path / "a.txt"), "--after", str(tmp_path / "b.txt"))
    assert "j2 +0.50" in out


def test_gate_judges_the_urdf_not_the_offsets(tmp_path):
    # A 2.5 N*m j3 offset is fitted perfectly, but corrected.urdf cannot carry it: the gate
    # must FAIL and say why, not pass on residuals that include the offset.
    poses, tr, holds, out = (tmp_path / n for n in ("poses.csv", "t.txt", "holds.csv", "fit"))
    run("posegrid", "--out", str(poses))
    run("synth", "--poses", str(poses), "--out", str(tr), "--payload", "0.25,0.02,0,0.12",
        "--offsets", "0,0.4,2.5,0.3,0,0,0")
    run("parse", "--poses", str(poses), "--out", str(holds), str(tr))
    run("fit", "--holds", str(holds), "--out-dir", str(out))
    report = (out / "report.md").read_text()
    assert "WARNING: offsets above 0.3" in report and "j3 (+2.5" in report
    assert "Per-joint limits: FAIL" in report


def test_candle_offsets(arm, tmp_path):
    # Two returns to candle, friction +-0.5 on j2 around a +1.0 offset: offset = mean.
    q0 = np.zeros(7)
    off = np.array([-0.2, 1.0, 1.9, 0.5, -0.1, 0.0, 0.2])
    fric = np.array([0, 0.5, 0.15, 0.15, 0, 0, 0])
    away = np.radians([10.0] * 7)
    text = []
    for sign in (+1, -1):
        text.append(format_table(sign * away, arm.gravity(sign * away), arm.gravity(sign * away)))
        text += [format_table(q0, arm.gravity(q0) + off + sign * fric, arm.gravity(q0))] * 10
    (tmp_path / "c.txt").write_text("\n".join(text))
    out = run("candle", str(tmp_path / "c.txt"))
    assert "2 candle holds (1 from +, 1 from -)" in out
    got = [float(x) for x in out.split("--torque-offset ")[1].split()[0].split(",")]
    assert np.allclose(got, off, atol=0.01)  # the transcript carries 0.01 N*m
    assert "j2 0.50" in out


def test_candle_refuses_one_sided(arm, tmp_path):
    q0, away = np.zeros(7), np.radians([10.0] * 7)
    text = []
    for _ in range(2):  # two returns, both from +10 deg
        text.append(format_table(away, arm.gravity(away), arm.gravity(away)))
        text += [format_table(q0, arm.gravity(q0), arm.gravity(q0))] * 10
    (tmp_path / "c.txt").write_text("\n".join(text))
    r = subprocess.run([sys.executable, "-m", "gravity_id", "candle", "--urdf", FIXTURE_URDF,
                        "--ee-frame", FIXTURE_EE, str(tmp_path / "c.txt")], capture_output=True, text=True)
    assert r.returncode != 0 and "both + and - sides" in r.stderr


def _candle_text(arm, sides, off, fric, lead_at_candle=False):
    q0, away = np.zeros(7), np.radians([10.0] * 7)
    text = [format_table(q0, arm.gravity(q0) + off, arm.gravity(q0))] * 10 if lead_at_candle else []
    for sign in sides:
        text.append(format_table(sign * away, arm.gravity(sign * away), arm.gravity(sign * away)))
        text += [format_table(q0, arm.gravity(q0) + off + sign * fric, arm.gravity(q0))] * 10
    return "\n".join(text)


def test_candle_balances_uneven_sides(arm, tmp_path):
    # Two returns from +, one from -: a plain mean would tilt toward +.
    off = np.array([-0.2, 1.0, 1.9, 0.5, -0.1, 0.0, 0.2])
    fric = np.array([0, 0.5, 0.15, 0.15, 0, 0, 0])
    (tmp_path / "c.txt").write_text(_candle_text(arm, [+1, +1, -1], off, fric, lead_at_candle=True))
    out = run("candle", str(tmp_path / "c.txt"))
    assert "3 candle holds (2 from +, 1 from -)" in out  # the leading candle stay has no side
    got = [float(x) for x in out.split("--torque-offset ")[1].split()[0].split(",")]
    assert np.allclose(got, off, atol=0.01)


def test_candle_sides_are_per_transcript(arm, tmp_path):
    # File b starts AT candle: its first stay must not inherit file a's last excursion.
    off = np.array([0.0, 1.0, 0, 0, 0, 0, 0])
    fric = np.array([0, 0.5, 0, 0, 0, 0, 0])
    (tmp_path / "a.txt").write_text(_candle_text(arm, [+1], off, fric))
    (tmp_path / "b.txt").write_text(_candle_text(arm, [-1], off, fric, lead_at_candle=True))
    out = run("candle", str(tmp_path / "a.txt"), str(tmp_path / "b.txt"))
    assert "2 candle holds (1 from +, 1 from -)" in out


def test_candle_side_from_the_turnaround_not_the_last_table(arm, tmp_path):
    # As on dojo: the table just before arrival is caught mid-return (1.5 deg), below the
    # 3 deg excursion threshold; the side must come from the +/-10 deg turnaround.
    q0 = np.zeros(7)
    text = []
    for sign in (+1, -1):
        for deg in (10.0, 1.5):
            q = np.radians([sign * deg] * 7)
            text.append(format_table(q, arm.gravity(q), arm.gravity(q)))
        text += [format_table(q0, arm.gravity(q0), arm.gravity(q0))] * 10
    (tmp_path / "c.txt").write_text("\n".join(text))
    assert "2 candle holds (1 from +, 1 from -)" in run("candle", str(tmp_path / "c.txt"))


def test_wrist_end_to_end(arm, tmp_path):
    from test_wrist import COM, MASS, OFFSETS, ripple, sweep
    from gravity_id.synth import point_mass, true_torque

    # Sensor noise matters here: the dry-run prints torque to 0.01 N*m, and without noise
    # dithering it a hold's median sits on that grid (~5 mm of COM bias along z).
    rng, tables = np.random.default_rng(0), []
    for p in sweep((30, 90)):
        tau = true_torque(arm, p.q, point_mass(7, MASS, COM), OFFSETS)
        tau[6] += ripple(p.q[6])
        tables += [format_table(p.q, tau + rng.normal(0, 0.01, 7), arm.gravity(p.q)) for _ in range(12)]
    (tmp_path / "t.txt").write_text("\n".join(tables))
    out = run("wrist", "--mass", str(MASS), "--out-urdf", str(tmp_path / "w.urdf"), str(tmp_path / "t.txt"))
    assert "24 holds" in out
    com = [float(v) for v in re.findall(r"[xyz] ([+-][\d.]+) \+-", out)]
    assert np.abs(np.array(com) - COM * 1e3).max() <= 5.0  # mm
    assert re.search(r"j7 \+0\.3[34]\d", out)
    added = ArmModel.load(str(tmp_path / "w.urdf"), FIXTURE_EE).params() - arm.params()
    # the URDF carries the FITTED COM (printed to 0.1 mm)
    assert np.allclose(added[24:28], point_mass(7, MASS, np.array(com) / 1e3)[24:28], atol=2e-5)

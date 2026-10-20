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

import numpy as np
import pinocchio as pin
import pytest

from conftest import DATA, FIXTURE_EE, FIXTURE_URDF
from gravity_id.model import ArmModel
from gravity_id.posegrid import Pose
from gravity_id.transcript import (
    check_against_model,
    format_table,
    match_holds,
    parse_tables,
    read_holds_csv,
    segment,
    write_holds_csv,
)

Q_A = np.radians([10.0, 30.0, -20.0, 80.0, 5.0, 40.0, -60.0])
Q_B = np.radians([-40.0, -20.0, 50.0, -70.0, 90.0, -30.0, 120.0])
Q_C = np.radians([0.0, 15.0, 0.0, 100.0, 0.0, -50.0, 0.0])


def _transcript(arm, qs_per_table, tau_fn=None):
    out = ["[dry-run] READ-ONLY gravity check", "[model] mount tilt 0.0 deg from vertical"]
    for q in qs_per_table:
        g = arm.gravity(q)
        out.append(format_table(q, g if tau_fn is None else tau_fn(g), g))
        out.append("  -> max|residual| = 0.000 Nm  (large/growing toward the tip => URDF payload mismatch)\n")
    return "\n".join(out)


def _three_holds(arm):
    moving = [Q_A + (Q_B - Q_A) * s for s in (0.2, 0.4, 0.6, 0.8)]
    moving2 = [Q_B + (Q_C - Q_B) * s for s in (0.2, 0.4, 0.6, 0.8)]
    return [Q_A] * 10 + moving + [Q_B] * 10 + moving2 + [Q_C] * 10


def test_parse_real_dry_run_output():
    tabs = parse_tables((DATA / "sim_dry_run.txt").read_text())
    assert len(tabs) >= 2
    assert np.allclose(tabs[0].grav, [-0.0, -0.01, -0.0, -0.01, -0.0, -0.02, -0.0])


def test_parse_roundtrip(arm):
    tabs = parse_tables(_transcript(arm, [Q_A]))
    assert len(tabs) == 1
    assert np.abs(np.degrees(tabs[0].q - Q_A)).max() <= 0.005 + 1e-9
    assert np.abs(tabs[0].grav - arm.gravity(Q_A)).max() <= 0.005 + 1e-9


def test_parse_ignores_noise_lines(arm):
    text = _transcript(arm, [Q_A, Q_B])
    assert len(parse_tables("garbage\n" + text + "\n[dry-run] done")) == 2


def test_parse_skips_truncated_table(arm):
    text = _transcript(arm, [Q_A]) + "\n" + format_table(Q_B, arm.gravity(Q_B), arm.gravity(Q_B))[:120]
    assert len(parse_tables(text)) == 1


def test_segment_finds_holds(arm):
    holds = segment(parse_tables(_transcript(arm, _three_holds(arm))))
    assert len(holds) == 3
    assert [h.n for h in holds] == [8, 8, 8]  # 10 tables minus settle=2
    assert np.abs(np.degrees(holds[1].q - Q_B)).max() < 0.01


def test_segment_wraps_continuous_joints(arm):
    q1 = Q_A.copy()
    q1[6] = np.radians(179.99)
    q2 = q1.copy()
    q2[6] = np.radians(-179.99)
    holds = segment(parse_tables(_transcript(arm, [q1, q2] * 5)))
    assert len(holds) == 1


def test_parse_rejects_wrong_urdf(arm):
    full = pin.buildModelFromUrdf(FIXTURE_URDF)
    for j in range(1, full.njoints):  # strip the tip: every body past joint 7 loses its mass
        if full.names[j] == "gen3_joint_7":
            full.inertias[j] = pin.Inertia.Zero()
    other = ArmModel.from_model(full, FIXTURE_EE)
    with pytest.raises(ValueError, match="grav"):
        check_against_model(parse_tables(_transcript(other, [Q_A, Q_B])), arm)


def test_parse_rejects_reaction_signed(arm):
    tabs = parse_tables(_transcript(arm, [Q_A, Q_B], tau_fn=lambda g: -g))
    with pytest.raises(ValueError, match="#74"):
        check_against_model(tabs, arm)


def test_check_accepts_matching_transcript(arm):
    check_against_model(parse_tables(_transcript(arm, [Q_A, Q_B, Q_C])), arm)


def test_match_holds_assigns_sets(arm, tmp_path):
    holds = segment(parse_tables(_transcript(arm, _three_holds(arm))))
    poses = [Pose("P00", "fit", Q_A), Pose("P01", "val", Q_C)]
    matched, unmatched = match_holds(holds, poses)
    assert [(h.pose_id, h.set) for h in matched] == [("P00", "fit"), ("P01", "val")]
    assert len(unmatched) == 1
    write_holds_csv(tmp_path / "h.csv", matched)
    back = read_holds_csv(tmp_path / "h.csv")
    assert [h.pose_id for h in back] == ["P00", "P01"]
    assert np.allclose(back[0].tau, matched[0].tau, atol=1e-6)

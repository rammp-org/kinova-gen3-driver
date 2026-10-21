import numpy as np
import pytest

from gravity_id.posegrid import Pose
from gravity_id.synth import make_holds, point_mass
from gravity_id.wrist import fit_wrist, plan_std, sweep_poses

BASE = np.radians([0.0, 30.0, 180.0, -90.0, 0.0, 30.0, 0.0])
MASS, COM = 0.15, np.array([0.03, -0.02, 0.05])  # a wrist camera, joint-7 frame
OFFSETS = np.array([0, 0, 0, 0, -0.1, -0.04, 0.34])


def sweep(j6s):
    """j7 through a full turn in 30 deg steps at each j6 angle."""
    out = []
    for j6 in j6s:
        for t in range(0, 360, 30):
            q = BASE.copy()
            q[5], q[6] = np.radians(j6), np.radians(t)
            out.append(Pose(f"j6={j6},j7={t}", "fit", q))
    return out


def ripple(q7):
    """dojo's j7 gearbox: 0.165 N*m at 3 per turn, plus a little at 1 per turn."""
    return 0.165 * np.sin(3 * q7 + 0.4) + 0.03 * np.cos(q7)


def wrist_holds(arm, j6s, noise=0.01):
    holds = make_holds(arm, sweep(j6s), point_mass(7, MASS, COM), OFFSETS, noise)
    for h in holds:
        h.tau[6] += ripple(h.q[6])
    return holds


def test_recovers_camera_com_offsets_and_ripple(arm):
    fit = fit_wrist(arm, wrist_holds(arm, (30, 90)), MASS)
    assert np.abs(fit.com - COM).max() <= 0.005
    assert np.all(fit.std_com < 0.01)  # along the tool axis it is the weakest, ~6 mm
    assert np.abs(fit.offsets - OFFSETS[4:]).max() <= 0.01
    assert fit.ripple[2] == pytest.approx(0.165, abs=0.01)
    assert fit.ripple[0] == pytest.approx(0.03, abs=0.01)
    assert fit.rms <= 0.02


def test_ripple_is_not_read_as_an_off_axis_mass(arm):
    # The payload sits ON the j7 axis; j7's gearbox ripple must not push it off.
    on_axis = np.array([0.0, 0.0, 0.05])
    holds = make_holds(arm, sweep((30, 90)), point_mass(7, MASS, on_axis), OFFSETS, 0.002)
    for h in holds:
        h.tau[6] += ripple(h.q[6])
    fit = fit_wrist(arm, holds, MASS)
    assert np.abs(fit.com[:2]).max() <= 0.001  # a leaked 1-per-turn ripple would be ~20 mm


@pytest.mark.parametrize("j6s", [(30,), (30, 35)])
def test_too_few_or_too_close_j6_angles_are_refused(arm, j6s):
    with pytest.raises(ValueError, match="two or more j6 angles"):
        fit_wrist(arm, wrist_holds(arm, j6s), MASS)


def test_j6_friction_two_sided_cancels_one_sided_does_not(arm):
    # j6 friction of 0.1 N*m, signed by approach side.
    def with_bias(sign_of_sweep):
        holds = []
        for j6, sweep_sign in ((30, +1), (90, -1)):
            for side in (+1, -1):
                hs = wrist_holds(arm, (j6,))
                for h in hs:
                    h.tau[5] += 0.1 * sign_of_sweep(sweep_sign, side)
                holds += hs
        return fit_wrist(arm, holds, MASS)

    two_sided = with_bias(lambda sweep, side: side)  # what wrist_sweep.sh does
    one_sided = with_bias(lambda sweep, side: sweep)  # each j6 angle reached one way
    assert np.abs(two_sided.com - COM).max() <= 0.005
    assert abs(one_sided.com[2] - COM[2]) >= 0.05


def test_plan_refuses_mirrored_j6_angles_however_far_apart(arm):
    base = [0.0, 30.0, 180.0, -90.0, 0.0]
    assert np.all(plan_std(arm, sweep_poses(base, [30, 90]), MASS) < 0.02)
    with pytest.raises(ValueError, match="mirror images"):
        plan_std(arm, sweep_poses(base, [0, 60]), MASS)  # 60 deg apart, still singular here

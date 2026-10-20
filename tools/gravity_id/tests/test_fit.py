import numpy as np
import pytest

from gravity_id.fit import fit_family, select
from gravity_id.posegrid import make_grid
from gravity_id.synth import make_holds, point_mass, scale_body

SIGMA = 0.05
OFFSETS = np.array([0.1, -0.4, 0.3, 0.2, -0.1, 0.05, 0.02])


@pytest.fixture(scope="module")
def poses(arm):
    return make_grid(arm, n_candidates=1500)


def test_recovers_tip_payload(arm, poses):
    com = np.array([0.02, 0.0, 0.12])
    sel = select(arm, make_holds(arm, poses, point_mass(7, 0.25, com), OFFSETS, SIGMA))
    assert sel.chosen == "F2"
    fit = sel.fits["F2"]
    dm, dh = fit.dpi[24], fit.dpi[25:28]
    assert abs(dm - 0.25) <= 0.02
    assert np.linalg.norm(dh / dm - com) <= 0.01
    assert np.abs(fit.offsets - OFFSETS).max() <= 0.05


def test_detects_proximal_link_error(arm, poses):
    # A 15% heavy body 3 is out of reach of the tip-only family, and is reproduced to
    # the noise floor by a distal family. Static gravity cannot say WHICH body is
    # heavy -- F3 (bodies 5-7) reproduces it as well as F4 -- only that a correction
    # beyond the tip is needed.
    sel = select(arm, make_holds(arm, poses, scale_body(arm, 3, 1.15), None, SIGMA))
    assert sel.chosen in {"F3", "F4"}
    assert np.all(sel.val_rms[sel.chosen] <= 2 * SIGMA)
    assert max(sel.val_rms["F2"][1], sel.val_rms["F2"][3]) > 2 * SIGMA


def test_tip_family_can_mimic_a_wrist_link(arm, poses):
    # The flip side, pinned so nobody reads "F2 chosen" as proof of a tip payload: a
    # 15% heavy body 5 is reproduced by the tip-only family. Attribution to physical
    # parts comes from weighing them (D3), not from the fit.
    sel = select(arm, make_holds(arm, poses, scale_body(arm, 5, 1.15), None, SIGMA))
    assert sel.chosen == "F2"
    assert np.all(sel.val_rms["F2"] <= 2 * SIGMA)


def test_no_error_no_phantom(arm, poses):
    sel = select(arm, make_holds(arm, poses, None, None, SIGMA))
    assert sel.chosen in {"F0", "F1"}


def test_rejects_unphysical(arm, poses):
    holds = make_holds(arm, poses, point_mass(7, -2.0, np.zeros(3)), None, SIGMA)
    fit = fit_family(arm, [h for h in holds if h.set == "fit"], "F2")
    assert fit.rejected is not None and "mass" in fit.rejected
    assert select(arm, holds).chosen != "F2"


def test_fit_requires_val_poses(arm, poses):
    holds = [h for h in make_holds(arm, poses) if h.set == "fit"]
    with pytest.raises(ValueError, match="val"):
        select(arm, holds)


def test_fit_requires_enough_fit_poses(arm, poses):
    holds = make_holds(arm, poses)
    few = [h for h in holds if h.set == "fit"][:5] + [h for h in holds if h.set == "val"]
    with pytest.raises(ValueError, match="fit holds"):
        select(arm, few)

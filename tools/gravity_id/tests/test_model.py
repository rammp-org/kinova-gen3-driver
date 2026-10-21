import numpy as np
import pytest

from conftest import FIXTURE_URDF
from gravity_id.model import ArmModel


def test_chain_is_seven_revolute(arm):
    assert arm.joint_names == [f"gen3_joint_{i}" for i in range(1, 8)]


def test_regressor_reproduces_gravity(arm):
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 7)
        assert np.abs(arm.regressor(q) @ arm.params() - arm.gravity(q)).max() < 1e-10


def test_gravity_matches_dry_run_print(arm):
    # From tests/data/sim_dry_run.txt: the C++ Dynamics via
    # `benchmark_grav_comp --sim --dry-run` at q = 0 with this URDF. With sim tau = 0
    # the printed resid (3 decimals) is -g(q).
    g_cpp = -np.array([0.000, 0.014, 0.000, 0.015, 0.000, 0.015, 0.000])
    assert np.abs(arm.gravity(np.zeros(7)) - g_cpp).max() < 0.002


def test_body7_lumps_gripper(arm):
    # bracelet 0.364 + 2F-85 0.920864 + camera 0.072 + camera mount 0.5
    assert arm.params()[24] == pytest.approx(1.856864, abs=1e-5)


def test_continuous_packing(arm):
    q = np.array([0.3, 0.4, -1.0, 1.2, 0.5, -0.7, 2.0])
    for k in np.flatnonzero(arm.continuous):
        dq = np.zeros(7)
        dq[k] = 2 * np.pi
        assert np.allclose(arm.gravity(q), arm.gravity(q + dq), atol=1e-12)


def test_missing_ee_frame_fails_loud():
    with pytest.raises(ValueError, match="not in URDF"):
        ArmModel.load(FIXTURE_URDF, "no_such_frame")

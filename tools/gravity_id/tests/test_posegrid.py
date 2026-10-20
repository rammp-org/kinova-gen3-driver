import numpy as np

from gravity_id.design import ALL_BODIES, design
from gravity_id.posegrid import make_grid, read_poses_csv, sample_candidates, write_poses_csv


def _stack(arm, qs):
    return np.vstack([design(arm, q, ALL_BODIES, True) for q in qs])


def _logdet(arm, qs, V):
    A = _stack(arm, qs) @ V
    return np.linalg.slogdet(A.T @ A)[1]


def test_grid_sizes_and_sets(arm):
    poses = make_grid(arm, n_candidates=600)
    assert sum(p.set == "fit" for p in poses) == 40
    assert sum(p.set == "val" for p in poses) == 10
    assert len({p.id for p in poses}) == 50


def test_grid_respects_limits_and_height(arm):
    margin = np.radians(10.0)
    for p in make_grid(arm, n_candidates=600, z_min=0.10):
        assert np.all(p.q >= arm.lower + margin - 1e-9) and np.all(p.q <= arm.upper - margin + 1e-9)
        assert arm.frame_heights(p.q).min() >= 0.10


def test_grid_respects_j1_range(arm):
    for p in make_grid(arm, n_candidates=600, j1_range_deg=(-30.0, 60.0)):
        assert -30.0 <= np.degrees(p.q[0]) <= 60.0


def test_grid_full_rank_for_f4(arm):
    cand = sample_candidates(arm, 1500, 0.10, (-180.0, 180.0), 10.0, np.random.default_rng(1))
    s = np.linalg.svd(_stack(arm, cand), compute_uv=False)
    r_all = int(np.sum(s > 1e-6 * s[0]))
    fit = [p.q for p in make_grid(arm, n_candidates=600) if p.set == "fit"]
    s_fit = np.linalg.svd(_stack(arm, fit), compute_uv=False)
    assert int(np.sum(s_fit > 1e-6 * s_fit[0])) == r_all


def test_grid_beats_random(arm):
    rng = np.random.default_rng(2)
    cand = sample_candidates(arm, 600, 0.10, (-180.0, 180.0), 10.0, np.random.default_rng(0))
    _, s, Vt = np.linalg.svd(_stack(arm, cand), full_matrices=False)
    V = Vt[s > 1e-6 * s[0]].T
    fit = [p.q for p in make_grid(arm, n_candidates=600) if p.set == "fit"]
    rand = [_logdet(arm, [cand[i] for i in rng.choice(len(cand), 40, replace=False)], V) for _ in range(20)]
    assert _logdet(arm, fit, V) > np.median(rand)


def test_csv_roundtrip(arm, tmp_path):
    poses = make_grid(arm, n_candidates=300)
    path = tmp_path / "poses.csv"
    write_poses_csv(path, poses)
    back = read_poses_csv(path)
    assert [p.id for p in back] == [p.id for p in poses]
    assert [p.set for p in back] == [p.set for p in poses]
    for a, b in zip(poses, back):
        d = np.angle(np.exp(1j * (a.q - b.q)))  # wrapped difference
        assert np.abs(d).max() < np.radians(0.01)

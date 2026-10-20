import xml.etree.ElementTree as ET

import numpy as np
import pytest

from conftest import FIXTURE_EE, FIXTURE_URDF
from gravity_id.model import ArmModel
from gravity_id.synth import point_mass, scale_body
from gravity_id.urdf_write import write_corrected_urdf


def _link(path, name):
    return next(l for l in ET.parse(path).getroot().iter("link") if l.get("name") == name)


def test_roundtrip_matches_fit(arm, tmp_path):
    dpi = scale_body(arm, 3, 1.2) + point_mass(7, -0.3, np.array([0.01, -0.02, 0.05]))
    out = tmp_path / "corrected.urdf"
    write_corrected_urdf(FIXTURE_URDF, FIXTURE_EE, dpi, str(out))
    new = ArmModel.load(str(out), FIXTURE_EE)
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 7)
        assert np.abs(new.gravity(q) - (arm.gravity(q) + arm.regressor(q) @ dpi)).max() < 1e-9


def test_write_targets_heaviest_link(arm, tmp_path):
    out = tmp_path / "corrected.urdf"
    changes = write_corrected_urdf(FIXTURE_URDF, FIXTURE_EE, point_mass(7, 0.1, np.zeros(3)), str(out))
    assert [c.link for c in changes] == ["gen3_robotiq_85_base_link"]
    assert changes[0].mass_new == pytest.approx(changes[0].mass_old + 0.1)


def test_keeps_rotational_inertia(arm, tmp_path):
    out = tmp_path / "corrected.urdf"
    write_corrected_urdf(FIXTURE_URDF, FIXTURE_EE, scale_body(arm, 4, 1.1), str(out))
    before = _link(FIXTURE_URDF, "gen3_forearm_link").find("inertial")
    after = _link(out, "gen3_forearm_link").find("inertial")
    assert ET.tostring(after.find("inertia")) == ET.tostring(before.find("inertia"))
    assert after.find("origin").get("rpy") == before.find("origin").get("rpy")
    assert after.find("mass").get("value") != before.find("mass").get("value")


def test_negative_mass_refused(arm, tmp_path):
    with pytest.raises(ValueError, match="mass"):
        write_corrected_urdf(FIXTURE_URDF, FIXTURE_EE, point_mass(7, -1.0, np.zeros(3)), str(tmp_path / "x.urdf"))


def test_untouched_bodies_unchanged(arm, tmp_path):
    out = tmp_path / "corrected.urdf"
    write_corrected_urdf(FIXTURE_URDF, FIXTURE_EE, point_mass(7, 0.1, np.zeros(3)), str(out))
    for name in ("gen3_shoulder_link", "gen3_forearm_link", "wrist_camera_mount_link"):
        assert ET.tostring(_link(out, name)) == ET.tostring(_link(FIXTURE_URDF, name))

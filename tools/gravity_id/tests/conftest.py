from pathlib import Path

import pytest

from gravity_id.model import ArmModel

REPO = Path(__file__).resolve().parents[3]
FIXTURE_URDF = str(REPO / "models" / "gen3_7dof_2f85.urdf")
FIXTURE_EE = "gen3_end_effector_link"
DATA = Path(__file__).resolve().parent / "data"


@pytest.fixture(scope="session")
def arm() -> ArmModel:
    return ArmModel.load(FIXTURE_URDF, FIXTURE_EE)

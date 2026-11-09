import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from hwil import scenario as scn  # noqa: E402
from hwil.run import run_scenario  # noqa: E402

SCENARIOS = ROOT / "scenarios"


@pytest.fixture(scope="session")
def reference_run(tmp_path_factory):
    """One builtin-controller run of the M1 reference scenario, shared by tests."""
    sc = scn.load(SCENARIOS / "takeoff_hold_land.yaml")
    out = tmp_path_factory.mktemp("runs")
    run_dir, man = run_scenario(sc, {"plots": True}, out, scenario_file="scenarios/takeoff_hold_land.yaml")
    return run_dir, man

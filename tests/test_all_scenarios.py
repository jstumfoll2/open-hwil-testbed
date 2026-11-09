"""Every scenario in scenarios/ must pass its own requirements with the builtin controller."""

import pytest

from hwil import scenario as scn
from hwil.run import run_scenario

from conftest import SCENARIOS

FILES = sorted(p for p in SCENARIOS.glob("*.yaml") if p.stem != "takeoff_hold_land")  # that one has its own tests


@pytest.mark.parametrize("path", FILES, ids=[p.stem for p in FILES])
def test_scenario_requirements(path, tmp_path):
    sc = scn.load(path)
    assert sc["requirements"], f"{path.name} has no requirements"
    _, man = run_scenario(sc, {"plots": False}, tmp_path)
    res = man["results"]
    assert not res["aborted"], res["aborted"]
    bad = [r for r in res["requirements"] if not r["passed"]]
    assert not bad, bad

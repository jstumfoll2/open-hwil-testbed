import pytest

from hwil import scenario as scn

from conftest import SCENARIOS


def test_all_scenario_files_validate():
    files = sorted(SCENARIOS.glob("*.yaml"))
    assert files
    for f in files:
        scn.load(f)


def test_defaults_are_merged():
    sc = scn.resolve({"name": "x"})
    assert sc["sim"]["rate_hz"] == 250
    assert sc["vehicle"]["model"] == "F450-hil"


def test_config_hash_is_order_independent():
    a = scn.resolve({"name": "x", "seed": 1, "wind": {"north_mps": 1.0, "east_mps": 2.0}})
    b = scn.resolve({"wind": {"east_mps": 2.0, "north_mps": 1.0}, "seed": 1, "name": "x"})
    assert scn.config_hash(a) == scn.config_hash(b)


@pytest.mark.parametrize("bad", [
    {"bogus": 1},
    {"sim": {"rate_hz": 5}},
    {"mission": [{"type": "fly_somewhere"}]},
    {"mission": [{"type": "takeoff", "alt_m": 500}]},
    {"faults": [{"type": "unknown"}]},
    {"requirements": [{"id": "R1", "text": "no spec"}]},
])
def test_bad_scenarios_rejected(bad):
    with pytest.raises(scn.ScenarioError):
        scn.resolve(bad)

import pytest

from hwil import scenario as scn

from conftest import SCENARIOS


def test_all_scenario_files_validate():
    files = sorted(SCENARIOS.glob("*.yaml"))
    assert len(files) >= 10, "M2 asks for at least 10 scenarios"
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
    {"mission": [{"type": "goto", "north_m": 5}]},
    {"mission": [{"type": "takeoff", "alt_m": 10}, {"type": "goto", "north_m": 5000}]},
    {"mission": [{"type": "takeoff", "alt_m": 10}, {"type": "takeoff", "alt_m": 20}]},
    {"mission": [{"type": "takeoff", "alt_m": 10}, {"type": "goto", "north_m": 5, "speed_mps": 50}]},
])
def test_bad_scenarios_rejected(bad):
    with pytest.raises(scn.ScenarioError):
        scn.resolve(bad)


def test_requirement_ids_are_unique():
    seen = {}
    for f in sorted(SCENARIOS.glob("*.yaml")):
        for r in scn.load(f)["requirements"]:
            assert r["id"] not in seen, f"{r['id']} used in {seen.get(r['id'])} and {f.name}"
            seen[r["id"]] = f.name

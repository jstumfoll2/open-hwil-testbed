"""Every run carries a manifest, and a manifest reproduces its run."""

import json

from hwil import scenario as scn
from hwil.run import reproduce, run_scenario

SHORT = {
    "name": "short_hop",
    "seed": 7,
    "sim": {"duration_s": 40},
    "wind": {"north_mps": 2.0, "turbulence_severity": 1},
    "mission": [{"type": "takeoff", "alt_m": 5.0}, {"type": "hold", "duration_s": 3.0}, {"type": "land"}],
}


def test_manifest_fields(tmp_path):
    run_dir, man = run_scenario(scn.resolve(SHORT), {"plots": False}, tmp_path)
    on_disk = json.loads((run_dir / "manifest.json").read_text())
    assert on_disk["config_hash"] == man["config_hash"]
    for key in ("git", "versions", "platform", "seed", "scenario", "options", "outputs", "results"):
        assert key in on_disk, key
    assert on_disk["seed"] == 7
    assert set(on_disk["git"]) == {"commit", "branch", "dirty"}
    assert on_disk["versions"]["jsbsim"]
    assert len(on_disk["outputs"]["log.csv"]) == 64
    assert "hostname" not in json.dumps(on_disk["platform"])


def test_reproduce_is_bit_identical(tmp_path):
    run_dir, _ = run_scenario(scn.resolve(SHORT), {"plots": False}, tmp_path)
    _, _, cmp = reproduce(run_dir / "manifest.json", tmp_path)
    assert cmp["config_hash_match"]
    assert cmp["log_match"], "same manifest gave a different log"


def test_seed_changes_result(tmp_path):
    a_dir, a = run_scenario(scn.resolve(SHORT), {"plots": False}, tmp_path / "a")
    b_dir, b = run_scenario(scn.resolve({**SHORT, "seed": 8}), {"plots": False}, tmp_path / "b")
    assert a["config_hash"] != b["config_hash"]
    assert a["outputs"]["log.csv"] != b["outputs"]["log.csv"]

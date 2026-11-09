"""Monte Carlo batches: deterministic dispersions, statistics, replayable failures."""

import json
from pathlib import Path

import pytest

from hwil.montecarlo import MonteCarloError, load_config, make_cases, run_montecarlo, set_path, wilson
from hwil.run import reproduce

from conftest import ROOT

MC = ROOT / "montecarlo"


def test_configs_load_and_cases_are_deterministic():
    for path in sorted(MC.glob("*.yaml")):
        cfg = load_config(path)
        cfg["runs"] = 5
        a, b = make_cases(cfg), make_cases(cfg)
        assert [v for _, v in a] == [v for _, v in b]
        assert len({sc["seed"] for sc, _ in a}) == 5


def test_dispersions_land_in_the_scenario():
    cfg = load_config(MC / "motor_loss_sweep.yaml")
    cfg["runs"] = 3
    for sc, vals in make_cases(cfg):
        assert sc["faults"][0]["factor"] == vals["faults[0].factor"]
        assert 0.35 <= vals["faults[0].factor"] <= 1.0
        assert sc["wind"]["north_mps"] == vals["wind.north_mps"]


def test_set_path_rejects_unknown_keys():
    with pytest.raises(MonteCarloError):
        set_path({"wind": {"north_mps": 0}}, "wind.nrth_mps", 1.0)


def test_wilson_interval():
    lo, hi = wilson(95, 100)
    assert 0.88 < lo < 0.9 and 0.97 < hi < 0.99
    assert wilson(0, 0) == (0.0, 1.0)


def test_small_batch_and_failure_replay(tmp_path):
    cfg_path = tmp_path / "mc.yaml"
    cfg_path.write_text(
        f"scenario: {MC.parent / 'scenarios' / 'motor_degraded.yaml'}\n"
        "runs: 6\nseed: 7\n"
        "dispersions:\n"
        "  faults[0].factor: {dist: uniform, low: 0.3, high: 0.4}\n")   # mostly failing on purpose
    mc_dir, summary = run_montecarlo(cfg_path, tmp_path / "out", jobs=2)
    assert summary["runs"] == 6 and summary["errors"] == 0
    assert (mc_dir / "montecarlo.html").is_file() and (mc_dir / "montecarlo.csv").is_file()
    assert json.loads((mc_dir / "montecarlo.json").read_text())["runs"] == 6
    assert summary["failures"], "a 60-70 % thrust loss should fail at least one run"
    # Any failed case replays bit-for-bit from its own manifest.
    first = summary["failures"][0]
    _, _, cmp = reproduce(Path(mc_dir) / f"run{first:04d}" / "manifest.json", tmp_path / "replay")
    assert cmp["log_match"]

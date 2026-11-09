"""Anomaly detection: the shipped baseline flags faults and leaves clean runs alone."""

import json

import numpy as np
import pytest

from hwil import anomaly as an
from hwil import scenario as scn
from hwil.run import run_scenario

from conftest import SCENARIOS

BASELINE = json.loads(an.default_baseline("builtin").read_text())


def _run(name, tmp_path, **overrides):
    sc = scn.load(SCENARIOS / f"{name}.yaml")
    for k, v in overrides.items():
        sc[k] = v
    run_dir, _ = run_scenario(sc, {"plots": False}, tmp_path)
    return run_dir


def test_baseline_shape():
    assert BASELINE["controllers"] == ["builtin"]
    for f in an.FEATURE_NAMES:
        for g in an.GROUPS:
            s = BASELINE["stats"][f][g]
            assert s["spread"] >= an.FEATURES[f][2]


def test_clean_run_not_flagged(tmp_path):
    # Seed and wind differ from anything in the baseline's training runs.
    run_dir = _run("takeoff_hold_land", tmp_path, seed=987654)
    res = an.score(run_dir, BASELINE)
    assert not res["anomalous"], res["segments"]
    page = an.write_outputs(res, BASELINE, plots=False)
    assert page.exists() and (run_dir / "anomaly.json").exists()


def test_motor_loss_flagged_on_torque(tmp_path):
    run_dir = _run("motor_degraded", tmp_path)  # motor 1 at 75 % from t = 30 s
    res = an.score(run_dir, BASELINE)
    assert res["segments"], "motor loss not detected"
    first = res["segments"][0]
    assert 30.0 <= first["t_start"] <= 33.0
    assert first["signals"][0]["feature"].startswith("torque_")


def test_gps_dropout_flagged(tmp_path):
    run_dir = _run("gps_dropout_hold", tmp_path)  # 40 s to 50 s
    res = an.score(run_dir, BASELINE)
    seg = [s for s in res["segments"] if s["signals"][0]["feature"] == "gps_invalid"]
    assert seg and 40.0 <= seg[0]["t_start"] <= 41.5 and 50.0 <= seg[0]["t_end"] <= 52.0


def test_segments_need_min_duration():
    t = np.arange(0, 10, an.BLOCK_S) + an.BLOCK_S / 2
    z = np.zeros((t.size, len(an.FEATURE_NAMES)))
    z[10:14, 0] = 9.0  # 0.4 s: too short
    z[50:70, 2] = -12.0  # 2 s
    segs = an.segments(t, z, 7.0, 1.0)
    assert len(segs) == 1
    assert segs[0]["t_start"] == pytest.approx(5.0) and segs[0]["t_end"] == pytest.approx(7.0)
    assert segs[0]["signals"][0] == {"feature": an.FEATURE_NAMES[2], "peak_z": 12.0, "sign": "-"}


def test_fit_round_trip(tmp_path):
    runs = [_run("takeoff_hold_land", tmp_path / str(s), seed=s) for s in (11, 12)]
    base = an.fit(runs)
    assert base["n_runs"] == 2 and len(base["thrust_curve"]) == 3
    res = an.score(runs[0], base)
    assert set(res["max_z"]) == set(an.FEATURE_NAMES)

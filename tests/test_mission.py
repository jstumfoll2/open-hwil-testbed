"""Mission sequencer and the suite runner."""

import json

import numpy as np

from hwil import scenario as scn
from hwil.mission import MissionSequencer, ned_to_latlon
from hwil.run import run_scenario
from hwil.suite import run_suite

from conftest import SCENARIOS


def test_ned_to_latlon_round_trip():
    lat, lon = ned_to_latlon(47.0, 8.0, 100.0, -50.0)
    assert abs((lat - 47.0) * np.pi / 180 * 6378137.0 - 100.0) < 1e-6
    assert lon < 8.0


def test_goto_reaches_target_with_builtin(tmp_path):
    sc = scn.resolve({"name": "goto", "sim": {"duration_s": 80},
                      "mission": [{"type": "takeoff", "alt_m": 8}, {"type": "goto", "north_m": 15, "east_m": -10, "alt_m": 12},
                                  {"type": "hold", "duration_s": 3}, {"type": "land"}]})
    run_dir, man = run_scenario(sc, {"plots": False}, tmp_path)
    phases = [e["phase"] for e in man["results"]["mission_events"]]
    assert phases == ["takeoff", "goto", "hold", "land", "landed"]
    log = np.genfromtxt(run_dir / "log.csv", delimiter=",", names=True)
    end = log[-1]
    assert abs(end["n_m"] - 15) < 1.5 and abs(end["e_m"] + 10) < 1.5


def test_goto_timeout_is_recorded():
    seq = MissionSequencer([{"type": "takeoff", "alt_m": 5}, {"type": "goto", "north_m": 50, "timeout_s": 2}])
    seq.phase, seq.idx, seq.alt_target = "takeoff", 0, 5.0

    class S:  # a vehicle that never moves
        t = 10.0
        pos_ned = np.array([0.0, 0.0, -5.0])
        vel_ned = np.zeros(3)
        alt_m = 5.0
        alt_agl_m = 5.0
    s = S()
    seq.update(s)                 # takeoff complete -> goto
    s.t = 13.0
    seq.update(s)                 # 3 s later -> timeout
    assert ("13.0", "goto_timeout") in [(f"{t:.1f}", p) for t, p in seq.events]


def test_suite_report(tmp_path):
    files = [SCENARIOS / "takeoff_hold_land.yaml", SCENARIOS / "altitude_steps.yaml"]
    suite_dir, summary = run_suite(files, {"plots": False}, tmp_path, jobs=2)
    assert summary["scenarios"] == 2 and summary["passed"] == 2
    assert (suite_dir / "suite.html").is_file()
    assert json.loads((suite_dir / "suite.json").read_text())["requirements_total"] == 8

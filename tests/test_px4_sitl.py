"""Full SIL run with a real PX4 SITL build in the loop.

Skipped unless HWIL_PX4_DIR points at a PX4-Autopilot checkout that has been
built with `make px4_sitl_default`. Run with:  HWIL_PX4_DIR=~/PX4-Autopilot pytest -m px4
"""

import os
import subprocess
import time
from pathlib import Path

import pytest

from hwil import scenario as scn
from hwil.run import run_scenario

from conftest import ROOT, SCENARIOS

PX4_DIR = os.environ.get("HWIL_PX4_DIR")

pytestmark = [
    pytest.mark.px4,
    pytest.mark.skipif(not PX4_DIR, reason="set HWIL_PX4_DIR to a built PX4-Autopilot checkout"),
]


@pytest.fixture
def px4_sitl(tmp_path):
    log = open(tmp_path / "px4.log", "w")
    env = {**os.environ, "PX4_DIR": PX4_DIR, "PX4_WORK_DIR": str(tmp_path / "px4")}
    proc = subprocess.Popen([str(ROOT / "ci" / "run_px4_sitl.sh")], env=env, stdout=log, stderr=subprocess.STDOUT)
    time.sleep(1.0)
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
    log.close()


FILES = sorted(SCENARIOS.glob("*.yaml"))


@pytest.mark.parametrize("path", FILES, ids=[p.stem for p in FILES])
def test_scenario_with_px4(px4_sitl, path, tmp_path):
    """The same scenario files and requirements that grade the builtin controller."""
    sc = scn.load(path)
    run_dir, man = run_scenario(sc, {"controller": "px4"}, tmp_path / "runs")
    info = man["controller_info"]
    assert info["autopilot_version"], "never heard AUTOPILOT_VERSION from PX4"
    assert not man["results"]["aborted"], man["results"]["aborted"]
    assert man["results"]["landed"]
    failed = [r for r in man["results"]["requirements"] if not r["passed"]]
    assert not failed, failed
    stats = info["link_stats"]
    assert stats["reply_timeouts"] < 0.01 * stats["frames"]

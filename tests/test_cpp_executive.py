"""The compiled executive must fly exactly like the Python one.

Skipped unless the binary exists (executive/build/hwil_exec or $HWIL_EXEC_BIN).
"""

import numpy as np
import pytest

from hwil import scenario as scn
from hwil.cppexec import CppExecError, binary
from hwil.report import load_log
from hwil.run import run_scenario

from conftest import SCENARIOS

try:
    binary()
    HAVE_BIN = True
except CppExecError:
    HAVE_BIN = False

pytestmark = pytest.mark.skipif(not HAVE_BIN, reason="compiled executive not built (see executive/README.md)")

# Compare everything except baro_alt_m: the C++ loop has no sensor noise models yet.
STATE_COLS = ["t", "phase", "mission_index", "n_m", "e_m", "alt_m", "alt_cmd_m", "vn_mps", "ve_mps", "vd_mps",
              "roll_rad", "pitch_rad", "yaw_rad", "p_rps", "q_rps", "r_rps", "m0", "m1", "m2", "m3",
              "n_cmd_m", "e_cmd_m"]


@pytest.mark.parametrize("name", ["waypoint_square", "motor_degraded", "crosswind_hold"])
def test_cpp_matches_python(name, tmp_path):
    sc = scn.load(SCENARIOS / f"{name}.yaml")
    py_dir, py = run_scenario(sc, {"plots": False}, tmp_path / "py")
    cc_dir, cc = run_scenario(sc, {"plots": False, "executive": "cpp"}, tmp_path / "cc")
    a, b = load_log(py_dir / "log.csv"), load_log(cc_dir / "log.csv")
    assert len(a["t"]) == len(b["t"])
    # In the air the two executives agree to round-off. At touchdown the gear
    # contact model switches on, which turns 1e-12 differences into small but
    # visible ones, so ground contact only has to agree physically.
    air = (a["phase"] < 3) | (a["alt_agl_m"] > 0.5)
    for col in STATE_COLS:
        err = np.abs(a[col] - b[col])
        assert err[air].max() < 1e-6, (col, err[air].max())
        assert err.max() < 0.05, (col, err.max())
    ev_py = {e["phase"]: e["t"] for e in py["results"]["mission_events"]}
    ev_cc = {e["phase"]: e["t"] for e in cc["results"]["mission_events"]}
    assert ev_py.keys() == ev_cc.keys()
    assert all(abs(ev_py[k] - ev_cc[k]) < 0.1 for k in ev_py)
    assert [r["passed"] for r in py["results"]["requirements"]] == [r["passed"] for r in cc["results"]["requirements"]]
    assert cc["controller_info"]["executive"] == "cpp"
    assert py["config_hash"] != cc["config_hash"]


def test_cpp_rejects_px4(tmp_path):
    with pytest.raises(CppExecError):
        run_scenario(scn.load(SCENARIOS / "takeoff_hold_land.yaml"), {"executive": "cpp", "controller": "px4"}, tmp_path)

"""M1 reference scenario, graded with RTAMT signal temporal logic."""

import numpy as np

from hwil.report import load_log
from hwil.requirements import evaluate, signals_from_log


def test_all_scenario_requirements_pass(reference_run):
    _, man = reference_run
    failed = [r for r in man["results"]["requirements"] if not r["passed"]]
    assert man["results"]["landed"]
    assert not failed, failed


def test_altitude_hold_within_5m_after_30s(reference_run):
    """Brief section 9, task 5: altitude stays within +-5 m of the commanded
    loiter altitude after 30 s. Checked with RTAMT, then cross-checked by hand."""
    run_dir, _ = reference_run
    log = load_log(run_dir / "log.csv")
    sig = signals_from_log(log)
    rob = evaluate("eventually[30s,30s](always((loiter > 0) -> (abs(alt_m - alt_cmd_m) <= 5.0)))", sig)
    assert rob > 0, f"altitude requirement violated, robustness {rob:.3f} m"

    # Independent check of the same property without STL (10 Hz samples, like RTAMT).
    window = (log["phase"] == 2) & (log["t"] >= 30.0)
    assert window.any(), "the hold phase never reached t >= 30 s"
    max_err = np.max(np.abs(log["alt_m"][window] - log["alt_cmd_m"][window]))
    assert abs((5.0 - max_err) - rob) < 0.05


def test_stl_catches_a_violation(reference_run):
    """The same spec with a 1 cm tolerance must fail: the grader is not a rubber stamp."""
    run_dir, _ = reference_run
    sig = signals_from_log(load_log(run_dir / "log.csv"))
    rob = evaluate("eventually[30s,30s](always((loiter > 0) -> (abs(alt_m - alt_cmd_m) <= 0.0001)))", sig)
    assert rob < 0


def test_outputs_exist(reference_run):
    run_dir, man = reference_run
    for name in ("log.csv", "timing.csv", "manifest.json", "report.html", "flight.png", "motors.png", "timing.png"):
        assert (run_dir / name).is_file(), name
    assert man["results"]["timing"]["frames"] > 0


def test_unexercised_requirement_does_not_pass():
    """A guard that never holds gives +BIG robustness: reported as not exercised, not as a pass."""
    from hwil.requirements import grade
    t = np.arange(0, 20, 0.1)
    log = {k: np.zeros_like(t) for k in ("phase", "alt_m", "alt_cmd_m", "vn_mps", "ve_mps",
                                          "n_m", "e_m", "roll_rad", "pitch_rad")}
    log["t"] = t
    (r,) = grade([{"id": "R", "text": "x", "stl": "always((loiter > 0) -> (alt_m <= 1.0))"}], log)
    assert r.vacuous and not r.passed

"""Grade a run log against requirement-style STL specs with RTAMT.

Each requirement in a scenario looks like::

    - id: REQ-ALT-001
      text: After 30 s, altitude stays within 5 m of the commanded loiter altitude.
      stl: "eventually[30s,30s](always((loiter > 0) -> (abs(alt_m - alt_cmd_m) <= 5.0)))"

Signals are the log columns (see ``hwil.executive.LOG_COLUMNS``) plus a few
derived ones listed in ``DERIVED``. The log is resampled to ``sample_hz``
before evaluation. A requirement passes when the robustness at t = 0 is
positive and the requirement was actually exercised (not vacuously true); the
robustness value is the margin (in the units of the spec).
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

import numpy as np
import rtamt

# Boolean signals are +BIG when true and -BIG when false, so a guard such as
# ``(loiter > 0) -> phi`` never caps the robustness: outside the guard the
# implication scores +BIG, inside it scores exactly phi's margin.
BIG = 1.0e6

DERIVED = {
    "loiter": "+BIG while in the hold phase, else -BIG (use as `loiter > 0`)",
    "landed": "+BIG once the vehicle has landed and disarmed, else -BIG (use as `landed > 0`)",
    "alt_err_m": "alt_m - alt_cmd_m",
    "hspeed_mps": "horizontal ground speed",
    "hdist_m": "horizontal distance from home",
    "hdist_cmd_m": "horizontal distance from the commanded point (goto target or hold point)",
    "pos_err_m": "3-D distance from the commanded point",
    "goto": "+BIG during a goto phase, else -BIG (use as `goto > 0`)",
    "tilt_deg": "angle between body z and vertical",
}

_IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")


@dataclass
class RequirementResult:
    id: str
    text: str
    stl: str
    robustness: float
    passed: bool
    vacuous: bool = False

    def as_dict(self) -> dict:
        return {"id": self.id, "text": self.text, "stl": self.stl,
                "robustness": self.robustness, "passed": self.passed, "vacuous": self.vacuous}


def signals_from_log(log: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    sig = dict(log)
    sig["loiter"] = np.where(log["phase"] == 2, BIG, -BIG)
    sig["landed"] = np.where(log["phase"] == 4, BIG, -BIG)
    sig["alt_err_m"] = log["alt_m"] - log["alt_cmd_m"]
    sig["hspeed_mps"] = np.hypot(log["vn_mps"], log["ve_mps"])
    sig["hdist_m"] = np.hypot(log["n_m"], log["e_m"])
    if "n_cmd_m" in log:
        sig["hdist_cmd_m"] = np.hypot(log["n_m"] - log["n_cmd_m"], log["e_m"] - log["e_cmd_m"])
        sig["pos_err_m"] = np.sqrt(sig["hdist_cmd_m"] ** 2 + (log["alt_m"] - log["alt_cmd_m"]) ** 2)
    sig["goto"] = np.where(log["phase"] == 5, BIG, -BIG)
    sig["tilt_deg"] = np.degrees(np.arccos(np.clip(np.cos(log["roll_rad"]) * np.cos(log["pitch_rad"]), -1, 1)))
    return sig


def evaluate(stl: str, signals: dict[str, np.ndarray], sample_hz: float = 10.0) -> float:
    t = signals["t"]
    period_ms = int(round(1000.0 / sample_hz))
    grid = np.arange(0.0, t[-1] + 1e-9, period_ms / 1000.0)
    names = sorted({tok for tok in _IDENT.findall(stl) if tok in signals and tok != "t"})
    if not names:
        raise ValueError(f"spec uses no known signals: {stl}")
    spec = rtamt.StlDiscreteTimeSpecification()
    for n in names:
        spec.declare_var(n, "float")
    spec.set_sampling_period(period_ms, "ms", 0.1)
    spec.spec = stl
    spec.parse()
    data = {"time": grid.tolist()}
    for n in names:
        # Zero-order hold onto the evaluation grid.
        idx = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)
        data[n] = signals[n][idx].tolist()
    rob = spec.evaluate(data)
    value = float(rob[0][1])
    return value if math.isfinite(value) else float("inf")


def grade(requirements: list[dict], log: dict[str, np.ndarray], sample_hz: float = 10.0) -> list[RequirementResult]:
    sig = signals_from_log(log)
    out = []
    for r in requirements:
        rob = evaluate(r["stl"], sig, sample_hz)
        # Infinity means the time window lies beyond the end of the run. For a
        # guarded spec ("guard -> phi"), +BIG means the guard never held (for
        # example the vehicle never loitered). Either way the requirement was
        # not exercised, so it does not pass.
        vacuous = (not math.isfinite(rob)) or ("->" in r["stl"] and rob >= BIG / 2)
        out.append(RequirementResult(r["id"], r["text"], r["stl"], rob, rob > 0 and not vacuous, vacuous))
    return out

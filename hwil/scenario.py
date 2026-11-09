"""Scenario files: initial conditions, mission, wind, faults and requirements.

A scenario is a YAML file. Anything not given falls back to DEFAULTS below, and
the fully resolved scenario (defaults merged in) is what goes into the run
manifest and the config hash, so a run can be rebuilt from its manifest alone.
"""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

SCHEMA_VERSION = 1

DEFAULTS: dict[str, Any] = {
    "schema": SCHEMA_VERSION,
    "name": "unnamed",
    "description": "",
    "seed": 0,
    "vehicle": {
        # Public F450 quadcopter from JSBSim plus a drag overlay
        # (models/jsbsim/F450-hil). Use "F450" for the unmodified upstream model.
        "model": "F450-hil",
    },
    "sim": {
        "rate_hz": 250,
        "duration_s": 120.0,
        # Stop early this many seconds after the vehicle has landed and disarmed.
        "stop_after_landed_s": 2.0,
    },
    "initial": {
        # Default home is the public PX4 SITL default location (Zurich).
        "lat_deg": 47.397742,
        "lon_deg": 8.545594,
        "ground_alt_msl_m": 488.0,
        "heading_deg": 0.0,
    },
    "wind": {
        "north_mps": 0.0,
        "east_mps": 0.0,
        "down_mps": 0.0,
        # JSBSim MIL-SPEC turbulence. 0 = off, 1..7 = light..severe.
        "turbulence_severity": 0,
    },
    "sensors": {
        "imu": {"accel_noise_mps2": 0.02, "gyro_noise_rps": 0.002,
                "accel_bias_mps2": [0.0, 0.0, 0.0], "gyro_bias_rps": [0.0, 0.0, 0.0]},
        "baro": {"noise_hpa": 0.01, "bias_hpa": 0.0},
        "mag": {"noise_gauss": 0.002,
                # Approximate public WMM field near the default home, NED, gauss.
                "field_ned_gauss": [0.216, 0.016, 0.424]},
        "gps": {"rate_hz": 10.0, "noise_h_m": 0.3, "noise_v_m": 0.5,
                "noise_vel_mps": 0.05, "satellites": 12},
    },
    "mission": [
        {"type": "takeoff", "alt_m": 10.0},
        {"type": "hold", "duration_s": 20.0},
        {"type": "land"},
    ],
    "faults": [],
    "requirements": [],
}

MISSION_TYPES = {"takeoff", "goto", "hold", "land"}
MAX_RANGE_M = 300.0
FAULT_TYPES = {"motor_efficiency", "gps_dropout", "baro_bias"}


class ScenarioError(ValueError):
    pass


def _merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for key, val in override.items():
        if isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], val)
        else:
            out[key] = copy.deepcopy(val)
    return out


def resolve(raw: dict) -> dict:
    """Merge defaults into a raw scenario dict and validate it."""
    if not isinstance(raw, dict):
        raise ScenarioError("scenario must be a mapping")
    unknown = set(raw) - set(DEFAULTS)
    if unknown:
        raise ScenarioError(f"unknown top-level keys: {sorted(unknown)}")
    sc = _merge(DEFAULTS, raw)
    validate(sc)
    return sc


def validate(sc: dict) -> None:
    if sc["schema"] != SCHEMA_VERSION:
        raise ScenarioError(f"schema {sc['schema']} not supported (expected {SCHEMA_VERSION})")
    rate = sc["sim"]["rate_hz"]
    if not (50 <= rate <= 2000):
        raise ScenarioError("sim.rate_hz must be between 50 and 2000")
    if sc["sim"]["duration_s"] <= 0:
        raise ScenarioError("sim.duration_s must be positive")
    if not sc["mission"]:
        raise ScenarioError("mission must have at least one phase")
    for i, ph in enumerate(sc["mission"]):
        t = ph.get("type")
        if t not in MISSION_TYPES:
            raise ScenarioError(f"mission[{i}]: type must be one of {sorted(MISSION_TYPES)}")
        if t == "takeoff" and not (1.0 <= float(ph.get("alt_m", 0)) <= 120.0):
            raise ScenarioError(f"mission[{i}]: takeoff alt_m must be 1..120 m")
        if t == "hold" and float(ph.get("duration_s", 0)) <= 0:
            raise ScenarioError(f"mission[{i}]: hold needs a positive duration_s")
        if t == "takeoff" and i != 0:
            raise ScenarioError(f"mission[{i}]: takeoff must be the first phase (use goto to change altitude)")
        if t == "goto":
            if i == 0:
                raise ScenarioError("mission[0]: goto needs a takeoff first")
            for key in ("north_m", "east_m"):
                if abs(float(ph.get(key, 0.0))) > MAX_RANGE_M:
                    raise ScenarioError(f"mission[{i}]: {key} must be within {MAX_RANGE_M:.0f} m of home")
            if "alt_m" in ph and not (1.0 <= float(ph["alt_m"]) <= 120.0):
                raise ScenarioError(f"mission[{i}]: goto alt_m must be 1..120 m")
            if not (0.5 <= float(ph.get("speed_mps", 3.0)) <= 10.0):
                raise ScenarioError(f"mission[{i}]: goto speed_mps must be 0.5..10")
        if t in ("hold", "land") and i == 0:
            raise ScenarioError(f"mission[0]: {t} needs a takeoff first")
    for i, f in enumerate(sc["faults"]):
        if f.get("type") not in FAULT_TYPES:
            raise ScenarioError(f"faults[{i}]: type must be one of {sorted(FAULT_TYPES)}")
    for i, r in enumerate(sc["requirements"]):
        for key in ("id", "text", "stl"):
            if key not in r:
                raise ScenarioError(f"requirements[{i}] is missing '{key}'")


def load(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    return resolve(raw)


def canonical_json(obj: Any) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"))


def config_hash(obj: Any) -> str:
    """SHA-256 of the canonical JSON form of a resolved config."""
    return hashlib.sha256(canonical_json(obj).encode("utf-8")).hexdigest()

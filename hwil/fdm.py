"""JSBSim flight dynamics wrapper.

Exposes a small, SI-unit, NED-frame view of the JSBSim state and a per-motor
command input. The default vehicle is the public F450 quadcopter that ships
with JSBSim (with the drag overlay in models/jsbsim/F450-hil). Its four external ESC inputs (fcs/cmd??_ext_nd) are driven
directly, which bypasses the model's own built-in stability augmentation, so
the flight controller in the loop (builtin or PX4) does all of the control.

Motor order matches the PX4 quad-x convention:
  0 = front right (CCW), 1 = aft left (CCW), 2 = front left (CW), 3 = aft right (CW)
"""

from __future__ import annotations

import contextlib
import math
import os
from dataclasses import dataclass

import jsbsim
import numpy as np

from .models import jsbsim_root, prepare_aircraft

FT = 0.3048
LBF = 4.4482216152605
SLUG = 14.593902937

MOTOR_INPUTS = ("fcs/cmdFR_ext_nd", "fcs/cmdAL_ext_nd", "fcs/cmdFL_ext_nd", "fcs/cmdAR_ext_nd")
EARTH_R = 6378137.0


@dataclass
class State:
    t: float
    pos_ned: np.ndarray      # m, relative to home (flat-earth)
    vel_ned: np.ndarray      # m/s
    euler: np.ndarray        # roll, pitch, yaw (rad)
    rates_body: np.ndarray   # p, q, r (rad/s)
    accel_body: np.ndarray   # specific force, body frame (m/s^2); ~[0,0,-g] at rest
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    alt_agl_m: float
    wind_ned: np.ndarray     # m/s

    @property
    def alt_m(self) -> float:
        """Altitude above home (positive up)."""
        return -float(self.pos_ned[2])


@contextlib.contextmanager
def _quiet_fd(fd: int = 1):
    """Silence C++ chatter on a file descriptor (JSBSim prints while loading)."""
    try:
        saved = os.dup(fd)
    except OSError:
        yield
        return
    try:
        with open(os.devnull, "w") as devnull:
            os.dup2(devnull.fileno(), fd)
            yield
    finally:
        os.dup2(saved, fd)
        os.close(saved)


def rotation_body_to_ned(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ])


class JsbsimFdm:
    def __init__(self, scenario: dict, quiet: bool = True):
        self.sc = scenario
        self.dt = 1.0 / scenario["sim"]["rate_hz"]
        init = scenario["initial"]
        self.home_lat = float(init["lat_deg"])
        self.home_lon = float(init["lon_deg"])
        self.ground_msl = float(init["ground_alt_msl_m"])

        ctx = _quiet_fd(1) if quiet else contextlib.nullcontext()
        with ctx:
            self.fdm = jsbsim.FGFDMExec(str(jsbsim_root()))
            self.fdm.set_debug_level(0)
            aircraft_path, model = prepare_aircraft(scenario["vehicle"]["model"])
            self.fdm.set_aircraft_path(str(aircraft_path))
            if not self.fdm.load_model(model):
                raise RuntimeError(f"JSBSim could not load model {scenario['vehicle']['model']!r}")
        self.fdm.set_dt(self.dt)
        self._set_initial_conditions(scenario)
        self.motor_cmd = np.zeros(4)
        self.home_alt_msl = self.fdm["position/h-sl-meters"]

    def _set_initial_conditions(self, sc: dict) -> None:
        f = self.fdm
        init = sc["initial"]
        # Seed JSBSim's own random numbers (turbulence, sensor noise in models).
        f["simulation/randomseed"] = int(sc["seed"]) & 0x7FFFFFFF
        f["ic/lat-geod-deg"] = self.home_lat
        f["ic/long-gc-deg"] = self.home_lon
        f["ic/terrain-elevation-ft"] = self.ground_msl / FT
        f["ic/h-agl-ft"] = 0.5  # resting on the landing gear
        f["ic/psi-true-deg"] = float(init["heading_deg"])
        f["ic/u-fps"] = 0.0
        f["ic/v-fps"] = 0.0
        f["ic/w-fps"] = 0.0
        if not f.run_ic():
            raise RuntimeError("JSBSim run_ic failed")
        # Wind must be set after run_ic, which resets the atmosphere.
        wind = sc["wind"]
        f["atmosphere/wind-north-fps"] = wind["north_mps"] / FT
        f["atmosphere/wind-east-fps"] = wind["east_mps"] / FT
        f["atmosphere/wind-down-fps"] = wind["down_mps"] / FT
        sev = int(wind.get("turbulence_severity", 0))
        if sev > 0:
            f["atmosphere/turb-type"] = 3  # MIL-SPEC (Dryden)
            speed = math.hypot(wind["north_mps"], wind["east_mps"])
            f["atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps"] = speed / FT
            f["atmosphere/turbulence/milspec/severity"] = sev
        for e in range(4):
            f[f"propulsion/engine[{e}]/set-running"] = 1

    def set_motors(self, cmd) -> None:
        cmd = np.clip(np.asarray(cmd, dtype=float), 0.0, 1.0)
        self.motor_cmd = cmd
        for prop, val in zip(MOTOR_INPUTS, cmd):
            self.fdm[prop] = float(val)

    def step(self) -> None:
        if not self.fdm.run():
            raise RuntimeError("JSBSim run() returned false")

    def state(self) -> State:
        f = self.fdm
        lat = f["position/lat-geod-deg"]
        lon = f["position/long-gc-deg"]
        alt_msl = f["position/h-sl-meters"]
        north = math.radians(lat - self.home_lat) * EARTH_R
        east = math.radians(lon - self.home_lon) * EARTH_R * math.cos(math.radians(self.home_lat))
        down = -(alt_msl - self.home_alt_msl)
        mass = f["inertia/mass-slugs"] * SLUG
        # Non-gravitational forces / mass = what an accelerometer measures.
        accel = np.array([f["forces/fbx-total-lbs"], f["forces/fby-total-lbs"],
                          f["forces/fbz-total-lbs"]]) * LBF / mass
        return State(
            t=f["simulation/sim-time-sec"],
            pos_ned=np.array([north, east, down]),
            vel_ned=np.array([f["velocities/v-north-fps"], f["velocities/v-east-fps"],
                              f["velocities/v-down-fps"]]) * FT,
            euler=np.array([f["attitude/phi-rad"], f["attitude/theta-rad"], f["attitude/psi-rad"]]),
            rates_body=np.array([f["velocities/p-rad_sec"], f["velocities/q-rad_sec"],
                                 f["velocities/r-rad_sec"]]),
            accel_body=accel,
            lat_deg=lat,
            lon_deg=lon,
            alt_msl_m=alt_msl,
            alt_agl_m=f["position/h-agl-ft"] * FT,
            wind_ned=np.array([f["atmosphere/total-wind-north-fps"], f["atmosphere/total-wind-east-fps"],
                               f["atmosphere/total-wind-down-fps"]]) * FT,
        )

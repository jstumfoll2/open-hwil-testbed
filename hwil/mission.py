"""Mission sequencer: walks the scenario's mission phases.

The same sequencer is used with every flight controller. With the builtin
controller it produces the position/velocity setpoints. With PX4 it decides
when to send the next command (arm + takeoff, reposition, land) and records the
commanded position, so logs and requirements look the same whichever
controller flew.

Phases: ``takeoff`` (climb at home), ``goto`` (fly to a point relative to
home), ``hold`` (position hold at the current target), ``land`` (descend at the
current target), then ``landed`` once disarmed on the ground.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .fdm import State

PHASE_IDS = {"preflight": 0, "takeoff": 1, "hold": 2, "land": 3, "landed": 4, "goto": 5}

TAKEOFF_CLIMB_MPS = 1.5
LAND_DESCENT_MPS = 0.7
PREFLIGHT_S = 1.0
LANDED_CONFIRM_S = 1.0
GOTO_DEFAULTS = {"speed_mps": 3.0, "accept_radius_m": 1.0, "timeout_s": 90.0}


@dataclass
class Setpoint:
    phase: str
    phase_index: int          # index into the scenario mission list (-1 = none)
    armed: bool
    pos_ned: np.ndarray
    vel_down: float | None    # when set, overrides vertical position control
    alt_cmd_m: float          # the commanded (target) altitude for this phase


class MissionSequencer:
    def __init__(self, mission: list[dict], on_enter: Callable[[str, dict], None] | None = None):
        self.mission = mission
        self.on_enter = on_enter
        self.idx = -1
        self.phase = "preflight"
        self.t_phase = 0.0
        self.alt_target = 0.0
        self.ne_target = np.zeros(2)       # commanded north/east (m from home)
        self._goto_start = np.zeros(3)     # NED where the current goto began
        self._goto_spec: dict = {}
        self._landed_since: float | None = None
        self.events: list[tuple[float, str]] = []

    @property
    def target_ned(self) -> np.ndarray:
        return np.array([self.ne_target[0], self.ne_target[1], -self.alt_target])

    def _advance(self, t: float, s: State | None = None) -> None:
        self.idx += 1
        if self.idx >= len(self.mission):
            self.phase = "landed" if self.phase == "landed" else "hold_end"
            return
        ph = self.mission[self.idx]
        self.phase = ph["type"]
        self.t_phase = t
        if self.phase == "takeoff":
            self.alt_target = float(ph["alt_m"])
        elif self.phase == "goto":
            self._goto_spec = {**GOTO_DEFAULTS, **ph}
            self._goto_start = self.target_ned
            self.ne_target = np.array([float(ph.get("north_m", self.ne_target[0])),
                                       float(ph.get("east_m", self.ne_target[1]))])
            self.alt_target = float(ph.get("alt_m", self.alt_target))
        self.events.append((t, self.phase))
        if self.on_enter:
            self.on_enter(self.phase, ph)

    @property
    def done(self) -> bool:
        return self.phase in ("landed", "hold_end")

    def update(self, s: State) -> Setpoint:
        t = s.t
        if self.phase == "preflight" and t >= PREFLIGHT_S:
            self._advance(t)
        elif self.phase == "takeoff":
            if abs(s.alt_m - self.alt_target) < 0.5 and abs(s.vel_ned[2]) < 0.5:
                self._advance(t)
        elif self.phase == "goto":
            g = self._goto_spec
            err = float(np.linalg.norm(s.pos_ned - self.target_ned))
            if err < float(g["accept_radius_m"]) and float(np.linalg.norm(s.vel_ned)) < 0.5:
                self._advance(t)
            elif t - self.t_phase >= float(g["timeout_s"]):
                self.events.append((t, "goto_timeout"))
                self._advance(t)
        elif self.phase == "hold":
            if t - self.t_phase >= float(self.mission[self.idx]["duration_s"]):
                self._advance(t)
        elif self.phase == "land":
            if s.alt_agl_m < 0.25 and abs(s.vel_ned[2]) < 0.2:
                if self._landed_since is None:
                    self._landed_since = t
                elif t - self._landed_since >= LANDED_CONFIRM_S:
                    self.phase = "landed"
                    self.events.append((t, "landed"))
                    if self.on_enter:
                        self.on_enter("landed", {})
            else:
                self._landed_since = None

        target = self.target_ned
        if self.phase == "preflight":
            return Setpoint("preflight", -1, False, np.zeros(3), None, 0.0)
        if self.phase == "takeoff":
            ramp = (t - self.t_phase) * TAKEOFF_CLIMB_MPS
            pos = np.array([target[0], target[1], -min(self.alt_target, ramp)])
            return Setpoint("takeoff", self.idx, True, pos, None, self.alt_target)
        if self.phase == "goto":
            # A "carrot" moving from the start point to the target at the requested speed.
            leg = target - self._goto_start
            dist = float(np.linalg.norm(leg))
            travelled = (t - self.t_phase) * float(self._goto_spec["speed_mps"])
            frac = 1.0 if dist < 1e-6 else min(1.0, travelled / dist)
            return Setpoint("goto", self.idx, True, self._goto_start + frac * leg, None, self.alt_target)
        if self.phase in ("hold", "hold_end"):
            return Setpoint("hold", self.idx, True, target, None, self.alt_target)
        if self.phase == "land":
            return Setpoint("land", self.idx, True, np.array([target[0], target[1], 0.0]), LAND_DESCENT_MPS, 0.0)
        return Setpoint("landed", self.idx, False, np.array([target[0], target[1], 0.0]), None, 0.0)


def ned_to_latlon(home_lat: float, home_lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Flat-earth offset from home, the inverse of the conversion in fdm.State."""
    r = 6378137.0
    lat = home_lat + math.degrees(north_m / r)
    lon = home_lon + math.degrees(east_m / (r * math.cos(math.radians(home_lat))))
    return lat, lon

"""Mission sequencer: walks the scenario's mission phases.

The same sequencer is used with every flight controller. With the builtin
controller it produces the position/velocity setpoints. With PX4 it decides
when to send the next command (arm + takeoff, land) and records the commanded
altitude, so logs and requirements look the same whichever controller flew.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from .fdm import State

PHASE_IDS = {"preflight": 0, "takeoff": 1, "hold": 2, "land": 3, "landed": 4}

TAKEOFF_CLIMB_MPS = 1.5
LAND_DESCENT_MPS = 0.7
PREFLIGHT_S = 1.0
LANDED_CONFIRM_S = 1.0


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
        self._landed_since: float | None = None
        self.events: list[tuple[float, str]] = []

    def _advance(self, t: float) -> None:
        self.idx += 1
        if self.idx >= len(self.mission):
            self.phase = "landed" if self.phase == "landed" else "hold_end"
            return
        ph = self.mission[self.idx]
        self.phase = ph["type"]
        self.t_phase = t
        if self.phase == "takeoff":
            self.alt_target = float(ph["alt_m"])
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

        home = np.zeros(3)
        if self.phase == "preflight":
            return Setpoint("preflight", -1, False, home, None, 0.0)
        if self.phase == "takeoff":
            ramp = (t - self.t_phase) * TAKEOFF_CLIMB_MPS
            pos = np.array([0.0, 0.0, -min(self.alt_target, ramp)])
            return Setpoint("takeoff", self.idx, True, pos, None, self.alt_target)
        if self.phase in ("hold", "hold_end"):
            pos = np.array([0.0, 0.0, -self.alt_target])
            return Setpoint("hold", self.idx, True, pos, None, self.alt_target)
        if self.phase == "land":
            return Setpoint("land", self.idx, True, home, LAND_DESCENT_MPS, 0.0)
        return Setpoint("landed", self.idx, False, home, None, 0.0)

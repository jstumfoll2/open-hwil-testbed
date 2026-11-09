"""Fixed-rate simulation executive.

One frame = read state -> sensor models -> flight controller -> faults ->
JSBSim step -> log. Two clock modes:

* ``afap`` (as fast as possible): no sleeping. Deterministic, used for SIL and CI.
  With PX4 SITL this is lockstep: PX4's clock follows the simulated time.
* ``realtime``: each frame starts on an absolute deadline (period = 1/rate).
  Frame start lateness, compute time and overruns are recorded for the timing
  report. Python is not a hard real-time language; this is the M1/M2 baseline
  that the C++ or Trick executive will be measured against.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field

import numpy as np

from .controllers import Controller, SensorFrame
from .fdm import JsbsimFdm
from .mission import PHASE_IDS
from .sensors import SensorSuite

LOG_COLUMNS = [
    "t", "phase", "mission_index",
    "n_m", "e_m", "d_m", "alt_m", "alt_cmd_m", "alt_agl_m",
    "vn_mps", "ve_mps", "vd_mps",
    "roll_rad", "pitch_rad", "yaw_rad",
    "p_rps", "q_rps", "r_rps",
    "ax_mps2", "ay_mps2", "az_mps2",
    "m0", "m1", "m2", "m3",
    "wind_n_mps", "wind_e_mps", "wind_d_mps",
    "lat_deg", "lon_deg",
    "baro_alt_m", "gps_valid",
]

TIMING_COLUMNS = ["frame", "t_sim", "late_us", "compute_us", "io_wait_us", "overrun"]


@dataclass
class TimingStats:
    period_us: float
    frames: int = 0
    overruns: int = 0
    late_us: list = field(default_factory=list)
    compute_us: list = field(default_factory=list)
    io_wait_us: list = field(default_factory=list)

    def summary(self) -> dict:
        def pct(xs, q):
            return float(np.percentile(xs, q)) if xs else 0.0
        return {
            "frames": self.frames,
            "period_us": self.period_us,
            "overruns": self.overruns,
            "late_us_p50": pct(self.late_us, 50),
            "late_us_p99": pct(self.late_us, 99),
            "late_us_max": float(max(self.late_us)) if self.late_us else 0.0,
            "compute_us_p50": pct(self.compute_us, 50),
            "compute_us_p99": pct(self.compute_us, 99),
            "compute_us_max": float(max(self.compute_us)) if self.compute_us else 0.0,
            "io_wait_us_p50": pct(self.io_wait_us, 50),
            "io_wait_us_p99": pct(self.io_wait_us, 99),
        }


def _sane(s) -> bool:
    """False once JSBSim has diverged (typically after a hard crash into the ground)."""
    vals = np.concatenate([s.pos_ned, s.vel_ned, s.euler, s.rates_body, s.accel_body])
    return bool(np.all(np.isfinite(vals)) and np.all(np.abs(vals) < 1e5))


class FaultInjector:
    """Applies the scenario's timed faults."""

    def __init__(self, faults: list[dict]):
        self.faults = faults

    def apply(self, t: float, motors: np.ndarray, sensors: SensorSuite) -> np.ndarray:
        motors = motors.copy()
        sensors.gps_denied = False
        sensors.extra_baro_bias_hpa = 0.0
        for f in self.faults:
            start = float(f.get("t_s", 0.0))
            dur = f.get("duration_s")
            active = t >= start and (dur is None or t < start + float(dur))
            if not active:
                continue
            if f["type"] == "motor_efficiency":
                motors[int(f["motor"])] *= float(f["factor"])
            elif f["type"] == "gps_dropout":
                sensors.gps_denied = True
            elif f["type"] == "baro_bias":
                sensors.extra_baro_bias_hpa += float(f["bias_hpa"])
        return motors


def set_realtime(priority: int | None, cpu: int | None) -> dict:
    """Best-effort SCHED_FIFO priority and CPU pinning (Linux, needs privileges)."""
    info = {"sched_fifo_priority": None, "cpu": None}
    if cpu is not None and hasattr(os, "sched_setaffinity"):
        os.sched_setaffinity(0, {cpu})
        info["cpu"] = cpu
    if priority is not None and hasattr(os, "sched_setscheduler"):
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
        info["sched_fifo_priority"] = priority
    return info


class Executive:
    def __init__(self, scenario: dict, controller: Controller, realtime: bool = False):
        self.sc = scenario
        self.ctrl = controller
        self.realtime = realtime
        self.fdm = JsbsimFdm(scenario)
        self.sensors = SensorSuite(scenario)
        self.faults = FaultInjector(scenario["faults"])
        self.dt = self.fdm.dt
        self.log = np.zeros((0, len(LOG_COLUMNS)))
        self.timing_rows = np.zeros((0, len(TIMING_COLUMNS)))
        self.timing = TimingStats(period_us=self.dt * 1e6)

    def run(self) -> dict:
        sc = self.sc
        n_max = int(math.ceil(sc["sim"]["duration_s"] / self.dt))
        stop_after = float(sc["sim"]["stop_after_landed_s"])
        log = np.full((n_max, len(LOG_COLUMNS)), np.nan)
        tim = np.zeros((n_max, len(TIMING_COLUMNS)))
        period_ns = int(round(self.dt * 1e9))
        t0 = time.perf_counter_ns()
        landed_at = None
        abort = None
        last_baro_alt = 0.0
        gps_valid = 0.0
        n = 0
        for n in range(n_max):
            deadline = t0 + n * period_ns
            if self.realtime:
                while True:
                    remaining = deadline - time.perf_counter_ns()
                    if remaining <= 0:
                        break
                    if remaining > 200_000:
                        time.sleep((remaining - 150_000) / 1e9)
            start = time.perf_counter_ns()
            late_us = (start - deadline) / 1e3 if self.realtime else 0.0

            s = self.fdm.state()
            if not _sane(s):
                abort = f"vehicle state became invalid at t={s.t:.3f} s (simulation diverged)"
                break
            gps = self.sensors.gps(s) if self.sensors.gps_due(s.t) else None
            frame = SensorFrame(s.t, self.sensors.imu(s), self.sensors.baro(s), self.sensors.mag(s), gps)
            io_before = getattr(self.ctrl, "io_wait_ns", 0)
            motors = self.ctrl.update(s, frame, self.dt)
            io_wait_us = (getattr(self.ctrl, "io_wait_ns", 0) - io_before) / 1e3
            motors = self.faults.apply(s.t, motors, self.sensors)
            self.fdm.set_motors(motors)
            self.fdm.step()

            sp_phase = self.ctrl.mission.phase
            last_baro_alt = frame.baro.pressure_alt_m - 0.0
            if gps is not None:
                gps_valid = 1.0 if gps.valid else 0.0
            mi = self.ctrl.mission
            alt_cmd = mi.alt_target if sp_phase in ("takeoff", "hold", "hold_end") else 0.0
            log[n] = [
                s.t, PHASE_IDS.get(sp_phase, PHASE_IDS["hold"]), mi.idx,
                *s.pos_ned, s.alt_m, alt_cmd, s.alt_agl_m,
                *s.vel_ned, *s.euler, *s.rates_body, *s.accel_body,
                *self.fdm.motor_cmd, *s.wind_ned, s.lat_deg, s.lon_deg,
                last_baro_alt, gps_valid,
            ]
            end = time.perf_counter_ns()
            compute_us = (end - start) / 1e3 - io_wait_us
            overrun = 1.0 if self.realtime and end > deadline + period_ns else 0.0
            tim[n] = [n, s.t, late_us, compute_us, io_wait_us, overrun]

            if mi.phase == "landed":
                landed_at = s.t if landed_at is None else landed_at
                if s.t - landed_at >= stop_after:
                    n += 1
                    break
        else:
            n = n_max
        if abort:
            n = int(np.sum(~np.isnan(log[:, 0])))
        self.ctrl.close()
        self.log = log[:n]
        self.timing_rows = tim[:n]
        t = self.timing
        t.frames = n
        t.overruns = int(self.timing_rows[:, 5].sum())
        t.late_us = self.timing_rows[:, 2].tolist()
        t.compute_us = self.timing_rows[:, 3].tolist()
        t.io_wait_us = self.timing_rows[:, 4].tolist()
        return {
            "sim_time_s": float(self.log[-1, 0]) if n else 0.0,
            "wall_time_s": (time.perf_counter_ns() - t0) / 1e9,
            "mission_events": [{"t": round(ev[0], 4), "phase": ev[1]} for ev in self.ctrl.mission.events],
            "landed": self.ctrl.mission.phase == "landed",
            "aborted": abort,
            "timing": t.summary(),
        }

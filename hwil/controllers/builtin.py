"""A small reference quadcopter autopilot that runs in-process.

This is the "no PX4 needed" controller used for fast SIL runs and CI. It is a
textbook cascaded PID (position -> velocity -> attitude -> rate -> mixer) and
it reads the true vehicle state, not the sensor models, so it tests the
simulation plumbing rather than state estimation. PX4 (SITL or HITL) is the
controller that consumes the sensor models.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..fdm import State
from ..mission import MissionSequencer

G = 9.80665

# Quad-x mixer, motor order FR, AL, FL, AR (PX4 convention).
#                    roll  pitch  yaw
MIX = np.array([[-1.0, 1.0, 1.0],    # front right, CCW
                [1.0, -1.0, 1.0],    # aft left,    CCW
                [1.0, 1.0, -1.0],    # front left,  CW
                [-1.0, -1.0, -1.0]])  # aft right,   CW


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Pid:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    i_limit: float = 1.0
    integ: float = 0.0
    prev: float | None = None

    def update(self, err: float, dt: float, deriv: float | None = None) -> float:
        self.integ = float(np.clip(self.integ + err * dt * self.ki, -self.i_limit, self.i_limit))
        if deriv is None:
            deriv = 0.0 if self.prev is None else (err - self.prev) / dt
        self.prev = err
        return self.kp * err + self.integ + self.kd * deriv

    def reset(self) -> None:
        self.integ = 0.0
        self.prev = None


@dataclass
class Gains:
    pos_p: float = 0.95
    vel_xy_p: float = 1.8
    vel_xy_i: float = 0.4
    vel_z_p: float = 4.0
    vel_z_i: float = 2.0
    att_p: float = 6.5
    yaw_p: float = 2.8
    rate_rp_p: float = 0.15
    rate_rp_i: float = 0.05
    rate_rp_d: float = 0.003
    rate_y_p: float = 0.3
    rate_y_i: float = 0.05
    hover_throttle: float = 0.42
    max_tilt_rad: float = math.radians(25)
    max_vel_xy: float = 3.0
    max_climb: float = 1.5
    max_descent: float = 1.0
    max_rate_rp: float = math.radians(200)


@dataclass
class BuiltinController:
    mission: MissionSequencer
    gains: Gains = field(default_factory=Gains)
    name: str = "builtin"

    def __post_init__(self):
        g = self.gains
        self.vn = Pid(g.vel_xy_p, g.vel_xy_i, i_limit=2.0)
        self.ve = Pid(g.vel_xy_p, g.vel_xy_i, i_limit=2.0)
        self.vd = Pid(g.vel_z_p, g.vel_z_i, i_limit=4.0)
        self.rp = Pid(g.rate_rp_p, g.rate_rp_i, g.rate_rp_d, i_limit=0.1)
        self.rq = Pid(g.rate_rp_p, g.rate_rp_i, g.rate_rp_d, i_limit=0.1)
        self.rr = Pid(g.rate_y_p, g.rate_y_i, i_limit=0.1)
        self.yaw_sp: float | None = None

    def info(self) -> dict:
        return {"controller": self.name, "gains": self.gains.__dict__.copy()}

    def close(self) -> None:
        pass

    def update(self, s: State, sensors=None, dt: float = 0.004) -> np.ndarray:
        """Return four normalized motor commands (0..1). Uses true state."""
        g = self.gains
        sp = self.mission.update(s)
        if not sp.armed:
            for pid in (self.vn, self.ve, self.vd, self.rp, self.rq, self.rr):
                pid.reset()
            return np.zeros(4)
        if self.yaw_sp is None:
            self.yaw_sp = float(s.euler[2])

        # Position -> velocity setpoint (NED).
        v_sp = np.zeros(3)
        v_sp[:2] = g.pos_p * (sp.pos_ned[:2] - s.pos_ned[:2])
        if sp.vel_down is not None:
            v_sp[2] = sp.vel_down
        else:
            v_sp[2] = g.pos_p * (sp.pos_ned[2] - s.pos_ned[2])
        hspeed = float(np.hypot(*v_sp[:2]))
        if hspeed > g.max_vel_xy:
            v_sp[:2] *= g.max_vel_xy / hspeed
        v_sp[2] = float(np.clip(v_sp[2], -g.max_climb, g.max_descent))

        # Velocity -> acceleration setpoint. While still on the ground, hold the
        # horizontal integrators so ground friction cannot wind them up.
        on_ground = s.alt_agl_m < 0.3 and sp.phase in ("takeoff", "land")
        err = v_sp - s.vel_ned
        a_n = self.vn.update(err[0], dt) if not on_ground else 0.0
        a_e = self.ve.update(err[1], dt) if not on_ground else 0.0
        a_d = self.vd.update(err[2], dt)

        # Acceleration -> attitude + collective (small-tilt geometry).
        yaw = float(s.euler[2])
        a_fwd = math.cos(yaw) * a_n + math.sin(yaw) * a_e
        a_right = -math.sin(yaw) * a_n + math.cos(yaw) * a_e
        lift = max(G - a_d, 1.0)
        pitch_sp = float(np.clip(-math.atan2(a_fwd, lift), -g.max_tilt_rad, g.max_tilt_rad))
        roll_sp = float(np.clip(math.atan2(a_right * math.cos(pitch_sp), lift), -g.max_tilt_rad, g.max_tilt_rad))
        tilt = max(math.cos(s.euler[0]) * math.cos(s.euler[1]), 0.5)
        thrust = g.hover_throttle * math.sqrt(lift / G / tilt)

        # Attitude -> body rates.
        p_sp = float(np.clip(g.att_p * (roll_sp - s.euler[0]), -g.max_rate_rp, g.max_rate_rp))
        q_sp = float(np.clip(g.att_p * (pitch_sp - s.euler[1]), -g.max_rate_rp, g.max_rate_rp))
        r_sp = float(np.clip(g.yaw_p * _wrap(self.yaw_sp - yaw), -1.0, 1.0))

        # Rates -> torque commands.
        rates = s.rates_body
        tau = np.array([
            self.rp.update(p_sp - rates[0], dt, deriv=None),
            self.rq.update(q_sp - rates[1], dt, deriv=None),
            self.rr.update(r_sp - rates[2], dt),
        ])
        motors = thrust + MIX @ np.clip(tau, -0.25, 0.25)
        return np.clip(motors, 0.0, 1.0)

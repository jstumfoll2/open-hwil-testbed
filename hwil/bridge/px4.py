"""PX4 in the loop over MAVLink (SITL over TCP now, HITL over serial in M8).

Simulation link (the same messages PX4 uses for SITL and HITL):
  sim -> PX4: HIL_SENSOR every frame, HIL_GPS at the GPS rate
  PX4 -> sim: HIL_ACTUATOR_CONTROLS (motor outputs 0..1, PX4 quad-x order)

SITL: PX4's ``simulator_mavlink`` module connects to us on TCP 4560. The run is
lockstep: PX4's clock is driven by the HIL_SENSOR timestamps, and after each
sensor message we wait for PX4's actuator reply before stepping JSBSim. The
time spent waiting is recorded as ``io_wait`` and is the seed of the M4/M8
end-to-end latency measurement.

Mission link: arm, takeoff and land go to PX4 as ordinary MAVLink commands on
the offboard API port (UDP 14540 in SITL; the same serial link in HITL).
"""

from __future__ import annotations

import math
import os
import struct
import time

import numpy as np

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from ..controllers import SensorFrame  # noqa: E402
from ..fdm import State  # noqa: E402
from ..mission import GOTO_DEFAULTS, MissionSequencer, ned_to_latlon  # noqa: E402

mavlink = mavutil.mavlink

# HIL_SENSOR fields_updated bits: accel(0-2), gyro(3-5), mag(6-8), abs pressure(9),
# diff pressure(10), pressure alt(11), temperature(12).
FIELDS_IMU_MAG_BARO = 0x1FFF

# PX4 parameters that adapt a stock SITL airframe (10016_none_iris) to this model
# and let it fly from MAVLink commands alone, with no RC transmitter.
SITL_PARAMS = {
    # F450 geometry: rotors at +-0.1651 m (see models/jsbsim/README.md).
    "CA_ROTOR0_PX": 0.1651, "CA_ROTOR0_PY": 0.1651,
    "CA_ROTOR1_PX": -0.1651, "CA_ROTOR1_PY": -0.1651,
    "CA_ROTOR2_PX": 0.1651, "CA_ROTOR2_PY": -0.1651,
    "CA_ROTOR3_PX": -0.1651, "CA_ROTOR3_PY": 0.1651,
    "MPC_THR_HOVER": 0.42,
    "COM_RC_IN_MODE": 4,      # no RC needed
    "NAV_RCL_ACT": 0,         # no failsafe on RC loss
    "NAV_DLL_ACT": 0,         # no failsafe on GCS link loss
    "COM_DISARM_LAND": 1.0,   # disarm 1 s after landing
}


class Px4Error(RuntimeError):
    pass


class Px4Controller:
    name = "px4"

    def __init__(self, scenario: dict, opts: dict):
        self.sc = scenario
        self.opts = opts
        self.mission = MissionSequencer(scenario["mission"], on_enter=self._on_phase)
        self.io_wait_ns = 0
        self.connect_timeout_s = float(opts.get("px4_connect_timeout_s", 120.0))
        self.reply_timeout_s = float(opts.get("px4_reply_timeout_s", 1.0))
        self.lockstep = opts.get("clock", "afap") == "afap" and not opts.get("px4_serial")
        self.motors = np.zeros(4)
        self.stats = {"frames": 0, "replies": 0, "reply_timeouts": 0, "commands": []}
        self.px4_armed = False
        self.autopilot_version = None
        self._pending: list[dict] = []
        self._connected = False
        self._version_requested = False
        self._params_sent = False

        if opts.get("px4_serial"):
            self.sim = mavutil.mavlink_connection(opts["px4_serial"], baud=int(opts["px4_baud"]),
                                                  source_system=1, source_component=51)
            self.gcs = self.sim
        else:
            self.sim = mavutil.mavlink_connection(f"tcpin:0.0.0.0:{int(opts['px4_sim_port'])}",
                                                  source_system=1, source_component=51)
            self.gcs = mavutil.mavlink_connection(opts["px4_gcs"], source_system=255, source_component=190)

    # ------------------------------------------------------------------ links
    def _wait_for_px4(self) -> None:
        deadline = time.monotonic() + self.connect_timeout_s
        while time.monotonic() < deadline:
            msg = self.sim.recv_match(blocking=True, timeout=0.5)
            if msg is not None and msg.get_type() == "HEARTBEAT":
                self._connected = True
                return
        raise Px4Error(f"PX4 did not connect within {self.connect_timeout_s:.0f} s "
                       f"(is PX4 SITL running with SYS_AUTOSTART=10016?)")

    def _poll_gcs(self) -> None:
        while True:
            msg = self.gcs.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.autopilot == mavlink.MAV_AUTOPILOT_PX4:
                self.px4_armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "COMMAND_ACK":
                for p in self._pending:
                    if p["cmd"] == msg.command and not p["done"]:
                        p["result"] = msg.result
                        if msg.result == mavlink.MAV_RESULT_ACCEPTED:
                            p["done"] = True
            elif t == "AUTOPILOT_VERSION":
                v = msg.flight_sw_version
                self.autopilot_version = f"{v >> 24 & 0xFF}.{v >> 16 & 0xFF}.{v >> 8 & 0xFF}"
            elif t == "STATUSTEXT":
                self.stats.setdefault("statustext", []).append(msg.text)

    def _set_params(self) -> None:
        for name, value in SITL_PARAMS.items():
            ptype = mavlink.MAV_PARAM_TYPE_INT32 if isinstance(value, int) else mavlink.MAV_PARAM_TYPE_REAL32
            if ptype == mavlink.MAV_PARAM_TYPE_INT32:
                # PX4 expects int params bit-cast into the float field.
                fval = struct.unpack("<f", struct.pack("<i", int(value)))[0]
            else:
                fval = float(value)
            self.gcs.mav.param_set_send(self.gcs.target_system, 1, name.encode(), fval, ptype)

    def _queue(self, cmd: int, params: list[float], label: str, t: float, as_int: bool = False) -> None:
        """Queue a command. ``as_int`` sends COMMAND_INT (params 5/6 are lat/lon in degE7)."""
        self._pending.append({"cmd": cmd, "params": params, "label": label, "done": False,
                              "next_try": t, "result": None, "tries": 0, "as_int": as_int})

    def _service_commands(self, t: float) -> None:
        if self.gcs.target_system == 0:
            return
        if not self._params_sent:
            self._params_sent = True
            self._set_params()
        if not self._version_requested:
            self._version_requested = True
            self.gcs.mav.command_long_send(self.gcs.target_system, 1, mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                           mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
        # Strictly in order: a command is only sent once every earlier one was accepted.
        p = next((p for p in self._pending if not p["done"]), None)
        if p is None or t < p["next_try"]:
            return
        p["tries"] += 1
        p["next_try"] = t + 1.0
        if p["as_int"]:
            q = p["params"]
            self.gcs.mav.command_int_send(self.gcs.target_system, 1, mavlink.MAV_FRAME_GLOBAL, p["cmd"], 0, 0,
                                          q[0], q[1], q[2], q[3], int(q[4]), int(q[5]), q[6])
        else:
            self.gcs.mav.command_long_send(self.gcs.target_system, 1, p["cmd"], 0, *p["params"])
        self.stats["commands"].append({"t": round(t, 3), "cmd": p["label"], "try": p["tries"],
                                       "last_result": p["result"]})

    def _on_phase(self, phase: str, spec: dict) -> None:
        self._phase_entry = (phase, spec)

    # ------------------------------------------------------------- main step
    def update(self, s: State, sensors: SensorFrame, dt: float) -> np.ndarray:
        if not self._connected:
            self._wait_for_px4()
        t_us = int(round(s.t * 1e6)) + 1  # PX4 dislikes a zero timestamp
        imu, baro, mag = sensors.imu, sensors.baro, sensors.mag
        self.sim.mav.hil_sensor_send(
            t_us,
            *[float(x) for x in imu.accel], *[float(x) for x in imu.gyro], *[float(x) for x in mag],
            float(baro.pressure_hpa), 0.0, float(baro.pressure_alt_m), float(baro.temperature_c),
            FIELDS_IMU_MAG_BARO, 0)
        if sensors.gps is not None:
            g = sensors.gps
            vel = g.vel_ned
            cog = (math.degrees(math.atan2(vel[1], vel[0])) % 360.0) * 100.0
            self.sim.mav.hil_gps_send(
                t_us, 3 if g.valid else 0,
                int(round(g.lat_deg * 1e7)), int(round(g.lon_deg * 1e7)), int(round(g.alt_msl_m * 1000)),
                int(g.eph_m * 100), int(g.epv_m * 100),
                int(round(math.hypot(vel[0], vel[1]) * 100)),
                int(round(vel[0] * 100)), int(round(vel[1] * 100)), int(round(vel[2] * 100)),
                int(cog), g.satellites)

        start = time.perf_counter_ns()
        got = self._read_actuators(blocking=self.lockstep)
        self.io_wait_ns += time.perf_counter_ns() - start
        self.stats["frames"] += 1
        if got:
            self.stats["replies"] += 1
        elif self.lockstep:
            self.stats["reply_timeouts"] += 1

        self._poll_gcs()
        self._mission_logic(s)
        self._service_commands(s.t)
        return self.motors.copy()

    def _read_actuators(self, blocking: bool) -> bool:
        deadline = time.monotonic() + self.reply_timeout_s
        got = False
        while True:
            timeout = max(0.0, deadline - time.monotonic()) if blocking and not got else 0.0
            msg = self.sim.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=timeout > 0, timeout=timeout)
            if msg is None:
                return got
            armed = bool(msg.mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.motors = np.clip(np.array(msg.controls[:4], dtype=float), 0.0, 1.0) if armed else np.zeros(4)
            got = True

    def _mission_logic(self, s: State) -> None:
        self.mission.update(s)
        entry = getattr(self, "_phase_entry", None)
        if entry is None:
            return
        self._phase_entry = None
        phase, spec = entry
        if phase == "takeoff":
            alt_msl = self.sc["initial"]["ground_alt_msl_m"] + float(spec["alt_m"])
            self._queue(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0], "arm", s.t + 0.5)
            self._queue(mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, float("nan"), float("nan"), float("nan"), alt_msl],
                        "takeoff", s.t + 0.5)
        elif phase == "goto":
            home = self.sc["initial"]
            lat, lon = ned_to_latlon(home["lat_deg"], home["lon_deg"], *self.mission.ne_target)
            alt_msl = home["ground_alt_msl_m"] + self.mission.alt_target
            speed = float(spec.get("speed_mps", GOTO_DEFAULTS["speed_mps"]))
            # MAV_CMD_DO_REPOSITION: speed, flags (1 = switch to reposition mode), -, yaw (NaN = keep)
            self._queue(mavlink.MAV_CMD_DO_REPOSITION,
                        [speed, 1, 0, float("nan"), round(lat * 1e7), round(lon * 1e7), alt_msl],
                        "goto", s.t, as_int=True)
        elif phase == "land":
            self._queue(mavlink.MAV_CMD_NAV_LAND, [0, 0, 0, float("nan"), float("nan"), float("nan"), float("nan")],
                        "land", s.t)

    def info(self) -> dict:
        return {"controller": self.name, "autopilot_version": self.autopilot_version,
                "lockstep": self.lockstep, "link_stats": {k: v for k, v in self.stats.items() if k != "statustext"},
                "px4_statustext": self.stats.get("statustext", [])[-50:],
                "sitl_params": SITL_PARAMS}

    def close(self) -> None:
        for link in {id(self.sim): self.sim, id(self.gcs): self.gcs}.values():
            try:
                link.close()
            except Exception:
                pass

"""The PX4 bridge against a fake PX4 that speaks the SITL MAVLink protocol.

No PX4 needed: a thread plays PX4's simulator_mavlink side (connects to TCP,
sends a heartbeat, answers every HIL_SENSOR with HIL_ACTUATOR_CONTROLS).
"""

import os
import socket
import threading
import time

import numpy as np

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from hwil import scenario as scn  # noqa: E402
from hwil.bridge.px4 import Px4Controller  # noqa: E402
from hwil.controllers import SensorFrame  # noqa: E402
from hwil.fdm import JsbsimFdm  # noqa: E402
from hwil.sensors import SensorSuite  # noqa: E402

mavlink = mavutil.mavlink


def _free_port(kind=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class FakePx4(threading.Thread):
    def __init__(self, port, frames, motor=0.55):
        super().__init__(daemon=True)
        self.port, self.frames, self.motor = port, frames, motor
        self.sensors, self.gps = [], []

    def run(self):
        for _ in range(100):
            try:
                link = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.port}", source_system=1, retries=0)
                break
            except OSError:
                time.sleep(0.05)
        link.mav.heartbeat_send(mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_PX4, 0, 0, 0)
        while len(self.sensors) < self.frames:
            msg = link.recv_match(type=["HIL_SENSOR", "HIL_GPS"], blocking=True, timeout=5)
            if msg is None:
                return
            if msg.get_type() == "HIL_GPS":
                self.gps.append(msg)
                continue
            self.sensors.append(msg)
            controls = [self.motor] * 4 + [0.0] * 12
            link.mav.hil_actuator_controls_send(msg.time_usec, controls,
                                                mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 1)
        link.close()


def test_sensor_out_actuators_in():
    sc = scn.resolve({"seed": 3})
    sim_port = _free_port()
    gcs_port = _free_port(socket.SOCK_DGRAM)
    ctrl = Px4Controller(sc, {"px4_sim_port": sim_port, "px4_gcs": f"udpin:127.0.0.1:{gcs_port}",
                              "clock": "afap", "px4_connect_timeout_s": 10})
    fake = FakePx4(sim_port, frames=50)
    fake.start()
    fdm, sensors = JsbsimFdm(sc), SensorSuite(sc)
    sent = []
    for _ in range(50):
        s = fdm.state()
        gps = sensors.gps(s) if sensors.gps_due(s.t) else None
        frame = SensorFrame(s.t, sensors.imu(s), sensors.baro(s), sensors.mag(s), gps)
        sent.append(frame)
        motors = ctrl.update(s, frame, fdm.dt)
        fdm.set_motors(motors)
        fdm.step()
    ctrl.close()
    fake.join(timeout=5)

    assert len(fake.sensors) == 50
    assert ctrl.stats["reply_timeouts"] == 0
    assert np.allclose(motors, 0.55)
    # Timestamps follow simulated time (lockstep) at the frame rate.
    dts = np.diff([m.time_usec for m in fake.sensors])
    assert np.all(dts == 4000)
    # Units: m/s^2, rad/s, gauss, hPa.
    m, f = fake.sensors[0], sent[0]
    assert np.allclose([m.xacc, m.yacc, m.zacc], f.imu.accel, atol=1e-5)
    assert np.allclose([m.xgyro, m.ygyro, m.zgyro], f.imu.gyro, atol=1e-6)
    assert np.allclose([m.xmag, m.ymag, m.zmag], f.mag, atol=1e-6)
    assert abs(m.abs_pressure - f.baro.pressure_hpa) < 1e-3
    assert m.fields_updated == 0x1FFF
    # GPS at 10 Hz: frames 0 and 25 in the first 50 frames at 250 Hz.
    assert len(fake.gps) == 2
    g = fake.gps[0]
    assert g.fix_type == 3
    assert abs(g.lat / 1e7 - sent[0].gps.lat_deg) < 1e-6

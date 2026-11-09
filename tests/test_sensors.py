import numpy as np

from hwil import scenario as scn
from hwil.fdm import State
from hwil.sensors import SensorSuite, isa_pressure_pa, pressure_altitude_m


def _state(alt=488.0):
    return State(t=0.0, pos_ned=np.zeros(3), vel_ned=np.zeros(3), euler=np.zeros(3),
                 rates_body=np.zeros(3), accel_body=np.array([0, 0, -9.80665]),
                 lat_deg=47.4, lon_deg=8.5, alt_msl_m=alt, alt_agl_m=0.15, wind_ned=np.zeros(3))


def test_isa_sea_level_and_inverse():
    assert abs(isa_pressure_pa(0.0) - 101325.0) < 1e-6
    for h in (0.0, 488.0, 3000.0):
        assert abs(pressure_altitude_m(isa_pressure_pa(h)) - h) < 1e-3


def test_sensor_noise_is_seeded():
    sc = scn.resolve({"seed": 42})
    a, b = SensorSuite(sc), SensorSuite(sc)
    s = _state()
    for _ in range(10):
        assert np.array_equal(a.imu(s).accel, b.imu(s).accel)
        assert a.baro(s).pressure_hpa == b.baro(s).pressure_hpa


def test_mag_level_north_matches_field():
    sc = scn.resolve({"sensors": {"mag": {"noise_gauss": 0.0}}})
    m = SensorSuite(sc).mag(_state())
    assert np.allclose(m, sc["sensors"]["mag"]["field_ned_gauss"])


def test_gps_rate():
    sc = scn.resolve({})
    s = SensorSuite(sc)
    due = sum(s.gps_due(k / 250.0) for k in range(250 * 5))
    assert due == 50

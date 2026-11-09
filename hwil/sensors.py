"""Simple, generic sensor models: IMU, barometer, magnetometer, GPS.

Each model draws its noise from its own seeded NumPy generator, so a run is
bit-for-bit repeatable from its seed. Models are deliberately plain (white
noise plus constant bias) and are meant to be swapped for richer ones later,
for example as FMUs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .fdm import State, rotation_body_to_ned

P0_PA = 101325.0
T0_K = 288.15
LAPSE = 0.0065


def isa_pressure_pa(alt_msl_m: float) -> float:
    """ISA troposphere pressure for a geometric altitude (good below 11 km)."""
    return P0_PA * (1.0 - LAPSE * alt_msl_m / T0_K) ** 5.25588


def isa_temperature_c(alt_msl_m: float) -> float:
    return T0_K - LAPSE * alt_msl_m - 273.15


def pressure_altitude_m(p_pa: float) -> float:
    return (T0_K / LAPSE) * (1.0 - (p_pa / P0_PA) ** (1.0 / 5.25588))


@dataclass
class ImuSample:
    accel: np.ndarray  # m/s^2, body
    gyro: np.ndarray   # rad/s, body


@dataclass
class BaroSample:
    pressure_hpa: float
    pressure_alt_m: float
    temperature_c: float


@dataclass
class GpsSample:
    valid: bool
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    vel_ned: np.ndarray
    eph_m: float
    epv_m: float
    satellites: int


class SensorSuite:
    def __init__(self, scenario: dict):
        cfg = scenario["sensors"]
        self.cfg = cfg
        streams = np.random.SeedSequence(int(scenario["seed"])).spawn(4)
        self.rng_imu, self.rng_baro, self.rng_mag, self.rng_gps = (np.random.default_rng(s) for s in streams)
        self.accel_bias = np.asarray(cfg["imu"]["accel_bias_mps2"], dtype=float)
        self.gyro_bias = np.asarray(cfg["imu"]["gyro_bias_rps"], dtype=float)
        self.mag_field = np.asarray(cfg["mag"]["field_ned_gauss"], dtype=float)
        self.gps_period = 1.0 / float(cfg["gps"]["rate_hz"])
        self._next_gps_t = 0.0
        self.extra_baro_bias_hpa = 0.0   # set by faults
        self.gps_denied = False          # set by faults

    def imu(self, s: State) -> ImuSample:
        c = self.cfg["imu"]
        accel = s.accel_body + self.accel_bias + self.rng_imu.normal(0, c["accel_noise_mps2"], 3)
        gyro = s.rates_body + self.gyro_bias + self.rng_imu.normal(0, c["gyro_noise_rps"], 3)
        return ImuSample(accel, gyro)

    def baro(self, s: State) -> BaroSample:
        c = self.cfg["baro"]
        p_hpa = isa_pressure_pa(s.alt_msl_m) / 100.0
        p_hpa += c["bias_hpa"] + self.extra_baro_bias_hpa + self.rng_baro.normal(0, c["noise_hpa"])
        return BaroSample(p_hpa, pressure_altitude_m(p_hpa * 100.0), isa_temperature_c(s.alt_msl_m))

    def mag(self, s: State) -> np.ndarray:
        r_bn = rotation_body_to_ned(*s.euler).T
        return r_bn @ self.mag_field + self.rng_mag.normal(0, self.cfg["mag"]["noise_gauss"], 3)

    def gps_due(self, t: float) -> bool:
        if t + 1e-9 >= self._next_gps_t:
            self._next_gps_t += self.gps_period
            return True
        return False

    def gps(self, s: State) -> GpsSample:
        c = self.cfg["gps"]
        n = self.rng_gps.normal(0, c["noise_h_m"], 2)
        dlat = math.degrees(n[0] / 6378137.0)
        dlon = math.degrees(n[1] / (6378137.0 * math.cos(math.radians(s.lat_deg))))
        alt = s.alt_msl_m + self.rng_gps.normal(0, c["noise_v_m"])
        vel = s.vel_ned + self.rng_gps.normal(0, c["noise_vel_mps"], 3)
        return GpsSample(
            valid=not self.gps_denied,
            lat_deg=s.lat_deg + dlat,
            lon_deg=s.lon_deg + dlon,
            alt_msl_m=alt,
            vel_ned=vel,
            eph_m=max(c["noise_h_m"], 0.1),
            epv_m=max(c["noise_v_m"], 0.1),
            satellites=int(c["satellites"]) if not self.gps_denied else 0,
        )

# Scenario file format

A scenario is a YAML file in `scenarios/`. Every key is optional; missing keys
take the defaults in `hwil/scenario.py`. The fully resolved scenario (with the
defaults filled in) is stored in each run's `manifest.json` and is what the
config hash covers. Check files with `hwil validate scenarios/*.yaml`.

```yaml
schema: 1
name: takeoff_hold_land
description: Free text.
seed: 1                      # seeds sensor noise and JSBSim turbulence

vehicle:
  model: F450-hil            # or F450 (upstream, no drag)

sim:
  rate_hz: 250               # 50..2000
  duration_s: 150            # hard stop
  stop_after_landed_s: 2.0   # stop early once landed and disarmed

initial:                     # home; defaults to the public PX4 SITL home
  lat_deg: 47.397742
  lon_deg: 8.545594
  ground_alt_msl_m: 488.0
  heading_deg: 0.0

wind:
  north_mps: 3.0             # direction the air moves toward
  east_mps: 1.0
  down_mps: 0.0
  turbulence_severity: 1     # 0 off, 1..7 = JSBSim MIL-SPEC light..severe

sensors:                     # 1-sigma white noise and constant biases
  imu:  {accel_noise_mps2: 0.02, gyro_noise_rps: 0.002,
         accel_bias_mps2: [0, 0, 0], gyro_bias_rps: [0, 0, 0]}
  baro: {noise_hpa: 0.01, bias_hpa: 0.0}
  mag:  {noise_gauss: 0.002, field_ned_gauss: [0.216, 0.016, 0.424]}
  gps:  {rate_hz: 10, noise_h_m: 0.3, noise_v_m: 0.5, noise_vel_mps: 0.05, satellites: 12}

mission:                     # run in order; takeoff must come first
  - {type: takeoff, alt_m: 20}     # climbs at home at 1.5 m/s (builtin) or PX4's takeoff speed
  - {type: hold, duration_s: 60}   # position hold at the current commanded point
  - {type: goto, north_m: 30, east_m: 0, alt_m: 20,   # metres from home; alt optional
     speed_mps: 3, accept_radius_m: 1.0, timeout_s: 90}
  - {type: land}                   # descend at the current commanded point, disarm on the ground

faults:                      # active from t_s for duration_s (or until the end)
  - {type: motor_efficiency, motor: 0, factor: 0.85, t_s: 40}
  - {type: gps_dropout, t_s: 50, duration_s: 5}
  - {type: baro_bias, bias_hpa: 0.5, t_s: 60}

requirements:
  - id: REQ-ALT-001
    text: After 30 s, altitude stays within 5 m of the commanded loiter altitude while loitering.
    stl: "eventually[30s,30s](always((loiter > 0) -> (abs(alt_m - alt_cmd_m) <= 5.0)))"
```

## Requirements in STL

Requirements are evaluated offline with RTAMT (discrete time, log resampled to
10 Hz). A requirement passes when its robustness at t = 0 is positive. The
robustness is the margin in the spec's units: 4.6 for the altitude spec above
means the worst error was 0.4 m against a 5 m limit.

Signals available to a spec: every column of `log.csv`
(`hwil.executive.LOG_COLUMNS`), plus:

| Signal | Meaning |
|---|---|
| `loiter` | true during the hold phase |
| `landed` | true once landed and disarmed |
| `alt_err_m` | `alt_m - alt_cmd_m` |
| `hspeed_mps` | horizontal ground speed |
| `hdist_m` | horizontal distance from home |
| `hdist_cmd_m` | horizontal distance from the commanded point (goto target or hold point) |
| `pos_err_m` | 3-D distance from the commanded point |
| `goto` | true during a goto phase |
| `tilt_deg` | tilt of the body z axis from vertical |

`loiter`, `landed` and `goto` are +10⁶ when true and -10⁶ when false. Write
them as `loiter > 0`. The large magnitude keeps a guard like `(loiter > 0) -> phi`
from capping the robustness, so the reported number is phi's real margin. A
spec made only of these booleans (for example
`eventually(always[0s,100s](loiter > 0))`) reports 1000000 when it passes.

A guarded requirement whose guard never held (the vehicle never loitered), or
one whose time window starts after the run ended, is reported as **not
exercised** and does not pass.

A `goto` ends when the vehicle is within `accept_radius_m` of the target and
slower than 0.5 m/s, or after `timeout_s` (recorded as a `goto_timeout` event).
With PX4 it is sent as `MAV_CMD_DO_REPOSITION`.

Useful patterns:

| Intent | STL |
|---|---|
| From 30 s onward, always P | `eventually[30s,30s](always(P))` |
| P whenever loitering | `always((loiter > 0) -> P)` |
| Eventually lands near home | `eventually((landed > 0) and (hdist_m <= 3.0))` |
| P holds for at least 5 s at some point | `eventually(always[0s,5s](P))` |

Keep time bounds short or anchored as above: RTAMT's offline evaluation of a
bounded `always[a,b]` gets slow when `b` is many times the run length.

## Running many scenarios

```bash
hwil suite scenarios/*.yaml --jobs 4                               # builtin, in parallel
hwil suite scenarios/*.yaml --controller px4 --px4-dir ~/src/PX4-Autopilot
```

The suite writes `suite.json` and `suite.html` (one row per scenario, every
requirement with its margin, links to each run's report). With PX4 a fresh SITL
instance is started for every scenario.

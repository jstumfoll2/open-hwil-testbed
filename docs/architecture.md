# Architecture

## The frame

Every frame (4 ms at the default 250 Hz) the executive does the same five things,
in this order:

```mermaid
sequenceDiagram
    participant X as Executive
    participant F as JSBSim (F450)
    participant S as Sensor models
    participant C as Controller (builtin or PX4)
    participant L as Logger
    X->>F: read state (NED, attitude, rates, specific force)
    X->>S: IMU, baro, mag every frame; GPS at 10 Hz
    X->>C: state + sensor frame
    C-->>X: 4 motor commands (0..1)
    X->>X: apply timed faults
    X->>F: set motors, step dt
    X->>L: one log row + one timing row
```

| Module | Role |
|---|---|
| `hwil/executive.py` | Fixed-rate loop, `afap` or `realtime` clock, overrun and jitter counters, fault injection |
| `hwil/fdm.py` | JSBSim wrapper. SI units, NED frame, flat-earth position relative to home |
| `hwil/models.py` | Builds the `F450-hil` model from the installed JSBSim F450 plus `models/jsbsim/F450-hil/` |
| `hwil/sensors.py` | IMU, baro (ISA), mag (fixed NED field), GPS. White noise + bias, seeded per sensor |
| `hwil/mission.py` | Mission phases: preflight, takeoff, hold, land, landed |
| `hwil/controllers/builtin.py` | Reference cascaded PID autopilot (uses true state) |
| `hwil/bridge/px4.py` | PX4 over MAVLink: HIL messages, lockstep, mission commands, SITL parameters |
| `hwil/requirements.py` | RTAMT STL grading of a log |
| `hwil/manifest.py`, `hwil/run.py` | Run folder, manifest, reproduce |
| `hwil/report.py` | PNG plots and `report.html` |

## Clocks

- **`afap`** (as fast as possible): no sleeping. With the builtin controller the
  run is deterministic and bit-reproducible. With PX4 SITL it is lockstep: PX4's
  clock follows the `HIL_SENSOR` timestamps, and the executive waits for PX4's
  `HIL_ACTUATOR_CONTROLS` reply before stepping JSBSim. PX4 runs are close to,
  but not bit-for-bit, repeatable because PX4 itself has threads.
- **`realtime`**: each frame starts on an absolute deadline. `timing.csv` records
  how late each frame started, its compute time and whether it overran the next
  deadline. `--rt-priority` and `--rt-cpu` request `SCHED_FIFO` and CPU pinning.
  Python is the M1 baseline; M2 measures it on PREEMPT_RT, and a C++ or Trick
  executive replaces it if it cannot hold the frame budget.

## PX4 link

| Direction | Message | Rate | Notes |
|---|---|---|---|
| sim to PX4 | `HIL_SENSOR` | every frame | accel m/s², gyro rad/s, mag gauss, pressure hPa, `fields_updated = 0x1FFF` |
| sim to PX4 | `HIL_GPS` | 10 Hz | fix type 3, or 0 during a GPS dropout fault |
| PX4 to sim | `HIL_ACTUATOR_CONTROLS` | every frame | `controls[0..3]` are motor outputs 0..1, zero when disarmed |
| sim to PX4 | `COMMAND_LONG` | on phase change | arm, `NAV_TAKEOFF`, `NAV_LAND`, retried each second until accepted |
| sim to PX4 | `PARAM_SET` | once | `SITL_PARAMS`: F450 rotor geometry, hover thrust, no-RC failsafes |

SITL uses TCP 4560 for the HIL link (PX4 connects to us) and UDP 14540 for
commands. HITL (M3) will use a single USB serial link for both; the bridge
already accepts `--px4-serial /dev/ttyACM0`, but that path has not been tested
on hardware yet.

## Vehicle model

`F450-hil` is the public JSBSim F450 quadcopter (1.4 kg, 9.4 in props). The
upstream model has zero aerodynamics, so wind would not move it; the overlay
adds one isotropic drag term (`CD·S ≈ 0.05 m²`). Its four ESC inputs are driven
directly, so the model's own stability augmentation is bypassed and the
controller in the loop does all the work. Motor order is PX4 quad-x: front
right, aft left, front left, aft right.

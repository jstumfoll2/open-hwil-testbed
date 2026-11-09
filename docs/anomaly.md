# Anomaly detection

STL requirements check what someone thought to write down. The anomaly detector
looks for anything that differs from normal flight, whether or not a
requirement covers it. It learns "normal" from nominal runs, then flags any
stretch of a new run that falls outside that envelope and names the signals
responsible.

```bash
hwil anomaly score runs/<run>                 # uses models/anomaly/baseline_builtin.json
hwil anomaly score runs/mc_<batch>            # every run in a batch or suite folder
hwil anomaly fit runs/<nominal runs> --out my_baseline.json
hwil anomaly score runs/<run> --baseline my_baseline.json --z 7 --min-s 1
```

`score` writes `anomaly.html` and `anomaly.json` into each run folder. With
`--fail-on-anomaly`, it exits 1 if any run is flagged, so a CI job can use it as
a gate.

## How it works

The log is averaged into 0.1 s blocks. Each feature is a residual that should
stay near zero whatever the mission is doing:

| Feature | What it is | What it catches |
|---|---|---|
| `torque_roll`, `torque_pitch`, `torque_yaw` | Roll, pitch and yaw torque the controller asks the mixer for (pseudo-inverse of the quad-x mix), 2 s mean | A weak motor or propeller, CG shift, a bent arm |
| `thrust_resid` | Collective command minus what a thrust curve fitted on nominal flights needs for the measured lift, 2 s mean | Loss of overall thrust, a heavier vehicle |
| `baro_resid` | Baro altitude minus true altitude, with the start-up offset removed, 2 s mean | Baro drift or bias |
| `gps_invalid` | Share of the last second without a valid GPS fix | GPS dropouts |
| `rate_noise` | Std of body rates over 1 s | Vibration, turbulence beyond the norm |

`fit` stores, per feature and per phase group (ground, climb or descent,
cruise), the median and a robust spread taken from the 0.1 and 99.9
percentiles. Each feature also has a floor on the spread. `score` turns every
block into a z-score per feature. A segment is flagged where any |z| stays above
7 for at least 1 s.

Altitude and position tracking errors are left to the STL requirements, because
they depend on the mission (a 20 m altitude step is a big error that is also
correct).

The motor columns `m0..m3` in `log.csv` hold the controller's command, as an
autopilot log would. A `motor_efficiency` fault changes what the motor delivers,
not the logged command, so a weak motor shows up as a steady torque demand.

## Results (builtin controller)

The shipped baseline was fitted on 68 nominal runs: the eight fault-free
scenarios, plus 60 Monte Carlo runs of `hold_wind_and_noise` (wind up to
±6 m/s per axis, turbulence severity 0 to 3, sensor noise). To regenerate it,
run `tools/fit_anomaly_baseline.sh`.

**Clean runs.** None of 40 held-out runs of the same dispersions (seed 5000,
not used for fitting) was flagged. The largest score in any of them was
|z| = 7.5, for less than the 1 s minimum.

**Fault scenarios:**

| Scenario | Fault | Flagged | Leading signal |
|---|---|---|---|
| `motor_degraded` | motor 1 at 75 % from 30 s | 30.8 s onward | torque, z ≈ 18 |
| `gps_dropout_hold` | no GPS 40 to 50 s | 40.4 to 50.7 s | gps_invalid, z = 20 |
| `baro_bias_step` | +0.6 hPa from 40 s | 40.4 s onward | baro_resid, z = 35 |
| `hold_with_faults` | motor 2 at 85 % (30 s), GPS (45 s), baro (55 s) | 31.5 s onward | all three in turn |
| `imu_bias` | constant IMU bias | not flagged | see below |

**Motor-loss sweep.** Over 80 runs of `motor_loss_sweep` (one motor at 35 to
100 % efficiency, random motor and onset, wind up to ±4 m/s), the detector:

| Motor efficiency | Runs | Detected | Median delay | Failed an STL requirement |
|---|---|---|---|---|
| 35 to 60 % | 32 | 32 | 0.04 s | 6 |
| 60 to 80 % | 23 | 23 | 0.7 s | 0 |
| 80 to 90 % | 15 | 15 | 1.6 s | 0 |
| 90 to 100 % | 10 | 0 | n/a | 0 |

It flags every loss of about 12 % or more, and it raised no flag before fault
onset. In 64 of the 80 runs, the detector caught the fault while every
requirement still passed. Those are degraded vehicles the STL suite alone would
call healthy.

## Limits

- **Builtin controller only.** The shipped baseline is fitted on runs with
  the builtin controller. PX4 flies differently, so a PX4 baseline needs PX4
  runs (`hwil anomaly fit` on a PX4 suite or Monte Carlo batch). The scorer
  warns when a run's controller does not match the baseline.
- **IMU bias is invisible here.** The builtin controller flies on true
  state, and the log records true rates and accelerations, so an IMU bias has no
  effect to detect. With PX4, the bias would show up through its effect on
  the flight.
- **The baseline only knows the conditions it was fitted on.** A run in
  conditions outside the nominal set, such as stronger wind or a different
  airframe, can be flagged without a fault. Refit with nominal runs covering
  the new envelope.
- **Simple by design.** Per-feature envelopes are easy to explain and
  check by hand. A multivariate model could find subtler patterns, at the
  cost of explanations that are harder to trust.

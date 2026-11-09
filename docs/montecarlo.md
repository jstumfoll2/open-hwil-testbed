# Monte Carlo batches

`hwil montecarlo` flies one scenario many times with dispersed parameters and
reports pass rates with confidence intervals, margin statistics, and which
parameter values the failures cluster at.

```bash
hwil montecarlo montecarlo/hold_wind_and_noise.yaml --jobs 4          # builtin, 200 runs, ~5 min on 4 cores
hwil montecarlo montecarlo/motor_loss_sweep.yaml --runs 20 \
     --controller px4 --px4-dir ~/src/PX4-Autopilot                     # PX4 SITL, one fresh instance per run
```

## Config file

```yaml
name: motor_loss_sweep
scenario: ../scenarios/motor_degraded.yaml   # relative to this file
runs: 200
seed: 2000
dispersions:                                 # dotted path into the resolved scenario
  faults[0].motor:   {dist: choice, values: [0, 1, 2, 3]}
  faults[0].factor:  {dist: uniform, low: 0.35, high: 1.0}
  wind.north_mps:    {dist: uniform, low: -4.0, high: 4.0}
  sensors.imu.accel_noise_mps2: {dist: normal, mean: 0.02, std: 0.01, min: 0.0}
```

Distributions: `uniform` (`low`, `high`), `normal` (`mean`, `std`, optional
`min`/`max` clamps), `choice` (`values`), `constant` (`value`). A path that
does not exist in the scenario is an error, so a typo cannot silently do
nothing.

## Reproducibility

- Dispersed values come from one generator seeded with `seed`, so the same
  config always produces the same set of cases, whatever `--jobs` is.
- Run `i` gets scenario seed `seed + i` for sensor noise and turbulence.
- Every run is a normal run folder with its own `manifest.json` holding the
  fully dispersed scenario. The report lists a `hwil reproduce` line for each
  failed run, and the replay is bit-identical for builtin runs.

## Outputs

| File | Contents |
|---|---|
| `montecarlo.html` | pass rate with 95 % Wilson interval, per-requirement margins (min, 5th percentile, median) with histograms, pass/fail strip per dispersed parameter, failed runs with their values and replay commands |
| `montecarlo.json` | the same summary, machine-readable |
| `montecarlo.csv` | one row per run: dispersed values, pass/fail, robustness per requirement |
| `run0000/` ... | the individual run folders (no plots, to save time and space) |

## First results (builtin controller, 2026-11-09)

| Batch | Runs | Passed | Finding |
|---|---|---|---|
| `hold_wind_and_noise` | 200 | 200 (95 % CI 98.1 to 100 %) | Wind up to ±6 m/s per axis, turbulence 0 to 3 and 0.5x to 3x sensor noise never broke the hold. Smallest margins: landing 2.0 m, position hold 2.7 m. |
| `motor_loss_sweep` | 200 | 179 (84.5 to 93.0 %) | Every failure had a motor at 42 % efficiency or less, and every run at 43 % or more passed, whichever motor failed and whatever the wind. The strip chart for `faults[0].factor` shows the edge. |

A first PX4 SITL batch of the same sweep (10 runs) shows the same edge with a
different failure mode. Below about 40 % efficiency PX4 does not flip over like
the builtin controller. It sinks about 12 m below the commanded altitude, fails
REQ-MOT-001, and still lands upright near home.

These results describe the controllers on the simplified F450 model, not a
real vehicle.

#!/usr/bin/env bash
# Regenerate models/anomaly/baseline_builtin.json from nominal runs:
# the eight fault-free scenarios plus 60 Monte Carlo runs of wind,
# turbulence and sensor noise. Takes a few minutes with 4 jobs.
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=${1:-runs/anomaly_train}
JOBS=${JOBS:-4}
rm -rf "$OUT"
hwil suite scenarios/{takeoff_hold_land,waypoint_square,altitude_steps,crosswind_hold,long_hold_60m,noisy_sensors,high_elevation_home,out_and_back_fast}.yaml \
  --jobs "$JOBS" --no-plots --out "$OUT"
hwil montecarlo montecarlo/hold_wind_and_noise.yaml --runs 60 --jobs "$JOBS" --out "$OUT"
hwil anomaly fit "$OUT" --out models/anomaly/baseline_builtin.json

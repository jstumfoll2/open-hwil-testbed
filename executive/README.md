# executive/

`hwil_exec` is the compiled fixed-rate executive. It is a small C++17 loop that runs
JSBSim (the C++ library) and a C++ port of the builtin reference controller.

The frame loop works like this:

- It sleeps with `clock_nanosleep` on absolute deadlines.
- With `--clock realtime` it can use `SCHED_FIFO`, `mlockall` and CPU pinning.
- All memory is allocated before the loop starts.

It writes the same `log.csv`, `timing.csv` and `events.csv` columns as the Python
executive (`hwil/executive.py`), so the pytest harness and the RTAMT requirements
grade both executives unchanged.

You normally don't call it directly. `hwil run --executive cpp` writes its config,
starts it, and then does the grading, plots and manifest.

## Build

You need JSBSim 1.3.3 built from source. The pip wheel has no C++ headers.

```bash
git clone --depth 1 --branch v1.3.3 https://github.com/JSBSim-Team/jsbsim.git
cmake -S jsbsim -B jsbsim/build -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_PYTHON_MODULE=OFF -DBUILD_DOCS=OFF -DCMAKE_INSTALL_PREFIX=$HOME/.local/jsbsim
cmake --build jsbsim/build -j4 && cmake --install jsbsim/build

cmake -S executive -B executive/build -DJSBSIM_ROOT=$HOME/.local/jsbsim
cmake --build executive/build -j4
```

The binary lands in `executive/build/hwil_exec`. To use a binary somewhere else,
set `HWIL_EXEC_BIN`.

## Use

```bash
hwil run scenarios/takeoff_hold_land.yaml --executive cpp
hwil run scenarios/long_hold_60m.yaml --executive cpp --clock realtime --rt-cpu 2
pytest tests/test_cpp_executive.py   # cross-checks against the Python executive
```

## Scope and limits

- **Builtin controller only.** PX4 still runs through the Python bridge
  (`hwil/bridge/px4.py`).
- **No sensor models yet.** `baro_alt_m` is the noise-free MSL altitude. The
  `baro_bias` and `imu_bias` faults, and sensor noise, are Python-only. The
  `motor_efficiency` and `gps_dropout` faults work in both executives.
- **Matches the Python executive.** In the air, the two flights agree to about
  1e-12. They differ slightly after touchdown, where gear contact amplifies
  floating-point differences. `tests/test_cpp_executive.py` checks both
  tolerances.

NASA Trick (M6, optional) would sit beside this, reading the same scenarios.

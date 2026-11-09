# Setup guide

Three layers, in the order you need them:

1. [Python environment and JSBSim](#1-python-environment-and-jsbsim) (enough for SIL with the builtin controller and for CI)
2. [PX4 SITL](#2-px4-sitl) (M1 with PX4 in the loop)
3. [PREEMPT_RT kernel and timing baseline](#3-preempt_rt-kernel-and-timing-baseline) (M7, needs a dedicated Linux PC)

Tested on Ubuntu 24.04 LTS (x86-64). Other recent Linux distributions should work.

## 1. Python environment and JSBSim

Use **Python 3.10, 3.11 or 3.12**. RTAMT pins `antlr4-python3-runtime==4.7`,
which imports `typing.io` and therefore fails on Python 3.13+.

```bash
sudo apt install python3.12-venv git
git clone https://github.com/jstumfoll2/open-hwil-testbed
cd open-hwil-testbed
python3.12 -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"
pytest
```

The `jsbsim` package on PyPI ships prebuilt wheels with the JSBSim C++ engine
and its aircraft data, including the F450 used here. Nothing else to build.

Building JSBSim from source is only needed to use the C++ library directly
(for example from a C++ or Trick executive in M4 or M6):

```bash
sudo apt install build-essential cmake
git clone --depth 1 --branch v1.3.3 https://github.com/JSBSim-Team/jsbsim.git
cd jsbsim && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j"$(nproc)"
sudo make install          # installs libJSBSim and the JSBSim CLI
```

Model files are cached in `~/.cache/hwil-testbed/` (set `HWIL_CACHE` to move it).

## 2. PX4 SITL

```bash
mkdir -p ~/src && cd ~/src
git clone --branch v1.15.4 --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools   # toolchain and Python deps, no Gazebo
make px4_sitl_default                                     # about 10 min on 4 cores
```

Notes:

- `--no-sim-tools` skips Gazebo. This testbed is the simulator, so PX4 only
  needs its `simulator_mavlink` module, which is in the default SITL build.
- If you make a **shallow** clone (`--depth 1 --shallow-submodules`) the build
  stops in `px_update_git_header.py` with `IndexError` because the NuttX
  submodule has no tags. Fix it with
  `git -C platforms/nuttx/NuttX/nuttx tag nuttx-11.0.0` and build again.
- The PX4 Python requirements are in `Tools/setup/requirements.txt` if you
  prefer a venv to `ubuntu.sh`.

Run a scenario with PX4 in the loop:

```bash
cd ~/open-hwil-testbed
PX4_DIR=~/src/PX4-Autopilot ci/run_px4_sitl.sh > px4.log 2>&1 &
hwil run scenarios/takeoff_hold_land.yaml --controller px4
kill %1
```

`ci/run_px4_sitl.sh` starts PX4 headless with the generic `none_iris` airframe
(`SYS_AUTOSTART=10016`) in a fresh working directory, so parameters never
carry over between runs. Once PX4 connects, the bridge sets the F450 rotor
geometry, hover thrust and no-RC failsafe parameters (`SITL_PARAMS` in
`hwil/bridge/px4.py`). Expect `Arming denied: Resolve system health failures
first` in `px4.log` for the first second or two while the EKF converges; the
bridge retries arming once per second.

Optional: QGroundControl on the same machine connects automatically on UDP
14550 and shows the vehicle.

## 3. PREEMPT_RT kernel and timing baseline

### Get a real-time kernel

Pick one:

- **Ubuntu Pro real-time kernel** (Ubuntu 22.04 / 24.04, Pro is free for
  personal use on a few machines):
  ```bash
  sudo pro attach <your-token>
  sudo pro enable realtime-kernel
  sudo reboot
  ```
- **Debian 13 or another distro with an `-rt` kernel package**, for example
  `sudo apt install linux-image-rt-amd64` on Debian.
- **Build a mainline kernel.** PREEMPT_RT is in mainline Linux since 6.12; build
  with `CONFIG_PREEMPT_RT=y` (General setup, Preemption Model, Fully
  Preemptible Kernel).

Check it:

```bash
uname -v                     # contains PREEMPT_RT
cat /sys/kernel/realtime     # prints 1
```

Every run manifest records `platform.preempt_rt`, so runs on a stock kernel
are easy to tell apart.

### Isolate a CPU for the executive

Example for a 4-core machine, keeping CPU 3 for the loop. In
`/etc/default/grub`:

```
GRUB_CMDLINE_LINUX_DEFAULT="quiet isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2"
```

Then `sudo update-grub && sudo reboot`. Also worth doing on a bench machine:
set the CPU governor to `performance`, disable deep C-states in the BIOS, and
stop `irqbalance` (`sudo systemctl disable --now irqbalance`).

### Allow real-time priority without root

```bash
sudo groupadd -f realtime && sudo usermod -aG realtime "$USER"
printf '@realtime - rtprio 95\n@realtime - memlock unlimited\n' | sudo tee /etc/security/limits.d/99-realtime.conf
# log out and back in
```

### Baseline with cyclictest

```bash
sudo apt install rt-tests
sudo cyclictest --mlockall --priority=90 --interval=1000 --affinity=3 \
     --duration=30m --histogram=400 --quiet > cyclictest_cpu3.txt
```

Record the max latency. On a tuned PREEMPT_RT desktop it is typically tens of
microseconds; on a stock kernel it can reach milliseconds. Save the output next
to the M7 timing report.

### In-loop timing

```bash
hwil run scenarios/takeoff_hold_land.yaml --clock realtime --rt-priority 80 --rt-cpu 3
```

`timing.csv` and `timing.png` in the run folder hold per-frame start
lateness, compute time and overruns, and `manifest.json` summarizes them
(`results.timing`). M7 extends this to a 30-minute soak and a jitter report.

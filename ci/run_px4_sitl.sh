#!/usr/bin/env bash
# Start a headless PX4 SITL instance that waits for this testbed on TCP 4560.
#
#   PX4_DIR=~/src/PX4-Autopilot ci/run_px4_sitl.sh            # foreground
#   PX4_DIR=... ci/run_px4_sitl.sh > px4.log 2>&1 &             # background
#
# Uses the generic "none_iris" SITL airframe (SYS_AUTOSTART=10016), which has no
# simulator of its own and simply connects to whatever listens on port 4560.
# The testbed then sets the rotor geometry and failsafe parameters it needs
# (see SITL_PARAMS in hwil/bridge/px4.py). Each start uses a fresh working
# directory so parameters from an earlier run never leak into the next one.
set -euo pipefail
: "${PX4_DIR:?set PX4_DIR to your PX4-Autopilot checkout (built with make px4_sitl_default)}"
BUILD="${PX4_DIR}/build/px4_sitl_default"
WORK="${PX4_WORK_DIR:-$(mktemp -d -t px4-sitl-XXXXXX)}"
mkdir -p "${WORK}"
echo "PX4 SITL working dir: ${WORK}" >&2
export PX4_SYS_AUTOSTART=10016
export PX4_SIM_MODEL=none_iris
exec "${BUILD}/bin/px4" -d -w "${WORK}" "${BUILD}/etc"

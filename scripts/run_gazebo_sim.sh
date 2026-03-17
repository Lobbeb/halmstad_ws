#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD="${1:-warehouse}"
GUI="${GUI:-true}"
ENABLE_WSL_SOFTWARE_RENDERING="${ENABLE_WSL_SOFTWARE_RENDERING:-auto}"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"

if [ "$#" -gt 0 ]; then
  shift
fi

if [ "$#" -gt 0 ] && { [ "$1" = "true" ] || [ "$1" = "false" ]; }; then
  GUI="$1"
  shift
fi

cleanup() {
  if [ -f "$SIM_PID_FILE" ] && [ "$(cat "$SIM_PID_FILE" 2>/dev/null || true)" = "$$" ]; then
    rm -f "$SIM_PID_FILE"
    rm -f "$SIM_WORLD_FILE"
  fi
}

trap cleanup EXIT

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash

cd "$WS_ROOT"
# Limit package discovery to the ROS workspace source tree.
# This avoids accidentally building reference repos or notes stored elsewhere
# under the workspace root.
colcon build --symlink-install --base-paths src

source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

mkdir -p "$STATE_DIR"
printf '%s\n' "$$" > "$SIM_PID_FILE"
printf '%s\n' "$WORLD" > "$SIM_WORLD_FILE"

# Gazebo + Ogre can crash on some WSL GPU driver stacks.
# Default to software rendering only on WSL unless explicitly disabled.
if [ "$ENABLE_WSL_SOFTWARE_RENDERING" = "auto" ]; then
  if [ -n "${WSL_INTEROP:-}" ] || grep -qi microsoft /proc/version 2>/dev/null; then
    if [ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
      export LIBGL_ALWAYS_SOFTWARE=1
      echo "[run_gazebo_sim] WSL detected: using LIBGL_ALWAYS_SOFTWARE=1 for stability"
    fi
  fi
elif [ "$ENABLE_WSL_SOFTWARE_RENDERING" = "true" ]; then
  if [ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    echo "[run_gazebo_sim] Forced software rendering: LIBGL_ALWAYS_SOFTWARE=1"
  fi
fi

ros2 launch lrs_halmstad managed_clearpath_sim.launch.py \
  world:="$WORLD" \
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath" \
  use_sim_time:=true \
  gui:="$GUI" \
  "$@"

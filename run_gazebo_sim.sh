#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-warehouse}"
GUI="${GUI:-true}"
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
colcon build --symlink-install

source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

mkdir -p "$STATE_DIR"
printf '%s\n' "$$" > "$SIM_PID_FILE"
printf '%s\n' "$WORLD" > "$SIM_WORLD_FILE"

ros2 launch lrs_halmstad managed_clearpath_sim.launch.py \
  world:="$WORLD" \
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath" \
  use_sim_time:=true \
  gui:="$GUI" \
  "$@"

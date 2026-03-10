#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
DEFAULT_WORLD="warehouse"
WORLD="$DEFAULT_WORLD"
MAP_PATH=""

if [ -f "$SIM_WORLD_FILE" ]; then
  sim_world="$(cat "$SIM_WORLD_FILE" 2>/dev/null || true)"
  if [ -n "$sim_world" ]; then
    WORLD="$sim_world"
  fi
fi

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  MAP_PATH="$1"
  shift
fi

if [ -z "$MAP_PATH" ]; then
  case "$WORLD" in
    warehouse*)
      MAP_PATH="/opt/ros/jazzy/share/clearpath_nav2_demos/maps/warehouse.yaml"
      ;;
    *)
      if [ -f "$WS_ROOT/maps/${WORLD}.yaml" ]; then
        MAP_PATH="$WS_ROOT/maps/${WORLD}.yaml"
      elif [ -f "$WS_ROOT/maps/${WORLD}_manual.yaml" ]; then
        MAP_PATH="$WS_ROOT/maps/${WORLD}_manual.yaml"
      else
        echo "[run_localization] No default map found for world '$WORLD'. Pass an explicit map path as the second argument." >&2
        exit 1
      fi
      ;;
  esac
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

ros2 launch clearpath_nav2_demos localization.launch.py \
  use_sim_time:=true \
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath" \
  scan_topic:=/a201_0000/sensors/lidar2d_0/scan \
  map:="$MAP_PATH" \
  "$@"

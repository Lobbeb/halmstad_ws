#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
DEFAULT_WORLD="warehouse"
WORLD="$DEFAULT_WORLD"
MAP_PATH=""
LOCAL_LOCALIZATION_LAUNCH="$WS_ROOT/src/lrs_halmstad/launch/localization_with_params.launch.py"

source "$SCRIPT_DIR/lidar_mode_common.sh"

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

lidar_mode_parse_args 2d "$@"
USE_POINTCLOUD_TO_LASERSCAN="false"

if [ -n "$LIDAR_POINTCLOUD_TOPIC" ] && [ "$LIDAR_SCAN_TOPIC" = "$(lidar_mode_scan_topic 3d)" ]; then
  USE_POINTCLOUD_TO_LASERSCAN="true"
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

ros2 launch "$LOCAL_LOCALIZATION_LAUNCH" \
  use_sim_time:=true \
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath" \
  scan_topic:="$LIDAR_SCAN_TOPIC" \
  pointcloud_topic:="$LIDAR_POINTCLOUD_TOPIC" \
  use_pointcloud_to_laserscan:="$USE_POINTCLOUD_TO_LASERSCAN" \
  map:="$MAP_PATH" \
  "${LIDAR_REMAINING_ARGS[@]}"

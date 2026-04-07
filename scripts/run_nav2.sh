#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
TMP_NAV2_PARAMS="$STATE_DIR/nav2_with_map_updates.yaml"
BASE_NAV2_PARAMS="/opt/ros/jazzy/share/clearpath_nav2_demos/config/a200/nav2.yaml"
LOCAL_NAV2_LAUNCH="$WS_ROOT/src/lrs_halmstad/launch/nav2_with_updates.launch.py"

source "$SCRIPT_DIR/lidar_mode_common.sh"

lidar_mode_parse_args 2d "$@"
USE_POINTCLOUD_TO_LASERSCAN="false"

if [ -n "$LIDAR_POINTCLOUD_TOPIC" ] && [ "$LIDAR_SCAN_TOPIC" = "$(lidar_mode_scan_topic 3d)" ]; then
  USE_POINTCLOUD_TO_LASERSCAN="true"
fi

mkdir -p "$STATE_DIR"

awk '
  {
    print
    if ($0 ~ /^[[:space:]]*plugin: "nav2_costmap_2d::StaticLayer"$/) {
      match($0, /^ */)
      indent = substr($0, 1, RLENGTH)
      print indent "subscribe_to_updates: true"
    }
  }
' "$BASE_NAV2_PARAMS" > "$TMP_NAV2_PARAMS"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

ros2 launch "$LOCAL_NAV2_LAUNCH" \
  use_sim_time:=true \
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath" \
  scan_topic:="$LIDAR_SCAN_TOPIC" \
  pointcloud_topic:="$LIDAR_POINTCLOUD_TOPIC" \
  use_pointcloud_to_laserscan:="$USE_POINTCLOUD_TO_LASERSCAN" \
  params_file:="$TMP_NAV2_PARAMS" \
  "${LIDAR_REMAINING_ARGS[@]}"

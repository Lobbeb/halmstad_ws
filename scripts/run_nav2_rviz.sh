#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_RVIZ_CONFIG="$WS_ROOT/src/lrs_halmstad/config/nav2_namespaced_waypoints.rviz"

source "$SCRIPT_DIR/lidar_mode_common.sh"

lidar_mode_parse_args 2d "$@"

RVIZ_CONFIG="$BASE_RVIZ_CONFIG"
if [ "$LIDAR_MODE" = "2d" ] || [ "$LIDAR_SCAN_TOPIC" != "$(lidar_mode_raw_scan_topic 3d)" ]; then
  RVIZ_SCAN_TOPIC="$LIDAR_SCAN_TOPIC"
  if [[ "$RVIZ_SCAN_TOPIC" == /a201_0000/* ]]; then
    RVIZ_SCAN_TOPIC="<robot_namespace>/${RVIZ_SCAN_TOPIC#/a201_0000/}"
  fi
  mkdir -p /tmp/halmstad_ws
  RVIZ_CONFIG="/tmp/halmstad_ws/nav2_namespaced_waypoints.$(echo "$LIDAR_MODE" | tr -cd '[:alnum:]').rviz"
  sed "s#<robot_namespace>/sensors/lidar[23]d_0/scan#${RVIZ_SCAN_TOPIC}#g" "$BASE_RVIZ_CONFIG" > "$RVIZ_CONFIG"
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 launch nav2_bringup rviz_launch.py \
  namespace:=a201_0000 \
  use_namespace:=true \
  use_sim_time:=true \
  rviz_config:="$RVIZ_CONFIG" \
  "${LIDAR_REMAINING_ARGS[@]}"

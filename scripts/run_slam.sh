#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCAL_SLAM_LAUNCH="$WS_ROOT/src/lrs_halmstad/launch/slam_with_params.launch.py"
STATE_DIR="/tmp/halmstad_ws"
ACTIVE_SLAM_SCAN_TOPIC_FILE="$STATE_DIR/slam.scan_topic"
ACTIVE_SLAM_LIDAR_MODE_FILE="$STATE_DIR/slam.lidar_mode"

source "$SCRIPT_DIR/lidar_mode_common.sh"

lidar_mode_parse_args 2d "$@"

USE_SCAN_RELAY=""
SCAN_RELAY_HZ=""
SCAN_RELAY_MAX_AGE_S=""
USE_POINTCLOUD_TO_LASERSCAN="false"
SYNC_SET="false"
for arg in "${LIDAR_REMAINING_ARGS[@]}"; do
  case "$arg" in
    use_scan_relay:=*)
      USE_SCAN_RELAY="${arg#use_scan_relay:=}"
      ;;
    scan_relay_hz:=*)
      SCAN_RELAY_HZ="${arg#scan_relay_hz:=}"
      ;;
    scan_relay_max_age_s:=*)
      SCAN_RELAY_MAX_AGE_S="${arg#scan_relay_max_age_s:=}"
      ;;
    sync:=*)
      SYNC_SET="true"
      ;;
  esac
done

if [ -n "$LIDAR_POINTCLOUD_TOPIC" ] && [ "$LIDAR_SCAN_TOPIC" = "$(lidar_mode_scan_topic 3d)" ]; then
  USE_POINTCLOUD_TO_LASERSCAN="true"
fi

if [ -z "$USE_SCAN_RELAY" ]; then
  if lidar_mode_is_3d_topic "$LIDAR_SCAN_TOPIC"; then
    USE_SCAN_RELAY="true"
  else
    USE_SCAN_RELAY="false"
  fi
fi

if lidar_mode_is_3d_topic "$LIDAR_SCAN_TOPIC"; then
  if [ "$SYNC_SET" = "false" ]; then
    LIDAR_REMAINING_ARGS+=("sync:=false")
  fi
  if [ -z "$SCAN_RELAY_HZ" ]; then
    SCAN_RELAY_HZ="4.0"
  fi
  if [ -z "$SCAN_RELAY_MAX_AGE_S" ]; then
    SCAN_RELAY_MAX_AGE_S="0.25"
  fi
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

mkdir -p "$STATE_DIR"
printf '%s\n' "$LIDAR_SCAN_TOPIC" > "$ACTIVE_SLAM_SCAN_TOPIC_FILE"
printf '%s\n' "$LIDAR_MODE" > "$ACTIVE_SLAM_LIDAR_MODE_FILE"

LAUNCH_ARGS=(
  use_sim_time:=true
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath"
  scan_topic:="$LIDAR_SCAN_TOPIC"
  use_pointcloud_to_laserscan:="$USE_POINTCLOUD_TO_LASERSCAN"
  use_scan_relay:="$USE_SCAN_RELAY"
)

if [ -n "$LIDAR_POINTCLOUD_TOPIC" ]; then
  LAUNCH_ARGS+=("pointcloud_topic:=$LIDAR_POINTCLOUD_TOPIC")
fi

if [ -n "$SCAN_RELAY_HZ" ]; then
  LAUNCH_ARGS+=("scan_relay_hz:=$SCAN_RELAY_HZ")
fi

if [ -n "$SCAN_RELAY_MAX_AGE_S" ]; then
  LAUNCH_ARGS+=("scan_relay_max_age_s:=$SCAN_RELAY_MAX_AGE_S")
fi

LAUNCH_ARGS+=("${LIDAR_REMAINING_ARGS[@]}")

ros2 launch "$LOCAL_SLAM_LAUNCH" "${LAUNCH_ARGS[@]}"

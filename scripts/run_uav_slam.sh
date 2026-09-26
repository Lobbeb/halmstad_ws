#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCAL_UAV_SLAM_LAUNCH="$WS_ROOT/src/lrs_halmstad/launch/uav_slam.launch.py"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh uav_slam [uav:=dji0] [laser_name:=laser0] [arg:=value ...]

Start slam_toolbox for the optional UAV 2D lidar.

Common options:
  uav:=dji0                         UAV namespace/model name.
  laser_name:=laser0                Laser name used at spawn time.
  scan_topic:=/dji0/laser0/scan     Override scan topic.
  laser_x/y/z:=...                  Static TF offset base_link -> laser frame.
  laser_roll/pitch/yaw:=...         Static TF rotation base_link -> laser frame.
  sync:=true|false                  slam_toolbox sync mode. Default: false.

Example:
  ./run.sh uav_slam uav:=dji0 scan_topic:=/dji0/laser0/scan
EOF
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
  esac
done

UAV_NAME="dji0"
LASER_NAME="laser0"
SCAN_TOPIC=""
SHOW_ARGS=false
REMAINING_ARGS=()

for arg in "$@"; do
  case "$arg" in
    --show-args)
      SHOW_ARGS=true
      REMAINING_ARGS+=("$arg")
      ;;
    uav:=*|uav_name:=*|name:=*)
      UAV_NAME="${arg#*:=}"
      ;;
    laser_name:=*)
      LASER_NAME="${arg#laser_name:=}"
      ;;
    scan_topic:=*)
      SCAN_TOPIC="${arg#scan_topic:=}"
      REMAINING_ARGS+=("$arg")
      ;;
    *)
      REMAINING_ARGS+=("$arg")
      ;;
  esac
done

if [ "$SHOW_ARGS" = true ]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$WS_ROOT/install/setup.bash"
  set -u

  ros2 launch "$LOCAL_UAV_SLAM_LAUNCH" --show-args
  exit 0
fi

if [ -z "$SCAN_TOPIC" ]; then
  SCAN_TOPIC="/${UAV_NAME}/${LASER_NAME}/scan"
  REMAINING_ARGS+=("scan_topic:=$SCAN_TOPIC")
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 launch "$LOCAL_UAV_SLAM_LAUNCH" \
  uav:="$UAV_NAME" \
  laser_name:="$LASER_NAME" \
  "${REMAINING_ARGS[@]}"

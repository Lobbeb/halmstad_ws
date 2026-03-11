#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
WORLD="warehouse"
OUTPUT_DIR=""
EXTRA_ARGS=()

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

OUTPUT_DIR="$WS_ROOT/datasets/${WORLD}_auto"

for arg in "$@"; do
  case "$arg" in
    out:=*)
      OUTPUT_DIR="${arg#out:=}"
      ;;
    class:=*)
      EXTRA_ARGS+=("-p" "class_name:=${arg#class:=}")
      ;;
    id:=*)
      EXTRA_ARGS+=("-p" "class_id:=${arg#id:=}")
      ;;
    hz:=*)
      EXTRA_ARGS+=("-p" "capture_hz:=${arg#hz:=}")
      ;;
    negatives:=*)
      EXTRA_ARGS+=("-p" "save_negative_examples:=${arg#negatives:=}")
      ;;
    *)
      EXTRA_ARGS+=("-p" "$arg")
      ;;
  esac
done

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 run lrs_halmstad sim_dataset_capture --ros-args \
  -p use_sim_time:=true \
  -p output_dir:="$OUTPUT_DIR" \
  -p dataset_name:="$WORLD" \
  -p target_pose_topic:=/a201_0000/amcl_pose_odom \
  "${EXTRA_ARGS[@]}"

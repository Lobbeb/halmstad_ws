#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
WORLD="warehouse"
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

for arg in "$@"; do
  case "$arg" in
    weights:=*)
      EXTRA_ARGS+=("yolo_weights:=${arg#weights:=}")
      ;;
    height:=*)
      EXTRA_ARGS+=("uav_start_z:=${arg#height:=}")
      ;;
    mount_pitch_deg:=*)
      EXTRA_ARGS+=("camera_mount_pitch_deg:=${arg#mount_pitch_deg:=}")
      ;;
    target:=*)
      EXTRA_ARGS+=("target_class_name:=${arg#target:=}")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 launch lrs_halmstad run_1to1_follow.launch.py \
  ugv_mode:=nav2 \
  ugv_set_initial_pose:=true \
  leader_mode:=estimate \
  start_leader_estimator:=true \
  leader_range_mode:=ground \
  yolo_weights:=detection/yolo26/yolo26l.pt \
  "${EXTRA_ARGS[@]}" \
  world:="$WORLD"

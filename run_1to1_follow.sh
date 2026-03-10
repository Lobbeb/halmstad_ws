#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
WORLD="warehouse"

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

EXTRA_ARGS=()
for arg in "$@"; do
  case "$arg" in
    camera:=*|camera_mode:=*)
      camera_mode="${arg#camera:=}"
      if [[ "$arg" == camera_mode:=* ]]; then
        camera_mode="${arg#camera_mode:=}"
      fi
      case "$camera_mode" in
        attached|integrated|integrated_joint)
          EXTRA_ARGS+=("uav_camera_mode:=integrated_joint")
          ;;
        detached|detached_model)
          EXTRA_ARGS+=("uav_camera_mode:=detached_model")
          ;;
        *)
          EXTRA_ARGS+=("uav_camera_mode:=$camera_mode")
          ;;
      esac
      ;;
    height:=*)
      EXTRA_ARGS+=("uav_start_z:=${arg#height:=}")
      ;;
    pan_enable:=*)
      EXTRA_ARGS+=("$arg")
      ;;
    use_tilt:=*)
      EXTRA_ARGS+=("tilt_enable:=${arg#use_tilt:=}")
      ;;
    tilt_enable:=*)
      EXTRA_ARGS+=("$arg")
      ;;
    mount_pitch_deg:=*)
      EXTRA_ARGS+=("camera_mount_pitch_deg:=${arg#mount_pitch_deg:=}")
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

set +e
ros2 launch lrs_halmstad run_1to1_follow.launch.py \
  ugv_mode:=nav2 \
  ugv_set_initial_pose:=true \
  leader_mode:=odom \
  "${EXTRA_ARGS[@]}" \
  world:="$WORLD"
STATUS=$?
set -e

case "$STATUS" in
  0|130)
    exit 0
    ;;
  *)
    exit "$STATUS"
    ;;
esac

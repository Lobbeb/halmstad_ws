#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
WORLD="warehouse"
DEFAULT_UAV_BODY_X_OFFSET="-7.0"
DEFAULT_UAV_BODY_Y_OFFSET="0.0"
DEFAULT_UAV_Z="7.0"

source "$SCRIPT_DIR/slam_state_common.sh"

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
HAVE_UAV_START_X="false"
HAVE_UAV_START_Y="false"
HAVE_UAV_START_YAW="false"
HAVE_UAV_START_Z="false"
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
          echo "Detached camera mode has been removed from simulation. Use camera:=attached." >&2
          exit 2
          ;;
        *)
      EXTRA_ARGS+=("uav_camera_mode:=$camera_mode")
          ;;
      esac
      ;;
    uav_start_x:=*)
      HAVE_UAV_START_X="true"
      EXTRA_ARGS+=("$arg")
      ;;
    uav_start_y:=*)
      HAVE_UAV_START_Y="true"
      EXTRA_ARGS+=("$arg")
      ;;
    uav_start_yaw_deg:=*)
      HAVE_UAV_START_YAW="true"
      EXTRA_ARGS+=("$arg")
      ;;
    uav_start_z:=*)
      HAVE_UAV_START_Z="true"
      EXTRA_ARGS+=("$arg")
      ;;
    height:=*)
      HAVE_UAV_START_Z="true"
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
    omnet:=*)
      EXTRA_ARGS+=("start_omnet_bridge:=${arg#omnet:=}")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

if [ "$HAVE_UAV_START_X" = "false" ] && [ "$HAVE_UAV_START_Y" = "false" ]; then
  if UAV_START_ENV="$(slam_state_capture_uav_spawn_from_ugv_env \
    "$WS_ROOT" \
    "$WORLD" \
    "$DEFAULT_UAV_BODY_X_OFFSET" \
    "$DEFAULT_UAV_BODY_Y_OFFSET" \
    "$DEFAULT_UAV_Z" \
    5)"; then
    eval "$UAV_START_ENV"
    EXTRA_ARGS+=("uav_start_x:=$uav_x" "uav_start_y:=$uav_y")
    if [ "$HAVE_UAV_START_Z" = "false" ]; then
      EXTRA_ARGS+=("uav_start_z:=$uav_z")
    fi
    if [ "$HAVE_UAV_START_YAW" = "false" ]; then
      EXTRA_ARGS+=("uav_start_yaw_deg:=$uav_yaw_deg")
    fi
    echo "[run_1to1_follow] Using UGV-relative UAV start x=${uav_x} y=${uav_y} z=${uav_z} yaw_deg=${uav_yaw_deg}"
  else
    echo "[run_1to1_follow] Warning: could not read the live UGV pose; falling back to the launch defaults." >&2
  fi
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

set +e
ros2 launch lrs_halmstad run_follow.launch.py \
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

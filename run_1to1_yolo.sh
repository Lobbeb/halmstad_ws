#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
WORLD="warehouse"
EXTRA_ARGS=()
USE_ESTIMATE="true"
USE_OBB="false"
WEIGHTS_REL=""
MODEL_SUBDIR=""
HAVE_UAV_START_X="false"
HAVE_UAV_START_Z="false"
HAVE_STARTUP_REPOSITION_ENABLE="false"
DEFAULT_CUSTOM_WEIGHTS="detection/mymodels/warehouse_v1-v1-yolo26n.pt"
DEFAULT_DETECTION_WEIGHTS="detection/mymodels/warehouse_v1-v1-yolo26n.pt"
DEFAULT_OBB_WEIGHTS="obb/yolo26/yolo26l-obb.pt"

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
    weights:=*)
      WEIGHTS_REL="${arg#weights:=}"
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
    target:=*)
      EXTRA_ARGS+=("target_class_name:=${arg#target:=}")
      ;;
    use_estimate:=*)
      USE_ESTIMATE="${arg#use_estimate:=}"
      ;;
    folder:=*|dir:=*|subdir:=*)
      MODEL_SUBDIR="${arg#*:=}"
      ;;
    obb:=*)
      USE_OBB="${arg#obb:=}"
      ;;
    uav_start_x:=*)
      HAVE_UAV_START_X="true"
      EXTRA_ARGS+=("$arg")
      ;;
    uav_start_z:=*)
      HAVE_UAV_START_Z="true"
      EXTRA_ARGS+=("$arg")
      ;;
    startup_reposition_enable:=*)
      HAVE_STARTUP_REPOSITION_ENABLE="true"
      EXTRA_ARGS+=("$arg")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

case "$USE_ESTIMATE" in
  true|false)
    ;;
  *)
    echo "Invalid use_estimate option: $USE_ESTIMATE" >&2
    echo "Use use_estimate:=true or use_estimate:=false" >&2
    exit 2
    ;;
esac

case "$USE_OBB" in
  true|false)
    ;;
  *)
    echo "Invalid obb option: $USE_OBB" >&2
    echo "Use obb:=true or obb:=false" >&2
    exit 2
    ;;
esac

if [ "$USE_ESTIMATE" = true ]; then
  LEADER_MODE="estimate"
else
  LEADER_MODE="odom"
fi

if [ -z "$WEIGHTS_REL" ]; then
  if [ "$USE_OBB" = true ]; then
    if [ -n "$MODEL_SUBDIR" ]; then
      WEIGHTS_REL="obb/$MODEL_SUBDIR/yolo26l-obb.pt"
    else
      WEIGHTS_REL="$DEFAULT_OBB_WEIGHTS"
    fi
  else
    if [ -n "$MODEL_SUBDIR" ]; then
      WEIGHTS_REL="detection/$MODEL_SUBDIR/yolo26l.pt"
    else
      WEIGHTS_REL="$DEFAULT_CUSTOM_WEIGHTS"
    fi
  fi
elif [[ "$WEIGHTS_REL" != /* ]] && [ ! -e "$WS_ROOT/models/$WEIGHTS_REL" ]; then
  if [[ "$WEIGHTS_REL" == */* ]]; then
    if [[ "$WEIGHTS_REL" != detection/* && "$WEIGHTS_REL" != obb/* ]]; then
      if [ "$USE_OBB" = true ]; then
        WEIGHTS_REL="obb/$WEIGHTS_REL"
      else
        WEIGHTS_REL="detection/$WEIGHTS_REL"
      fi
    fi
  else
    if [ "$USE_OBB" = true ]; then
      WEIGHTS_REL="obb/${MODEL_SUBDIR:-yolo26}/$WEIGHTS_REL"
    else
      if [ -n "$MODEL_SUBDIR" ]; then
        WEIGHTS_REL="detection/$MODEL_SUBDIR/$WEIGHTS_REL"
      else
        WEIGHTS_REL="detection/mymodels/$WEIGHTS_REL"
      fi
    fi
  fi
fi

if [ "$USE_ESTIMATE" = true ]; then
  if [ "$HAVE_STARTUP_REPOSITION_ENABLE" != true ]; then
    EXTRA_ARGS+=("startup_reposition_enable:=true")
  fi
  if [ "$HAVE_UAV_START_X" != true ]; then
    EXTRA_ARGS+=("uav_start_x:=-7.0")
  fi
  if [ "$HAVE_UAV_START_Z" != true ]; then
    EXTRA_ARGS+=("uav_start_z:=7.0")
  fi
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 launch lrs_halmstad run_1to1_follow.launch.py \
  ugv_mode:=nav2 \
  ugv_set_initial_pose:=true \
  leader_mode:="$LEADER_MODE" \
  start_leader_estimator:=true \
  leader_range_mode:=ground \
  yolo_weights:="$WEIGHTS_REL" \
  "${EXTRA_ARGS[@]}" \
  world:="$WORLD"

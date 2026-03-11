#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
WORLD="warehouse"
EXTRA_ARGS=()
USE_ESTIMATE="true"
USE_ACTUAL_HEADING=""
HAVE_LEADER_ACTUAL_HEADING_ENABLE="false"
USE_OBB="false"
USE_TRACKER="false"
EXTERNAL_DETECTION_NODE="detector"
WEIGHTS_REL=""
MODEL_SUBDIR=""
HAVE_UAV_START_X="false"
HAVE_UAV_START_Z="false"
HAVE_STARTUP_REPOSITION_ENABLE="false"
DEFAULT_CUSTOM_WEIGHTS="detection/mymodels/warehouse_v1-v1-yolo26n.pt"
DEFAULT_DETECTION_WEIGHTS="detection/mymodels/warehouse_v1-v1-yolo26n.pt"
DEFAULT_OBB_WEIGHTS="obb/mymodels/warehouse-v1-yolo26n-obb.pt"

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
    use_actual_heading:=*)
      USE_ACTUAL_HEADING="${arg#use_actual_heading:=}"
      ;;
    leader_actual_heading_enable:=*)
      USE_ACTUAL_HEADING="${arg#leader_actual_heading_enable:=}"
      HAVE_LEADER_ACTUAL_HEADING_ENABLE="true"
      EXTRA_ARGS+=("$arg")
      ;;
    leader_actual_heading_topic:=*)
      EXTRA_ARGS+=("$arg")
      ;;
    folder:=*|dir:=*|subdir:=*)
      MODEL_SUBDIR="${arg#*:=}"
      ;;
    obb:=*)
      USE_OBB="${arg#obb:=}"
      ;;
    tracker:=*)
      USE_TRACKER="${arg#tracker:=}"
      ;;
    external_detection_node:=*)
      EXTERNAL_DETECTION_NODE="${arg#external_detection_node:=}"
      ;;
    tracker_config:=*)
      EXTRA_ARGS+=("$arg")
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

if [ -n "$USE_ACTUAL_HEADING" ]; then
  case "$USE_ACTUAL_HEADING" in
    true|false)
      ;;
    *)
      echo "Invalid use_actual_heading option: $USE_ACTUAL_HEADING" >&2
      echo "Use use_actual_heading:=true or use_actual_heading:=false" >&2
      exit 2
      ;;
  esac
fi

case "$USE_TRACKER" in
  true)
    EXTERNAL_DETECTION_NODE="tracker"
    ;;
  false)
    ;;
  *)
    echo "Invalid tracker option: $USE_TRACKER" >&2
    echo "Use tracker:=true or tracker:=false" >&2
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
      if [ -n "$MODEL_SUBDIR" ]; then
        WEIGHTS_REL="obb/$MODEL_SUBDIR/$WEIGHTS_REL"
      else
        WEIGHTS_REL="obb/mymodels/$WEIGHTS_REL"
      fi
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
  if [ -z "$USE_ACTUAL_HEADING" ] && [ "$HAVE_LEADER_ACTUAL_HEADING_ENABLE" != true ]; then
    EXTRA_ARGS+=("leader_actual_heading_enable:=true")
  elif [ -n "$USE_ACTUAL_HEADING" ] && [ "$HAVE_LEADER_ACTUAL_HEADING_ENABLE" != true ]; then
    EXTRA_ARGS+=("leader_actual_heading_enable:=$USE_ACTUAL_HEADING")
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
  external_detection_enable:=true \
  external_detection_node:="$EXTERNAL_DETECTION_NODE" \
  leader_range_mode:=ground \
  yolo_weights:="$WEIGHTS_REL" \
  "${EXTRA_ARGS[@]}" \
  world:="$WORLD"

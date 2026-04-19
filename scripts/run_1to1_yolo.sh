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
USE_OBB="true"
USE_TRACKER="true"
EXTERNAL_DETECTION_NODE="detector"
LEADER_RANGE_MODE="auto"
WEIGHTS_REL=""
MODEL_SUBDIR=""
HAVE_UAV_START_X="false"
HAVE_UAV_START_Y="false"
HAVE_UAV_START_YAW="false"
HAVE_UAV_START_Z="false"
HAVE_LEADER_ACTUAL_POSE_ENABLE="false"
HAVE_UGV_ODOM_TOPIC="false"
HAVE_LEADER_ACTUAL_POSE_TOPIC="false"
HAVE_CAMERA_LEADER_ACTUAL_POSE_TOPIC="false"
HAVE_UGV_USE_AMCL_ODOM_FALLBACK="false"
HAVE_START_UGV_GROUND_TRUTH_BRIDGE="false"
HAVE_PUBLISH_FOLLOW_DEBUG_TOPICS="false"
HAVE_PUBLISH_POSE_CMD_TOPICS="false"
HAVE_PUBLISH_CAMERA_DEBUG_TOPICS="false"
DEFAULT_CUSTOM_WEIGHTS="detection/mymodels/warehouse_v1-v2-yolo26n.pt"
DEFAULT_DETECTION_WEIGHTS="detection/mymodels/warehouse_v1-v2-yolo26n.pt"
DEFAULT_OBB_WEIGHTS="obb/mymodels/warehouse-v1-yolo26n-obb.pt"
MODELS_ROOT="${LRS_HALMSTAD_MODELS_ROOT:-$WS_ROOT/models}"
DEFAULT_UAV_BODY_X_OFFSET="-7.0"
DEFAULT_UAV_BODY_Y_OFFSET="0.0"
DEFAULT_UAV_Z="7.0"
UAV_NAME="dji0"

source "$SCRIPT_DIR/slam_state_common.sh"

case "$MODELS_ROOT" in
  "~")
    MODELS_ROOT="$HOME"
    ;;
  "~/"*)
    MODELS_ROOT="$HOME/${MODELS_ROOT#\~/}"
    ;;
esac

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
          echo "Detached camera mode has been removed from simulation. Use camera:=attached." >&2
          exit 2
          ;;
        *)
          EXTRA_ARGS+=("uav_camera_mode:=$camera_mode")
          ;;
      esac
      ;;
    uav_name:=*)
      UAV_NAME="${arg#uav_name:=}"
      EXTRA_ARGS+=("$arg")
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
    leader_actual_pose_enable:=*)
      HAVE_LEADER_ACTUAL_POSE_ENABLE="true"
      EXTRA_ARGS+=("$arg")
      ;;
    leader_actual_pose_topic:=*)
      HAVE_LEADER_ACTUAL_POSE_TOPIC="true"
      EXTRA_ARGS+=("$arg")
      ;;
    leader_actual_heading_topic:=*)
      EXTRA_ARGS+=("$arg")
      ;;
    camera_leader_actual_pose_topic:=*)
      HAVE_CAMERA_LEADER_ACTUAL_POSE_TOPIC="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_odom_topic:=*)
      HAVE_UGV_ODOM_TOPIC="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_use_amcl_odom_fallback:=*)
      HAVE_UGV_USE_AMCL_ODOM_FALLBACK="true"
      EXTRA_ARGS+=("$arg")
      ;;
    start_ugv_ground_truth_bridge:=*)
      HAVE_START_UGV_GROUND_TRUTH_BRIDGE="true"
      EXTRA_ARGS+=("$arg")
      ;;
    publish_follow_debug_topics:=*)
      HAVE_PUBLISH_FOLLOW_DEBUG_TOPICS="true"
      EXTRA_ARGS+=("$arg")
      ;;
    publish_pose_cmd_topics:=*)
      HAVE_PUBLISH_POSE_CMD_TOPICS="true"
      EXTRA_ARGS+=("$arg")
      ;;
    publish_camera_debug_topics:=*)
      HAVE_PUBLISH_CAMERA_DEBUG_TOPICS="true"
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
    range_mode:=*)
      LEADER_RANGE_MODE="${arg#range_mode:=}"
      ;;
    leader_range_mode:=*)
      LEADER_RANGE_MODE="${arg#leader_range_mode:=}"
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
    omnet:=*)
      EXTRA_ARGS+=("start_omnet_bridge:=${arg#omnet:=}")
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

case "$LEADER_RANGE_MODE" in
  auto|depth|ground|const)
    ;;
  *)
    echo "Invalid range_mode option: $LEADER_RANGE_MODE" >&2
    echo "Use range_mode:=auto|depth|ground|const" >&2
    exit 2
    ;;
esac

if [ "$USE_TRACKER" = true ]; then
  ARG_WEIGHTS_ROOT="obb"
else
  ARG_WEIGHTS_ROOT="detection"
fi

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
      if [ "$ARG_WEIGHTS_ROOT" = "obb" ]; then
        WEIGHTS_REL="obb/$WEIGHTS_REL"
      else
        WEIGHTS_REL="detection/$WEIGHTS_REL"
      fi
    fi
  else
    if [ "$ARG_WEIGHTS_ROOT" = "obb" ]; then
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

if [[ "$WEIGHTS_REL" = /* ]]; then
  WEIGHTS_PATH="$WEIGHTS_REL"
else
  WEIGHTS_PATH="$MODELS_ROOT/$WEIGHTS_REL"
fi

if [ ! -f "$WEIGHTS_PATH" ]; then
  echo "YOLO weights file not found: $WEIGHTS_PATH" >&2
  echo "Resolved from weights:=${WEIGHTS_REL}" >&2
  echo "Use an existing absolute path or a path relative to: $MODELS_ROOT" >&2
  exit 2
fi

if [ "$USE_ESTIMATE" = true ]; then
  if [ -z "$USE_ACTUAL_HEADING" ] && [ "$HAVE_LEADER_ACTUAL_HEADING_ENABLE" != true ]; then
    EXTRA_ARGS+=("leader_actual_heading_enable:=false")
  elif [ -n "$USE_ACTUAL_HEADING" ] && [ "$HAVE_LEADER_ACTUAL_HEADING_ENABLE" != true ]; then
    EXTRA_ARGS+=("leader_actual_heading_enable:=$USE_ACTUAL_HEADING")
  fi
fi

if [ "$HAVE_UAV_START_X" = "false" ] && [ "$HAVE_UAV_START_Y" = "false" ]; then
  if UAV_START_ENV="$(slam_state_capture_gazebo_named_pose_env \
    "$WORLD" \
    "$UAV_NAME" \
    5)"; then
    eval "$UAV_START_ENV"
    EXTRA_ARGS+=("uav_start_x:=$spawn_x" "uav_start_y:=$spawn_y")
    if [ "$HAVE_UAV_START_Z" = "false" ]; then
      EXTRA_ARGS+=("uav_start_z:=$spawn_z")
    fi
    if [ "$HAVE_UAV_START_YAW" = "false" ]; then
      EXTRA_ARGS+=("uav_start_yaw_deg:=$(python3 - "$spawn_yaw" <<'PY'
import math
import sys
print(f"{math.degrees(float(sys.argv[1])):.9f}")
PY
)")
    fi
    echo "[run_1to1_yolo] Using live UAV pose for simulator start x=${spawn_x} y=${spawn_y} z=${spawn_z} yaw=${spawn_yaw}"
  elif UAV_START_ENV="$(slam_state_capture_uav_spawn_from_ugv_env \
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
    echo "[run_1to1_yolo] Using UGV-relative UAV start x=${uav_x} y=${uav_y} z=${uav_z} yaw_deg=${uav_yaw_deg}"
  else
    if [[ "$WORLD" == baylands* ]]; then
      echo "[run_1to1_yolo] Error: could not read the live UGV pose for Baylands, so refusing to fall back to the generic UAV start (-7,0,7)." >&2
      echo "[run_1to1_yolo] Start Gazebo and the UGV first, or pass explicit uav_start_x:=... uav_start_y:=... uav_start_z:=... uav_start_yaw_deg:=..." >&2
      exit 2
    fi
    echo "[run_1to1_yolo] Warning: could not read the live UGV pose; falling back to the launch defaults." >&2
  fi
fi

if [[ "$WORLD" == baylands* ]]; then
  UGV_NAMESPACE="$(slam_state_namespace "$WS_ROOT" 2>/dev/null || true)"
  if [ -z "$UGV_NAMESPACE" ]; then
    UGV_NAMESPACE="a201_0000"
  fi
  if [ "$HAVE_UGV_ODOM_TOPIC" != "true" ]; then
    EXTRA_ARGS+=("ugv_odom_topic:=/$UGV_NAMESPACE/ground_truth/odom")
  fi
  if [ "$HAVE_LEADER_ACTUAL_POSE_TOPIC" != "true" ]; then
    EXTRA_ARGS+=("leader_actual_pose_topic:=/$UGV_NAMESPACE/ground_truth/odom")
  fi
  if [ "$HAVE_CAMERA_LEADER_ACTUAL_POSE_TOPIC" != "true" ]; then
    EXTRA_ARGS+=("camera_leader_actual_pose_topic:=/$UGV_NAMESPACE/ground_truth/odom")
  fi
  if [ "$HAVE_UGV_USE_AMCL_ODOM_FALLBACK" != "true" ]; then
    EXTRA_ARGS+=("ugv_use_amcl_odom_fallback:=false")
  fi
  if [ "$HAVE_START_UGV_GROUND_TRUTH_BRIDGE" != "true" ]; then
    EXTRA_ARGS+=("start_ugv_ground_truth_bridge:=true")
  fi
fi

if [ "$HAVE_LEADER_ACTUAL_POSE_ENABLE" != true ]; then
  EXTRA_ARGS+=("leader_actual_pose_enable:=false")
fi
if [ "$HAVE_PUBLISH_FOLLOW_DEBUG_TOPICS" != true ]; then
  EXTRA_ARGS+=("publish_follow_debug_topics:=false")
fi
if [ "$HAVE_PUBLISH_POSE_CMD_TOPICS" != true ]; then
  EXTRA_ARGS+=("publish_pose_cmd_topics:=false")
fi
if [ "$HAVE_PUBLISH_CAMERA_DEBUG_TOPICS" != true ]; then
  EXTRA_ARGS+=("publish_camera_debug_topics:=false")
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 launch lrs_halmstad run_follow.launch.py \
  ugv_mode:=nav2 \
  ugv_set_initial_pose:=true \
  leader_mode:="$LEADER_MODE" \
  start_leader_estimator:=true \
  external_detection_enable:=true \
  external_detection_node:="$EXTERNAL_DETECTION_NODE" \
  leader_range_mode:="$LEADER_RANGE_MODE" \
  yolo_weights:="$WEIGHTS_REL" \
  start_visual_actuation_bridge:=true \
  start_visual_follow_point_generator:=true \
  start_visual_follow_planner:=true \
  "${EXTRA_ARGS[@]}" \
  world:="$WORLD"

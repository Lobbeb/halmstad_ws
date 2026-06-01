#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
SIM_SPAWN_WAYPOINT_FILE="$STATE_DIR/gazebo_sim.spawn_waypoint"
BAYLANDS_DEFAULT_NAV2_GOALS="baylands_waypoints"
WORLD="baylands"
EXTRA_ARGS=()
CONTROL_ARGS=()
USE_ESTIMATE="true"
USE_OBB="true"
USE_TRACKER="false"
EXTERNAL_DETECTION_NODE="detector"
RANGE_MODE="auto"
START_OMNET_BRIDGE=""
YOLO_CONTROL_MODE="follow_uav_estimate"
WEIGHTS_REL=""
MODEL_SUBDIR=""
HAVE_UAV_START_X="false"
HAVE_UAV_START_Y="false"
HAVE_UAV_START_YAW="false"
HAVE_UAV_START_Z="false"
HAVE_UGV_USE_AMCL_ODOM_FALLBACK="false"
HAVE_YOLO_DEVICE="false"
HAVE_UGV_START_DELAY="false"
HAVE_DETECTOR_BACKEND="false"
HAVE_RANGE_MODE="false"
HAVE_UGV_INITIAL_POSE_X="false"
HAVE_UGV_INITIAL_POSE_Y="false"
HAVE_UGV_INITIAL_POSE_YAW="false"
HAVE_START_CAMERA_TRACKER="false"
HAVE_START_VISUAL_ACTUATION_BRIDGE="false"
HAVE_START_VISUAL_FOLLOW_CONTROLLER="false"
HAVE_START_VISUAL_FOLLOW_POINT_GENERATOR="false"
HAVE_START_VISUAL_FOLLOW_PLANNER="false"
HAVE_UGV_GOAL_SEQUENCE="false"
DETECTOR_BACKEND=""
DETECTOR_ONNX_MODEL_ARG=""
USE_CONDA="false"
CONDA_ENV_NAME="${LRS_HALMSTAD_GPU_ENV_NAME:-}"
DEFAULT_CUSTOM_WEIGHTS="/home/ruben/halmstad_ws/models/obb/mymodels/baylands-leader-v9-tuned-full.pt"
DEFAULT_DETECTION_WEIGHTS="warehouse_v1-v2-yolo26n.pt"
DEFAULT_OBB_WEIGHTS="/home/ruben/halmstad_ws/models/obb/mymodels/baylands-leader-v9-tuned-full.pt"
DEFAULT_BAYLANDS_OBB_WEIGHTS="/home/ruben/halmstad_ws/models/obb/mymodels/baylands-leader-v9-tuned-full.pt"
MODELS_ROOT="${LRS_HALMSTAD_MODELS_ROOT:-$WS_ROOT/models}"
DEFAULT_UAV_BODY_X_OFFSET="-7.0"
DEFAULT_UAV_BODY_Y_OFFSET="0.0"
DEFAULT_UAV_Z="7.0"
UAV_NAME="dji0"
LEADER_MODE="estimate"

source "$SCRIPT_DIR/slam_state_common.sh"
source "$SCRIPT_DIR/baylands_waypoint_common.sh"
BAYLANDS_GROUP_WAYPOINT_CSV="$(baylands_group_waypoint_csv)"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh 1to1_yolo [world] [options...]

Start YOLO detection, leader estimation, UAV follow, and the Nav2-backed UGV
route driver. YOLO mode intentionally refuses UGV odom/ground-truth inputs.

Common options:
  waypoint:=rotundan_0
  nav2_goals:=rotundan|path.yaml
  weights:=model.pt|models/obb/mymodels/model.pt|obb/mymodels/model.pt|/path/model.pt
  obb:=true|false
  tracker:=true|false
  external_detection_node:=detector|tracker
  use_estimate:=true
  yolo_control_mode:=follow_uav_estimate|visual_bridge
  detector_backend:=ultralytics|onnx_cpu|onnx_directml
  detector_async_inference:=true|false
  detector_latest_frame_only:=true|false
  detector_stale_detection_threshold_ms:=3000
  yolo_device:=auto|cpu|0
  range_mode:=auto|depth|radio|const
  omnet:=true|false

Examples:
  ./run.sh 1to1_yolo baylands waypoint:=rotundan_0 nav2_goals:=rotundan
  ./run.sh 1to1_yolo baylands weights:=/home/ruben/halmstad_ws/models/obb/mymodels/baylands-leader-v9-tuned-full.pt obb:=true
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

resolve_baylands_amcl_waypoint_pose() {
  local waypoint_name="$1"
  python3 - "$waypoint_name" "$BAYLANDS_GROUP_WAYPOINT_CSV" <<'PY'
import csv
import math
import sys

name, *paths = sys.argv[1:]

for path in paths:
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                waypoint_name = str(row.get("place", "")).strip()
                if waypoint_name != name:
                    continue
                try:
                    amcl_x = float(row["amcl_x"])
                    amcl_y = float(row["amcl_y"])
                    amcl_yaw = float(row["amcl_yaw"])
                except (KeyError, TypeError, ValueError):
                    raise SystemExit(1)

                print(f"ugv_waypoint_name={waypoint_name!r}")
                print(f"ugv_waypoint_distance_m={0.0:.9f}")
                print(f"ugv_amcl_x={amcl_x:.9f}")
                print(f"ugv_amcl_y={amcl_y:.9f}")
                print(f"ugv_amcl_yaw_deg={math.degrees(amcl_yaw):.9f}")
                raise SystemExit(0)
    except FileNotFoundError:
        continue

raise SystemExit(1)
PY
}

case "$MODELS_ROOT" in
  "~")
    MODELS_ROOT="$HOME"
    ;;
  "~/"*)
    MODELS_ROOT="$HOME/${MODELS_ROOT#\~/}"
    ;;
esac

weight_exists_relative_to_roots() {
  local rel_path="$1"
  [[ -e "$WS_ROOT/$rel_path" || -e "$MODELS_ROOT/$rel_path" ]]
}

default_weights_rel_dir() {
  if [ "$ARG_WEIGHTS_ROOT" = "obb" ]; then
    echo "obb/mymodels"
  else
    echo "detection/mymodels"
  fi
}

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

if [[ "$WORLD" == baylands* ]]; then
  baylands_sync_waypoints false
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
    yolo_control_mode:=*)
      YOLO_CONTROL_MODE="${arg#yolo_control_mode:=}"
      ;;
    use_actual_heading:=*|leader_actual_heading_enable:=*|leader_actual_heading_topic:=*|leader_actual_pose_enable:=*|leader_actual_pose_topic:=*|camera_actual_pose_reacquire_enable:=*|camera_leader_actual_pose_topic:=*|ugv_odom_topic:=*|start_ugv_ground_truth_bridge:=*)
      echo "Argument '$arg' is disabled in YOLO mode; the YOLO pipeline must not consume UGV odom, AMCL, or ground-truth pose." >&2
      exit 2
      ;;
    ugv_use_amcl_odom_fallback:=*)
      HAVE_UGV_USE_AMCL_ODOM_FALLBACK="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_initial_pose_x:=*)
      HAVE_UGV_INITIAL_POSE_X="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_initial_pose_y:=*)
      HAVE_UGV_INITIAL_POSE_Y="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_initial_pose_yaw_deg:=*)
      HAVE_UGV_INITIAL_POSE_YAW="true"
      EXTRA_ARGS+=("$arg")
      ;;
    goal_sequence_file:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      if [[ "$WORLD" == baylands* ]]; then
        EXTRA_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "${arg#goal_sequence_file:=}")")
      else
        EXTRA_ARGS+=("nav2_goals:=${arg#goal_sequence_file:=}")
      fi
      ;;
    goal_sequence_csv:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      EXTRA_ARGS+=("ugv_goal_sequence_csv:=${arg#goal_sequence_csv:=}")
      ;;
    nav2_goals:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      if [[ "$WORLD" == baylands* ]]; then
        EXTRA_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "${arg#nav2_goals:=}")")
      else
        EXTRA_ARGS+=("$arg")
      fi
      ;;
    ugv_goal_sequence_file:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      if [[ "$WORLD" == baylands* ]]; then
        EXTRA_ARGS+=("ugv_goal_sequence_file:=$(baylands_route_yaml_path "${arg#ugv_goal_sequence_file:=}")")
      else
        EXTRA_ARGS+=("$arg")
      fi
      ;;
    ugv_goal_sequence_csv:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      EXTRA_ARGS+=("$arg")
      ;;
    params_file:=*)
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
    device:=*|yolo_device:=*)
      HAVE_YOLO_DEVICE="true"
      EXTRA_ARGS+=("yolo_device:=${arg#*:=}")
      ;;
    detector_backend:=*)
      HAVE_DETECTOR_BACKEND="true"
      DETECTOR_BACKEND="${arg#detector_backend:=}"
      EXTRA_ARGS+=("$arg")
      ;;
    detector_onnx_model:=*)
      DETECTOR_ONNX_MODEL_ARG="$arg"
      ;;
    onnx_model:=*)
      DETECTOR_ONNX_MODEL_ARG="detector_onnx_model:=${arg#onnx_model:=}"
      ;;
    start_camera_tracker:=*)
      HAVE_START_CAMERA_TRACKER="true"
      EXTRA_ARGS+=("$arg")
      ;;
    start_visual_actuation_bridge:=*)
      HAVE_START_VISUAL_ACTUATION_BRIDGE="true"
      EXTRA_ARGS+=("$arg")
      ;;
    start_visual_follow_controller:=*)
      HAVE_START_VISUAL_FOLLOW_CONTROLLER="true"
      EXTRA_ARGS+=("$arg")
      ;;
    start_visual_follow_point_generator:=*)
      HAVE_START_VISUAL_FOLLOW_POINT_GENERATOR="true"
      EXTRA_ARGS+=("$arg")
      ;;
    start_visual_follow_planner:=*)
      HAVE_START_VISUAL_FOLLOW_PLANNER="true"
      EXTRA_ARGS+=("$arg")
      ;;
    conda_env:=*)
      CONDA_ENV_NAME="${arg#conda_env:=}"
      ;;
    use_conda:=*)
      USE_CONDA="${arg#use_conda:=}"
      ;;
    range_mode:=*)
      RANGE_MODE="${arg#range_mode:=}"
      HAVE_RANGE_MODE="true"
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
    ugv_start_delay_s:=*)
      HAVE_UGV_START_DELAY="true"
      EXTRA_ARGS+=("$arg")
      ;;
    omnet:=*)
      START_OMNET_BRIDGE="${arg#omnet:=}"
      EXTRA_ARGS+=("start_omnet_bridge:=$START_OMNET_BRIDGE")
      ;;
    start_omnet_bridge:=*)
      START_OMNET_BRIDGE="${arg#start_omnet_bridge:=}"
      EXTRA_ARGS+=("$arg")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

if [[ "$WORLD" == baylands* ]] && [ "$HAVE_UGV_GOAL_SEQUENCE" = "false" ]; then
  EXTRA_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "$BAYLANDS_DEFAULT_NAV2_GOALS")")
fi

if [ "$HAVE_RANGE_MODE" = "false" ]; then
  RANGE_MODE="auto"
fi

if [ "$HAVE_START_CAMERA_TRACKER" = "false" ]; then
  EXTRA_ARGS+=("start_camera_tracker:=false")
fi

if [ "$YOLO_CONTROL_MODE" != "visual_bridge" ]; then
  if [ "$HAVE_START_VISUAL_ACTUATION_BRIDGE" = "false" ]; then
    EXTRA_ARGS+=("start_visual_actuation_bridge:=false")
  fi
  if [ "$HAVE_START_VISUAL_FOLLOW_CONTROLLER" = "false" ]; then
    EXTRA_ARGS+=("start_visual_follow_controller:=false")
  fi
  if [ "$HAVE_START_VISUAL_FOLLOW_POINT_GENERATOR" = "false" ]; then
    EXTRA_ARGS+=("start_visual_follow_point_generator:=false")
  fi
  if [ "$HAVE_START_VISUAL_FOLLOW_PLANNER" = "false" ]; then
    EXTRA_ARGS+=("start_visual_follow_planner:=false")
  fi
fi

case "$USE_ESTIMATE" in
  true)
    ;;
  false)
    echo "use_estimate:=false is disabled for run_1to1_yolo; use the normal follow/odom launcher for odom follow." >&2
    exit 2
    ;;
  *)
    echo "Invalid use_estimate option: $USE_ESTIMATE" >&2
    echo "Use use_estimate:=true" >&2
    exit 2
    ;;
esac

case "$YOLO_CONTROL_MODE" in
  visual_bridge|follow_uav_estimate)
    ;;
  *)
    echo "Invalid yolo_control_mode option: $YOLO_CONTROL_MODE" >&2
    echo "Use yolo_control_mode:=visual_bridge or yolo_control_mode:=follow_uav_estimate" >&2
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

case "$RANGE_MODE" in
  auto|depth|radio|const)
    ;;
  *)
    echo "Invalid range_mode option: $RANGE_MODE" >&2
    echo "Use range_mode:=auto|depth|radio|const" >&2
    exit 2
    ;;
esac

case "$DETECTOR_BACKEND" in
  onnx|onnxruntime|onnx_cpu|onnx_directml)
    if [ -n "$DETECTOR_ONNX_MODEL_ARG" ]; then
      EXTRA_ARGS+=("$DETECTOR_ONNX_MODEL_ARG")
    fi
    ;;
  *)
    if [ -n "$DETECTOR_ONNX_MODEL_ARG" ]; then
      echo "[run_1to1_yolo] Ignoring $DETECTOR_ONNX_MODEL_ARG because detector_backend is '${DETECTOR_BACKEND:-default ultralytics}'." >&2
    fi
    ;;
esac

if [ "$USE_OBB" = true ]; then
  ARG_WEIGHTS_ROOT="obb"
else
  ARG_WEIGHTS_ROOT="detection"
fi

if [ "$YOLO_CONTROL_MODE" = "visual_bridge" ]; then
  CONTROL_ARGS=(
      "start_visual_actuation_bridge:=true"
      "start_visual_follow_point_generator:=true"
      "start_visual_follow_planner:=true"
  )
else
  LEADER_MODE="estimate"
fi



if [ -z "$WEIGHTS_REL" ]; then
  if [ "$USE_OBB" = true ]; then
    if [ -n "$MODEL_SUBDIR" ]; then
      WEIGHTS_REL="obb/$MODEL_SUBDIR/yolo26l-obb.pt"
    elif [[ "$WORLD" == baylands* ]] && [ -f "$MODELS_ROOT/$DEFAULT_BAYLANDS_OBB_WEIGHTS" ]; then
      WEIGHTS_REL="$DEFAULT_BAYLANDS_OBB_WEIGHTS"
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
elif [[ "$WEIGHTS_REL" != /* ]] && ! weight_exists_relative_to_roots "$WEIGHTS_REL"; then
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
        WEIGHTS_REL="$(default_weights_rel_dir)/$WEIGHTS_REL"
      fi
    else
      if [ -n "$MODEL_SUBDIR" ]; then
        WEIGHTS_REL="detection/$MODEL_SUBDIR/$WEIGHTS_REL"
      else
        WEIGHTS_REL="$(default_weights_rel_dir)/$WEIGHTS_REL"
      fi
    fi
  fi
fi

if [[ "$WEIGHTS_REL" = /* ]]; then
  WEIGHTS_PATH="$WEIGHTS_REL"
elif [ -e "$WS_ROOT/$WEIGHTS_REL" ]; then
  WEIGHTS_PATH="$WS_ROOT/$WEIGHTS_REL"
else
  WEIGHTS_PATH="$MODELS_ROOT/$WEIGHTS_REL"
fi

if [ ! -f "$WEIGHTS_PATH" ]; then
  echo "YOLO weights file not found: $WEIGHTS_PATH" >&2
  echo "Resolved from weights:=${WEIGHTS_REL}" >&2
  echo "Bare filenames are resolved under $(default_weights_rel_dir) for obb:=$USE_OBB." >&2
  echo "Use an existing absolute path, workspace-relative path, or path relative to: $MODELS_ROOT" >&2
  exit 2
fi

echo "[run_1to1_yolo] Using YOLO weights: $WEIGHTS_PATH"

LIVE_UAV_POSE_TIMEOUT_S=5
if [[ "$WORLD" == baylands* ]]; then
  LIVE_UAV_POSE_TIMEOUT_S=30
fi

if [ "$HAVE_UAV_START_X" = "false" ] && [ "$HAVE_UAV_START_Y" = "false" ]; then
  if UAV_START_ENV="$(slam_state_capture_gazebo_named_pose_env \
    "$WORLD" \
    "$UAV_NAME" \
    "$LIVE_UAV_POSE_TIMEOUT_S")"; then
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
  elif [[ "$WORLD" != baylands* ]] && UAV_START_ENV="$(slam_state_capture_uav_spawn_from_ugv_env \
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
      echo "[run_1to1_yolo] Error: could not read the live UAV pose for Baylands, so refusing to invent a startup pose." >&2
      echo "[run_1to1_yolo] Start Gazebo and spawn the UAV first, or pass explicit uav_start_x:=... uav_start_y:=... uav_start_z:=... uav_start_yaw_deg:=..." >&2
      exit 2
    fi
    echo "[run_1to1_yolo] Warning: could not read the live UGV pose; falling back to the launch defaults." >&2
  fi
fi

add_ugv_initial_pose_from_baylands_waypoint_map() {
  local timeout_s="$1"
  local ugv_pose_env=""
  local amcl_pose_env=""
  local spawn_waypoint=""

  if [ -f "$SIM_SPAWN_WAYPOINT_FILE" ]; then
    spawn_waypoint="$(tr -d '\r\n' < "$SIM_SPAWN_WAYPOINT_FILE" 2>/dev/null || true)"
    if [ -n "$spawn_waypoint" ]; then
      if amcl_pose_env="$(resolve_baylands_amcl_waypoint_pose "$spawn_waypoint")"; then
        eval "$amcl_pose_env"
        if [ "$HAVE_UGV_INITIAL_POSE_X" = "false" ]; then
          EXTRA_ARGS+=("ugv_initial_pose_x:=$ugv_amcl_x")
        fi
        if [ "$HAVE_UGV_INITIAL_POSE_Y" = "false" ]; then
          EXTRA_ARGS+=("ugv_initial_pose_y:=$ugv_amcl_y")
        fi
        if [ "$HAVE_UGV_INITIAL_POSE_YAW" = "false" ]; then
          EXTRA_ARGS+=("ugv_initial_pose_yaw_deg:=$ugv_amcl_yaw_deg")
        fi
        echo "[run_1to1_yolo] Using exact Baylands waypoint '$ugv_waypoint_name' for Nav2 initial pose x=${ugv_amcl_x} y=${ugv_amcl_y} yaw_deg=${ugv_amcl_yaw_deg}"
        return 0
      fi
      echo "[run_1to1_yolo] Warning: exact Baylands waypoint '$spawn_waypoint' has no usable AMCL pose; falling back to nearest world-position match." >&2
    fi
  fi

  ugv_pose_env="$(slam_state_capture_gazebo_pose_env "$WS_ROOT" "$WORLD" "$timeout_s")" || return 1
  eval "$ugv_pose_env"
  amcl_pose_env="$(
    python3 - "$spawn_x" "$spawn_y" "$BAYLANDS_GROUP_WAYPOINT_CSV" <<'PY'
import csv
import math
import sys

world_x = float(sys.argv[1])
world_y = float(sys.argv[2])
paths = sys.argv[3:]
best = None

for path in paths:
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                try:
                    x = float(row["x"])
                    y = float(row["y"])
                    amcl_x = float(row["amcl_x"])
                    amcl_y = float(row["amcl_y"])
                    amcl_yaw = float(row["amcl_yaw"])
                except (KeyError, TypeError, ValueError):
                    continue
                distance = math.hypot(x - world_x, y - world_y)
                if best is None or distance < best[0]:
                    best = (distance, row.get("place", ""), amcl_x, amcl_y, amcl_yaw)
    except FileNotFoundError:
        continue

if best is None or best[0] > 3.0:
    raise SystemExit(1)

distance, place, amcl_x, amcl_y, amcl_yaw = best
print(f"ugv_waypoint_name={place!r}")
print(f"ugv_waypoint_distance_m={distance:.9f}")
print(f"ugv_amcl_x={amcl_x:.9f}")
print(f"ugv_amcl_y={amcl_y:.9f}")
print(f"ugv_amcl_yaw_deg={math.degrees(amcl_yaw):.9f}")
PY
  )" || return 1
  eval "$amcl_pose_env"
  if [ "$HAVE_UGV_INITIAL_POSE_X" = "false" ]; then
    EXTRA_ARGS+=("ugv_initial_pose_x:=$ugv_amcl_x")
  fi
  if [ "$HAVE_UGV_INITIAL_POSE_Y" = "false" ]; then
    EXTRA_ARGS+=("ugv_initial_pose_y:=$ugv_amcl_y")
  fi
  if [ "$HAVE_UGV_INITIAL_POSE_YAW" = "false" ]; then
    EXTRA_ARGS+=("ugv_initial_pose_yaw_deg:=$ugv_amcl_yaw_deg")
  fi
  echo "[run_1to1_yolo] Using Baylands waypoint '$ugv_waypoint_name' for Nav2 initial pose x=${ugv_amcl_x} y=${ugv_amcl_y} yaw_deg=${ugv_amcl_yaw_deg} (world match ${ugv_waypoint_distance_m}m)"
}

if [[ "$WORLD" == baylands* ]]; then
  if ! add_ugv_initial_pose_from_baylands_waypoint_map 10; then
    echo "[run_1to1_yolo] Warning: could not resolve the live UGV world pose to a Baylands waypoint; Nav2 initial pose will use launch defaults." >&2
  fi
  if [ "$HAVE_UGV_USE_AMCL_ODOM_FALLBACK" != "true" ]; then
    EXTRA_ARGS+=("ugv_use_amcl_odom_fallback:=false")
  fi
fi
if [ "$HAVE_YOLO_DEVICE" != true ]; then
  EXTRA_ARGS+=("yolo_device:=auto")
fi
if [ "$HAVE_UGV_START_DELAY" != true ]; then
  EXTRA_ARGS+=("ugv_start_delay_s:=12.0")
fi
case "$USE_CONDA" in
  ""|true|false)
    ;;
  *)
    echo "Invalid use_conda option: $USE_CONDA" >&2
    echo "Use use_conda:=true or use_conda:=false" >&2
    exit 2
    ;;
esac

activate_conda_env() {
  local env_name="$1"
  if [ -z "$env_name" ]; then
    echo "Conda activation requested but no environment name was provided." >&2
    echo "Pass conda_env:=<env> or set LRS_HALMSTAD_GPU_ENV_NAME." >&2
    exit 2
  fi
  if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
  elif [ -x /opt/anaconda3/bin/conda ]; then
    eval "$(/opt/anaconda3/bin/conda shell.bash hook)"
  else
    echo "conda was not found. Expected it in PATH or at /opt/anaconda3/bin/conda." >&2
    exit 2
  fi
  conda activate "$env_name"
}

if [ "$USE_CONDA" = true ] || { [ -z "$USE_CONDA" ] && [ -n "$CONDA_ENV_NAME" ]; }; then
  activate_conda_env "$CONDA_ENV_NAME"
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
  range_mode:="$RANGE_MODE" \
  yolo_weights:="$WEIGHTS_PATH" \
  "${EXTRA_ARGS[@]}" \
  "${CONTROL_ARGS[@]}" \
  world:="$WORLD"

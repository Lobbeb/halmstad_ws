#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
FOLLOW_SIM=false
LAUNCH_PID=""
WATCH_PID=""
MODELS_ROOT="${LRS_HALMSTAD_MODELS_ROOT:-$WS_ROOT/models}"
DEFAULT_ONNX_MODEL=""
DEFAULT_YOLO_WEIGHTS="$MODELS_ROOT/obb/mymodels/baylands-leader-v4-3.pt"
DETECTOR_BACKEND="ultralytics"
DETECTOR_ONNX_MODEL="$DEFAULT_ONNX_MODEL"
YOLO_WEIGHTS="$DEFAULT_YOLO_WEIGHTS"
YOLO_DEVICE="auto"
DJI2_ENABLE=true
DJI2_ENABLE_EXPLICIT=false
HAZARD_FUSION_ENABLE=false
EXTRA_ARGS=()

sim_helper_running() {
  if [ ! -f "$SIM_PID_FILE" ]; then
    return 1
  fi

  local sim_pid
  sim_pid="$(cat "$SIM_PID_FILE" 2>/dev/null || true)"
  if [ -z "$sim_pid" ]; then
    return 1
  fi

  kill -0 "$sim_pid" 2>/dev/null
}

launch_running() {
  if [ -z "$LAUNCH_PID" ]; then
    return 1
  fi

  kill -0 "$LAUNCH_PID" 2>/dev/null
}

stop_launch_group() {
  local signal="$1"
  local timeout_s="$2"
  local waited_s=0
  local launch_pgid=""

  if ! launch_running; then
    return 0
  fi

  launch_pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
  if [ -n "$launch_pgid" ]; then
    /bin/kill "-$signal" -- "-$launch_pgid" 2>/dev/null || true
  else
    kill "-$signal" "$LAUNCH_PID" 2>/dev/null || true
  fi

  while launch_running && [ "$waited_s" -lt "$timeout_s" ]; do
    sleep 1
    waited_s=$((waited_s + 1))
  done
}

cleanup() {
  if [ -n "$WATCH_PID" ] && kill -0 "$WATCH_PID" 2>/dev/null; then
    kill "$WATCH_PID" 2>/dev/null || true
    wait "$WATCH_PID" 2>/dev/null || true
  fi

  if launch_running; then
    stop_launch_group INT 5
    stop_launch_group TERM 3
    if launch_running; then
      stop_launch_group KILL 0
    fi
  fi

  if [ -n "$LAUNCH_PID" ]; then
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup INT TERM EXIT

if sim_helper_running; then
  FOLLOW_SIM=true
  echo "[run_support_observation] Gazebo helper detected; stopping this overlay when the sim helper exits."
fi

DEFAULT_WORLD="baylands"
if sim_helper_running && [ -f "$SIM_WORLD_FILE" ]; then
  sim_world="$(cat "$SIM_WORLD_FILE" 2>/dev/null || true)"
  if [ -n "$sim_world" ]; then
    DEFAULT_WORLD="$sim_world"
  fi
fi

WORLD="$DEFAULT_WORLD"
if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

ORIG_ROS_DOMAIN_ID="${ROS_DOMAIN_ID-}"

for arg in "$@"; do
  case "$arg" in
    detector_backend:=*)
      DETECTOR_BACKEND="${arg#detector_backend:=}"
      ;;
    detector_onnx_model:=*|onnx_model:=*)
      DETECTOR_ONNX_MODEL="${arg#*:=}"
      ;;
    yolo_weights:=*|weights:=*)
      YOLO_WEIGHTS="${arg#*:=}"
      ;;
    yolo_device:=*|device:=*)
      YOLO_DEVICE="${arg#*:=}"
      ;;
    target:=*)
      EXTRA_ARGS+=("target_class_name:=${arg#target:=}")
      ;;
    target_class_name:=*|target_class_id:=*)
      EXTRA_ARGS+=("$arg")
      ;;
    dji2_enable:=*)
      DJI2_ENABLE="${arg#dji2_enable:=}"
      DJI2_ENABLE_EXPLICIT=true
      EXTRA_ARGS+=("$arg")
      ;;
    hazard_fusion_enable:=*)
      HAZARD_FUSION_ENABLE="${arg#hazard_fusion_enable:=}"
      EXTRA_ARGS+=("$arg")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

if [ "$HAZARD_FUSION_ENABLE" = true ] && [ "$DJI2_ENABLE_EXPLICIT" = false ]; then
  DJI2_ENABLE=false
  EXTRA_ARGS+=("dji2_enable:=false")
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

if [ -n "$ORIG_ROS_DOMAIN_ID" ]; then
  export ROS_DOMAIN_ID="$ORIG_ROS_DOMAIN_ID"
fi

topic_exists() {
  local topic="$1"
  ros2 topic list 2>/dev/null | grep -qx "$topic"
}

topic_exists_any() {
  local topic
  for topic in "$@"; do
    if topic_exists "$topic"; then
      return 0
    fi
  done
  return 1
}

if [ "$DJI2_ENABLE" = "true" ]; then
  echo "[run_support_observation] Starting dji1+dji2 support observation overlay on top of the support-follow motion baseline."
else
  echo "[run_support_observation] Starting dji1-only support observation overlay on top of the support-follow motion baseline."
fi
if ! topic_exists '/dji1/pose'; then
  echo "[run_support_observation] Warning: /dji1/pose is not visible yet. Start support_follow_odom first." >&2
fi
if [ "$DJI2_ENABLE" = "true" ] && ! topic_exists '/dji2/pose'; then
  echo "[run_support_observation] Warning: /dji2/pose is not visible yet. Start support_follow_odom first." >&2
fi
if ! topic_exists_any '/dji1/camera0/image_raw' '/dji1/camera0/image'; then
  echo "[run_support_observation] Warning: neither /dji1/camera0/image_raw nor /dji1/camera0/image is visible. Restart support_follow_odom with support_with_camera:=true." >&2
fi
if [ "$DJI2_ENABLE" = "true" ] && ! topic_exists_any '/dji2/camera0/image_raw' '/dji2/camera0/image'; then
  echo "[run_support_observation] Warning: neither /dji2/camera0/image_raw nor /dji2/camera0/image is visible. Restart support_follow_odom with support_with_camera:=true." >&2
fi
if [ "$DETECTOR_BACKEND" != "ultralytics" ] && [ ! -f "$DETECTOR_ONNX_MODEL" ]; then
  echo "[run_support_observation] Warning: detector_onnx_model not found: $DETECTOR_ONNX_MODEL" >&2
fi
if [ "$DETECTOR_BACKEND" = "ultralytics" ] && [ ! -f "$YOLO_WEIGHTS" ]; then
  echo "[run_support_observation] Warning: yolo_weights not found: $YOLO_WEIGHTS" >&2
fi

LAUNCH_ARGS=(
  "world:=$WORLD"
  "detector_backend:=$DETECTOR_BACKEND"
  "yolo_weights:=$YOLO_WEIGHTS"
  "yolo_device:=$YOLO_DEVICE"
)
if [ -n "$DETECTOR_ONNX_MODEL" ]; then
  LAUNCH_ARGS+=("detector_onnx_model:=$DETECTOR_ONNX_MODEL")
fi

setsid ros2 launch lrs_halmstad support_observation.launch.py \
  "${LAUNCH_ARGS[@]}" \
  "${EXTRA_ARGS[@]}" &
LAUNCH_PID=$!

if [ "$FOLLOW_SIM" = true ]; then
  (
    while launch_running; do
      if ! sim_helper_running; then
        echo "[run_support_observation] Gazebo helper exited; stopping overlay."
        stop_launch_group INT 5
        stop_launch_group TERM 3
        if launch_running; then
          echo "[run_support_observation] Overlay ignored shutdown signals; forcing exit."
          stop_launch_group KILL 0
        fi
        exit 0
      fi
      sleep 1
    done
  ) &
  WATCH_PID=$!
fi

set +e
wait "$LAUNCH_PID"
STATUS=$?
set -e
trap - INT TERM EXIT
cleanup
exit "$STATUS"

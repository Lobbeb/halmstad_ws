#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUNTIME_STATE_DIR="/tmp/halmstad_ws"
ACTIVE_SLAM_SCAN_TOPIC_FILE="$RUNTIME_STATE_DIR/slam.scan_topic"

source "$SHARED_SCRIPTS_DIR/lidar_mode_common.sh"
source "$SHARED_SCRIPTS_DIR/slam_state_common.sh"

STATE_NAME="$(slam_state_default_name)"
SAVE_SCAN_TOPIC=""
SAVE_ARGS=()
MAP_SAVE_RETRIES="3"
MAP_SAVE_RETRY_SLEEP_S="2"

while [ "$#" -gt 0 ]; do
  case "$1" in
    state_name:=*)
      STATE_NAME="${1#state_name:=}"
      ;;
    name:=*)
      STATE_NAME="${1#name:=}"
      ;;
    lidar:=*|scan_sensor:=*|scan_topic:=*)
      SAVE_ARGS+=("$1")
      ;;
    map_save_retries:=*)
      MAP_SAVE_RETRIES="${1#map_save_retries:=}"
      ;;
    map_save_retry_sleep_s:=*)
      MAP_SAVE_RETRY_SLEEP_S="${1#map_save_retry_sleep_s:=}"
      ;;
    *=*)
      echo "[run_slam_save_state] Unknown argument: $1" >&2
      exit 1
      ;;
    *)
      STATE_NAME="$1"
      ;;
  esac
  shift
done

if [ "${#SAVE_ARGS[@]}" -gt 0 ]; then
  lidar_mode_parse_args 2d "${SAVE_ARGS[@]}"
  SAVE_SCAN_TOPIC="$LIDAR_SCAN_TOPIC"
elif [ -f "$ACTIVE_SLAM_SCAN_TOPIC_FILE" ]; then
  SAVE_SCAN_TOPIC="$(cat "$ACTIVE_SLAM_SCAN_TOPIC_FILE" 2>/dev/null || true)"
fi

if [ -z "$SAVE_SCAN_TOPIC" ]; then
  lidar_mode_parse_args 2d
  SAVE_SCAN_TOPIC="$LIDAR_SCAN_TOPIC"
fi

WORLD_NAME="$(slam_state_default_name)"
NAMESPACE="$(slam_state_namespace "$WS_ROOT")"
POSEGRAPH_BASE="$(slam_posegraph_path_for_name "$WS_ROOT" "$STATE_NAME")"
STATE_DIR="$(dirname "$POSEGRAPH_BASE")"
METADATA_PATH="$(slam_metadata_path_for_name "$WS_ROOT" "$STATE_NAME")"
MAP_BASE="$(slam_map_base_for_name "$WS_ROOT" "$STATE_NAME")"
SERVICE_BASE="$(slam_service_base "$NAMESPACE")"
SERIALIZE_SERVICE="$SERVICE_BASE/serialize_map"
SAVE_SPAWN_X=""
SAVE_SPAWN_Y=""
SAVE_SPAWN_Z=""
SAVE_SPAWN_YAW=""
SAVE_SPAWN_QZ=""
SAVE_SPAWN_QW=""
SAVE_MAP_POSE_X=""
SAVE_MAP_POSE_Y=""
SAVE_MAP_POSE_YAW=""
SAVE_MAP_POSE_QZ=""
SAVE_MAP_POSE_QW=""

mkdir -p "$STATE_DIR"
mkdir -p "$RUNTIME_STATE_DIR"

slam_state_source_env "$WS_ROOT"

if ! slam_wait_for_service "$SERIALIZE_SERVICE" 5; then
  echo "[run_slam_save_state] $SERIALIZE_SERVICE is not available. Start ./run.sh slam first." >&2
  exit 1
fi

echo "[run_slam_save_state] Serializing SLAM pose graph to base path $POSEGRAPH_BASE"
ros2 service call \
  "$SERIALIZE_SERVICE" \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$POSEGRAPH_BASE'}"

if ! slam_posegraph_artifacts_exist "$POSEGRAPH_BASE"; then
  echo "[run_slam_save_state] Expected pose-graph artifacts were not created for base path: $POSEGRAPH_BASE" >&2
  exit 1
fi

echo "[run_slam_save_state] Saving occupancy map snapshot to ${MAP_BASE}.yaml/.pgm"
MAP_SAVE_OK="false"
for (( attempt = 1; attempt <= MAP_SAVE_RETRIES; attempt++ )); do
  ATTEMPT_MAP_BASE="${RUNTIME_STATE_DIR}/map_save_${STATE_NAME}_attempt${attempt}"

  if [ "$attempt" -gt 1 ]; then
    echo "[run_slam_save_state] Retrying map save (${attempt}/${MAP_SAVE_RETRIES}) after transient failure"
    sleep "$MAP_SAVE_RETRY_SLEEP_S"
  fi

  if ros2 run nav2_map_server map_saver_cli \
    -f "$ATTEMPT_MAP_BASE" \
    --ros-args \
    -r map:="/${NAMESPACE}/map"; then
    if [ -f "${ATTEMPT_MAP_BASE}.yaml" ] && [ -f "${ATTEMPT_MAP_BASE}.pgm" ]; then
      mv -f "${ATTEMPT_MAP_BASE}.yaml" "${MAP_BASE}.yaml"
      mv -f "${ATTEMPT_MAP_BASE}.pgm" "${MAP_BASE}.pgm"
      FINAL_MAP_IMAGE_BASENAME="$(basename "${MAP_BASE}.pgm")"
      if grep -q '^image:' "${MAP_BASE}.yaml"; then
        sed -i "s#^image: .*#image: ${FINAL_MAP_IMAGE_BASENAME}#" "${MAP_BASE}.yaml"
      fi
      MAP_SAVE_OK="true"
      break
    fi
  fi
done

if [ "$MAP_SAVE_OK" != "true" ]; then
  echo "[run_slam_save_state] Expected map snapshot files were not created under $MAP_BASE after ${MAP_SAVE_RETRIES} attempt(s)" >&2
  exit 1
fi

if SPAWN_ENV="$(slam_state_capture_gazebo_pose_env "$WS_ROOT" "$WORLD_NAME" 5)"; then
  eval "$SPAWN_ENV"
  SAVE_SPAWN_X="${spawn_x:-}"
  SAVE_SPAWN_Y="${spawn_y:-}"
  SAVE_SPAWN_Z="${spawn_z:-}"
  SAVE_SPAWN_YAW="${spawn_yaw:-}"
  SAVE_SPAWN_QZ="${spawn_qz:-}"
  SAVE_SPAWN_QW="${spawn_qw:-}"
else
  echo "[run_slam_save_state] Warning: could not capture current Gazebo spawn pose; checkpoint is still saved" >&2
fi

if MAP_POSE_ENV="$(slam_state_capture_map_pose_env "$WS_ROOT" 5)"; then
  eval "$MAP_POSE_ENV"
  SAVE_MAP_POSE_X="${map_pose_x:-}"
  SAVE_MAP_POSE_Y="${map_pose_y:-}"
  SAVE_MAP_POSE_YAW="${map_pose_yaw:-}"
  SAVE_MAP_POSE_QZ="${map_pose_qz:-}"
  SAVE_MAP_POSE_QW="${map_pose_qw:-}"
else
  echo "[run_slam_save_state] Warning: could not capture current SLAM map pose; resume may fall back to map origin" >&2
fi

slam_state_write_metadata \
  "$METADATA_PATH" \
  "$STATE_NAME" \
  "$WORLD_NAME" \
  "$NAMESPACE" \
  "$POSEGRAPH_BASE" \
  "$MAP_BASE" \
  "$SAVE_SCAN_TOPIC" \
  "$SAVE_SPAWN_X" \
  "$SAVE_SPAWN_Y" \
  "$SAVE_SPAWN_Z" \
  "$SAVE_SPAWN_YAW" \
  "$SAVE_SPAWN_QZ" \
  "$SAVE_SPAWN_QW" \
  "$SAVE_MAP_POSE_X" \
  "$SAVE_MAP_POSE_Y" \
  "$SAVE_MAP_POSE_YAW" \
  "$SAVE_MAP_POSE_QZ" \
  "$SAVE_MAP_POSE_QW"

echo "[run_slam_save_state] Saved checkpoint '$STATE_NAME'"
echo "  pose graph: $(slam_posegraph_file_for_base "$POSEGRAPH_BASE")"
echo "  pose data:  $(slam_posegraph_data_file_for_base "$POSEGRAPH_BASE")"
echo "  map yaml:   ${MAP_BASE}.yaml"
echo "  map pgm:    ${MAP_BASE}.pgm"
if [ -n "$SAVE_SPAWN_X" ] && [ -n "$SAVE_SPAWN_Y" ] && [ -n "$SAVE_SPAWN_YAW" ]; then
  echo "  spawn pose: x=${SAVE_SPAWN_X} y=${SAVE_SPAWN_Y} z=${SAVE_SPAWN_Z} yaw=${SAVE_SPAWN_YAW}"
fi
if [ -n "$SAVE_MAP_POSE_X" ] && [ -n "$SAVE_MAP_POSE_Y" ] && [ -n "$SAVE_MAP_POSE_YAW" ]; then
  echo "  map pose:   x=${SAVE_MAP_POSE_X} y=${SAVE_MAP_POSE_Y} yaw=${SAVE_MAP_POSE_YAW}"
fi

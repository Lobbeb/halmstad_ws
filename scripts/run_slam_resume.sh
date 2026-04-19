#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
TMP_SLAM_PARAMS="$STATE_DIR/slam_resume.params.yaml"
BASE_SLAM_PARAMS="/opt/ros/jazzy/share/clearpath_nav2_demos/config/a200/slam.yaml"

source "$SCRIPT_DIR/slam_state_common.sh"
source "$SCRIPT_DIR/lidar_mode_common.sh"

STATE_NAME="$(slam_state_default_name)"
DEFAULT_STATE_NAME="$STATE_NAME"
STATE_NAME_SET="false"
POSEGRAPH_BASE=""
START_MODE="pose"
POSE_X="0.0"
POSE_Y="0.0"
POSE_YAW="0.0"
POSE_SET="false"
RESUME_WARMUP_S="2.0"
RESUME_QUEUE_FLUSH_S="2.0"
SYNC_SET="false"
SCAN_TOPIC_SET="false"
SCAN_RELAY_HZ_SET="false"
SCAN_RELAY_MAX_AGE_SET="false"
SCAN_RELAY_START_DELAY_SET="false"
RESUME_TUNED_THROTTLE_SCANS=""
RESUME_TUNED_MIN_TIME_INTERVAL=""
RUN_SLAM_ARGS=()
RESUME_PROFILE="standard"

while [ "$#" -gt 0 ]; do
  case "$1" in
    state_name:=*)
      STATE_NAME="${1#state_name:=}"
      STATE_NAME_SET="true"
      ;;
    name:=*)
      STATE_NAME="${1#name:=}"
      STATE_NAME_SET="true"
      ;;
    posegraph:=*)
      POSEGRAPH_BASE="$(slam_posegraph_base_from_input "${1#posegraph:=}")"
      ;;
    start:=*)
      START_MODE="${1#start:=}"
      ;;
    pose:=*)
      IFS=',' read -r POSE_X POSE_Y POSE_YAW <<<"${1#pose:=}"
      POSE_SET="true"
      ;;
    resume_warmup_s:=*)
      RESUME_WARMUP_S="${1#resume_warmup_s:=}"
      ;;
    resume_queue_flush_s:=*)
      RESUME_QUEUE_FLUSH_S="${1#resume_queue_flush_s:=}"
      ;;
    sync:=*)
      SYNC_SET="true"
      RUN_SLAM_ARGS+=("$1")
      ;;
    scan_relay_hz:=*)
      SCAN_RELAY_HZ_SET="true"
      RUN_SLAM_ARGS+=("$1")
      ;;
    scan_relay_max_age_s:=*)
      SCAN_RELAY_MAX_AGE_SET="true"
      RUN_SLAM_ARGS+=("$1")
      ;;
    scan_relay_start_delay_s:=*)
      SCAN_RELAY_START_DELAY_SET="true"
      RUN_SLAM_ARGS+=("$1")
      ;;
    profile:=*)
      RESUME_PROFILE="${1#profile:=}"
      ;;
    lifelong:=*)
      case "${1#lifelong:=}" in
        true) RESUME_PROFILE="lifelong" ;;
        false) RESUME_PROFILE="standard" ;;
        *)
          echo "[run_slam_resume] lifelong:= must be true or false" >&2
          exit 2
          ;;
      esac
      ;;
    lidar:=*|scan_sensor:=*|scan_topic:=*)
      SCAN_TOPIC_SET="true"
      RUN_SLAM_ARGS+=("$1")
      ;;
    *=*)
      RUN_SLAM_ARGS+=("$1")
      ;;
    *)
      if [ "$STATE_NAME_SET" = "false" ] && [ "$STATE_NAME" = "$DEFAULT_STATE_NAME" ]; then
        STATE_NAME="$1"
        STATE_NAME_SET="true"
      else
        RUN_SLAM_ARGS+=("$1")
      fi
      ;;
  esac
  shift
done

case "$RESUME_PROFILE" in
  standard|lifelong)
    ;;
  *)
    echo "[run_slam_resume] Unsupported profile '$RESUME_PROFILE'. Use standard or lifelong." >&2
    exit 2
    ;;
esac

if [ -z "$POSEGRAPH_BASE" ]; then
  POSEGRAPH_BASE="$(slam_posegraph_path_for_name "$WS_ROOT" "$STATE_NAME")"
fi

if ! POSEGRAPH_BASE="$(slam_posegraph_resolve_base "$POSEGRAPH_BASE")"; then
  echo "[run_slam_resume] Pose-graph checkpoint not found for base path: $POSEGRAPH_BASE" >&2
  echo "[run_slam_resume] Save one first with ./run.sh slam_save_state ${STATE_NAME}" >&2
  exit 1
fi

case "$START_MODE" in
  dock)
    MATCH_TYPE=1
    ;;
  pose)
    MATCH_TYPE=2
    ;;
  localize)
    MATCH_TYPE=3
    ;;
  *)
    echo "[run_slam_resume] Unsupported start mode '$START_MODE'. Use dock, pose, or localize." >&2
    exit 1
    ;;
esac

NAMESPACE="$(slam_state_namespace "$WS_ROOT")"
SERVICE_BASE="$(slam_service_base "$NAMESPACE")"
CLEAR_QUEUE_SERVICE="$SERVICE_BASE/clear_queue"
mkdir -p "$STATE_DIR"

STATE_METADATA_PATH="$(slam_metadata_path_for_name "$WS_ROOT" "$STATE_NAME")"
STATE_SCAN_TOPIC=""
STATE_SPAWN_X=""
STATE_SPAWN_Y=""
STATE_SPAWN_Z=""
STATE_SPAWN_YAW=""
STATE_WORLD_NAME=""
STATE_MAP_POSE_X=""
STATE_MAP_POSE_Y=""
STATE_MAP_POSE_YAW=""
RESUME_SCAN_TOPIC=""
if [ -f "$STATE_METADATA_PATH" ]; then
  # shellcheck disable=SC1090
  source "$STATE_METADATA_PATH"
  STATE_SCAN_TOPIC="${scan_topic:-}"
  STATE_SPAWN_X="${spawn_x:-}"
  STATE_SPAWN_Y="${spawn_y:-}"
  STATE_SPAWN_Z="${spawn_z:-}"
  STATE_SPAWN_YAW="${spawn_yaw:-}"
  STATE_WORLD_NAME="${world:-}"
  STATE_MAP_POSE_X="${map_pose_x:-}"
  STATE_MAP_POSE_Y="${map_pose_y:-}"
  STATE_MAP_POSE_YAW="${map_pose_yaw:-}"
fi

if [ "$POSE_SET" = "false" ] && [ -n "$STATE_MAP_POSE_X" ] && [ -n "$STATE_MAP_POSE_Y" ] && [ -n "$STATE_MAP_POSE_YAW" ]; then
  POSE_X="$STATE_MAP_POSE_X"
  POSE_Y="$STATE_MAP_POSE_Y"
  POSE_YAW="$STATE_MAP_POSE_YAW"
  echo "[run_slam_resume] Using saved SLAM map pose from checkpoint metadata: x=${POSE_X} y=${POSE_Y} yaw=${POSE_YAW}"
fi

if [ "$RESUME_PROFILE" = "lifelong" ]; then
  echo "[run_slam_resume] Using lifelong mapping profile for continuous map refinement"
fi

if [ "$SCAN_TOPIC_SET" = "false" ] && [ -n "$STATE_SCAN_TOPIC" ]; then
  if lidar_mode_is_3d_topic "$STATE_SCAN_TOPIC"; then
    RESUME_SCAN_TOPIC="$(lidar_mode_scan_topic 3d)"
    echo "[run_slam_resume] Using pointcloud_to_laserscan default for resumed 3D lidar checkpoint"
  else
    RESUME_SCAN_TOPIC="$STATE_SCAN_TOPIC"
  fi
  RUN_SLAM_ARGS+=("scan_topic:=$RESUME_SCAN_TOPIC")
fi

if [ -z "$RESUME_SCAN_TOPIC" ]; then
  RESUME_SCAN_TOPIC="${STATE_SCAN_TOPIC:-}"
fi

if [ "$SYNC_SET" = "false" ] && lidar_mode_is_3d_topic "$RESUME_SCAN_TOPIC"; then
  RUN_SLAM_ARGS+=("sync:=false")
  RESUME_TUNED_THROTTLE_SCANS="4"
  RESUME_TUNED_MIN_TIME_INTERVAL="0.75"
  echo "[run_slam_resume] Using async SLAM by default for resumed 3D lidar checkpoints"
  echo "[run_slam_resume] Applying conservative scan decimation for resumed 3D mapping"
fi

if lidar_mode_is_3d_topic "$RESUME_SCAN_TOPIC"; then
  if [ "$SCAN_RELAY_HZ_SET" = "false" ]; then
    RUN_SLAM_ARGS+=("scan_relay_hz:=2.0")
  fi
  if [ "$SCAN_RELAY_MAX_AGE_SET" = "false" ]; then
    RUN_SLAM_ARGS+=("scan_relay_max_age_s:=0.15")
  fi
  if [ "$SCAN_RELAY_START_DELAY_SET" = "false" ]; then
    RUN_SLAM_ARGS+=("scan_relay_start_delay_s:=4.0")
  fi
  echo "[run_slam_resume] Holding relayed 3D scans briefly so resume starts from a fresh scan only"
fi

awk \
  -v posegraph_base="$POSEGRAPH_BASE" \
  -v start_mode="$START_MODE" \
  -v pose_x="$POSE_X" \
  -v pose_y="$POSE_Y" \
  -v pose_yaw="$POSE_YAW" \
  -v resume_profile="$RESUME_PROFILE" \
  -v tuned_throttle_scans="$RESUME_TUNED_THROTTLE_SCANS" \
  -v tuned_min_time_interval="$RESUME_TUNED_MIN_TIME_INTERVAL" \
  '
  {
    if (resume_profile == "lifelong" && $0 ~ /^[[:space:]]*minimum_travel_distance:/) {
      match($0, /^ */)
      indent = substr($0, 1, RLENGTH)
      print indent "minimum_travel_distance: 0.5"
      next
    }
    if (resume_profile == "lifelong" && $0 ~ /^[[:space:]]*minimum_travel_heading:/) {
      match($0, /^ */)
      indent = substr($0, 1, RLENGTH)
      print indent "minimum_travel_heading: 0.5"
      next
    }
    if (tuned_throttle_scans != "" && $0 ~ /^[[:space:]]*throttle_scans:/) {
      match($0, /^ */)
      indent = substr($0, 1, RLENGTH)
      print indent "throttle_scans: " tuned_throttle_scans
      next
    }
    if (tuned_min_time_interval != "" && $0 ~ /^[[:space:]]*minimum_time_interval:/) {
      match($0, /^ */)
      indent = substr($0, 1, RLENGTH)
      print indent "minimum_time_interval: " tuned_min_time_interval
      next
    }
    print
    if ($0 ~ /^[[:space:]]*mode:[[:space:]]*mapping/) {
      match($0, /^ */)
      indent = substr($0, 1, RLENGTH)
      if (resume_profile == "lifelong") {
        print indent "lifelong_search_use_tree: false"
        print indent "lifelong_minimum_score: 0.1"
        print indent "lifelong_iou_match: 0.85"
        print indent "lifelong_node_removal_score: 0.04"
        print indent "lifelong_overlap_score_scale: 0.06"
        print indent "lifelong_constraint_multiplier: 0.08"
        print indent "lifelong_nearby_penalty: 0.001"
        print indent "lifelong_candidates_scale: 0.03"
      }
      print indent "map_file_name: " posegraph_base
      if (start_mode == "dock") {
        print indent "map_start_at_dock: true"
      } else if (start_mode == "pose" || start_mode == "localize") {
        print indent "map_start_pose: [" pose_x ", " pose_y ", " pose_yaw "]"
      }
      print indent "restamp_tf: true"
      print indent "scan_queue_size: 1"
    }
  }
' "$BASE_SLAM_PARAMS" > "$TMP_SLAM_PARAMS"

if [ "$RESUME_WARMUP_S" != "0" ] && [ "$RESUME_WARMUP_S" != "0.0" ]; then
  echo "[run_slam_resume] Waiting ${RESUME_WARMUP_S}s before starting resumed SLAM"
  sleep "$RESUME_WARMUP_S"
fi

if [ -n "$STATE_SPAWN_X" ] && [ -n "$STATE_SPAWN_Y" ] && [ -n "$STATE_SPAWN_YAW" ]; then
  echo "[run_slam_resume] Matching Gazebo respawn is available via:"
  echo "  ./run.sh gazebo_sim ${STATE_WORLD_NAME:-$STATE_NAME} true state:=${STATE_NAME}"
fi

echo "[run_slam_resume] Starting resumed SLAM from base path $POSEGRAPH_BASE"
slam_state_source_env "$WS_ROOT"

"$SCRIPT_DIR/run_slam.sh" \
  slam_params_file:="$TMP_SLAM_PARAMS" \
  "${RUN_SLAM_ARGS[@]}" &
SLAM_PID=$!

forward_signal() {
  local signal="$1"
  if kill -0 "$SLAM_PID" 2>/dev/null; then
    kill "-$signal" "$SLAM_PID" 2>/dev/null || kill "$SLAM_PID" 2>/dev/null || true
  fi
}

cleanup() {
  forward_signal TERM
}

trap 'forward_signal INT' INT
trap 'forward_signal TERM' TERM
trap cleanup EXIT

flush_queue() {
  ros2 service call \
    "$CLEAR_QUEUE_SERVICE" \
    slam_toolbox/srv/ClearQueue \
    "{}" >/dev/null 2>&1 || true
}

wait_result=0
if slam_wait_for_service_or_process "$CLEAR_QUEUE_SERVICE" "$SLAM_PID" 20; then
  echo "[run_slam_resume] Clearing SLAM scan queue after startup"
  flush_queue
  if [ "$RESUME_QUEUE_FLUSH_S" != "0" ] && [ "$RESUME_QUEUE_FLUSH_S" != "0.0" ]; then
    sleep "$RESUME_QUEUE_FLUSH_S"
    flush_queue
  fi
else
  wait_result=$?
  if [ "$wait_result" -eq 1 ]; then
    echo "[run_slam_resume] Warning: $CLEAR_QUEUE_SERVICE did not appear before timeout" >&2
  fi
fi

wait "$SLAM_PID"
SLAM_STATUS=$?
trap - INT TERM EXIT
exit "$SLAM_STATUS"

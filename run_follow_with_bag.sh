#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_ROOT="${RUN_ROOT:-$WS_ROOT/runs}"
STATE_DIR="${STATE_DIR:-/tmp/halmstad_ws}"
MODE_FILE="$STATE_DIR/run_mode.lock"
MODE_LOCK_ENABLE="${MODE_LOCK_ENABLE:-true}"
CURRENT_MODE="follow"

is_truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

is_launch_assignment() {
  [[ "${1:-}" == *":="* ]]
}

has_launch_assignment_key() {
  local key="$1"
  local arg
  for arg in "${EXTRA_LAUNCH_ARGS[@]}"; do
    if [[ "$arg" == "${key}:="* ]]; then
      return 0
    fi
  done
  return 1
}

get_launch_assignment_value() {
  local key="$1"
  local default_value="$2"
  local value="$default_value"
  local arg
  for arg in "${EXTRA_LAUNCH_ARGS[@]}"; do
    if [[ "$arg" == "${key}:="* ]]; then
      value="${arg#*:=}"
    fi
  done
  printf '%s' "$value"
}

normalize_topic() {
  local topic="$1"
  if [[ "$topic" == /* ]]; then
    printf '%s' "$topic"
  else
    printf '/%s/%s' "$UGV_NAMESPACE" "${topic#/}"
  fi
}

pop_positional_or_default() {
  local __out_var="$1"
  local default_value="$2"
  local value="$default_value"
  if [[ "${#REMAINING_ARGS[@]}" -gt 0 ]] && ! is_launch_assignment "${REMAINING_ARGS[0]}"; then
    value="${REMAINING_ARGS[0]}"
    REMAINING_ARGS=("${REMAINING_ARGS[@]:1}")
  fi
  printf -v "$__out_var" '%s' "$value"
}

REMAINING_ARGS=("$@")
pop_positional_or_default RUN_ID "run_$(date +%Y%m%d_%H%M%S)"
pop_positional_or_default WORLD "orchard"
pop_positional_or_default UAV_NAME "dji0"
pop_positional_or_default LEADER_MODE "odom"
EXTRA_LAUNCH_ARGS=("${REMAINING_ARGS[@]}")

# ros2 launch rejects empty assignment values like key:= .
# Drop them early with a warning so runs fail less often on CLI typos.
if [[ "${#EXTRA_LAUNCH_ARGS[@]}" -gt 0 ]]; then
  CLEAN_LAUNCH_ARGS=()
  for _arg in "${EXTRA_LAUNCH_ARGS[@]}"; do
    if [[ "$_arg" == *":=" ]]; then
      echo "[run_follow_with_bag] warning: dropping empty launch arg '$_arg'"
      continue
    fi
    CLEAN_LAUNCH_ARGS+=("$_arg")
  done
  EXTRA_LAUNCH_ARGS=("${CLEAN_LAUNCH_ARGS[@]}")
fi

# Allow `world:=...` in extra launch args to override the positional world for
# all wrapper phases (preflight, metadata, launch command).
EFFECTIVE_WORLD="$(get_launch_assignment_value "world" "$WORLD")"

DEFAULT_UGV_NAMESPACE="${UGV_NAMESPACE:-a201_0000}"
UGV_NAMESPACE="$(get_launch_assignment_value "ugv_namespace" "$DEFAULT_UGV_NAMESPACE")"
UGV_ODOM_TOPIC="$(normalize_topic "$(get_launch_assignment_value "ugv_odom_topic" "/${UGV_NAMESPACE}/platform/odom")")"
UGV_CMD_TOPIC="$(normalize_topic "$(get_launch_assignment_value "ugv_cmd_topic" "/${UGV_NAMESPACE}/cmd_vel")")"

LEADER_PERCEPTION_ENABLE="${LEADER_PERCEPTION_ENABLE:-false}"
START_LEADER_ESTIMATOR="${START_LEADER_ESTIMATOR:-auto}"
LEADER_DEPENDENCY_MODE="${LEADER_DEPENDENCY_MODE:-ugv_state}"
UAV_BACKEND="${UAV_BACKEND:-setpose}"
YOLO_WEIGHTS="${YOLO_WEIGHTS:-}"
YOLO_DEVICE="${YOLO_DEVICE:-cpu}"
UGV_START_DELAY_S="${UGV_START_DELAY_S:-0.0}"
UAV_ONLY_UGV_START_DELAY_S="${UAV_ONLY_UGV_START_DELAY_S:-4.0}"
UAV_ONLY_REQUIRE_YOLO="${UAV_ONLY_REQUIRE_YOLO:-true}"
UAV_ONLY_REQUIRE_LEADER_FLOW="${UAV_ONLY_REQUIRE_LEADER_FLOW:-true}"
UAV_ONLY_READY_TIMEOUT_S="${UAV_ONLY_READY_TIMEOUT_S:-20.0}"
SHUTDOWN_WHEN_UGV_DONE="${SHUTDOWN_WHEN_UGV_DONE:-true}"
SOFT_RESET_BEFORE_RUN="${SOFT_RESET_BEFORE_RUN:-true}"
SOFT_RESET_AFTER_RUN="${SOFT_RESET_AFTER_RUN:-true}"
RESET_UAV_BEFORE_RUN="${RESET_UAV_BEFORE_RUN:-false}"
RESET_UAV_X="${RESET_UAV_X:--2.0}"
RESET_UAV_Y="${RESET_UAV_Y:-0.0}"
RESET_UAV_Z="${RESET_UAV_Z:-5.0}"
BAG_STORAGE="${BAG_STORAGE:-mcap}"
RECORD_DEBUG_IMAGE="${RECORD_DEBUG_IMAGE:-false}"
FOLLOW_PROFILE="${FOLLOW_PROFILE:-default}"

PREFLIGHT_ENABLE="${PREFLIGHT_ENABLE:-true}"
PREFLIGHT_TIMEOUT_S="${PREFLIGHT_TIMEOUT_S:-15}"
PREFLIGHT_ACTIVATE_CONTROLLER="${PREFLIGHT_ACTIVATE_CONTROLLER:-true}"
PREFLIGHT_REQUIRE_CLOCK="${PREFLIGHT_REQUIRE_CLOCK:-true}"
PREFLIGHT_REQUIRE_SET_POSE_SERVICE="${PREFLIGHT_REQUIRE_SET_POSE_SERVICE:-true}"
PREFLIGHT_REQUIRE_UAV_NAMESPACE_TOPICS="${PREFLIGHT_REQUIRE_UAV_NAMESPACE_TOPICS:-true}"
PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS="${PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS:-auto}"
PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW="${PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW:-auto}"
PREFLIGHT_MODE_GUARD_ENABLE="${PREFLIGHT_MODE_GUARD_ENABLE:-true}"
PREFLIGHT_FORBID_EXISTING_FOLLOW_NODES="${PREFLIGHT_FORBID_EXISTING_FOLLOW_NODES:-true}"
PREFLIGHT_NODE_LIST_TIMEOUT_S="${PREFLIGHT_NODE_LIST_TIMEOUT_S:-5}"
PREFLIGHT_SET_POSE_TIMEOUT_S="${PREFLIGHT_SET_POSE_TIMEOUT_S:-$PREFLIGHT_TIMEOUT_S}"
PREFLIGHT_UAV_TOPICS_TIMEOUT_S="${PREFLIGHT_UAV_TOPICS_TIMEOUT_S:-$PREFLIGHT_TIMEOUT_S}"
PREFLIGHT_REQUIRE_UGV_ODOM_TOPIC="${PREFLIGHT_REQUIRE_UGV_ODOM_TOPIC:-auto}"
PREFLIGHT_CONTROLLER_MANAGER="${PREFLIGHT_CONTROLLER_MANAGER:-/${UGV_NAMESPACE}/controller_manager}"
PREFLIGHT_CONTROLLER_NAME="${PREFLIGHT_CONTROLLER_NAME:-platform_velocity_controller}"
PREFLIGHT_SCRIPT="${PREFLIGHT_SCRIPT:-$WS_ROOT/run_follow_preflight.sh}"
STRICT_PARAMS_FILE="${STRICT_PARAMS_FILE:-$WS_ROOT/src/lrs_halmstad/config/run_round_follow_observe_strict.yaml}"

RUN_DIR="$RUN_ROOT/$RUN_ID"
BAG_DIR="$RUN_DIR/bag"
META_FILE="$RUN_DIR/meta.yaml"
LAUNCH_LOG="$RUN_DIR/launch.log"
BAG_LOG="$RUN_DIR/rosbag.log"
BAG_INFO_FILE="$RUN_DIR/bag_info.txt"
PREFLIGHT_LOG="$RUN_DIR/preflight.log"

BAG_PID=""
BAG_STOPPED=0
SUMMARY_WRITTEN=0
BAG_STOP_INT_TIMEOUT_S="${BAG_STOP_INT_TIMEOUT_S:-4}"
BAG_STOP_TERM_TIMEOUT_S="${BAG_STOP_TERM_TIMEOUT_S:-3}"
BAG_STOP_KILL_TIMEOUT_S="${BAG_STOP_KILL_TIMEOUT_S:-2}"

case "$FOLLOW_PROFILE" in
  default|strict) ;;
  *)
    echo "[run_follow_with_bag] invalid FOLLOW_PROFILE='$FOLLOW_PROFILE' (expected default|strict)"
    exit 14
    ;;
esac

UAV_BACKEND="$(get_launch_assignment_value "uav_backend" "$UAV_BACKEND")"
case "$UAV_BACKEND" in
  setpose|controller) ;;
  *)
    echo "[run_follow_with_bag] invalid uav_backend='$UAV_BACKEND' (expected setpose|controller)"
    exit 9
    ;;
esac

LEADER_MODE="$(get_launch_assignment_value "leader_mode" "$LEADER_MODE")"
LEADER_DEPENDENCY_MODE="$(get_launch_assignment_value "leader_dependency_mode" "$LEADER_DEPENDENCY_MODE")"
case "$LEADER_DEPENDENCY_MODE" in
  uav_only|ugv_state) ;;
  *)
    echo "[run_follow_with_bag] invalid leader_dependency_mode='$LEADER_DEPENDENCY_MODE' (expected uav_only|ugv_state)"
    exit 10
    ;;
esac
if [[ "$LEADER_DEPENDENCY_MODE" == "uav_only" ]]; then
  case "$LEADER_MODE" in
    pose|estimate) ;;
    *)
      echo "[run_follow_with_bag] leader_dependency_mode=uav_only requires leader_mode:=pose (or estimate alias), got '$LEADER_MODE'"
      exit 11
      ;;
  esac
fi

if [[ "$PREFLIGHT_REQUIRE_UGV_ODOM_TOPIC" == "auto" ]]; then
  if [[ "$LEADER_DEPENDENCY_MODE" == "uav_only" ]]; then
    PREFLIGHT_REQUIRE_UGV_ODOM_TOPIC="false"
  else
    PREFLIGHT_REQUIRE_UGV_ODOM_TOPIC="true"
  fi
fi
if [[ "$PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS" == "auto" ]]; then
  if [[ "$LEADER_DEPENDENCY_MODE" == "uav_only" ]]; then
    PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS="true"
  else
    PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS="false"
  fi
fi
if [[ "$PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW" == "auto" ]]; then
  if [[ "$LEADER_DEPENDENCY_MODE" == "uav_only" ]]; then
    PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW="true"
  else
    PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW="false"
  fi
fi

EFFECTIVE_LEADER_PERCEPTION_ENABLE="$(get_launch_assignment_value "leader_perception_enable" "$LEADER_PERCEPTION_ENABLE")"
EFFECTIVE_START_LEADER_ESTIMATOR="$(get_launch_assignment_value "start_leader_estimator" "$START_LEADER_ESTIMATOR")"
EFFECTIVE_SHUTDOWN_WHEN_UGV_DONE="$(get_launch_assignment_value "shutdown_when_ugv_done" "$SHUTDOWN_WHEN_UGV_DONE")"
EFFECTIVE_YOLO_WEIGHTS="$(get_launch_assignment_value "yolo_weights" "$YOLO_WEIGHTS")"
EFFECTIVE_YOLO_DEVICE="$(get_launch_assignment_value "yolo_device" "$YOLO_DEVICE")"
EFFECTIVE_UGV_START_DELAY_S="$(get_launch_assignment_value "ugv_start_delay_s" "$UGV_START_DELAY_S")"
UAV_ONLY_AUTO_START_DELAY_APPLIED="false"
if [[ "$LEADER_DEPENDENCY_MODE" == "uav_only" ]] && ! has_launch_assignment_key "ugv_start_delay_s"; then
  EFFECTIVE_UGV_START_DELAY_S="$UAV_ONLY_UGV_START_DELAY_S"
  UAV_ONLY_AUTO_START_DELAY_APPLIED="true"
fi
EFFECTIVE_UGV_READY_REQUIRE_LEADER_FLOW="$(get_launch_assignment_value "ugv_ready_require_leader_flow" "false")"
EFFECTIVE_UGV_READY_TIMEOUT_S="$(get_launch_assignment_value "ugv_ready_timeout_s" "8.0")"
UAV_ONLY_AUTO_READY_GATE_APPLIED="false"
if [[ "$LEADER_DEPENDENCY_MODE" == "uav_only" ]] && ! has_launch_assignment_key "ugv_ready_require_leader_flow"; then
  if is_truthy "$UAV_ONLY_REQUIRE_LEADER_FLOW"; then
    EFFECTIVE_UGV_READY_REQUIRE_LEADER_FLOW="true"
    UAV_ONLY_AUTO_READY_GATE_APPLIED="true"
  fi
fi
if [[ "$LEADER_DEPENDENCY_MODE" == "uav_only" ]] && ! has_launch_assignment_key "ugv_ready_timeout_s"; then
  EFFECTIVE_UGV_READY_TIMEOUT_S="$UAV_ONLY_READY_TIMEOUT_S"
fi
if [[ "$LEADER_DEPENDENCY_MODE" == "uav_only" ]] && is_truthy "$UAV_ONLY_REQUIRE_YOLO"; then
  if [[ -z "${EFFECTIVE_YOLO_WEIGHTS}" ]]; then
    echo "[run_follow_with_bag] leader_dependency_mode=uav_only requires yolo_weights:=<path> (UAV_ONLY_REQUIRE_YOLO=true)"
    exit 12
  fi
  if [[ ! -f "${EFFECTIVE_YOLO_WEIGHTS}" ]]; then
    echo "[run_follow_with_bag] YOLO weights not found: ${EFFECTIVE_YOLO_WEIGHTS}"
    exit 13
  fi
fi
EFFECTIVE_CONTROLLER_CMD_TOPIC="$(get_launch_assignment_value "controller_cmd_topic" "/${UAV_NAME}/psdk_ros2/flight_control_setpoint_ENUposition_yaw")"
EFFECTIVE_CONTROLLER_PAN_TOPIC="$(get_launch_assignment_value "controller_pan_topic" "/${UAV_NAME}/update_pan")"
EFFECTIVE_CONTROLLER_TILT_TOPIC="$(get_launch_assignment_value "controller_tilt_topic" "/${UAV_NAME}/update_tilt")"
INTENT_TOPIC="/${UAV_NAME}/pose_cmd"
EFFECTIVE_PARAMS_FILE="<launch_default>"
if has_launch_assignment_key "params_file"; then
  EFFECTIVE_PARAMS_FILE="$(get_launch_assignment_value "params_file" "$EFFECTIVE_PARAMS_FILE")"
elif [[ "$FOLLOW_PROFILE" == "strict" ]]; then
  EFFECTIVE_PARAMS_FILE="$STRICT_PARAMS_FILE"
fi
if [[ "$FOLLOW_PROFILE" == "strict" ]] && ! has_launch_assignment_key "params_file"; then
  if [[ ! -f "$STRICT_PARAMS_FILE" ]]; then
    echo "[run_follow_with_bag] strict profile params file not found: $STRICT_PARAMS_FILE"
    exit 15
  fi
fi

acquire_mode_lock() {
  if ! is_truthy "$MODE_LOCK_ENABLE"; then
    return
  fi
  mkdir -p "$STATE_DIR"

  if [[ -f "$MODE_FILE" ]]; then
    local active_mode=""
    local active_pid=""
    read -r active_mode active_pid < "$MODE_FILE" || true
    if [[ -n "$active_pid" ]] && kill -0 "$active_pid" 2>/dev/null; then
      if [[ "$active_mode" != "$CURRENT_MODE" ]]; then
        echo "[run_follow_with_bag] mode conflict: active mode '$active_mode' (pid=$active_pid). Stop it before starting '$CURRENT_MODE'."
        exit 21
      fi
      echo "[run_follow_with_bag] '$CURRENT_MODE' already running (pid=$active_pid)."
      exit 22
    fi
    rm -f "$MODE_FILE"
  fi

  printf '%s %s\n' "$CURRENT_MODE" "$$" > "$MODE_FILE"
}

release_mode_lock() {
  if ! is_truthy "$MODE_LOCK_ENABLE"; then
    return
  fi
  if [[ -f "$MODE_FILE" ]]; then
    local active_mode=""
    local active_pid=""
    read -r active_mode active_pid < "$MODE_FILE" || true
    if [[ "$active_mode" == "$CURRENT_MODE" && "$active_pid" == "$$" ]]; then
      rm -f "$MODE_FILE"
    fi
  fi
}

wait_for_pid_exit() {
  local pid="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))
  while kill -0 "$pid" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 0.2
  done
  return 0
}

stop_bag() {
  if [[ "$BAG_STOPPED" -eq 1 ]]; then
    return
  fi
  BAG_STOPPED=1

  if [[ -z "$BAG_PID" ]] || ! kill -0 "$BAG_PID" 2>/dev/null; then
    return
  fi

  kill -INT "$BAG_PID" 2>/dev/null || true
  if ! wait_for_pid_exit "$BAG_PID" "$BAG_STOP_INT_TIMEOUT_S"; then
    echo "[run_follow_with_bag] rosbag still running after SIGINT; escalating to SIGTERM"
    kill -TERM "$BAG_PID" 2>/dev/null || true
    if ! wait_for_pid_exit "$BAG_PID" "$BAG_STOP_TERM_TIMEOUT_S"; then
      echo "[run_follow_with_bag] rosbag still running after SIGTERM; escalating to SIGKILL"
      kill -KILL "$BAG_PID" 2>/dev/null || true
      wait_for_pid_exit "$BAG_PID" "$BAG_STOP_KILL_TIMEOUT_S" || true
    fi
  fi

  wait "$BAG_PID" 2>/dev/null || true
}

write_bag_summary() {
  if [[ "$SUMMARY_WRITTEN" -eq 1 ]]; then
    return
  fi
  SUMMARY_WRITTEN=1

  if [[ -d "$BAG_DIR" ]]; then
    ros2 bag info "$BAG_DIR" > "$BAG_INFO_FILE" 2>&1 || true
  fi
}

print_quick_exit_hints() {
  local launch_log="$1"
  local preflight_log="$2"
  echo "[run_follow_with_bag] quick-exit diagnostics:"
  if rg -q "simulator backend exited in controller mode" "$launch_log" 2>/dev/null; then
    echo "  - Controller backend simulator exited early."
    echo "    Check launch log for simulator traceback and verify /world/<world>/set_pose exists."
  fi
  if rg -q "set_pose failed for camera model|set_pose reported failure" "$launch_log" 2>/dev/null; then
    echo "  - Gazebo set_pose failures detected."
    echo "    Verify UAV/camera entities exist and UAV name/world match spawn."
  fi
  if rg -q "YOLO weights file not found|yolo_load_failed|ultralytics_not_installed" "$launch_log" 2>/dev/null; then
    echo "  - YOLO initialization issue detected."
    echo "    Verify yolo_weights path and Python env."
  fi
  if [[ -f "$preflight_log" ]]; then
    if rg -q "mode conflict|camera image did not publish|camera info did not publish|controller backend conflict|intent conflict" "$preflight_log" 2>/dev/null; then
      echo "  - Preflight conflict/readiness failure detected:"
      tail -n 20 "$preflight_log" | sed 's/^/    /'
    fi
  fi
}

soft_zero_velocity() {
  local zero_twist
  zero_twist="{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
  ros2 topic pub --once "$UGV_CMD_TOPIC" geometry_msgs/msg/TwistStamped "$zero_twist" >/dev/null 2>&1 || true
  ros2 topic pub --once "/${UGV_NAMESPACE}/platform/cmd_vel" geometry_msgs/msg/TwistStamped "$zero_twist" >/dev/null 2>&1 || true
}

gazebo_world_name() {
  local world="$1"
  if [[ "$world" == "construction" ]]; then
    printf 'office_construction'
  else
    printf '%s' "$world"
  fi
}

soft_reset_uav_pose() {
  local gz_world
  gz_world="$(gazebo_world_name "$EFFECTIVE_WORLD")"
  ros2 service call "/world/${gz_world}/set_pose" ros_gz_interfaces/srv/SetEntityPose \
    "{entity:{name:'${UAV_NAME}',type:2},pose:{position:{x:${RESET_UAV_X},y:${RESET_UAV_Y},z:${RESET_UAV_Z}},orientation:{x:0.0,y:0.0,z:0.0,w:1.0}}}" \
    >/dev/null 2>&1 || true
}

cleanup() {
  stop_bag
  write_bag_summary
  release_mode_lock
}

trap cleanup EXIT INT TERM

cd "$WS_ROOT"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

mkdir -p "$RUN_DIR"
acquire_mode_lock

if is_truthy "$PREFLIGHT_ENABLE"; then
  if [[ ! -f "$PREFLIGHT_SCRIPT" ]]; then
    echo "[run_follow_with_bag] preflight script not found: $PREFLIGHT_SCRIPT"
    exit 11
  fi
  set +e
  PREFLIGHT_TIMEOUT_S="$PREFLIGHT_TIMEOUT_S" \
  PREFLIGHT_ACTIVATE_CONTROLLER="$PREFLIGHT_ACTIVATE_CONTROLLER" \
  PREFLIGHT_REQUIRE_CLOCK="$PREFLIGHT_REQUIRE_CLOCK" \
  PREFLIGHT_REQUIRE_SET_POSE_SERVICE="$PREFLIGHT_REQUIRE_SET_POSE_SERVICE" \
  PREFLIGHT_REQUIRE_UAV_NAMESPACE_TOPICS="$PREFLIGHT_REQUIRE_UAV_NAMESPACE_TOPICS" \
  PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS="$PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS" \
  PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW="$PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW" \
  PREFLIGHT_MODE_GUARD_ENABLE="$PREFLIGHT_MODE_GUARD_ENABLE" \
  PREFLIGHT_FORBID_EXISTING_FOLLOW_NODES="$PREFLIGHT_FORBID_EXISTING_FOLLOW_NODES" \
  PREFLIGHT_NODE_LIST_TIMEOUT_S="$PREFLIGHT_NODE_LIST_TIMEOUT_S" \
  PREFLIGHT_SET_POSE_TIMEOUT_S="$PREFLIGHT_SET_POSE_TIMEOUT_S" \
  PREFLIGHT_UAV_TOPICS_TIMEOUT_S="$PREFLIGHT_UAV_TOPICS_TIMEOUT_S" \
  PREFLIGHT_REQUIRE_UGV_ODOM_TOPIC="$PREFLIGHT_REQUIRE_UGV_ODOM_TOPIC" \
  bash "$PREFLIGHT_SCRIPT" "$UGV_ODOM_TOPIC" "$PREFLIGHT_CONTROLLER_MANAGER" "$PREFLIGHT_CONTROLLER_NAME" "$EFFECTIVE_WORLD" "$UAV_NAME" "$UAV_BACKEND" "$LEADER_DEPENDENCY_MODE" \
    > "$PREFLIGHT_LOG" 2>&1
  PREFLIGHT_RC=$?
  set -e
  if [[ "$PREFLIGHT_RC" -ne 0 ]]; then
    echo "[run_follow_with_bag] preflight failed (rc=$PREFLIGHT_RC); see $PREFLIGHT_LOG"
    exit 12
  fi
fi

# ros2 bag record requires the output folder to not exist.
# If this run id was used before, keep previous bags and create a unique bag path.
if [[ -e "$BAG_DIR" ]]; then
  _bag_idx=1
  while [[ -e "${RUN_DIR}/bag_${_bag_idx}" ]]; do
    _bag_idx=$((_bag_idx + 1))
  done
  BAG_DIR="${RUN_DIR}/bag_${_bag_idx}"
fi

GIT_COMMIT="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
RUN_DATE_UTC="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"

BAG_TOPICS=(
  /clock
  /coord/events
  "/${UGV_NAMESPACE}/platform/odom"
  "/${UGV_NAMESPACE}/platform/odom/filtered"
  "$UGV_ODOM_TOPIC"
  "/${UGV_NAMESPACE}/cmd_vel"
  "/${UGV_NAMESPACE}/platform/cmd_vel"
  "/${UGV_NAMESPACE}/tf"
  "/${UGV_NAMESPACE}/tf_static"
  "/${UAV_NAME}/pose_cmd"
  "/${UAV_NAME}/pose"
  "/${UAV_NAME}/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
  "/${UAV_NAME}/update_pan"
  "/${UAV_NAME}/update_tilt"
  /coord/leader_estimate
  /coord/leader_estimate_odom
  /coord/leader_estimate_status
  /coord/follow_debug_status
)
if is_truthy "$RECORD_DEBUG_IMAGE"; then
  BAG_TOPICS+=(/coord/leader_debug_image)
fi

declare -A _bag_topic_seen
_bag_topics_unique=()
for _topic in "${BAG_TOPICS[@]}"; do
  if [[ -z "$_topic" ]]; then
    continue
  fi
  if [[ -z "${_bag_topic_seen["$_topic"]+x}" ]]; then
    _bag_topic_seen["$_topic"]=1
    _bag_topics_unique+=("$_topic")
  fi
done
BAG_TOPICS=("${_bag_topics_unique[@]}")
unset _bag_topic_seen
unset _bag_topics_unique

{
  echo "run_id: \"$RUN_ID\""
  echo "date_utc: \"$RUN_DATE_UTC\""
  echo "git_commit: \"$GIT_COMMIT\""
  echo "world: \"$EFFECTIVE_WORLD\""
  echo "uav_name: \"$UAV_NAME\""
  echo "uav_backend: \"$UAV_BACKEND\""
  echo "leader_mode: \"$LEADER_MODE\""
  echo "leader_dependency_mode: \"$LEADER_DEPENDENCY_MODE\""
  echo "ugv_namespace: \"$UGV_NAMESPACE\""
  echo "ugv_cmd_topic: \"$UGV_CMD_TOPIC\""
  echo "ugv_odom_topic: \"$UGV_ODOM_TOPIC\""
  echo "leader_perception_enable: \"$EFFECTIVE_LEADER_PERCEPTION_ENABLE\""
  echo "start_leader_estimator: \"$EFFECTIVE_START_LEADER_ESTIMATOR\""
  echo "shutdown_when_ugv_done: \"$EFFECTIVE_SHUTDOWN_WHEN_UGV_DONE\""
  echo "yolo_weights: \"${EFFECTIVE_YOLO_WEIGHTS}\""
  echo "yolo_device: \"$EFFECTIVE_YOLO_DEVICE\""
  echo "follow_profile: \"$FOLLOW_PROFILE\""
  echo "params_file_effective: \"$EFFECTIVE_PARAMS_FILE\""
  echo "ugv_start_delay_s: \"$EFFECTIVE_UGV_START_DELAY_S\""
  echo "ugv_ready_require_leader_flow: \"$EFFECTIVE_UGV_READY_REQUIRE_LEADER_FLOW\""
  echo "ugv_ready_timeout_s: \"$EFFECTIVE_UGV_READY_TIMEOUT_S\""
  echo "uav_only_auto_start_delay_applied: \"$UAV_ONLY_AUTO_START_DELAY_APPLIED\""
  echo "uav_only_auto_ready_gate_applied: \"$UAV_ONLY_AUTO_READY_GATE_APPLIED\""
  echo "control_chain:"
  echo "  profile: \"planner_to_backend_v1\""
  echo "  manual_uav_cmd_allowed: \"false\""
  echo "  planner_node: \"follow_uav\""
  echo "  intent_topic: \"$INTENT_TOPIC\""
  echo "  leader_input_type: \"$LEADER_MODE\""
  echo "  leader_dependency_mode: \"$LEADER_DEPENDENCY_MODE\""
  echo "  backend: \"$UAV_BACKEND\""
  if [[ "$UAV_BACKEND" == "controller" ]]; then
    echo "  adapter_cmd_topic: \"$EFFECTIVE_CONTROLLER_CMD_TOPIC\""
    echo "  adapter_pan_topic: \"$EFFECTIVE_CONTROLLER_PAN_TOPIC\""
    echo "  adapter_tilt_topic: \"$EFFECTIVE_CONTROLLER_TILT_TOPIC\""
  else
    echo "  adapter_service: \"/world/$(gazebo_world_name "$EFFECTIVE_WORLD")/set_pose\""
  fi
  echo "bag_storage: \"$BAG_STORAGE\""
  echo "bag_output_dir: \"$BAG_DIR\""
  echo "preflight:"
  echo "  enabled: \"$PREFLIGHT_ENABLE\""
  echo "  timeout_s: \"$PREFLIGHT_TIMEOUT_S\""
  echo "  activate_controller: \"$PREFLIGHT_ACTIVATE_CONTROLLER\""
  echo "  controller_manager: \"$PREFLIGHT_CONTROLLER_MANAGER\""
  echo "  controller_name: \"$PREFLIGHT_CONTROLLER_NAME\""
  echo "  require_set_pose_service: \"$PREFLIGHT_REQUIRE_SET_POSE_SERVICE\""
  echo "  require_uav_namespace_topics: \"$PREFLIGHT_REQUIRE_UAV_NAMESPACE_TOPICS\""
  echo "  require_uav_camera_topics: \"$PREFLIGHT_REQUIRE_UAV_CAMERA_TOPICS\""
  echo "  require_uav_camera_flow: \"$PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW\""
  echo "  mode_guard_enable: \"$PREFLIGHT_MODE_GUARD_ENABLE\""
  echo "  forbid_existing_follow_nodes: \"$PREFLIGHT_FORBID_EXISTING_FOLLOW_NODES\""
  echo "  node_list_timeout_s: \"$PREFLIGHT_NODE_LIST_TIMEOUT_S\""
  echo "  require_ugv_odom_topic: \"$PREFLIGHT_REQUIRE_UGV_ODOM_TOPIC\""
  echo "  set_pose_timeout_s: \"$PREFLIGHT_SET_POSE_TIMEOUT_S\""
  echo "  uav_topics_timeout_s: \"$PREFLIGHT_UAV_TOPICS_TIMEOUT_S\""
  echo "bag_topics:"
  for topic in "${BAG_TOPICS[@]}"; do
    echo "  - \"$topic\""
  done
  echo "extra_launch_args:"
  if [[ "${#EXTRA_LAUNCH_ARGS[@]}" -eq 0 ]]; then
    echo "  - \"\""
  else
    for arg in "${EXTRA_LAUNCH_ARGS[@]}"; do
      echo "  - \"$arg\""
    done
  fi
} > "$META_FILE"

if is_truthy "$SOFT_RESET_BEFORE_RUN"; then
  soft_zero_velocity
  if is_truthy "$RESET_UAV_BEFORE_RUN"; then
    soft_reset_uav_pose
  fi
fi

ros2 bag record -o "$BAG_DIR" -s "$BAG_STORAGE" --topics "${BAG_TOPICS[@]}" > "$BAG_LOG" 2>&1 &
BAG_PID="$!"
sleep 1.0
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "[run_follow_with_bag] rosbag failed to start; see $BAG_LOG"
  exit 2
fi

LAUNCH_CMD=(
  ros2 launch lrs_halmstad run_round_follow_motion.launch.py
  "world:=${EFFECTIVE_WORLD}"
  "uav_name:=${UAV_NAME}"
  "leader_mode:=${LEADER_MODE}"
  "leader_dependency_mode:=${LEADER_DEPENDENCY_MODE}"
  "uav_backend:=${UAV_BACKEND}"
  "shutdown_when_ugv_done:=${SHUTDOWN_WHEN_UGV_DONE}"
)
if [[ "$FOLLOW_PROFILE" == "strict" ]] && ! has_launch_assignment_key "params_file"; then
  LAUNCH_CMD+=("params_file:=${STRICT_PARAMS_FILE}")
fi
if ! has_launch_assignment_key "ugv_start_delay_s"; then
  LAUNCH_CMD+=("ugv_start_delay_s:=${EFFECTIVE_UGV_START_DELAY_S}")
fi
if ! has_launch_assignment_key "ugv_ready_require_leader_flow"; then
  LAUNCH_CMD+=("ugv_ready_require_leader_flow:=${EFFECTIVE_UGV_READY_REQUIRE_LEADER_FLOW}")
fi
if ! has_launch_assignment_key "ugv_ready_timeout_s"; then
  LAUNCH_CMD+=("ugv_ready_timeout_s:=${EFFECTIVE_UGV_READY_TIMEOUT_S}")
fi
if ! has_launch_assignment_key "leader_perception_enable"; then
  LAUNCH_CMD+=("leader_perception_enable:=${LEADER_PERCEPTION_ENABLE}")
fi
if ! has_launch_assignment_key "start_leader_estimator"; then
  LAUNCH_CMD+=("start_leader_estimator:=${START_LEADER_ESTIMATOR}")
fi
if [[ -n "$YOLO_WEIGHTS" ]] && ! has_launch_assignment_key "yolo_weights"; then
  LAUNCH_CMD+=("yolo_weights:=${YOLO_WEIGHTS}")
fi
if [[ -n "$YOLO_DEVICE" ]] && ! has_launch_assignment_key "yolo_device"; then
  LAUNCH_CMD+=("yolo_device:=${YOLO_DEVICE}")
fi
LAUNCH_CMD+=("${EXTRA_LAUNCH_ARGS[@]}")

echo "[run_follow_with_bag] run_id=${RUN_ID}"
echo "[run_follow_with_bag] run_dir=${RUN_DIR}"
echo "[run_follow_with_bag] launching: ${LAUNCH_CMD[*]}"

LAUNCH_T0=$SECONDS
set +e
"${LAUNCH_CMD[@]}" 2>&1 | tee "$LAUNCH_LOG"
LAUNCH_RC=${PIPESTATUS[0]}
set -e
LAUNCH_DURATION_S=$((SECONDS - LAUNCH_T0))

if is_truthy "$SOFT_RESET_AFTER_RUN"; then
  soft_zero_velocity
fi

stop_bag
write_bag_summary

if (( LAUNCH_DURATION_S < 15 )) || [[ "$LAUNCH_RC" -ne 0 ]]; then
  print_quick_exit_hints "$LAUNCH_LOG" "$PREFLIGHT_LOG"
fi

echo "[run_follow_with_bag] launch_rc=${LAUNCH_RC}"
echo "[run_follow_with_bag] launch_duration_s=${LAUNCH_DURATION_S}"
echo "[run_follow_with_bag] artifacts:"
echo "  - ${META_FILE}"
echo "  - ${PREFLIGHT_LOG}"
echo "  - ${LAUNCH_LOG}"
echo "  - ${BAG_LOG}"
echo "  - ${BAG_INFO_FILE}"
echo "  - ${BAG_DIR}"

exit "$LAUNCH_RC"

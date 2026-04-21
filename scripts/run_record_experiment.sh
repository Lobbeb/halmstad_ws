#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
WORLD="warehouse"
MODE="follow"
UAV_NAME="dji0"
UGV_NAMESPACE="a201_0000"
PROFILE="default"
TAG=""
RUN_DIR=""
SESSION_STATE_FILE=""
DRY_RUN=false
BAG_STORAGE="mcap"
VALIDATE_PREFLIGHT=true
PREFLIGHT_TIMEOUT_S="20"
PREFLIGHT_STRICT=true
ORIGINAL_ARGS=("$@")

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
    mode:=*)
      MODE="${arg#mode:=}"
      ;;
    uav_name:=*)
      UAV_NAME="${arg#uav_name:=}"
      ;;
    ugv_namespace:=*)
      UGV_NAMESPACE="${arg#ugv_namespace:=}"
      ;;
    profile:=*)
      PROFILE="${arg#profile:=}"
      ;;
    tag:=*)
      TAG="${arg#tag:=}"
      ;;
    out:=*)
      RUN_DIR="${arg#out:=}"
      ;;
    session_state:=*)
      SESSION_STATE_FILE="${arg#session_state:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    validate_preflight:=*)
      VALIDATE_PREFLIGHT="${arg#validate_preflight:=}"
      ;;
    preflight_timeout_s:=*)
      PREFLIGHT_TIMEOUT_S="${arg#preflight_timeout_s:=}"
      ;;
    preflight_strict:=*)
      PREFLIGHT_STRICT="${arg#preflight_strict:=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      echo "Usage: $0 [world] [mode:=follow|yolo] [uav_name:=dji0] [ugv_namespace:=a201_0000] [profile:=default|step2_light|vision|lab_full] [tag:=name] [out:=bags/experiments/...] [session_state:=/tmp/halmstad_ws/tmux_sessions/<session>.env] [dry_run:=true|false] [validate_preflight:=true|false] [preflight_timeout_s:=20] [preflight_strict:=true|false]" >&2
      exit 2
      ;;
  esac
done

case "$MODE" in
  follow|yolo)
    ;;
  *)
    echo "Invalid mode: $MODE" >&2
    exit 2
    ;;
esac

case "$PROFILE" in
  default|step2_light|vision|lab_full)
    ;;
  *)
    echo "Invalid profile: $PROFILE" >&2
    exit 2
    ;;
esac

case "$DRY_RUN" in
  true|false)
    ;;
  *)
    echo "Invalid dry_run option: $DRY_RUN" >&2
    exit 2
    ;;
esac

case "$VALIDATE_PREFLIGHT" in
  true|false)
    ;;
  *)
    echo "Invalid validate_preflight option: $VALIDATE_PREFLIGHT" >&2
    exit 2
    ;;
esac

case "$PREFLIGHT_STRICT" in
  true|false)
    ;;
  *)
    echo "Invalid preflight_strict option: $PREFLIGHT_STRICT" >&2
    exit 2
    ;;
esac

if ! [[ "$PREFLIGHT_TIMEOUT_S" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Invalid preflight_timeout_s option: $PREFLIGHT_TIMEOUT_S" >&2
  exit 2
fi

timestamp="$(date +%m%d-%H%M%S)"
safe_tag="$(printf '%s' "$TAG" | tr -c 'A-Za-z0-9_.-' '_')"
run_name="${MODE}_${timestamp}"
if [ -n "$safe_tag" ]; then
  run_name="${safe_tag}_${run_name}"
fi

if [ -n "$RUN_DIR" ]; then
  case "$RUN_DIR" in
    /*)
      RUN_DIR_ABS="$RUN_DIR"
      ;;
    *)
      RUN_DIR_ABS="$WS_ROOT/$RUN_DIR"
      ;;
  esac
else
  RUN_DIR_ABS="$WS_ROOT/bags/experiments/$WORLD/$run_name"
fi

BAG_DIR="$RUN_DIR_ABS/bag"
TOPICS_FILE="$RUN_DIR_ABS/topics.txt"
METADATA_FILE="$RUN_DIR_ABS/metadata.json"
ARTIFACTS_DIR="$RUN_DIR_ABS/artifacts"
SESSION_STATE_COPY="$ARTIFACTS_DIR/session_state.env"
FOLLOW_DEFAULTS_FILE="$WS_ROOT/src/lrs_halmstad/config/run_follow_defaults.yaml"
FOLLOW_DEFAULTS_COPY="$ARTIFACTS_DIR/run_follow_defaults.yaml"
ROSBAG_QOS_OVERRIDES_FILE="$WS_ROOT/config/rosbag_qos.yaml"
ROSBAG_QOS_OVERRIDES_COPY="$ARTIFACTS_DIR/rosbag_qos.yaml"
GIT_STATUS_FILE="$RUN_DIR_ABS/git_status.txt"
PREFLIGHT_REPORT_FILE="$RUN_DIR_ABS/preflight_check.txt"
BAG_INFO_FILE="$RUN_DIR_ABS/bag_info.txt"
RUN_SUMMARY_FILE="$RUN_DIR_ABS/run_summary.json"
MAP_ARTIFACT_DIR="$ARTIFACTS_DIR/map"
ROUTE_ARTIFACT_DIR="$ARTIFACTS_DIR/route"

TOPICS=(
  "/clock"
  "/coord/events"
  "/coord/leader_estimate_fault"
  "/$UGV_NAMESPACE/amcl_pose_odom"
  "/$UGV_NAMESPACE/cmd_vel"
  "/$UGV_NAMESPACE/platform/cmd_vel"
  "/$UGV_NAMESPACE/platform/odom"
  "/$UGV_NAMESPACE/platform/odom/filtered"
  "/$UGV_NAMESPACE/tf"
  "/$UGV_NAMESPACE/tf_static"
  "/$UGV_NAMESPACE/sensors/lidar2d_0/scan"
  "/$UAV_NAME/pose"
  "/$UAV_NAME/pose_cmd"
  "/$UAV_NAME/pose_cmd/odom"
  "/$UAV_NAME/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
  "/$UAV_NAME/update_pan"
  "/$UAV_NAME/update_tilt"
  "/$UAV_NAME/camera0/actual/center_pose"
  "/$UAV_NAME/camera0/target/center_pose"
  "/$UAV_NAME/camera0/target/world_yaw_rad"
  "/$UAV_NAME/camera0/target_horizontal_distance_m"
  "/$UAV_NAME/camera0/target_distance_3d_m"
  "/$UAV_NAME/camera0/target_vertical_delta_m"
  "/$UAV_NAME/camera0/debug/tilt_target_cmd_deg"
  "/$UAV_NAME/camera0/debug/tracking_uav_pose_source"
  "/$UAV_NAME/camera0/debug/image_error_x_deg"
  "/$UAV_NAME/camera0/debug/image_error_y_deg"
  "/$UAV_NAME/camera0/debug/pan_image_correction_deg"
  "/$UAV_NAME/camera0/debug/tilt_image_correction_deg"
)

if [[ "$WORLD" == baylands* ]]; then
  TOPICS+=("/$UGV_NAMESPACE/ground_truth/odom")
fi

if [ "$MODE" = "follow" ]; then
  TOPICS+=(
    "/$UAV_NAME/follow/target/pan_deg"
    "/$UAV_NAME/follow/target/tilt_deg"
    "/$UAV_NAME/follow/actual/pan_deg"
    "/$UAV_NAME/follow/actual/tilt_deg"
    "/$UAV_NAME/follow/error/pan_deg"
    "/$UAV_NAME/follow/error/tilt_deg"
    "/$UAV_NAME/follow/target/d_target_m"
    "/$UAV_NAME/follow/target/xy_target_m"
    "/$UAV_NAME/follow/target/anchor_pose"
    "/$UAV_NAME/follow/target/z_min_m"
    "/$UAV_NAME/follow/target/d_euclidean_m"
    "/$UAV_NAME/follow/target/yaw_rad"
    "/$UAV_NAME/follow/actual/xy_distance_m"
    "/$UAV_NAME/follow/actual/distance_3d_m"
    "/$UAV_NAME/follow/actual/yaw_rad"
    "/$UAV_NAME/follow/error/xy_distance_m"
    "/$UAV_NAME/follow/error/anchor_distance_m"
    "/$UAV_NAME/follow/error/anchor_along_m"
    "/$UAV_NAME/follow/error/anchor_cross_m"
    "/$UAV_NAME/follow/error/yaw_rad"
    "/$UAV_NAME/follow/debug/yaw_target_raw_rad"
    "/$UAV_NAME/follow/debug/yaw_target_unwrapped_rad"
    "/$UAV_NAME/follow/debug/yaw_actual_unwrapped_rad"
    "/$UAV_NAME/follow/debug/yaw_error_raw_rad"
    "/$UAV_NAME/follow/debug/yaw_wrap_correction_rad"
    "/$UAV_NAME/follow/debug/yaw_wrap_active"
    "/$UAV_NAME/follow/debug/yaw_step_limit_rad"
    "/$UAV_NAME/follow/debug/yaw_cmd_delta_rad"
    "/$UAV_NAME/follow/debug/yaw_mode"
    "/$UAV_NAME/follow/debug/leader_heading_source"
    "/$UAV_NAME/follow/debug/leader_follow_yaw_rad"
    "/$UAV_NAME/follow/debug/leader_estimate_yaw_rad"
    "/$UAV_NAME/follow/debug/leader_actual_heading_yaw_rad"
  )
fi

if [ "$MODE" = "yolo" ]; then
  TOPICS+=(
    "/coord/leader_detection"
    "/coord/leader_estimate"
    "/coord/leader_distance_debug"
    "/coord/leader_estimate_status"
    "/coord/leader_estimate_error"
    "/coord/leader_follow_point"
    "/coord/leader_follow_point_status"
    "/coord/leader_planned_target"
    "/coord/leader_planned_target_status"
    "/coord/leader_visual_control"
    "/coord/leader_visual_control_status"
    "/coord/leader_visual_actuation_bridge_status"
    "/coord/leader_detection_status"
    "/coord/leader_selected_target"
    "/coord/leader_selected_target_filtered"
    "/coord/leader_selected_target_filtered_status"
    "/coord/leader_visual_target_estimate"
    "/coord/leader_visual_target_estimate_status"
    "/$UAV_NAME/follow/target/pan_deg"
    "/$UAV_NAME/follow/target/tilt_deg"
    "/$UAV_NAME/follow/actual/pan_deg"
    "/$UAV_NAME/follow/actual/tilt_deg"
    "/$UAV_NAME/follow/error/pan_deg"
    "/$UAV_NAME/follow/error/tilt_deg"
  )
fi

if [ "$PROFILE" = "vision" ]; then
  TOPICS+=(
    "/coord/leader_debug_image"
    "/$UAV_NAME/camera0/image_raw"
    "/$UAV_NAME/camera0/camera_info"
  )
fi

if [ "$PROFILE" = "lab_full" ]; then
  TOPICS+=(
    "/coord/leader_debug_image"
    "/$UAV_NAME/camera0/image_raw"
    "/$UAV_NAME/camera0/camera_info"
    "/$UAV_NAME/camera0/depth_image"
    "/$UGV_NAMESPACE/sensors/camera_0/color/image"
    "/$UGV_NAMESPACE/sensors/camera_0/color/compressed"
    "/$UGV_NAMESPACE/sensors/camera_0/color/camera_info"
    "/$UGV_NAMESPACE/sensors/camera_0/depth/image"
    "/$UGV_NAMESPACE/sensors/camera_0/depth/compressedDepth"
    "/$UGV_NAMESPACE/planned_path"
    "/$UGV_NAMESPACE/initialpose"
    "/$UGV_NAMESPACE/test_goal_path"
  )
fi

shell_join() {
  local out=""
  local part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

json_escape() {
  local value="$1"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  value="${value//$'\n'/\\n}"
  value="${value//$'\r'/\\r}"
  value="${value//$'\t'/\\t}"
  printf '%s' "$value"
}

copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [ -f "$src" ]; then
    cp "$src" "$dst"
    return 0
  fi
  return 1
}

resolve_existing_file() {
  local candidate="$1"
  local session_dir=""
  if [ -z "$candidate" ]; then
    return 1
  fi

  if [ -f "$candidate" ]; then
    printf '%s' "$candidate"
    return 0
  fi

  if [ -f "$WS_ROOT/$candidate" ]; then
    printf '%s' "$WS_ROOT/$candidate"
    return 0
  fi

  if [ -n "${SESSION_STATE_SOURCE:-}" ]; then
    session_dir="$(dirname "$SESSION_STATE_SOURCE")"
    if [ -f "$session_dir/$candidate" ]; then
      printf '%s' "$session_dir/$candidate"
      return 0
    fi
  fi

  return 1
}

git_branch="$(git -C "$WS_ROOT" branch --show-current 2>/dev/null || true)"
git_head="$(git -C "$WS_ROOT" rev-parse --short HEAD 2>/dev/null || true)"
if [ -n "$(git -C "$WS_ROOT" status --porcelain=v1 2>/dev/null || true)" ]; then
  git_dirty=true
else
  git_dirty=false
fi

SESSION_NAME=""
SESSION_LAYOUT=""
SESSION_GUI=""
SESSION_MAP_PATH=""
SESSION_ROS_DOMAIN_ID=""
SESSION_RMW_IMPLEMENTATION=""
SESSION_OMNET="false"
SESSION_OMNET_NETWORK=""
SESSION_OMNET_UI=""
SESSION_FOLLOW_CMD=""
SESSION_RECORD_CMD=""
SESSION_GOAL_SEQUENCE_FILE=""
SESSION_GOAL_SEQUENCE_CSV=""
SESSION_STATE_SOURCE=""
SESSION_STATE_AVAILABLE=false

MAP_FILE_ARTIFACT=""
MAP_IMAGE_ARTIFACT=""
GOAL_SEQUENCE_FILE_ARTIFACT=""
GOAL_SEQUENCE_CSV_ARTIFACT=""
PREFLIGHT_STATUS="skipped"

RECORDING_STARTED=false
RECORD_EXIT_CODE=0
POSTRUN_SUMMARY_WRITTEN=false

if [ -n "$SESSION_STATE_FILE" ] && [ -f "$SESSION_STATE_FILE" ]; then
  SESSION_STATE_SOURCE="$SESSION_STATE_FILE"
  SESSION_STATE_AVAILABLE=true
  set +u
  source "$SESSION_STATE_FILE"
  set -u
  SESSION_NAME="${SESSION:-}"
  SESSION_LAYOUT="${LAYOUT:-}"
  SESSION_GUI="${EFFECTIVE_GUI:-}"
  SESSION_MAP_PATH="${MAP_PATH:-}"
  SESSION_ROS_DOMAIN_ID="${ROS_DOMAIN_ID_EFFECTIVE:-}"
  SESSION_RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_EFFECTIVE:-}"
  SESSION_OMNET="${OMNET:-false}"
  SESSION_OMNET_NETWORK="${OMNET_NETWORK:-}"
  SESSION_OMNET_UI="${OMNET_UI:-}"
  SESSION_FOLLOW_CMD="${FOLLOW_CMD_STR:-}"
  SESSION_RECORD_CMD="${RECORD_CMD_STR:-}"
  SESSION_GOAL_SEQUENCE_FILE="${GOAL_SEQUENCE_FILE:-}"
  SESSION_GOAL_SEQUENCE_CSV="${GOAL_SEQUENCE_CSV:-}"
fi

if [ "$PROFILE" = "lab_full" ] && [ "$SESSION_OMNET" = "true" ]; then
  TOPICS+=(
    "/omnet/sim_time"
    "/omnet/link_distance"
    "/omnet/rssi_dbm"
    "/omnet/snir_db"
    "/omnet/packet_error_rate"
    "/omnet/radio_distance"
  )
fi

hostname_value="$(hostname 2>/dev/null || true)"
started_at="$(date -Is)"
started_at_epoch="$(date +%s)"

if [ "$DRY_RUN" = true ]; then
  echo "Run dir: $RUN_DIR_ABS"
  echo "Bag dir: $BAG_DIR"
  echo "Profile: $PROFILE"
  echo "Topics: ${#TOPICS[@]}"
  printf '%s\n' "${TOPICS[@]}"
  exit 0
fi

mkdir -p "$RUN_DIR_ABS"
mkdir -p "$ARTIFACTS_DIR"
printf '%s\n' "${TOPICS[@]}" > "$TOPICS_FILE"
git -C "$WS_ROOT" status --short --branch > "$GIT_STATUS_FILE" 2>/dev/null || true

SESSION_STATE_ARTIFACT=""
if [ "$SESSION_STATE_AVAILABLE" = true ] && copy_if_exists "$SESSION_STATE_SOURCE" "$SESSION_STATE_COPY"; then
  SESSION_STATE_ARTIFACT="$SESSION_STATE_COPY"
fi

FOLLOW_DEFAULTS_ARTIFACT=""
if copy_if_exists "$FOLLOW_DEFAULTS_FILE" "$FOLLOW_DEFAULTS_COPY"; then
  FOLLOW_DEFAULTS_ARTIFACT="$FOLLOW_DEFAULTS_COPY"
fi

resolved_map_path=""
if resolved_map_path="$(resolve_existing_file "$SESSION_MAP_PATH")"; then
  mkdir -p "$MAP_ARTIFACT_DIR"
  MAP_FILE_ARTIFACT="$MAP_ARTIFACT_DIR/$(basename "$resolved_map_path")"
  cp "$resolved_map_path" "$MAP_FILE_ARTIFACT"

  case "$resolved_map_path" in
    *.yaml|*.yml)
      map_image_ref="$(awk '/^[[:space:]]*image[[:space:]]*:/ { sub(/^[[:space:]]*image[[:space:]]*:[[:space:]]*/, "", $0); print; exit }' "$resolved_map_path" 2>/dev/null || true)"
      map_image_ref="${map_image_ref%\"}"
      map_image_ref="${map_image_ref#\"}"
      map_image_ref="${map_image_ref%\'}"
      map_image_ref="${map_image_ref#\'}"
      if [ -n "$map_image_ref" ]; then
        case "$map_image_ref" in
          /*)
            map_image_source="$map_image_ref"
            ;;
          *)
            map_image_source="$(dirname "$resolved_map_path")/$map_image_ref"
            ;;
        esac
        if [ -f "$map_image_source" ]; then
          MAP_IMAGE_ARTIFACT="$MAP_ARTIFACT_DIR/$(basename "$map_image_source")"
          cp "$map_image_source" "$MAP_IMAGE_ARTIFACT"
        fi
      fi
      ;;
  esac
fi

resolved_goal_sequence_file=""
if resolved_goal_sequence_file="$(resolve_existing_file "$SESSION_GOAL_SEQUENCE_FILE")"; then
  mkdir -p "$ROUTE_ARTIFACT_DIR"
  GOAL_SEQUENCE_FILE_ARTIFACT="$ROUTE_ARTIFACT_DIR/$(basename "$resolved_goal_sequence_file")"
  cp "$resolved_goal_sequence_file" "$GOAL_SEQUENCE_FILE_ARTIFACT"
fi

if [ -n "$SESSION_GOAL_SEQUENCE_CSV" ]; then
  mkdir -p "$ROUTE_ARTIFACT_DIR"
  GOAL_SEQUENCE_CSV_ARTIFACT="$ROUTE_ARTIFACT_DIR/goal_sequence_inline.csv"
  printf '%s\n' "$SESSION_GOAL_SEQUENCE_CSV" > "$GOAL_SEQUENCE_CSV_ARTIFACT"
fi

ROSBAG_QOS_ARTIFACT=""
if [ -f "$ROSBAG_QOS_OVERRIDES_FILE" ]; then
  sed "s#/a201_0000/#/$UGV_NAMESPACE/#g" "$ROSBAG_QOS_OVERRIDES_FILE" > "$ROSBAG_QOS_OVERRIDES_COPY"
  ROSBAG_QOS_ARTIFACT="$ROSBAG_QOS_OVERRIDES_COPY"
fi

ROSBAG_QOS_ARGS=()
if [ -n "$ROSBAG_QOS_ARTIFACT" ]; then
  ROSBAG_QOS_ARGS+=(--qos-profile-overrides-path "$ROSBAG_QOS_ARTIFACT")
fi

ROSBAG_CUSTOM_DATA=(
  "world=$WORLD"
  "mode=$MODE"
  "profile=$PROFILE"
  "uav_name=$UAV_NAME"
  "ugv_namespace=$UGV_NAMESPACE"
  "run_name=$run_name"
)
if [ -n "$TAG" ]; then
  ROSBAG_CUSTOM_DATA+=("tag=$TAG")
fi

BAG_CMD=(ros2 bag record -o "$BAG_DIR" --storage "$BAG_STORAGE")
if [ "${#ROSBAG_QOS_ARGS[@]}" -gt 0 ]; then
  BAG_CMD+=("${ROSBAG_QOS_ARGS[@]}")
fi
BAG_CMD+=(--topics "${TOPICS[@]}")
if [ "${#ROSBAG_CUSTOM_DATA[@]}" -gt 0 ]; then
  BAG_CMD+=(--custom-data "${ROSBAG_CUSTOM_DATA[@]}")
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

run_preflight_checks() {
  local timeout_whole="${PREFLIGHT_TIMEOUT_S%.*}"
  local deadline_epoch=0
  local contract_rc=0
  local extras_rc=0
  local topic_list=""
  local topic=""
  local -a contract_env=()
  local -a extra_topics=()
  local -a missing_topics=()

  if [ -z "$timeout_whole" ] || [ "$timeout_whole" -lt 1 ] 2>/dev/null; then
    timeout_whole=1
  fi

  : > "$PREFLIGHT_REPORT_FILE"
  printf '[preflight] world=%s mode=%s profile=%s ugv=%s uav=%s timeout_s=%s\n' "$WORLD" "$MODE" "$PROFILE" "$UGV_NAMESPACE" "$UAV_NAME" "$PREFLIGHT_TIMEOUT_S" >> "$PREFLIGHT_REPORT_FILE"
  printf '[preflight] strict=%s\n' "$PREFLIGHT_STRICT" >> "$PREFLIGHT_REPORT_FILE"

  contract_env=(
    "UGV_NAMESPACE=$UGV_NAMESPACE"
    "REQUIRE_FLOW=1"
    "REQUIRE_UAV_ADAPTER=1"
  )
  if [ "$MODE" = "follow" ] || [ "$MODE" = "yolo" ]; then
    contract_env+=("REQUIRE_FOLLOW_STACK=1")
  fi
  if [ "$MODE" = "yolo" ]; then
    contract_env+=("REQUIRE_DETECTION=1" "REQUIRE_ESTIMATOR=1")
  fi

  printf '[preflight] running contract_check\n' >> "$PREFLIGHT_REPORT_FILE"
  set +e
  env "${contract_env[@]}" ros2 run lrs_halmstad contract_check "$WORLD" "$UAV_NAME" "$PREFLIGHT_TIMEOUT_S" >> "$PREFLIGHT_REPORT_FILE" 2>&1
  contract_rc=$?
  set -e

  if [ "$contract_rc" -ne 0 ]; then
    printf '[preflight] contract_check failed rc=%s\n' "$contract_rc" >> "$PREFLIGHT_REPORT_FILE"
  else
    printf '[preflight] contract_check passed\n' >> "$PREFLIGHT_REPORT_FILE"
  fi

  if [ "$PROFILE" = "lab_full" ]; then
    extra_topics=(
      "/$UAV_NAME/camera0/depth_image"
      "/$UGV_NAMESPACE/sensors/camera_0/color/image"
      "/$UGV_NAMESPACE/sensors/camera_0/color/camera_info"
      "/$UGV_NAMESPACE/sensors/camera_0/depth/image"
      "/$UGV_NAMESPACE/planned_path"
      "/$UGV_NAMESPACE/initialpose"
    )
    if [ "$SESSION_OMNET" = "true" ]; then
      extra_topics+=(
        "/omnet/sim_time"
        "/omnet/link_distance"
        "/omnet/rssi_dbm"
        "/omnet/snir_db"
        "/omnet/packet_error_rate"
        "/omnet/radio_distance"
      )
    fi

    deadline_epoch=$(( $(date +%s) + timeout_whole ))
    while :; do
      topic_list="$(ros2 topic list 2>/dev/null || true)"
      missing_topics=()
      for topic in "${extra_topics[@]}"; do
        if ! printf '%s\n' "$topic_list" | grep -Fxq "$topic"; then
          missing_topics+=("$topic")
        fi
      done

      if [ "${#missing_topics[@]}" -eq 0 ]; then
        extras_rc=0
        break
      fi

      if [ "$(date +%s)" -ge "$deadline_epoch" ]; then
        extras_rc=1
        break
      fi
      sleep 1
    done

    if [ "$extras_rc" -ne 0 ]; then
      printf '[preflight] missing lab_full topics after %ss:\n' "$PREFLIGHT_TIMEOUT_S" >> "$PREFLIGHT_REPORT_FILE"
      printf '  %s\n' "${missing_topics[@]}" >> "$PREFLIGHT_REPORT_FILE"
    else
      printf '[preflight] lab_full topic availability check passed\n' >> "$PREFLIGHT_REPORT_FILE"
    fi
  fi

  if [ "$contract_rc" -eq 0 ] && [ "$extras_rc" -eq 0 ]; then
    return 0
  fi
  return 1
}

if [ "$VALIDATE_PREFLIGHT" = true ]; then
  if run_preflight_checks; then
    PREFLIGHT_STATUS="passed"
    echo "[run_record_experiment] Preflight checks passed"
  else
    PREFLIGHT_STATUS="failed"
    echo "[run_record_experiment] Preflight checks failed (see $PREFLIGHT_REPORT_FILE)" >&2
  fi
fi

invocation="$(shell_join "$0" "${ORIGINAL_ARGS[@]}")"
bag_command="$(shell_join "${BAG_CMD[@]}")"

{
  printf '{\n'
  printf '  "started_at": "%s",\n' "$(json_escape "$started_at")"
  printf '  "world": "%s",\n' "$(json_escape "$WORLD")"
  printf '  "mode": "%s",\n' "$(json_escape "$MODE")"
  printf '  "profile": "%s",\n' "$(json_escape "$PROFILE")"
  printf '  "uav_name": "%s",\n' "$(json_escape "$UAV_NAME")"
  printf '  "ugv_namespace": "%s",\n' "$(json_escape "$UGV_NAMESPACE")"
  printf '  "tag": "%s",\n' "$(json_escape "$TAG")"
  printf '  "run_name": "%s",\n' "$(json_escape "$run_name")"
  printf '  "run_dir": "%s",\n' "$(json_escape "$RUN_DIR_ABS")"
  printf '  "bag_dir": "%s",\n' "$(json_escape "$BAG_DIR")"
  printf '  "topics_file": "%s",\n' "$(json_escape "$TOPICS_FILE")"
  printf '  "session_state_file": "%s",\n' "$(json_escape "$SESSION_STATE_ARTIFACT")"
  printf '  "run_follow_defaults_file": "%s",\n' "$(json_escape "$FOLLOW_DEFAULTS_ARTIFACT")"
  printf '  "rosbag_qos_overrides_file": "%s",\n' "$(json_escape "$ROSBAG_QOS_ARTIFACT")"
  printf '  "git_status_file": "%s",\n' "$(json_escape "$GIT_STATUS_FILE")"
  printf '  "workspace_root": "%s",\n' "$(json_escape "$WS_ROOT")"
  printf '  "hostname": "%s",\n' "$(json_escape "$hostname_value")"
  printf '  "git_branch": "%s",\n' "$(json_escape "$git_branch")"
  printf '  "git_head": "%s",\n' "$(json_escape "$git_head")"
  printf '  "git_dirty": %s,\n' "$git_dirty"
  printf '  "session_name": "%s",\n' "$(json_escape "$SESSION_NAME")"
  printf '  "layout": "%s",\n' "$(json_escape "$SESSION_LAYOUT")"
  printf '  "gui": "%s",\n' "$(json_escape "$SESSION_GUI")"
  printf '  "map_path": "%s",\n' "$(json_escape "$SESSION_MAP_PATH")"
  printf '  "map_snapshot_file": "%s",\n' "$(json_escape "$MAP_FILE_ARTIFACT")"
  printf '  "map_image_snapshot_file": "%s",\n' "$(json_escape "$MAP_IMAGE_ARTIFACT")"
  printf '  "goal_sequence_file": "%s",\n' "$(json_escape "$SESSION_GOAL_SEQUENCE_FILE")"
  printf '  "goal_sequence_csv": "%s",\n' "$(json_escape "$SESSION_GOAL_SEQUENCE_CSV")"
  printf '  "goal_sequence_file_snapshot": "%s",\n' "$(json_escape "$GOAL_SEQUENCE_FILE_ARTIFACT")"
  printf '  "goal_sequence_csv_snapshot": "%s",\n' "$(json_escape "$GOAL_SEQUENCE_CSV_ARTIFACT")"
  printf '  "ros_domain_id": "%s",\n' "$(json_escape "$SESSION_ROS_DOMAIN_ID")"
  printf '  "rmw_implementation": "%s",\n' "$(json_escape "$SESSION_RMW_IMPLEMENTATION")"
  printf '  "omnet_enabled": "%s",\n' "$(json_escape "$SESSION_OMNET")"
  printf '  "omnet_network": "%s",\n' "$(json_escape "$SESSION_OMNET_NETWORK")"
  printf '  "omnet_ui": "%s",\n' "$(json_escape "$SESSION_OMNET_UI")"
  printf '  "follow_command": "%s",\n' "$(json_escape "$SESSION_FOLLOW_CMD")"
  printf '  "record_command_tmux": "%s",\n' "$(json_escape "$SESSION_RECORD_CMD")"
  printf '  "preflight_enabled": %s,\n' "$VALIDATE_PREFLIGHT"
  printf '  "preflight_strict": %s,\n' "$PREFLIGHT_STRICT"
  printf '  "preflight_timeout_s": "%s",\n' "$(json_escape "$PREFLIGHT_TIMEOUT_S")"
  printf '  "preflight_status": "%s",\n' "$(json_escape "$PREFLIGHT_STATUS")"
  printf '  "preflight_report_file": "%s",\n' "$(json_escape "$PREFLIGHT_REPORT_FILE")"
  printf '  "invocation": "%s",\n' "$(json_escape "$invocation")"
  printf '  "bag_command": "%s",\n' "$(json_escape "$bag_command")"
  printf '  "bag_info_file": "%s",\n' "$(json_escape "$BAG_INFO_FILE")"
  printf '  "run_summary_file": "%s",\n' "$(json_escape "$RUN_SUMMARY_FILE")"
  printf '  "bag_storage": "%s",\n' "$(json_escape "$BAG_STORAGE")"
  printf '  "topic_count": %s,\n' "${#TOPICS[@]}"
  printf '  "topics": [\n'
  for i in "${!TOPICS[@]}"; do
    suffix=","
    if [ "$i" -eq "$(( ${#TOPICS[@]} - 1 ))" ]; then
      suffix=""
    fi
    printf '    "%s"%s\n' "$(json_escape "${TOPICS[$i]}")" "$suffix"
  done
  printf '  ]\n'
  printf '}\n'
} > "$METADATA_FILE"

if [ "$PREFLIGHT_STATUS" = "failed" ] && [ "$PREFLIGHT_STRICT" = true ]; then
  echo "[run_record_experiment] Aborting recording due to strict preflight failure" >&2
  echo "[run_record_experiment] See: $PREFLIGHT_REPORT_FILE" >&2
  exit 2
fi

write_post_run_summary() {
  local ended_at=""
  local ended_at_epoch=""
  local bag_info_rc=0
  local summary_rc=0

  if [ "$RECORDING_STARTED" != true ] || [ "$POSTRUN_SUMMARY_WRITTEN" = true ]; then
    return 0
  fi
  POSTRUN_SUMMARY_WRITTEN=true

  ended_at="$(date -Is)"
  ended_at_epoch="$(date +%s)"

  set +e
  ros2 bag info "$BAG_DIR" > "$BAG_INFO_FILE" 2>&1
  bag_info_rc=$?
  python3 - "$TOPICS_FILE" "$BAG_INFO_FILE" "$RUN_SUMMARY_FILE" "$started_at" "$ended_at" "$started_at_epoch" "$ended_at_epoch" "$RECORD_EXIT_CODE" "$BAG_DIR" "$bag_info_rc" <<'PY'
import json
import os
import re
import sys

(
    topics_file,
    bag_info_file,
    summary_file,
    started_at,
    ended_at,
    started_epoch,
    ended_epoch,
    record_exit_code,
    bag_dir,
    bag_info_exit_code,
) = sys.argv[1:]

expected_topics = []
try:
    with open(topics_file, "r", encoding="utf-8") as handle:
        expected_topics = [line.strip() for line in handle if line.strip()]
except FileNotFoundError:
    expected_topics = []

bag_info_text = ""
try:
    with open(bag_info_file, "r", encoding="utf-8") as handle:
        bag_info_text = handle.read()
except FileNotFoundError:
    bag_info_text = ""

topic_counts = {}
for match in re.finditer(r"Topic:\s+(\S+)\s+\|[^\n]*?Count:\s*([0-9]+)", bag_info_text):
    topic_counts[match.group(1)] = int(match.group(2))

if not topic_counts:
    current_topic = None
    for line in bag_info_text.splitlines():
        stripped = line.strip()
        if stripped.startswith("name:"):
            current_topic = stripped.split(":", 1)[1].strip()
            continue
        if stripped.startswith("message_count:") and current_topic:
            try:
                topic_counts[current_topic] = int(stripped.split(":", 1)[1].strip())
            except ValueError:
                pass
            current_topic = None

missing_topics = [topic for topic in expected_topics if topic not in topic_counts]
silent_topics = [topic for topic in expected_topics if topic_counts.get(topic, 0) <= 0]
topics_with_data = [topic for topic in expected_topics if topic_counts.get(topic, 0) > 0]

bag_size_bytes = 0
if os.path.isdir(bag_dir):
    for root, _dirs, files in os.walk(bag_dir):
        for name in files:
            path = os.path.join(root, name)
            try:
                bag_size_bytes += os.path.getsize(path)
            except OSError:
                pass

started_epoch_int = int(started_epoch)
ended_epoch_int = int(ended_epoch)
duration_s = max(0, ended_epoch_int - started_epoch_int)

summary = {
    "started_at": started_at,
    "ended_at": ended_at,
    "duration_s": duration_s,
    "record_exit_code": int(record_exit_code),
    "bag_info_exit_code": int(bag_info_exit_code),
    "coverage_ok": len(missing_topics) == 0 and len(silent_topics) == 0,
    "expected_topic_count": len(expected_topics),
    "topics_with_data_count": len(topics_with_data),
    "topics_missing_count": len(missing_topics),
    "topics_silent_count": len(silent_topics),
    "bag_dir": bag_dir,
    "bag_size_bytes": bag_size_bytes,
    "topics_missing": missing_topics,
    "topics_silent": silent_topics,
    "topics_with_data": topics_with_data,
    "topic_message_counts": {topic: topic_counts[topic] for topic in expected_topics if topic in topic_counts},
}

os.makedirs(os.path.dirname(summary_file), exist_ok=True)
with open(summary_file, "w", encoding="utf-8") as handle:
    json.dump(summary, handle, indent=2, sort_keys=True)
    handle.write("\\n")
PY
  summary_rc=$?
  set -e

  if [ "$summary_rc" -ne 0 ]; then
    echo "[run_record_experiment] Failed to build post-run summary" >&2
    return 0
  fi

  echo "[run_record_experiment] Post-run summary: $RUN_SUMMARY_FILE"
}

trap write_post_run_summary EXIT

echo "[run_record_experiment] Recording profile=$PROFILE mode=$MODE topics=${#TOPICS[@]} storage=$BAG_STORAGE bag_dir=$BAG_DIR"
RECORDING_STARTED=true
set +e
"${BAG_CMD[@]}"
RECORD_EXIT_CODE=$?
set -e
exit "$RECORD_EXIT_CODE"

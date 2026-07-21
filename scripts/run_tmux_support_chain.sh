#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
WORLD="baylands"
SESSION=""
LAYOUT="panes"
TMUX_ATTACH=true
DRY_RUN=false
MODE_SET=false
SUPPORT_FOLLOW_DELAY_S="0"
SUPPORT_OBSERVATION_DELAY_S="0"
ROS_DOMAIN_ID_EFFECTIVE="${ROS_DOMAIN_ID:-3}"
RMW_IMPLEMENTATION_EFFECTIVE="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
UGV_NAMESPACE="a201_0000"
BASE_ARGS=()
SUPPORT_FOLLOW_ARGS=()
SUPPORT_OBSERVATION_ARGS=()
HAZARD_CHAIN_ENABLE=false
AERIAL_SUPPORT_LAYER_ENABLE=false
HAZARD_SYNTHETIC_ENABLE=false
HAZARD_SYNTHETIC_DELAY_S="2"
HAZARD_SYNTHETIC_TOPIC="/coord/support/dji1/aerial_hazards"
HAZARD_SYNTHETIC_SOURCE_UAV="dji1"
HAZARD_SYNTHETIC_CLASS_NAME="hazard"
HAZARD_SYNTHETIC_TRACK_ID="synthetic_hazard_0"
HAZARD_SYNTHETIC_X="-20.0"
HAZARD_SYNTHETIC_Y="230.0"
HAZARD_SYNTHETIC_Z="0.5"
HAZARD_SYNTHETIC_YAW="0.0"
HAZARD_SYNTHETIC_SIZE_X="2.0"
HAZARD_SYNTHETIC_SIZE_Y="2.0"
HAZARD_SYNTHETIC_SIZE_Z="1.0"
HAZARD_SYNTHETIC_CONFIDENCE="0.9"
HAZARD_SYNTHETIC_STATE="1"
HAZARD_SYNTHETIC_START_DELAY_S="0.0"
HAZARD_SYNTHETIC_PUBLISH_RATE_HZ="2.0"
HAZARD_SYNTHETIC_TTL_S="4.0"
HAZARD_SYNTHETIC_ACTIVE_DURATION_S="15.0"
HAZARD_SYNTHETIC_PUBLISH_EMPTY="true"
HAZARD_SYNTHETIC_STAMP_OFFSET_S="0.0"
HAZARD_SYNTHETIC_SUPPORT_QUALITY="1.0"
HAZARD_SYNTHETIC_PROVENANCE="synthetic_task4"

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

SESSION="halmstad-${WORLD}-support-chain"

shell_join() {
  local out=""
  local part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

build_line() {
  local delay_s="$1"
  local ready_cmd="$2"
  shift 2
  local line=""
  printf -v line 'cd %q && ' "$WS_ROOT"
  printf -v line '%sexport ROS_DOMAIN_ID=%q && export RMW_IMPLEMENTATION=%q && ' \
    "$line" "$ROS_DOMAIN_ID_EFFECTIVE" "$RMW_IMPLEMENTATION_EFFECTIVE"
  if [ "$delay_s" != "0" ] && [ "$delay_s" != "0.0" ]; then
    printf -v line '%ssleep %q && ' "$line" "$delay_s"
  fi
  if [ -n "$ready_cmd" ]; then
    printf -v line '%sbash -lc %q && ' "$line" "$ready_cmd"
  fi
  printf -v line '%s%s' "$line" "$(shell_join "$@")"
  printf '%s' "$line"
}

build_wait_for_topic_message_fn() {
  cat <<'EOF'
wait_for_topic_message() {
  local topic="$1"
  local label="$2"
  local timeout_s="${3:-10}"
  echo "[$label] waiting for message on $topic"
  while true; do
    if timeout "${timeout_s}s" ros2 topic echo --once "$topic" >/dev/null 2>&1; then
      echo "[$label] received message on $topic"
      return 0
    fi
    echo "[$label] still waiting for $topic"
    sleep 1
  done
}
EOF
}

build_support_follow_ready_cmd() {
  cat <<EOF
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_EFFECTIVE"
export RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION_EFFECTIVE"
set -u
$(build_wait_for_topic_message_fn)
wait_for_topic_message '/dji0/camera0/camera_info' 'support_follow_ready'
EOF
}

build_support_observation_ready_cmd() {
  cat <<EOF
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_EFFECTIVE"
export RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION_EFFECTIVE"
set -u
$(build_wait_for_topic_message_fn)
wait_for_topic_message '/dji1/pose' 'support_observation_ready'
wait_for_topic_message '/dji2/pose' 'support_observation_ready'
wait_for_topic_message '/dji1/camera0/image_raw' 'support_observation_ready'
wait_for_topic_message '/dji2/camera0/image_raw' 'support_observation_ready'
EOF
}

signal_processes_by_pattern() {
  local pattern="$1"
  local pids=()
  local pid=""
  while IFS= read -r pid; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    pids+=("$pid")
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
  if [ "${#pids[@]}" -eq 0 ]; then
    return 0
  fi
  kill -INT "${pids[@]}" 2>/dev/null || true
  sleep 1
  kill -TERM "${pids[@]}" 2>/dev/null || true
  sleep 1
  kill -KILL "${pids[@]}" 2>/dev/null || true
}

signal_named_nodes() {
  local names_regex="$1"
  signal_processes_by_pattern "__node:=($names_regex)(\\s|$)"
}

prelaunch_support_cleanup() {
  signal_processes_by_pattern 'scripts/run_support_follow_odom\.sh'
  signal_processes_by_pattern 'scripts/run_support_observation\.sh'
  signal_processes_by_pattern 'ros2 run lrs_halmstad synthetic_hazard_publisher'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad support_follow_odom\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad support_observation\.launch\.py'
  signal_named_nodes \
    'support_follow_dji0_pose_to_odom|support_follow_dji1_simulator|support_follow_dji1_odom_controller|support_follow_dji2_simulator|support_follow_dji2_odom_controller|support_dji1_leader_detector|support_dji2_leader_detector|support_detection_mux|support_hazard_fusion|dji0_to_ugv_forwarder|synthetic_hazard_publisher'
}

for arg in "$@"; do
  case "$arg" in
    session:=*)
      SESSION="${arg#session:=}"
      ;;
    layout:=*)
      LAYOUT="${arg#layout:=}"
      BASE_ARGS+=("$arg")
      ;;
    tmux_attach:=*|attach:=*)
      TMUX_ATTACH="${arg#*:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    mode:=*)
      MODE_SET=true
      BASE_ARGS+=("$arg")
      ;;
    uav_start_x:=*)
      BASE_ARGS+=("$arg")
      SUPPORT_FOLLOW_ARGS+=("leader_start_x:=${arg#uav_start_x:=}")
      ;;
    uav_start_y:=*)
      BASE_ARGS+=("$arg")
      SUPPORT_FOLLOW_ARGS+=("leader_start_y:=${arg#uav_start_y:=}")
      ;;
    uav_start_z:=*)
      BASE_ARGS+=("$arg")
      SUPPORT_FOLLOW_ARGS+=("leader_nominal_z:=${arg#uav_start_z:=}")
      ;;
    uav_start_yaw_deg:=*)
      BASE_ARGS+=("$arg")
      SUPPORT_FOLLOW_ARGS+=("leader_start_yaw_deg:=${arg#uav_start_yaw_deg:=}")
      ;;
    height:=*)
      BASE_ARGS+=("$arg")
      SUPPORT_FOLLOW_ARGS+=("leader_nominal_z:=${arg#height:=}")
      ;;
    support_follow_delay_s:=*)
      SUPPORT_FOLLOW_DELAY_S="${arg#support_follow_delay_s:=}"
      ;;
    support_observation_delay_s:=*)
      SUPPORT_OBSERVATION_DELAY_S="${arg#support_observation_delay_s:=}"
      ;;
    hazard_chain_enable:=*)
      HAZARD_CHAIN_ENABLE="${arg#hazard_chain_enable:=}"
      ;;
    aerial_support_layer_enable:=*)
      AERIAL_SUPPORT_LAYER_ENABLE="${arg#aerial_support_layer_enable:=}"
      ;;
    hazard_synthetic_enable:=*)
      HAZARD_SYNTHETIC_ENABLE="${arg#hazard_synthetic_enable:=}"
      ;;
    hazard_synthetic_delay_s:=*)
      HAZARD_SYNTHETIC_DELAY_S="${arg#hazard_synthetic_delay_s:=}"
      ;;
    hazard_synthetic_topic:=*)
      HAZARD_SYNTHETIC_TOPIC="${arg#hazard_synthetic_topic:=}"
      ;;
    hazard_synthetic_source_uav:=*)
      HAZARD_SYNTHETIC_SOURCE_UAV="${arg#hazard_synthetic_source_uav:=}"
      ;;
    hazard_synthetic_class_name:=*)
      HAZARD_SYNTHETIC_CLASS_NAME="${arg#hazard_synthetic_class_name:=}"
      ;;
    hazard_synthetic_track_id:=*)
      HAZARD_SYNTHETIC_TRACK_ID="${arg#hazard_synthetic_track_id:=}"
      ;;
    hazard_synthetic_x:=*)
      HAZARD_SYNTHETIC_X="${arg#hazard_synthetic_x:=}"
      ;;
    hazard_synthetic_y:=*)
      HAZARD_SYNTHETIC_Y="${arg#hazard_synthetic_y:=}"
      ;;
    hazard_synthetic_z:=*)
      HAZARD_SYNTHETIC_Z="${arg#hazard_synthetic_z:=}"
      ;;
    hazard_synthetic_yaw:=*)
      HAZARD_SYNTHETIC_YAW="${arg#hazard_synthetic_yaw:=}"
      ;;
    hazard_synthetic_size_x:=*)
      HAZARD_SYNTHETIC_SIZE_X="${arg#hazard_synthetic_size_x:=}"
      ;;
    hazard_synthetic_size_y:=*)
      HAZARD_SYNTHETIC_SIZE_Y="${arg#hazard_synthetic_size_y:=}"
      ;;
    hazard_synthetic_size_z:=*)
      HAZARD_SYNTHETIC_SIZE_Z="${arg#hazard_synthetic_size_z:=}"
      ;;
    hazard_synthetic_confidence:=*)
      HAZARD_SYNTHETIC_CONFIDENCE="${arg#hazard_synthetic_confidence:=}"
      ;;
    hazard_synthetic_state:=*)
      HAZARD_SYNTHETIC_STATE="${arg#hazard_synthetic_state:=}"
      ;;
    hazard_synthetic_start_delay_s:=*)
      HAZARD_SYNTHETIC_START_DELAY_S="${arg#hazard_synthetic_start_delay_s:=}"
      ;;
    hazard_synthetic_publish_rate_hz:=*)
      HAZARD_SYNTHETIC_PUBLISH_RATE_HZ="${arg#hazard_synthetic_publish_rate_hz:=}"
      ;;
    hazard_synthetic_ttl_s:=*)
      HAZARD_SYNTHETIC_TTL_S="${arg#hazard_synthetic_ttl_s:=}"
      ;;
    hazard_synthetic_active_duration_s:=*)
      HAZARD_SYNTHETIC_ACTIVE_DURATION_S="${arg#hazard_synthetic_active_duration_s:=}"
      ;;
    hazard_synthetic_publish_empty_after_active_duration:=*)
      HAZARD_SYNTHETIC_PUBLISH_EMPTY="${arg#hazard_synthetic_publish_empty_after_active_duration:=}"
      ;;
    hazard_synthetic_stamp_offset_s:=*)
      HAZARD_SYNTHETIC_STAMP_OFFSET_S="${arg#hazard_synthetic_stamp_offset_s:=}"
      ;;
    hazard_synthetic_support_quality:=*)
      HAZARD_SYNTHETIC_SUPPORT_QUALITY="${arg#hazard_synthetic_support_quality:=}"
      ;;
    hazard_synthetic_provenance:=*)
      HAZARD_SYNTHETIC_PROVENANCE="${arg#hazard_synthetic_provenance:=}"
      ;;
    support_with_camera:=*)
      case "${arg#support_with_camera:=}" in
        true|yes|1|on)
          ;;
        false|no|0|off)
          echo "tmux_support_chain always requires support_with_camera:=true." >&2
          exit 2
          ;;
        *)
          echo "Invalid support_with_camera option: ${arg#support_with_camera:=}" >&2
          exit 2
          ;;
      esac
      ;;
    support_vertical_offset_m:=*|support_camera_pitch_offset_deg:=*|support_camera_update_rate:=*|support_bridge_gimbal:=*|leader_nominal_z:=*|leader_start_x:=*|leader_start_y:=*|leader_start_yaw_deg:=*|leader_pose_topic:=*|leader_odom_topic:=*|uav_mode:=*|params_file:=*|dji1_name:=*|dji1_d_target:=*|dji1_forward_offset_m:=*|dji1_start_x:=*|dji1_start_y:=*|dji1_start_z:=*|dji1_start_yaw_deg:=*|dji1_lateral_offset_m:=*|dji2_name:=*|dji2_d_target:=*|dji2_forward_offset_m:=*|dji2_start_x:=*|dji2_start_y:=*|dji2_start_z:=*|dji2_start_yaw_deg:=*|dji2_lateral_offset_m:=*)
      SUPPORT_FOLLOW_ARGS+=("$arg")
      ;;
    support_mux_enable:=*|support_mux_publish_rate_hz:=*|support_mux_source_stale_timeout_s:=*|support_mux_out_detection_topic:=*|support_mux_out_status_topic:=*|support_mux_out_summary_topic:=*|support_mux_relation_source:=*|support_mux_relation_quality:=*|support_mux_relation_note:=*|hazard_fusion_enable:=*|hazard_fusion_dji1_topic:=*|hazard_fusion_dji2_topic:=*|hazard_fusion_out_topic:=*|hazard_fusion_stale_timeout_s:=*|hazard_fusion_max_covariance:=*|hazard_fusion_publish_rate_hz:=*|ugv_forward_enable:=*|ugv_forward_owner:=*|ugv_forward_stage:=*|ugv_forward_out_detection_topic:=*|ugv_forward_out_status_topic:=*|ugv_forward_out_summary_topic:=*|hazard_forward_enable:=*|hazard_in_topic:=*|hazard_out_topic:=*|start_ugv_support_awareness:=*|support_awareness_publish_advisory:=*|ugv_support_awareness_status_topic:=*|ugv_support_path_advisory_topic:=*|support_camera_scan_enable:=*|support_camera_scan_uavs:=*|support_camera_scan_yaw_center_deg:=*|support_camera_scan_yaw_amplitude_deg:=*|support_camera_scan_period_s:=*|support_camera_scan_pan_phase_offsets_deg:=*|support_camera_scan_pitch_deg:=*|support_camera_scan_pitch_amplitude_deg:=*|support_camera_scan_pitch_period_s:=*|support_camera_scan_pitch_phase_offsets_deg:=*|support_camera_scan_rate_hz:=*|support_detector_backend:=*|support_detector_onnx_model:=*|support_yolo_weights:=*|dji1_detector_backend:=*|dji1_detector_onnx_model:=*|dji1_yolo_weights:=*|dji2_detector_backend:=*|dji2_detector_onnx_model:=*|dji2_yolo_weights:=*|camera_name:=*|yolo_weights:=*|target_class_name:=*|target_class_id:=*)
      SUPPORT_OBSERVATION_ARGS+=("$arg")
      ;;
    detector_backend:=*|detector_onnx_model:=*|detector_async_inference:=*|detector_latest_frame_only:=*|detector_stale_detection_threshold_ms:=*|detector_metrics_window_s:=*|detector_benchmark_csv_path:=*|detector_image_qos_depth:=*|detector_image_qos_reliability:=*|yolo_device:=*)
      BASE_ARGS+=("$arg")
      SUPPORT_OBSERVATION_ARGS+=("$arg")
      ;;
    device:=*)
      BASE_ARGS+=("$arg")
      SUPPORT_OBSERVATION_ARGS+=("yolo_device:=${arg#device:=}")
      ;;
    target:=*)
      BASE_ARGS+=("$arg")
      SUPPORT_OBSERVATION_ARGS+=("$arg")
      ;;
    *)
      BASE_ARGS+=("$arg")
      ;;
  esac
done

if [ "$MODE_SET" != true ]; then
  BASE_ARGS=("mode:=yolo" "${BASE_ARGS[@]}")
fi

case "$LAYOUT" in
  windows|panes)
    ;;
  *)
    echo "Invalid layout: $LAYOUT" >&2
    echo "Use layout:=windows or layout:=panes" >&2
    exit 2
    ;;
esac

case "$TMUX_ATTACH" in
  true|false)
    ;;
  *)
    echo "Invalid tmux_attach option: $TMUX_ATTACH" >&2
    echo "Use tmux_attach:=true or tmux_attach:=false" >&2
    exit 2
    ;;
esac

case "$DRY_RUN" in
  true|false)
    ;;
  *)
    echo "Invalid dry_run option: $DRY_RUN" >&2
    echo "Use dry_run:=true or dry_run:=false" >&2
    exit 2
    ;;
esac

validate_boolean() {
  local label="$1"
  local value="$2"
  case "$value" in
    true|false)
      ;;
    *)
      echo "Invalid $label option: $value" >&2
      echo "Use true or false." >&2
      exit 2
      ;;
  esac
}

validate_boolean "hazard_chain_enable" "$HAZARD_CHAIN_ENABLE"
validate_boolean "aerial_support_layer_enable" "$AERIAL_SUPPORT_LAYER_ENABLE"
validate_boolean "hazard_synthetic_enable" "$HAZARD_SYNTHETIC_ENABLE"
validate_boolean "hazard_synthetic_publish_empty_after_active_duration" "$HAZARD_SYNTHETIC_PUBLISH_EMPTY"

if [ "$HAZARD_SYNTHETIC_ENABLE" = true ] && [ "$HAZARD_CHAIN_ENABLE" = false ]; then
  echo "[tmux_support_chain] enabling the typed hazard chain for the synthetic source" >&2
  HAZARD_CHAIN_ENABLE=true
fi
if [ "$HAZARD_CHAIN_ENABLE" = true ]; then
  SUPPORT_OBSERVATION_ARGS+=("hazard_fusion_enable:=true" "hazard_forward_enable:=true")
fi
if [ "$AERIAL_SUPPORT_LAYER_ENABLE" = true ]; then
  BASE_ARGS+=("aerial_support_layer_enable:=true")
fi

BASE_CMD=(
  ./run.sh tmux_1to1 "$WORLD"
  "session:=$SESSION"
  "tmux_attach:=false"
  "layout:=$LAYOUT"
  "follow_wait_topics:=/dji1/pose,/dji2/pose"
  "${BASE_ARGS[@]}"
)
SUPPORT_FOLLOW_CMD=(./run.sh support_follow_odom "$WORLD" "support_with_camera:=true" "${SUPPORT_FOLLOW_ARGS[@]}")
SUPPORT_OBSERVATION_CMD=(./run.sh support_observation "$WORLD" "${SUPPORT_OBSERVATION_ARGS[@]}")
SYNTHETIC_HAZARD_ROS_COMMAND=(
  ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args
  -p use_sim_time:=true
  -p "topic:=$HAZARD_SYNTHETIC_TOPIC"
  -p "source_uav:=$HAZARD_SYNTHETIC_SOURCE_UAV"
  -p "class_name:=$HAZARD_SYNTHETIC_CLASS_NAME"
  -p "stable_track_id:=$HAZARD_SYNTHETIC_TRACK_ID"
  -p "center_x:=$HAZARD_SYNTHETIC_X"
  -p "center_y:=$HAZARD_SYNTHETIC_Y"
  -p "center_z:=$HAZARD_SYNTHETIC_Z"
  -p "yaw:=$HAZARD_SYNTHETIC_YAW"
  -p "dimension_x:=$HAZARD_SYNTHETIC_SIZE_X"
  -p "dimension_y:=$HAZARD_SYNTHETIC_SIZE_Y"
  -p "dimension_z:=$HAZARD_SYNTHETIC_SIZE_Z"
  -p "confidence:=$HAZARD_SYNTHETIC_CONFIDENCE"
  -p "state:=$HAZARD_SYNTHETIC_STATE"
  -p "start_delay_s:=$HAZARD_SYNTHETIC_START_DELAY_S"
  -p "publish_rate_hz:=$HAZARD_SYNTHETIC_PUBLISH_RATE_HZ"
  -p "ttl_s:=$HAZARD_SYNTHETIC_TTL_S"
  -p "active_duration_s:=$HAZARD_SYNTHETIC_ACTIVE_DURATION_S"
  -p "publish_empty_after_active_duration:=$HAZARD_SYNTHETIC_PUBLISH_EMPTY"
  -p "stamp_offset_s:=$HAZARD_SYNTHETIC_STAMP_OFFSET_S"
  -p "support_quality:=$HAZARD_SYNTHETIC_SUPPORT_QUALITY"
  -p "provenance:=$HAZARD_SYNTHETIC_PROVENANCE"
)
SYNTHETIC_HAZARD_COMMAND=(
  bash -lc "source /opt/ros/jazzy/setup.bash && source $(printf '%q' "$WS_ROOT/install/setup.bash") && exec $(shell_join "${SYNTHETIC_HAZARD_ROS_COMMAND[@]}")"
)

SUPPORT_FOLLOW_READY_CMD="$(build_support_follow_ready_cmd)"
SUPPORT_OBSERVATION_READY_CMD="$(build_support_observation_ready_cmd)"
SUPPORT_FOLLOW_LINE="$(build_line "$SUPPORT_FOLLOW_DELAY_S" "$SUPPORT_FOLLOW_READY_CMD" "${SUPPORT_FOLLOW_CMD[@]}")"
SUPPORT_OBSERVATION_LINE="$(build_line "$SUPPORT_OBSERVATION_DELAY_S" "$SUPPORT_OBSERVATION_READY_CMD" "${SUPPORT_OBSERVATION_CMD[@]}")"
SYNTHETIC_HAZARD_LINE="$(build_line "$HAZARD_SYNTHETIC_DELAY_S" "" "${SYNTHETIC_HAZARD_COMMAND[@]}")"

if [ "$DRY_RUN" = true ]; then
  echo "Session: $SESSION"
  echo "Layout: $LAYOUT"
  echo "Attach: $TMUX_ATTACH"
  echo "[base]"
  "${BASE_CMD[@]}" "dry_run:=true"
  echo "[support_follow] $SUPPORT_FOLLOW_LINE"
  echo "[support_observation] $SUPPORT_OBSERVATION_LINE"
  if [ "$HAZARD_SYNTHETIC_ENABLE" = true ]; then
    echo "[synthetic_hazard] $SYNTHETIC_HAZARD_LINE"
  fi
  exit 0
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  echo "Attach with: tmux attach -t $SESSION" >&2
  exit 1
fi

prelaunch_support_cleanup

"${BASE_CMD[@]}"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Base tmux session did not start: $SESSION" >&2
  exit 1
fi

if tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -Fxq support; then
  echo "Support window already exists in tmux session: $SESSION" >&2
  exit 1
fi

tmux new-window -d -t "$SESSION" -n support
support_follow_pane="$(tmux display-message -p -t "$SESSION:support.0" '#{pane_id}')"
support_observation_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$support_follow_pane" -v -l 50%)"

tmux select-pane -t "$support_follow_pane" -T support_follow
tmux select-pane -t "$support_observation_pane" -T support_observation

tmux send-keys -t "$support_follow_pane" "$SUPPORT_FOLLOW_LINE" C-m
tmux send-keys -t "$support_observation_pane" "$SUPPORT_OBSERVATION_LINE" C-m

if [ "$HAZARD_SYNTHETIC_ENABLE" = true ]; then
  synthetic_hazard_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$support_observation_pane" -v -l 35%)"
  tmux select-pane -t "$synthetic_hazard_pane" -T synthetic_hazard
  tmux send-keys -t "$synthetic_hazard_pane" "$SYNTHETIC_HAZARD_LINE" C-m
fi

tmux select-window -t "$SESSION:support"
tmux select-pane -t "$support_follow_pane"

if [ "$TMUX_ATTACH" = true ]; then
  if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "$SESSION"
    exit 0
  fi
  exec tmux attach -t "$SESSION"
fi

echo "Started tmux session: $SESSION"
echo "Attach with: tmux attach -t $SESSION"

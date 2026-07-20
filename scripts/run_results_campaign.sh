#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

CONDITION=""
RUNS=1
START_INDEX=1
DURATION_S=120
WARMUP_S=15
WORLD="baylands"
OUT_ROOT="bags/results"
WAYPOINT="parkinglot_west_0"
ROUTE="parkinglot_west"
ROUTE_SCHEDULE=""
GUI="false"
YOLO_WEIGHTS="${LRS_BAYLANDS_YOLO_WEIGHTS:-}"
DETECTOR_BACKEND="${LRS_BAYLANDS_DETECTOR_BACKEND:-ultralytics}"
DETECTOR_CONF_THRESHOLD="0.05"
DETECTOR_IOU_THRESHOLD="0.5"
TRACKER_ENABLE="true"
EXTERNAL_DETECTION_NODE=""
SUPPORT_ONNX_MODEL="${LRS_BAYLANDS_SUPPORT_ONNX_MODEL:-}"
SUPPORT_YOLO_WEIGHTS="${LRS_BAYLANDS_SUPPORT_YOLO_WEIGHTS:-}"
SUPPORT_BACKEND="${LRS_BAYLANDS_SUPPORT_BACKEND:-}"
OMNET_PROJECT="${LRS_OMNET_PROJECT_ROOT:-$HOME/omnet_workspace/UAV_UGV}"
OMNET_NETWORK="wifi"
VISUAL_REACQUIRE_ASSIST="false"
VISUAL_REACQUIRE_STALE_TIMEOUT_S="2.0"
VISUAL_REACQUIRE_RETURN_FRESH_S="1.0"
VISUAL_REACQUIRE_SOURCE="amcl_odom"
RECORD_START_TIMEOUT_S=360
EXPECTED_BAYLANDS_WEIGHTS="$WS_ROOT/models/obb/mymodels/baylands-leader-v0.pt"
EXPECTED_BAYLANDS_ONNX="$WS_ROOT/models/obb/mymodels/baylands-leader-v0.onnx"

ACTIVE_STOP_CMD=()
YOLO_WEIGHTS_ABS=""
SUPPORT_ONNX_ABS=""
SUPPORT_WEIGHTS_ABS=""
SUPPORT_BACKEND_EFFECTIVE=""
DETECTOR_BACKEND_EFFECTIVE=""
DETECTOR_MODEL_DOMAIN=""
DETECTOR_MODEL_DOMAIN_NOTE=""
MODEL_PATH=""
MODEL_FILENAME=""
SUPPORT_MODEL_PATH=""
YOLO_CONTROL_MODE=""
VISUAL_FOLLOW_LOGIC=""
RECORD_PROFILE=""
GIT_BRANCH="$(git -C "$WS_ROOT" branch --show-current 2>/dev/null || true)"
GIT_COMMIT="$(git -C "$WS_ROOT" rev-parse --short HEAD 2>/dev/null || true)"
if [ -n "$(git -C "$WS_ROOT" status --porcelain=v1 2>/dev/null || true)" ]; then
  GIT_DIRTY="true"
else
  GIT_DIRTY="false"
fi

usage() {
  cat <<EOF
Usage: $0 --condition C1|C2|C3|C4|C5 --runs N --duration 90|120 --warmup 10|15 [options]

Required campaign rules:
  --world baylands only
  same --waypoint and --route are used for every run

Options:
  --out PATH              Output root, default bags/results
  --start-index N         First run number, default 1. Useful for resuming a batch without overwriting rXX folders.
  --waypoint NAME         Baylands spawn waypoint, default parkinglot_west_0
  --route NAME            Baylands route/nav2_goals shorthand, default parkinglot_west
  --route-schedule LIST   Comma list used per run, e.g. rotundan,road_to_west,road_to_spawn,spawn,parkinglot_west,parkinglot_east,road_to_east,strip
                          Entries may also be route:waypoint. The schedule repeats when --runs exceeds its length.
  --gui true|false        Gazebo GUI, default false
  --weights PATH          YOLO .pt path for C2/C3/C5
  --detector-backend NAME ultralytics|onnx_cpu|onnx_directml for C2/C3/C5, default ultralytics
  --tracker true|false    Use leader_tracker when true, leader_detector when false, default true
  --external-detection-node detector|tracker
                          Explicit perception node for C2/C3/C5; normally inferred from --tracker
  --support-onnx PATH     Support detector ONNX path for C4
  --support-weights PATH  Support detector .pt path for C4 ultralytics
  --support-backend NAME  ultralytics|onnx_cpu|onnx_directml for C4
  --omnet-project PATH    External OMNeT project for C5
  --omnet-network NAME    wifi|5g|lora for C5, default wifi
  --visual-reacquire-assist true|false
                          Opt-in odometry-assisted visual reacquisition for C2/C3/C5, default false
  --visual-reacquire-stale-timeout S
                          Stale visual estimate timeout before assist, default 2.0
  --visual-reacquire-return-fresh S
                          Fresh visual estimate duration before returning, default 1.0
EOF
}

route_default_waypoint() {
  printf '%s_0\n' "$1"
}

select_route_for_run() {
  local run_index="$1"
  local entries=()
  local entry=""
  local selected=""
  local idx=0

  if [ -z "$ROUTE_SCHEDULE" ]; then
    return 0
  fi

  IFS=',' read -r -a entries <<< "$ROUTE_SCHEDULE"
  if [ "${#entries[@]}" -eq 0 ]; then
    echo "Empty --route-schedule." >&2
    exit 2
  fi

  idx=$(( (run_index - 1) % ${#entries[@]} ))
  selected="${entries[$idx]}"
  selected="${selected//[[:space:]]/}"
  if [ -z "$selected" ]; then
    echo "Invalid empty route schedule entry in: $ROUTE_SCHEDULE" >&2
    exit 2
  fi

  if [[ "$selected" == *:* ]]; then
    ROUTE="${selected%%:*}"
    WAYPOINT="${selected#*:}"
  else
    ROUTE="$selected"
    WAYPOINT="$(route_default_waypoint "$ROUTE")"
  fi

  case "$ROUTE" in
    rotundan|road_to_west|road_to_spawn|spawn|parkinglot_west|parkinglot_east|road_to_east|strip)
      ;;
    *)
      echo "Unsupported Baylands Results route in --route-schedule: $ROUTE" >&2
      exit 2
      ;;
  esac
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

shell_join() {
  local out=""
  local part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

condition_dir_name() {
  case "$1" in
    C1) printf 'C1_odom\n' ;;
    C2) printf 'C2_direct\n' ;;
    C3) printf 'C3_bridge\n' ;;
    C4) printf 'C4_support\n' ;;
    C5) printf 'C5_omnet\n' ;;
    *) return 1 ;;
  esac
}

resolve_existing_path() {
  local value="$1"
  if [ -z "$value" ]; then
    return 1
  fi
  if [[ "$value" = /* ]]; then
    [ -f "$value" ] && readlink -f "$value"
    return $?
  fi
  if [ -f "$value" ]; then
    readlink -f "$value"
    return 0
  fi
  if [ -f "$WS_ROOT/$value" ]; then
    printf '%s/%s\n' "$WS_ROOT" "$value"
    return 0
  fi
  if [ -f "$WS_ROOT/models/$value" ]; then
    printf '%s/models/%s\n' "$WS_ROOT" "$value"
    return 0
  fi
  return 1
}

default_yolo_weights() {
  local candidate=""
  for candidate in \
    "$YOLO_WEIGHTS" \
    "$EXPECTED_BAYLANDS_WEIGHTS" \
    "$WS_ROOT/models/obb/mymodels/baylands-leader-v0.pt"; do
    if resolved="$(resolve_existing_path "$candidate" 2>/dev/null)"; then
      printf '%s\n' "$resolved"
      return 0
    fi
  done
  return 1
}

default_support_onnx() {
  local candidate=""
  for candidate in \
    "$SUPPORT_ONNX_MODEL" \
    "$EXPECTED_BAYLANDS_ONNX" \
    "$WS_ROOT/models/obb/mymodels/baylands-leader-v0.onnx"; do
    if resolved="$(resolve_existing_path "$candidate" 2>/dev/null)"; then
      printf '%s\n' "$resolved"
      return 0
    fi
  done
  return 1
}

default_support_weights() {
  local candidate=""
  for candidate in \
    "$SUPPORT_YOLO_WEIGHTS" \
    "$EXPECTED_BAYLANDS_WEIGHTS" \
    "$WS_ROOT/models/obb/mymodels/baylands-leader-v0.pt"; do
    if resolved="$(resolve_existing_path "$candidate" 2>/dev/null)"; then
      printf '%s\n' "$resolved"
      return 0
    fi
  done
  return 1
}

print_missing_baylands_models() {
  local kind="$1"
  echo "Missing Baylands detector model for $kind." >&2
  echo "Expected Baylands .pt:   $EXPECTED_BAYLANDS_WEIGHTS" >&2
  echo "Expected Baylands .onnx: $EXPECTED_BAYLANDS_ONNX" >&2
  echo "You may explicitly override with --weights, --support-weights, or --support-onnx only when that model choice is intended for this Baylands run." >&2
}

infer_model_domain() {
  local path="$1"
  local base
  base="$(basename "$path" | tr '[:upper:]' '[:lower:]')"
  case "$base" in
    *baylands*) printf 'baylands\n' ;;
    *warehouse*) printf 'warehouse\n' ;;
    *) printf 'unknown\n' ;;
  esac
}

domain_note_for() {
  case "$1" in
    baylands)
      printf 'model_name_indicates_baylands_training_domain\n'
      ;;
    warehouse)
      printf 'model_name_indicates_warehouse_training_domain_used_in_baylands_world; valid_for_pipeline_control_smoke_results_not_baylands_detector_generalisation_claims\n'
      ;;
    *)
      printf 'model_training_domain_unknown_from_filename\n'
      ;;
  esac
}

write_run_json() {
  local path="$1"
  local condition="$2"
  local run_id="$3"
  local run_dir="$4"
  local status="$5"
  local reason="$6"
  shift 6
  local command_text
  command_text="$(shell_join "$@")"
  mkdir -p "$(dirname "$path")"
  {
    printf '{\n'
    printf '  "condition": "%s",\n' "$(json_escape "$condition")"
    printf '  "run_id": "%s",\n' "$(json_escape "$run_id")"
    printf '  "world": "%s",\n' "$(json_escape "$WORLD")"
    printf '  "waypoint": "%s",\n' "$(json_escape "$WAYPOINT")"
    printf '  "route": "%s",\n' "$(json_escape "$ROUTE")"
    printf '  "nav2_goals": "%s",\n' "$(json_escape "$ROUTE")"
    printf '  "route_schedule": "%s",\n' "$(json_escape "$ROUTE_SCHEDULE")"
    printf '  "duration_s": %.3f,\n' "$DURATION_S"
    printf '  "warmup_s": %.3f,\n' "$WARMUP_S"
    printf '  "run_dir": "%s",\n' "$(json_escape "$run_dir")"
    printf '  "detector_backend": "%s",\n' "$(json_escape "$DETECTOR_BACKEND_EFFECTIVE")"
    printf '  "tracker": "%s",\n' "$(json_escape "$TRACKER_ENABLE")"
    printf '  "external_detection_node": "%s",\n' "$(json_escape "$EXTERNAL_DETECTION_NODE")"
    printf '  "model_path": "%s",\n' "$(json_escape "$MODEL_PATH")"
    printf '  "model_filename": "%s",\n' "$(json_escape "$MODEL_FILENAME")"
    printf '  "model_domain": "%s",\n' "$(json_escape "$DETECTOR_MODEL_DOMAIN")"
    printf '  "yolo_control_mode": "%s",\n' "$(json_escape "$YOLO_CONTROL_MODE")"
    printf '  "visual_follow_logic": "%s",\n' "$(json_escape "$VISUAL_FOLLOW_LOGIC")"
    printf '  "support_backend": "%s",\n' "$(json_escape "$SUPPORT_BACKEND_EFFECTIVE")"
    printf '  "support_model_path": "%s",\n' "$(json_escape "$SUPPORT_MODEL_PATH")"
    printf '  "record_profile": "%s",\n' "$(json_escape "$RECORD_PROFILE")"
    printf '  "visual_reacquire_assist_enable": "%s",\n' "$(json_escape "$VISUAL_REACQUIRE_ASSIST")"
    printf '  "visual_reacquire_stale_timeout_s": %.3f,\n' "$VISUAL_REACQUIRE_STALE_TIMEOUT_S"
    printf '  "visual_reacquire_return_fresh_s": %.3f,\n' "$VISUAL_REACQUIRE_RETURN_FRESH_S"
    printf '  "visual_reacquire_source": "%s",\n' "$(json_escape "$VISUAL_REACQUIRE_SOURCE")"
    printf '  "yolo_weights_path": "%s",\n' "$(json_escape "$YOLO_WEIGHTS_ABS")"
    printf '  "support_detector_backend": "%s",\n' "$(json_escape "$SUPPORT_BACKEND_EFFECTIVE")"
    printf '  "support_onnx_model_path": "%s",\n' "$(json_escape "$SUPPORT_ONNX_ABS")"
    printf '  "support_yolo_weights_path": "%s",\n' "$(json_escape "$SUPPORT_WEIGHTS_ABS")"
    printf '  "detector_model_training_domain": "%s",\n' "$(json_escape "$DETECTOR_MODEL_DOMAIN")"
    printf '  "detector_model_domain_note": "%s",\n' "$(json_escape "$DETECTOR_MODEL_DOMAIN_NOTE")"
    printf '  "completion_status": "%s",\n' "$(json_escape "$status")"
    printf '  "failure_reason": "%s",\n' "$(json_escape "$reason")"
    printf '  "git_branch": "%s",\n' "$(json_escape "$GIT_BRANCH")"
    printf '  "git_commit": "%s",\n' "$(json_escape "$GIT_COMMIT")"
    printf '  "git_dirty": %s,\n' "$GIT_DIRTY"
    printf '  "timestamp": "%s",\n' "$(date -Is)"
    printf '  "updated_at": "%s",\n' "$(date -Is)"
    printf '  "command": "%s"\n' "$(json_escape "$command_text")"
    printf '}\n'
  } > "$path"
}

wait_for_recording() {
  local run_dir="$1"
  local deadline=$((SECONDS + RECORD_START_TIMEOUT_S))
  while [ "$SECONDS" -le "$deadline" ]; do
    if [ -f "$run_dir/topics.txt" ] || [ -d "$run_dir/bag" ]; then
      return 0
    fi
    sleep 2
  done
  return 1
}

cleanup_active() {
  if [ "${#ACTIVE_STOP_CMD[@]}" -gt 0 ]; then
    (cd "$WS_ROOT" && "${ACTIVE_STOP_CMD[@]}") || true
    ACTIVE_STOP_CMD=()
  fi
}

trap cleanup_active INT TERM EXIT

while [ "$#" -gt 0 ]; do
  case "$1" in
    --condition)
      CONDITION="${2:-}"
      shift 2
      ;;
    --runs)
      RUNS="${2:-}"
      shift 2
      ;;
    --start-index)
      START_INDEX="${2:-}"
      shift 2
      ;;
    --duration)
      DURATION_S="${2:-}"
      shift 2
      ;;
    --warmup)
      WARMUP_S="${2:-}"
      shift 2
      ;;
    --world)
      WORLD="${2:-}"
      shift 2
      ;;
    --out)
      OUT_ROOT="${2:-}"
      shift 2
      ;;
    --waypoint)
      WAYPOINT="${2:-}"
      shift 2
      ;;
    --route)
      ROUTE="${2:-}"
      shift 2
      ;;
    --route-schedule)
      ROUTE_SCHEDULE="${2:-}"
      shift 2
      ;;
    --gui)
      GUI="${2:-}"
      shift 2
      ;;
    --conf-threshold)
      DETECTOR_CONF_THRESHOLD="${2:-}"
      shift 2
      ;;
    --weights)
      YOLO_WEIGHTS="${2:-}"
      shift 2
      ;;
    --iou-threshold)
      DETECTOR_IOU_THRESHOLD="${2:-}"
      shift 2
      ;;
    --detector-backend)
      DETECTOR_BACKEND="${2:-}"
      shift 2
      ;;
    --tracker)
      TRACKER_ENABLE="${2:-}"
      shift 2
      ;;
    --external-detection-node)
      EXTERNAL_DETECTION_NODE="${2:-}"
      shift 2
      ;;
    --support-onnx)
      SUPPORT_ONNX_MODEL="${2:-}"
      shift 2
      ;;
    --support-weights)
      SUPPORT_YOLO_WEIGHTS="${2:-}"
      shift 2
      ;;
    --support-backend)
      SUPPORT_BACKEND="${2:-}"
      shift 2
      ;;
    --omnet-project)
      OMNET_PROJECT="${2:-}"
      shift 2
      ;;
    --omnet-network)
      OMNET_NETWORK="${2:-}"
      shift 2
      ;;
    --visual-reacquire-assist)
      VISUAL_REACQUIRE_ASSIST="${2:-}"
      shift 2
      ;;
    --visual-reacquire-stale-timeout)
      VISUAL_REACQUIRE_STALE_TIMEOUT_S="${2:-}"
      shift 2
      ;;
    --visual-reacquire-return-fresh)
      VISUAL_REACQUIRE_RETURN_FRESH_S="${2:-}"
      shift 2
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! condition_name="$(condition_dir_name "$CONDITION")"; then
  echo "Invalid or missing --condition. Use C1, C2, C3, C4, or C5." >&2
  exit 2
fi
case "$RUNS" in
  ''|*[!0-9]*) echo "Invalid --runs value: $RUNS" >&2; exit 2 ;;
esac
case "$START_INDEX" in
  ''|*[!0-9]*) echo "Invalid --start-index value: $START_INDEX" >&2; exit 2 ;;
esac
if [ "$RUNS" -lt 1 ]; then
  echo "--runs must be at least 1." >&2
  exit 2
fi
if [ "$START_INDEX" -lt 1 ]; then
  echo "--start-index must be at least 1." >&2
  exit 2
fi
if [ "$WORLD" != "baylands" ]; then
  echo "Results campaign is Baylands-only. Refusing world=$WORLD." >&2
  exit 2
fi
case "$GUI" in
  true|false) ;;
  *) echo "Invalid --gui value: $GUI" >&2; exit 2 ;;
esac
case "$OMNET_NETWORK" in
  wifi|5g|lora) ;;
  *) echo "Invalid --omnet-network value: $OMNET_NETWORK" >&2; exit 2 ;;
esac
case "$VISUAL_REACQUIRE_ASSIST" in
  true|false) ;;
  *) echo "Invalid --visual-reacquire-assist value: $VISUAL_REACQUIRE_ASSIST" >&2; exit 2 ;;
esac
case "$SUPPORT_BACKEND" in
  ""|ultralytics|onnx_cpu|onnx_directml) ;;
  *) echo "Invalid --support-backend value: $SUPPORT_BACKEND" >&2; exit 2 ;;
esac
case "$DETECTOR_BACKEND" in
  ultralytics|onnx_cpu|onnx_directml) ;;
  *) echo "Invalid --detector-backend value: $DETECTOR_BACKEND" >&2; exit 2 ;;
esac
case "$TRACKER_ENABLE" in
  true|false) ;;
  *) echo "Invalid --tracker value: $TRACKER_ENABLE" >&2; exit 2 ;;
esac
case "$EXTERNAL_DETECTION_NODE" in
  ""|detector|tracker) ;;
  *) echo "Invalid --external-detection-node value: $EXTERNAL_DETECTION_NODE" >&2; exit 2 ;;
esac
if [ -z "$EXTERNAL_DETECTION_NODE" ]; then
  if [ "$TRACKER_ENABLE" = true ]; then
    EXTERNAL_DETECTION_NODE="tracker"
  else
    EXTERNAL_DETECTION_NODE="detector"
  fi
fi

OUT_ROOT_ABS="$OUT_ROOT"
if [[ "$OUT_ROOT_ABS" != /* ]]; then
  OUT_ROOT_ABS="$WS_ROOT/$OUT_ROOT_ABS"
fi
CONDITION_ROOT="$OUT_ROOT_ABS/$condition_name"
mkdir -p "$CONDITION_ROOT"

if [[ "$CONDITION" == C2 || "$CONDITION" == C3 || "$CONDITION" == C5 ]]; then
  if ! YOLO_WEIGHTS_ABS="$(default_yolo_weights)"; then
    print_missing_baylands_models "$CONDITION visual detector"
    exit 1
  fi
  DETECTOR_MODEL_DOMAIN="$(infer_model_domain "$YOLO_WEIGHTS_ABS")"
  DETECTOR_MODEL_DOMAIN_NOTE="$(domain_note_for "$DETECTOR_MODEL_DOMAIN")"
  DETECTOR_BACKEND_EFFECTIVE="$DETECTOR_BACKEND"
  MODEL_PATH="$YOLO_WEIGHTS_ABS"
  MODEL_FILENAME="$(basename "$MODEL_PATH")"
fi
if [ "$CONDITION" = C4 ]; then
  if [ -z "$SUPPORT_BACKEND" ]; then
    if [ -n "$SUPPORT_YOLO_WEIGHTS" ] && [ -z "$SUPPORT_ONNX_MODEL" ]; then
      SUPPORT_BACKEND_EFFECTIVE="ultralytics"
    else
      SUPPORT_BACKEND_EFFECTIVE="onnx_cpu"
    fi
  else
    SUPPORT_BACKEND_EFFECTIVE="$SUPPORT_BACKEND"
  fi

  case "$SUPPORT_BACKEND_EFFECTIVE" in
    ultralytics)
      if ! SUPPORT_WEIGHTS_ABS="$(default_support_weights)"; then
        print_missing_baylands_models "C4 ultralytics support detector"
        exit 1
      fi
      DETECTOR_MODEL_DOMAIN="$(infer_model_domain "$SUPPORT_WEIGHTS_ABS")"
      SUPPORT_MODEL_PATH="$SUPPORT_WEIGHTS_ABS"
      ;;
    onnx_cpu|onnx_directml)
      if ! SUPPORT_ONNX_ABS="$(default_support_onnx)"; then
        print_missing_baylands_models "C4 $SUPPORT_BACKEND_EFFECTIVE support detector"
        exit 1
      fi
      DETECTOR_MODEL_DOMAIN="$(infer_model_domain "$SUPPORT_ONNX_ABS")"
      SUPPORT_MODEL_PATH="$SUPPORT_ONNX_ABS"
      ;;
  esac
  DETECTOR_MODEL_DOMAIN_NOTE="$(domain_note_for "$DETECTOR_MODEL_DOMAIN")"
  DETECTOR_BACKEND_EFFECTIVE="$SUPPORT_BACKEND_EFFECTIVE"
  MODEL_PATH="$SUPPORT_MODEL_PATH"
  MODEL_FILENAME="$(basename "$MODEL_PATH")"
else
  SUPPORT_BACKEND_EFFECTIVE=""
  if [ "$CONDITION" = C1 ]; then
    DETECTOR_MODEL_DOMAIN="none"
    DETECTOR_MODEL_DOMAIN_NOTE="no_detector_model_used"
    DETECTOR_BACKEND_EFFECTIVE=""
    MODEL_PATH=""
    MODEL_FILENAME=""
  fi
fi
if [ "$CONDITION" = C5 ]; then
  if [ ! -x "$OMNET_PROJECT/UAV_UGV" ] || [ ! -f "$OMNET_PROJECT/omnetpp.ini" ]; then
    echo "OMNeT project is not installed/verified at $OMNET_PROJECT. C5 is optional; ask Ruben to confirm the external setup." >&2
    exit 1
  fi
fi

last_run_index=$((START_INDEX + RUNS - 1))
for run_index in $(seq "$START_INDEX" "$last_run_index"); do
  select_route_for_run "$run_index"
  run_id="$(printf 'r%02d' "$run_index")"
  run_dir="$CONDITION_ROOT/$run_id"
  metadata_path="$run_dir/campaign_run.json"
  session="results-${condition_name}-${run_id}"

  if [ -e "$run_dir" ] && { [ -d "$run_dir/bag" ] || [ -f "$run_dir/campaign_run.json" ]; }; then
    echo "Run directory already exists: $run_dir" >&2
    echo "Move it aside before rerunning to avoid mixing Results evidence." >&2
    exit 1
  fi
  mkdir -p "$run_dir"

  COMMON_ARGS=(
    "$WORLD"
    "waypoint:=$WAYPOINT"
    "nav2_goals:=$ROUTE"
    "gui:=$GUI"
    "delay_s:=60"
    "gazebo_ready_settle_s:=60"
    "record:=true"
    "record_out:=$run_dir"
    "record_tag:=${condition_name}_${run_id}"
    "session:=$session"
    "tmux_attach:=false"
    "publish_pose_cmd_topics:=true"
    "publish_camera_debug_topics:=true"
  )

  START_CMD=()
  STOP_CMD=()
  case "$CONDITION" in
    C1)
      RECORD_PROFILE="default"
      YOLO_CONTROL_MODE=""
      VISUAL_FOLLOW_LOGIC=""
      START_CMD=(./run.sh tmux_1to1 "${COMMON_ARGS[@]}" mode:=follow record_profile:=default publish_follow_debug_topics:=true)
      STOP_CMD=(./stop.sh tmux_1to1 "$WORLD" "session:=$session")
      ;;
    C2)
      RECORD_PROFILE="default"
      YOLO_CONTROL_MODE="follow_uav_estimate"
      VISUAL_FOLLOW_LOGIC=""
      START_CMD=(./run.sh tmux_1to1 "${COMMON_ARGS[@]}" mode:=yolo record_profile:=default publish_follow_debug_topics:=true yolo_control_mode:=follow_uav_estimate "detector_backend:=$DETECTOR_BACKEND" "weights:=$YOLO_WEIGHTS_ABS" "detector_benchmark_csv_path:=$run_dir/detector.csv" "tracker:=$TRACKER_ENABLE" "external_detection_node:=$EXTERNAL_DETECTION_NODE" "visual_reacquire_assist_enable:=$VISUAL_REACQUIRE_ASSIST" "visual_reacquire_stale_timeout_s:=$VISUAL_REACQUIRE_STALE_TIMEOUT_S" "visual_reacquire_return_fresh_s:=$VISUAL_REACQUIRE_RETURN_FRESH_S" "visual_reacquire_source:=$VISUAL_REACQUIRE_SOURCE" "detector_conf_threshold:=$DETECTOR_CONF_THRESHOLD" "detector_iou_threshold:=$DETECTOR_IOU_THRESHOLD")
      STOP_CMD=(./stop.sh tmux_1to1 "$WORLD" "session:=$session")
      ;;
    C3)
      RECORD_PROFILE="default"
      YOLO_CONTROL_MODE="visual_bridge"
      VISUAL_FOLLOW_LOGIC="follow_core"
      START_CMD=(./run.sh tmux_1to1 "${COMMON_ARGS[@]}" mode:=yolo record_profile:=default yolo_control_mode:=visual_bridge visual_follow_logic:=follow_core "detector_backend:=$DETECTOR_BACKEND" "weights:=$YOLO_WEIGHTS_ABS" "detector_benchmark_csv_path:=$run_dir/detector.csv" "tracker:=$TRACKER_ENABLE" "external_detection_node:=$EXTERNAL_DETECTION_NODE" "visual_reacquire_assist_enable:=$VISUAL_REACQUIRE_ASSIST" "visual_reacquire_stale_timeout_s:=$VISUAL_REACQUIRE_STALE_TIMEOUT_S" "visual_reacquire_return_fresh_s:=$VISUAL_REACQUIRE_RETURN_FRESH_S" "visual_reacquire_source:=$VISUAL_REACQUIRE_SOURCE" "detector_conf_threshold:=$DETECTOR_CONF_THRESHOLD" "detector_iou_threshold:=$DETECTOR_IOU_THRESHOLD")
      STOP_CMD=(./stop.sh tmux_1to1 "$WORLD" "session:=$session")
      ;;
    C4)
      RECORD_PROFILE="support"
      YOLO_CONTROL_MODE=""
      VISUAL_FOLLOW_LOGIC=""
      START_CMD=(./run.sh tmux_support_chain "${COMMON_ARGS[@]}" mode:=follow record_profile:=support support_mux_relation_source:=odom support_mux_source_stale_timeout_s:=4.0 support_camera_scan_enable:=true support_bridge_gimbal:=true "support_detector_backend:=$SUPPORT_BACKEND_EFFECTIVE")
      if [ "$SUPPORT_BACKEND_EFFECTIVE" = "ultralytics" ]; then
        START_CMD+=("support_yolo_weights:=$SUPPORT_WEIGHTS_ABS")
      else
        START_CMD+=("support_detector_onnx_model:=$SUPPORT_ONNX_ABS")
      fi
      STOP_CMD=(./stop.sh tmux_support_chain "$WORLD" "session:=$session")
      ;;
    C5)
      RECORD_PROFILE="omnet"
      YOLO_CONTROL_MODE="follow_uav_estimate"
      VISUAL_FOLLOW_LOGIC=""
      START_CMD=(./run.sh tmux_1to1 "${COMMON_ARGS[@]}" mode:=yolo record_profile:=omnet yolo_control_mode:=follow_uav_estimate "detector_backend:=$DETECTOR_BACKEND" "weights:=$YOLO_WEIGHTS_ABS" "detector_benchmark_csv_path:=$run_dir/detector.csv" "tracker:=$TRACKER_ENABLE" "external_detection_node:=$EXTERNAL_DETECTION_NODE" "visual_reacquire_assist_enable:=$VISUAL_REACQUIRE_ASSIST" "visual_reacquire_stale_timeout_s:=$VISUAL_REACQUIRE_STALE_TIMEOUT_S" "visual_reacquire_return_fresh_s:=$VISUAL_REACQUIRE_RETURN_FRESH_S" "visual_reacquire_source:=$VISUAL_REACQUIRE_SOURCE" omnet:=true "omnet_network:=$OMNET_NETWORK" omnet_ui:=cmdenv "omnet_project:=$OMNET_PROJECT" "omnet_result_dir:=$run_dir/omnet" "detector_conf_threshold:=$DETECTOR_CONF_THRESHOLD" "detector_iou_threshold:=$DETECTOR_IOU_THRESHOLD")
      STOP_CMD=(./stop.sh tmux_1to1 "$WORLD" "session:=$session")
      ;;
  esac

  echo "[results_campaign] Starting $CONDITION $run_id -> $run_dir"
  write_run_json "$metadata_path" "$CONDITION" "$run_id" "$run_dir" "starting" "" "${START_CMD[@]}"
  if ! (cd "$WS_ROOT" && "${START_CMD[@]}"); then
    write_run_json "$metadata_path" "$CONDITION" "$run_id" "$run_dir" "failed" "launch_failed" "${START_CMD[@]}"
    exit 1
  fi

  ACTIVE_STOP_CMD=("${STOP_CMD[@]}")
  if ! wait_for_recording "$run_dir"; then
    echo "[results_campaign] Recording did not start within ${RECORD_START_TIMEOUT_S}s for $run_id" >&2
    (cd "$WS_ROOT" && "${STOP_CMD[@]}") || true
    ACTIVE_STOP_CMD=()
    write_run_json "$metadata_path" "$CONDITION" "$run_id" "$run_dir" "failed" "record_start_timeout" "${START_CMD[@]}"
    exit 1
  fi

  write_run_json "$metadata_path" "$CONDITION" "$run_id" "$run_dir" "recording" "" "${START_CMD[@]}"
  echo "[results_campaign] Recording active; sleeping ${DURATION_S}s"
  sleep "$DURATION_S"

  echo "[results_campaign] Stopping $CONDITION $run_id"
  if ! (cd "$WS_ROOT" && "${STOP_CMD[@]}"); then
    ACTIVE_STOP_CMD=()
    write_run_json "$metadata_path" "$CONDITION" "$run_id" "$run_dir" "failed" "stop_failed" "${START_CMD[@]}"
    exit 1
  fi
  ACTIVE_STOP_CMD=()
  write_run_json "$metadata_path" "$CONDITION" "$run_id" "$run_dir" "completed" "" "${START_CMD[@]}"
done

summary_dir="$CONDITION_ROOT/summary"
mkdir -p "$summary_dir"
if python3 "$WS_ROOT/scripts/results_summarize_bag.py" "$CONDITION_ROOT" --out "$summary_dir" --warmup "$WARMUP_S"; then
  echo "[results_campaign] Summary written to $summary_dir"
else
  echo "[results_campaign] Warning: summary failed; bags are still in $CONDITION_ROOT" >&2
fi

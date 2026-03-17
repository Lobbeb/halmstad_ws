#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
WORLD="warehouse"
MODE="follow"
UAV_NAME="dji0"
PROFILE="default"
TAG=""
RUN_DIR=""
DRY_RUN=false

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
    profile:=*)
      PROFILE="${arg#profile:=}"
      ;;
    tag:=*)
      TAG="${arg#tag:=}"
      ;;
    out:=*)
      RUN_DIR="${arg#out:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      echo "Usage: $0 [world] [mode:=follow|yolo] [uav_name:=dji0] [profile:=default|vision] [tag:=name] [out:=bags/experiments/...] [dry_run:=true|false]" >&2
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
  default|vision)
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

TOPICS=(
  "/clock"
  "/coord/events"
  "/a201_0000/amcl_pose_odom"
  "/$UAV_NAME/pose"
  "/$UAV_NAME/pose_cmd"
  "/$UAV_NAME/pose_cmd/odom"
  "/$UAV_NAME/update_pan"
  "/$UAV_NAME/update_tilt"
  "/$UAV_NAME/camera0/actual/center_pose"
  "/$UAV_NAME/camera0/target/center_pose"
  "/$UAV_NAME/camera0/target_horizontal_distance_m"
  "/$UAV_NAME/camera0/target_distance_3d_m"
  "/$UAV_NAME/camera0/target_vertical_delta_m"
)

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
    "/$UAV_NAME/follow/actual/xy_distance_m"
    "/$UAV_NAME/follow/actual/distance_3d_m"
    "/$UAV_NAME/follow/error/xy_distance_m"
    "/$UAV_NAME/follow/error/anchor_distance_m"
  )
fi

if [ "$MODE" = "yolo" ]; then
  TOPICS+=(
    "/coord/leader_estimate"
    "/coord/leader_distance_debug"
    "/coord/leader_estimate_status"
    "/coord/leader_estimate_error"
    "/$UAV_NAME/follow/target/pan_deg"
    "/$UAV_NAME/follow/target/tilt_deg"
    "/$UAV_NAME/follow/actual/pan_deg"
    "/$UAV_NAME/follow/actual/tilt_deg"
    "/$UAV_NAME/follow/error/pan_deg"
    "/$UAV_NAME/follow/error/tilt_deg"
    "/$UAV_NAME/follow/target/d_target_m"
    "/$UAV_NAME/follow/target/xy_target_m"
    "/$UAV_NAME/follow/actual/xy_distance_m"
    "/$UAV_NAME/follow/actual/distance_3d_m"
    "/$UAV_NAME/follow/error/xy_distance_m"
    "/$UAV_NAME/follow/error/anchor_distance_m"
  )
fi

if [ "$PROFILE" = "step2_light" ]; then
  TOPICS+=(
    "/coord/leader_detection_status"
    "/coord/leader_selected_target"
    "/coord/leader_selected_target_filtered"
    "/coord/leader_selected_target_filtered_status"
    "/coord/leader_visual_target_estimate"
    "/coord/leader_visual_target_estimate_status"
    "/coord/leader_follow_point"
    "/coord/leader_follow_point_status"
    "/coord/leader_planned_target"
    "/coord/leader_planned_target_status"
    "/coord/leader_visual_control"
    "/coord/leader_visual_control_status"
    "/coord/leader_visual_actuation_bridge_status"
    "/$UAV_NAME/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
    "/$UAV_NAME/pose_cmd"
    "/$UAV_NAME/pose"
  )
fi

if [ "$PROFILE" = "vision" ]; then
  TOPICS+=(
    "/$UAV_NAME/camera0/image_raw"
    "/$UAV_NAME/camera0/camera_info"
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

git_branch="$(git -C "$WS_ROOT" branch --show-current 2>/dev/null || true)"
git_head="$(git -C "$WS_ROOT" rev-parse --short HEAD 2>/dev/null || true)"
if [ -n "$(git -C "$WS_ROOT" status --porcelain=v1 2>/dev/null || true)" ]; then
  git_dirty=true
else
  git_dirty=false
fi

invocation="$(shell_join "$0" "$WORLD" "mode:=$MODE" "uav_name:=$UAV_NAME" "profile:=$PROFILE" "$@")"
bag_command="$(shell_join ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}")"
hostname_value="$(hostname 2>/dev/null || true)"
started_at="$(date -Is)"

if [ "$DRY_RUN" = true ]; then
  echo "Run dir: $RUN_DIR_ABS"
  echo "Bag dir: $BAG_DIR"
  echo "Profile: $PROFILE"
  echo "Topics: ${#TOPICS[@]}"
  printf '%s\n' "${TOPICS[@]}"
  exit 0
fi

mkdir -p "$RUN_DIR_ABS"
printf '%s\n' "${TOPICS[@]}" > "$TOPICS_FILE"

{
  printf '{\n'
  printf '  "started_at": "%s",\n' "$(json_escape "$started_at")"
  printf '  "world": "%s",\n' "$(json_escape "$WORLD")"
  printf '  "mode": "%s",\n' "$(json_escape "$MODE")"
  printf '  "profile": "%s",\n' "$(json_escape "$PROFILE")"
  printf '  "uav_name": "%s",\n' "$(json_escape "$UAV_NAME")"
  printf '  "tag": "%s",\n' "$(json_escape "$TAG")"
  printf '  "run_name": "%s",\n' "$(json_escape "$run_name")"
  printf '  "run_dir": "%s",\n' "$(json_escape "$RUN_DIR_ABS")"
  printf '  "bag_dir": "%s",\n' "$(json_escape "$BAG_DIR")"
  printf '  "topics_file": "%s",\n' "$(json_escape "$TOPICS_FILE")"
  printf '  "workspace_root": "%s",\n' "$(json_escape "$WS_ROOT")"
  printf '  "hostname": "%s",\n' "$(json_escape "$hostname_value")"
  printf '  "git_branch": "%s",\n' "$(json_escape "$git_branch")"
  printf '  "git_head": "%s",\n' "$(json_escape "$git_head")"
  printf '  "git_dirty": %s,\n' "$git_dirty"
  printf '  "invocation": "%s",\n' "$(json_escape "$invocation")"
  printf '  "bag_command": "%s",\n' "$(json_escape "$bag_command")"
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

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

echo "[run_record_experiment] Recording profile=$PROFILE mode=$MODE topics=${#TOPICS[@]} bag_dir=$BAG_DIR"
exec ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}"

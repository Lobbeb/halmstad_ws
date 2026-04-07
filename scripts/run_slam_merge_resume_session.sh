#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MERGE_SESSION_ROOT="$WS_ROOT/maps/slam_merge_sessions"
MERGE_RUNTIME_DIR="/tmp/halmstad_ws"
MERGE_RUNTIME_ENV="$MERGE_RUNTIME_DIR/slam_merge_runtime.env"
DEFAULT_OUTPUT_NAME="baylands_merged"

RVIZ="true"
RVIZ_WARMUP_S="3"
MERGE_PID=""
RVIZ_PID=""
LOADED_STATE_INFO_LINES=()
LOADED_MARKER_NAMES=()

usage() {
  cat <<'EOF'
Usage: ./run.sh slam_merge_resume_session <session_name> [rviz:=true|false] [rviz_warmup_s:=3] [output:=baylands_merged]

Restores a saved merge session by:
  - relaunching merge_maps_kinematic
  - reloading the saved ordered submaps
  - replaying the saved interactive-marker poses

After restore, you can add more submaps manually in RViz.

Examples:
  ./run.sh slam_merge_resume_session baylands_parking_chain
  ./run.sh slam_merge_resume_session baylands_parking_chain rviz_warmup_s:=5
EOF
}

coerce_bool() {
  case "$1" in
    true|false)
      printf '%s\n' "$1"
      ;;
    *)
      echo "Invalid boolean value: $1" >&2
      exit 2
      ;;
  esac
}

resolve_service_name() {
  local suffix="$1"
  local service=""
  local services=""

  services="$(ros2 service list --no-daemon 2>/dev/null || true)"

  while IFS= read -r service; do
    [ -n "$service" ] || continue
    case "$service" in
      "$suffix"|*/"$suffix")
        printf '%s\n' "$service"
        return 0
        ;;
    esac
  done <<< "$services"

  return 1
}

wait_for_service_suffix() {
  local suffix="$1"
  local timeout_s="${2:-30}"
  local waited=0
  local resolved=""

  while [ "$waited" -lt "$timeout_s" ]; do
    if resolved="$(resolve_service_name "$suffix")"; then
      printf '%s\n' "$resolved"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done

  return 1
}

cleanup() {
  if [ -n "$RVIZ_PID" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill -INT "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  if [ -n "$MERGE_PID" ] && kill -0 "$MERGE_PID" 2>/dev/null; then
    kill -INT "$MERGE_PID" 2>/dev/null || true
    wait "$MERGE_PID" 2>/dev/null || true
  fi
}

write_runtime_file() {
  mkdir -p "$MERGE_RUNTIME_DIR"
  {
    printf 'world=%q\n' "baylands"
    printf 'output_name=%q\n' "$OUTPUT_NAME"
    printf 'autoload=%q\n' "true"
    printf 'autoplace=%q\n' "false"
    printf 'rviz=%q\n' "$RVIZ"
    printf 'loaded_state_count=%q\n' "${#LOADED_STATE_INFO_LINES[@]}"
    local idx=0
    local line=""
    local state_name=""
    local posegraph_base=""
    local metadata_path=""
    for line in "${LOADED_STATE_INFO_LINES[@]}"; do
      IFS='|' read -r state_name posegraph_base metadata_path <<< "$line"
      printf 'loaded_state_%d=%q\n' "$idx" "$state_name"
      printf 'loaded_posegraph_base_%d=%q\n' "$idx" "$posegraph_base"
      printf 'loaded_metadata_path_%d=%q\n' "$idx" "$metadata_path"
      idx=$((idx + 1))
    done
  } > "$MERGE_RUNTIME_ENV"
}

if [ $# -lt 1 ]; then
  usage >&2
  exit 1
fi

case "${1:-}" in
  help|-h|--help)
    usage
    exit 0
    ;;
esac

SESSION_NAME="$1"
shift

OUTPUT_NAME=""

for arg in "$@"; do
  case "$arg" in
    rviz:=*)
      RVIZ="$(coerce_bool "${arg#rviz:=}")"
      ;;
    rviz_warmup_s:=*)
      RVIZ_WARMUP_S="${arg#rviz_warmup_s:=}"
      ;;
    output:=*)
      OUTPUT_NAME="${arg#output:=}"
      ;;
    *)
      echo "[run_slam_merge_resume_session] Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

SESSION_ENV="$MERGE_SESSION_ROOT/$SESSION_NAME/session.env"
if [ ! -f "$SESSION_ENV" ]; then
  echo "[run_slam_merge_resume_session] Missing session file: $SESSION_ENV" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$SESSION_ENV"

if [ "${world:-}" != "baylands" ]; then
  echo "[run_slam_merge_resume_session] Session '$SESSION_NAME' belongs to world '${world:-unknown}', not baylands" >&2
  exit 1
fi

if [ -z "$OUTPUT_NAME" ]; then
  OUTPUT_NAME="${saved_output_name:-$DEFAULT_OUTPUT_NAME}"
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

trap cleanup EXIT INT TERM

ros2 launch slam_toolbox merge_maps_kinematic_launch.py &
MERGE_PID=$!

ADD_SUBMAP_SERVICE="$(wait_for_service_suffix "slam_toolbox/add_submap" 30 || true)"
if [ -z "$ADD_SUBMAP_SERVICE" ]; then
  ADD_SUBMAP_SERVICE="$(wait_for_service_suffix "add_submap" 5 || true)"
fi

if [ -z "$ADD_SUBMAP_SERVICE" ]; then
  echo "[run_slam_merge_resume_session] Failed to find add_submap service from merge_maps_kinematic." >&2
  exit 1
fi

if [ "$RVIZ" = "true" ]; then
  rviz2 -d /opt/ros/jazzy/share/slam_toolbox/config/slam_toolbox_default.rviz &
  RVIZ_PID=$!
  sleep "$RVIZ_WARMUP_S"
fi

for ((i=0; i<state_count; i++)); do
  state_var="state_${i}"
  posegraph_var="posegraph_base_${i}"
  metadata_var="metadata_path_${i}"
  marker_x_var="marker_x_${i}"
  marker_y_var="marker_y_${i}"
  marker_yaw_var="marker_yaw_${i}"

  state_name="${!state_var}"
  posegraph_base="${!posegraph_var}"
  metadata_path="${!metadata_var}"
  marker_x="${!marker_x_var}"
  marker_y="${!marker_y_var}"
  marker_yaw="${!marker_yaw_var}"

  known_markers="$(python3 "$SCRIPT_DIR/merge_autoplace_markers.py" list --namespace /merge_maps_tool 2>/dev/null || true)"

  echo "[run_slam_merge_resume_session] Restoring $state_name"
  ros2 service call \
    "$ADD_SUBMAP_SERVICE" \
    slam_toolbox/srv/AddSubmap \
    "{filename: '$posegraph_base'}" >/dev/null

  if ! marker_name="$(python3 "$SCRIPT_DIR/merge_autoplace_markers.py" \
    wait-new \
    --namespace /merge_maps_tool \
    --known-markers "$known_markers")"; then
    echo "[run_slam_merge_resume_session] Failed to detect merge marker for $state_name" >&2
    exit 1
  fi

  LOADED_STATE_INFO_LINES+=("$state_name|$posegraph_base|$metadata_path")
  LOADED_MARKER_NAMES+=("$marker_name")
done

for ((i=0; i<state_count; i++)); do
  state_var="state_${i}"
  marker_x_var="marker_x_${i}"
  marker_y_var="marker_y_${i}"
  marker_yaw_var="marker_yaw_${i}"

  state_name="${!state_var}"
  marker_name="${LOADED_MARKER_NAMES[$i]}"
  marker_x="${!marker_x_var}"
  marker_y="${!marker_y_var}"
  marker_yaw="${!marker_yaw_var}"

  echo "[run_slam_merge_resume_session] Applying saved pose for $state_name via marker $marker_name"
  if ! python3 "$SCRIPT_DIR/merge_autoplace_markers.py" \
    set-by-name \
    --namespace /merge_maps_tool \
    --marker-name "$marker_name" \
    --x "$marker_x" \
    --y "$marker_y" \
    --yaw "$marker_yaw"; then
    echo "[run_slam_merge_resume_session] Failed to apply saved pose for $state_name" >&2
    exit 1
  fi
done

write_runtime_file

echo "[run_slam_merge_resume_session] Restored $state_count submaps from session '$SESSION_NAME'"
echo "You can now add more submaps manually in the RViz Slam Toolbox panel."
echo
echo "When you are happy with the alignment, run in another terminal:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source $WS_ROOT/install/setup.bash"
echo "  ros2 service call /slam_toolbox/merge_submaps slam_toolbox/srv/MergeMaps '{}'"
echo "  ros2 run nav2_map_server map_saver_cli -f $WS_ROOT/maps/$OUTPUT_NAME --ros-args -r map:=/map"

if [ "$RVIZ" = "true" ]; then
  wait "$RVIZ_PID"
else
  wait "$MERGE_PID"
fi

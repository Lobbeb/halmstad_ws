#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_ROOT="$WS_ROOT/maps/slam_states"
MERGE_SESSION_ROOT="$WS_ROOT/maps/slam_merge_sessions"
MERGE_RUNTIME_ENV="/tmp/halmstad_ws/slam_merge_runtime.env"
DEFAULT_OUTPUT_NAME="baylands_merged"

usage() {
  cat <<'EOF'
Usage: ./run.sh slam_merge_save_session <session_name> [state ...]

Saves the currently loaded Slam Toolbox merge session as:
  - ordered submap list
  - current interactive marker poses

If no states are provided, the command uses the currently active autoloaded
merge session from /tmp/halmstad_ws/slam_merge_runtime.env.

If you added submaps manually in RViz, provide the states in the same load order.

Examples:
  ./run.sh slam_merge_save_session baylands_parking_chain
  ./run.sh slam_merge_save_session baylands_parking_chain baylands_parkinglot_4 baylands_parkinglot_5
  ./run.sh slam_merge_save_session baylands_parking_chain parkinglot_4 parkinglot_5
EOF
}

resolve_state_name() {
  local token="$1"
  if [ -d "$STATE_ROOT/$token" ]; then
    printf '%s\n' "$token"
    return 0
  fi
  if [ -d "$STATE_ROOT/baylands_$token" ]; then
    printf 'baylands_%s\n' "$token"
    return 0
  fi
  echo "[run_slam_merge_save_session] Unknown state: $token" >&2
  exit 2
}

load_state_info_line() {
  local state_name="$1"
  local metadata_path="$STATE_ROOT/$state_name/metadata.env"

  if [ ! -f "$metadata_path" ]; then
    echo "[run_slam_merge_save_session] Missing metadata for '$state_name': $metadata_path" >&2
    exit 1
  fi

  # shellcheck disable=SC1090
  source "$metadata_path"

  if [ ! -f "${posegraph_file:-}" ] || [ ! -f "${posegraph_data:-}" ]; then
    echo "[run_slam_merge_save_session] Missing posegraph artifacts for '$state_name'" >&2
    exit 1
  fi

  printf '%s|%s|%s\n' "$state_name" "${posegraph_base:-}" "$metadata_path"
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

STATE_INFO_LINES=()
OUTPUT_NAME="$DEFAULT_OUTPUT_NAME"

if [ "$#" -gt 0 ]; then
  for token in "$@"; do
    state_name="$(resolve_state_name "$token")"
    STATE_INFO_LINES+=("$(load_state_info_line "$state_name")")
  done
elif [ -f "$MERGE_RUNTIME_ENV" ]; then
  # shellcheck disable=SC1090
  source "$MERGE_RUNTIME_ENV"
  OUTPUT_NAME="${output_name:-$DEFAULT_OUTPUT_NAME}"
  loaded_count="${loaded_state_count:-0}"
  if [ "${loaded_count:-0}" -eq 0 ]; then
    echo "[run_slam_merge_save_session] Active merge runtime contains no tracked loaded states." >&2
    echo "Provide the states manually in the same load order they were added." >&2
    exit 1
  fi

  for ((i=0; i<loaded_count; i++)); do
    state_var="loaded_state_${i}"
    posegraph_var="loaded_posegraph_base_${i}"
    metadata_var="loaded_metadata_path_${i}"
    state_name="${!state_var}"
    posegraph_base="${!posegraph_var}"
    metadata_path="${!metadata_var}"
    STATE_INFO_LINES+=("$state_name|$posegraph_base|$metadata_path")
  done
else
  echo "[run_slam_merge_save_session] No active merge runtime found at $MERGE_RUNTIME_ENV" >&2
  echo "Provide the states manually in the same load order they were added." >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

mapfile -t MARKER_LINES < <(python3 "$SCRIPT_DIR/merge_autoplace_markers.py" dump --namespace /merge_maps_tool)

if [ "${#MARKER_LINES[@]}" -eq 0 ]; then
  echo "[run_slam_merge_save_session] No merge markers were found. Is merge_maps_kinematic running?" >&2
  exit 1
fi

if [ "${#MARKER_LINES[@]}" -ne "${#STATE_INFO_LINES[@]}" ]; then
  echo "[run_slam_merge_save_session] Marker count (${#MARKER_LINES[@]}) does not match tracked state count (${#STATE_INFO_LINES[@]})." >&2
  echo "If you added submaps manually, provide the states explicitly in the same load order." >&2
  exit 1
fi

SESSION_DIR="$MERGE_SESSION_ROOT/$SESSION_NAME"
mkdir -p "$SESSION_DIR"
SESSION_ENV="$SESSION_DIR/session.env"

{
  printf 'session_name=%q\n' "$SESSION_NAME"
  printf 'world=%q\n' "baylands"
  printf 'saved_output_name=%q\n' "$OUTPUT_NAME"
  printf 'created_at_utc=%q\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf 'state_count=%q\n' "${#STATE_INFO_LINES[@]}"

  for ((i=0; i<${#STATE_INFO_LINES[@]}; i++)); do
    IFS='|' read -r state_name posegraph_base metadata_path <<< "${STATE_INFO_LINES[$i]}"
    IFS=$'\t' read -r marker_name marker_x marker_y marker_yaw <<< "${MARKER_LINES[$i]}"

    printf 'state_%d=%q\n' "$i" "$state_name"
    printf 'posegraph_base_%d=%q\n' "$i" "$posegraph_base"
    printf 'metadata_path_%d=%q\n' "$i" "$metadata_path"
    printf 'marker_name_%d=%q\n' "$i" "$marker_name"
    printf 'marker_x_%d=%q\n' "$i" "$marker_x"
    printf 'marker_y_%d=%q\n' "$i" "$marker_y"
    printf 'marker_yaw_%d=%q\n' "$i" "$marker_yaw"
  done
} > "$SESSION_ENV"

echo "[run_slam_merge_save_session] Saved merge session to $SESSION_ENV"
echo "Resume it later with:"
echo "  ./run.sh slam_merge_resume_session $SESSION_NAME"

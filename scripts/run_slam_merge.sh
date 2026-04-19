#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_ROOT="$WS_ROOT/maps/slam_states"
MERGE_RUNTIME_DIR="/tmp/halmstad_ws"
MERGE_RUNTIME_ENV="$MERGE_RUNTIME_DIR/slam_merge_runtime.env"
DEFAULT_OUTPUT_NAME="baylands_merged"

RVIZ="true"
DRY_RUN="false"
AUTOLOAD="true"
AUTOPLACE="false"
OUTPUT_NAME="$DEFAULT_OUTPUT_NAME"
RVIZ_WARMUP_S="3"
TOKENS=()
STATE_NAMES=()
STATE_INFO_LINES=()
AUTOPLACE_REFERENCE_STATE=""
AUTOPLACE_REFERENCE_METADATA=""
MERGE_PID=""
RVIZ_PID=""
LOADED_STATE_INFO_LINES=()

usage() {
  cat <<'EOF'
Usage: ./run.sh slam_merge [selection ...] [rviz:=true|false] [autoload:=true|false] [autoplace:=true|false] [output:=baylands_merged] [rviz_warmup_s:=3] [dry_run:=true|false]

Selections:
  all
  baylands_*
  parkinglot_1-5
  parkinglot2_0-2
  spawn_0-2
  baylands_parkinglot_1
  baylands_parkinglot2_2
  baylands_spawn_1

Examples:
  ./run.sh slam_merge
  ./run.sh slam_merge parkinglot_1-5 parkinglot2_0-2 spawn_0-2
  ./run.sh slam_merge autoload:=false
  ./run.sh slam_merge parkinglot_4 parkinglot_5 autoplace:=true
  ./run.sh slam_merge baylands_parkinglot_5 baylands_spawn_1
  ./run.sh slam_merge dry_run:=true
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

append_unique() {
  local value="$1"
  local existing=""
  for existing in "${STATE_NAMES[@]:-}"; do
    if [ "$existing" = "$value" ]; then
      return 0
    fi
  done
  STATE_NAMES+=("$value")
}

resolve_glob_states() {
  local pattern="$1"
  local matched=0
  local dir=""
  shopt -s nullglob
  for dir in "$STATE_ROOT"/$pattern; do
    [ -d "$dir" ] || continue
    append_unique "$(basename "$dir")"
    matched=1
  done
  shopt -u nullglob
  return $((1 - matched))
}

expand_range_token() {
  local token="$1"
  local prefix=""
  local start=""
  local end=""
  local i=""

  case "$token" in
    parkinglot_[0-9]-[0-9])
      prefix="baylands_parkinglot_"
      start="${token#parkinglot_}"
      start="${start%-*}"
      end="${token##*-}"
      ;;
    parkinglot2_[0-9]-[0-9])
      prefix="baylands_parkinglot2_"
      start="${token#parkinglot2_}"
      start="${start%-*}"
      end="${token##*-}"
      ;;
    spawn_[0-9]-[0-9])
      prefix="baylands_spawn_"
      start="${token#spawn_}"
      start="${start%-*}"
      end="${token##*-}"
      ;;
    *)
      return 1
      ;;
  esac

  for ((i=start; i<=end; i++)); do
    append_unique "${prefix}${i}"
  done
  return 0
}

resolve_state_token() {
  local token="$1"

  case "$token" in
    all)
      resolve_glob_states "baylands_*" || true
      return 0
      ;;
    baylands_*)
      if [[ "$token" == *"*"* ]]; then
        if ! resolve_glob_states "$token"; then
          echo "[run_slam_merge] No states matched pattern '$token'" >&2
          exit 2
        fi
      else
        append_unique "$token"
      fi
      return 0
      ;;
  esac

  if expand_range_token "$token"; then
    return 0
  fi

  case "$token" in
    parkinglot_[0-9]|parkinglot_[0-9][0-9])
      append_unique "baylands_${token}"
      return 0
      ;;
    parkinglot2_[0-9]|parkinglot2_[0-9][0-9])
      append_unique "baylands_${token}"
      return 0
      ;;
    spawn_[0-9]|spawn_[0-9][0-9])
      append_unique "baylands_${token}"
      return 0
      ;;
  esac

  echo "[run_slam_merge] Unknown selection token: $token" >&2
  exit 2
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
    printf 'autoload=%q\n' "$AUTOLOAD"
    printf 'autoplace=%q\n' "$AUTOPLACE"
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

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    rviz:=*)
      RVIZ="$(coerce_bool "${arg#rviz:=}")"
      ;;
    autoload:=*)
      AUTOLOAD="$(coerce_bool "${arg#autoload:=}")"
      ;;
    autoplace:=*)
      AUTOPLACE="$(coerce_bool "${arg#autoplace:=}")"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${arg#dry_run:=}")"
      ;;
    rviz_warmup_s:=*)
      RVIZ_WARMUP_S="${arg#rviz_warmup_s:=}"
      ;;
    output:=*)
      OUTPUT_NAME="${arg#output:=}"
      ;;
    *)
      TOKENS+=("$arg")
      ;;
  esac
done

if [ "$AUTOLOAD" = "true" ] && [ "${#TOKENS[@]}" -eq 0 ]; then
  TOKENS=("parkinglot_1-5" "parkinglot2_0-2" "spawn_0-2")
fi

if [ "${#TOKENS[@]}" -gt 0 ]; then
  for token in "${TOKENS[@]}"; do
    resolve_state_token "$token"
  done
fi

if [ "$AUTOLOAD" = "true" ] && [ "${#STATE_NAMES[@]}" -eq 0 ]; then
  echo "[run_slam_merge] No matching states were selected." >&2
  exit 1
fi

for state_name in "${STATE_NAMES[@]}"; do
  metadata_path="$STATE_ROOT/$state_name/metadata.env"
  if [ ! -f "$metadata_path" ]; then
    echo "[run_slam_merge] Missing metadata for state '$state_name': $metadata_path" >&2
    exit 1
  fi

  # shellcheck disable=SC1090
  source "$metadata_path"

  if [ ! -f "${posegraph_file:-}" ] || [ ! -f "${posegraph_data:-}" ]; then
    echo "[run_slam_merge] Missing posegraph artifacts for '$state_name'" >&2
    exit 1
  fi

  if [ "${world:-}" != "baylands" ]; then
    echo "[run_slam_merge] State '$state_name' belongs to world '${world:-unknown}', not baylands" >&2
    exit 1
  fi

  has_autoplace="false"
  if [ -n "${spawn_x:-}" ] && [ -n "${spawn_y:-}" ] && [ -n "${spawn_yaw:-}" ] && \
     [ -n "${map_pose_x:-}" ] && [ -n "${map_pose_y:-}" ] && [ -n "${map_pose_yaw:-}" ]; then
    has_autoplace="true"
    if [ -z "$AUTOPLACE_REFERENCE_STATE" ]; then
      AUTOPLACE_REFERENCE_STATE="$state_name"
      AUTOPLACE_REFERENCE_METADATA="$metadata_path"
    fi
  fi

  STATE_INFO_LINES+=("$state_name|${posegraph_base:-}|${map_yaml:-}|$metadata_path|$has_autoplace")
done

if [ "$DRY_RUN" = "true" ]; then
  if [ "${#STATE_NAMES[@]}" -gt 0 ]; then
    echo "Selected states:"
    printf '  %s\n' "${STATE_NAMES[@]}"
    echo
  fi
  echo "Would launch:"
  echo "  ros2 launch slam_toolbox merge_maps_kinematic_launch.py"
  if [ "$RVIZ" = "true" ]; then
    echo "  rviz2 -d /opt/ros/jazzy/share/slam_toolbox/config/slam_toolbox_default.rviz"
  fi
  echo
  if [ "$AUTOPLACE" = "true" ]; then
    if [ -n "$AUTOPLACE_REFERENCE_STATE" ]; then
      echo "Auto-place reference state: $AUTOPLACE_REFERENCE_STATE"
    else
      echo "Auto-place requested, but no selected states contain both spawn_* and map_pose_* metadata."
    fi
    echo
  fi
  if [ "$AUTOLOAD" = "true" ]; then
    echo "Would add submaps:"
    for line in "${STATE_INFO_LINES[@]}"; do
      IFS='|' read -r state_name posegraph_base map_yaml metadata_path has_autoplace <<< "$line"
      if [ "$AUTOPLACE" = "true" ]; then
        echo "  $state_name -> $posegraph_base (autoplace=$has_autoplace)"
      else
        echo "  $state_name -> $posegraph_base"
      fi
    done
    echo
  else
    echo "Autoload disabled; add submaps manually from the RViz plugin."
    if [ "${#STATE_INFO_LINES[@]}" -gt 0 ]; then
      echo "Candidate submap paths:"
      for line in "${STATE_INFO_LINES[@]}"; do
        IFS='|' read -r state_name posegraph_base map_yaml metadata_path has_autoplace <<< "$line"
        echo "  $state_name -> $posegraph_base"
      done
      echo
    fi
  fi
  echo "After alignment, merge with:"
  echo "  ros2 service call /slam_toolbox/merge_submaps slam_toolbox/srv/MergeMaps '{}'"
  echo "  ros2 run nav2_map_server map_saver_cli -f $WS_ROOT/maps/$OUTPUT_NAME --ros-args -r map:=/map"
  exit 0
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
  echo "[run_slam_merge] Failed to find add_submap service from merge_maps_kinematic." >&2
  exit 1
fi

if [ "$RVIZ" = "true" ]; then
  rviz2 -d /opt/ros/jazzy/share/slam_toolbox/config/slam_toolbox_default.rviz &
  RVIZ_PID=$!
  sleep "$RVIZ_WARMUP_S"
fi

if [ "$AUTOLOAD" = "false" ] && [ "$AUTOPLACE" = "true" ]; then
  echo "[run_slam_merge] Warning: autoplace:=true only applies to autoloaded submaps." >&2
fi

if [ "$AUTOLOAD" = "true" ]; then
  for line in "${STATE_INFO_LINES[@]}"; do
    IFS='|' read -r state_name posegraph_base map_yaml metadata_path has_autoplace <<< "$line"
    known_markers=""
    if [ "$AUTOPLACE" = "true" ] && [ -n "$AUTOPLACE_REFERENCE_METADATA" ]; then
      known_markers="$(python3 "$SCRIPT_DIR/merge_autoplace_markers.py" list --namespace /merge_maps_tool 2>/dev/null || true)"
    fi

    echo "[run_slam_merge] Adding $state_name"
    ros2 service call \
      "$ADD_SUBMAP_SERVICE" \
      slam_toolbox/srv/AddSubmap \
      "{filename: '$posegraph_base'}" >/dev/null

    LOADED_STATE_INFO_LINES+=("$state_name|$posegraph_base|$metadata_path")

    if [ "$AUTOPLACE" = "true" ] && [ -n "$AUTOPLACE_REFERENCE_METADATA" ]; then
      if [ "$has_autoplace" = "true" ]; then
        echo "[run_slam_merge] Auto-placing $state_name from metadata"
        if ! python3 "$SCRIPT_DIR/merge_autoplace_markers.py" \
          set-new-from-metadata \
          --namespace /merge_maps_tool \
          --known-markers "$known_markers" \
          --reference-metadata "$AUTOPLACE_REFERENCE_METADATA" \
          --target-metadata "$metadata_path"; then
          echo "[run_slam_merge] Warning: failed to auto-place $state_name; leave it for manual alignment." >&2
        fi
      else
        echo "[run_slam_merge] $state_name is missing spawn_* or map_pose_* metadata; place it manually." >&2
      fi
    fi
  done

  echo "[run_slam_merge] Loaded ${#STATE_NAMES[@]} submaps"
  echo "Selected:"
  printf '  %s\n' "${STATE_NAMES[@]}"
else
  echo "[run_slam_merge] Autoload disabled"
  echo "Add submaps manually in the RViz Slam Toolbox panel."
  if [ "${#STATE_INFO_LINES[@]}" -gt 0 ]; then
    echo "Suggested submap paths:"
    for line in "${STATE_INFO_LINES[@]}"; do
      IFS='|' read -r state_name posegraph_base map_yaml <<< "$line"
      echo "  $state_name -> $posegraph_base"
    done
  fi
fi

write_runtime_file

echo
echo "When you are happy with the alignment, run in another terminal:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source $WS_ROOT/install/setup.bash"
echo "  ros2 service call /slam_toolbox/merge_submaps slam_toolbox/srv/MergeMaps '{}'"
echo "  ros2 run nav2_map_server map_saver_cli -f $WS_ROOT/maps/$OUTPUT_NAME --ros-args -r map:=/map"
echo
echo "To save this merge session for later continuation:"
echo "  ./run.sh slam_merge_save_session <session_name>"
echo "To reopen it later:"
echo "  ./run.sh slam_merge_resume_session <session_name>"

if [ "$RVIZ" = "true" ]; then
  wait "$RVIZ_PID"
else
  wait "$MERGE_PID"
fi

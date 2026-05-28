#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
SIM_SPAWN_WAYPOINT_FILE="$STATE_DIR/gazebo_sim.spawn_waypoint"
DEFAULT_WORLD="baylands"
DEFAULT_BAYLANDS_MAP="$WS_ROOT/maps/baylands.yaml"
DEFAULT_BAYLANDS_INITIAL_POSE_MAP="$DEFAULT_BAYLANDS_MAP"
DEFAULT_BAYLANDS_INITIAL_POSE_COMPAT_MAPS=(
  "$WS_ROOT/maps/baylands.yaml"
  "$WS_ROOT/maps/baylands_merged.yaml"
)
DEFAULT_BAYLANDS_INITIAL_POSE_X="-17.8523709280687"
DEFAULT_BAYLANDS_INITIAL_POSE_Y="6.112792742664274"
DEFAULT_BAYLANDS_INITIAL_POSE_YAW="0.0064542265"
LOCAL_BAYLANDS_LOCALIZATION_PARAMS="$WS_ROOT/src/lrs_halmstad/config/localization.yaml"
WORLD="$DEFAULT_WORLD"
MAP_PATH=""
PARAMS_FILE_OVERRIDE=""
LOCAL_LOCALIZATION_LAUNCH="$WS_ROOT/src/lrs_halmstad/launch/localization_with_params.launch.py"

source "$SCRIPT_DIR/lidar_mode_common.sh"
source "$SCRIPT_DIR/baylands_waypoint_common.sh"
source "$SCRIPT_DIR/baylands_route_lidar_common.sh"
DEFAULT_BAYLANDS_GROUP_WAYPOINT_CSV="$(baylands_group_waypoint_csv)"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh localization [world] [map.yaml] [lidar:=2d|3d] [options...]

Start map server, AMCL, scan relay, and optional pointcloud_to_laserscan for
the active simulated UGV.

Common options:
  lidar:=2d|3d
  scan_topic:=/topic
  pointcloud_topic:=/topic
  use_scan_relay:=true|false
  scan_relay_hz:=10.0
  scan_relay_max_age_s:=...
  params_file:=path/to/localization.yaml
  set_initial_pose:=true
  initial_pose_x:=... initial_pose_y:=... initial_pose_yaw:=...

Examples:
  ./run.sh localization baylands lidar:=3d
  ./run.sh localization baylands maps/baylands.yaml lidar:=2d
EOF
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
  esac
done

resolve_localization_map_path() {
  local requested_path="$1"

  if [[ "$requested_path" = /* ]]; then
    printf '%s\n' "$requested_path"
    return 0
  fi

  if [ -f "$requested_path" ]; then
    readlink -f "$requested_path" 2>/dev/null || printf '%s\n' "$requested_path"
    return 0
  fi

  if [ -f "$WS_ROOT/$requested_path" ]; then
    printf '%s\n' "$WS_ROOT/$requested_path"
    return 0
  fi

  if [ -f "$WS_ROOT/maps/$requested_path" ]; then
    printf '%s\n' "$WS_ROOT/maps/$requested_path"
    return 0
  fi

  printf '%s\n' "$requested_path"
}

resolve_baylands_amcl_waypoint_pose() {
  local waypoint_name="$1"
  python3 - "$waypoint_name" "$DEFAULT_BAYLANDS_GROUP_WAYPOINT_CSV" <<'PY'
import csv
import sys

name, *paths = sys.argv[1:]
names = []

for path in paths:
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                waypoint_name = str(row.get("place", "")).strip()
                if waypoint_name:
                    names.append(waypoint_name)
                if waypoint_name != name:
                    continue

                amcl_x = row.get("amcl_x")
                amcl_y = row.get("amcl_y")
                amcl_yaw = row.get("amcl_yaw")
                if amcl_x in (None, "") or amcl_y in (None, "") or amcl_yaw in (None, ""):
                    raise SystemExit(
                        f"Waypoint '{name}' is missing amcl_x/amcl_y/amcl_yaw in {path}"
                    )

                print(f"initial_pose_x={float(amcl_x)}")
                print(f"initial_pose_y={float(amcl_y)}")
                print(f"initial_pose_yaw={float(amcl_yaw)}")
                raise SystemExit(0)
    except FileNotFoundError:
        continue

available = ", ".join(dict.fromkeys(names))
raise SystemExit(f"Waypoint '{name}' was not found. Available: {available}")
PY
}

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

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  MAP_PATH="$1"
  shift
fi

if [ -n "$MAP_PATH" ]; then
  MAP_PATH="$(resolve_localization_map_path "$MAP_PATH")"
fi

lidar_mode_parse_args 3d "$@"
USE_POINTCLOUD_TO_LASERSCAN="false"
USE_SCAN_RELAY="false"
SCAN_RELAY_HZ=""
SCAN_RELAY_MAX_AGE_S=""
SCAN_RELAY_START_DELAY_S=""
SIM_SPAWN_WAYPOINT=""
ROUTE_LIDAR_ARGS=()

if [ -f "$SIM_SPAWN_WAYPOINT_FILE" ]; then
  SIM_SPAWN_WAYPOINT="$(tr -d '\r\n' < "$SIM_SPAWN_WAYPOINT_FILE" 2>/dev/null || true)"
fi

if [ -n "$LIDAR_POINTCLOUD_TOPIC" ] && [ "$LIDAR_SCAN_TOPIC" = "$(lidar_mode_scan_topic 3d)" ]; then
  USE_POINTCLOUD_TO_LASERSCAN="true"
fi

has_initial_pose_override="false"
for arg in "${LIDAR_REMAINING_ARGS[@]}"; do
  case "$arg" in
    use_scan_relay:=*)
      USE_SCAN_RELAY="${arg#use_scan_relay:=}"
      ;;
    scan_relay_hz:=*)
      SCAN_RELAY_HZ="${arg#scan_relay_hz:=}"
      ;;
    scan_relay_max_age_s:=*)
      SCAN_RELAY_MAX_AGE_S="${arg#scan_relay_max_age_s:=}"
      ;;
    scan_relay_start_delay_s:=*)
      SCAN_RELAY_START_DELAY_S="${arg#scan_relay_start_delay_s:=}"
      ;;
    params_file:=*)
      PARAMS_FILE_OVERRIDE="${arg#params_file:=}"
      ;;
    set_initial_pose:=*|always_reset_initial_pose:=*|initial_pose_x:=*|initial_pose_y:=*|initial_pose_yaw:=*)
      has_initial_pose_override="true"
      ;;
  esac
done

if [ -n "$PARAMS_FILE_OVERRIDE" ]; then
  PARAMS_FILE_OVERRIDE="$(resolve_localization_map_path "$PARAMS_FILE_OVERRIDE")"
elif [[ "$WORLD" == baylands* ]] && [ -f "$LOCAL_BAYLANDS_LOCALIZATION_PARAMS" ]; then
  PARAMS_FILE_OVERRIDE="$LOCAL_BAYLANDS_LOCALIZATION_PARAMS"
fi

if [[ "$WORLD" == baylands* ]] && [ "$LIDAR_MODE" = "2d" ]; then
  if [ -z "$USE_SCAN_RELAY" ]; then
    USE_SCAN_RELAY="true"
  fi
  if [ -z "$SCAN_RELAY_HZ" ]; then
    SCAN_RELAY_HZ="8.0"
  fi
elif [ "$LIDAR_MODE" = "3d" ] && [ "$USE_POINTCLOUD_TO_LASERSCAN" = "true" ]; then
  if [ -z "$USE_SCAN_RELAY" ]; then
    USE_SCAN_RELAY="true"
  fi
  if [ -z "$SCAN_RELAY_HZ" ]; then
    SCAN_RELAY_HZ="10.0"
  fi
  if [ -z "$SCAN_RELAY_START_DELAY_S" ]; then
    SCAN_RELAY_START_DELAY_S="1.0"
  fi
elif [ -z "$USE_SCAN_RELAY" ]; then
  USE_SCAN_RELAY="false"
fi

if [ -z "$MAP_PATH" ]; then
  case "$WORLD" in
    warehouse*)
      MAP_PATH="/opt/ros/jazzy/share/clearpath_nav2_demos/maps/warehouse.yaml"
      ;;
    baylands*)
      MAP_PATH="$DEFAULT_BAYLANDS_MAP"
      ;;
    *)
      if [ -f "$WS_ROOT/maps/${WORLD}.yaml" ]; then
        MAP_PATH="$WS_ROOT/maps/${WORLD}.yaml"
      elif [ -f "$WS_ROOT/maps/${WORLD}_manual.yaml" ]; then
        MAP_PATH="$WS_ROOT/maps/${WORLD}_manual.yaml"
      else
        echo "[run_localization] No default map found for world '$WORLD'. Pass an explicit map path as the second argument." >&2
        exit 1
      fi
      ;;
  esac
fi

if [ ! -f "$MAP_PATH" ]; then
  echo "[run_localization] Map file not found: $MAP_PATH" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

LAUNCH_ARGS=(
  use_sim_time:=true
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath"
  scan_topic:="$LIDAR_SCAN_TOPIC"
  use_pointcloud_to_laserscan:="$USE_POINTCLOUD_TO_LASERSCAN"
  use_scan_relay:="$USE_SCAN_RELAY"
  map:="$MAP_PATH"
)

if [ "$LIDAR_MODE" = "3d" ] && [ "$USE_POINTCLOUD_TO_LASERSCAN" = "true" ]; then
  if [ "$USE_SCAN_RELAY" = "true" ]; then
    echo "[run_localization] 3D lidar mode: using restamped scan relay for AMCL timing stability" >&2
  else
    echo "[run_localization] 3D lidar mode: using direct pc2ls scan; AMCL may drop scans if TF lags behind timestamps" >&2
  fi
fi

resolved_map_path="$(readlink -f "$MAP_PATH" 2>/dev/null || printf '%s' "$MAP_PATH")"
baylands_pose_compatible_map="false"
for compat_map in "${DEFAULT_BAYLANDS_INITIAL_POSE_COMPAT_MAPS[@]}"; do
  resolved_compat_map="$(readlink -f "$compat_map" 2>/dev/null || printf '%s' "$compat_map")"
  if [ "$resolved_map_path" = "$resolved_compat_map" ]; then
    baylands_pose_compatible_map="true"
    break
  fi
done

if [[ "$WORLD" == baylands* ]] && [ "$baylands_pose_compatible_map" = "true" ] && [ "$has_initial_pose_override" = "false" ]; then
  if [ -n "$SIM_SPAWN_WAYPOINT" ] && [ -f "$DEFAULT_BAYLANDS_GROUP_WAYPOINT_CSV" ]; then
      if waypoint_pose_env="$(resolve_baylands_amcl_waypoint_pose "$SIM_SPAWN_WAYPOINT" 2>/dev/null)"; then
        eval "$waypoint_pose_env"
        echo "[run_localization] Baylands waypoint '$SIM_SPAWN_WAYPOINT' detected: using matching AMCL initial pose" >&2
        LAUNCH_ARGS+=(
          set_initial_pose:=true
          always_reset_initial_pose:=true
          initial_pose_x:="$initial_pose_x"
          initial_pose_y:="$initial_pose_y"
          initial_pose_yaw:="$initial_pose_yaw"
        )
      else
        echo "[run_localization] Baylands waypoint '$SIM_SPAWN_WAYPOINT' has no complete AMCL pose; falling back to saved default initial pose" >&2
      fi
  fi

  if ! printf '%s\n' "${LAUNCH_ARGS[@]}" | rg -q '^set_initial_pose:=true$'; then
    echo "[run_localization] Baylands pose-compatible nav map detected: using saved AMCL initial pose" >&2
    LAUNCH_ARGS+=(
      set_initial_pose:=true
      always_reset_initial_pose:=true
      initial_pose_x:="$DEFAULT_BAYLANDS_INITIAL_POSE_X"
      initial_pose_y:="$DEFAULT_BAYLANDS_INITIAL_POSE_Y"
      initial_pose_yaw:="$DEFAULT_BAYLANDS_INITIAL_POSE_YAW"
    )
  fi
fi

if [[ "$WORLD" == baylands* ]] && [ "$LIDAR_MODE" = "3d" ] && [ -n "$SIM_SPAWN_WAYPOINT" ]; then
  spawn_route="$(route_lidar_waypoint_route "$SIM_SPAWN_WAYPOINT")"
  mapfile -t ROUTE_LIDAR_ARGS < <(route_lidar_preset_args "$spawn_route" "$LIDAR_MODE" "${LIDAR_REMAINING_ARGS[@]}")
  if [ "${#ROUTE_LIDAR_ARGS[@]}" -gt 0 ]; then
    echo "[run_localization] Applying route lidar defaults for '$spawn_route': ${ROUTE_LIDAR_ARGS[*]}" >&2
  fi
fi

if [ -n "$LIDAR_POINTCLOUD_TOPIC" ]; then
  LAUNCH_ARGS+=("pointcloud_topic:=$LIDAR_POINTCLOUD_TOPIC")
fi

if [ "$USE_SCAN_RELAY" = "true" ] && [ -n "$SCAN_RELAY_HZ" ]; then
  LAUNCH_ARGS+=("scan_relay_hz:=$SCAN_RELAY_HZ")
fi

if [ "$USE_SCAN_RELAY" = "true" ] && [ -n "$SCAN_RELAY_MAX_AGE_S" ]; then
  LAUNCH_ARGS+=("scan_relay_max_age_s:=$SCAN_RELAY_MAX_AGE_S")
fi

if [ "$USE_SCAN_RELAY" = "true" ] && [ -n "$SCAN_RELAY_START_DELAY_S" ]; then
  LAUNCH_ARGS+=("scan_relay_start_delay_s:=$SCAN_RELAY_START_DELAY_S")
fi

if [ -n "$PARAMS_FILE_OVERRIDE" ]; then
  LAUNCH_ARGS+=("params_file:=$PARAMS_FILE_OVERRIDE")
fi

LAUNCH_ARGS+=("${ROUTE_LIDAR_ARGS[@]}")
LAUNCH_ARGS+=("${LIDAR_REMAINING_ARGS[@]}")

ros2 launch "$LOCAL_LOCALIZATION_LAUNCH" "${LAUNCH_ARGS[@]}"

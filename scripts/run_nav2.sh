#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
TMP_NAV2_PARAMS="$STATE_DIR/nav2_with_map_updates.yaml"
DEFAULT_NAV2_PARAMS="/opt/ros/jazzy/share/clearpath_nav2_demos/config/a200/nav2.yaml"
BAYLANDS_NAV2_PARAMS="$WS_ROOT/src/lrs_halmstad/config/nav2_baylands_large_map.yaml"
LOCAL_NAV2_LAUNCH="$WS_ROOT/src/lrs_halmstad/launch/nav2_with_updates.launch.py"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
BASE_NAV2_PARAMS="$DEFAULT_NAV2_PARAMS"
BAYLANDS_2D_SCAN_RELAY_TOPIC="/a201_0000/sensors/lidar2d_0/scan_relay"

source "$SCRIPT_DIR/lidar_mode_common.sh"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh nav2 [lidar:=2d|3d] [options...]

Start the Nav2 stack for the active simulated UGV. Baylands is detected from
/tmp/halmstad_ws/gazebo_sim.world and uses nav2_baylands_large_map.yaml.

Common options:
  lidar:=2d|3d
  start_collision_monitor:=true|false
  use_scan_relay:=true|false
  scan_relay_stamp_offset_s:=-0.20
  scan_topic:=/topic

Examples:
  ./run.sh nav2 lidar:=3d
  ./run.sh nav2 lidar:=2d start_collision_monitor:=false
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

if [ -f "$SIM_WORLD_FILE" ]; then
  sim_world="$(cat "$SIM_WORLD_FILE" 2>/dev/null || true)"
  if [[ "$sim_world" == baylands* ]]; then
    BASE_NAV2_PARAMS="$BAYLANDS_NAV2_PARAMS"
  fi
fi

if [ "$BASE_NAV2_PARAMS" = "$BAYLANDS_NAV2_PARAMS" ]; then
  echo "[run_nav2] Baylands detected: using large-map Nav2 profile" >&2
fi

lidar_mode_parse_args 3d "$@"
USE_POINTCLOUD_TO_LASERSCAN="false"
USE_SCAN_RELAY="false"
USE_SCAN_RELAY_OVERRIDE=""
SCAN_RELAY_STAMP_OFFSET_S=""
PC2LS_ARGS_IGNORED=()
NAV2_PASSTHROUGH_ARGS=()

for arg in "${LIDAR_REMAINING_ARGS[@]}"; do
  case "$arg" in
    use_scan_relay:=*)
      USE_SCAN_RELAY_OVERRIDE="${arg#use_scan_relay:=}"
      ;;
    scan_relay_stamp_offset_s:=*)
      SCAN_RELAY_STAMP_OFFSET_S="${arg#scan_relay_stamp_offset_s:=}"
      ;;
    use_pointcloud_to_laserscan:=*|pc2ls_*|pointcloud_topic:=*)
      PC2LS_ARGS_IGNORED+=("$arg")
      ;;
    *)
      NAV2_PASSTHROUGH_ARGS+=("$arg")
      ;;
  esac
done
LIDAR_REMAINING_ARGS=("${NAV2_PASSTHROUGH_ARGS[@]}")

if [ "$LIDAR_SCAN_TOPIC" = "$(lidar_mode_scan_topic 3d)" ]; then
  if [ "$USE_SCAN_RELAY_OVERRIDE" != "true" ]; then
    USE_SCAN_RELAY="false"
    echo "[run_nav2] 3D lidar mode: using direct pc2ls scan $LIDAR_SCAN_TOPIC" >&2
  else
    USE_SCAN_RELAY="true"
    if [ -z "$SCAN_RELAY_STAMP_OFFSET_S" ]; then
      SCAN_RELAY_STAMP_OFFSET_S="-0.20"
    fi
    echo "[run_nav2] 3D lidar mode: using restamped scan relay for TF timing stability (${SCAN_RELAY_STAMP_OFFSET_S}s)" >&2
  fi
  if [ "${#PC2LS_ARGS_IGNORED[@]}" -gt 0 ]; then
    echo "[run_nav2] 3D lidar mode: ignoring pc2ls args in Nav2; pass them to localization instead." >&2
  fi
elif [ "$BASE_NAV2_PARAMS" = "$BAYLANDS_NAV2_PARAMS" ] && [ "$LIDAR_MODE" = "2d" ] && [ "$LIDAR_SCAN_TOPIC" = "$(lidar_mode_scan_topic 2d)" ]; then
  if [ "$USE_SCAN_RELAY_OVERRIDE" = "true" ]; then
    USE_SCAN_RELAY="true"
    echo "[run_nav2] Baylands 2D lidar mode: starting Nav2-owned scan relay for $LIDAR_SCAN_TOPIC" >&2
  else
    LIDAR_SCAN_TOPIC="$BAYLANDS_2D_SCAN_RELAY_TOPIC"
    USE_SCAN_RELAY="false"
    echo "[run_nav2] Baylands 2D lidar mode: reusing localization-owned scan relay $LIDAR_SCAN_TOPIC" >&2
  fi
elif [ -n "$USE_SCAN_RELAY_OVERRIDE" ]; then
  USE_SCAN_RELAY="$USE_SCAN_RELAY_OVERRIDE"
fi

mkdir -p "$STATE_DIR"

awk '
  {
    print
    if ($0 ~ /^[[:space:]]*plugin: "nav2_costmap_2d::StaticLayer"$/) {
      match($0, /^ */)
      indent = substr($0, 1, RLENGTH)
      print indent "subscribe_to_updates: true"
    }
  }
' "$BASE_NAV2_PARAMS" > "$TMP_NAV2_PARAMS"

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
  params_file:="$TMP_NAV2_PARAMS"
)

if [ "$USE_SCAN_RELAY" = "true" ] && [ -n "$SCAN_RELAY_STAMP_OFFSET_S" ]; then
  LAUNCH_ARGS+=("scan_relay_stamp_offset_s:=$SCAN_RELAY_STAMP_OFFSET_S")
fi

LAUNCH_ARGS+=("${LIDAR_REMAINING_ARGS[@]}")

ros2 launch "$LOCAL_NAV2_LAUNCH" "${LAUNCH_ARGS[@]}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
TMUX_STATE_DIR="$STATE_DIR/tmux_sessions"
DEFAULT_BAYLANDS_TUNING_MAP="$WS_ROOT/maps/baylands.yaml"

ACTION="start"
PROFILE="standard"
WORLD="baylands"
WAYPOINT="rotundan_0"
NAV2_GOALS="rotundan"
NAV2_GOALS_SOURCE=""
LIDAR="3d"
PAUSE_AFTER_GOAL_S="0.0"
GUI="false"
PERSPECTIVE="NAV2"
START_RQT="false"
START_COLLISION_MONITOR="false"
MUTE_UGV_CAMERA="true"
CLOCK_MODE="direct"
START_OPTIONAL_TELEOP="true"
MAP_PATH="maps/baylands.yaml"
SESSION=""
TMUX_ATTACH="true"
SPAWN_UAV="true"
UAV_CAMERA_UPDATE_RATE="10"
START_CAMERA_TRACKER="false"
WITH_FOLLOW="true"
REBUILD="false"
REBUILD_DONE="false"
DRY_RUN="false"
REALIGN="true"
FOLLOW_PARAMS_SOURCE="$WS_ROOT/src/lrs_halmstad/config/run_follow_defaults.yaml"

# Delay between starting Nav2 and starting the route/follow driver.
FOLLOW_START_DELAY_S="1"
# Wait for localization lifecycle nodes, mainly map_server and AMCL, to become active.
LOCALIZATION_READY_TIMEOUT_S="5"
# Wait for the localization scan topic. In 3D mode this is the pc2ls scan_from_points topic.
LOCALIZATION_SCAN_READY_TIMEOUT_S="5"
# Fixed delay after localization/realign/scan before Nav2 starts. This gives
# AMCL and odom TF time to settle without depending on tf2_echo readiness checks.
NAV2_START_DELAY_S="1"
# Grace period after Ctrl-C before force-killing stack/base processes.
STACK_STOP_GRACE_S="3"
# Fixed settle time after starting gazebo_sim, before localization starts.
GAZEBO_POST_READY_DELAY_S="5"
# Delay before spawning UAV, used so the UGV/Gazebo side has settled first.
SPAWN_PRE_DELAY_S="15"
# Delay after spawning UAV, used before starting localization/Nav2/follow.
SPAWN_POST_DELAY_S="2"
SPAWN_GAZEBO_HELPER_TIMEOUT_S="10"

SESSION_EXPLICIT="false"
PROFILE_EXPLICIT="false"
WAYPOINT_EXPLICIT="false"
NAV2_GOALS_EXPLICIT="false"
LIDAR_EXPLICIT="false"
GUI_EXPLICIT="false"
PERSPECTIVE_EXPLICIT="false"
START_RQT_EXPLICIT="false"
START_COLLISION_MONITOR_EXPLICIT="false"
MUTE_UGV_CAMERA_EXPLICIT="false"
CLOCK_MODE_EXPLICIT="false"
START_OPTIONAL_TELEOP_EXPLICIT="false"
MAP_EXPLICIT="false"
SPAWN_UAV_EXPLICIT="false"
UAV_CAMERA_UPDATE_RATE_EXPLICIT="false"
START_CAMERA_TRACKER_EXPLICIT="false"
WITH_FOLLOW_EXPLICIT="false"
PAUSE_AFTER_GOAL_EXPLICIT="false"
FOLLOW_START_DELAY_EXPLICIT="false"
LOCALIZATION_READY_TIMEOUT_EXPLICIT="false"
LOCALIZATION_SCAN_READY_TIMEOUT_EXPLICIT="false"
NAV2_START_DELAY_EXPLICIT="false"
GAZEBO_POST_READY_DELAY_EXPLICIT="false"
SPAWN_PRE_DELAY_EXPLICIT="false"
SPAWN_POST_DELAY_EXPLICIT="false"

GAZEBO_PANE_ID=""
SPAWN_PANE_ID=""
RQT_PANE_ID=""
LOCALIZATION_PANE_ID=""
NAV2_PANE_ID=""
FOLLOW_PANE_ID=""

CLI_WORLD=""
CLI_PROFILE=""
CLI_WAYPOINT=""
CLI_NAV2_GOALS=""
CLI_NAV2_GOALS_SOURCE=""
CLI_LIDAR=""
CLI_PAUSE_AFTER_GOAL_S=""
CLI_GUI=""
CLI_PERSPECTIVE=""
CLI_START_RQT=""
CLI_START_COLLISION_MONITOR=""
CLI_MUTE_UGV_CAMERA=""
CLI_CLOCK_MODE=""
CLI_START_OPTIONAL_TELEOP=""
CLI_MAP_PATH=""
CLI_SPAWN_UAV=""
CLI_UAV_CAMERA_UPDATE_RATE=""
CLI_START_CAMERA_TRACKER=""
CLI_WITH_FOLLOW=""
CLI_FOLLOW_START_DELAY_S=""
CLI_LOCALIZATION_READY_TIMEOUT_S=""
CLI_LOCALIZATION_SCAN_READY_TIMEOUT_S=""
CLI_NAV2_START_DELAY_S=""
CLI_GAZEBO_POST_READY_DELAY_S=""
CLI_SPAWN_PRE_DELAY_S=""
CLI_SPAWN_POST_DELAY_S=""
CLI_TUNING_LIDAR_ARGS=()
TUNING_LIDAR_ARGS=()

source "$SCRIPT_DIR/baylands_route_lidar_common.sh"
source "$SCRIPT_DIR/baylands_waypoint_common.sh"
source "$SCRIPT_DIR/lidar_mode_common.sh"

# Nav2 tuning is the workflow where we actively test per-route/per-waypoint pc2ls
# settings. Keep this on by default, but allow callers to disable it with:
# BAYLANDS_ROUTE_LIDAR_OVERRIDES=false ./run.sh nav2_tuning ...
: "${BAYLANDS_ROUTE_LIDAR_OVERRIDES:=true}"
export BAYLANDS_ROUTE_LIDAR_OVERRIDES

shell_join() {
  local out=""
  local part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

usage() {
  cat <<EOF
Usage: ./run.sh nav2_tuning [start|restart|stack_stop|follow|route_stop|stop|attach|status]
  [world:=baylands]
  [profile:=standard|minimal]
  [waypoint:=rotundan_0] [nav2_goals:=rotundan] [lidar:=2d|3d]
  [pause_after_goal_s:=0.0]
  [gui:=true|false] [perspective:=NAV2] [start_rqt:=true|false] [start_collision_monitor:=true|false] [map:=maps/baylands.yaml]
  [clock_mode:=guarded|direct]
  [start_optional_teleop:=true|false]
  [spawn_uav:=true|false] [start_camera_tracker:=true|false] [with_route_driver:=true|false] [follow_start_delay_s:=10.0]
  [uav_camera_update_rate:=20]
  [localization_ready_timeout_s:=20] [localization_scan_ready_timeout_s:=20] [nav2_start_delay_s:=10]
  [spawn_pre_delay_s:=20] [spawn_post_delay_s:=5]
  [params_file:=path/to/run_follow_defaults.yaml]
  [rebuild:=true|false]
  [session:=name] [tmux_attach:=true|false] [dry_run:=true|false]

Examples:
  ./run.sh nav2_tuning start
  ./run.sh nav2_tuning start profile:=minimal
  ./run.sh nav2_tuning start waypoint:=rotundan_0
  ./run.sh nav2_tuning restart
  ./run.sh nav2_tuning stack_stop
  ./run.sh nav2_tuning follow
  ./run.sh nav2_tuning route_stop
  ./run.sh nav2_tuning stop
  ./run.sh nav2_tuning attach

Notes:
  - This keeps Gazebo alive in tmux and only restarts the tuning stack.
  - By default it keeps the normal UAV/follow flow alive, lowers the UAV camera update rate.
  - rqt is optional here and defaults to off.
  - Changes to nav2_baylands_large_map.yaml do not require a build; run_nav2.sh reads it from src/.
  - Use rebuild:=true when you change Python/launch/package-installed files and want a symlink build first.
  - RViz is intentionally not managed here. Start it separately with: ./run.sh nav2_rviz lidar:=3d
  - Explicit waypoint:=... starts Gazebo there and slices nav2_goals from that waypoint onward.
  - Startup is sequenced: Gazebo pane start -> optional UAV spawn -> localization -> realign_yaw -> Nav2/route driver.
EOF
}

default_session() {
  printf 'halmstad-%s-nav2-tuning\n' "$WORLD"
}

update_session_default() {
  if [ "$SESSION_EXPLICIT" != "true" ]; then
    SESSION="$(default_session)"
  fi
}

tmux_has_session() {
  tmux has-session -t "$SESSION" 2>/dev/null
}

session_safe() {
  printf '%s' "$SESSION" | tr -c 'A-Za-z0-9_.-' '_'
}

session_state_file() {
  printf '%s/%s.env\n' "$TMUX_STATE_DIR" "$(session_safe)"
}

write_state() {
  mkdir -p "$TMUX_STATE_DIR"
  {
    printf 'SESSION=%q\n' "$SESSION"
    printf 'PROFILE=%q\n' "$PROFILE"
    printf 'WORLD=%q\n' "$WORLD"
    printf 'WAYPOINT=%q\n' "$WAYPOINT"
    printf 'NAV2_GOALS=%q\n' "$NAV2_GOALS"
    printf 'NAV2_GOALS_SOURCE=%q\n' "$NAV2_GOALS_SOURCE"
    printf 'LIDAR=%q\n' "$LIDAR"
    printf 'PAUSE_AFTER_GOAL_S=%q\n' "$PAUSE_AFTER_GOAL_S"
    printf 'GUI=%q\n' "$GUI"
    printf 'PERSPECTIVE=%q\n' "$PERSPECTIVE"
    printf 'START_RQT=%q\n' "$START_RQT"
    printf 'START_COLLISION_MONITOR=%q\n' "$START_COLLISION_MONITOR"
    printf 'MUTE_UGV_CAMERA=%q\n' "$MUTE_UGV_CAMERA"
    printf 'CLOCK_MODE=%q\n' "$CLOCK_MODE"
    printf 'START_OPTIONAL_TELEOP=%q\n' "$START_OPTIONAL_TELEOP"
    printf 'MAP_PATH=%q\n' "$MAP_PATH"
    printf 'SPAWN_UAV=%q\n' "$SPAWN_UAV"
    printf 'UAV_CAMERA_UPDATE_RATE=%q\n' "$UAV_CAMERA_UPDATE_RATE"
    printf 'START_CAMERA_TRACKER=%q\n' "$START_CAMERA_TRACKER"
    printf 'WITH_FOLLOW=%q\n' "$WITH_FOLLOW"
    printf 'FOLLOW_START_DELAY_S=%q\n' "$FOLLOW_START_DELAY_S"
    printf 'LOCALIZATION_READY_TIMEOUT_S=%q\n' "$LOCALIZATION_READY_TIMEOUT_S"
    printf 'LOCALIZATION_SCAN_READY_TIMEOUT_S=%q\n' "$LOCALIZATION_SCAN_READY_TIMEOUT_S"
    printf 'NAV2_START_DELAY_S=%q\n' "$NAV2_START_DELAY_S"
    printf 'GAZEBO_POST_READY_DELAY_S=%q\n' "$GAZEBO_POST_READY_DELAY_S"
    printf 'SPAWN_PRE_DELAY_S=%q\n' "$SPAWN_PRE_DELAY_S"
    printf 'SPAWN_POST_DELAY_S=%q\n' "$SPAWN_POST_DELAY_S"
    printf 'TUNING_LIDAR_ARGS=('
    local arg=""
    for arg in "${TUNING_LIDAR_ARGS[@]}"; do
      printf '%q ' "$arg"
    done
    printf ')\n'
    printf 'GAZEBO_PANE_ID=%q\n' "$GAZEBO_PANE_ID"
    printf 'SPAWN_PANE_ID=%q\n' "$SPAWN_PANE_ID"
    printf 'RQT_PANE_ID=%q\n' "$RQT_PANE_ID"
    printf 'LOCALIZATION_PANE_ID=%q\n' "$LOCALIZATION_PANE_ID"
    printf 'NAV2_PANE_ID=%q\n' "$NAV2_PANE_ID"
    printf 'FOLLOW_PANE_ID=%q\n' "$FOLLOW_PANE_ID"
  } > "$(session_state_file)"
}

load_state() {
  local state_file
  state_file="$(session_state_file)"
  if [ ! -f "$state_file" ]; then
    echo "No nav2 tuning session state found for $SESSION" >&2
    echo "Start it first with: ./run.sh nav2_tuning start" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  source "$state_file"
}

pane_exists() {
  local pane_id="$1"
  [ -n "$pane_id" ] || return 1
  tmux list-panes -a -t "$SESSION" -F '#{pane_id}' 2>/dev/null | grep -Fxq "$pane_id"
}

signal_processes_by_pattern() {
  local label="$1"
  local pattern="$2"
  local matched=""

  matched="$(pgrep -a -f "$pattern" 2>/dev/null || true)"
  [ -n "$matched" ] || return 1

  echo "[nav2_tuning] Fallback cleanup matched $label"
  # printf '%s\n' "$matched"

  if [ "$DRY_RUN" != "true" ]; then
    pkill -INT -f "$pattern" 2>/dev/null || true
    sleep 1
    pkill -TERM -f "$pattern" 2>/dev/null || true
    sleep 1
    pkill -KILL -f "$pattern" 2>/dev/null || true
  fi
  return 0
}

signal_named_nodes() {
  local label="$1"
  local names_regex="$2"
  signal_processes_by_pattern "$label" "__node:=($names_regex)(\\s|$)"
}

send_ctrl_c_pane() {
  local pane_id="$1"
  local label="$2"
  if pane_exists "$pane_id"; then
    echo "[nav2_tuning] Stopping $label"
    tmux send-keys -t "$pane_id" C-c
  fi
}

send_pane_command() {
  local pane_id="$1"
  shift
  local line
  line="$(shell_join "$@")"
  tmux send-keys -t "$pane_id" C-c
  tmux send-keys -t "$pane_id" "clear" C-m
  tmux send-keys -t "$pane_id" "$line" C-m
}

run_ros_probe() {
  local timeout_s="$1"
  shift
  bash -lc "set +u; source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source \"$WS_ROOT/install/setup.bash\" >/dev/null 2>&1; source \"$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash\" >/dev/null 2>&1; set -u; timeout ${timeout_s}s $*"
}

gazebo_cmd() {
  local cmd=(
    ./run.sh gazebo_sim "$WORLD"
    "gui:=$GUI"
    "waypoint:=$WAYPOINT"
    "clock_mode:=$CLOCK_MODE"
  )
  echo "$(shell_join "${cmd[@]}")"
}

spawn_cmd() {
  local cmd=(
    ./run.sh spawn_uav "$WORLD"
    "camera_update_rate:=$UAV_CAMERA_UPDATE_RATE"
  )
  echo "$(shell_join "${cmd[@]}")"
}

rqt_cmd() {
  local cmd=(./run.sh rqt_perspective "$PERSPECTIVE")
  echo "$(shell_join "${cmd[@]}")"
}

effective_map_path() {
  if [ -n "$MAP_PATH" ]; then
    printf '%s\n' "$MAP_PATH"
  elif [[ "$WORLD" == baylands* ]]; then
    printf '%s\n' "$DEFAULT_BAYLANDS_TUNING_MAP"
  fi
}

resolve_first_waypoint_for_nav2_goals() {
  local goals="$1"
  python3 - "$WS_ROOT" "$goals" <<'PY'
import csv
import sys
from pathlib import Path

import yaml

ws_root = Path(sys.argv[1])
goals = sys.argv[2]
config_dir = ws_root / "src" / "lrs_halmstad" / "config"
csv_path = ws_root / "maps" / "waypoints_baylands_groups.csv"


def goal_name(value: str) -> str:
    name = Path(value).name
    if name.endswith(".yaml"):
        name = name[:-5]
    if name.startswith("baylands_waypoints_"):
        name = name[len("baylands_waypoints_") :]
    if name.endswith("_rviz"):
        name = name[:-5]
    return name


def candidate_yaml_files(value: str):
    raw = Path(value)
    if raw.is_absolute() or "/" in value:
        yield raw if raw.is_absolute() else ws_root / raw
        return

    yield config_dir / value
    yield config_dir / "baylands_waypoints" / value
    if not value.endswith(".yaml"):
        yield config_dir / f"{value}.yaml"
        yield config_dir / "baylands_waypoints" / f"{value}.yaml"
        yield config_dir / "baylands_waypoints" / f"baylands_waypoints_{value}.yaml"


for path in candidate_yaml_files(goals):
    if not path.is_file():
        continue
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    waypoints = data.get("waypoints") or []
    if waypoints:
        first = waypoints[0].get("name")
        if first:
            print(first)
            raise SystemExit(0)

group = goal_name(goals)
if csv_path.is_file():
    with csv_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("group") == group and row.get("place"):
                print(row["place"])
                raise SystemExit(0)

raise SystemExit(1)
PY
}

resolve_nav2_goals_for_waypoint() {
  local waypoint="$1"
  python3 - "$WS_ROOT" "$waypoint" <<'PY'
import csv
import re
import sys
from pathlib import Path

import yaml

ws_root = Path(sys.argv[1])
waypoint_name = sys.argv[2]
config_dir = ws_root / "src" / "lrs_halmstad" / "config"
waypoint_dir = config_dir / "baylands_waypoints"
csv_path = ws_root / "maps" / "waypoints_baylands_groups.csv"

if csv_path.is_file():
    with csv_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("place") == waypoint_name and row.get("group"):
                print(row["group"])
                raise SystemExit(0)

if waypoint_dir.is_dir():
    for path in sorted(waypoint_dir.glob("baylands_waypoints_*.yaml")):
        if path.name.endswith("_rviz.yaml"):
            continue
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        group = str(data.get("group") or "").strip()
        for waypoint in data.get("waypoints") or []:
            if isinstance(waypoint, dict) and waypoint.get("name") == waypoint_name:
                if group:
                    print(group)
                    raise SystemExit(0)
                name = path.stem
                if name.startswith("baylands_waypoints_"):
                    print(name[len("baylands_waypoints_"):])
                    raise SystemExit(0)

match = re.match(r"^(.+)_\d+$", waypoint_name)
if match:
    print(match.group(1))
    raise SystemExit(0)

raise SystemExit(1)
PY
}

safe_slug() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
}

apply_waypoint_default_nav2_goals() {
  if [ "$WAYPOINT_EXPLICIT" != "true" ]; then
    return 0
  fi
  if [ "$NAV2_GOALS_EXPLICIT" = "true" ]; then
    return 0
  fi
  if [[ "$WORLD" != baylands* ]]; then
    return 0
  fi

  local resolved=""
  resolved="$(resolve_nav2_goals_for_waypoint "$WAYPOINT" 2>/dev/null || true)"
  if [ -z "$resolved" ]; then
    return 0
  fi

  if [ "$NAV2_GOALS" != "$resolved" ]; then
    echo "[nav2_tuning] waypoint:=$WAYPOINT implies nav2_goals:=$resolved"
  fi
  NAV2_GOALS="$resolved"
  NAV2_GOALS_SOURCE="$resolved"
}

apply_nav2_goals_default_waypoint() {
  if [ "$WAYPOINT_EXPLICIT" = "true" ]; then
    return 0
  fi
  if [ "$NAV2_GOALS_EXPLICIT" != "true" ]; then
    return 0
  fi

  local resolved=""
  resolved="$(resolve_first_waypoint_for_nav2_goals "$NAV2_GOALS" 2>/dev/null || true)"
  if [ -z "$resolved" ]; then
    echo "Could not resolve first waypoint for nav2_goals: $NAV2_GOALS" >&2
    exit 2
  fi
  WAYPOINT="$resolved"
}

apply_nav2_goals_waypoint_slice() {
  if [ "$WAYPOINT_EXPLICIT" != "true" ]; then
    return 0
  fi
  if [ -z "$WAYPOINT" ] || [ -z "$NAV2_GOALS" ]; then
    return 0
  fi
  if [[ "$WORLD" != baylands* ]]; then
    return 0
  fi

  if [ "$NAV2_GOALS_EXPLICIT" = "true" ] || [ -z "$NAV2_GOALS_SOURCE" ]; then
    NAV2_GOALS_SOURCE="$NAV2_GOALS"
  fi

  local source_path=""
  source_path="$(baylands_route_yaml_path "$NAV2_GOALS_SOURCE")"
  if [ ! -f "$source_path" ]; then
    echo "[nav2_tuning] Warning: cannot slice nav2_goals '$NAV2_GOALS_SOURCE'; missing $source_path" >&2
    return 0
  fi

  local slice_dir="$STATE_DIR/nav2_tuning_goal_slices"
  local source_label waypoint_label slice_path result
  source_label="$(safe_slug "$(basename "$NAV2_GOALS_SOURCE" .yaml)")"
  waypoint_label="$(safe_slug "$WAYPOINT")"
  slice_path="$slice_dir/$(session_safe)_${source_label}_from_${waypoint_label}.yaml"

  if ! result="$(python3 - "$source_path" "$WAYPOINT" "$slice_path" "$DRY_RUN" <<'PY'
import os
import sys
from pathlib import Path

import yaml

source_path = Path(sys.argv[1])
start_name = sys.argv[2]
out_path = Path(sys.argv[3])
dry_run = sys.argv[4].lower() == "true"

data = yaml.safe_load(source_path.read_text(encoding="utf-8")) or {}
waypoints = data.get("waypoints") or []
if not isinstance(waypoints, list):
    raise SystemExit(f"{source_path} does not contain a waypoint list")

start_index = None
for index, waypoint in enumerate(waypoints):
    if isinstance(waypoint, dict) and str(waypoint.get("name", "")).strip() == start_name:
        start_index = index
        break

if start_index is None:
    raise SystemExit(f"waypoint '{start_name}' is not present in {source_path}")

out_data = dict(data)
out_data["source_goal_sequence_file"] = str(source_path)
out_data["source_start_waypoint"] = start_name
out_data["source_start_index"] = start_index
out_data["waypoints"] = waypoints[start_index:]

if not dry_run:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(out_data, sort_keys=False), encoding="utf-8")

print(f"{out_path}\t{start_index}\t{len(waypoints)}\t{len(out_data['waypoints'])}")
PY
)"; then
    echo "[nav2_tuning] Warning: $result" >&2
    return 0
  fi

  local resolved_slice start_index total_count slice_count
  IFS=$'\t' read -r resolved_slice start_index total_count slice_count <<< "$result"
  NAV2_GOALS="$resolved_slice"
  echo "[nav2_tuning] Using Nav2 goal slice from $WAYPOINT: $slice_count/$total_count waypoints starting at index $start_index"
}

localization_cmd() {
  local effective_map=""
  effective_map="$(effective_map_path)"
  local cmd=(./run.sh localization "$WORLD" "lidar:=$LIDAR")
  if [ -n "$effective_map" ]; then
    cmd+=("$effective_map")
  fi
  cmd+=("${TUNING_LIDAR_ARGS[@]}")
  echo "$(shell_join "${cmd[@]}")"
}

nav2_cmd() {
  local cmd=(./run.sh nav2 "lidar:=$LIDAR" "start_collision_monitor:=$START_COLLISION_MONITOR")
  echo "$(shell_join "${cmd[@]}")"
}

rviz_cmd() {
  local cmd=(./run.sh nav2_rviz "lidar:=$LIDAR")
  echo "$(shell_join "${cmd[@]}")"
}

follow_cmd() {
  local follow_params
  follow_params="$(follow_params_file)"
  local cmd=(
    ./run.sh 1to1_follow "$WORLD"
    "waypoint:=$WAYPOINT"
    "nav2_goals:=$NAV2_GOALS"
    "params_file:=$follow_params"
  )
  if [ "$SPAWN_UAV" != "true" ]; then
    local camera_tracker_value="false"
    if [ "$START_CAMERA_TRACKER_EXPLICIT" = "true" ]; then
      camera_tracker_value="$START_CAMERA_TRACKER"
    fi
    cmd+=(
      "start_uav_simulator:=false"
      "start_uav_follow:=false"
      "start_camera_tracker:=$camera_tracker_value"
      "require_uav_actual_before_motion:=false"
    )
  elif [ "$START_CAMERA_TRACKER_EXPLICIT" = "true" ]; then
    cmd+=("start_camera_tracker:=$START_CAMERA_TRACKER")
  fi
  echo "$(shell_join "${cmd[@]}")"
}

follow_extra_args() {
  if [ "$SPAWN_UAV" != "true" ]; then
    local camera_tracker_value="false"
    if [ "$START_CAMERA_TRACKER_EXPLICIT" = "true" ]; then
      camera_tracker_value="$START_CAMERA_TRACKER"
    fi
    printf '%s\n' \
      "start_uav_simulator:=false" \
      "start_uav_follow:=false" \
      "start_camera_tracker:=$camera_tracker_value" \
      "require_uav_actual_before_motion:=false"
  elif [ "$START_CAMERA_TRACKER_EXPLICIT" = "true" ]; then
    printf '%s\n' "start_camera_tracker:=$START_CAMERA_TRACKER"
  fi
}

follow_params_file() {
  printf '%s/nav2_tuning_follow_params.yaml\n' "$STATE_DIR"
}

write_follow_params_file() {
  local output_file
  output_file="$(follow_params_file)"
  mkdir -p "$STATE_DIR"
  python3 - "$FOLLOW_PARAMS_SOURCE" "$output_file" "$PAUSE_AFTER_GOAL_S" "$(route_lidar_config_path)" "$(route_lidar_pc2ls_node_name)" <<'PY'
import sys
from pathlib import Path
import yaml

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
pause_after_goal_s = float(sys.argv[3])
lidar_settings_file = Path(sys.argv[4])
pc2ls_node_name = sys.argv[5]

data = yaml.safe_load(src.read_text(encoding="utf-8"))
params = data.setdefault("ugv_nav2_driver", {}).setdefault("ros__parameters", {})
params["pause_after_goal_s"] = pause_after_goal_s
params["lidar_settings_file"] = str(lidar_settings_file) if lidar_settings_file.is_file() else "disabled"
params["pc2ls_node_name"] = pc2ls_node_name

dst.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY
}

create_session() {
  if [ "$DRY_RUN" = "true" ]; then
    echo "[nav2_tuning] Would create tmux session: $SESSION"
    echo "[sim/gazebo] $(gazebo_cmd)"
    if [ "$SPAWN_UAV" = "true" ]; then
      echo "[sim/spawn]  $(spawn_cmd)"
    else
      echo "[sim/spawn]  <disabled>"
    fi
    if [ "$START_RQT" = "true" ]; then
      echo "[sim/rqt]    $(rqt_cmd)"
    else
      echo "[sim/rqt]    <disabled>"
    fi
    echo "[stack/localization] $(localization_cmd)"
    echo "[stack/nav2]         $(nav2_cmd)"
    echo "[stack/rviz]         <not managed: $(rviz_cmd)>"
    if [ "$WITH_FOLLOW" = "true" ]; then
      echo "[stack/route]        $(follow_cmd)"
    fi
    return 0
  fi

  if tmux_has_session; then
    echo "tmux session already exists: $SESSION" >&2
    exit 1
  fi

  tmux new-session -d -s "$SESSION" -n sim
  GAZEBO_PANE_ID="$(tmux display-message -p -t "$SESSION:sim.0" '#{pane_id}')"
  SPAWN_PANE_ID="$(tmux split-window -d -P -F '#{pane_id}' -t "$GAZEBO_PANE_ID" -h -l 50%)"
  RQT_PANE_ID="$(tmux split-window -d -P -F '#{pane_id}' -t "$SPAWN_PANE_ID" -v -l 50%)"

  tmux new-window -d -t "$SESSION" -n stack
  LOCALIZATION_PANE_ID="$(tmux display-message -p -t "$SESSION:stack.0" '#{pane_id}')"
  NAV2_PANE_ID="$(tmux split-window -d -P -F '#{pane_id}' -t "$LOCALIZATION_PANE_ID" -v -l 66%)"
  FOLLOW_PANE_ID="$(tmux split-window -d -P -F '#{pane_id}' -t "$NAV2_PANE_ID" -v -l 50%)"
  tmux select-layout -t "$SESSION:stack" even-vertical >/dev/null

  tmux select-pane -t "$GAZEBO_PANE_ID" -T gazebo
  tmux select-pane -t "$SPAWN_PANE_ID" -T spawn
  tmux select-pane -t "$RQT_PANE_ID" -T rqt
  tmux select-pane -t "$LOCALIZATION_PANE_ID" -T localization
  tmux select-pane -t "$NAV2_PANE_ID" -T nav2
  tmux select-pane -t "$FOLLOW_PANE_ID" -T follow

  write_state
}

start_base_processes() {
  if [ "$DRY_RUN" = "true" ]; then
    echo "[nav2_tuning] Would start base processes in order:"
    if [ "$REBUILD" = "true" ]; then
      echo "  0. colcon build --symlink-install"
    fi
    echo "  1. $(gazebo_cmd) rebuild:=false"
    echo "  2. wait ${GAZEBO_POST_READY_DELAY_S}s after starting gazebo_sim"
    if [ "$SPAWN_UAV" = "true" ]; then
      echo "  4. wait ${SPAWN_PRE_DELAY_S}s"
      echo "  5. $(spawn_cmd)"
      echo "  6. wait ${SPAWN_POST_DELAY_S}s"
      if [ "$START_RQT" = "true" ]; then
        echo "  7. $(rqt_cmd)"
      fi
    else
      if [ "$START_RQT" = "true" ]; then
        echo "  4. $(rqt_cmd)"
      fi
    fi
    return 0
  fi

  maybe_rebuild
  send_pane_command "$GAZEBO_PANE_ID" ./run.sh gazebo_sim "$WORLD" "gui:=$GUI" "waypoint:=$WAYPOINT" "clock_mode:=$CLOCK_MODE" "rebuild:=false"
  if [ "$GAZEBO_POST_READY_DELAY_S" != "0" ] && [ "$GAZEBO_POST_READY_DELAY_S" != "0.0" ]; then
    echo "[nav2_tuning] Waiting ${GAZEBO_POST_READY_DELAY_S}s after starting gazebo_sim"
    sleep "$GAZEBO_POST_READY_DELAY_S"
  fi
  cleanup_optional_teleop_nodes
  if [ "$SPAWN_UAV" = "true" ]; then
    if [ "$SPAWN_PRE_DELAY_S" != "0" ] && [ "$SPAWN_PRE_DELAY_S" != "0.0" ]; then
      echo "[nav2_tuning] Waiting ${SPAWN_PRE_DELAY_S}s before spawn"
      sleep "$SPAWN_PRE_DELAY_S"
    fi
    send_pane_command "$SPAWN_PANE_ID" ./run.sh spawn_uav "$WORLD" "camera_update_rate:=$UAV_CAMERA_UPDATE_RATE"
    if [ "$SPAWN_POST_DELAY_S" != "0" ] && [ "$SPAWN_POST_DELAY_S" != "0.0" ]; then
      echo "[nav2_tuning] Waiting ${SPAWN_POST_DELAY_S}s after spawn"
      sleep "$SPAWN_POST_DELAY_S"
    fi
  else
    send_pane_command "$SPAWN_PANE_ID" bash -lc "printf 'UAV spawn disabled for nav2_tuning\\n'; exec bash"
  fi
  if [ "$START_RQT" = "true" ]; then
    send_pane_command "$RQT_PANE_ID" ./run.sh rqt_perspective "$PERSPECTIVE"
  else
    send_pane_command "$RQT_PANE_ID" bash -lc "printf 'rqt disabled for nav2_tuning\\n'; exec bash"
  fi
}

wait_for_localization_ready() {
  local waited=0
  echo "[nav2_tuning] Waiting for localization readiness..."
  while [ "$waited" -lt "$LOCALIZATION_READY_TIMEOUT_S" ]; do
    if bash -lc "set +u; source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source \"$WS_ROOT/install/setup.bash\" >/dev/null 2>&1; set -u; lifecycle_state_matches() { local node=\"\$1\"; local cli_regex=\"\$2\"; local service_regex=\"\$3\"; ros2 lifecycle get \"\$node\" 2>/dev/null | grep -Eq \"\$cli_regex\" && return 0; timeout 3s ros2 service call \"\${node}/get_state\" lifecycle_msgs/srv/GetState \"{}\" 2>/dev/null | grep -Eq \"\$service_regex\"; }; lifecycle_state_matches /a201_0000/map_server 'active \\[3\\]' \"id=3|label='active'\" && lifecycle_state_matches /a201_0000/amcl 'active \\[3\\]' \"id=3|label='active'\""; then
      echo "[nav2_tuning] Localization ready"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "[nav2_tuning] Timed out waiting for localization" >&2
  return 1
}

wait_for_localization_scan_ready() {
  local waited=0
  local scan_topic=""
  scan_topic="$(lidar_mode_scan_topic "$LIDAR" 2>/dev/null || true)"
  if [ -z "$scan_topic" ]; then
    return 0
  fi

  echo "[nav2_tuning] Waiting for localization scan on $scan_topic..."
  while [ "$waited" -lt "$LOCALIZATION_SCAN_READY_TIMEOUT_S" ]; do
    if bash -lc "set +u; source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source \"$WS_ROOT/install/setup.bash\" >/dev/null 2>&1; source \"$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash\" >/dev/null 2>&1; set -u; timeout 2s ros2 topic echo --no-daemon --once \"$scan_topic\" >/dev/null 2>&1"; then
      echo "[nav2_tuning] Localization scan ready"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo "[nav2_tuning] Warning: no localization scan on $scan_topic within ${LOCALIZATION_SCAN_READY_TIMEOUT_S}s; starting Nav2 anyway" >&2
  return 1
}

delay_before_nav2_start() {
  if [ "$NAV2_START_DELAY_S" = "0" ] || [ "$NAV2_START_DELAY_S" = "0.0" ]; then
    return 0
  fi
  echo "[nav2_tuning] Waiting ${NAV2_START_DELAY_S}s before Nav2 start"
  sleep "$NAV2_START_DELAY_S"
}

maybe_rebuild() {
  if [ "$REBUILD" != "true" ]; then
    return 0
  fi
  if [ "$REBUILD_DONE" = "true" ]; then
    return 0
  fi
  echo "[nav2_tuning] Rebuilding with symlink install."
  (
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
    cd "$WS_ROOT"
    colcon build --symlink-install
  )
  REBUILD_DONE="true"
}

realign_yaw() {
  if [ "$REALIGN" != "true" ]; then
    return 0
  fi
  echo "[nav2_tuning] Realigning yaw at waypoint $WAYPOINT"
  bash "$SCRIPT_DIR/run_realign_yaw.sh" "$WORLD" "waypoint:=$WAYPOINT" "with_uav:=$SPAWN_UAV"
}

apply_startup_waypoint_lidar_settings() {
  if [ "$LIDAR" != "3d" ]; then
    return 0
  fi
  if [ -z "$WAYPOINT" ]; then
    return 0
  fi

  local -a lidar_args=()
  mapfile -t lidar_args < <(route_lidar_waypoint_args "$WAYPOINT")
  if [ "${#lidar_args[@]}" -eq 0 ]; then
    return 0
  fi

  local -a pending_lidar_args=()
  local arg prefix desired current
  for arg in "${lidar_args[@]}"; do
    prefix="${arg%%:=*}:="
    desired="${arg#*:=}"
    current="$(arg_value_for_prefix "$prefix" "${TUNING_LIDAR_ARGS[@]}" 2>/dev/null || true)"
    if [ -n "$current" ] && [ "$current" = "$desired" ]; then
      continue
    fi
    pending_lidar_args+=("$arg")
  done

  if [ "${#pending_lidar_args[@]}" -eq 0 ]; then
    echo "[nav2_tuning] Startup waypoint '$WAYPOINT' lidar settings already applied by localization"
    return 0
  fi

  route_lidar_apply_pc2ls_args "startup waypoint '$WAYPOINT'" "20" "${pending_lidar_args[@]}" || true
}

stop_stack_processes() {
  if [ "$DRY_RUN" = "true" ]; then
    echo "[nav2_tuning] Would stop tuning stack for session $SESSION"
    echo "[nav2_tuning] Targets: route, nav2, localization, scan helpers, Nav2 child nodes, twist_mux"
    return 0
  fi

  send_ctrl_c_pane "$FOLLOW_PANE_ID" route
  send_ctrl_c_pane "$NAV2_PANE_ID" nav2
  send_ctrl_c_pane "$LOCALIZATION_PANE_ID" localization
  sleep "$STACK_STOP_GRACE_S"

  signal_processes_by_pattern "realign_yaw helper" 'scripts/run_realign_yaw\.sh' || true
  signal_processes_by_pattern "route helper" 'scripts/run_1to1_follow\.sh' || true
  signal_processes_by_pattern "route launch" 'ros2 launch lrs_halmstad run_1to1_follow\.launch\.py' || true
  signal_processes_by_pattern "route launch" 'ros2 launch lrs_halmstad run_follow\.launch\.py' || true
  signal_processes_by_pattern "route launch" 'ros2 launch .*/run_follow\.launch\.py' || true

  signal_processes_by_pattern "Nav2 helper" 'scripts/run_nav2\.sh' || true
  signal_processes_by_pattern "Nav2 launch" 'ros2 launch clearpath_nav2_demos nav2\.launch\.py' || true
  signal_processes_by_pattern "Nav2 launch" 'ros2 launch .*/nav2_with_updates\.launch\.py' || true
  signal_processes_by_pattern "Nav2 child processes" '/opt/ros/[^/]+/lib/nav2_' || true
  signal_processes_by_pattern "teleop base launch" 'ros2 launch clearpath_control teleop_base\.launch\.py' || true
  signal_processes_by_pattern "twist mux process" '(^|/)twist_mux($| )' || true

  signal_processes_by_pattern "localization helper" 'scripts/run_localization\.sh' || true
  signal_processes_by_pattern "localization launch" 'ros2 launch clearpath_nav2_demos localization\.launch\.py' || true
  signal_processes_by_pattern "localization launch" 'ros2 launch .*/localization_with_params\.launch\.py' || true
  signal_processes_by_pattern "pointcloud_to_laserscan process" '(^|/)pointcloud_to_laserscan_node($| )' || true
  signal_processes_by_pattern "latest_scan_relay process" '(^|/)latest_scan_relay($| )' || true

  signal_named_nodes "tuning stack nodes" 'amcl|map_server|planner_server|controller_server|collision_monitor|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|route_server|docking_server|lifecycle_manager_localization|lifecycle_manager_navigation|pointcloud_to_laserscan|latest_scan_relay|ugv_nav2_driver|ugv_amcl_to_odom|ugv_amcl_to_platform_odom|ugv_amcl_to_platform_filtered_odom|ugv_platform_odom_to_tf|ugv_ground_truth_bridge|follow_uav|follow_uav_odom|camera_tracker|twist_mux|twist_server_node' || true
}

stop_base_processes() {
  if [ "$DRY_RUN" = "true" ]; then
    echo "[nav2_tuning] Would stop base processes for session $SESSION"
    echo "[nav2_tuning] Targets: rqt, spawn, gazebo, ros_gz_bridge, clock bridge/guard"
    return 0
  fi

  send_ctrl_c_pane "$RQT_PANE_ID" rqt
  send_ctrl_c_pane "$SPAWN_PANE_ID" spawn
  send_ctrl_c_pane "$GAZEBO_PANE_ID" gazebo
  sleep "$STACK_STOP_GRACE_S"

  signal_processes_by_pattern "spawn helper" 'scripts/run_spawn_uav\.sh' || true
  signal_processes_by_pattern "spawn launch" 'ros2 launch lrs_halmstad spawn_uav_1to1\.launch\.py' || true
  signal_named_nodes "base child nodes" 'uav_simulator|clock_bridge|clock_guard|ugv_ground_truth_bridge|twist_server_node' || true
  signal_processes_by_pattern "ros_gz_bridge" 'ros_gz_bridge/(bridge_node|parameter_bridge|image_bridge)' || true
  signal_processes_by_pattern "interactive marker server" 'interactive_marker_twist_server/marker_server --ros-args -r __node:=twist_server_node' || true
  signal_processes_by_pattern "Gazebo sim" '(^|/)gz sim($| )' || true
}

restart_stack() {
  if [ "$DRY_RUN" = "true" ]; then
    echo "[nav2_tuning] Would restart localization/nav2$( [ "$WITH_FOLLOW" = "true" ] && printf '/route' )"
    echo "[localization] $(localization_cmd)"
    echo "[realign] ./run.sh realign_yaw $WORLD waypoint:=$WAYPOINT"
    echo "[lidar] wait up to ${LOCALIZATION_SCAN_READY_TIMEOUT_S}s for localization scan"
    echo "[nav2-delay] wait ${NAV2_START_DELAY_S}s before Nav2 start"
    echo "[nav2] $(nav2_cmd)"
    echo "[lidar] apply waypoint-specific pc2ls settings for $WAYPOINT after Nav2 start"
    echo "[rviz] <not managed: $(rviz_cmd)>"
    if [ "$WITH_FOLLOW" = "true" ]; then
      echo "[route] $(follow_cmd)"
    fi
    return 0
  fi
  maybe_rebuild
  write_follow_params_file
  stop_stack_processes

  local effective_map=""
  effective_map="$(effective_map_path)"
  if [ -n "$effective_map" ]; then
    send_pane_command "$LOCALIZATION_PANE_ID" ./run.sh localization "$WORLD" "$effective_map" "lidar:=$LIDAR" "${TUNING_LIDAR_ARGS[@]}"
  else
    send_pane_command "$LOCALIZATION_PANE_ID" ./run.sh localization "$WORLD" "lidar:=$LIDAR" "${TUNING_LIDAR_ARGS[@]}"
  fi
  wait_for_localization_ready
  realign_yaw
  wait_for_localization_scan_ready || true
  delay_before_nav2_start
  send_pane_command "$NAV2_PANE_ID" ./run.sh nav2 "lidar:=$LIDAR" "start_collision_monitor:=$START_COLLISION_MONITOR"
  apply_startup_waypoint_lidar_settings

  if [ "$WITH_FOLLOW" = "true" ]; then
    echo "[nav2_tuning] Waiting ${FOLLOW_START_DELAY_S}s before route/follow start"
    sleep "$FOLLOW_START_DELAY_S"
    local -a follow_args=()
    mapfile -t follow_args < <(follow_extra_args)
    echo "[nav2_tuning] Starting route/follow driver"
    send_pane_command "$FOLLOW_PANE_ID" ./run.sh 1to1_follow "$WORLD" "waypoint:=$WAYPOINT" "nav2_goals:=$NAV2_GOALS" "params_file:=$(follow_params_file)" "${follow_args[@]}"
  fi
}

restart_follow_only() {
  if [ "$DRY_RUN" = "true" ]; then
    echo "[nav2_tuning] Would restart route driver"
    echo "[route] $(follow_cmd)"
    return 0
  fi
  write_follow_params_file
  send_ctrl_c_pane "$FOLLOW_PANE_ID" route
  sleep 1
  local -a follow_args=()
  mapfile -t follow_args < <(follow_extra_args)
  send_pane_command "$FOLLOW_PANE_ID" ./run.sh 1to1_follow "$WORLD" "waypoint:=$WAYPOINT" "nav2_goals:=$NAV2_GOALS" "params_file:=$(follow_params_file)" "${follow_args[@]}"
}

stop_route_only() {
  if [ "$DRY_RUN" = "true" ]; then
    echo "[nav2_tuning] Would stop ugv_nav2_driver only"
    return 0
  fi
  signal_named_nodes "route driver" 'ugv_nav2_driver' || true
}

stop_all() {
  if [ "$DRY_RUN" = "true" ]; then
    echo "[nav2_tuning] Would kill tmux session $SESSION"
    return 0
  fi
  if tmux_has_session; then
    stop_stack_processes || true
    sleep 2
    stop_base_processes || true
    sleep 2
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    stop_base_processes || true
  fi
  rm -f "$(session_state_file)"
}

print_status() {
  local map_status=""
  map_status="$(effective_map_path)"
  if [ -z "$map_status" ]; then
    map_status="<default>"
  fi
  cat <<EOF
Session: $SESSION
Profile: $PROFILE
World: $WORLD
Waypoint: $WAYPOINT
Nav2 goals: $NAV2_GOALS
Nav2 goals source: ${NAV2_GOALS_SOURCE:-$NAV2_GOALS}
Lidar: $LIDAR
Pause after goal: $PAUSE_AFTER_GOAL_S
GUI: $GUI
Perspective: $PERSPECTIVE
Start rqt: $START_RQT
Start collision monitor: $START_COLLISION_MONITOR
Clock mode: $CLOCK_MODE
Start optional teleop: $START_OPTIONAL_TELEOP
Map: $map_status
Spawn UAV: $SPAWN_UAV
UAV camera update rate: $UAV_CAMERA_UPDATE_RATE
With route driver: $WITH_FOLLOW
Follow start delay: $FOLLOW_START_DELAY_S
Localization lifecycle timeout: $LOCALIZATION_READY_TIMEOUT_S
Localization scan timeout: $LOCALIZATION_SCAN_READY_TIMEOUT_S
Nav2 start delay: $NAV2_START_DELAY_S
Spawn settle delay: $SPAWN_POST_DELAY_S
EOF
}

apply_profile_defaults() {
  case "$PROFILE" in
    standard|"")
      PROFILE="standard"
      ;;
    minimal)
      if [ "$SPAWN_UAV_EXPLICIT" != "true" ]; then
        SPAWN_UAV="false"
      fi
      if [ "$LIDAR_EXPLICIT" != "true" ]; then
        LIDAR="3d"
      fi
      if [ "$CLOCK_MODE_EXPLICIT" != "true" ]; then
        CLOCK_MODE="direct"
      fi
      if [ "$GUI_EXPLICIT" != "true" ]; then
        GUI="false"
      fi
      if [ "$START_RQT_EXPLICIT" != "true" ]; then
        START_RQT="false"
      fi
      if [ "$MUTE_UGV_CAMERA_EXPLICIT" != "true" ]; then
        MUTE_UGV_CAMERA="true"
      fi
      if [ "$START_OPTIONAL_TELEOP_EXPLICIT" != "true" ]; then
        START_OPTIONAL_TELEOP="false"
      fi
      ;;
    *)
      echo "Unknown profile: $PROFILE" >&2
      usage >&2
      exit 2
      ;;
  esac
}

cleanup_optional_teleop_nodes() {
  if [ "$START_OPTIONAL_TELEOP" = "true" ]; then
    return 0
  fi
  echo "[nav2_tuning] Stopping optional teleop nodes"
  signal_named_nodes "optional teleop nodes" 'joy_node|teleop_twist_joy_node|twist_server_node' || true
}

if [ "$#" -gt 0 ]; then
  case "$1" in
    start|restart|stack_stop|follow|route_stop|stop|attach|status)
      ACTION="$1"
      shift
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown nav2_tuning action: $1" >&2
      echo "Use an explicit action, for example: ./run.sh nav2_tuning start $1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

update_session_default

for arg in "$@"; do
  case "$arg" in
    world:=*)
      WORLD="${arg#world:=}"
      ;;
    profile:=*)
      PROFILE="${arg#profile:=}"
      PROFILE_EXPLICIT="true"
      ;;
    waypoint:=*)
      WAYPOINT="${arg#waypoint:=}"
      WAYPOINT_EXPLICIT="true"
      ;;
    nav2_goals:=*)
      NAV2_GOALS="${arg#nav2_goals:=}"
      NAV2_GOALS_EXPLICIT="true"
      ;;
    lidar:=*)
      LIDAR="${arg#lidar:=}"
      LIDAR_EXPLICIT="true"
      ;;
    pause_after_goal_s:=*)
      PAUSE_AFTER_GOAL_S="${arg#pause_after_goal_s:=}"
      PAUSE_AFTER_GOAL_EXPLICIT="true"
      ;;
    gui:=*)
      GUI="${arg#gui:=}"
      GUI_EXPLICIT="true"
      ;;
    perspective:=*)
      PERSPECTIVE="${arg#perspective:=}"
      PERSPECTIVE_EXPLICIT="true"
      ;;
    start_rqt:=*)
      START_RQT="${arg#start_rqt:=}"
      START_RQT_EXPLICIT="true"
      ;;
    start_collision_monitor:=*)
      START_COLLISION_MONITOR="${arg#start_collision_monitor:=}"
      START_COLLISION_MONITOR_EXPLICIT="true"
      ;;
    mute_ugv_camera:=*)
      MUTE_UGV_CAMERA="${arg#mute_ugv_camera:=}"
      MUTE_UGV_CAMERA_EXPLICIT="true"
      ;;
    clock_mode:=*)
      CLOCK_MODE="${arg#clock_mode:=}"
      CLOCK_MODE_EXPLICIT="true"
      ;;
    start_optional_teleop:=*)
      START_OPTIONAL_TELEOP="${arg#start_optional_teleop:=}"
      START_OPTIONAL_TELEOP_EXPLICIT="true"
      ;;
    map:=*)
      MAP_PATH="${arg#map:=}"
      MAP_EXPLICIT="true"
      ;;
    spawn_uav:=*|with_uav:=*)
      SPAWN_UAV="${arg#*=}"
      SPAWN_UAV_EXPLICIT="true"
      ;;
    uav_camera_update_rate:=*)
      UAV_CAMERA_UPDATE_RATE="${arg#uav_camera_update_rate:=}"
      UAV_CAMERA_UPDATE_RATE_EXPLICIT="true"
      ;;
    start_camera_tracker:=*)
      START_CAMERA_TRACKER="${arg#start_camera_tracker:=}"
      START_CAMERA_TRACKER_EXPLICIT="true"
      ;;
    with_route_driver:=*)
      WITH_FOLLOW="${arg#with_route_driver:=}"
      WITH_FOLLOW_EXPLICIT="true"
      ;;
    with_follow:=*)
      WITH_FOLLOW="${arg#with_follow:=}"
      WITH_FOLLOW_EXPLICIT="true"
      ;;
    follow_start_delay_s:=*)
      FOLLOW_START_DELAY_S="${arg#follow_start_delay_s:=}"
      FOLLOW_START_DELAY_EXPLICIT="true"
      ;;
    localization_ready_timeout_s:=*)
      LOCALIZATION_READY_TIMEOUT_S="${arg#localization_ready_timeout_s:=}"
      LOCALIZATION_READY_TIMEOUT_EXPLICIT="true"
      ;;
    localization_scan_ready_timeout_s:=*)
      LOCALIZATION_SCAN_READY_TIMEOUT_S="${arg#localization_scan_ready_timeout_s:=}"
      LOCALIZATION_SCAN_READY_TIMEOUT_EXPLICIT="true"
      ;;
    nav2_start_delay_s:=*)
      NAV2_START_DELAY_S="${arg#nav2_start_delay_s:=}"
      NAV2_START_DELAY_EXPLICIT="true"
      ;;
    gazebo_post_ready_delay_s:=*)
      GAZEBO_POST_READY_DELAY_S="${arg#gazebo_post_ready_delay_s:=}"
      GAZEBO_POST_READY_DELAY_EXPLICIT="true"
      ;;
    spawn_pre_delay_s:=*)
      SPAWN_PRE_DELAY_S="${arg#spawn_pre_delay_s:=}"
      SPAWN_PRE_DELAY_EXPLICIT="true"
      ;;
    spawn_post_delay_s:=*)
      SPAWN_POST_DELAY_S="${arg#spawn_post_delay_s:=}"
      SPAWN_POST_DELAY_EXPLICIT="true"
      ;;
    params_file:=*)
      FOLLOW_PARAMS_SOURCE="${arg#params_file:=}"
      ;;
    rebuild:=*)
      REBUILD="${arg#rebuild:=}"
      ;;
    realign:=*)
      REALIGN="${arg#realign:=}"
      ;;
    session:=*)
      SESSION="${arg#session:=}"
      SESSION_EXPLICIT="true"
      ;;
    tmux_attach:=*)
      TMUX_ATTACH="${arg#tmux_attach:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    pc2ls_*)
      TUNING_LIDAR_ARGS+=("$arg")
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

apply_profile_defaults

if [[ "$WORLD" == baylands* ]]; then
  baylands_sync_waypoints "$DRY_RUN"
fi

apply_waypoint_default_nav2_goals
apply_nav2_goals_default_waypoint
mapfile -t route_default_lidar_args < <(route_lidar_preset_args "$NAV2_GOALS" "$LIDAR" "${TUNING_LIDAR_ARGS[@]}")
TUNING_LIDAR_ARGS+=("${route_default_lidar_args[@]}")

CLI_WORLD="$WORLD"
CLI_PROFILE="$PROFILE"
CLI_WAYPOINT="$WAYPOINT"
CLI_NAV2_GOALS="$NAV2_GOALS"
CLI_NAV2_GOALS_SOURCE="$NAV2_GOALS_SOURCE"
CLI_LIDAR="$LIDAR"
CLI_PAUSE_AFTER_GOAL_S="$PAUSE_AFTER_GOAL_S"
CLI_GUI="$GUI"
CLI_PERSPECTIVE="$PERSPECTIVE"
CLI_START_RQT="$START_RQT"
CLI_START_COLLISION_MONITOR="$START_COLLISION_MONITOR"
CLI_MUTE_UGV_CAMERA="$MUTE_UGV_CAMERA"
CLI_CLOCK_MODE="$CLOCK_MODE"
CLI_START_OPTIONAL_TELEOP="$START_OPTIONAL_TELEOP"
CLI_MAP_PATH="$MAP_PATH"
CLI_SPAWN_UAV="$SPAWN_UAV"
CLI_UAV_CAMERA_UPDATE_RATE="$UAV_CAMERA_UPDATE_RATE"
CLI_WITH_FOLLOW="$WITH_FOLLOW"
CLI_FOLLOW_START_DELAY_S="$FOLLOW_START_DELAY_S"
CLI_LOCALIZATION_READY_TIMEOUT_S="$LOCALIZATION_READY_TIMEOUT_S"
CLI_LOCALIZATION_SCAN_READY_TIMEOUT_S="$LOCALIZATION_SCAN_READY_TIMEOUT_S"
CLI_NAV2_START_DELAY_S="$NAV2_START_DELAY_S"
CLI_GAZEBO_POST_READY_DELAY_S="$GAZEBO_POST_READY_DELAY_S"
CLI_SPAWN_PRE_DELAY_S="$SPAWN_PRE_DELAY_S"
CLI_SPAWN_POST_DELAY_S="$SPAWN_POST_DELAY_S"
CLI_TUNING_LIDAR_ARGS=("${TUNING_LIDAR_ARGS[@]}")

if [ "$SESSION_EXPLICIT" != "true" ]; then
  SESSION="$(default_session)"
fi

if [ "$ACTION" != "start" ] && [ "$DRY_RUN" != "true" ]; then
  load_state
  if [ "$PROFILE_EXPLICIT" = "true" ]; then
    PROFILE="$CLI_PROFILE"
    apply_profile_defaults
  fi
  if [ "$WAYPOINT_EXPLICIT" = "true" ]; then
    WAYPOINT="$CLI_WAYPOINT"
  fi
  if [ "$NAV2_GOALS_EXPLICIT" = "true" ]; then
    NAV2_GOALS="$CLI_NAV2_GOALS"
    NAV2_GOALS_SOURCE="$CLI_NAV2_GOALS_SOURCE"
  fi
  if [ "$LIDAR_EXPLICIT" = "true" ]; then
    LIDAR="$CLI_LIDAR"
  fi
  if [ "$PAUSE_AFTER_GOAL_EXPLICIT" = "true" ]; then
    PAUSE_AFTER_GOAL_S="$CLI_PAUSE_AFTER_GOAL_S"
  fi
  if [ "$GUI_EXPLICIT" = "true" ]; then
    GUI="$CLI_GUI"
  fi
  if [ "$PERSPECTIVE_EXPLICIT" = "true" ]; then
    PERSPECTIVE="$CLI_PERSPECTIVE"
  fi
  if [ "$START_RQT_EXPLICIT" = "true" ]; then
    START_RQT="$CLI_START_RQT"
  fi
  if [ "$START_COLLISION_MONITOR_EXPLICIT" = "true" ]; then
    START_COLLISION_MONITOR="$CLI_START_COLLISION_MONITOR"
  fi
  if [ "$MUTE_UGV_CAMERA_EXPLICIT" = "true" ]; then
    MUTE_UGV_CAMERA="$CLI_MUTE_UGV_CAMERA"
  fi
  if [ "$CLOCK_MODE_EXPLICIT" = "true" ]; then
    CLOCK_MODE="$CLI_CLOCK_MODE"
  fi
  if [ "$START_OPTIONAL_TELEOP_EXPLICIT" = "true" ]; then
    START_OPTIONAL_TELEOP="$CLI_START_OPTIONAL_TELEOP"
  fi
  if [ "$MAP_EXPLICIT" = "true" ]; then
    MAP_PATH="$CLI_MAP_PATH"
  fi
  if [ "$SPAWN_UAV_EXPLICIT" = "true" ]; then
    SPAWN_UAV="$CLI_SPAWN_UAV"
  fi
  if [ "$UAV_CAMERA_UPDATE_RATE_EXPLICIT" = "true" ]; then
    UAV_CAMERA_UPDATE_RATE="$CLI_UAV_CAMERA_UPDATE_RATE"
  fi
  if [ "$WITH_FOLLOW_EXPLICIT" = "true" ]; then
    WITH_FOLLOW="$CLI_WITH_FOLLOW"
  fi
  if [ "$FOLLOW_START_DELAY_EXPLICIT" = "true" ]; then
    FOLLOW_START_DELAY_S="$CLI_FOLLOW_START_DELAY_S"
  fi
  if [ "$LOCALIZATION_READY_TIMEOUT_EXPLICIT" = "true" ]; then
    LOCALIZATION_READY_TIMEOUT_S="$CLI_LOCALIZATION_READY_TIMEOUT_S"
  fi
  if [ "$LOCALIZATION_SCAN_READY_TIMEOUT_EXPLICIT" = "true" ]; then
    LOCALIZATION_SCAN_READY_TIMEOUT_S="$CLI_LOCALIZATION_SCAN_READY_TIMEOUT_S"
  fi
  if [ "$NAV2_START_DELAY_EXPLICIT" = "true" ]; then
    NAV2_START_DELAY_S="$CLI_NAV2_START_DELAY_S"
  fi
  if [ "$GAZEBO_POST_READY_DELAY_EXPLICIT" = "true" ]; then
    GAZEBO_POST_READY_DELAY_S="$CLI_GAZEBO_POST_READY_DELAY_S"
  fi
  if [ "$SPAWN_PRE_DELAY_EXPLICIT" = "true" ]; then
    SPAWN_PRE_DELAY_S="$CLI_SPAWN_PRE_DELAY_S"
  fi
  if [ "$SPAWN_POST_DELAY_EXPLICIT" = "true" ]; then
    SPAWN_POST_DELAY_S="$CLI_SPAWN_POST_DELAY_S"
  fi
fi

apply_waypoint_default_nav2_goals
apply_nav2_goals_default_waypoint
apply_nav2_goals_waypoint_slice
if [ "$ACTION" != "start" ] && [ "$DRY_RUN" != "true" ]; then
  write_state
fi

case "$ACTION" in
  start)
    create_session
    start_base_processes
    restart_stack
    if [ "$TMUX_ATTACH" = "true" ] && [ "$DRY_RUN" != "true" ]; then
      exec tmux attach -t "$SESSION"
    fi
    ;;
  restart)
    restart_stack
    ;;
  stack_stop)
    stop_stack_processes
    ;;
  follow)
    restart_follow_only
    ;;
  route_stop)
    stop_route_only
    ;;
  stop)
    stop_all
    ;;
  attach)
    if [ "$DRY_RUN" = "true" ]; then
      echo "tmux attach -t $SESSION"
    else
      exec tmux attach -t "$SESSION"
    fi
    ;;
  status)
    print_status
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

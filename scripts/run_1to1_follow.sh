#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
SIM_SPAWN_WAYPOINT_FILE="$STATE_DIR/gazebo_sim.spawn_waypoint"
BAYLANDS_DEFAULT_NAV2_GOALS="baylands_waypoints.yaml"
WORLD="baylands"
DEFAULT_UAV_BODY_X_OFFSET="-7.0"
DEFAULT_UAV_BODY_Y_OFFSET="0.0"
DEFAULT_UAV_Z="7.0"
UAV_START_Z_VALUE="$DEFAULT_UAV_Z"
UAV_NAME="dji0"
REQUESTED_WAYPOINT_NAME=""

source "$SCRIPT_DIR/slam_state_common.sh"
source "$SCRIPT_DIR/baylands_waypoint_common.sh"
BAYLANDS_GROUP_WAYPOINT_CSV="$(baylands_group_waypoint_csv)"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh 1to1_follow [world] [options...]

Start the odom/follow pipeline and the Nav2-backed UGV route driver.

Common options:
  waypoint:=rotundan_0
  nav2_goals:=rotundan|path.yaml
  start_uav_simulator:=true|false
  start_uav_follow:=true|false
  start_camera_tracker:=true|false
  start_ugv_ground_truth_bridge:=true|false
  ugv_odom_topic:=/a201_0000/ground_truth/odom
  params_file:=path/to/run_follow_defaults.yaml
  height:=7
  uav_start_x:=... uav_start_y:=... uav_start_z:=... uav_start_yaw_deg:=...

Examples:
  ./run.sh 1to1_follow baylands waypoint:=rotundan_0 nav2_goals:=rotundan
  ./run.sh 1to1_follow baylands start_uav_simulator:=false start_uav_follow:=false
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

resolve_baylands_amcl_waypoint_pose() {
  local waypoint_name="$1"
  python3 - "$waypoint_name" "$BAYLANDS_GROUP_WAYPOINT_CSV" <<'PY'
import csv
import math
import sys

name, *paths = sys.argv[1:]

for path in paths:
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                waypoint_name = str(row.get("place", "")).strip()
                if waypoint_name != name:
                    continue
                try:
                    amcl_x = float(row["amcl_x"])
                    amcl_y = float(row["amcl_y"])
                    amcl_yaw = float(row["amcl_yaw"])
                except (KeyError, TypeError, ValueError):
                    raise SystemExit(1)

                print(f"ugv_waypoint_name={waypoint_name!r}")
                print(f"ugv_waypoint_distance_m={0.0:.9f}")
                print(f"ugv_amcl_x={amcl_x:.9f}")
                print(f"ugv_amcl_y={amcl_y:.9f}")
                print(f"ugv_amcl_yaw_deg={math.degrees(amcl_yaw):.9f}")
                raise SystemExit(0)
    except FileNotFoundError:
        continue

raise SystemExit(1)
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

if [[ "$WORLD" == baylands* ]]; then
  baylands_sync_waypoints false
fi

EXTRA_ARGS=()
HAVE_UAV_START_X="false"
HAVE_UAV_START_Y="false"
HAVE_UAV_START_YAW="false"
HAVE_UAV_START_Z="false"
HAVE_REQUIRE_UAV_ACTUAL_BEFORE_MOTION="false"
HAVE_UGV_ODOM_TOPIC="false"
HAVE_UGV_USE_AMCL_ODOM_FALLBACK="false"
HAVE_START_UGV_GROUND_TRUTH_BRIDGE="false"
HAVE_CAMERA_LEADER_ACTUAL_POSE_TOPIC="false"
HAVE_UGV_INITIAL_POSE_X="false"
HAVE_UGV_INITIAL_POSE_Y="false"
HAVE_UGV_INITIAL_POSE_YAW="false"
HAVE_UGV_GOAL_SEQUENCE="false"
HAVE_UGV_SET_INITIAL_POSE="false"
START_UAV_SIMULATOR_VALUE="true"
HAVE_START_UAV_FOLLOW="false"
HAVE_START_CAMERA_TRACKER="false"
for arg in "$@"; do
  case "$arg" in
    waypoint:=*)
      REQUESTED_WAYPOINT_NAME="${arg#waypoint:=}"
      ;;
    start_uav_simulator:=*)
      START_UAV_SIMULATOR_VALUE="${arg#start_uav_simulator:=}"
      EXTRA_ARGS+=("$arg")
      ;;
    start_uav_follow:=*)
      HAVE_START_UAV_FOLLOW="true"
      EXTRA_ARGS+=("$arg")
      ;;
    start_camera_tracker:=*)
      HAVE_START_CAMERA_TRACKER="true"
      EXTRA_ARGS+=("$arg")
      ;;
    camera:=*|camera_mode:=*)
      camera_mode="${arg#camera:=}"
      if [[ "$arg" == camera_mode:=* ]]; then
        camera_mode="${arg#camera_mode:=}"
      fi
      case "$camera_mode" in
        attached|integrated|integrated_joint)
          EXTRA_ARGS+=("uav_camera_mode:=integrated_joint")
          ;;
        detached|detached_model)
          echo "Detached camera mode has been removed from simulation. Use camera:=attached." >&2
          exit 2
          ;;
        *)
      EXTRA_ARGS+=("uav_camera_mode:=$camera_mode")
          ;;
      esac
      ;;
    uav_name:=*)
      UAV_NAME="${arg#uav_name:=}"
      EXTRA_ARGS+=("$arg")
      ;;
    uav_start_x:=*)
      HAVE_UAV_START_X="true"
      EXTRA_ARGS+=("$arg")
      ;;
    uav_start_y:=*)
      HAVE_UAV_START_Y="true"
      EXTRA_ARGS+=("$arg")
      ;;
    uav_start_yaw_deg:=*)
      HAVE_UAV_START_YAW="true"
      EXTRA_ARGS+=("$arg")
      ;;
    uav_start_z:=*)
      HAVE_UAV_START_Z="true"
      UAV_START_Z_VALUE="${arg#uav_start_z:=}"
      EXTRA_ARGS+=("$arg")
      ;;
    require_uav_actual_before_motion:=*)
      HAVE_REQUIRE_UAV_ACTUAL_BEFORE_MOTION="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_odom_topic:=*)
      HAVE_UGV_ODOM_TOPIC="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_use_amcl_odom_fallback:=*)
      HAVE_UGV_USE_AMCL_ODOM_FALLBACK="true"
      EXTRA_ARGS+=("$arg")
      ;;
    start_ugv_ground_truth_bridge:=*)
      HAVE_START_UGV_GROUND_TRUTH_BRIDGE="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_initial_pose_x:=*)
      HAVE_UGV_INITIAL_POSE_X="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_initial_pose_y:=*)
      HAVE_UGV_INITIAL_POSE_Y="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_initial_pose_yaw_deg:=*)
      HAVE_UGV_INITIAL_POSE_YAW="true"
      EXTRA_ARGS+=("$arg")
      ;;
    ugv_set_initial_pose:=*)
      HAVE_UGV_SET_INITIAL_POSE="true"
      EXTRA_ARGS+=("$arg")
      ;;
    goal_sequence_file:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      if [[ "$WORLD" == baylands* ]]; then
        EXTRA_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "${arg#goal_sequence_file:=}")")
      else
        EXTRA_ARGS+=("nav2_goals:=${arg#goal_sequence_file:=}")
      fi
      ;;
    goal_sequence_csv:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      EXTRA_ARGS+=("ugv_goal_sequence_csv:=${arg#goal_sequence_csv:=}")
      ;;
    nav2_goals:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      if [[ "$WORLD" == baylands* ]]; then
        EXTRA_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "${arg#nav2_goals:=}")")
      else
        EXTRA_ARGS+=("$arg")
      fi
      ;;
    ugv_goal_sequence_file:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      if [[ "$WORLD" == baylands* ]]; then
        EXTRA_ARGS+=("ugv_goal_sequence_file:=$(baylands_route_yaml_path "${arg#ugv_goal_sequence_file:=}")")
      else
        EXTRA_ARGS+=("$arg")
      fi
      ;;
    ugv_goal_sequence_csv:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      EXTRA_ARGS+=("$arg")
      ;;
    params_file:=*)
      EXTRA_ARGS+=("$arg")
      ;;
    camera_leader_actual_pose_topic:=*)
      HAVE_CAMERA_LEADER_ACTUAL_POSE_TOPIC="true"
      EXTRA_ARGS+=("$arg")
      ;;
    height:=*)
      HAVE_UAV_START_Z="true"
      UAV_START_Z_VALUE="${arg#height:=}"
      EXTRA_ARGS+=("uav_start_z:=${arg#height:=}")
      ;;
    pan_enable:=*)
      EXTRA_ARGS+=("$arg")
      ;;
    use_tilt:=*)
      EXTRA_ARGS+=("tilt_enable:=${arg#use_tilt:=}")
      ;;
    tilt_enable:=*)
      EXTRA_ARGS+=("$arg")
      ;;
    mount_pitch_deg:=*)
      EXTRA_ARGS+=("camera_mount_pitch_deg:=${arg#mount_pitch_deg:=}")
      ;;
    omnet:=*)
      EXTRA_ARGS+=("start_omnet_bridge:=${arg#omnet:=}")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

if [ "$START_UAV_SIMULATOR_VALUE" = "false" ]; then
  if [ "$HAVE_START_UAV_FOLLOW" = "false" ]; then
    EXTRA_ARGS+=("start_uav_follow:=false")
  fi
  if [ "$HAVE_START_CAMERA_TRACKER" = "false" ]; then
    EXTRA_ARGS+=("start_camera_tracker:=false")
  fi
fi

if [[ "$WORLD" == baylands* ]] && [ "$HAVE_UGV_GOAL_SEQUENCE" = "false" ]; then
  EXTRA_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "$BAYLANDS_DEFAULT_NAV2_GOALS")")
fi

add_uav_start_from_live_uav_pose() {
  local timeout_s="$1"
  local uav_start_env=""

  uav_start_env="$(slam_state_capture_gazebo_named_pose_env \
    "$WORLD" \
    "$UAV_NAME" \
    "$timeout_s")" || return 1
  eval "$uav_start_env"
  EXTRA_ARGS+=("uav_start_x:=$spawn_x" "uav_start_y:=$spawn_y")
  if [ "$HAVE_UAV_START_Z" = "false" ]; then
    EXTRA_ARGS+=("uav_start_z:=$spawn_z")
  fi
  if [ "$HAVE_UAV_START_YAW" = "false" ]; then
    EXTRA_ARGS+=("uav_start_yaw_deg:=$(python3 - "$spawn_yaw" <<'PY'
import math
import sys
print(f"{math.degrees(float(sys.argv[1])):.9f}")
PY
)")
  fi
  echo "[run_1to1_follow] Using live UAV pose for simulator start x=${spawn_x} y=${spawn_y} z=${spawn_z} yaw=${spawn_yaw}"
}

add_uav_start_from_live_ugv_pose() {
  local timeout_s="$1"
  local uav_start_env=""

  uav_start_env="$(slam_state_capture_uav_spawn_from_ugv_env \
    "$WS_ROOT" \
    "$WORLD" \
    "$DEFAULT_UAV_BODY_X_OFFSET" \
    "$DEFAULT_UAV_BODY_Y_OFFSET" \
    "$UAV_START_Z_VALUE" \
    "$timeout_s")" || return 1
  eval "$uav_start_env"
  EXTRA_ARGS+=("uav_start_x:=$uav_x" "uav_start_y:=$uav_y")
  if [ "$HAVE_UAV_START_Z" = "false" ]; then
    EXTRA_ARGS+=("uav_start_z:=$uav_z")
  fi
  if [ "$HAVE_UAV_START_YAW" = "false" ]; then
    EXTRA_ARGS+=("uav_start_yaw_deg:=$uav_yaw_deg")
  fi
  echo "[run_1to1_follow] Using live UGV-relative UAV start x=${uav_x} y=${uav_y} z=${uav_z} yaw_deg=${uav_yaw_deg}"
}

add_ugv_initial_pose_from_baylands_waypoint_map() {
  local timeout_s="$1"
  local ugv_pose_env=""
  local amcl_pose_env=""
  local spawn_waypoint=""

  if [ -n "$REQUESTED_WAYPOINT_NAME" ]; then
    spawn_waypoint="$REQUESTED_WAYPOINT_NAME"
  elif [ -f "$SIM_SPAWN_WAYPOINT_FILE" ]; then
    spawn_waypoint="$(tr -d '\r\n' < "$SIM_SPAWN_WAYPOINT_FILE" 2>/dev/null || true)"
  fi

  if [ -n "$spawn_waypoint" ]; then
    if amcl_pose_env="$(resolve_baylands_amcl_waypoint_pose "$spawn_waypoint")"; then
      eval "$amcl_pose_env"
      if [ "$HAVE_UGV_INITIAL_POSE_X" = "false" ]; then
        EXTRA_ARGS+=("ugv_initial_pose_x:=$ugv_amcl_x")
      fi
      if [ "$HAVE_UGV_INITIAL_POSE_Y" = "false" ]; then
        EXTRA_ARGS+=("ugv_initial_pose_y:=$ugv_amcl_y")
      fi
      if [ "$HAVE_UGV_INITIAL_POSE_YAW" = "false" ]; then
        EXTRA_ARGS+=("ugv_initial_pose_yaw_deg:=$ugv_amcl_yaw_deg")
      fi
      echo "[run_1to1_follow] Using exact Baylands waypoint '$ugv_waypoint_name' for Nav2 initial pose x=${ugv_amcl_x} y=${ugv_amcl_y} yaw_deg=${ugv_amcl_yaw_deg}"
      return 0
    fi
    echo "[run_1to1_follow] Warning: exact Baylands waypoint '$spawn_waypoint' has no usable AMCL pose; falling back to nearest world-position match." >&2
  fi

  ugv_pose_env="$(slam_state_capture_gazebo_pose_env "$WS_ROOT" "$WORLD" "$timeout_s")" || return 1
  eval "$ugv_pose_env"
  amcl_pose_env="$(
    python3 - "$spawn_x" "$spawn_y" "$BAYLANDS_GROUP_WAYPOINT_CSV" <<'PY'
import csv
import math
import sys

world_x = float(sys.argv[1])
world_y = float(sys.argv[2])
paths = sys.argv[3:]
best = None

for path in paths:
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                try:
                    x = float(row["x"])
                    y = float(row["y"])
                    amcl_x = float(row["amcl_x"])
                    amcl_y = float(row["amcl_y"])
                    amcl_yaw = float(row["amcl_yaw"])
                except (KeyError, TypeError, ValueError):
                    continue
                distance = math.hypot(x - world_x, y - world_y)
                if best is None or distance < best[0]:
                    best = (distance, row.get("place", ""), amcl_x, amcl_y, amcl_yaw)
    except FileNotFoundError:
        continue

if best is None or best[0] > 3.0:
    raise SystemExit(1)

distance, place, amcl_x, amcl_y, amcl_yaw = best
print(f"ugv_waypoint_name={place!r}")
print(f"ugv_waypoint_distance_m={distance:.9f}")
print(f"ugv_amcl_x={amcl_x:.9f}")
print(f"ugv_amcl_y={amcl_y:.9f}")
print(f"ugv_amcl_yaw_deg={math.degrees(amcl_yaw):.9f}")
PY
  )" || return 1
  eval "$amcl_pose_env"
  if [ "$HAVE_UGV_INITIAL_POSE_X" = "false" ]; then
    EXTRA_ARGS+=("ugv_initial_pose_x:=$ugv_amcl_x")
  fi
  if [ "$HAVE_UGV_INITIAL_POSE_Y" = "false" ]; then
    EXTRA_ARGS+=("ugv_initial_pose_y:=$ugv_amcl_y")
  fi
  if [ "$HAVE_UGV_INITIAL_POSE_YAW" = "false" ]; then
    EXTRA_ARGS+=("ugv_initial_pose_yaw_deg:=$ugv_amcl_yaw_deg")
  fi
  echo "[run_1to1_follow] Using Baylands waypoint '$ugv_waypoint_name' for Nav2 initial pose x=${ugv_amcl_x} y=${ugv_amcl_y} yaw_deg=${ugv_amcl_yaw_deg} (world match ${ugv_waypoint_distance_m}m)"
}

if [ "$HAVE_UAV_START_X" = "false" ] && [ "$HAVE_UAV_START_Y" = "false" ]; then
  if [[ "$WORLD" == baylands* ]]; then
    if add_uav_start_from_live_uav_pose 30; then
      :
    elif add_uav_start_from_live_ugv_pose 10; then
      echo "[run_1to1_follow] Warning: live UAV pose was not ready for Baylands start-up, so falling back to the live UGV-relative spawn pose." >&2
    else
      echo "[run_1to1_follow] Error: could not read the live UAV pose or the live UGV pose for Baylands, so refusing to invent a startup pose." >&2
      echo "[run_1to1_follow] Start Gazebo and spawn the UAV first, or pass explicit uav_start_x:=... uav_start_y:=... uav_start_z:=... uav_start_yaw_deg:=..." >&2
      exit 2
    fi
  elif add_uav_start_from_live_uav_pose 5; then
    :
  elif add_uav_start_from_live_ugv_pose 5; then
    :
  else
    if [[ "$WORLD" == baylands* ]]; then
      echo "[run_1to1_follow] Error: could not read the live UGV pose for Baylands, so refusing to fall back to the generic UAV start (-7,0,7)." >&2
      echo "[run_1to1_follow] Start Gazebo and the UGV first, or pass explicit uav_start_x:=... uav_start_y:=... uav_start_z:=... uav_start_yaw_deg:=..." >&2
      exit 2
    fi
    echo "[run_1to1_follow] Warning: could not read the live UGV pose; falling back to the launch defaults." >&2
  fi
fi

if [ "$HAVE_REQUIRE_UAV_ACTUAL_BEFORE_MOTION" = "false" ]; then
  EXTRA_ARGS+=("require_uav_actual_before_motion:=true")
fi

if [[ "$WORLD" == baylands* ]]; then
  UGV_NAMESPACE="$(slam_state_namespace "$WS_ROOT" 2>/dev/null || true)"
  if [ -z "$UGV_NAMESPACE" ]; then
    UGV_NAMESPACE="a201_0000"
  fi
  if ! add_ugv_initial_pose_from_baylands_waypoint_map 10; then
    echo "[run_1to1_follow] Warning: could not resolve the live UGV world pose to a Baylands waypoint; Nav2 initial pose will use launch defaults." >&2
  fi
  if [ "$HAVE_UGV_ODOM_TOPIC" = "false" ]; then
    EXTRA_ARGS+=("ugv_odom_topic:=/$UGV_NAMESPACE/ground_truth/odom")
  fi
  if [ "$HAVE_START_UGV_GROUND_TRUTH_BRIDGE" = "false" ]; then
    EXTRA_ARGS+=("start_ugv_ground_truth_bridge:=true")
  fi
  if [ "$HAVE_UGV_USE_AMCL_ODOM_FALLBACK" = "false" ]; then
    EXTRA_ARGS+=("ugv_use_amcl_odom_fallback:=false")
  fi
  if [ "$HAVE_CAMERA_LEADER_ACTUAL_POSE_TOPIC" = "false" ]; then
    EXTRA_ARGS+=("camera_leader_actual_pose_topic:=/$UGV_NAMESPACE/ground_truth/odom")
  fi
fi

if [ "$HAVE_UGV_SET_INITIAL_POSE" = "false" ]; then
  EXTRA_ARGS+=("ugv_set_initial_pose:=true")
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

set +e
ros2 launch lrs_halmstad run_follow.launch.py \
  ugv_mode:=nav2 \
  leader_mode:=odom \
  "${EXTRA_ARGS[@]}" \
  world:="$WORLD"
STATUS=$?
set -e

case "$STATUS" in
  0|130)
    exit 0
    ;;
  *)
    exit "$STATUS"
    ;;
esac

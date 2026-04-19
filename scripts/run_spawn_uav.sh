#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
FOLLOW_SIM=false
LAUNCH_PID=""
WATCH_PID=""
DEFAULT_UAV_BODY_X_OFFSET="-7.0"
DEFAULT_UAV_BODY_Y_OFFSET="0.0"
DEFAULT_UAV_Z="7.0"

source "$SCRIPT_DIR/slam_state_common.sh"

sim_helper_running() {
  if [ ! -f "$SIM_PID_FILE" ]; then
    return 1
  fi

  local sim_pid
  sim_pid="$(cat "$SIM_PID_FILE" 2>/dev/null || true)"
  if [ -z "$sim_pid" ]; then
    return 1
  fi

  kill -0 "$sim_pid" 2>/dev/null
}

launch_running() {
  if [ -z "$LAUNCH_PID" ]; then
    return 1
  fi

  kill -0 "$LAUNCH_PID" 2>/dev/null
}

launch_group_running() {
  local launch_pgid

  if ! launch_running; then
    return 1
  fi

  launch_pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
  if [ -z "$launch_pgid" ]; then
    return 1
  fi

  /bin/kill -0 -- "-$launch_pgid" 2>/dev/null
}

stop_launch_group() {
  local signal="$1"
  local timeout_s="$2"
  local waited_s=0
  local launch_pgid=""

  if ! launch_running; then
    return 0
  fi

  launch_pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
  if [ -n "$launch_pgid" ]; then
    /bin/kill "-$signal" -- "-$launch_pgid" 2>/dev/null || true
  else
    kill "-$signal" "$LAUNCH_PID" 2>/dev/null || true
  fi

  while launch_running && [ "$waited_s" -lt "$timeout_s" ]; do
    sleep 1
    waited_s=$((waited_s + 1))
  done
}

cleanup() {
  if [ -n "$WATCH_PID" ] && kill -0 "$WATCH_PID" 2>/dev/null; then
    kill "$WATCH_PID" 2>/dev/null || true
    wait "$WATCH_PID" 2>/dev/null || true
  fi

  if launch_running; then
    stop_launch_group INT 5
    stop_launch_group TERM 3
    if launch_running; then
      stop_launch_group KILL 0
    fi
  fi

  if [ -n "$LAUNCH_PID" ]; then
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup INT TERM EXIT

if sim_helper_running; then
  FOLLOW_SIM=true
  echo "[run_spawn_uav] Gazebo helper detected; stopping this launcher when the sim helper exits."
fi

DEFAULT_WORLD="warehouse"
if sim_helper_running && [ -f "$SIM_WORLD_FILE" ]; then
  sim_world="$(cat "$SIM_WORLD_FILE" 2>/dev/null || true)"
  if [ -n "$sim_world" ]; then
    DEFAULT_WORLD="$sim_world"
  fi
fi

WORLD="$DEFAULT_WORLD"
if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

ARGS=()
HAVE_X="false"
HAVE_Y="false"
HAVE_Z="false"
HAVE_YAW="false"
for arg in "$@"; do
  case "$arg" in
    x:=*)
      HAVE_X="true"
      ARGS+=("$arg")
      ;;
    y:=*)
      HAVE_Y="true"
      ARGS+=("$arg")
      ;;
    z:=*)
      HAVE_Z="true"
      ARGS+=("$arg")
      ;;
    yaw:=*)
      HAVE_YAW="true"
      ARGS+=("$arg")
      ;;
    camera:=*)
      camera_mode="${arg#camera:=}"
      case "$camera_mode" in
        attached|integrated|integrated_joint)
          ARGS+=("uav_camera_mode:=integrated_joint")
          ;;
        detached|detached_model)
          echo "Detached camera mode has been removed from simulation. Use camera:=attached." >&2
          exit 2
          ;;
        *)
          ARGS+=("uav_camera_mode:=$camera_mode")
          ;;
      esac
      ;;
    name:=*)
      ARGS+=("uav_name:=${arg#name:=}")
      ;;
    height:=*)
      HAVE_Z="true"
      ARGS+=("z:=${arg#height:=}")
      ;;
    mount_pitch_deg:=*)
      ARGS+=("camera_pitch_offset_deg:=${arg#mount_pitch_deg:=}")
      ;;
    *)
      ARGS+=("$arg")
      ;;
  esac
done

if [ "$HAVE_X" = "false" ] && [ "$HAVE_Y" = "false" ]; then
  if UAV_SPAWN_ENV="$(slam_state_capture_uav_spawn_from_ugv_env \
    "$WS_ROOT" \
    "$WORLD" \
    "$DEFAULT_UAV_BODY_X_OFFSET" \
    "$DEFAULT_UAV_BODY_Y_OFFSET" \
    "$DEFAULT_UAV_Z" \
    5)"; then
    eval "$UAV_SPAWN_ENV"
    ARGS+=("x:=$uav_x" "y:=$uav_y")
    if [ "$HAVE_Z" = "false" ]; then
      ARGS+=("z:=$uav_z")
    fi
    if [ "$HAVE_YAW" = "false" ]; then
      ARGS+=("yaw:=$uav_yaw")
    fi
    echo "[run_spawn_uav] Using UGV-relative UAV spawn x=${uav_x} y=${uav_y} z=${uav_z} yaw=${uav_yaw} from UGV x=${ugv_x} y=${ugv_y} yaw=${ugv_yaw}"
  else
    echo "[run_spawn_uav] Warning: could not read the live UGV pose; falling back to the launch defaults." >&2
  fi
fi

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

setsid ros2 launch lrs_halmstad spawn_uav_1to1.launch.py world:="$WORLD" "${ARGS[@]}" &
LAUNCH_PID=$!

if [ "$FOLLOW_SIM" = true ]; then
  (
    while launch_running; do
      if ! sim_helper_running; then
        echo "[run_spawn_uav] Gazebo helper exited; stopping UAV launcher."
        stop_launch_group INT 5
        stop_launch_group TERM 3
        if launch_running; then
          echo "[run_spawn_uav] UAV launcher ignored shutdown signals; forcing exit."
          stop_launch_group KILL 0
        fi
        exit 0
      fi
      sleep 1
    done
  ) &
  WATCH_PID=$!
fi

set +e
wait "$LAUNCH_PID"
STATUS=$?
set -e
trap - INT TERM EXIT
cleanup
exit "$STATUS"

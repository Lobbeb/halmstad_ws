#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
FOLLOW_SIM=true
LAUNCH_PID=""
WATCH_PID=""
NEXT_TO_UGV="auto"
DRY_RUN="false"
POSE_TIMEOUT_S="5"
UAV_MODE="${UAV_MODE:-teleport}"
POSITION_ARG_SET="false"
ARGS=()
POSE_ARGS=()
DJI0_BODY_X_OFFSET="-7.0"
DJI0_BODY_Y_OFFSET="0.0"
DJI0_Z="7.0"
DJI1_BODY_X_OFFSET="-10.0"
DJI1_BODY_Y_OFFSET="4.0"
DJI1_Z="8.0"
DJI2_BODY_X_OFFSET="-10.0"
DJI2_BODY_Y_OFFSET="-4.0"
DJI2_Z="8.0"

source "$SCRIPT_DIR/slam_state_common.sh"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh spawn_uavs [world] [arg:=value ...]

Spawn dji0, dji1, and dji2. By default, if Gazebo is running, the wrapper
tries to place them next to the active UGV instead of using world origin.

Common arguments:
  world                         World key. Default: active Gazebo world, else baylands.
  world:=NAME                   Same as positional world.
  next_to_ugv:=auto|true|false  auto uses UGV pose when available. Default: auto.
  pose_timeout_s:=5             Timeout for reading Gazebo UGV pose.
  dry_run:=true|false           Print launch command only. Default: false.
  uav_mode:=teleport|physics    Forwarded to launch. Default: teleport.
  camera_update_rate:=20        Forwarded to launch.

Relative-to-UGV defaults:
  dji0_body_x_offset:=-7.0   dji0_body_y_offset:=0.0   dji0_z:=7.0
  dji1_body_x_offset:=-10.0  dji1_body_y_offset:=4.0   dji1_z:=8.0
  dji2_body_x_offset:=-10.0  dji2_body_y_offset:=-4.0  dji2_z:=8.0

Manual absolute pose overrides:
  dji0_x:=M dji0_y:=M dji0_z:=M dji0_yaw:=RAD
  dji1_x:=M dji1_y:=M dji1_z:=M dji1_yaw:=RAD
  dji2_x:=M dji2_y:=M dji2_z:=M dji2_yaw:=RAD

Examples:
  ./run.sh spawn_uavs baylands
  ./run.sh spawn_uavs baylands next_to_ugv:=true dry_run:=true
  ./run.sh spawn_uavs baylands dji0_x:=0 dji0_y:=0 dji0_z:=7 next_to_ugv:=false
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

compute_multi_uav_pose_args() {
  local ugv_x="$1"
  local ugv_y="$2"
  local ugv_yaw="$3"
  shift 3

  python3 - "$ugv_x" "$ugv_y" "$ugv_yaw" "$@" <<'PY'
import math
import sys

ugv_x = float(sys.argv[1])
ugv_y = float(sys.argv[2])
ugv_yaw = float(sys.argv[3])
values = [float(v) for v in sys.argv[4:]]

for idx, base in enumerate(range(0, len(values), 3)):
    body_x = values[base]
    body_y = values[base + 1]
    z = values[base + 2]
    x = ugv_x + body_x * math.cos(ugv_yaw) - body_y * math.sin(ugv_yaw)
    y = ugv_y + body_x * math.sin(ugv_yaw) + body_y * math.cos(ugv_yaw)
    print(f"dji{idx}_x={x:.9f}")
    print(f"dji{idx}_y={y:.9f}")
    print(f"dji{idx}_z={z:.9f}")
    print(f"dji{idx}_yaw={ugv_yaw:.9f}")
PY
}

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

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
  esac
done

if sim_helper_running; then
  FOLLOW_SIM=true
  echo "[run_spawn_uavs] Gazebo helper detected; stopping this launcher when the sim helper exits."
fi

DEFAULT_WORLD="baylands"
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

for arg in "$@"; do
  case "$arg" in
    world:=*)
      WORLD="${arg#world:=}"
      ;;
    next_to_ugv:=*)
      NEXT_TO_UGV="${arg#next_to_ugv:=}"
      ;;
    pose_timeout_s:=*)
      POSE_TIMEOUT_S="${arg#pose_timeout_s:=}"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${arg#dry_run:=}")"
      ;;
    uav_mode:=*)
      UAV_MODE="${arg#uav_mode:=}"
      ;;
    dji0_body_x_offset:=*)
      DJI0_BODY_X_OFFSET="${arg#dji0_body_x_offset:=}"
      ;;
    dji0_body_y_offset:=*)
      DJI0_BODY_Y_OFFSET="${arg#dji0_body_y_offset:=}"
      ;;
    dji1_body_x_offset:=*)
      DJI1_BODY_X_OFFSET="${arg#dji1_body_x_offset:=}"
      ;;
    dji1_body_y_offset:=*)
      DJI1_BODY_Y_OFFSET="${arg#dji1_body_y_offset:=}"
      ;;
    dji2_body_x_offset:=*)
      DJI2_BODY_X_OFFSET="${arg#dji2_body_x_offset:=}"
      ;;
    dji2_body_y_offset:=*)
      DJI2_BODY_Y_OFFSET="${arg#dji2_body_y_offset:=}"
      ;;
    dji0_z:=*|dji1_z:=*|dji2_z:=*|dji0_x:=*|dji0_y:=*|dji0_yaw:=*|dji1_x:=*|dji1_y:=*|dji1_yaw:=*|dji2_x:=*|dji2_y:=*|dji2_yaw:=*)
      POSITION_ARG_SET="true"
      ARGS+=("$arg")
      ;;
    *)
      ARGS+=("$arg")
      ;;
  esac
done

case "$NEXT_TO_UGV" in
  auto|true|false)
    ;;
  *)
    echo "[run_spawn_uavs] Invalid next_to_ugv option: $NEXT_TO_UGV" >&2
    echo "Use next_to_ugv:=auto, next_to_ugv:=true, or next_to_ugv:=false" >&2
    exit 2
    ;;
esac

if [ "$POSITION_ARG_SET" = "true" ] && [ "$NEXT_TO_UGV" != "false" ]; then
  echo "[run_spawn_uavs] Manual dji*_x/y/z/yaw args provided; disabling next_to_ugv placement."
  NEXT_TO_UGV="false"
fi

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

if [ "$NEXT_TO_UGV" != "false" ]; then
  if UGV_POSE_ENV="$(slam_state_capture_gazebo_pose_env "$WS_ROOT" "$WORLD" "$POSE_TIMEOUT_S")"; then
    eval "$UGV_POSE_ENV"
    UAV_POSE_ENV="$(compute_multi_uav_pose_args \
      "$spawn_x" \
      "$spawn_y" \
      "$spawn_yaw" \
      "$DJI0_BODY_X_OFFSET" "$DJI0_BODY_Y_OFFSET" "$DJI0_Z" \
      "$DJI1_BODY_X_OFFSET" "$DJI1_BODY_Y_OFFSET" "$DJI1_Z" \
      "$DJI2_BODY_X_OFFSET" "$DJI2_BODY_Y_OFFSET" "$DJI2_Z")"
    eval "$UAV_POSE_ENV"
    POSE_ARGS=(
      "dji0_x:=$dji0_x" "dji0_y:=$dji0_y" "dji0_z:=$dji0_z" "dji0_yaw:=$dji0_yaw"
      "dji1_x:=$dji1_x" "dji1_y:=$dji1_y" "dji1_z:=$dji1_z" "dji1_yaw:=$dji1_yaw"
      "dji2_x:=$dji2_x" "dji2_y:=$dji2_y" "dji2_z:=$dji2_z" "dji2_yaw:=$dji2_yaw"
    )
    echo "[run_spawn_uavs] Using UGV-relative spawn near x=${spawn_x} y=${spawn_y} yaw=${spawn_yaw}"
  elif [ "$NEXT_TO_UGV" = "true" ]; then
    echo "[run_spawn_uavs] Failed to read UGV pose for next_to_ugv:=true" >&2
    exit 1
  else
    echo "[run_spawn_uavs] UGV pose unavailable; using launch defaults."
  fi
fi

CMD=(ros2 launch lrs_halmstad spawn_uavs.launch.py world:="$WORLD" uav_mode:="$UAV_MODE" "${POSE_ARGS[@]}" "${ARGS[@]}")

if [ "$DRY_RUN" = "true" ]; then
  printf '[run_spawn_uavs] '
  printf '%q ' "${CMD[@]}"
  printf '\n'
  exit 0
fi

setsid "${CMD[@]}" &
LAUNCH_PID=$!

if [ "$FOLLOW_SIM" = true ]; then
  (
    while launch_running; do
      if ! sim_helper_running; then
        echo "[run_spawn_uavs] Gazebo helper exited; stopping UAV launcher."
        stop_launch_group INT 5
        stop_launch_group TERM 3
        if launch_running; then
          echo "[run_spawn_uavs] UAV launcher ignored shutdown signals; forcing exit."
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

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
WAIT_FOR_GAZEBO="${WAIT_FOR_GAZEBO:-true}"
GAZEBO_READY_TIMEOUT_S="${GAZEBO_READY_TIMEOUT_S:-180}"
NEXT_TO_UGV="false"
POSE_TIMEOUT_S="5"
UAV_BODY_X_OFFSET="-7.0"
UAV_BODY_Y_OFFSET="0.0"
UAV_Z=""
POSITION_ARG_SET="false"

source "$SCRIPT_DIR/slam_state_common.sh"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh spawn_uav [world] [options...]

Spawn the DJI UAV model into the active Gazebo world.

Common options:
  world                         World key. Default: active Gazebo world, else baylands.
  wait_for_gazebo:=true|false   Wait for /clock and /world/<world>/create. Default: true.
  gazebo_ready_timeout_s:=180   Wait timeout in seconds.
  name:=dji0                    UAV entity/name. Forwarded as uav_name.
  uav_mode:=teleport            UAV simulator mode forwarded to launch.
  camera:=attached              Camera mode. detached is intentionally unsupported.
  camera_name:=camera0          Camera namespace/name.
  camera_update_rate:=20        Camera update rate.
  height:=7                     Alias for z.
  x:=... y:=... z:=... yaw:=... Spawn pose.
  next_to_ugv:=true|false       Compute spawn behind current UGV. Default: false.
  pose_timeout_s:=5             Timeout for reading Gazebo UGV pose.
  uav_body_x_offset:=-7.0       Offset in UGV body x when next_to_ugv:=true.
  uav_body_y_offset:=0.0        Offset in UGV body y when next_to_ugv:=true.
  mount_pitch_deg:=45           Alias for camera_pitch_offset_deg.

Forwarded launch arguments:
  uav_name:=NAME
  uav_camera_mode:=integrated_joint
  camera_pitch_offset_deg:=DEG

Environment:
  WAIT_FOR_GAZEBO=true|false
  GAZEBO_READY_TIMEOUT_S=180

Examples:
  ./run.sh spawn_uav baylands name:=dji0 height:=7
  ./run.sh spawn_uav baylands x:=0 y:=0 z:=7 yaw:=0
  ./run.sh spawn_uav baylands next_to_ugv:=true height:=7
  ./run.sh spawn_uav baylands camera_update_rate:=10 mount_pitch_deg:=45
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

ARGS=()
for arg in "$@"; do
  case "$arg" in
    wait_for_gazebo:=*)
      WAIT_FOR_GAZEBO="${arg#wait_for_gazebo:=}"
      ;;
    gazebo_ready_timeout_s:=*|sim_ready_timeout_s:=*)
      GAZEBO_READY_TIMEOUT_S="${arg#*:=}"
      ;;
    next_to_ugv:=*)
      NEXT_TO_UGV="${arg#next_to_ugv:=}"
      ;;
    pose_timeout_s:=*)
      POSE_TIMEOUT_S="${arg#pose_timeout_s:=}"
      ;;
    uav_body_x_offset:=*|uav_x_offset:=*)
      UAV_BODY_X_OFFSET="${arg#*:=}"
      ;;
    uav_body_y_offset:=*|uav_y_offset:=*)
      UAV_BODY_Y_OFFSET="${arg#*:=}"
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
      UAV_Z="${arg#height:=}"
      ;;
    z:=*)
      UAV_Z="${arg#z:=}"
      POSITION_ARG_SET="true"
      ARGS+=("$arg")
      ;;
    x:=*|y:=*|yaw:=*)
      POSITION_ARG_SET="true"
      ARGS+=("$arg")
      ;;
    mount_pitch_deg:=*)
      ARGS+=("camera_pitch_offset_deg:=${arg#mount_pitch_deg:=}")
      ;;
    *)
      ARGS+=("$arg")
      ;;
  esac
done

case "$NEXT_TO_UGV" in
  true|false)
    ;;
  *)
    echo "[run_spawn_uav] Invalid next_to_ugv option: $NEXT_TO_UGV (use true or false)" >&2
    exit 2
    ;;
esac

if [ -z "$UAV_Z" ]; then
  UAV_Z="7.0"
fi

if [ "$NEXT_TO_UGV" = "false" ] && [ -n "$UAV_Z" ]; then
  if ! printf '%s\n' "${ARGS[*]}" | grep -q 'z:='; then
    ARGS+=("z:=$UAV_Z")
  fi
fi

if [ "$NEXT_TO_UGV" = "true" ] && [ "$POSITION_ARG_SET" = "true" ]; then
  echo "[run_spawn_uav] next_to_ugv:=true ignores manual x/y/z/yaw spawn pose args." >&2
  FILTERED_ARGS=()
  for arg in "${ARGS[@]}"; do
    case "$arg" in
      x:=*|y:=*|z:=*|yaw:=*)
        ;;
      *)
        FILTERED_ARGS+=("$arg")
        ;;
    esac
  done
  ARGS=("${FILTERED_ARGS[@]}")
fi

ORIG_ROS_DOMAIN_ID="${ROS_DOMAIN_ID-}"

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

if [ -n "$ORIG_ROS_DOMAIN_ID" ]; then
  export ROS_DOMAIN_ID="$ORIG_ROS_DOMAIN_ID"
fi

if [ "$NEXT_TO_UGV" = "true" ]; then
  if UAV_SPAWN_ENV="$(slam_state_capture_uav_spawn_from_ugv_env "$WS_ROOT" "$WORLD" "$UAV_BODY_X_OFFSET" "$UAV_BODY_Y_OFFSET" "$UAV_Z" "$POSE_TIMEOUT_S")"; then
    eval "$UAV_SPAWN_ENV"
    ARGS+=("x:=$uav_x" "y:=$uav_y" "z:=$uav_z" "yaw:=$uav_yaw")
    echo "[run_spawn_uav] Using UGV-relative spawn x=${uav_x} y=${uav_y} z=${uav_z} yaw=${uav_yaw}"
  else
    echo "[run_spawn_uav] Failed to read UGV pose for next_to_ugv:=true" >&2
    exit 1
  fi
fi

gazebo_world_name() {
  case "$1" in
    construction)
      printf '%s\n' "office_construction"
      ;;
    *)
      printf '%s\n' "$1"
      ;;
  esac
}

wait_for_gazebo_ready() {
  local gz_world
  local deadline
  gz_world="$(gazebo_world_name "$WORLD")"
  deadline=$((SECONDS + GAZEBO_READY_TIMEOUT_S))

  echo "[run_spawn_uav] Waiting for Gazebo clock and /world/${gz_world}/create"
  while (( SECONDS < deadline )); do
    if timeout 4s ros2 topic echo --no-daemon --once /clock >/dev/null 2>&1; then
      echo "[run_spawn_uav] Gazebo clock is publishing."
      break
    fi
    sleep 2
  done
  if (( SECONDS >= deadline )); then
    echo "[run_spawn_uav] Timed out waiting for /clock after ${GAZEBO_READY_TIMEOUT_S}s" >&2
    return 1
  fi

  while (( SECONDS < deadline )); do
    if command -v gz >/dev/null 2>&1 && timeout 4s gz service -l 2>/dev/null | grep -qx "/world/${gz_world}/create"; then
      echo "[run_spawn_uav] Gazebo create service is ready."
      return 0
    fi
    sleep 2
  done

  echo "[run_spawn_uav] Timed out waiting for /world/${gz_world}/create after ${GAZEBO_READY_TIMEOUT_S}s" >&2
  return 1
}

case "$WAIT_FOR_GAZEBO" in
  true)
    wait_for_gazebo_ready
    ;;
  false)
    ;;
  *)
    echo "[run_spawn_uav] Invalid wait_for_gazebo option: $WAIT_FOR_GAZEBO (use true or false)" >&2
    exit 2
    ;;
esac

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

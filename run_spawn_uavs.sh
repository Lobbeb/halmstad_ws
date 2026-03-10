#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
FOLLOW_SIM=false
LAUNCH_PID=""
WATCH_PID=""

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
  echo "[run_spawn_uavs] Gazebo helper detected; stopping this launcher when the sim helper exits."
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

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

UAV_MODE="${UAV_MODE:-teleport}"
setsid ros2 launch lrs_halmstad spawn_uavs.launch.py world:="$WORLD" uav_mode:="$UAV_MODE" "$@" &
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

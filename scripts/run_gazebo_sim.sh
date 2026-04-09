#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD="${1:-warehouse}"
GUI="${GUI:-true}"
ENABLE_WSL_SOFTWARE_RENDERING="${ENABLE_WSL_SOFTWARE_RENDERING:-auto}"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"

if [ "$#" -gt 0 ]; then
  shift
fi

if [ "$#" -gt 0 ] && { [ "$1" = "true" ] || [ "$1" = "false" ]; }; then
  GUI="$1"
  shift
fi

cleanup() {
  if [ -f "$SIM_PID_FILE" ] && [ "$(cat "$SIM_PID_FILE" 2>/dev/null || true)" = "$$" ]; then
    rm -f "$SIM_PID_FILE"
    rm -f "$SIM_WORLD_FILE"
  fi
}

signal_processes_by_pattern() {
  local pattern="$1"
  local pids=()
  local pid=""
  while IFS= read -r pid; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    pids+=("$pid")
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
  if [ "${#pids[@]}" -eq 0 ]; then
    return 0
  fi
  kill -INT "${pids[@]}" 2>/dev/null || true
  sleep 1
  kill -TERM "${pids[@]}" 2>/dev/null || true
  sleep 1
  kill -KILL "${pids[@]}" 2>/dev/null || true
}

signal_named_nodes() {
  local names_regex="$1"
  signal_processes_by_pattern "__node:=($names_regex)(\\s|$)"
}

prelaunch_safety_cleanup() {
  rm -f "$SIM_PID_FILE" "$SIM_WORLD_FILE"
  signal_processes_by_pattern 'scripts/run_gazebo_sim\.sh'
  signal_processes_by_pattern 'scripts/run_spawn_uav\.sh'
  signal_processes_by_pattern 'scripts/run_localization\.sh'
  signal_processes_by_pattern 'scripts/run_nav2\.sh'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_1to1_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch clearpath_nav2_demos nav2\.launch\.py'
  signal_processes_by_pattern 'ros2 launch clearpath_nav2_demos localization\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad spawn_uav_1to1\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad managed_clearpath_sim\.launch\.py'
  signal_processes_by_pattern '/ros_gz_bridge/(bridge_node|parameter_bridge|image_bridge)(\\s|$)'
  signal_named_nodes 'amcl|map_server|planner_server|controller_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|route_server|lifecycle_manager_localization|lifecycle_manager_navigation|ugv_nav2_driver|ugv_amcl_to_odom|ugv_amcl_to_platform_odom|ugv_amcl_to_platform_filtered_odom|ugv_platform_odom_to_tf|uav_simulator|leader_detector|leader_tracker|leader_estimator|selected_target_filter|visual_target_estimator|follow_point_generator|follow_point_planner|visual_actuation_bridge|camera_tracker'
  signal_processes_by_pattern '(^|/)gz sim($| )'
}

trap cleanup EXIT

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash

cd "$WS_ROOT"
# Limit package discovery to the ROS workspace source tree.
# This avoids accidentally building reference repos or notes stored elsewhere
# under the workspace root.
colcon build --symlink-install --base-paths src

source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

mkdir -p "$STATE_DIR"
prelaunch_safety_cleanup
printf '%s\n' "$$" > "$SIM_PID_FILE"
printf '%s\n' "$WORLD" > "$SIM_WORLD_FILE"

# Gazebo + Ogre can crash on some WSL GPU driver stacks.
# Default to software rendering only on WSL unless explicitly disabled.
if [ "$ENABLE_WSL_SOFTWARE_RENDERING" = "auto" ]; then
  if [ -n "${WSL_INTEROP:-}" ] || grep -qi microsoft /proc/version 2>/dev/null; then
    if [ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
      export LIBGL_ALWAYS_SOFTWARE=1
      echo "[run_gazebo_sim] WSL detected: using LIBGL_ALWAYS_SOFTWARE=1 for stability"
    fi
  fi
elif [ "$ENABLE_WSL_SOFTWARE_RENDERING" = "true" ]; then
  if [ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    echo "[run_gazebo_sim] Forced software rendering: LIBGL_ALWAYS_SOFTWARE=1"
  fi
fi

ros2 launch lrs_halmstad managed_clearpath_sim.launch.py \
  world:="$WORLD" \
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath" \
  use_sim_time:=true \
  gui:="$GUI" \
  "$@"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCAL_LAUNCH="$WS_ROOT/src/lrs_halmstad/launch/nav2_semantic.launch.py"
CONTROLLER_MANAGER="/a201_0000/controller_manager"
LIST_SERVICE="${CONTROLLER_MANAGER}/list_controllers"
WAIT_TIMEOUT_S="${WAIT_TIMEOUT_S:-75}"
WAIT_SLEEP_S="${WAIT_SLEEP_S:-2}"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

controller_state() {
  local controller_name="$1"
  local response
  response="$(
    timeout 8s ros2 service call \
      "$LIST_SERVICE" \
      controller_manager_msgs/srv/ListControllers \
      "{}" 2>/dev/null || true
  )"
  printf '%s\n' "$response" \
    | sed -n "s/.*ControllerState(name='${controller_name}', state='\([^']*\)'.*/\1/p" \
    | head -n 1
}

wait_for_active_controllers() {
  local waited=0
  local js_state velocity_state

  echo "[run_nav2_semantic] Waiting for Gazebo controllers to become active..."
  while [ "$waited" -lt "$WAIT_TIMEOUT_S" ]; do
    js_state="$(controller_state joint_state_broadcaster || true)"
    velocity_state="$(controller_state platform_velocity_controller || true)"

    if [ "$js_state" = "active" ] && [ "$velocity_state" = "active" ]; then
      echo "[run_nav2_semantic] Controllers are active."
      return 0
    fi

    sleep "$WAIT_SLEEP_S"
    waited=$((waited + WAIT_SLEEP_S))
  done

  echo "[run_nav2_semantic] Warning: controllers were not both active within ${WAIT_TIMEOUT_S}s; continuing anyway." >&2
  return 0
}

wait_for_active_controllers

ros2 launch "$LOCAL_LAUNCH" \
  use_sim_time:=true \
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath" \
  "$@"

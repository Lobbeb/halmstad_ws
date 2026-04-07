#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROBOT_NAMESPACE="${1:-a201_0000}"
STARTUP_DELAY_S="${STARTUP_DELAY_S:-30}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-5}"
ATTEMPT_SLEEP_S="${ATTEMPT_SLEEP_S:-8}"
CONTROLLER_MANAGER="/${ROBOT_NAMESPACE}/controller_manager"
LIST_SERVICE="${CONTROLLER_MANAGER}/list_controllers"

log() {
  printf '[recover_sim_controllers] %s\n' "$*"
}

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

wait_for_controller_manager() {
  local waited=0
  while [ "$waited" -lt "$STARTUP_DELAY_S" ]; do
    if ros2 service type "$LIST_SERVICE" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
}

activate_controller() {
  local controller_name="$1"
  local attempt state

  for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
    state="$(controller_state "$controller_name" || true)"
    if [ "$state" = "active" ]; then
      log "$controller_name is already active"
      return 0
    fi

    log "activating $controller_name (attempt $attempt/$MAX_ATTEMPTS)"
    if ros2 run controller_manager spawner \
      --controller-manager "$CONTROLLER_MANAGER" \
      --controller-manager-timeout 60 \
      --switch-timeout 30 \
      --service-call-timeout 60 \
      "$controller_name"; then
      return 0
    fi

    sleep "$ATTEMPT_SLEEP_S"
  done

  log "failed to activate $controller_name"
  return 1
}

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

log "waiting for baylands startup to settle"
sleep "$STARTUP_DELAY_S"

if ! wait_for_controller_manager; then
  log "controller manager service did not appear in time"
  exit 0
fi

activate_controller joint_state_broadcaster || true
sleep 2
activate_controller platform_velocity_controller || true

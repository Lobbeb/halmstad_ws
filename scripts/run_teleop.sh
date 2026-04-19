#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/slam_state_common.sh"

SPEED="0.20"
TURN="0.25"
TOPIC=""
NAMESPACE="$(slam_state_namespace "$WS_ROOT")"
DRY_RUN="false"

usage() {
  cat <<EOF
Usage: ./run.sh teleop [speed:=0.20] [turn:=0.25] [namespace:=a201_0000] [topic:=/a201_0000/cmd_vel] [dry_run:=true|false]

Examples:
  ./run.sh teleop
  ./run.sh teleop speed:=0.30
  ./run.sh teleop speed:=0.25 turn:=0.15
  ./run.sh teleop topic:=/a201_0000/cmd_vel
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

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    speed:=*)
      SPEED="${arg#speed:=}"
      ;;
    turn:=*)
      TURN="${arg#turn:=}"
      ;;
    namespace:=*)
      NAMESPACE="${arg#namespace:=}"
      ;;
    topic:=*)
      TOPIC="${arg#topic:=}"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${arg#dry_run:=}")"
      ;;
    *)
      echo "[run_teleop] Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -z "$TOPIC" ]; then
  TOPIC="/${NAMESPACE}/cmd_vel"
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

if [ "$DRY_RUN" = "true" ]; then
  cat <<EOF
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p speed:=${SPEED} -p turn:=${TURN} -r cmd_vel:=${TOPIC}
EOF
  exit 0
fi

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -p speed:="${SPEED}" \
  -p turn:="${TURN}" \
  -r cmd_vel:="${TOPIC}"

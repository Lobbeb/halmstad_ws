#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/slam_state_common.sh"

WORLD=""
GUI="false"
SAFE_Z_FLOOR="0.10"
COMMAND_ONLY="false"
STANDARD_YAW="0.5"

usage() {
  cat <<'EOF'
Usage: ./run.sh current_pose [world] [gui:=true|false] [safe_z:=0.10] [standard_yaw:=0.5] [command_only:=true|false]

Examples:
  ./run.sh current_pose
  ./run.sh current_pose baylands
  ./run.sh current_pose baylands gui:=true
  ./run.sh current_pose baylands safe_z:=0.20
  ./run.sh current_pose baylands standard_yaw:=0.5
  ./run.sh current_pose baylands command_only:=true
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

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    help|-h|--help)
      usage
      exit 0
      ;;
    world:=*)
      WORLD="${1#world:=}"
      ;;
    gui:=*)
      GUI="$(coerce_bool "${1#gui:=}")"
      ;;
    safe_z:=*)
      SAFE_Z_FLOOR="${1#safe_z:=}"
      ;;
    standard_yaw:=*)
      STANDARD_YAW="${1#standard_yaw:=}"
      ;;
    command_only:=*)
      COMMAND_ONLY="$(coerce_bool "${1#command_only:=}")"
      ;;
    *)
      echo "[run_current_pose] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [ -z "$WORLD" ]; then
  WORLD="$(slam_state_default_name)"
fi

if ! POSE_ENV="$(slam_state_capture_gazebo_pose_env "$WS_ROOT" "$WORLD" 5)"; then
  echo "[run_current_pose] Failed to read current Gazebo pose for world '$WORLD'." >&2
  echo "Make sure gazebo_sim is running for that world, then try again." >&2
  exit 1
fi

eval "$POSE_ENV"

STANDARD_COMMAND="./run.sh gazebo_sim $WORLD $GUI x:=$spawn_x y:=$spawn_y z:=$spawn_z yaw:=$STANDARD_YAW"
EXACT_COMMAND="./run.sh gazebo_sim $WORLD $GUI x:=$spawn_x y:=$spawn_y z:=$spawn_z yaw:=$spawn_yaw"
SAFE_COMMAND="$STANDARD_COMMAND"
USE_SAFE_Z="false"

if awk -v current="$spawn_z" -v floor="$SAFE_Z_FLOOR" 'BEGIN { exit !(current < floor) }'; then
  SAFE_COMMAND="./run.sh gazebo_sim $WORLD $GUI x:=$spawn_x y:=$spawn_y z:=$SAFE_Z_FLOOR yaw:=$STANDARD_YAW"
  USE_SAFE_Z="true"
fi

if [ "$COMMAND_ONLY" = "true" ]; then
  printf '%s\n' "$SAFE_COMMAND"
  exit 0
fi

cat <<EOF
[run_current_pose] World: $WORLD
x:=$spawn_x
y:=$spawn_y
z:=$spawn_z
yaw:=$spawn_yaw

Relaunch (standard yaw $STANDARD_YAW):
$STANDARD_COMMAND

Exact-current-yaw relaunch / readjustment:
$EXACT_COMMAND
EOF

if [ "$USE_SAFE_Z" = "true" ]; then
  cat <<EOF

Safer relaunch (standard yaw $STANDARD_YAW):
$SAFE_COMMAND
EOF
fi

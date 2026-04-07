#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
GZ_BIN="/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"

source "$SCRIPT_DIR/slam_state_common.sh"

WORLD=""
TARGET_X=""
TARGET_Y=""
TARGET_Z=""
TARGET_YAW="0.5"
KEEP_PAUSED="false"
DRY_RUN="false"

BRIDGE_PID=""
STARTED_BRIDGE="false"
PAUSED_SIM="false"

usage() {
  cat <<'EOF'
Usage: ./run.sh realign_yaw [world] [yaw:=0.5] [keep_paused:=false] [dry_run:=false]

Examples:
  ./run.sh realign_yaw
  ./run.sh realign_yaw baylands
  ./run.sh realign_yaw yaw:=0.5
  ./run.sh realign_yaw x:=10.0 y:=20.0
  ./run.sh realign_yaw baylands x:=10.0 y:=20.0 z:=0.2 yaw:=0.0
  ./run.sh realign_yaw baylands yaw:=0.0
  ./run.sh realign_yaw dry_run:=true
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

cleanup() {
  if [ "$KEEP_PAUSED" != "true" ] && [ "$PAUSED_SIM" = "true" ] && [ "$DRY_RUN" != "true" ]; then
    "$GZ_BIN" service \
      -s "/world/${GZ_WORLD}/control" \
      --reqtype gz.msgs.WorldControl \
      --reptype gz.msgs.Boolean \
      --timeout 3000 \
      --req 'pause: false' >/dev/null 2>&1 || true
  fi

  if [ "$STARTED_BRIDGE" = "true" ] && [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill -INT "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
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
    x:=*)
      TARGET_X="${1#x:=}"
      ;;
    y:=*)
      TARGET_Y="${1#y:=}"
      ;;
    z:=*)
      TARGET_Z="${1#z:=}"
      ;;
    yaw:=*)
      TARGET_YAW="${1#yaw:=}"
      ;;
    keep_paused:=*)
      KEEP_PAUSED="$(coerce_bool "${1#keep_paused:=}")"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${1#dry_run:=}")"
      ;;
    *)
      echo "[run_realign_yaw] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [ -z "$WORLD" ]; then
  WORLD="$(slam_state_default_name)"
fi

GZ_WORLD="$(slam_state_gazebo_world_name "$WORLD")"
ENTITY_NAME="$(slam_state_robot_entity_name "$WS_ROOT")"
SERVICE_NAME="/world/${GZ_WORLD}/set_pose"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

if ! POSE_ENV="$(slam_state_capture_gazebo_pose_env "$WS_ROOT" "$WORLD" 5)"; then
  echo "[run_realign_yaw] Failed to read current Gazebo pose for world '$WORLD'." >&2
  exit 1
fi

eval "$POSE_ENV"

if [ -z "$TARGET_X" ]; then
  TARGET_X="$spawn_x"
fi
if [ -z "$TARGET_Y" ]; then
  TARGET_Y="$spawn_y"
fi
if [ -z "$TARGET_Z" ]; then
  TARGET_Z="$spawn_z"
fi

read -r TARGET_QZ TARGET_QW <<< "$(awk -v yaw="$TARGET_YAW" 'BEGIN { printf "%.9f %.9f\n", sin(yaw / 2.0), cos(yaw / 2.0) }')"

cat <<EOF
[run_realign_yaw] World: $WORLD
Entity: $ENTITY_NAME
Current pose:
  x=$spawn_x
  y=$spawn_y
  z=$spawn_z
  yaw=$spawn_yaw
Target pose:
  x=$TARGET_X
  y=$TARGET_Y
  z=$TARGET_Z
  yaw=$TARGET_YAW
  qz=$TARGET_QZ
  qw=$TARGET_QW
EOF

if [ "$DRY_RUN" = "true" ]; then
  cat <<EOF

[dry-run] Would pause:
$GZ_BIN service -s /world/${GZ_WORLD}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'

[dry-run] Would ensure bridge:
ros2 run ros_gz_bridge parameter_bridge /world/${GZ_WORLD}/set_pose@ros_gz_interfaces/srv/SetEntityPose

[dry-run] Would set pose:
ros2 service call ${SERVICE_NAME} ros_gz_interfaces/srv/SetEntityPose "{entity: {name: '${ENTITY_NAME}', type: 2}, pose: {position: {x: ${TARGET_X}, y: ${TARGET_Y}, z: ${TARGET_Z}}, orientation: {x: 0.0, y: 0.0, z: ${TARGET_QZ}, w: ${TARGET_QW}}}}"
EOF
  if [ "$KEEP_PAUSED" != "true" ]; then
    cat <<EOF

[dry-run] Would unpause:
$GZ_BIN service -s /world/${GZ_WORLD}/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
EOF
  fi
  exit 0
fi

trap cleanup EXIT INT TERM

echo "[run_realign_yaw] Pausing simulation"
"$GZ_BIN" service \
  -s "/world/${GZ_WORLD}/control" \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'pause: true' >/dev/null
PAUSED_SIM="true"

if ! slam_wait_for_service "$SERVICE_NAME" 1; then
  echo "[run_realign_yaw] Starting temporary set_pose bridge"
  ros2 run ros_gz_bridge parameter_bridge \
    "/world/${GZ_WORLD}/set_pose@ros_gz_interfaces/srv/SetEntityPose" >/dev/null 2>&1 &
  BRIDGE_PID=$!
  STARTED_BRIDGE="true"

  if ! slam_wait_for_service_or_process "$SERVICE_NAME" "$BRIDGE_PID" 10; then
    echo "[run_realign_yaw] Failed to bring up ${SERVICE_NAME}" >&2
    exit 1
  fi
fi

echo "[run_realign_yaw] Setting new yaw"
ros2 service call \
  "$SERVICE_NAME" \
  ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: '${ENTITY_NAME}', type: 2}, pose: {position: {x: ${TARGET_X}, y: ${TARGET_Y}, z: ${TARGET_Z}}, orientation: {x: 0.0, y: 0.0, z: ${TARGET_QZ}, w: ${TARGET_QW}}}}" >/dev/null

echo "[run_realign_yaw] Done"
if [ "$KEEP_PAUSED" = "true" ]; then
  echo "[run_realign_yaw] Simulation left paused on request"
  PAUSED_SIM="false"
fi

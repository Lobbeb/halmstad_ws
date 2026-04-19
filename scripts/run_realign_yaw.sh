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
TARGET_YAW="0.0"
YAW_SET="false"
WAYPOINT_NAME=""
KEEP_PAUSED="false"
DRY_RUN="false"
BAYLANDS_WAYPOINT_CSV="$WS_ROOT/maps/waypoints_baylands.csv"
BAYLANDS_GROUP_WAYPOINT_CSV="$WS_ROOT/maps/waypoints_baylands_groups.csv"

BRIDGE_PID=""
STARTED_BRIDGE="false"
PAUSED_SIM="false"

usage() {
  cat <<'EOF'
Usage: ./run.sh realign_yaw [world] [x:=...] [y:=...] [z:=...] [yaw:=0.0] [waypoint:=name] [keep_paused:=false] [dry_run:=false]

Examples:
  ./run.sh realign_yaw
  ./run.sh realign_yaw baylands
  ./run.sh realign_yaw yaw:=0.0
  ./run.sh realign_yaw x:=10.0 y:=20.0
  ./run.sh realign_yaw baylands waypoint:=art
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

resolve_baylands_waypoint() {
  local waypoint_name="$1"
  python3 - "$waypoint_name" "$BAYLANDS_GROUP_WAYPOINT_CSV" "$BAYLANDS_WAYPOINT_CSV" <<'PY'
import csv
import sys

name, *paths = sys.argv[1:]
names = []

for path in paths:
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                waypoint_name = str(row.get("place", "")).strip()
                if waypoint_name:
                    names.append(waypoint_name)
                if waypoint_name != name:
                    continue

                x_val = row.get("x")
                y_val = row.get("y")
                if x_val in (None, "") or y_val in (None, ""):
                    raise SystemExit(f"Waypoint '{name}' is missing world x/y in {path}")

                print(f"waypoint_x={float(x_val)}")
                print(f"waypoint_y={float(y_val)}")

                z_val = row.get("z")
                if z_val not in (None, ""):
                    print(f"waypoint_z={float(z_val)}")

                yaw_val = row.get("yaw")
                if yaw_val not in (None, ""):
                    print(f"waypoint_yaw={float(yaw_val)}")
                raise SystemExit(0)
    except FileNotFoundError:
        continue

available = ", ".join(dict.fromkeys(names))
raise SystemExit(f"Waypoint '{name}' was not found. Available: {available}")
PY
}

stop_bridge_process() {
  local pid="${1:-}"
  local sig=""

  if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  for sig in INT TERM KILL; do
    kill "-$sig" "$pid" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return 0
      fi
      sleep 1
    done
  done

  return 1
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
    stop_bridge_process "$BRIDGE_PID" || true
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
      YAW_SET="true"
      ;;
    waypoint:=*)
      WAYPOINT_NAME="${1#waypoint:=}"
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

if [ -n "$WAYPOINT_NAME" ]; then
  if [[ "$WORLD" != baylands* ]]; then
    echo "[run_realign_yaw] waypoint:=... is currently supported for Baylands only." >&2
    exit 2
  fi
  if [ ! -f "$BAYLANDS_GROUP_WAYPOINT_CSV" ] && [ ! -f "$BAYLANDS_WAYPOINT_CSV" ]; then
    echo "[run_realign_yaw] Baylands waypoint CSVs not found: $BAYLANDS_GROUP_WAYPOINT_CSV or $BAYLANDS_WAYPOINT_CSV" >&2
    exit 1
  fi
  WAYPOINT_ENV="$(resolve_baylands_waypoint "$WAYPOINT_NAME")" || {
    echo "[run_realign_yaw] Failed to resolve waypoint '$WAYPOINT_NAME'." >&2
    exit 1
  }
  eval "$WAYPOINT_ENV"

  if [ -z "$TARGET_X" ]; then
    TARGET_X="$waypoint_x"
  fi
  if [ -z "$TARGET_Y" ]; then
    TARGET_Y="$waypoint_y"
  fi
  if [ -z "$TARGET_Z" ] && [ -n "${waypoint_z:-}" ]; then
    TARGET_Z="$waypoint_z"
  fi
  if [ "$YAW_SET" != "true" ] && [ -n "${waypoint_yaw:-}" ]; then
    TARGET_YAW="$waypoint_yaw"
  fi
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

echo "[run_realign_yaw] Setting new pose"
ros2 service call \
  "$SERVICE_NAME" \
  ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: '${ENTITY_NAME}', type: 2}, pose: {position: {x: ${TARGET_X}, y: ${TARGET_Y}, z: ${TARGET_Z}}, orientation: {x: 0.0, y: 0.0, z: ${TARGET_QZ}, w: ${TARGET_QW}}}}" >/dev/null

if [ "$STARTED_BRIDGE" = "true" ] && [ -n "$BRIDGE_PID" ]; then
  stop_bridge_process "$BRIDGE_PID" || true
  STARTED_BRIDGE="false"
  BRIDGE_PID=""
fi

echo "[run_realign_yaw] Done"
if [ "$KEEP_PAUSED" = "true" ]; then
  echo "[run_realign_yaw] Simulation left paused on request"
  PAUSED_SIM="false"
fi

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

source "$SHARED_SCRIPTS_DIR/slam_state_common.sh"

WORLD=""
GUI="false"
SAFE_Z_FLOOR="0.10"
COMMAND_ONLY="false"
STANDARD_YAW="0.5"
SHOW_AMCL="true"

usage() {
  cat <<'EOF'
Usage: ./run.sh current_pose [world] [gui:=true|false] [safe_z:=0.10] [standard_yaw:=0.5] [command_only:=true|false] [show_amcl:=true|false]

Examples:
  ./run.sh current_pose
  ./run.sh current_pose baylands
  ./run.sh current_pose baylands gui:=true
  ./run.sh current_pose baylands safe_z:=0.20
  ./run.sh current_pose baylands standard_yaw:=0.5
  ./run.sh current_pose baylands command_only:=true
  ./run.sh current_pose baylands show_amcl:=false
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
    show_amcl:=*)
      SHOW_AMCL="$(coerce_bool "${1#show_amcl:=}")"
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

if awk -v current="$spawn_z" -v floor="$SAFE_Z_FLOOR" 'BEGIN { exit !(current < floor) }'; then
  SAFE_COMMAND="./run.sh gazebo_sim $WORLD $GUI x:=$spawn_x y:=$spawn_y z:=$SAFE_Z_FLOOR yaw:=$STANDARD_YAW"
fi

if [ "$COMMAND_ONLY" = "true" ]; then
  printf '%s\n' "$SAFE_COMMAND"
  exit 0
fi

AMCL_WARNING=""
if [ "$SHOW_AMCL" = "true" ]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$WS_ROOT/install/setup.bash"
  source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
  set -u

  AMCL_TMP="$(mktemp)"
  AMCL_ERR_TMP="$(mktemp)"
  if timeout 5s ros2 topic echo --no-daemon --once /a201_0000/amcl_pose >"$AMCL_TMP" 2>"$AMCL_ERR_TMP"; then
    if AMCL_ENV="$(python3 - "$AMCL_TMP" <<'PY'
import math
import sys
from pathlib import Path
import yaml

text = Path(sys.argv[1]).read_text(encoding="utf-8")
docs = [doc for doc in yaml.safe_load_all(text) if isinstance(doc, dict)]
if not docs:
    raise SystemExit(1)
msg = None
for doc in reversed(docs):
    pose = (((doc.get("pose") or {}).get("pose")) or {})
    position = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    if "x" in position and "y" in position and "z" in orientation and "w" in orientation:
        msg = doc
        break
if msg is None:
    raise SystemExit(1)

pose = (((msg.get("pose") or {}).get("pose")) or {})
position = pose.get("position") or {}
orientation = pose.get("orientation") or {}

x = float(position["x"])
y = float(position["y"])
qx = float(orientation.get("x", 0.0))
qy = float(orientation.get("y", 0.0))
qz = float(orientation["z"])
qw = float(orientation["w"])

# Standard quaternion -> yaw conversion.
yaw = math.atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz),
)
print(f"amcl_x={x!r}")
print(f"amcl_y={y!r}")
print(f"amcl_yaw={yaw!r}")
PY
)"; then
      eval "$AMCL_ENV"
      printf '%s\n' "$spawn_x,$spawn_y,$spawn_z,$spawn_yaw,$amcl_x,$amcl_y,$amcl_yaw"
      rm -f "$AMCL_TMP"
      rm -f "$AMCL_ERR_TMP"
      exit 0
    fi
  else
    AMCL_ERR_TEXT="$(tr '\n' ' ' < "$AMCL_ERR_TMP" | sed 's/[[:space:]]\+/ /g' | sed 's/^ //; s/ $//')"
    if [ -z "$AMCL_ERR_TEXT" ]; then
      AMCL_WARNING="[run_current_pose] AMCL pose unavailable. Make sure localization is running and /a201_0000/amcl_pose is reachable on the current ROS domain."
    else
      AMCL_WARNING="[run_current_pose] AMCL pose unavailable: $AMCL_ERR_TEXT"
    fi
  fi
  rm -f "$AMCL_TMP"
  rm -f "$AMCL_ERR_TMP"
fi

if [ -n "$AMCL_WARNING" ]; then
  printf '%s\n' "$AMCL_WARNING" >&2
  exit 1
fi

printf '%s\n' "$spawn_x,$spawn_y,$spawn_z,$spawn_yaw"

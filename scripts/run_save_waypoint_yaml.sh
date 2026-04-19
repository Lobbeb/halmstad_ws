#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/slam_state_common.sh"

NAME=""
WORLD=""
OUTPUT_PATH="$WS_ROOT/maps/captured_waypoints.yaml"
GROUP=""
FRAME_ID="map"
REPLACE="true"
DRY_RUN="false"
POSITIONAL_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./run.sh save_waypoint_yaml <name> [world] [output:=path] [group:=name] [frame_id:=map] [replace:=true|false] [dry_run:=true|false]
       ./run.sh save_waypoint_yaml [world] [output:=path] [group:=name] [frame_id:=map] [replace:=true|false] [dry_run:=true|false] name:=<name>

Examples:
  ./run.sh save_waypoint_yaml strip_test
  ./run.sh save_waypoint_yaml strip_test baylands
  ./run.sh save_waypoint_yaml strip_test output:=src/lrs_halmstad/config/baylands_waypoints.yaml
  ./run.sh save_waypoint_yaml baylands output:=maps/captured_waypoints.yaml group:=strip name:=strip_test
  ./run.sh save_waypoint_yaml strip_test output:=maps/captured_waypoints.yaml group:=strip
  ./run.sh save_waypoint_yaml strip_test dry_run:=true
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

resolve_output_path() {
  local requested_path="$1"

  if [[ "$requested_path" = /* ]]; then
    printf '%s\n' "$requested_path"
    return 0
  fi

  if [[ "$requested_path" == */* ]]; then
    printf '%s\n' "$WS_ROOT/$requested_path"
    return 0
  fi

  printf '%s\n' "$WS_ROOT/maps/$requested_path"
}

if [ "$#" -eq 0 ]; then
  usage >&2
  exit 2
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    help|-h|--help)
      usage
      exit 0
      ;;
    name:=*|row:=*)
      NAME="${1#*=}"
      ;;
    world:=*)
      WORLD="${1#world:=}"
      ;;
    output:=*)
      OUTPUT_PATH="$(resolve_output_path "${1#output:=}")"
      ;;
    group:=*)
      GROUP="${1#group:=}"
      ;;
    frame_id:=*)
      FRAME_ID="${1#frame_id:=}"
      ;;
    replace:=*)
      REPLACE="$(coerce_bool "${1#replace:=}")"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${1#dry_run:=}")"
      ;;
    *)
      if [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
        POSITIONAL_ARGS+=("$1")
      else
        echo "[run_save_waypoint_yaml] Unknown argument: $1" >&2
        usage >&2
        exit 2
      fi
      ;;
  esac
  shift
done

if [ -n "$NAME" ]; then
  case "${#POSITIONAL_ARGS[@]}" in
    0)
      ;;
    1)
      if [ -z "$WORLD" ]; then
        WORLD="${POSITIONAL_ARGS[0]}"
      else
        echo "[run_save_waypoint_yaml] Too many positional arguments." >&2
        usage >&2
        exit 2
      fi
      ;;
    *)
      echo "[run_save_waypoint_yaml] Too many positional arguments." >&2
      usage >&2
      exit 2
      ;;
  esac
else
  case "${#POSITIONAL_ARGS[@]}" in
    0)
      echo "[run_save_waypoint_yaml] Missing waypoint name." >&2
      usage >&2
      exit 2
      ;;
    1)
      NAME="${POSITIONAL_ARGS[0]}"
      ;;
    2)
      NAME="${POSITIONAL_ARGS[0]}"
      if [ -z "$WORLD" ]; then
        WORLD="${POSITIONAL_ARGS[1]}"
      else
        echo "[run_save_waypoint_yaml] Too many positional arguments." >&2
        usage >&2
        exit 2
      fi
      ;;
    *)
      echo "[run_save_waypoint_yaml] Too many positional arguments." >&2
      usage >&2
      exit 2
      ;;
  esac
fi

if [ -z "$WORLD" ]; then
  WORLD="$(slam_state_default_name)"
fi

if ! GZ_POSE_ENV="$(slam_state_capture_gazebo_pose_env "$WS_ROOT" "$WORLD" 5)"; then
  echo "[run_save_waypoint_yaml] Failed to read current Gazebo pose for world '$WORLD'." >&2
  exit 1
fi

eval "$GZ_POSE_ENV"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

NAMESPACE="$(slam_state_namespace "$WS_ROOT")" || exit 1
AMCL_TOPIC="/${NAMESPACE}/amcl_pose"
AMCL_TMP="$(mktemp)"
AMCL_ERR_TMP="$(mktemp)"

cleanup() {
  rm -f "$AMCL_TMP" "$AMCL_ERR_TMP"
}
trap cleanup EXIT

if ! timeout 5s ros2 topic echo --no-daemon --once "$AMCL_TOPIC" >"$AMCL_TMP" 2>"$AMCL_ERR_TMP"; then
  AMCL_ERR_TEXT="$(tr '\n' ' ' < "$AMCL_ERR_TMP" | sed 's/[[:space:]]\+/ /g' | sed 's/^ //; s/ $//')"
  if [ -z "$AMCL_ERR_TEXT" ]; then
    echo "[run_save_waypoint_yaml] Failed to read AMCL pose from $AMCL_TOPIC." >&2
  else
    echo "[run_save_waypoint_yaml] Failed to read AMCL pose from $AMCL_TOPIC: $AMCL_ERR_TEXT" >&2
  fi
  exit 1
fi

if ! AMCL_ENV="$(python3 - "$AMCL_TMP" <<'PY'
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
yaw = math.atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz),
)
yaw_deg = math.degrees(yaw)

print(f"amcl_x={x!r}")
print(f"amcl_y={y!r}")
print(f"amcl_yaw={yaw!r}")
print(f"amcl_yaw_deg={yaw_deg!r}")
PY
)"; then
  echo "[run_save_waypoint_yaml] Failed to parse AMCL pose from $AMCL_TOPIC." >&2
  exit 1
fi

eval "$AMCL_ENV"

mkdir -p "$(dirname "$OUTPUT_PATH")"

if [ "$DRY_RUN" = "true" ]; then
  python3 - "$NAME" "$FRAME_ID" "$GROUP" "$REPLACE" "$OUTPUT_PATH" \
    "$spawn_x" "$spawn_y" "$spawn_z" "$spawn_yaw" \
    "$amcl_x" "$amcl_y" "$amcl_yaw" "$amcl_yaw_deg" <<'PY'
import sys
import yaml

(
    name,
    frame_id,
    group,
    replace,
    output_path,
    gz_x,
    gz_y,
    gz_z,
    gz_yaw,
    amcl_x,
    amcl_y,
    amcl_yaw,
    amcl_yaw_deg,
) = sys.argv[1:14]

entry = {
    "name": name,
    "x": float(amcl_x),
    "y": float(amcl_y),
    "yaw_deg": float(amcl_yaw_deg),
    "gz_x": float(gz_x),
    "gz_y": float(gz_y),
    "gz_z": float(gz_z),
    "gz_yaw": float(gz_yaw),
    "amcl_x": float(amcl_x),
    "amcl_y": float(amcl_y),
    "amcl_yaw": float(amcl_yaw),
}
if group:
    entry["group"] = group

data = {"frame_id": frame_id, "waypoints": [entry]}
print(f"# dry-run: would write to {output_path} (replace={replace})")
print(yaml.safe_dump(data, sort_keys=False), end="")
PY
  exit 0
fi

python3 - "$OUTPUT_PATH" "$NAME" "$FRAME_ID" "$GROUP" "$REPLACE" \
  "$spawn_x" "$spawn_y" "$spawn_z" "$spawn_yaw" \
  "$amcl_x" "$amcl_y" "$amcl_yaw" "$amcl_yaw_deg" <<'PY'
import sys
from pathlib import Path
import yaml

(
    output_path,
    name,
    frame_id,
    group,
    replace,
    gz_x,
    gz_y,
    gz_z,
    gz_yaw,
    amcl_x,
    amcl_y,
    amcl_yaw,
    amcl_yaw_deg,
) = sys.argv[1:14]

path = Path(output_path)
if path.exists():
    existing = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
else:
    existing = {}

if not isinstance(existing, dict):
    raise SystemExit(f"Existing YAML at {output_path} must be a mapping with 'waypoints'")

data = dict(existing)
if "frame_id" not in data or not data["frame_id"]:
    data["frame_id"] = frame_id

waypoints = data.get("waypoints")
if waypoints is None:
    waypoints = []
elif not isinstance(waypoints, list):
    raise SystemExit(f"'waypoints' in {output_path} must be a list")

entry = {
    "name": name,
    "x": float(amcl_x),
    "y": float(amcl_y),
    "yaw_deg": float(amcl_yaw_deg),
    "gz_x": float(gz_x),
    "gz_y": float(gz_y),
    "gz_z": float(gz_z),
    "gz_yaw": float(gz_yaw),
    "amcl_x": float(amcl_x),
    "amcl_y": float(amcl_y),
    "amcl_yaw": float(amcl_yaw),
}
if group:
    entry["group"] = group

if replace.lower() == "true":
    waypoints = [wp for wp in waypoints if not (isinstance(wp, dict) and str(wp.get("name", "")).strip() == name)]

waypoints.append(entry)
data["waypoints"] = waypoints

path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY

printf '%s\n' "[run_save_waypoint_yaml] Saved waypoint '$NAME' to $OUTPUT_PATH"

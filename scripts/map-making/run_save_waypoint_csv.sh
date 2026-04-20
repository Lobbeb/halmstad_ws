#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

source "$SHARED_SCRIPTS_DIR/slam_state_common.sh"

NAME=""
WORLD=""
OUTPUT_PATH="$WS_ROOT/maps/waypoints_baylands.csv"
GROUP=""
REPLACE="true"
DRY_RUN="false"
POSITIONAL_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./run.sh save_waypoint_csv <name> [world] [output:=path] [group:=name] [replace:=true|false] [dry_run:=true|false]
       ./run.sh save_waypoint_csv [world] [output:=path] [group:=name] [replace:=true|false] [dry_run:=true|false] name:=<name>

Examples:
  ./run.sh save_waypoint_csv strip_test
  ./run.sh save_waypoint_csv strip_test baylands
  ./run.sh save_waypoint_csv strip_test output:=waypoints_baylands_groups.csv group:=strip
  ./run.sh save_waypoint_csv baylands output:=waypoints_baylands_groups.csv group:=strip name:=strip_test
  ./run.sh save_waypoint_csv strip_test output:=maps/waypoints_baylands.csv
  ./run.sh save_waypoint_csv strip_test dry_run:=true
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
        echo "[run_save_waypoint_csv] Unknown argument: $1" >&2
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
        echo "[run_save_waypoint_csv] Too many positional arguments." >&2
        usage >&2
        exit 2
      fi
      ;;
    *)
      echo "[run_save_waypoint_csv] Too many positional arguments." >&2
      usage >&2
      exit 2
      ;;
  esac
else
  case "${#POSITIONAL_ARGS[@]}" in
    0)
      echo "[run_save_waypoint_csv] Missing waypoint name." >&2
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
        echo "[run_save_waypoint_csv] Too many positional arguments." >&2
        usage >&2
        exit 2
      fi
      ;;
    *)
      echo "[run_save_waypoint_csv] Too many positional arguments." >&2
      usage >&2
      exit 2
      ;;
  esac
fi

if [ -z "$WORLD" ]; then
  WORLD="$(slam_state_default_name)"
fi

if ! GZ_POSE_ENV="$(slam_state_capture_gazebo_pose_env "$WS_ROOT" "$WORLD" 5)"; then
  echo "[run_save_waypoint_csv] Failed to read current Gazebo pose for world '$WORLD'." >&2
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
    echo "[run_save_waypoint_csv] Failed to read AMCL pose from $AMCL_TOPIC." >&2
  else
    echo "[run_save_waypoint_csv] Failed to read AMCL pose from $AMCL_TOPIC: $AMCL_ERR_TEXT" >&2
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

print(f"amcl_x={x!r}")
print(f"amcl_y={y!r}")
print(f"amcl_yaw={yaw!r}")
PY
)"; then
  echo "[run_save_waypoint_csv] Failed to parse AMCL pose from $AMCL_TOPIC." >&2
  exit 1
fi

eval "$AMCL_ENV"

mkdir -p "$(dirname "$OUTPUT_PATH")"

python3 - "$OUTPUT_PATH" "$NAME" "$GROUP" "$REPLACE" "$DRY_RUN" \
  "$spawn_x" "$spawn_y" "$spawn_z" "$spawn_yaw" \
  "$amcl_x" "$amcl_y" "$amcl_yaw" <<'PY'
import csv
import sys
from pathlib import Path

(
    output_path,
    name,
    group,
    replace,
    dry_run,
    gz_x,
    gz_y,
    gz_z,
    gz_yaw,
    amcl_x,
    amcl_y,
    amcl_yaw,
) = sys.argv[1:13]

path = Path(output_path)

base_headers = ["place", "x", "y", "z", "yaw", "amcl_x", "amcl_y", "amcl_yaw"]
rows = []
headers = list(base_headers)

if path.exists():
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames:
            headers = list(reader.fieldnames)
        for row in reader:
            rows.append(dict(row))

if group and "group" not in headers:
    headers.append("group")

for required in base_headers:
    if required not in headers:
        headers.append(required)

entry = {
    "place": name,
    "x": gz_x,
    "y": gz_y,
    "z": gz_z,
    "yaw": gz_yaw,
    "amcl_x": amcl_x,
    "amcl_y": amcl_y,
    "amcl_yaw": amcl_yaw,
}
if "group" in headers:
    entry["group"] = group

if replace.lower() == "true":
    replaced = False
    new_rows = []
    for row in rows:
        if str(row.get("place", "")).strip() == name and not replaced:
            merged = dict(row)
            merged.update(entry)
            new_rows.append(merged)
            replaced = True
        else:
            new_rows.append(row)
    rows = new_rows
    if not replaced:
        rows.append(entry)
else:
    rows.append(entry)

normalized_rows = []
for row in rows:
    normalized = {header: row.get(header, "") for header in headers}
    normalized_rows.append(normalized)

if dry_run.lower() == "true":
    writer = csv.DictWriter(sys.stdout, fieldnames=headers)
    writer.writeheader()
    for row in normalized_rows[-3:]:
        writer.writerow(row)
    raise SystemExit(0)

with path.open("w", encoding="utf-8", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=headers)
    writer.writeheader()
    writer.writerows(normalized_rows)
PY

if [ "$DRY_RUN" = "true" ]; then
  exit 0
fi

printf '%s\n' "[run_save_waypoint_csv] Saved waypoint '$NAME' to $OUTPUT_PATH"

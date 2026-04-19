#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

INPUT_CSV="$WS_ROOT/maps/waypoints_baylands_groups.csv"
OUTPUT_ROUTE_YAML="$WS_ROOT/src/lrs_halmstad/config/baylands_waypoints.yaml"
OUTPUT_RVIZ_DIR="$WS_ROOT/src/lrs_halmstad/config/baylands_waypoints"
FRAME_ID="map"
DRY_RUN="false"

usage() {
  cat <<'EOF'
Usage: ./run.sh sync_waypoints_csv [input:=path] [route_output:=path] [rviz_dir:=path] [frame_id:=map] [dry_run:=true|false]

Examples:
  ./run.sh sync_waypoints_csv
  ./run.sh sync_waypoints_csv input:=waypoints_baylands_groups.csv
  ./run.sh sync_waypoints_csv route_output:=src/lrs_halmstad/config/baylands_waypoints.yaml
  ./run.sh sync_waypoints_csv rviz_dir:=src/lrs_halmstad/config/baylands_waypoints
  ./run.sh sync_waypoints_csv dry_run:=true
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

resolve_path() {
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

resolve_dir_path() {
  local requested_path="$1"

  if [[ "$requested_path" = /* ]]; then
    printf '%s\n' "$requested_path"
    return 0
  fi

  printf '%s\n' "$WS_ROOT/$requested_path"
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    help|-h|--help)
      usage
      exit 0
      ;;
    input:=*)
      INPUT_CSV="$(resolve_path "${1#input:=}")"
      ;;
    route_output:=*)
      OUTPUT_ROUTE_YAML="$(resolve_path "${1#route_output:=}")"
      ;;
    rviz_dir:=*)
      OUTPUT_RVIZ_DIR="$(resolve_dir_path "${1#rviz_dir:=}")"
      ;;
    frame_id:=*)
      FRAME_ID="${1#frame_id:=}"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${1#dry_run:=}")"
      ;;
    *)
      echo "[run_sync_waypoints_csv] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [ ! -f "$INPUT_CSV" ]; then
  echo "[run_sync_waypoints_csv] Input CSV not found: $INPUT_CSV" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT_ROUTE_YAML")"
mkdir -p "$OUTPUT_RVIZ_DIR"

python3 - "$INPUT_CSV" "$OUTPUT_ROUTE_YAML" "$OUTPUT_RVIZ_DIR" "$FRAME_ID" "$DRY_RUN" <<'PY'
import csv
import math
import re
import sys
from pathlib import Path

import yaml

input_csv = Path(sys.argv[1])
route_output = Path(sys.argv[2])
rviz_dir = Path(sys.argv[3])
frame_id = sys.argv[4]
dry_run = sys.argv[5].lower() == "true"


def slugify(value: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9]+", "_", value.strip().lower()).strip("_")
    return slug or "group"


def yaw_to_quaternion(yaw_rad: float):
    return [math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)]


rows = []
with input_csv.open("r", encoding="utf-8", newline="") as handle:
    reader = csv.DictReader(handle)
    for row in reader:
        place = str(row.get("place", "")).strip()
        if not place:
            continue

        amcl_x = row.get("amcl_x")
        amcl_y = row.get("amcl_y")
        if amcl_x in (None, "") or amcl_y in (None, ""):
            raise SystemExit(f"Row '{place}' is missing amcl_x/amcl_y in {input_csv}")

        amcl_yaw = row.get("amcl_yaw")
        if amcl_yaw in (None, ""):
            amcl_yaw = "0.0"

        group = str(row.get("group", "")).strip()

        rows.append(
            {
                "name": place,
                "group": group,
                "x": float(amcl_x),
                "y": float(amcl_y),
                "yaw_rad": float(amcl_yaw),
                "yaw_deg": math.degrees(float(amcl_yaw)),
            }
        )

if not rows:
    raise SystemExit(f"No waypoint rows found in {input_csv}")

route_waypoints = []
for row in rows:
    entry = {
        "name": row["name"],
        "x": row["x"],
        "y": row["y"],
        "yaw_deg": row["yaw_deg"],
    }
    if row["group"]:
        entry["group"] = row["group"]
    route_waypoints.append(entry)

route_doc = {"frame_id": frame_id, "waypoints": route_waypoints}

grouped = {}
for row in rows:
    if row["group"]:
        grouped.setdefault(row["group"], []).append(row)

rviz_docs = {}

def build_rviz_doc(selected_rows):
    waypoints = {}
    for idx, row in enumerate(selected_rows):
        waypoints[f"waypoint{idx}"] = {
            "pose": [row["x"], row["y"], 0.0],
            "orientation": yaw_to_quaternion(row["yaw_rad"]),
        }
    return {"waypoints": waypoints}

rviz_docs["baylands_waypoints_rviz.yaml"] = build_rviz_doc(rows)
for group_name, selected_rows in grouped.items():
    filename = f"baylands_waypoints_{slugify(group_name)}_rviz.yaml"
    rviz_docs[filename] = build_rviz_doc(selected_rows)

if dry_run:
    print(f"# dry-run: would write route YAML to {route_output}")
    print(yaml.safe_dump(route_doc, sort_keys=False), end="")
    print(f"# dry-run: would write {len(rviz_docs)} RViz YAML file(s) to {rviz_dir}")
    for name in sorted(rviz_docs):
        print(f"#   {name}")
    raise SystemExit(0)

route_output.write_text(yaml.safe_dump(route_doc, sort_keys=False), encoding="utf-8")
for name, doc in rviz_docs.items():
    (rviz_dir / name).write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")

print(f"[run_sync_waypoints_csv] Wrote route YAML: {route_output}")
for name in sorted(rviz_docs):
    print(f"[run_sync_waypoints_csv] Wrote RViz YAML: {rviz_dir / name}")
PY

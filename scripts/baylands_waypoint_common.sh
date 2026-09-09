#!/usr/bin/env bash

BAYLANDS_WAYPOINT_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

baylands_group_waypoint_csv() {
  printf '%s/maps/waypoints_baylands_groups.csv\n' "$WS_ROOT"
}

baylands_waypoint_config_dir() {
  printf '%s/src/lrs_halmstad/config/baylands_waypoints\n' "$WS_ROOT"
}

baylands_config_dir() {
  printf '%s/src/lrs_halmstad/config\n' "$WS_ROOT"
}

baylands_all_waypoints_yaml() {
  printf '%s/baylands_waypoints.yaml\n' "$(baylands_config_dir)"
}

baylands_sync_waypoints() {
  local dry_run="${1:-false}"
  local cmd=(
    bash "$BAYLANDS_WAYPOINT_COMMON_DIR/map-making/run_sync_waypoints_csv.sh"
    "input:=$(baylands_group_waypoint_csv)"
    "route_output:=$(baylands_all_waypoints_yaml)"
    "rviz_dir:=$(baylands_waypoint_config_dir)"
    "dry_run:=$dry_run"
  )
  if [ "$dry_run" = "true" ]; then
    "${cmd[@]}" >/dev/null
    echo "[baylands_waypoints] dry-run: waypoint YAML sync would use $(baylands_group_waypoint_csv)"
    return 0
  fi
  "${cmd[@]}" >/dev/null
  echo "[baylands_waypoints] synced generated waypoint YAMLs from $(baylands_group_waypoint_csv)"
}

baylands_route_yaml_path() {
  local route="$1"
  local ws_name
  ws_name="$(basename "$WS_ROOT")"
  if [[ "$route" == "$ws_name/"* ]]; then
    route="${route#"$ws_name"/}"
  fi

  local name
  name="$(basename "$route")"
  if [[ "$name" == *.yaml ]]; then
    if [[ "$route" = /* ]]; then
      printf '%s\n' "$route"
    elif [ -f "$route" ]; then
      readlink -f "$route"
    elif [ -f "$WS_ROOT/$route" ]; then
      printf '%s/%s\n' "$WS_ROOT" "$route"
    elif [ -f "$(baylands_config_dir)/$route" ]; then
      printf '%s/%s\n' "$(baylands_config_dir)" "$route"
    elif [[ "$route" == */* ]]; then
      printf '%s/%s\n' "$WS_ROOT" "$route"
    else
      printf '%s/%s\n' "$(baylands_waypoint_config_dir)" "$route"
    fi
    return 0
  fi
  if [ "$name" = "baylands_waypoints" ]; then
    baylands_all_waypoints_yaml
    return 0
  fi
  if [ -f "$(baylands_config_dir)/$name.yaml" ]; then
    printf '%s/%s.yaml\n' "$(baylands_config_dir)" "$name"
    return 0
  fi
  printf '%s/baylands_waypoints_%s.yaml\n' "$(baylands_waypoint_config_dir)" "$name"
}

baylands_first_waypoint_for_route() {
  local route="$1"
  python3 - "$WS_ROOT" "$route" <<'PY'
import csv
import sys
from pathlib import Path

import yaml

ws_root = Path(sys.argv[1])
route = sys.argv[2]
config_dir = ws_root / "src" / "lrs_halmstad" / "config"
csv_path = ws_root / "maps" / "waypoints_baylands_groups.csv"


def route_name(value: str) -> str:
    name = Path(value).name
    if name.endswith(".yaml"):
        name = name[:-5]
    if name.startswith("baylands_waypoints_"):
        name = name[len("baylands_waypoints_") :]
    if name.endswith("_rviz"):
        name = name[:-5]
    return name


def candidate_yaml_files(value: str):
    raw = Path(value).expanduser()
    if raw.is_absolute() or "/" in value:
        yield raw if raw.is_absolute() else ws_root / raw
        return

    yield config_dir / value
    yield config_dir / "baylands_waypoints" / value
    if not value.endswith(".yaml"):
        yield config_dir / f"{value}.yaml"
        yield config_dir / "baylands_waypoints" / f"{value}.yaml"
        yield config_dir / "baylands_waypoints" / f"baylands_waypoints_{value}.yaml"


for path in candidate_yaml_files(route):
    if not path.is_file():
        continue
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    waypoints = data.get("waypoints") or []
    if isinstance(waypoints, dict):
        for name in waypoints:
            print(name)
            raise SystemExit(0)
    for waypoint in waypoints:
        if isinstance(waypoint, dict) and waypoint.get("name"):
            print(waypoint["name"])
            raise SystemExit(0)

group = route_name(route)
if csv_path.is_file():
    with csv_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("group") == group and row.get("place"):
                print(row["place"])
                raise SystemExit(0)

raise SystemExit(1)
PY
}

baylands_route_for_waypoint() {
  local waypoint="$1"
  python3 - "$WS_ROOT" "$waypoint" <<'PY'
import csv
import re
import sys
from pathlib import Path

ws_root = Path(sys.argv[1])
waypoint = sys.argv[2]
csv_path = ws_root / "maps" / "waypoints_baylands_groups.csv"

if csv_path.is_file():
    with csv_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("place") == waypoint and row.get("group"):
                print(row["group"])
                raise SystemExit(0)

match = re.match(r"^(.+)_\d+$", waypoint)
if match:
    print(match.group(1))
    raise SystemExit(0)

raise SystemExit(1)
PY
}

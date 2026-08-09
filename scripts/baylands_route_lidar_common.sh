#!/usr/bin/env bash

export BAYLANDS_ROUTE_LIDAR_OVERRIDES="true"

route_lidar_config_path() {
  printf '%s/src/lrs_halmstad/config/baylands_route_lidar.yaml\n' "$WS_ROOT"
}

route_lidar_overrides_enabled() {
  [ "${BAYLANDS_ROUTE_LIDAR_OVERRIDES:-true}" = "true" ]
}

route_lidar_arg_present() {
  local prefix="$1"
  shift
  local arg=""
  for arg in "$@"; do
    [[ "$arg" == "$prefix"* ]] && return 0
  done
  return 1
}

route_lidar_preset_args() {
  local route="$1"
  local lidar="${2:-3d}"
  shift 2 || true

  route_lidar_overrides_enabled || return 0
  [ "$lidar" = "3d" ] || return 0

  python3 - "$(route_lidar_config_path)" "$route" "$@" <<'PY'
import sys
from pathlib import Path

import yaml

config_path = Path(sys.argv[1])
route = Path(sys.argv[2]).name
if route.endswith(".yaml"):
    route = route[:-5]
if route.startswith("baylands_waypoints_"):
    route = route[len("baylands_waypoints_") :]
if route.endswith("_rviz"):
    route = route[:-5]
existing = sys.argv[3:]

if not config_path.is_file():
    raise SystemExit(0)

data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
settings = (data.get("routes") or {}).get(route) or {}

for key, value in settings.items():
    prefix = f"{key}:="
    if any(arg.startswith(prefix) for arg in existing):
        continue
    print(f"{prefix}{value}")
PY
}

route_lidar_waypoint_route() {
  local waypoint="$1"
  printf '%s\n' "${waypoint%_*}"
}

route_lidar_waypoint_args() {
  local waypoint="$1"
  shift || true

  route_lidar_overrides_enabled || return 0
  [ -n "$waypoint" ] || return 0

  python3 - "$(route_lidar_config_path)" "$waypoint" "$@" <<'PY'
import re
import sys
from pathlib import Path

import yaml

config_path = Path(sys.argv[1])
waypoint = sys.argv[2]
existing = sys.argv[3:]

if not config_path.is_file():
    raise SystemExit(0)

data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
routes = data.get("routes") or {}
waypoints = data.get("waypoints") or {}

route = re.sub(r"_[0-9]+$", "", waypoint)
settings = {}
if isinstance(routes, dict) and isinstance(routes.get(route), dict):
    settings.update(routes[route])
if isinstance(waypoints, dict) and isinstance(waypoints.get(waypoint), dict):
    settings.update(waypoints[waypoint])

for key in ("pc2ls_min_height", "pc2ls_max_height", "min_height", "max_height"):
    if key not in settings:
        continue
    out_key = key
    if out_key == "min_height":
        out_key = "pc2ls_min_height"
    elif out_key == "max_height":
        out_key = "pc2ls_max_height"
    prefix = f"{out_key}:="
    if any(arg.startswith(prefix) for arg in existing):
        continue
    print(f"{out_key}:={float(settings[key])}")
PY
}

route_lidar_pc2ls_node_name() {
  printf '%s\n' "/a201_0000/pointcloud_to_laserscan"
}

route_lidar_pc2ls_scan_topic() {
  printf '%s\n' "/a201_0000/sensors/lidar3d_0/scan_from_points"
}

route_lidar_wait_for_pc2ls_service() {
  local timeout_s="${1:-8}"
  local node="${2:-$(route_lidar_pc2ls_node_name)}"
  local service="${node}/set_parameters"
  local timeout_i="${timeout_s%.*}"
  local deadline=$((SECONDS + timeout_i))

  while [ "$SECONDS" -le "$deadline" ]; do
    if ros2 service list --no-daemon 2>/dev/null | grep -Fxq "$service"; then
      return 0
    fi
    sleep 1
  done

  return 1
}

route_lidar_set_pc2ls_param() {
  local label="$1"
  local node="$2"
  local param_name="$3"
  local param_value="$4"
  local timeout_s="${5:-8}"
  local timeout_i="${timeout_s%.*}"
  local deadline=$((SECONDS + timeout_i))
  local output=""
  local last_error=""

  while [ "$SECONDS" -le "$deadline" ]; do
    if ! route_lidar_wait_for_pc2ls_service "1" "$node"; then
      last_error="${node}/set_parameters not available"
      sleep 1
      continue
    fi

    if output="$(ros2 param set --no-daemon "$node" "$param_name" "$param_value" 2>&1)"; then
      echo "[baylands_route_lidar] Applied ${label} lidar setting: ${param_name}=${param_value}"
      return 0
    fi

    last_error="$output"
    sleep 1
  done

  echo "[baylands_route_lidar] Warning: failed to set ${node}.${param_name}=${param_value} within ${timeout_s}s" >&2
  if [ -n "$last_error" ]; then
    printf '%s\n' "$last_error" >&2
  fi
  return 1
}

route_lidar_apply_pc2ls_args() {
  local label="${1:-lidar}"
  local timeout_s="${2:-8}"
  shift 2 || true
  local node
  node="$(route_lidar_pc2ls_node_name)"

  [ "$#" -gt 0 ] || return 0

  if ! route_lidar_wait_for_pc2ls_service "$timeout_s" "$node"; then
    echo "[baylands_route_lidar] ${node}/set_parameters not available; skipping ${label} lidar settings" >&2
    return 0
  fi

  local arg param_name param_value failed
  failed=0
  for arg in "$@"; do
    [ -n "$arg" ] || continue
    param_name="${arg%%:=*}"
    param_value="${arg#*:=}"
    case "$param_name" in
      pc2ls_min_height)
        param_name="min_height"
        ;;
      pc2ls_max_height)
        param_name="max_height"
        ;;
    esac
    route_lidar_set_pc2ls_param "$label" "$node" "$param_name" "$param_value" "$timeout_s" || failed=1
  done

  return 0
}

route_lidar_wait_for_scan_once() {
  local topic="${1:-$(route_lidar_pc2ls_scan_topic)}"
  local timeout_s="${2:-3}"

  timeout "${timeout_s}s" ros2 topic echo --no-daemon --once "$topic" >/dev/null 2>&1
}

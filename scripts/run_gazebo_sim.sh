#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORLD=""
GUI="${GUI:-true}"
GUI_SET="false"
VIEW_FOLLOW_SPAWN_SET="false"
VIEW_DISTANCE_SET="false"
VIEW_HEIGHT_SET="false"
ENABLE_WSL_SOFTWARE_RENDERING="${ENABLE_WSL_SOFTWARE_RENDERING:-auto}"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
SIM_SPAWN_WAYPOINT_FILE="$STATE_DIR/gazebo_sim.spawn_waypoint"
CONTROLLER_RECOVERY_PID_FILE="$STATE_DIR/gazebo_sim.controller_recovery.pid"
SPAWN_STATE_NAME=""
WAYPOINT_NAME=""
X_SET="false"
Y_SET="false"
Z_SET="false"
YAW_SET="false"
PASSTHROUGH_ARGS=()
BAYLANDS_DEFAULT_X="0.0"
BAYLANDS_DEFAULT_Y="0.0"
BAYLANDS_DEFAULT_Z="0.8"
BAYLANDS_DEFAULT_YAW="0.0"
BAYLANDS_WAYPOINT_CSV="$WS_ROOT/maps/waypoints_baylands.csv"
BAYLANDS_GROUP_WAYPOINT_CSV="$WS_ROOT/maps/waypoints_baylands_groups.csv"

source "$SCRIPT_DIR/slam_state_common.sh"

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

                print(f"x={float(x_val)}")
                print(f"y={float(y_val)}")

                z_val = row.get("z")
                if z_val not in (None, ""):
                    print(f"z={float(z_val)}")

                yaw_val = row.get("yaw")
                if yaw_val not in (None, ""):
                    print(f"yaw={float(yaw_val)}")
                raise SystemExit(0)
    except FileNotFoundError:
        continue

available = ", ".join(dict.fromkeys(names))
raise SystemExit(f"Waypoint '{name}' was not found. Available: {available}")
PY
}

if [ "$#" -gt 0 ] && [[ "$1" != *=* ]] && [ "$1" != "true" ] && [ "$1" != "false" ]; then
  WORLD="$1"
  shift
fi

if [ "$#" -gt 0 ] && { [ "$1" = "true" ] || [ "$1" = "false" ]; }; then
  GUI="$1"
  GUI_SET="true"
  shift
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    state:=*)
      SPAWN_STATE_NAME="${1#state:=}"
      ;;
    spawn_state:=*)
      SPAWN_STATE_NAME="${1#spawn_state:=}"
      ;;
    state_name:=*)
      SPAWN_STATE_NAME="${1#state_name:=}"
      ;;
    waypoint:=*)
      WAYPOINT_NAME="${1#waypoint:=}"
      ;;
    gui:=*)
      GUI="${1#gui:=}"
      GUI_SET="true"
      ;;
    view_follow_spawn:=*)
      VIEW_FOLLOW_SPAWN_SET="true"
      PASSTHROUGH_ARGS+=("$1")
      ;;
    view_distance:=*)
      VIEW_DISTANCE_SET="true"
      PASSTHROUGH_ARGS+=("$1")
      ;;
    view_height:=*)
      VIEW_HEIGHT_SET="true"
      PASSTHROUGH_ARGS+=("$1")
      ;;
    true|false)
      if [ "$GUI_SET" = "false" ]; then
        GUI="$1"
        GUI_SET="true"
      else
        PASSTHROUGH_ARGS+=("$1")
      fi
      ;;
    x:=*)
      X_SET="true"
      PASSTHROUGH_ARGS+=("$1")
      ;;
    y:=*)
      Y_SET="true"
      PASSTHROUGH_ARGS+=("$1")
      ;;
    z:=*)
      Z_SET="true"
      PASSTHROUGH_ARGS+=("$1")
      ;;
    yaw:=*)
      YAW_SET="true"
      PASSTHROUGH_ARGS+=("$1")
      ;;
    *)
      PASSTHROUGH_ARGS+=("$1")
      ;;
  esac
  shift
done

if [ -n "$SPAWN_STATE_NAME" ] && [ -n "$WAYPOINT_NAME" ]; then
  echo "[run_gazebo_sim] Use either state:=... or waypoint:=..., not both." >&2
  exit 2
fi

if [ -n "$SPAWN_STATE_NAME" ]; then
  METADATA_PATH="$(slam_metadata_path_for_name "$WS_ROOT" "$SPAWN_STATE_NAME")"
  if [ ! -f "$METADATA_PATH" ]; then
    echo "[run_gazebo_sim] Saved SLAM state metadata not found: $METADATA_PATH" >&2
    exit 1
  fi

  # shellcheck disable=SC1090
  source "$METADATA_PATH"

  if [ -n "$WORLD" ] && [ -n "${world:-}" ] && [ "$WORLD" != "$world" ]; then
    echo "[run_gazebo_sim] Saved state '$SPAWN_STATE_NAME' belongs to world '$world', not '$WORLD'" >&2
    exit 1
  fi

  if [ -z "$WORLD" ]; then
    WORLD="${world:-}"
  fi

  if [ "$X_SET" = "false" ] && [ -n "${spawn_x:-}" ]; then
    PASSTHROUGH_ARGS+=("x:=$spawn_x")
  fi
  if [ "$Y_SET" = "false" ] && [ -n "${spawn_y:-}" ]; then
    PASSTHROUGH_ARGS+=("y:=$spawn_y")
  fi
  if [ "$Z_SET" = "false" ] && [ -n "${spawn_z:-}" ]; then
    PASSTHROUGH_ARGS+=("z:=$spawn_z")
  fi
  if [ "$YAW_SET" = "false" ] && [ -n "${spawn_yaw:-}" ]; then
    PASSTHROUGH_ARGS+=("yaw:=$spawn_yaw")
  fi

  if [ -n "${spawn_x:-}" ] && [ -n "${spawn_y:-}" ] && [ -n "${spawn_yaw:-}" ]; then
    echo "[run_gazebo_sim] Using saved spawn pose from checkpoint '$SPAWN_STATE_NAME': x=${spawn_x} y=${spawn_y} z=${spawn_z} yaw=${spawn_yaw}"
  else
    echo "[run_gazebo_sim] Warning: checkpoint '$SPAWN_STATE_NAME' has no saved spawn pose; using defaults/explicit args" >&2
  fi
fi

if [ -z "$WORLD" ]; then
  WORLD="warehouse"
fi

if [ -n "$WAYPOINT_NAME" ]; then
  if [[ "$WORLD" != baylands* ]]; then
    echo "[run_gazebo_sim] waypoint:=... is currently supported for Baylands only." >&2
    exit 2
  fi
  if [ ! -f "$BAYLANDS_GROUP_WAYPOINT_CSV" ] && [ ! -f "$BAYLANDS_WAYPOINT_CSV" ]; then
    echo "[run_gazebo_sim] Baylands waypoint CSVs not found: $BAYLANDS_GROUP_WAYPOINT_CSV or $BAYLANDS_WAYPOINT_CSV" >&2
    exit 1
  fi

  WAYPOINT_ENV="$(resolve_baylands_waypoint "$WAYPOINT_NAME")" || {
    echo "[run_gazebo_sim] Failed to resolve waypoint '$WAYPOINT_NAME'." >&2
    exit 1
  }
  eval "$WAYPOINT_ENV"

  if [ "$X_SET" = "false" ] && [ -n "${x:-}" ]; then
    PASSTHROUGH_ARGS+=("x:=$x")
    X_SET="true"
  fi
  if [ "$Y_SET" = "false" ] && [ -n "${y:-}" ]; then
    PASSTHROUGH_ARGS+=("y:=$y")
    Y_SET="true"
  fi
  if [ "$Z_SET" = "false" ]; then
    if [ -n "${z:-}" ]; then
      PASSTHROUGH_ARGS+=("z:=$z")
    else
      PASSTHROUGH_ARGS+=("z:=$BAYLANDS_DEFAULT_Z")
    fi
    Z_SET="true"
  fi
  if [ "$YAW_SET" = "false" ]; then
    if [ -n "${yaw:-}" ]; then
      PASSTHROUGH_ARGS+=("yaw:=$yaw")
    else
      PASSTHROUGH_ARGS+=("yaw:=$BAYLANDS_DEFAULT_YAW")
    fi
    YAW_SET="true"
  fi

  echo "[run_gazebo_sim] Using Baylands waypoint '$WAYPOINT_NAME': x=${x:-?} y=${y:-?} z=${z:-$BAYLANDS_DEFAULT_Z} yaw=${yaw:-$BAYLANDS_DEFAULT_YAW}"
fi

if [ "$WORLD" = "baylands" ] && \
   [ -z "$SPAWN_STATE_NAME" ] && \
   [ -z "$WAYPOINT_NAME" ] && \
   [ "$X_SET" = "false" ] && \
   [ "$Y_SET" = "false" ] && \
   [ "$Z_SET" = "false" ] && \
   [ "$YAW_SET" = "false" ]; then
  PASSTHROUGH_ARGS+=(
    "x:=$BAYLANDS_DEFAULT_X"
    "y:=$BAYLANDS_DEFAULT_Y"
    "z:=$BAYLANDS_DEFAULT_Z"
    "yaw:=$BAYLANDS_DEFAULT_YAW"
  )
  echo "[run_gazebo_sim] Using Baylands default origin spawn: x=${BAYLANDS_DEFAULT_X} y=${BAYLANDS_DEFAULT_Y} z=${BAYLANDS_DEFAULT_Z} yaw=${BAYLANDS_DEFAULT_YAW}"
fi

if [ "$WORLD" = "baylands" ] && [ "$GUI" = "true" ]; then
  if [ "$VIEW_FOLLOW_SPAWN_SET" = "false" ]; then
    PASSTHROUGH_ARGS+=("view_follow_spawn:=true")
  fi
  if [ "$VIEW_DISTANCE_SET" = "false" ]; then
    PASSTHROUGH_ARGS+=("view_distance:=10.0")
  fi
  if [ "$VIEW_HEIGHT_SET" = "false" ]; then
    PASSTHROUGH_ARGS+=("view_height:=10.0")
  fi
  echo "[run_gazebo_sim] Baylands GUI follow camera defaults: x offset=-10.0, z offset=10.0"
elif [ "$GUI" = "true" ] && [ "$VIEW_FOLLOW_SPAWN_SET" = "false" ]; then
  PASSTHROUGH_ARGS+=("view_follow_spawn:=true")
  echo "[run_gazebo_sim] GUI enabled: starting camera near the UGV spawn pose"
fi

cleanup() {
  if [ -f "$SIM_PID_FILE" ] && [ "$(cat "$SIM_PID_FILE" 2>/dev/null || true)" = "$$" ]; then
    if [ -f "$CONTROLLER_RECOVERY_PID_FILE" ]; then
      kill "$(cat "$CONTROLLER_RECOVERY_PID_FILE" 2>/dev/null || true)" 2>/dev/null || true
      rm -f "$CONTROLLER_RECOVERY_PID_FILE"
    fi
    rm -f "$SIM_PID_FILE"
    rm -f "$SIM_WORLD_FILE"
    rm -f "$SIM_SPAWN_WAYPOINT_FILE"
  fi
}

trap cleanup EXIT

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash

cd "$WS_ROOT"
# Limit package discovery to the ROS workspace source tree.
# This avoids accidentally building reference repos or notes stored elsewhere
# under the workspace root.
colcon build --symlink-install --base-paths src

source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

mkdir -p "$STATE_DIR"
printf '%s\n' "$$" > "$SIM_PID_FILE"
printf '%s\n' "$WORLD" > "$SIM_WORLD_FILE"
if [ -n "$WAYPOINT_NAME" ]; then
  printf '%s\n' "$WAYPOINT_NAME" > "$SIM_SPAWN_WAYPOINT_FILE"
else
  rm -f "$SIM_SPAWN_WAYPOINT_FILE"
fi

if [ "$WORLD" = "baylands" ]; then
  "$SCRIPT_DIR/recover_sim_controllers.sh" a201_0000 &
  printf '%s\n' "$!" > "$CONTROLLER_RECOVERY_PID_FILE"
fi

# Gazebo + Ogre can crash on some WSL GPU driver stacks.
# Default to software rendering only on WSL unless explicitly disabled.
if [ "$ENABLE_WSL_SOFTWARE_RENDERING" = "auto" ]; then
  if [ -n "${WSL_INTEROP:-}" ] || grep -qi microsoft /proc/version 2>/dev/null; then
    if [ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
      export LIBGL_ALWAYS_SOFTWARE=1
      echo "[run_gazebo_sim] WSL detected: using LIBGL_ALWAYS_SOFTWARE=1 for stability"
    fi
  fi
elif [ "$ENABLE_WSL_SOFTWARE_RENDERING" = "true" ]; then
  if [ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    echo "[run_gazebo_sim] Forced software rendering: LIBGL_ALWAYS_SOFTWARE=1"
  fi
fi

ros2 launch lrs_halmstad managed_clearpath_sim.launch.py \
  world:="$WORLD" \
  setup_path:="$WS_ROOT/src/lrs_halmstad/clearpath" \
  use_sim_time:=true \
  gui:="$GUI" \
  "${PASSTHROUGH_ARGS[@]}"

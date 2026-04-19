#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BASE_RVIZ_CONFIG="$WS_ROOT/src/lrs_halmstad/config/nav2_namespaced_waypoints.rviz"

source "$SCRIPT_DIR/lidar_mode_common.sh"

lidar_mode_parse_args 2d "$@"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

RVIZ_VIEW_X=""
RVIZ_VIEW_Y=""
AMCL_TMP="$(mktemp)"
AMCL_ERR_TMP="$(mktemp)"
if timeout 2s ros2 topic echo --no-daemon --once /a201_0000/amcl_pose >"$AMCL_TMP" 2>"$AMCL_ERR_TMP"; then
  if AMCL_VIEW_ENV="$(python3 - "$AMCL_TMP" <<'PY'
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
    if "x" in position and "y" in position:
        msg = doc
        break

if msg is None:
    raise SystemExit(1)

pose = (((msg.get("pose") or {}).get("pose")) or {})
position = pose.get("position") or {}
x = float(position["x"])
y = float(position["y"])
print(f"RVIZ_VIEW_X={x!r}")
print(f"RVIZ_VIEW_Y={y!r}")
PY
)"; then
    eval "$AMCL_VIEW_ENV"
    echo "[run_nav2_rviz] Centering RViz view on AMCL pose: x=$RVIZ_VIEW_X y=$RVIZ_VIEW_Y"
  fi
fi
rm -f "$AMCL_TMP" "$AMCL_ERR_TMP"

RVIZ_CONFIG="$BASE_RVIZ_CONFIG"
if [ "$LIDAR_MODE" = "2d" ] || [ "$LIDAR_SCAN_TOPIC" != "$(lidar_mode_raw_scan_topic 3d)" ] || [ -n "$RVIZ_VIEW_X" ] || [ -n "$RVIZ_VIEW_Y" ]; then
  RVIZ_SCAN_TOPIC="$LIDAR_SCAN_TOPIC"
  if [[ "$RVIZ_SCAN_TOPIC" == /a201_0000/* ]]; then
    RVIZ_SCAN_TOPIC="<robot_namespace>/${RVIZ_SCAN_TOPIC#/a201_0000/}"
  fi
  mkdir -p /tmp/halmstad_ws
  RVIZ_CONFIG="/tmp/halmstad_ws/nav2_namespaced_waypoints.$(echo "$LIDAR_MODE" | tr -cd '[:alnum:]').rviz"
  RVIZ_SCAN_TOPIC="$RVIZ_SCAN_TOPIC" RVIZ_VIEW_X="$RVIZ_VIEW_X" RVIZ_VIEW_Y="$RVIZ_VIEW_Y" \
    python3 - "$BASE_RVIZ_CONFIG" "$RVIZ_CONFIG" <<'PY'
import os
import re
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
text = src.read_text(encoding="utf-8")

scan_topic = os.environ.get("RVIZ_SCAN_TOPIC", "")
if scan_topic:
    text = text.replace("<robot_namespace>/sensors/lidar2d_0/scan", scan_topic)
    text = text.replace("<robot_namespace>/sensors/lidar3d_0/scan_from_points", scan_topic)
    text = text.replace("<robot_namespace>/sensors/lidar3d_0/scan", scan_topic)

view_x = os.environ.get("RVIZ_VIEW_X", "").strip()
view_y = os.environ.get("RVIZ_VIEW_Y", "").strip()
if view_x and view_y:
    marker = "      Name: Current View\n"
    start = text.find(marker)
    if start != -1:
        start += len(marker)
        end = text.find("    Saved: ~", start)
        if end != -1:
            block = text[start:end]
            block, x_count = re.subn(r"(?m)^      X: .*$", f"      X: {view_x}", block, count=1)
            block, y_count = re.subn(r"(?m)^      Y: .*$", f"      Y: {view_y}", block, count=1)
            if x_count and y_count:
                text = text[:start] + block + text[end:]

dst.write_text(text, encoding="utf-8")
PY
fi

if [ -n "${WSL_INTEROP:-}" ] || grep -qi microsoft /proc/version 2>/dev/null; then
  if command -v xdpyinfo >/dev/null 2>&1 && xdpyinfo -display "${DISPLAY:-:0}" >/dev/null 2>&1; then
    export QT_QPA_PLATFORM=xcb
    unset WAYLAND_DISPLAY
    echo "[run_nav2_rviz] WSL detected: using QT_QPA_PLATFORM=xcb"
  else
    echo "[run_nav2_rviz] Warning: X11 display '${DISPLAY:-:0}' is not reachable. WSLg may need a reset with 'wsl --shutdown'." >&2
  fi
fi

ros2 launch nav2_bringup rviz_launch.py \
  namespace:=a201_0000 \
  use_namespace:=true \
  use_sim_time:=true \
  rviz_config:="$RVIZ_CONFIG" \
  "${LIDAR_REMAINING_ARGS[@]}"

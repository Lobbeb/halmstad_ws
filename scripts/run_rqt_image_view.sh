#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TOPIC="${1:-}"

if [ "$#" -gt 0 ]; then
  shift
fi

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

if [ -n "${WSL_INTEROP:-}" ] || grep -qi microsoft /proc/version 2>/dev/null; then
  if command -v xdpyinfo >/dev/null 2>&1 && xdpyinfo -display "${DISPLAY:-:0}" >/dev/null 2>&1; then
    export QT_QPA_PLATFORM=xcb
    unset WAYLAND_DISPLAY
    echo "[run_rqt_image_view] WSL detected: using QT_QPA_PLATFORM=xcb"
  else
    echo "[run_rqt_image_view] Warning: X11 display '${DISPLAY:-:0}' is not reachable. WSLg may need a reset with 'wsl --shutdown'." >&2
  fi
fi

ARGS=()
if [ -n "$TOPIC" ]; then
  ARGS+=("$TOPIC")
fi

ros2 run rqt_image_view rqt_image_view "${ARGS[@]}" "$@"

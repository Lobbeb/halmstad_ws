#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_PERSPECTIVE="$WS_ROOT/perspectives/monitor_UGVUAV_with_YOLO"
PERSPECTIVE="${1:-$DEFAULT_PERSPECTIVE}"

if [ "$#" -gt 0 ]; then
  shift
fi

if [[ "$PERSPECTIVE" != /* ]]; then
  if [ -f "$WS_ROOT/perspectives/$PERSPECTIVE" ]; then
    PERSPECTIVE="$WS_ROOT/perspectives/$PERSPECTIVE"
  fi
fi

if [ ! -f "$PERSPECTIVE" ]; then
  echo "Perspective file not found: $PERSPECTIVE" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 run rqt_gui rqt_gui --perspective-file "$PERSPECTIVE" "$@"

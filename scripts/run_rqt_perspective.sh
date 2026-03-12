#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PERSPECTIVE_DIR="$WS_ROOT/perspectives"
DEFAULT_PERSPECTIVE="$PERSPECTIVE_DIR/YOLO.perspective"
PERSPECTIVE="${1:-$DEFAULT_PERSPECTIVE}"

resolve_perspective() {
  local candidate="$1"
  local stem=""

  if [ -f "$candidate" ]; then
    printf '%s\n' "$candidate"
    return 0
  fi

  if [ -f "$PERSPECTIVE_DIR/$candidate" ]; then
    printf '%s\n' "$PERSPECTIVE_DIR/$candidate"
    return 0
  fi

  stem="${candidate##*/}"
  if [ -f "$PERSPECTIVE_DIR/$stem" ]; then
    printf '%s\n' "$PERSPECTIVE_DIR/$stem"
    return 0
  fi

  if [ -f "$PERSPECTIVE_DIR/${candidate}.perspective" ]; then
    printf '%s\n' "$PERSPECTIVE_DIR/${candidate}.perspective"
    return 0
  fi

  if [ -f "$PERSPECTIVE_DIR/${stem}.perspective" ]; then
    printf '%s\n' "$PERSPECTIVE_DIR/${stem}.perspective"
    return 0
  fi

  return 1
}

print_available() {
  echo "Available perspectives in $PERSPECTIVE_DIR:" >&2
  find "$PERSPECTIVE_DIR" -maxdepth 1 -type f -printf '  %f\n' | sort >&2
}

if [ "$#" -gt 0 ]; then
  shift
fi

if [ "$PERSPECTIVE" = "list" ] || [ "$PERSPECTIVE" = "--list" ]; then
  print_available
  exit 0
fi

if ! PERSPECTIVE="$(resolve_perspective "$PERSPECTIVE")"; then
  echo "Perspective file not found: $PERSPECTIVE" >&2
  print_available
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 run rqt_gui rqt_gui --perspective-file "$PERSPECTIVE" "$@"

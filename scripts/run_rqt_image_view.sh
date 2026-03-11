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

ARGS=()
if [ -n "$TOPIC" ]; then
  ARGS+=("$TOPIC")
fi

ros2 run rqt_image_view rqt_image_view "${ARGS[@]}" "$@"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

exec ros2 run lrs_halmstad run_follow_debug "$@"

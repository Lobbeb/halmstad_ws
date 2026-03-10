#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

exec python3 "$WS_ROOT/src/lrs_halmstad/lrs_halmstad/run_follow_control.py" "$@"

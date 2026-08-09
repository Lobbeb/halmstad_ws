#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh follow_control [options...]

Interactive follow-controller helper for changing follow parameters at runtime.
All options are forwarded to: ros2 run lrs_halmstad run_follow_control

Common:
  ./run.sh follow_control
  ./run.sh follow_control --help
EOF
}

case "${1:-}" in
  help|-h|--help)
    usage
    exit 0
    ;;
esac

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

exec ros2 run lrs_halmstad run_follow_control "$@"

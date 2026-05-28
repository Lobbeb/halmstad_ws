#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh collect_leader_dataset [options...]

Run the leader dataset collection CLI.
Arguments are forwarded to: ros2 run lrs_halmstad collect_leader_dataset

Examples:
  ./run.sh collect_leader_dataset --help
  ./run.sh collect_leader_dataset --input DIR --output DIR
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

export LRS_HALMSTAD_WS_ROOT="$WS_ROOT"

exec ros2 run lrs_halmstad collect_leader_dataset "$@"

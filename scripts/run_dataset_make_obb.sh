#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh dataset_make_obb [options...]

Run the dataset OBB conversion helper.
Arguments are forwarded to: ros2 run lrs_halmstad run_dataset_make_obb

Examples:
  ./run.sh dataset_make_obb --help
  ./run.sh dataset_make_obb --input DATASET_DIR --output OUT_DIR
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

exec ros2 run lrs_halmstad run_dataset_make_obb "$@"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

case "${1:-}" in
  help|-h|--help)
    exec "$SHARED_SCRIPTS_DIR/run_nav2_rviz.sh" --help
    ;;
esac

exec "$SHARED_SCRIPTS_DIR/run_nav2_rviz.sh" "$@"

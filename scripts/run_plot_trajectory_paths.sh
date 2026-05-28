#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "${1:-}" in
  help|-h|--help)
    exec "$SCRIPT_DIR/plot_trajectory_paths.py" --help
    ;;
esac

exec "$SCRIPT_DIR/plot_trajectory_paths.py" "$@"

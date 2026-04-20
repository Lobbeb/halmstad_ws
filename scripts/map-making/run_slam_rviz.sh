#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

exec "$SHARED_SCRIPTS_DIR/run_nav2_rviz.sh" "$@"

#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-orchard}"

exec bash "${WS_ROOT}/run_sim_orchard.sh" "${WORLD}"

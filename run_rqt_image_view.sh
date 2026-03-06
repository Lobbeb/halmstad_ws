#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UAV_NAME="${1:-dji0}"
VIEW_KIND="${2:-debug}"

case "${VIEW_KIND}" in
  raw)
    TOPIC="/${UAV_NAME}/camera0/image_raw"
    ;;
  debug)
    TOPIC="/coord/leader_debug_image"
    ;;
  *)
    echo "[run_rqt_image_view] usage: bash run_rqt_image_view.sh [uav_name] [raw|debug]"
    exit 2
    ;;
esac

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  source "${WS_ROOT}/install/setup.bash"
fi
if [[ -f "${WS_ROOT}/src/lrs_halmstad/clearpath/setup.bash" ]]; then
  source "${WS_ROOT}/src/lrs_halmstad/clearpath/setup.bash"
fi
set -u

echo "[run_rqt_image_view] topic=${TOPIC}"
exec ros2 run rqt_image_view rqt_image_view "${TOPIC}"

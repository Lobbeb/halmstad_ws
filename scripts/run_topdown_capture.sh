#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

TOPIC="/baylands/topdown/image"
OUTPUT=""
TIMEOUT_S="20"

while [ "$#" -gt 0 ]; do
  case "$1" in
    topic:=*)
      TOPIC="${1#topic:=}"
      ;;
    output:=*)
      OUTPUT="${1#output:=}"
      ;;
    timeout_s:=*)
      TIMEOUT_S="${1#timeout_s:=}"
      ;;
    *)
      echo "[run_topdown_capture] Unknown argument: $1" >&2
      echo "Usage: ./run.sh topdown_capture [topic:=/baylands/topdown/image] [output:=/abs/path.png] [timeout_s:=20]" >&2
      exit 1
      ;;
  esac
  shift
done

if [ -z "$OUTPUT" ]; then
  OUTPUT="$WS_ROOT/maps/baylands_topdown_$(date +%Y%m%d_%H%M%S).png"
fi

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

python3 "$SCRIPT_DIR/save_ros_image.py" \
  --topic "$TOPIC" \
  --output "$OUTPUT" \
  --timeout "$TIMEOUT_S"

echo "[run_topdown_capture] Saved top-down image to $OUTPUT"

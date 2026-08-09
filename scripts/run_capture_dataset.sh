#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
WORLD="baylands"
OUTPUT_DIR=""
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  ./run.sh capture_dataset [world] [arg:=value ...]

Start the simulation dataset capture node.

Common arguments:
  world                 Dataset/world name. Default: current gazebo world file, else baylands.
  out:=PATH             Output directory. Default: datasets/<world>_auto
  class:=NAME           Class name parameter.
  id:=INT               Class id parameter.
  hz:=FLOAT             Capture rate in Hz.
  negatives:=true|false Save negative examples.

Any other arg:=value is forwarded as a ROS parameter.

Examples:
  ./run.sh capture_dataset baylands out:=datasets/baylands_auto class:=leader id:=0 hz:=2
  ./run.sh capture_dataset warehouse negatives:=true
EOF
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
  esac
done

if [ -f "$SIM_WORLD_FILE" ]; then
  sim_world="$(cat "$SIM_WORLD_FILE" 2>/dev/null || true)"
  if [ -n "$sim_world" ]; then
    WORLD="$sim_world"
  fi
fi

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

OUTPUT_DIR="$WS_ROOT/datasets/${WORLD}_auto"

for arg in "$@"; do
  case "$arg" in
    out:=*)
      OUTPUT_DIR="${arg#out:=}"
      ;;
    class:=*)
      EXTRA_ARGS+=("-p" "class_name:=${arg#class:=}")
      ;;
    id:=*)
      EXTRA_ARGS+=("-p" "class_id:=${arg#id:=}")
      ;;
    hz:=*)
      EXTRA_ARGS+=("-p" "capture_hz:=${arg#hz:=}")
      ;;
    negatives:=*)
      EXTRA_ARGS+=("-p" "save_negative_examples:=${arg#negatives:=}")
      ;;
    *)
      EXTRA_ARGS+=("-p" "$arg")
      ;;
  esac
done

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 run lrs_halmstad sim_dataset_capture --ros-args \
  -p use_sim_time:=true \
  -p output_dir:="$OUTPUT_DIR" \
  -p dataset_name:="$WORLD" \
  "${EXTRA_ARGS[@]}"

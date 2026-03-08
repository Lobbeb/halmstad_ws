#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXTRA_ARGS=()

for arg in "$@"; do
  case "$arg" in
    weights:=*)
      EXTRA_ARGS+=("yolo_version:=${arg#weights:=}")
      ;;
    target:=*)
      EXTRA_ARGS+=("target_class_name:=${arg#target:=}")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

ros2 launch lrs_halmstad run_round_follow_yolo.launch.py \
  yolo_version:=detection/yolo26/yolo26l.pt \
  "${EXTRA_ARGS[@]}"

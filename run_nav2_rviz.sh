#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

ros2 launch nav2_bringup rviz_launch.py \
  namespace:=a201_0000 \
  use_namespace:=true \
  use_sim_time:=true \
  rviz_config:="$WS_ROOT/src/lrs_halmstad/config/nav2_namespaced_waypoints.rviz" \
  "$@"

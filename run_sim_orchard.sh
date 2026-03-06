#!/usr/bin/env bash
set -euo pipefail

WORLD="${1:-orchard}"

# ROS setup scripts are not strictly nounset-safe; source them with +u.
set +u
source /opt/ros/jazzy/setup.bash
source ~/halmstad_ws/install/setup.bash
source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
set -u

exec ros2 launch clearpath_gz simulation.launch.py world:="$WORLD" use_sim_time:=true gui:=true

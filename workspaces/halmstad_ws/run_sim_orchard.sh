#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source ~/halmstad_ws/install/setup.bash
source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
ros2 launch clearpath_gz simulation.launch.py world:=orchard use_sim_time:=true gui:=true

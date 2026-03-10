source /opt/ros/jazzy/setup.bash

CLEARPATH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$CLEARPATH_DIR/../../.." && pwd)"
source "$WS_ROOT/install/setup.bash"

export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

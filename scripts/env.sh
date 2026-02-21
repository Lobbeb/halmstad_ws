# Usage: source scripts/env.sh
# Safe to source (does not exit your shell)

# Source ROS
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "ERROR: /opt/ros/jazzy/setup.bash not found"
  return 1
fi

# Workspace root (absolute)
export WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source overlay if built
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
else
  echo "WARN: $WS_ROOT/install/setup.bash not found. Run colcon build."
fi

# Keep DDS settings aligned with clearpath/robot.yaml defaults.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

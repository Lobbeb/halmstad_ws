#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh node_audit

Print a compact audit of active ROS nodes and topics:
  - required platform / Nav2 / localization nodes
  - optional nodes that should be absent in minimal runs
  - bridge, camera, lidar, follow-debug, and clock topics

No arguments are required.
EOF
}

case "${1:-}" in
  help|-h|--help)
    usage
    exit 0
    ;;
esac

set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
fi
set -u

node_list="$(mktemp)"
topic_list="$(mktemp)"
trap 'rm -f "$node_list" "$topic_list"' EXIT

ros2 node list --no-daemon 2>/dev/null | sort > "$node_list" || true
ros2 topic list --no-daemon 2>/dev/null | sort > "$topic_list" || true

print_matches() {
  local title="$1"
  local pattern="$2"
  echo
  echo "[$title]"
  if ! grep -E "$pattern" "$node_list"; then
    echo "<none>"
  fi
}

print_topic_matches() {
  local title="$1"
  local pattern="$2"
  echo
  echo "[$title]"
  if ! grep -E "$pattern" "$topic_list"; then
    echo "<none>"
  fi
}

echo "ROS node audit"
echo "=============="

print_matches "Required motion/platform nodes" \
  '(^|/)(robot_state_publisher|controller_manager|platform_velocity_controller|joint_state_broadcaster|cmd_vel_bridge|odom_base_tf_bridge|ekf_node)$'

print_matches "Required Nav2/localization nodes" \
  '(^|/)(map_server|amcl|planner_server|controller_server|behavior_server|bt_navigator|lifecycle_manager_localization|lifecycle_manager_navigation|pointcloud_to_laserscan)$'

print_matches "Optional nodes that should be absent in profile:=minimal" \
  '(^|/)(joy_node|teleop_twist_joy_node|twist_server_node|camera_0_static_tf|camera_0_gz_bridge|camera_0_gz_image_bridge|camera_0_gz_depth_bridge|follow_uav|camera_tracker|uav_simulator|rviz2|rqt_gui)$'

print_matches "Bridge nodes" \
  '(_bridge$|parameter_bridge|image_bridge)'

print_topic_matches "Camera/depth topics" \
  '/a201_0000/sensors/camera_0/'

print_topic_matches "UAV follow/camera debug topics" \
  '^/dji[0-9]+/(camera[0-9]+|follow)/(target|actual|error|debug)/|^/dji[0-9]+/camera[0-9]+/target_|^/coord/.+(debug|status|fault|events)'

print_topic_matches "Lidar topics" \
  '/a201_0000/sensors/lidar(2d|3d)_0/|/a201_0000/.*/scan_from_points|/a201_0000/scan'

print_topic_matches "Clock topics" \
  '^/clock(_raw)?$'

echo
echo "[All nodes]"
cat "$node_list"

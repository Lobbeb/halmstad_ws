#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="/tmp/halmstad_ws"
TMUX_STATE_DIR="$STATE_DIR/tmux_sessions"
DRY_RUN=false
KILL_TMUX_SESSIONS=true

usage() {
  cat <<'EOF'
Usage: ./run.sh kill_all_ros2 [dry_run:=true|false] [kill_tmux_sessions:=true|false]

This script aggressively stops the Halmstad ROS 2 simulation stack, including:
  - nav2, localization, follow, support-chain, SLAM helpers
  - RViz and rqt
  - ros_gz_bridge / ros_gz_image
  - Gazebo / gz sim
  - lingering ros2 topic/param/service/launch/bag CLI processes

Examples:
  ./run.sh kill_all_ros2
  ./run.sh kill_all_ros2 dry_run:=true
EOF
}

coerce_bool() {
  case "$1" in
    true|false)
      printf '%s\n' "$1"
      ;;
    *)
      echo "Invalid boolean value: $1" >&2
      exit 2
      ;;
  esac
}

for arg in "$@"; do
  case "$arg" in
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${arg#dry_run:=}")"
      ;;
    kill_tmux_sessions:=*)
      KILL_TMUX_SESSIONS="$(coerce_bool "${arg#kill_tmux_sessions:=}")"
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

signal_processes_by_pattern() {
  local label="$1"
  local pattern="$2"
  local matched=""

  matched="$(pgrep -a -f "$pattern" 2>/dev/null || true)"
  [ -n "$matched" ] || return 1

  echo "[kill_all_ros2] Matched $label"
  printf '%s\n' "$matched"

  if [ "$DRY_RUN" != true ]; then
    pkill -INT -f "$pattern" 2>/dev/null || true
    sleep 1
    pkill -TERM -f "$pattern" 2>/dev/null || true
    sleep 1
    pkill -KILL -f "$pattern" 2>/dev/null || true
  fi
  return 0
}

signal_named_nodes() {
  local label="$1"
  local names_regex="$2"
  signal_processes_by_pattern "$label" "__node:=($names_regex)(\\s|$)"
}

kill_tmux_session_if_present() {
  local session="$1"
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "[kill_all_ros2] Killing tmux session $session"
    if [ "$DRY_RUN" != true ]; then
      tmux kill-session -t "$session" 2>/dev/null || true
    fi
  fi
}

kill_known_tmux_sessions() {
  [ -d "$TMUX_STATE_DIR" ] || return 0

  local state_file=""
  local session=""
  while IFS= read -r state_file; do
    [ -n "$state_file" ] || continue
    session="$(basename "$state_file" .env)"
    session="${session//_/-}"
    kill_tmux_session_if_present "$session"
  done < <(find "$TMUX_STATE_DIR" -maxdepth 1 -type f -name '*.env' | sort)

  while IFS= read -r session; do
    [ -n "$session" ] || continue
    case "$session" in
      halmstad-*|halmstad-1to1)
        kill_tmux_session_if_present "$session"
        ;;
    esac
  done < <(tmux ls 2>/dev/null | cut -d: -f1 || true)
}

cleanup_state_files() {
  echo "[kill_all_ros2] Removing runtime state files"
  if [ "$DRY_RUN" != true ]; then
    rm -f \
      "$STATE_DIR"/gazebo_sim.pid \
      "$STATE_DIR"/gazebo_sim.controller_recovery.pid \
      "$STATE_DIR"/nav2_with_map_updates.yaml \
      "$STATE_DIR"/nav2_tuning_follow_params.yaml \
      "$STATE_DIR"/slam.map_name \
      "$STATE_DIR"/slam.scan_topic
    rm -rf "$TMUX_STATE_DIR"
  fi
}

if [ "$KILL_TMUX_SESSIONS" = true ]; then
  kill_known_tmux_sessions
fi

signal_processes_by_pattern "RViz" '(^|/)(rviz2|rviz)($| )' || true
signal_processes_by_pattern "rqt" '(^|/)(rqt|rqt_gui|rqt_gui_cpp|rqt_image_view)($| )' || true
signal_processes_by_pattern "Gazebo / gz" '(^|/)(gz|gazebo|gzserver|gzclient)($| )' || true
signal_processes_by_pattern "ros_gz_bridge" 'ros_gz_bridge/(bridge_node|parameter_bridge|image_bridge)' || true
signal_processes_by_pattern "ros_gz_image" '(^|/)image_bridge($| )' || true
signal_processes_by_pattern "ros2 launch CLI" '(^|/)ros2($| ).* launch ' || true
signal_processes_by_pattern "ros2 bag CLI" '(^|/)ros2($| ).* bag ' || true
signal_processes_by_pattern "ros2 topic CLI" '(^|/)ros2($| ).* topic (echo|pub|hz|bw|info|list) ' || true
signal_processes_by_pattern "ros2 param CLI" '(^|/)ros2($| ).* param ' || true
signal_processes_by_pattern "ros2 service CLI" '(^|/)ros2($| ).* service ' || true

signal_processes_by_pattern "Gazebo helper" 'scripts/run_gazebo_sim\.sh' || true
signal_processes_by_pattern "spawn helper" 'scripts/run_spawn_uav(s)?\.sh' || true
signal_processes_by_pattern "localization helper" 'scripts/run_localization\.sh' || true
signal_processes_by_pattern "Nav2 helper" 'scripts/run_nav2\.sh' || true
signal_processes_by_pattern "Nav2 tuning helper" 'scripts/run_nav2_tuning\.sh' || true
signal_processes_by_pattern "1to1 helper" 'scripts/run_1to1_(follow|yolo)\.sh' || true
signal_processes_by_pattern "support helper" 'scripts/run_support_(follow_odom|observation|camera_scan|capture_pair)\.sh' || true
signal_processes_by_pattern "realign_yaw helper" 'scripts/run_realign_yaw\.sh' || true
signal_processes_by_pattern "route sweep helper" 'scripts/run_nav2_route_sweep\.sh' || true
signal_processes_by_pattern "waypoint localization helper" 'scripts/run_waypoint_localization\.sh' || true
signal_processes_by_pattern "SLAM helper" 'scripts/map-making/run_slam(\.sh|_save_state\.sh)?' || true

signal_processes_by_pattern "launchers" 'ros2 launch (lrs_halmstad|clearpath_nav2_demos|clearpath_control) ' || true
signal_processes_by_pattern "Clearpath generators" '(^|/)(generate_description|generate_semantic_description|generate_launch|generate_param)($| )' || true
signal_processes_by_pattern "UGV spawn entity" '__node:=spawn_entity(\s|$)' || true
signal_processes_by_pattern "Nav2 child processes" '/opt/ros/[^/]+/lib/nav2_' || true
signal_processes_by_pattern "pointcloud_to_laserscan" '(^|/)pointcloud_to_laserscan_node($| )' || true
signal_processes_by_pattern "latest_scan_relay" '(^|/)latest_scan_relay($| )' || true
signal_processes_by_pattern "robot_state_publisher" '(^|/)robot_state_publisher($| )' || true
signal_processes_by_pattern "static_transform_publisher" '(^|/)static_transform_publisher($| )' || true
signal_processes_by_pattern "joy nodes" '(^|/)(joy_node|teleop_twist_joy_node)($| )' || true
signal_processes_by_pattern "controller_manager spawners" '(^|/)(spawner|controller_manager)($| )' || true
signal_processes_by_pattern "robot_localization" '(^|/)(ekf_node|ukf_node)($| )' || true
signal_processes_by_pattern "twist mux" '(^|/)twist_mux($| )' || true
signal_processes_by_pattern "interactive marker twist server" 'interactive_marker_twist_server/marker_server' || true
signal_processes_by_pattern "OMNeT bridge sim" '(^|/)UAV_UGV($| ).*-c Communication-GazeboBridge-' || true

signal_named_nodes \
  "ROS nodes" \
  'amcl|map_server|planner_server|controller_server|collision_monitor|collision_detector|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|route_server|docking_server|lifecycle_manager_localization|lifecycle_manager_navigation|pointcloud_to_laserscan|latest_scan_relay|robot_state_publisher|spawn_entity|generate_description|generate_semantic_description|generate_launch|generate_param|joy_node|teleop_twist_joy_node|camera_0_static_tf|lidar3d_0_static_tf|ugv_nav2_driver|ugv_amcl_to_odom|ugv_amcl_to_platform_odom|ugv_amcl_to_platform_filtered_odom|ugv_platform_odom_to_tf|ugv_ground_truth_bridge|uav_simulator|follow_uav|follow_uav_odom|leader_detector|leader_tracker|leader_estimator|selected_target_filter|visual_target_estimator|follow_point_generator|follow_point_planner|visual_actuation_bridge|camera_tracker|clock_bridge|omnet_uav_pose_to_odom|omnet_tcp_bridge|omnet_metrics_bridge|support_follow_dji0_pose_to_odom|support_follow_dji1_simulator|support_follow_dji1_odom_controller|support_follow_dji2_simulator|support_follow_dji2_odom_controller|support_dji1_leader_detector|support_dji2_leader_detector|support_detection_mux|dji0_to_ugv_forwarder|support_camera_scanner|twist_server_node|rviz2|rqt|rqt_gui' || true

cleanup_state_files

echo "[kill_all_ros2] Cleanup complete."

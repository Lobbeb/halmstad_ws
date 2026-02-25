#!/usr/bin/env bash
set -euo pipefail

# Project-local cleanup helper for repeated runtime smoke tests.
# Safe to run between tests; it only targets common ROS/Gazebo/bridge processes
# used by this workspace.

echo "[kill_everything] Stopping rosbag, project nodes, launches, bridges, and Gazebo..."

pkill -f "ros2 bag record" 2>/dev/null || true
pkill -f "ros2 run lrs_halmstad" 2>/dev/null || true
pkill -f "leader_estimator" 2>/dev/null || true
pkill -f "follow_uav" 2>/dev/null || true
pkill -f "contract_check" 2>/dev/null || true
pkill -f "ros2 topic pub.*a201_0000" 2>/dev/null || true
pkill -f "ros_gz_bridge/parameter_bridge" 2>/dev/null || true
pkill -f "ros_gz_image/image_bridge" 2>/dev/null || true
pkill -f "interactive_marker_twist_server/marker_server" 2>/dev/null || true
pkill -f "a201_0000" 2>/dev/null || true
pkill -f "ros_gz_sim create" 2>/dev/null || true
pkill -f "spawner_joint_state_broadcaster" 2>/dev/null || true
pkill -f "spawner_platform_velocity_controller" 2>/dev/null || true
pkill -f "ros2 launch" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "ros2cli.daemon.daemonize" 2>/dev/null || true

# Give processes a short moment to exit cleanly.
sleep 1

# Escalate for stubborn leftovers from repeated Gazebo/bridge runs.
pkill -9 -f "ros_gz_image/image_bridge" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge/parameter_bridge" 2>/dev/null || true
pkill -9 -f "a201_0000" 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true

echo "[kill_everything] Remaining relevant processes (ros2/gz/bridge):"
ps aux | grep -E "ros2|gz sim|ros_gz_|parameter_bridge|image_bridge|a201_0000" | grep -v grep || true

echo "[kill_everything] Done."

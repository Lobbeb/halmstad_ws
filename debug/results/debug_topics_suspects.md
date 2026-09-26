<!-- Generated from debug_topics_suspects.csv. Compact view; details remain in CSV. -->
# DEBUG SUSPECTS

***Thresholds***: BW >= 1 KB/s **AND** Hz >= 5. Rows: 62.

| <div style="width:100">Topic</div> | <div style="width:42px">Status</div> | Fix | Frequency | <div style="width:80px">Bandwidth</div> | <div style="width:160px">Peak Hz</div> | <div style="width:180px">Peak BW</div> | <div style="width:180px">Modes</div> |<div style="width:290px">Publishers</div> | <div style="width:290px">Subscribers</div> |
| :---: | :----: | :---: | -: | -: | :------ | :------ | :--- | :--------- | :---------- |
| /a201_0000/sensors/camera_0/color/image | ✓ | Disabled in `ugv_rgb_camera_bridge.launch.py` | 14.598 | 12.83 MB/s | - | - | FOLLOW YOLO | ugv_camera_0_rgb_bridge | - |
| /coord/leader_visual_control_debug_image | ✓ | Does not publish in `visual_follow_controller.py` when no subscribers. | 14.0 | 12.31 MB/s | 14.000 (FULL) | 12.31 MB/s (FULL) | VISUAL BRIDGE, FULL | visual_follow_controller | - |
| /a201_0000/sensors/lidar3d_0/points | ✓ | Lowered from 20 hz to 10 hz, lowers bw to ~ 4 MB/s. | 16.191 | 12.10 MB/s | 16.191 (VISUAL BRIDGE) | 12.10 MB/s (VISUAL BRIDGE) | FOLLOW YOLO TRACKER VISUAL BRIDGE FULL SUPPORT YOLO | ugv_lidar3d_0_points_bridge | /a201_0000/pointcloud_to_laserscan |
| /coord/leader_debug_image | ✓ | Does not publish in `leader_estimator.py` when no subscribers. | 12.793 | 11.24 MB/s | - | - | YOLO | leader_estimator | - |
| /dji0/camera0/depth_image | ✓ | Disabled in *FOLLOW* mode. | 7.196 | 8.43 MB/s | 7.196 (VISUAL BRIDGE) | 8.43 MB/s (VISUAL BRIDGE) | YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | uav_depth_bridge | leader_estimator |
| /dji2/camera0/image_raw | ✓ | Lowered from 10 to 5 update rate in `support_follow_odom.launch.py`. | 7.798 | 6.85 MB/s | 7.798 (SUPPORT YOLO) | 6.85 MB/s (SUPPORT YOLO) | SUPPORT YOLO | /support_follow_spawn_dji2/uav_camera_bridge | support_dji2_leader_detector |
| /dji1/camera0/image_raw | ✓ | Lowered from 10 to 5 update rate in `support_follow_odom.launch.py`. | 7.599 | 6.68 MB/s | 7.599 (SUPPORT YOLO) | 6.68 MB/s (SUPPORT YOLO) | SUPPORT YOLO | /support_follow_spawn_dji1/uav_camera_bridge | support_dji1_leader_detector |
| /dji0/camera0/image_raw | - | - | 7.4 | 6.50 MB/s | 7.400 (FULL) | 6.50 MB/s (FULL) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | uav_camera_bridge | leader_detector, leader_estimator, leader_tracker, +1 |
| /a201_0000/controller_manager/statistics/full | ✓ | Added to ignore list so rosbag and audit does not subscribe. | 247.334 | 596.11 KB/s | 247.334 (VISUAL BRIDGE) | 596.11 KB/s (VISUAL BRIDGE) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/controller_manager | - |
| /a201_0000/controller_manager/statistics/values | ✓ | Added to ignore list so rosbag and audit does not subscribe. | 246.934 | 79.10 KB/s | 246.934 (VISUAL BRIDGE) | 79.10 KB/s (VISUAL BRIDGE) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/controller_manager | - |
| /a201_0000/sensors/lidar3d_0/scan_from_points | ✓ | Now only about ~15 KB/s, increased scan_time to 0.10 in `localization_with_params.launch.py`. | 13.399 | 48.05 KB/s | - | - | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/pointcloud_to_laserscan | /a201_0000/local_costmap/local_costmap, /a201_0000/amcl, /a201_0000/latest_scan_relay |
| /a201_0000/controller_manager/introspection_data/full | ✓ | Subscribers can make these hot; ignore unless a non-audit subscriber exists. | 19.798 | 42.30 KB/s | - | - | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/controller_manager | - |
| /a201_0000/sensors/lidar3d_0/scan_from_points_relay | ✓ | Lowered update rate to 5 hz in `localization_with_params.launch.py`. | 10.199 | 36.57 KB/s | 10.199 (FULL) | 36.57 KB/s (FULL) | FULL | /a201_0000/latest_scan_relay | /a201_0000/amcl |
| /a201_0000/platform/odom/filtered | - | - | 44.988 | 31.81 KB/s | 44.988 (VISUAL BRIDGE) | 31.81 KB/s (VISUAL BRIDGE) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/ekf_node | /a201_0000/bt_navigator |
| /coord/ugv/support_observation_summary | - | - | 8.733 | 18.35 KB/s | 8.733 (SUPPORT YOLO) | 18.35 KB/s (SUPPORT YOLO) | SUPPORT YOLO | dji0_to_ugv_forwarder | - |
| /coord/dji0/support_observation_summary | - | - | 8.798 | 17.14 KB/s | 8.798 (SUPPORT YOLO) | 17.14 KB/s (SUPPORT YOLO) | SUPPORT YOLO | support_detection_mux | dji0_to_ugv_forwarder |
| /coord/leader_visual_target_estimate | - | - | 19.795 | 14.15 KB/s | 19.795 (VISUAL BRIDGE) | 14.15 KB/s (VISUAL BRIDGE) | VISUAL BRIDGE, FULL | visual_target_estimator | follow_point_generator, follow_point_planner, +2, +3 |
| /a201_0000/platform/odom | - | - | 19.193 | 13.57 KB/s | 19.193 (YOLO) | 13.57 KB/s (YOLO) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/platform_velocity_controller | /a201_0000/ekf_node, /a201_0000/controller_server |
| /dji0/pose/odom | - | - | 17.466 | 12.21 KB/s | 17.466 (SUPPORT YOLO) | 12.21 KB/s (SUPPORT YOLO) | FULL, SUPPORT YOLO | omnet_uav_pose_to_odom, support_follow_dji0_pose_to_odom | omnet_tcp_bridge, support_follow_dji1_odom_controller, support_follow_dji2_odom_controller |
| /a201_0000/ground_truth/odom | - | - | 16.992 | 12.01 KB/s | - | - | FOLLOW, FULL | /a201_0000/ugv_ground_truth_bridge | follow_uav, rosbag2_recorder, omnet_tcp_bridge |
| /coord/leader_visual_target_estimate_status | - | - | 19.795 | 10.27 KB/s | 19.795 (VISUAL BRIDGE) | 10.27 KB/s (VISUAL BRIDGE) | VISUAL BRIDGE, FULL | visual_target_estimator | - |
| /coord/leader_estimate_status | - | - | 19.995 | 10.06 KB/s | 19.995 (VISUAL BRIDGE) | 10.06 KB/s (VISUAL BRIDGE) | YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | leader_estimator | rosbag2_recorder, camera_tracker |
| /coord/leader_visual_control_status | - | - | 19.6 | 9.58 KB/s | 19.600 (FULL) | 9.58 KB/s (FULL) | VISUAL BRIDGE, FULL | visual_follow_controller | - |
| /coord/leader_selected_target_filtered | - | - | 19.795 | 9.57 KB/s | 19.795 (VISUAL BRIDGE) | 9.57 KB/s (VISUAL BRIDGE) | VISUAL BRIDGE, FULL | selected_target_filter | visual_target_estimator, visual_follow_controller |
| /coord/leader_visual_actuation_bridge_status | - | - | 19.6 | 9.41 KB/s | 19.600 (FULL) | 9.41 KB/s (FULL) | VISUAL BRIDGE, FULL | visual_actuation_bridge | - |
| /a201_0000/sensors/camera_0/color/camera_info | - | - | 24.397 | 9.15 KB/s | - | - | FOLLOW, YOLO | ugv_camera_0_rgb_bridge | - |
| /coord/ugv/leader_detection | - | - | 8.733 | 8.51 KB/s | 8.733 (SUPPORT YOLO) | 8.51 KB/s (SUPPORT YOLO) | SUPPORT YOLO | dji0_to_ugv_forwarder | - |
| /coord/leader_selected_target_filtered_status | - | - | 19.795 | 7.72 KB/s | 19.795 (VISUAL BRIDGE) | 7.72 KB/s (VISUAL BRIDGE) | VISUAL BRIDGE, FULL | selected_target_filter | - |
| /coord/dji0/leader_detection | - | - | 8.798 | 7.33 KB/s | 8.798 (SUPPORT YOLO) | 7.33 KB/s (SUPPORT YOLO) | SUPPORT YOLO | support_detection_mux | dji0_to_ugv_forwarder |
| /a201_0000/dynamic_joint_states | - | - | 19.6 | 7.12 KB/s | 19.600 (FULL) | 7.12 KB/s (YOLO) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/joint_state_broadcaster | - |
| /dji2/pose_cmd/odom | - | - | 9.799 | 6.93 KB/s | 9.799 (SUPPORT YOLO) | 6.93 KB/s (SUPPORT YOLO) | SUPPORT YOLO | support_follow_dji2_odom_controller | - |
| /dji1/pose_cmd/odom | - | - | 9.798 | 6.93 KB/s | 9.798 (SUPPORT YOLO) | 6.93 KB/s (SUPPORT YOLO) | SUPPORT YOLO | support_follow_dji1_odom_controller | - |
| /coord/leader_detection | - | - | 6.797 | 6.91 KB/s | 5.399 (TRACKER) | 5.44 KB/s (TRACKER) | YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | leader_detector, leader_tracker | leader_estimator |
| /coord/leader_planned_target_status | - | - | 19.795 | 6.86 KB/s | 19.795 (VISUAL BRIDGE) | 6.86 KB/s (VISUAL BRIDGE) | VISUAL BRIDGE, FULL | follow_point_planner | - |
| /a201_0000/diagnostics | - | - | 9.397 | 6.57 KB/s | 9.397 (VISUAL BRIDGE) | 6.57 KB/s (VISUAL BRIDGE) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/ekf_node, /a201_0000/twist_mux, /a201_0000/joy_node, +1, +1 | - |
| /a201_0000/controller_manager/introspection_data/values | ✓ | Subscribers can make these hot; ignore unless a non-audit subscriber exists. | 19.793 | 5.57 KB/s | 19.793 (VISUAL BRIDGE) | 5.57 KB/s (VISUAL BRIDGE) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/controller_manager | - |
| /coord/leader_follow_point_status | - | - | 19.995 | 5.51 KB/s | 19.995 (VISUAL BRIDGE) | 5.51 KB/s (VISUAL BRIDGE) | VISUAL BRIDGE, FULL | follow_point_generator | - |
| /a201_0000/platform/joint_states | - | - | 19.2 | 4.87 KB/s | 19.200 (FULL) | 4.87 KB/s (FULL) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/joint_state_broadcaster | /a201_0000/robot_state_publisher |
| /coord/leader_detection_status | - | - | 6.797 | 4.65 KB/s | 5.399 (TRACKER) | 1.96 KB/s (TRACKER) | YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | leader_detector, leader_tracker | leader_estimator, rosbag2_recorder |
| /coord/support/dji2/leader_detection | - | - | 5.599 | 4.17 KB/s | 5.599 (SUPPORT YOLO) | 4.17 KB/s (SUPPORT YOLO) | SUPPORT YOLO | support_dji2_leader_detector | support_detection_mux |
| /dji0/camera0/camera_info | - | - | 9.9 | 3.71 KB/s | 9.599 (TRACKER) | 3.60 KB/s (TRACKER) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | uav_camera_bridge | leader_estimator, visual_target_estimator, +1 |
| /coord/ugv/support_awareness_status | - | - | 8.733 | 3.49 KB/s | 8.733 (SUPPORT YOLO) | 3.49 KB/s (SUPPORT YOLO) | SUPPORT YOLO | dji0_to_ugv_forwarder | - |
| /dji2/camera0/camera_info | - | - | 8.799 | 3.30 KB/s | 8.799 (SUPPORT YOLO) | 3.30 KB/s (SUPPORT YOLO) | SUPPORT YOLO | /support_follow_spawn_dji2/uav_camera_bridge | - |
| /dji1/camera0/camera_info | - | - | 8.498 | 3.19 KB/s | 8.498 (SUPPORT YOLO) | 3.19 KB/s (SUPPORT YOLO) | SUPPORT YOLO | /support_follow_spawn_dji1/uav_camera_bridge | - |
| /coord/leader_estimate_fault | - | - | 15.4 | 3.13 KB/s | 15.400 (VISUAL BRIDGE) | 3.13 KB/s (VISUAL BRIDGE) | YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | leader_estimator | - |
| /coord/leader_selected_target | - | - | 19.795 | 2.93 KB/s | 19.795 (VISUAL BRIDGE) | 2.93 KB/s (YOLO) | YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | leader_estimator | selected_target_filter |
| /coord/ugv/leader_detection_status | - | - | 8.733 | 2.90 KB/s | 8.733 (SUPPORT YOLO) | 2.90 KB/s (SUPPORT YOLO) | SUPPORT YOLO | dji0_to_ugv_forwarder | - |
| /coord/dji0/leader_detection_status | - | - | 8.798 | 2.54 KB/s | 8.798 (SUPPORT YOLO) | 2.54 KB/s (SUPPORT YOLO) | SUPPORT YOLO | support_detection_mux | dji0_to_ugv_forwarder |
| /coord/leader_distance_debug | - | - | 19.995 | 2.50 KB/s | 19.995 (VISUAL BRIDGE) | 2.50 KB/s (VISUAL BRIDGE) | YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | leader_estimator | - |
| /coord/ugv/support_path_advisory | - | - | 8.733 | 2.36 KB/s | 8.733 (SUPPORT YOLO) | 2.36 KB/s (SUPPORT YOLO) | SUPPORT YOLO | dji0_to_ugv_forwarder | - |
| /coord/support/dji2/leader_detection_status | - | - | 5.599 | 2.30 KB/s | 5.599 (SUPPORT YOLO) | 2.30 KB/s (SUPPORT YOLO) | SUPPORT YOLO | support_dji2_leader_detector | support_detection_mux |
| /coord/leader_estimate | - | - | 19.9 | 1.63 KB/s | 19.597 (TRACKER) | 1.61 KB/s (TRACKER) | YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | leader_estimator | follow_uav, follow_point_generator, rosbag2_recorder, +1 |
| /coord/leader_visual_control | - | - | 19.6 | 1.61 KB/s | 19.600 (FULL) | 1.61 KB/s (FULL) | VISUAL BRIDGE, FULL | visual_follow_controller | visual_actuation_bridge |
| /dji0/pose | - | - | 19.595 | 1.61 KB/s | 19.595 (VISUAL BRIDGE) | 1.59 KB/s (TRACKER) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | uav_simulator | follow_uav, leader_estimator, follow_point_generator, +2, +4 |
| /a201_0000/platform/cmd_vel | - | - | 20.794 | 1.54 KB/s | 20.794 (VISUAL BRIDGE) | 1.54 KB/s (VISUAL BRIDGE) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/twist_mux | /a201_0000/cmd_vel_bridge, /a201_0000/platform_velocity_controller |
| /coord/leader_follow_point | - | - | 19.6 | 1.45 KB/s | 19.600 (FULL) | 1.45 KB/s (VISUAL BRIDGE) | VISUAL BRIDGE, FULL | follow_point_generator | follow_point_planner, visual_actuation_bridge |
| /coord/leader_planned_target | - | - | 19.589 | 1.45 KB/s | 19.589 (VISUAL BRIDGE) | 1.45 KB/s (VISUAL BRIDGE) | VISUAL BRIDGE, FULL | follow_point_planner | visual_actuation_bridge |
| /dji1/pose | - | - | 17.596 | 1.44 KB/s | 17.596 (SUPPORT YOLO) | 1.44 KB/s (SUPPORT YOLO) | SUPPORT YOLO | support_follow_dji1_simulator | support_follow_dji1_odom_controller |
| /dji2/pose | - | - | 17.396 | 1.43 KB/s | 17.396 (SUPPORT YOLO) | 1.43 KB/s (SUPPORT YOLO) | SUPPORT YOLO | support_follow_dji2_simulator | support_follow_dji2_odom_controller |
| /a201_0000/ground_truth/pose | - | - | 16.792 | 1.38 KB/s | - | - | FOLLOW, FULL | /a201_0000/ugv_ground_truth_bridge | - |
| /a201_0000/platform_velocity_controller/cmd_vel_out | - | - | 19.2 | 1.27 KB/s | 19.200 (FULL) | 1.27 KB/s (FULL) | FOLLOW, YOLO, TRACKER, VISUAL BRIDGE, FULL, SUPPORT YOLO | /a201_0000/platform_velocity_controller | - |
| /dji0/pose_cmd | - | - | 15.4 | 1.14 KB/s | 15.400 (VISUAL BRIDGE) | 1.14 KB/s (VISUAL BRIDGE) | VISUAL BRIDGE, FULL | visual_actuation_bridge | camera_tracker |
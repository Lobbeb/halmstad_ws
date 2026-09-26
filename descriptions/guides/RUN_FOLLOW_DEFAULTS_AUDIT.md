# run_follow_defaults.yaml Audit

## Current Session Context

This audit is now mostly a reference document. The active work has shifted toward **dataset capture reliability** for:

- the leader UAV (`dji0`)
- the two support UAVs (`dji1`, `dji2`)

Important current context:

- Prefer reusing the existing launch scripts, follow tools, capture tools, and manifests instead of adding new files.
- Do not treat this audit as a request to keep inventing more config cleanup work unless the user explicitly asks for it.
- The main current Baylands problem is **AMCL pose / Nav2 waypoint pose consistency**, especially when trying to move between capture areas reliably.
- Current automated leader capture uses `collect_leader_dataset`, with scenario retries and a slower default capture rate.
- Current support capture work uses the existing support follow, support camera scan, and paired capture tooling.

This audits `src/lrs_halmstad/config/run_follow_defaults.yaml` by section, showing where each parameter is consumed and whether it is needed for the current launch paths.

## Summary

- I did not find a YAML parameter that is plainly unused by static code search.
- Many parameters are not used in the default simplified YOLO path because they belong to the optional visual-follow bridge pipeline.
- The most important cleanup is conceptual: keep the default YOLO path small, and treat the visual pipeline blocks as optional/experimental.
- I fixed two issues found while auditing:
  - `range_mode` is now the only range-source parameter; launch overrides use the same name as the YAML key.
  - support-chain odom slot offsets were passed to `follow_uav_odom` but not initialized or applied; now `forward_offset_m` and `lateral_offset_m` work for support slots.

## Current Default Path

Default `mode:=yolo` with `yolo_control_mode:=follow_uav_estimate` uses:

- `leader_detector` or `leader_tracker`
- `leader_estimator`
- `camera_tracker`
- `follow_uav`
- `ugv_nav2_driver`
- `uav_simulator` when enabled by launch

Default simplified YOLO does not use these optional visual bridge nodes unless explicitly enabled:

- `selected_target_filter`
- `visual_target_estimator`
- `visual_follow_controller`
- `follow_point_generator`
- `follow_point_planner`
- `visual_actuation_bridge`

## Global Parameters

Consumer files: `sim/simulator.py`, `follow/camera_tracker.py`, `follow/follow_point_generator.py`, `follow/follow_uav.py`, `follow/follow_uav_odom.py`, `perception/leader_estimator.py`.

Keep all of these. They are shared geometry or canonical follow distance.

- `camera_x_offset_m`: used by simulator, camera tracker, and visual follow-point geometry.
- `camera_y_offset_m`: used by simulator, camera tracker, and visual follow-point geometry.
- `camera_z_offset_m`: used by simulator, camera tracker, and leader estimator camera projection.
- `camera_mount_pitch_deg`: used by simulator/controller and camera tracker.
- `camera_yaw_offset_deg`: used by simulator and camera tracker.
- `camera_pan_sign`: used by simulator and camera tracker.
- `leader_look_target_x_m`: used by camera tracker look-at geometry.
- `leader_look_target_y_m`: used by camera tracker look-at geometry.
- `d_target`: used by `follow_uav`, `follow_uav_odom`, and `leader_estimator` constant-range fallback.

## follow_uav

Consumer files: `follow/follow_uav.py`, `follow/follow_uav_odom.py`, `launch/run_follow.launch.py`, `launch/support_follow_odom.launch.py`.

Keep all of these for the simplified follow controllers.

- `xy_anchor_max`: clamps the anchor around the leader.
- `seed_uav_cmd_on_start`: seeds command state from configured start pose.
- `uav_start_x`: fallback/launch start X.
- `uav_start_y`: fallback/launch start Y.
- `uav_start_z`: fallback/hold altitude.
- `uav_start_yaw_deg`: fallback/launch yaw.
- `require_uav_actual_before_motion`: blocks motion until UAV pose is fresh.
- `use_uav_actual_z_on_start`: latches current UAV altitude at startup.
- `start_delay_s`: startup hold delay.
- `tick_hz`: controller tick rate.
- `pose_timeout_s`: leader/UAV pose freshness timeout.
- `min_cmd_period_s`: command publish throttle.
- `follow_speed_mps`: maximum XY follow speed.
- `follow_speed_gain`: proportional speed term.
- `follow_yaw`: enables yaw control.
- `follow_yaw_rate_rad_s`: maximum yaw rate.
- `follow_yaw_rate_gain`: proportional yaw-rate term.
- `leader_heading_offset_deg`: odom-only heading correction.
- `publish_pose_cmd_topics`: enables `pose_cmd` mirrors.

## camera_tracker

Consumer file: `follow/camera_tracker.py`; launched by `run_follow.launch.py`.

Keep these while camera pan/tilt tracking is part of YOLO/follow runs.

- `camera_look_target_z_m`: vertical look target offset.
- `default_pan_deg`: pan fallback.
- `default_tilt_deg`: tilt fallback and launch override target.
- `pan_enable`: enables pan tracking.
- `tilt_enable`: enables tilt tracking.
- `image_center_correction_enable`: enables image-space recentering.
- `image_center_correction_timeout_s`: direct box recenter timeout.
- `image_center_correction_max_latency_ms`: rejects late detections for recentering.
- `image_center_memory_timeout_s`: short memory for recentering after detection loss.
- `pan_image_center_gain`: pan image correction gain.
- `pan_image_center_max_deg`: pan correction cap.
- `tilt_image_center_gain`: tilt image correction gain.
- `tilt_image_center_max_deg`: tilt correction cap.
- `weak_status_center_correction_enable`: uses weak detector status for gimbal-only recentering.
- `weak_status_center_correction_timeout_s`: weak-status recenter timeout.
- `weak_status_center_conf_threshold`: weak-status confidence floor.
- `weak_status_center_min_area_norm`: weak-status area floor.
- `weak_status_center_correction_scale`: weak-status correction scale.
- `visual_target_estimate_timeout_s`: visual estimate fallback timeout.
- `prefer_visual_estimate_tilt_recovery`: uses visual estimate earlier for tilt recovery.
- `publish_debug_topics`: enables camera debug topics.
- `pose_timeout_s`: UAV/leader pose freshness.
- `actual_pose_reacquire_enable`: optional truth-pose reacquire.
- `trackable_hold_timeout_s`: hold trackable pose during inference gaps.
- `tick_hz`: gimbal update rate.
- `gimbal_override_hold_s`: external override hold.

## uav_simulator

Consumer files: `sim/simulator.py`, `sim/controller.py`; launched by `run_follow.launch.py` and support launch files.

Keep all while using simulated UAV/gimbal behavior.

- `gimbal_pitch_min_rad`: gimbal pitch lower limit.
- `gimbal_pitch_max_rad`: gimbal pitch upper limit.
- `pan_rate_deg_s`: simulated pan slew rate.
- `tilt_rate_deg_s`: simulated tilt slew rate.
- `startup_set_pose_grace_s`: avoids SetEntityPose spam immediately after startup.
- `set_pose_failure_backoff_s`: backoff after SetEntityPose failure.

## leader_detector

Consumer file: `perception/leader_detector.py`; launch overrides in `run_follow.launch.py`.

Keep for detector mode.

- `event_topic`: detector event topic.
- `out_topic`: detector output topic.
- `status_topic`: detector status topic.
- `publish_events`: enables detector events.
- `device`: inference device.
- `backend`: inference backend.
- `onnx_model`: optional ONNX path.
- `imgsz`: inference image size.
- `predict_hz`: maximum publish/inference loop rate.
- `async_inference`: worker-thread inference.
- `latest_frame_only`: drops stale pending frames.
- `stale_detection_threshold_ms`: stale detection budget.
- `metrics_window_s`: rolling metrics window.
- `benchmark_csv_path`: optional benchmark CSV.
- `image_qos_depth`: image subscription depth.
- `image_qos_reliability`: image QoS reliability.
- `conf_threshold`: confidence floor.
- `iou_threshold`: NMS IOU threshold.
- `target_class_id`: class ID filter.
- `target_class_name`: class name filter.
- `bbox_continuity_class_bonus`: continuity class bonus.
- `bbox_continuity_max_px`: continuity distance gate.
- `bbox_continuity_weight`: continuity score weight.

## leader_tracker

Consumer file: `perception/leader_tracker.py`; launch overrides in `run_follow.launch.py`.

Keep for tracker mode.

- `event_topic`: tracker event topic.
- `out_topic`: tracker output topic.
- `status_topic`: tracker status topic.
- `publish_events`: enables tracker events.
- `device`: inference device.
- `backend`: inference backend.
- `onnx_model`: optional ONNX path.
- `imgsz`: inference image size.
- `predict_hz`: maximum publish/inference loop rate.
- `tracker_config`: Ultralytics tracker config.
- `async_inference`: worker-thread inference.
- `latest_frame_only`: drops stale pending frames.
- `stale_detection_threshold_ms`: stale detection budget.
- `metrics_window_s`: rolling metrics window.
- `benchmark_csv_path`: optional benchmark CSV.
- `image_qos_depth`: image subscription depth.
- `image_qos_reliability`: image QoS reliability.
- `pseudo_track_max_center_jump_px`: ONNX fallback pseudo-track gate.
- `conf_threshold`: confidence floor.
- `iou_threshold`: tracker IOU threshold.
- `target_class_id`: class ID filter.
- `target_class_name`: class name filter.

## leader_estimator

Consumer file: `perception/leader_estimator.py`; launch overrides in `run_follow.launch.py`.

Keep all for YOLO-estimate mode. The `range_mode` key is used directly by YAML and launch overrides.

- `estimate_error_topic`: optional estimate-vs-truth error topic.
- `event_topic`: estimator event topic.
- `external_detection_max_latency_ms`: accepts delayed detector/tracker outputs.
- `external_detection_timeout_s`: detection freshness timeout.
- `external_detection_topic`: detector/tracker input topic.
- `external_detection_status_topic`: detector/tracker status input.
- `leader_actual_pose_enable`: optional truth/diagnostic pose use.
- `leader_actual_pose_timeout_s`: truth pose freshness timeout.
- `leader_actual_pose_topic`: truth pose topic.
- `out_topic`: published leader estimate.
- `publish_debug_image`: enables overlay image.
- `publish_events`: enables estimator events.
- `publish_fault_status`: enables fault topic.
- `publish_status`: enables estimator status topic.
- `status_topic`: estimator status output.
- `fault_status_topic`: estimator fault output.
- `debug_image_topic`: debug overlay image output.
- `cam_pitch_offset_deg`: estimator camera pitch offset.
- `cam_yaw_offset_deg`: estimator camera yaw offset.
- `cam_roll_offset_deg`: reserved/declared camera roll offset.
- `cam_x_offset_m`: estimator camera X offset.
- `cam_y_offset_m`: estimator camera Y offset.
- `cam_z_offset_m`: estimator camera Z offset.
- `image_timeout_s`: image freshness timeout.
- `uav_pose_timeout_s`: UAV pose freshness timeout.
- `est_hz`: estimator loop rate.
- `range_mode`: YAML fallback range source.
- `depth_max_m`: max accepted depth.
- `depth_min_m`: min accepted depth.
- `depth_scale`: raw depth conversion scale.
- `depth_timeout_s`: depth freshness timeout.
- `depth_patch_min_valid_px`: min valid depth pixels.
- `depth_percentile`: patch percentile used as depth sample.
- `depth_max_estimate_jump_m`: depth jump rejection.
- `target_ground_z_m`: ground plane height for OBB projection.
- `radio_range_topic`: OMNeT/radio range topic.
- `radio_range_timeout_s`: radio range freshness timeout.

## ugv_nav2_driver

Consumer file: `nav/ugv_nav2_driver.py`; launch overlays in `run_follow.launch.py`.

Keep all while `ugv_mode:=nav2` is supported.

- `nav2_required_lifecycle_nodes`: lifecycle nodes that must be active.
- `continue_on_goal_failure`: advances after goal failure.
- `path_topic`: debug path topic.
- `pose_topic`: UGV pose source.
- `pose_topic_type`: UGV pose message type.
- `initial_pose_covariance_xy`: AMCL initial pose XY covariance.
- `initial_pose_covariance_yaw`: AMCL initial pose yaw covariance.
- `initial_pose_frame_id`: initial pose frame.
- `initial_pose_publish_hz`: initial pose repeat rate.
- `initial_pose_skip_wait_s`: wait before publishing initial pose.
- `initial_pose_timeout_s`: initial pose timeout.
- `initial_pose_topic`: initial pose topic.
- `initial_pose_x`: initial pose X.
- `initial_pose_y`: initial pose Y.
- `initial_pose_yaw_deg`: initial pose yaw.
- `set_initial_pose_enable`: enables automatic initial pose.
- `goal_action_name`: Nav2 action name.
- `goal_frame_id`: goal frame.
- `goal_reject_retry_count`: rejected-goal retries.
- `goal_reject_retry_delay_s`: retry delay.
- `goal_result_timeout_s`: goal result timeout.
- `goal_sequence_csv`: inline route.
- `goal_sequence_file`: route YAML.
- `goal_sequence_random_reverse`: random reverse toggle.
- `goal_sequence_randomize`: random order toggle.
- `goal_sequence_relative_to_current_pose`: relative route toggle.
- `goal_sequence_seed`: route random seed.
- `goal_server_timeout_s`: action server timeout.
- `goal_start_delay_s`: startup delay before first goal.
- `min_goal_xy_delta_m`: skip too-close goals.
- `min_goal_yaw_delta_deg`: skip too-similar yaw goals.
- `pause_after_goal_s`: dwell after each goal.
- `cycles`: generated-route cycle count.
- `forward_speed`: generated forward segment speed.
- `forward_time_s`: generated forward segment duration.
- `motion_profile`: generated-route profile.
- `pause_every_n`: generated pause cadence.
- `pause_time_s`: generated pause duration.
- `turn_pattern`: generated turn pattern.
- `turn_speed`: generated turn speed.
- `turn_time_s`: generated turn duration.
- `variation_amplitude`: generated-route variation magnitude.
- `variation_enable`: generated-route variation toggle.
- `pose_stale_timeout_s`: stale localization timeout.
- `pose_timeout_s`: first pose timeout.
- `start_delay_s`: route handling start delay.

## Optional Visual Pipeline

These sections are not part of the default simplified YOLO control path. Keep them only if we want to preserve the visual bridge experiments:

- `selected_target_filter`
- `visual_target_estimator`
- `visual_follow_controller`
- `follow_point_generator`
- `follow_point_planner`
- `visual_actuation_bridge`

### selected_target_filter

Consumer file: `follow/selected_target_filter.py`. Needed only when the visual pipeline is enabled.

- `strong_confidence_threshold`
- `weak_confidence_threshold`
- `min_confidence_threshold`
- `track_switch_min_confidence`
- `hold_timeout_s`
- `prediction_enable`
- `prediction_max_gap_s`

### visual_target_estimator

Consumer file: `perception/visual_target_estimator.py`. Needed only when the visual pipeline is enabled.

The old `reference_range_m` duplicate was removed; area-based range now uses shared `d_target` as its reference range.

- `range_signal_mode`
- `reference_area_norm`
- `position_alpha_projected`
- `position_alpha_area`
- `velocity_beta_projected`
- `velocity_beta_area`
- `prediction_max_gap_s`
- `degraded_after_s`
- `hard_lost_after_s`
- `continuity_miss_decay_per_s`
- `predicted_velocity_decay`
- `degraded_velocity_decay`
- `visibility_center_weight`
- `visibility_edge_weight`
- `visibility_area_weight`
- `visibility_center_soft_radius_norm`
- `visibility_edge_soft_margin_norm`

### visual_follow_controller

Consumer file: `follow/visual_follow_controller.py`. Needed only when visual controller/bridge mode is enabled.

- `yaw_gain`
- `forward_range_gain`
- `forward_area_gain`
- `yaw_rate_max_rad_s`
- `forward_speed_max_mps`
- `loss_hold_timeout_s`
- `recovery_recenter_error_norm`
- `recovery_quality_threshold`
- `tracked_forward_suppress_scale`
- `tracked_yaw_boost_scale`
- `predicted_forward_suppress_scale`
- `degraded_forward_suppress_scale`
- `recovery_yaw_boost_scale`

### follow_point_generator

Consumer file: `follow/follow_point_generator.py`. Needed only when follow-point generator/planner/bridge mode is enabled.

- `prefer_target_pose_position`
- `prefer_target_pose_heading`
- `lateral_offset_m`
- `lookahead_horizon_s`
- `heading_dir_alpha`
- `point_alpha`
- `max_follow_point_jump_m`
- `hold_timeout_s`
- `require_motion_to_start`
- `min_target_speed_mps`
- `target_velocity_alpha`
- `max_target_meas_speed_mps`
- `heading_hold_timeout_s`
- `predicted_lookahead_scale`
- `degraded_lookahead_scale`
- `degraded_point_alpha_scale`
- `predicted_recovery_blend`
- `degraded_recovery_blend`
- `tracked_recovery_quality_threshold`
- `tracked_follow_distance_scale_min`
- `tracked_lateral_offset_scale_min`
- `tracked_recovery_blend_max`
- `predicted_follow_distance_scale`
- `degraded_follow_distance_scale`
- `predicted_lateral_offset_scale`
- `degraded_lateral_offset_scale`

### follow_point_planner

Consumer file: `follow/follow_point_planner.py`. Needed only when planner/bridge mode is enabled.

- `xy_alpha`
- `z_alpha`
- `yaw_alpha`
- `max_planned_step_m`
- `max_planned_z_step_m`
- `max_planned_yaw_step_rad`
- `target_estimate_timeout_s`
- `hold_timeout_s`
- `stale_fp_thresh_m`
- `stale_alpha_scale`
- `predicted_xy_alpha_scale`
- `degraded_xy_alpha_scale`
- `predicted_yaw_alpha_scale`
- `degraded_yaw_alpha_scale`
- `degraded_step_scale`

### visual_actuation_bridge

Consumer file: `follow/visual_actuation_bridge.py`. Needed only when visual actuation bridge mode is enabled.

- `input_mode`
- `forward_scale`
- `yaw_scale`
- `max_xy_step_m`
- `max_yaw_step_rad`
- `use_current_altitude`
- `publish_pose_cmd_mirror`
- `yaw_cmd_sign`
- `target_estimate_timeout_s`
- `predicted_step_scale`
- `degraded_step_scale`
- `lost_recovery_hold_s`
- `lost_step_scale`

## Cleanup Recommendations

Safe to do now:

- Keep all YAML parameters that belong to the default simplified path.
- Keep optional visual pipeline YAML only if we still want `yolo_control_mode:=visual_bridge` and related experiments.
- Keep distance/altitude defaults centralized: `d_target` is the canonical standoff, and the visual follow-point altitude follows `uav_start_z`.

Possible future cleanup:

- Split optional visual pipeline parameters into a separate visual-pipeline defaults file, so `run_follow_defaults.yaml` stays focused on the default path.
- Add a smaller "simple YOLO/follow" defaults file for the current preferred mode.
- Remove old helper code in `follow_core.py` that refers to no-longer-configured bounded-distance concepts, after verifying no optional visual code calls it.
- Decide whether advanced parameters declared in code but absent from YAML should stay hidden/internal or be documented in this file.

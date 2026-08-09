# Ruben handoff: C3 visual pipeline + C4 support-chain reference

This file is the practical settings handoff from William's simulation side.
Use it to match the same route setup, model, detector settings, and control modes when adding the communication/OMNeT++ side.

## 1. What to copy exactly

For C3, use this core setup:

| Item | Value |
|---|---|
| condition | `C3` |
| world | `baylands` |
| control mode | `yolo_control_mode:=visual_bridge` |
| visual logic | `visual_follow_logic:=follow_core` |
| detector backend | `ultralytics` |
| model | `models/obb/mymodels/baylands-leader-v4-3.pt` |
| tracker | `true` |
| external detection node | `tracker` |
| confidence threshold | `0.11` |
| IoU threshold | `0.3` |
| range mode | `auto` |
| visual reacquire assist | `true` |
| reacquire stale timeout | `2.0 s` |
| reacquire return fresh | `1.0 s` |
| duration | `300 s` |
| warmup | `30 s` |
| GUI | `false` |

Final C3 route data is in:

```text
bags/results_c3_final_8routes_cdh_updated/
```

The selected-run pointer file is:

```text
bags/results_c3_final_8routes_cdh_updated/csv/c3_selected_run_pointers_final.csv
```

## 2. C3 pipeline meaning

C3 is the full visual-follow pipeline:

```text
camera / detector / tracker
-> leader_estimator
-> selected_target_filter
-> visual_target_estimator
-> follow_point_generator
-> follow_point_planner
-> visual_actuation_bridge
-> final UAV command
```

Important:

- C3 must not become C2/direct follow.
- `visual_actuation_bridge` is the final command source.
- `follow_uav.py` / `follow_uav_odom.py` must not compete as final controller in C3.
- The final command topic is:

```text
/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw
```

Logging mirror topics are also recorded:

```text
/dji0/pose_cmd
/dji0/pose_cmd/odom
```

## 3. C3 command pattern

Campaign-style command:

```bash
./run.sh results_campaign \
  --condition C3 \
  --runs <N> \
  --duration 300 \
  --warmup 30 \
  --world baylands \
  --gui false \
  --route-schedule rotundan,road_to_west,road_to_spawn,spawn,parkinglot_west,parkinglot_east,road_to_east,strip \
  --weights models/obb/mymodels/baylands-leader-v4-3.pt \
  --detector-backend ultralytics \
  --tracker true \
  --external-detection-node tracker \
  --conf-threshold 0.11 \
  --iou-threshold 0.3 \
  --visual-reacquire-assist true \
  --visual-reacquire-stale-timeout 2.0 \
  --visual-reacquire-return-fresh 1.0 \
  --out <output_folder>
```

The recorded per-run command internally used `tmux_1to1` like this:

```bash
./run.sh tmux_1to1 baylands \
  waypoint:=<route_waypoint> \
  nav2_goals:=<route_name> \
  gui:=false \
  delay_s:=60 \
  gazebo_ready_settle_s:=60 \
  record:=true \
  record_profile:=default \
  publish_pose_cmd_topics:=true \
  publish_camera_debug_topics:=true \
  mode:=yolo \
  yolo_control_mode:=visual_bridge \
  visual_follow_logic:=follow_core \
  detector_backend:=ultralytics \
  weights:=/home/william/halmstad_ws/models/obb/mymodels/baylands-leader-v4-3.pt \
  tracker:=true \
  external_detection_node:=tracker \
  visual_reacquire_assist_enable:=true \
  visual_reacquire_stale_timeout_s:=2.0 \
  visual_reacquire_return_fresh_s:=1.0 \
  visual_reacquire_source:=amcl_odom \
  detector_conf_threshold:=0.11 \
  detector_iou_threshold:=0.3
```

## 4. Route mapping

Use the same route mapping for C1/C2/C3/C4:

| Route | Route name | Waypoint |
|---|---|---|
| Route A | `rotundan` | `rotundan_0` |
| Route B | `road_to_west` | `road_to_west_0` |
| Route C | `road_to_spawn` | `road_to_spawn_0` |
| Route D | `spawn` | `spawn_0` |
| Route E | `parkinglot_west` | `parkinglot_west_0` |
| Route F | `parkinglot_east` | `parkinglot_east_0` |
| Route G | `road_to_east` | `road_to_east_0` |
| Route H | `strip` | `strip_0` |

## 5. Final selected C3 runs

| Route | Final selected runs | Source |
|---|---:|---|
| Route A | 5 | old accepted C3 |
| Route B | 5 | old accepted C3 |
| Route C | 3 | C/D/H rerun replacement |
| Route D | 5 | C/D/H rerun replacement |
| Route E | 5 | old accepted C3 |
| Route F | 5 | old accepted C3 |
| Route G | 5 | old accepted C3 |
| Route H | 5 | C/D/H rerun replacement |

Raw source folders:

```text
bags/results_c3_final_8routes/C3_bridge/
bags/results_c3_rerun_routes_cdh_batch01/C3_bridge/
bags/results_c3_rerun_routes_dh_extra_batch01/C3_bridge/
```

## 6. Key C3 default parameters

These come from:

```text
src/lrs_halmstad/config/run_follow_defaults.yaml
src/lrs_halmstad/launch/run_follow.launch.py
```

Detector/tracker:

| Parameter | Value |
|---|---|
| `backend` | `ultralytics` |
| `device` | `auto` |
| `imgsz` | `640` |
| `predict_hz` | `60.0` |
| `async_inference` | `true` |
| `latest_frame_only` | `true` |
| `tracker_config` | `bytetrack.yaml` |
| `image_qos_depth` | `1` |
| `image_qos_reliability` | `best_effort` |

Leader estimator:

| Parameter | Value |
|---|---|
| output | `/coord/leader_estimate` |
| status | `/coord/leader_estimate_status` |
| `est_hz` | `20.0` |
| `image_timeout_s` | `2.0` |
| `uav_pose_timeout_s` | `2.0` |
| `external_detection_timeout_s` | `2.5` |
| `range_mode` | `auto` |
| `depth_timeout_s` | `0.5` |
| `depth_min_m` | `1.0` |
| `depth_max_m` | `20.0` |

Note on detector stale latency:

- YAML has `stale_detection_threshold_ms: 3000.0`.
- The launch argument default is `500.0`.
- The final C3 command did not override it.
- Treat the active value as `500 ms` unless a ROS parameter dump says otherwise.

Visual pipeline:

| Stage | Important settings |
|---|---|
| `selected_target_filter` | strong `0.65`, weak `0.35`, min `0.08`, hold `0.45 s`, prediction gap `0.20 s` |
| `visual_target_estimator` | prediction gap `0.35 s`, degraded after `0.75 s`, hard lost after `1.35 s` |
| `follow_point_generator` | `follow_z_offset_m=7.0`, `require_motion_to_start=true`, `hold_timeout_s=0.5` |
| `follow_point_planner` | `xy_alpha=0.3`, `z_alpha=0.45`, max step `0.25 m`, hold `0.5 s` |
| `visual_actuation_bridge` | `input_mode=planned_target`, `publish_pose_cmd_mirror=true`, `use_current_altitude=true`, `max_xy_step_m=0.12`, `max_yaw_step_rad=0.06` |

Follow/control geometry:

| Parameter | Value |
|---|---|
| `d_target` | `10.0 m` |
| `follow_z_offset_m` | `7.0 m` |
| `follow_yaw` | `true` |
| `follow_yaw_rate_rad_s` | `0.8` |
| `follow_yaw_rate_gain` | `2.5` |
| `follow_speed_mps` | `5.0` |
| `follow_speed_gain` | `2.0` |

## 7. C2 comparison reference

C2 final package:

```text
bags/results_c2_final_8routes/
```

C2 uses the same detector/tracker/model settings as C3:

```text
ultralytics
baylands-leader-v4-3.pt
tracker=true
external_detection_node=tracker
conf=0.11
iou=0.3
```

But C2 control is direct estimate-follow:

```text
camera / detector / tracker
-> leader_estimator
-> follow_uav.py
-> final UAV command
```

C2 mode:

```text
yolo_control_mode=follow_uav_estimate
visual_follow_logic is not used
```

So the main difference is:

```text
C2 = direct leader_estimator -> follow_uav.py
C3 = full visual bridge chain -> visual_actuation_bridge
```

## 8. C4 reference for communication/OMNeT++ work

William's C4 simulation result is support-chain validation only. It does not include OMNeT++ communication metrics.

Final C4 source package:

```text
bags/results_c4_final_8routes_cpu/
```

C4 structure:

```text
dji1 support observation/status
dji2 support observation/status
-> dji0 support mux / aggregation
-> UGV support summary
-> UGV awareness status + path advisory
```

C4 important limits:

- dji0 is the main UAV.
- dji1 and dji2 are support UAVs.
- UGV outputs are monitor/advisory only.
- No active UGV replanning.
- No Nav2 costmap update.
- No OMNeT/radio in William's C4 sim results.
- No DirectML/GPU; final C4 used CPU support detectors.
- Support detections are support/object-observation availability, not UGV detection success.

Final C4 command pattern from metadata:

```bash
./run.sh tmux_support_chain baylands \
  waypoint:=<route_waypoint> \
  nav2_goals:=<route_name> \
  gui:=false \
  delay_s:=60 \
  gazebo_ready_settle_s:=60 \
  record:=true \
  record_profile:=support \
  publish_pose_cmd_topics:=true \
  publish_camera_debug_topics:=true \
  mode:=follow \
  support_mux_relation_source:=odom \
  support_mux_source_stale_timeout_s:=4.0 \
  support_camera_scan_enable:=true \
  support_bridge_gimbal:=true \
  support_detector_backend:=ultralytics \
  support_yolo_weights:=/home/william/halmstad_ws/models/obb/mymodels/baylands-leader-v4-3.pt
```

If Ruben adds communication/OMNeT++ on top, the support-chain baseline to preserve is:

```text
dji1/dji2 -> dji0 mux -> UGV advisory interface
```

Do not turn C4 into C1/C2/C3 follow-quality evaluation. C4 validates support information flow.

## 9. Files used to verify this handoff

Result metadata:

```text
bags/results_c3_final_8routes_cdh_updated/csv/c3_selected_runs_final_cdh_updated.csv
bags/results_c3_final_8routes_cdh_updated/csv/c3_selected_run_pointers_final.csv
bags/results_c3_final_8routes_cdh_updated/csv/c3_final_route_means.csv
bags/results_c3_final_8routes/C3_bridge/*/campaign_run.json
bags/results_c3_rerun_routes_cdh_batch01/C3_bridge/*/campaign_run.json
bags/results_c3_rerun_routes_dh_extra_batch01/C3_bridge/*/campaign_run.json
bags/results_c2_final_8routes/csv/c2_final_route_run_selection.csv
bags/results_c4_final_8routes_cpu/C4_support/*/campaign_run.json
```

Scripts/config:

```text
scripts/run_results_campaign.sh
scripts/run_1to1_yolo.sh
scripts/run_tmux_1to1.sh
src/lrs_halmstad/launch/run_follow.launch.py
src/lrs_halmstad/config/run_follow_defaults.yaml
src/lrs_halmstad/lrs_halmstad/perception/leader_estimator.py
src/lrs_halmstad/lrs_halmstad/follow/selected_target_filter.py
src/lrs_halmstad/lrs_halmstad/perception/visual_target_estimator.py
src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py
src/lrs_halmstad/lrs_halmstad/follow/follow_point_planner.py
src/lrs_halmstad/lrs_halmstad/follow/visual_actuation_bridge.py
src/lrs_halmstad/lrs_halmstad/follow/follow_uav.py
```

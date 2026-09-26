# Follow Modes And Defaults Guide

## Current Session Focus

Right now the most important practical workflows are dataset-capture workflows, not mode redesign.

Active data-collection priorities:
- leader UAV dataset from `dji0`
- support UAV datasets from `dji1` and `dji2`

Recommended current commands to remember:

Leader dataset collector:

```bash
./run.sh collect_leader_dataset baylands
```

Support capture workflow:

```bash
./run.sh tmux_1to1 baylands mode:=follow
./run.sh support_follow_odom baylands support_with_camera:=true support_bridge_gimbal:=true
./run.sh support_camera_scan baylands yaw_amplitude_deg:=120 period_s:=12 pan_phase_offsets_deg:=0,180 pitch_deg:=-22
./run.sh support_capture_pair baylands out:=datasets/baylands_support_parkinglot_west hz:=1.0 save_overlay:=false
```

Current known next-session issue:
- Baylands **AMCL poses for Nav2 waypoint points / route starts** still need work.
- If a Baylands collection run fails, do not assume the follow logic is wrong first; check AMCL / Nav2 waypoint alignment and startup readiness first.

Operator preference:
- reuse current code and scripts as much as possible
- avoid creating new files unless clearly necessary
- confirm the plan with the user before broader implementation changes

This guide explains how `src/lrs_halmstad/config/run_follow_defaults.yaml` maps to the launch scripts.

Important distinction:

- `run_follow_defaults.yaml` does not choose what runs by itself. It provides ROS parameters for nodes.
- The shell scripts and launch args choose the active mode, then the matching nodes read their section from the YAML.
- Most YAML values are startup parameters. When in doubt, edit the YAML and restart the launch. A few follow parameters can be set at runtime, but do not assume every node supports dynamic changes.

## Quick Choices

Use this when you trust/broadcast the UGV odom pose and want the simple Gazebo baseline:

```bash
./run.sh tmux_1to1 baylands mode:=follow
```

Use this for the current preferred YOLO path:

```bash
./run.sh tmux_1to1 baylands mode:=yolo tracker:=false
```

Use this for YOLO plus LoRa/OMNeT range:

```bash
./run.sh tmux_1to1 baylands mode:=yolo tracker:=false omnet:=true omnet_network:=lora
```

Use this if you want the old/experimental visual actuation bridge path:

```bash
./run.sh tmux_1to1 baylands mode:=yolo yolo_control_mode:=visual_bridge
```

Use this for the multi-UAV support-chain overlay:

```bash
./run.sh tmux_support_chain warehouse mode:=yolo
```

Dry-run any tmux command before launching:

```bash
./run.sh tmux_1to1 baylands mode:=yolo omnet:=true omnet_network:=lora dry_run:=true tmux_attach:=false
```

## Main Run Modes

### 1. `mode:=follow`

Pipeline:

```text
UGV odom or ground-truth odom -> follow_uav_odom -> UAV setpoint
```

What it does:

- Uses the actual UGV odom pose as the leader pose.
- Uses the simple anchor logic in `follow_uav_odom.py`.
- Places the UAV behind the UGV heading at `d_target`.
- Holds current UAV altitude.
- Publishes the anchor point on `/<uav>/follow/target/anchor_point`.
- Good for Gazebo baseline tests, not for strict "no UGV pose broadcast" estimate experiments.

Launch:

```bash
./run.sh tmux_1to1 baylands mode:=follow
```

Common options:

```bash
waypoint:=parkinglot_west_0          # UGV/Gazebo spawn waypoint in Baylands
nav2_goals:=parkinglot_west          # Nav2 goal YAML shorthand under config/baylands_waypoints
follow_yaw:=true                     # yaw UAV toward UGV
pan_enable:=false tilt_enable:=false # hold camera pan/tilt defaults
height:=7                            # UAV spawn/follow start height
publish_pose_cmd_topics:=false       # avoid pose_cmd mirror topics if you only want setpoints
```

Example:

```bash
./run.sh tmux_1to1 baylands mode:=follow waypoint:=parkinglot_west_0 nav2_goals:=parkinglot_west
```

### 2. `mode:=yolo`, simple estimate follow

Pipeline:

```text
camera image -> leader_detector or leader_tracker -> leader_estimator -> /coord/leader_estimate -> follow_uav -> UAV setpoint
```

What it does:

- This is the current recommended YOLO path.
- The UAV follows `/coord/leader_estimate`, not the actual UGV pose.
- `follow_uav.py` uses the same simple anchor idea as odom follow, but the leader pose comes from the estimator.
- In normal use, keep `leader_actual_pose_enable:=false`.
- The optional visual bridge pipeline is off unless explicitly requested.

Launch:

```bash
./run.sh tmux_1to1 baylands mode:=yolo tracker:=false
```

Common options:

```bash
weights:=baylands-v1-yolo26n-obb.pt  # model name under models/obb/mymodels, or full/relative path
tracker:=false                       # detector node only
tracker:=true                        # tracker node instead of detector
detector_backend:=ultralytics        # ultralytics, onnx_cpu, onnx_directml
yolo_device:=auto                    # auto, cpu, 0, cuda:1, etc.
target:=ugv                          # class-name filter, if model names support it
target_class_id:=0                   # class-id filter, -1 means any
range_mode:=auto                     # auto, depth, radio, const
pan_enable:=true tilt_enable:=true   # camera tracker gimbal behavior
follow_yaw:=true                     # yaw UAV toward estimated leader center
```

Range options:

- `range_mode:=auto`: estimator tries depth, then radio if available, then const fallback.
- `range_mode:=depth`: use the depth image from `/<uav>/camera0/depth_image`.
- `range_mode:=radio`: use `/omnet/radio_distance`; requires OMNeT metrics.
- `range_mode:=const`: use `d_target` as a fixed range fallback.

OMNeT/LoRa example:

```bash
./run.sh tmux_1to1 baylands mode:=yolo tracker:=false omnet:=true omnet_network:=lora pan_enable:=false tilt_enable:=false
```

Without OMNeT, `tmux_1to1` defaults the estimator range source to `range_mode:=auto`. With `omnet:=true`, it defaults to `range_mode:=radio`.

### 3. `mode:=yolo yolo_control_mode:=visual_bridge`

Pipeline:

```text
camera image
  -> leader_detector/tracker
  -> leader_estimator
  -> selected_target_filter
  -> visual_target_estimator
  -> follow_point_generator
  -> follow_point_planner
  -> visual_actuation_bridge
  -> UAV setpoint
```

What it does:

- This is the older/experimental visual actuation path.
- It does not use `follow_uav.py` for final actuation when the bridge is enabled.
- It starts the visual bridge and the follow-point pipeline.
- It has more recovery logic, prediction, filtering, and bridge motion shaping.
- We have generally seen the simple estimate follow path behave better, so keep this as an experiment path.

Launch:

```bash
./run.sh tmux_1to1 baylands mode:=yolo yolo_control_mode:=visual_bridge
```

Useful options:

```bash
visual_follow_logic:=legacy          # default visual pipeline behavior
visual_follow_logic:=follow_core     # use follow-core alignment in compatible visual nodes
start_visual_follow_controller:=true # use direct image-space controller path
follow_point_prefer_target_pose_heading:=true
follow_point_prefer_target_pose_position:=true
```

Bridge input modes are controlled inside `visual_actuation_bridge.input_mode`:

- `auto`: prefer planned target, fall back to follow point.
- `planned_target`: only use `follow_point_planner`.
- `follow_point`: skip planner and use follow points directly.
- `control`: use `visual_follow_controller` image-space control.

### 4. Support-chain mode

Pipeline:

```text
base dji0 run
  + dji1/dji2 support follow around dji0
  + dji1/dji2 support detectors
  + support_detection_mux
  + dji0_to_ugv_forwarder
```

What it does:

- Starts the normal 1-to-1 stack, then adds `dji1` and `dji2`.
- Support UAVs follow `dji0` in fixed slots.
- Support observations are summarized on `/coord/dji0/support_observation_summary`.
- UGV-facing support information is forwarded to `/coord/ugv/support_observation_summary`.
- Optional UGV status/advisory topics:
  - `/coord/ugv/support_awareness_status`
  - `/coord/ugv/support_path_advisory`

Launch:

```bash
./run.sh tmux_support_chain warehouse mode:=yolo
```

Useful options:

```bash
support_camera_scan_enable:=true
support_bridge_gimbal:=true
support_mux_relation_source:=odom
support_detector_backend:=onnx_cpu
support_detector_onnx_model:=/absolute/path/to/model.onnx
support_yolo_weights:=/absolute/path/to/model.pt
dji1_yolo_weights:=/absolute/path/to/dji1.pt
dji2_yolo_weights:=/absolute/path/to/dji2.pt
```

## Direct `run_follow.launch.py` Modes

The wrappers above eventually call:

```bash
ros2 launch lrs_halmstad run_follow.launch.py ...
```

Direct launch is useful for debugging, but the tmux scripts handle Gazebo, spawn, localization, Nav2, and OMNeT ordering for you.

Important direct launch args:

```bash
leader_mode:=odom       # start follow_uav_odom
leader_mode:=estimate   # start follow_uav using /coord/leader_estimate
leader_pose_topic:=/coord/leader_estimate
ugv_mode:=nav2          # run the internal UGV Nav2 driver
ugv_mode:=external      # do not send built-in Nav2 goals
start_leader_estimator:=true|false|auto
external_detection_enable:=true|false
external_detection_node:=detector|tracker
start_omnet_bridge:=true|false
```

Direct odom example:

```bash
ros2 launch lrs_halmstad run_follow.launch.py world:=baylands leader_mode:=odom ugv_mode:=nav2
```

Direct estimate example:

```bash
ros2 launch lrs_halmstad run_follow.launch.py \
  world:=baylands \
  leader_mode:=estimate \
  start_leader_estimator:=true \
  external_detection_enable:=true \
  external_detection_node:=detector \
  yolo_weights:=obb/mymodels/baylands-v1-yolo26n-obb.pt
```

## What Each YAML Section Does

### `/**`

Shared parameters inherited by multiple nodes.

Main values:

- `camera_x_offset_m`, `camera_y_offset_m`, `camera_z_offset_m`: camera position relative to UAV body.
- `camera_mount_pitch_deg`, `camera_yaw_offset_deg`, `camera_pan_sign`: camera/gimbal convention.
- `leader_look_target_x_m`, `leader_look_target_y_m`: point on the leader body for camera look-at geometry.
- `d_target`: canonical simple follow standoff. Used by `follow_uav`; also used by `leader_estimator` only as const/fallback range.

### `follow_uav`

Used by:

- `follow_uav.py` in estimate/pose mode.
- `follow_uav_odom.py` in odom mode.

What it controls:

- UAV standoff around the leader.
- Movement rate toward the anchor.
- Yaw-rate limiting toward the leader.
- Startup pose and whether to seed/hold commands.

Most useful values:

- `xy_anchor_max`: clamp the anchor radius around the leader.
- `follow_speed_mps`: max horizontal speed.
- `follow_speed_gain`: proportional speed gain toward the anchor.
- `follow_yaw`: turn UAV toward leader.
- `follow_yaw_rate_rad_s`: max yaw rate.
- `pose_timeout_s`: stale pose timeout.
- `publish_pose_cmd_topics`: mirror command on `/<uav>/pose_cmd` and `/<uav>/pose_cmd/odom`.

### `camera_tracker`

Runs the gimbal pan/tilt controller for the UAV camera.

What it controls:

- Whether pan/tilt actively track the target.
- Default pan/tilt when tracking is disabled.
- Image-center correction from YOLO boxes.
- Debug camera topics.

Common launch overrides:

```bash
pan_enable:=false
tilt_enable:=false
publish_camera_debug_topics:=false
```

If you want the camera to stay fixed, use:

```bash
pan_enable:=false tilt_enable:=false
```

### `uav_simulator`

Simulated UAV control bridge.

What it does:

- Reads UAV setpoints.
- Updates the Gazebo UAV pose.
- Applies gimbal pan/tilt commands.

Useful values:

- `pan_rate_deg_s`, `tilt_rate_deg_s`: gimbal slew rates.
- `startup_set_pose_grace_s`: avoids early Gazebo pose-service spam.
- `set_pose_failure_backoff_s`: retry backoff after Gazebo set-pose failures.

### `leader_detector`

Single-frame YOLO detector.

Pipeline:

```text
camera image -> leader_detector -> /coord/leader_detection
```

Use when:

- You want simpler detection without persistent track IDs.
- You launch with `tracker:=false` or `external_detection_node:=detector`.

Common options:

```bash
tracker:=false
weights:=baylands-v1-yolo26n-obb.pt
detector_backend:=ultralytics
detector_backend:=onnx_cpu
yolo_device:=auto
target_class_id:=-1
target:=some_class_name
```

### `leader_tracker`

YOLO tracker, usually ByteTrack.

Pipeline:

```text
camera image -> leader_tracker -> /coord/leader_detection
```

Use when:

- You want track continuity between frames.
- You launch with `tracker:=true` or `external_detection_node:=tracker`.

Common options:

```bash
tracker:=true
tracker_config:=bytetrack.yaml
tracker_config:=botsort.yaml
```

### `leader_estimator`

Turns detections into a world-frame leader pose estimate.

Pipeline:

```text
/coord/leader_detection + camera info + UAV pose + range source -> /coord/leader_estimate
```

What it controls:

- Detection freshness.
- Image/depth/UAV pose timeout handling.
- Range source for projecting the detection into world coordinates.
- Diagnostic status and debug image output.

Important range options:

- `range_mode: auto`: try depth/radio/const according to available sources.
- `range_mode: depth`: use depth image only.
- `range_mode: radio`: use `/omnet/radio_distance`.
- `range_mode: const`: use `d_target` as fixed range.

Launch override:

```bash
range_mode:=auto
range_mode:=depth
range_mode:=radio
range_mode:=const
```

Important truth/diagnostic warning:

- `leader_actual_pose_enable` should stay `false` for estimate-only experiments.
- `leader_actual_pose_topic` may exist for diagnostics or Baylands mapping, but the estimator does not use it unless `leader_actual_pose_enable:=true`.

### `ugv_nav2_driver`

Sends goals to Nav2 for the UGV.

What it controls:

- Initial pose publication.
- Goal sequence loading.
- Goal retry behavior.
- Internal generated route behavior when no file/csv route is supplied.

Common launch options:

```bash
nav2_goals:=parkinglot_west
nav2_goals:=art
ugv_goal_sequence_csv:="1,0,0;2,0,90"
ugv_goal_sequence_randomize:=false
ugv_goal_sequence_random_reverse:=false
ugv_goal_sequence_relative_to_current_pose:=false
ugv_start_delay_s:=3.0
```

Baylands shorthand:

- `nav2_goals:=parkinglot_west` resolves to `config/baylands_waypoints/baylands_waypoints_parkinglot_west.yaml`.
- `nav2_goals:=art` resolves to `config/baylands_waypoints/baylands_waypoints_art.yaml`.

### `selected_target_filter`

First stage of the optional visual bridge pipeline.

What it does:

- Filters raw detector/tracker targets.
- Drops very weak detections.
- Holds/predicts briefly through short detector gaps.
- Publishes `/coord/leader_selected_target_filtered`.

Only active when at least one visual pipeline node is started.

### `visual_target_estimator`

Optional visual pipeline estimator.

What it does:

- Converts selected image target state into a visual target estimate.
- Can estimate range from projected geometry or bbox area.
- Produces `/coord/leader_visual_target_estimate`.

Main mode:

- `range_signal_mode: auto`: choose projected/area fallback internally.
- `projected`: use projected range path.
- `area`: use bbox-area range estimate.

### `visual_follow_controller`

Optional image-space controller.

What it does:

- Converts image error and range error into yaw/forward control.
- Can be used by `visual_actuation_bridge` with `input_mode:=control`.

Usually off in the current simple YOLO path.

### `follow_point_generator`

Optional visual pipeline spatial target generator.

What it does:

- Turns the visual target estimate into a world-frame follow point.
- Uses shared `d_target` for standoff and `uav_start_z` for nominal altitude.
- Can wait for target motion before it starts moving the follow point.

Important:

- The visual bridge still has its own recovery scales, smoothing, and planner settings.
- It no longer has separate nominal distance/altitude defaults.

### `follow_point_planner`

Optional visual pipeline smoother.

What it does:

- Smooths follow point position, altitude, and yaw.
- Limits step size so bridge targets do not jump too hard.
- Publishes `/coord/leader_planned_target`.

Usually active only for:

```bash
yolo_control_mode:=visual_bridge
```

### `visual_actuation_bridge`

Final actuator for the optional visual bridge pipeline.

What it does:

- Converts visual control, follow point, or planned target into UAV setpoints.
- When enabled, it disables the normal `follow_uav`/`follow_uav_odom` actuation path.

Input modes:

- `auto`: prefer planned target, fall back to follow point.
- `planned_target`: use planner output only.
- `follow_point`: use generator output directly.
- `control`: use image-space visual controller output.

Useful values:

- `max_xy_step_m`: bridge movement limit per tick.
- `max_yaw_step_rad`: yaw limit per tick.
- `publish_pose_cmd_mirror`: publish mirror pose command for debugging.
- `predicted_step_scale`, `degraded_step_scale`: conservative scaling during weaker visual states.

## Option Cheat Sheet

World/start:

```bash
world: baylands is selected by the positional argument in ./run.sh tmux_1to1 baylands
gui:=true|false
waypoint:=parkinglot_west_0
height:=7
uav_name:=dji0
```

UGV Nav2:

```bash
nav2_goals:=parkinglot_west
ugv_goal_sequence_csv:="x,y,yaw_deg;x,y,yaw_deg"
ugv_goal_sequence_randomize:=false
ugv_goal_sequence_relative_to_current_pose:=false
```

YOLO:

```bash
weights:=baylands-v1-yolo26n-obb.pt
tracker:=false
tracker:=true
detector_backend:=ultralytics|onnx_cpu|onnx_directml
yolo_device:=auto|cpu|0|cuda:1
target:=class_name
target_class_id:=0
```

Range:

```bash
range_mode:=auto|depth|radio|const
omnet:=true omnet_network:=lora
```

Camera:

```bash
pan_enable:=true|false
tilt_enable:=true|false
camera_default_tilt_deg:=-45
mount_pitch_deg:=45
publish_camera_debug_topics:=true|false
```

Follow:

```bash
follow_yaw:=true|false
publish_pose_cmd_topics:=true|false
uav_start_delay_s:=0.0
ugv_start_delay_s:=3.0
```

Visual bridge:

```bash
yolo_control_mode:=visual_bridge
visual_follow_logic:=legacy|follow_core
start_visual_follow_controller:=true|false
start_visual_follow_point_generator:=true|false
start_visual_follow_planner:=true|false
start_visual_actuation_bridge:=true|false
```

Support chain:

```bash
support_camera_scan_enable:=true|false
support_bridge_gimbal:=true|false
support_detector_backend:=ultralytics|onnx_cpu|onnx_directml
support_yolo_weights:=/path/to/model.pt
dji1_yolo_weights:=/path/to/model.pt
dji2_yolo_weights:=/path/to/model.pt
```

## Recommended Defaults Right Now

For normal Baylands YOLO follow:

```bash
./run.sh tmux_1to1 baylands mode:=yolo tracker:=false omnet:=true omnet_network:=lora pan_enable:=false tilt_enable:=false
```

For baseline Gazebo odom follow:

```bash
./run.sh tmux_1to1 baylands mode:=follow
```

For support-chain evidence collection:

```bash
./run.sh tmux_support_chain warehouse mode:=yolo support_camera_scan_enable:=true support_bridge_gimbal:=true
```

## Current State

Assumption:
- start in the workspace root
- wrapper files now live under `scripts/`
- user-facing entrypoints are `./run.sh <name>` and `./stop.sh <name>`

### Active Baseline

Recommended start/stop flow:
1. `./run.sh tmux_1to1 warehouse`
2. `./stop.sh tmux_1to1 warehouse`

Recommended tmux variants:
- normal follow: `./run.sh tmux_1to1 warehouse`
- YOLO detection path: `./run.sh tmux_1to1 warehouse mode:=yolo`
- YOLO tracker path: `./run.sh tmux_1to1 warehouse mode:=yolo tracker:=true`

Current tmux default:
- `layout:=panes`
- `gui:=false`
- row 1: `gazebo | spawn`
- row 2: `localization | nav2`
- row 3: `follow`
- `record:=false`

Current tmux default delays:
- `gui:=true` -> `spawn=6s`, `localization/nav2=8s`, `follow=10s`
- `gui:=false` -> `spawn=8s`, `localization/nav2=10s`, `follow=12s`

Per-stage delay overrides now exist:
- `spawn_delay_s:=...`
- `localization_delay_s:=...`
- `nav2_delay_s:=...`
- `follow_delay_s:=...`

New tmux extras:
- `record:=true|false`
- `record_profile:=default|vision`
- `record_tag:=...`
- `record_out:=bags/experiments/...`
- `record_delay_s:=...`
- `rtf:=...` / `real_time_factor:=...`

Equivalent manual run order:
1. `./run.sh gazebo_sim warehouse`
2. `./run.sh spawn_uav warehouse`
3. `./run.sh localization warehouse`
4. `./run.sh nav2`
5. `./run.sh 1to1_follow warehouse`

### Main Runtime Chain

Top-level wrapper chain:
- `scripts/run_tmux_1to1.sh`
  - dispatches through `./run.sh gazebo_sim`
  - dispatches through `./run.sh spawn_uav`
  - dispatches through `./run.sh localization`
  - dispatches through `./run.sh nav2`
  - dispatches through `./run.sh 1to1_follow` or `./run.sh 1to1_yolo`

Follow launch chain:
- `scripts/run_1to1_follow.sh`
  - launches `src/lrs_halmstad/launch/run_follow.launch.py`
- `run_follow.launch.py`
  - starts the runtime nodes:
    - `uav_simulator`
    - `ugv_amcl_to_odom` (`pose_cov_to_odom`)
    - `follow_uav_odom` or `follow_uav`
    - `camera_tracker`
    - `ugv_nav2_driver`
    - optional perception node: `leader_detector` or `leader_tracker`
    - optional `leader_estimator`
- `run_follow_motion.launch.py`
  - compatibility shim including `run_follow.launch.py`
- `run_1to1_follow.launch.py`
  - compatibility shim including `run_follow.launch.py`

Spawn chain:
- `scripts/run_spawn_uav.sh`
  - launches `spawn_uav_1to1.launch.py`
- `spawn_uav_1to1.launch.py`
  - always spawns the UAV body
  - spawns detached camera/gimbal when `uav_camera_mode:=detached_model`
  - starts the pose bridge and delayed camera bridge

Important separation:
- `scripts/run_spawn_uav.sh` creates the Gazebo entities
- `scripts/run_1to1_follow.sh` / `scripts/run_1to1_yolo.sh` start the runtime logic
- nothing actually follows until the follow launch is running

Current Python package layout under `src/lrs_halmstad/lrs_halmstad`:
- `perception/`
  - `leader_detector.py`
  - `leader_tracker.py`
  - `leader_estimator.py`
  - shared helpers: `detection_protocol.py`, `yolo_common.py`
- `follow/`
  - `follow_uav.py`
  - `follow_uav_odom.py`
  - `camera_tracker.py`
  - `follow_debug.py`
  - `follow_math.py`
- `sim/`
  - `simulator.py`
  - `controller.py`
  - `clock_guard.py`
  - `pose_cov_to_odom.py`
  - `pose_cmd_to_odom.py`
  - `gazebo_pose_tcp_bridge.py`
- `nav/`
  - `ugv_nav2_driver.py`
  - `ugv_nav2_goal_tester.py`
  - `ugv_motion_profile.py`
- `dataset/`
  - `sim_dataset_capture.py`
  - `make_obb.py`
  - `sync_check.py`
- `tools/`
  - `follow_control.py`
  - `uav_command_logger.py`
- `common/`
  - `world_names.py`
  - `paths.py`

### UGV / Nav2 State

The validated UGV truth path is still:
- `/<ugv>/amcl_pose`
- `pose_cov_to_odom`
- `/<ugv>/amcl_pose_odom`

Important interpretation:
- `leader_mode:=odom` means "consume an `Odometry` topic"
- the validated odom topic is `/<ugv>/amcl_pose_odom`
- do not switch the active follow path back to raw `/platform/odom`
- do not switch dataset capture back to raw `/platform/odom`

Current relevant files:
- `src/lrs_halmstad/launch/run_follow.launch.py`
- `src/lrs_halmstad/lrs_halmstad/sim/pose_cov_to_odom.py`
- `src/lrs_halmstad/lrs_halmstad/follow/follow_uav_odom.py`
- `src/lrs_halmstad/lrs_halmstad/nav/ugv_nav2_driver.py`
- `src/lrs_halmstad/lrs_halmstad/dataset/sim_dataset_capture.py`

Clearpath path:
- Gazebo sim, localization, Nav2, and SLAM now use the workspace Clearpath copy:
  - `src/lrs_halmstad/clearpath`
- the old `/home/ruben/clearpath` path is no longer the active runtime path

### Perception Split

This is the biggest structural change from earlier in the project.

Current active perception architecture:
- `perception/leader_detector.py`
  - plain detection node
  - runs a model on camera images
  - publishes the best detection to `/coord/leader_detection`
  - publishes detector-owned detection status to `/coord/leader_detection_status`
- `perception/leader_tracker.py`
  - separate Ultralytics `track()` node
  - uses tracker configs from `src/lrs_halmstad/config/trackers`
  - also publishes tracked detections to `/coord/leader_detection`
  - also publishes tracker-owned detection status to `/coord/leader_detection_status`
- `perception/leader_estimator.py`
  - estimator-only node
  - consumes `/coord/leader_detection`
  - consumes `/coord/leader_detection_status` only for debug-image overlay
  - projects detection into world pose
  - may use OBB corners to derive heading
  - publishes `/coord/leader_estimate`, status, fault, debug image, and optional error-to-truth
  - estimate status no longer mirrors tracker-owned detection metadata

Important consequence:
- `perception/leader_estimator.py` is no longer the predictor/tracker brain
- the prediction backend is now swappable
- detector and tracker are separate runtime choices

Current selection logic:
- `external_detection_node:=detector`
  - starts `leader_detector`
- `external_detection_node:=tracker`
  - starts `leader_tracker`
- `./run.sh 1to1_yolo tracker:=true`
  - sets `external_detection_node:=tracker`

### YOLO / Tracker Path

Active YOLO wrapper:
- `./run.sh 1to1_yolo warehouse`

Current default behavior of `scripts/run_1to1_yolo.sh`:
- `use_estimate:=true`
- `leader_mode:=estimate`
- `start_leader_estimator:=true`
- `external_detection_enable:=true`
- `external_detection_node:=detector`
- `tracker:=false`
- `obb:=true`
- `leader_range_mode:=auto`
- default weights with no override:
  - `obb/mymodels/warehouse-v1-yolo26n-obb.pt`
- important override nuance:
  - `run_follow.launch.py` still declares `leader_actual_pose_enable`, `publish_follow_debug_topics`, `publish_pose_cmd_topics`, and `publish_camera_debug_topics` with launch-default `true`
  - that means quiet mode is guaranteed by `scripts/run_1to1_yolo.sh`, not by a bare `ros2 launch lrs_halmstad run_follow.launch.py ...`
- `leader_actual_pose_enable:=false`
- `publish_follow_debug_topics:=false`
- `publish_pose_cmd_topics:=false`
- `publish_camera_debug_topics:=false`
- if `use_estimate:=true` and not overridden:
  - `uav_start_x:=-7.0`
  - `uav_start_z:=7.0`
  - `leader_actual_heading_enable:=true`
- if you want estimator-only heading behavior instead of shared AMCL heading:
  - pass `use_actual_heading:=false`

Current test variants:
- shared truth pose control instead of estimate:
  - `./run.sh 1to1_yolo warehouse use_estimate:=false`
- tracker instead of plain detector:
  - `./run.sh 1to1_yolo warehouse tracker:=true`
- estimator-only heading instead of shared actual heading:
  - `./run.sh 1to1_yolo warehouse use_actual_heading:=false`
- explicit tracker config:
  - `./run.sh 1to1_yolo warehouse tracker:=true tracker_config:=botsort.yaml`
  - bare tracker config names resolve under `src/lrs_halmstad/config/trackers`
- plain detection-family weights instead of the OBB default:
  - `./run.sh 1to1_yolo warehouse obb:=false`

Current weights resolution:
- detection default root: `models/detection/mymodels`
- OBB default root: `models/obb/mymodels`
- plain detector default weight when `obb:=false`:
  - `detection/mymodels/warehouse_v1-v2-yolo26n.pt`
- OBB default weight:
  - `obb/mymodels/warehouse-v1-yolo26n-obb.pt`
- bare `weights:=foo.pt` resolution:
  - `tracker:=false` -> resolve under `detection/...`
  - `tracker:=true` -> resolve under `obb/...`

### Follow Data Flow

Normal odom follow:
- `follow_uav_odom`
  - subscribes to `/<ugv>/amcl_pose_odom`
  - subscribes to `/<uav>/pose`
  - can publish `/<uav>/pose_cmd`
  - can publish `/<uav>/pose_cmd/odom`

Estimate follow:
- `leader_detector` or `leader_tracker`
  - publishes `/coord/leader_detection`
  - publishes `/coord/leader_detection_status`
- `leader_estimator`
  - consumes `/coord/leader_detection`
  - consumes `/coord/leader_detection_status` for debug-image overlay only
  - publishes `/coord/leader_estimate`
  - publishes `/coord/leader_estimate_status`
  - publishes `/coord/leader_estimate_fault`
- `follow_uav`
  - subscribes to `/coord/leader_estimate`
  - may also use shared AMCL heading if enabled
  - can publish legacy `/<uav>/pose_cmd*` helper topics when enabled
  - can publish `/<uav>/follow/{target,actual,error,debug}/*` topics when enabled

Camera side:
- `camera_tracker`
  - subscribes to the same leader topic used by the active follow path
  - also listens to `/coord/leader_estimate_status`
  - publishes:
    - `/<uav>/update_pan`
    - `/<uav>/update_tilt`
    - optional `/<uav>/camera/target/*` debug topics

Simulator side:
- `uav_simulator`
  - consumes `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
  - consumes `/<uav>/update_pan` and `/<uav>/update_tilt`
  - publishes `/<uav>/pose` and follow/camera debug topics

### Current Active Debug Target

Current live focus in YOLO detect/track follow:
- estimator/follow behavior is much better than earlier, but close-range behavior is still the main problem
- the UAV can struggle when the UGV passes underneath or tries to cross the UAV path
- the estimator can still end up holding the fallback/const range at the wrong moment
- body yaw on sharp turns / reversing and jumpy camera pan/tilt are the active runtime tuning/debug areas

Where to start:
- `follow/follow_uav.py`
- `follow/camera_tracker.py`
- `sim/simulator.py`
- `perception/leader_estimator.py`

Relevant topics to inspect:
- `/coord/leader_detection`
- `/coord/leader_detection_status`
- `/coord/leader_estimate`
- `/coord/leader_estimate_status`
- `/coord/leader_debug_image`
- `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- `/<uav>/update_pan`
- `/<uav>/update_tilt`

Important current assumption:
- this does not look like a missing detector/tracker wiring problem anymore
- it looks more like a runtime interaction between estimate state, body yaw/XY commands, and camera pan/tilt commands

### Camera Baseline

Current validated baseline:
- detached camera model
- spawned UAV camera is now RGBD-capable by default
- existing RGB topics remain:
  - `/<uav>/camera0/image_raw`
  - `/<uav>/camera0/camera_info`
- additional UAV depth topic is now available:
  - `/<uav>/camera0/depth_image`
- `uav_camera_mode:=detached_model`
- `pan_enable: true`
- `tilt_enable: true`
- `default_pan_deg: 0.0`
- `default_tilt_deg: -45.0`

Important consequence:
- `camera:=attached` is now an override path, not the default baseline
- detached remains the primary path for Gazebo visual correctness

### Current Defaults Worth Remembering

From `run_follow_defaults.yaml`:

Follow controller:
- `d_target: 10.0`
- `xy_min: 1.0`
- `z_min: 5.0`
- `tick_hz: 20.0`
- `follow_speed_mps: 5.0`
- `follow_speed_gain: 1.2`
- `follow_z_speed_mps: 5.0`
- `follow_z_speed_gain: 1.0`
- `follow_yaw_rate_rad_s: 4.0`
- `follow_yaw_rate_gain: 3.0`
- `follow_yaw_accel_rad_s2: 12.0`
- `leader_actual_heading_enable: false` in YAML
- `estimate_heading_from_motion_enable: true`
- `traj_max_speed_mps: 5.0`
- `traj_max_accel_mps2: 4.0`
- `traj_pos_gain: 1.0`

Camera tracker:
- `tick_hz: 30.0`
- `default_pan_deg: 0.0`
- `default_tilt_deg: -45.0`
- `pan_enable: true`
- `tilt_enable: true`
- `image_center_correction_enable: true`
- `pan_image_center_gain: 0.25`
- `pan_image_center_max_deg: 3.0`
- `tilt_image_center_gain: 0.5`
- `tilt_image_center_max_deg: 12.0`
- tracks from actual UAV pose consistently; the older cmd-pose/actual-pose switching path was removed

Estimator:
- `est_hz: 20.0`
- `range_mode: auto`
- `d_target` now seeds the estimator const-mode target too; the old explicit `constant_range_m` launch override is no longer part of the normal run path
- `use_depth_range: false`
- `external_detection_timeout_s: 2.0`
- const fallback now uses a recomputed target-based range (`const_target`), not the old shrinking `const_hold` XY reuse

Detector:
- `predict_hz: 60.0`
- `conf_threshold: 0.4`
- `bbox_continuity_weight: 0.10`
- `bbox_continuity_max_px: 180.0`

Tracker:
- `predict_hz: 60.0`
- `tracker_config: bytetrack.yaml` in YAML
- launch default still overrides tracker config to `botsort.yaml` unless you pass your own override
- `conf_threshold: 0.4`
- `iou_threshold: 0.5`

Important effective-runtime nuance:
- `run_follow.launch.py` still sets launch-default `tracker_config:=botsort.yaml`
- `run_follow.launch.py` still sets launch-default `leader_actual_pose_enable:=true`
- `run_follow.launch.py` still sets launch-default `publish_follow_debug_topics:=true`
- `run_follow.launch.py` still sets launch-default `publish_pose_cmd_topics:=true`
- `run_follow.launch.py` still sets launch-default `publish_camera_debug_topics:=true`
- `scripts/run_1to1_yolo.sh` overrides those to a quieter runtime unless explicitly told otherwise

### Runtime Helpers

Moved helper wrappers now live under `scripts/` and can be launched either
directly as `./scripts/run_<name>.sh` or through the root dispatcher as
`./run.sh <name>`.

`scripts/run_follow_control.sh`
- live tuning helper for follow parameters
- wrapper now resolves the installed `run_follow_control` entrypoint from `lrs_halmstad.tools.follow_control`
- current useful modes:
  - default keyboard mode for `d_target` / `z_min`
  - `--mode params`
  - `--mode random`
- the temporary direct manual pose-steering override path was removed

`uav_command_logger.py`
- the preferred runtime command logger for follow/camera behavior
- logs under:
  - `debug_logs/uav_commands/`
- useful for:
  - body yaw vs camera pan separation
  - anchor/XY error
  - heading-source inspection
  - estimator state/fault visibility

### Dataset State

Current rule:
- new dataset capture must use `/<ugv>/amcl_pose_odom`
- `./run.sh capture_dataset` already enforces this

Current dataset additions:
- `scripts/run_dataset_make_obb.sh`
  - wrapper for installed `run_dataset_make_obb` from `lrs_halmstad.dataset.make_obb`
  - creates `labels_obb/` from saved metadata/projected points
- current generated OBB label dirs:
  - `datasets/warehouse_v1/run1/labels_obb`
  - `datasets/warehouse_v1/run2/labels_obb`

Current dataset note:
- `datasets/warehouse_v1/run1` and `run2` now have OBB labels generated separately under `labels_obb`
- plain detection labels under `labels/` were left untouched

### What Is Obsolete

These old descriptions should not be trusted anymore:
- `leader_estimator` as the monolithic YOLO + tracking + debounce + hold + reject node
- notes about estimator hold/debounce reuse being the main active behavior
- notes about the old direct YOLO path through `run_yolo.sh` / `run_follow_yolo.launch.py`
- notes about manual pose-steering keyboard override in `scripts/run_follow_control.sh`
- notes about the old `/home/ruben/clearpath` runtime path

Also obsolete in the runbook:
- `run_record_pose_alignment.sh`
- `run_analyze_pose_alignment.py`

Those files are not present in the workspace and are not part of the active workflow anymore.

### Guardrails

Things that should not be "fixed back":
- do not switch follow back to raw `/platform/odom`
- do not switch dataset capture back to raw `/platform/odom`
- do not fold model prediction back into `leader_estimator.py`
- do not add tracker logic into `leader_detector.py`
- keep predictor/tracker/estimator as separate concerns
- do not remove `clock_guard` unless the backward-jump problem is proven fixed
- do not treat attached camera mode as the default baseline

### Current Loose Ends

Important current handoff note:
- the package split is now the only active module layout
- the old top-level compatibility shims have been removed
- the only compatibility shims still intentionally left are the launch-file shims:
  - `run_follow_motion.launch.py`
  - `run_1to1_follow.launch.py`

Current priority is not another refactor:
- first debug the YOLO follow movement/camera snap behavior in live sim
- only refactor again after the motion cause is understood

### Build / Validation State

Current validation baseline:
1. `colcon build --symlink-install --packages-select lrs_halmstad` passes
2. the wrapper/launch rename to `run_follow.launch.py` is in place
3. the runtime stack starts successfully through the current wrapper flow
4. detector/tracker follow still needs focused live debugging for the movement issue above

Recommended next-session verification order:
1. clean/rebuild `lrs_halmstad`
2. restart the tmux stack in YOLO mode
3. verify which external perception node is active:
   - `leader_detector`
   - or `leader_tracker`
4. verify `/coord/leader_detection` is alive
5. verify `leader_estimator` is consuming it
6. inspect `/coord/leader_estimate_status`, `/coord/leader_debug_image`, and the UAV pan/tilt + body command topics during the camera-snap event

If a new chat continues from here, start by reading:
- this file
- `RUNNING_SIM.md`

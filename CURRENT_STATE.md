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
- row 1: `gazebo | spawn`
- row 2: `localization | nav2`
- row 3: `follow`

Current tmux default delays:
- `gui:=true` -> `spawn=7s`, `localization/nav2=9s`, `follow=13s`
- `gui:=false` -> `spawn=9s`, `localization/nav2=11s`, `follow=15s`

Per-stage delay overrides now exist:
- `spawn_delay_s:=...`
- `localization_delay_s:=...`
- `nav2_delay_s:=...`
- `follow_delay_s:=...`

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
  - launches `src/lrs_halmstad/launch/run_1to1_follow.launch.py`
- `run_1to1_follow.launch.py`
  - mostly forwards world/runtime arguments
  - includes `src/lrs_halmstad/launch/run_follow_motion.launch.py`
- `run_follow_motion.launch.py`
  - starts the runtime nodes:
    - `uav_simulator`
    - `ugv_amcl_to_odom` (`pose_cov_to_odom`)
    - `follow_uav_odom` or `follow_uav`
    - `camera_tracker`
    - `ugv_nav2_driver`
    - optional perception node: `leader_detector` or `leader_tracker`
    - optional `leader_estimator`

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
  - `follow_debug_cli.py`
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
- `src/lrs_halmstad/launch/run_follow_motion.launch.py`
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
  - plain prediction node
  - runs a model on camera images
  - publishes the best detection to `/coord/leader_detection`
- `perception/leader_tracker.py`
  - separate Ultralytics `track()` node
  - uses tracker configs from `src/lrs_halmstad/config/trackers`
  - also publishes tracked detections to `/coord/leader_detection`
- `perception/leader_estimator.py`
  - estimator-only node
  - consumes `/coord/leader_detection`
  - projects detection into world pose
  - may use OBB corners to derive heading
  - publishes `/coord/leader_estimate`, status, fault, debug image, and optional error-to-truth

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
- `leader_range_mode:=ground`
- if `use_estimate:=true` and not overridden:
  - `startup_reposition_enable:=true`
  - `uav_start_x:=-7.0`
  - `uav_start_z:=7.0`
  - `leader_actual_heading_enable:=true`

Current test variants:
- shared truth pose control instead of estimate:
  - `./run.sh 1to1_yolo warehouse use_estimate:=false`
- tracker instead of plain detector:
  - `./run.sh 1to1_yolo warehouse tracker:=true`
- explicit tracker config:
  - `./run.sh 1to1_yolo warehouse tracker:=true tracker_config:=botsort.yaml`
- OBB weights:
  - `./run.sh 1to1_yolo warehouse obb:=true`

Current weights resolution:
- detection default root: `models/detection/mymodels`
- OBB default root: `models/obb/mymodels`
- plain detector default weight:
  - `detection/mymodels/warehouse_v1-v1-yolo26n.pt`
- OBB default weight:
  - `obb/mymodels/warehouse-v1-yolo26n-obb.pt`

### Follow Data Flow

Normal odom follow:
- `follow_uav_odom`
  - subscribes to `/<ugv>/amcl_pose_odom`
  - subscribes to `/<uav>/pose`
  - publishes `/<uav>/pose_cmd`
  - publishes `/<uav>/pose_cmd/odom`

Estimate follow:
- `leader_detector` or `leader_tracker`
  - publishes `/coord/leader_detection`
- `leader_estimator`
  - publishes `/coord/leader_estimate`
  - publishes `/coord/leader_estimate_status`
  - publishes `/coord/leader_estimate_fault`
- `follow_uav`
  - subscribes to `/coord/leader_estimate`
  - may also use shared AMCL heading if enabled

Camera side:
- `camera_tracker`
  - subscribes to the same leader topic used by the active follow path
  - also listens to `/coord/leader_estimate_status`
  - publishes:
    - `/<uav>/update_pan`
    - `/<uav>/update_tilt`
    - `/<uav>/camera/target/*`

Simulator side:
- `uav_simulator`
  - consumes `/<uav>/pose_cmd`
  - consumes `/<uav>/update_pan` and `/<uav>/update_tilt`
  - publishes `/<uav>/pose` and follow/camera debug topics

### Camera Baseline

Current validated baseline:
- detached camera model
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
- `d_target: 7.0`
- `z_alt: 7.0`
- `tick_hz: 20.0`
- `follow_speed_mps: 5.0`
- `follow_speed_gain: 2.0`
- `follow_z_speed_mps: 8.0`
- `follow_z_speed_gain: 3.0`
- `follow_yaw_rate_rad_s: 3.0`
- `follow_yaw_rate_gain: 2.0`
- `yaw_deadband_rad: 0.02`
- `yaw_update_xy_gate_m: 0.0`

Camera tracker:
- `tick_hz: 20.0`
- `default_pan_deg: 0.0`
- `default_tilt_deg: -45.0`
- `pan_enable: true`
- `tilt_enable: true`

Estimator:
- `est_hz: 20.0`
- `range_mode: auto`
- `constant_range_m: 7.0`
- `use_depth_range: true`
- `external_detection_timeout_s: 1.0`

Detector:
- `predict_hz: 20.0`
- `conf_threshold: 0.12`
- `bbox_continuity_weight: 0.10`
- `bbox_continuity_max_px: 180.0`

Tracker:
- `predict_hz: 20.0`
- `tracker_config: botsort.yaml`
- `conf_threshold: 0.12`
- `iou_threshold: 0.45`

### Runtime Helpers

Moved helper wrappers now live under `scripts/` and can be launched either
directly as `./scripts/run_<name>.sh` or through the root dispatcher as
`./run.sh <name>`.

`scripts/run_follow_control.sh`
- live tuning helper for follow parameters
- wrapper now resolves the installed `run_follow_control` entrypoint from `lrs_halmstad.tools.follow_control`
- current useful modes:
  - default keyboard mode for `d_target` / `z_alt`
  - `--mode params`
  - `--mode random`
- the temporary direct manual pose-steering override path was removed

`scripts/run_follow_debug.sh`
- runtime analyzer for follow/camera behavior
- wrapper now resolves the installed `run_follow_debug` entrypoint from `lrs_halmstad.tools.follow_debug_cli`
- logs under:
  - `debug_logs/follow_debug/`
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

Validation state of the latest refactor:
- `python3 -m py_compile` passed for the touched Python and launch files
- `bash -n` passed for the touched shell wrappers
- `colcon build --symlink-install --packages-select lrs_halmstad` passed after the package split
- a full end-to-end ROS/Gazebo verification was **not** run after the package split

### Build / Validation State

If testing the new perception split:
1. clean/rebuild `lrs_halmstad`
2. restart the tmux stack
3. verify which external perception node is active:
   - `leader_detector`
   - or `leader_tracker`
4. verify `/coord/leader_detection` is alive
5. verify `leader_estimator` is consuming it

If a new chat continues from here, start by reading:
- this file
- `RUNNING_SIM.md`

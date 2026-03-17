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

Current tmux startup behavior:
- the tmux wrapper now relies on staged startup delays and cleanup, but it no longer blocks the follow pane on Nav2-localization readiness checks
- the driver-side guard now carries the Nav2 readiness responsibility, which avoids circular startup waits during AMCL initial-pose seeding
- `ugv_nav2_driver` now also performs its own wait for those required Nav2 lifecycle nodes before sending the first goal and before retrying a rejected goal, so the guard does not depend only on tmux orchestration
- on WSL, `gazebo_sim` now defaults to `LIBGL_ALWAYS_SOFTWARE=1` for GUI stability unless you override it
- tmux start/stop safety cleanup now also kills leftover child ROS nodes from the 1-to-1 stack, not just the top-level `ros2 launch ...` wrappers
- `camera_tracker` now uses an odom-frame UGV pose as a camera-only reacquisition fallback when estimate tracking starts in `NO_DET`, so the detached camera can re-aim toward the UGV without mixing `map` and `odom` geometry or handing motion control back to truth pose

Per-stage delay overrides now exist:
- `spawn_delay_s:=...`
- `localization_delay_s:=...`
- `nav2_delay_s:=...`
- `follow_delay_s:=...`

New tmux extras:
- `record:=true|false`
- `record_profile:=default|step2_light|vision`
- `record_tag:=...`
- `record_out:=bags/experiments/...`
- `record_delay_s:=...`
- `rtf:=...` / `real_time_factor:=...`
- in `mode:=yolo`, the recorder now also captures the Step 2 handoff/control topics:
  - `/coord/leader_detection`
  - `/coord/leader_detection_status`
  - `/coord/leader_estimate`
  - `/coord/leader_estimate_status`
  - `/coord/leader_estimate_error`
  - `/coord/leader_selected_target`
  - `/coord/leader_selected_target_filtered`
  - `/coord/leader_selected_target_filtered_status`
  - `/coord/leader_visual_target_estimate`
  - `/coord/leader_visual_target_estimate_status`
  - `/coord/leader_visual_control`
  - `/coord/leader_visual_control_status`
- `record_profile:=vision` also captures:
  - `/<uav>/camera0/image_raw`
  - `/<uav>/camera0/camera_info`
  - `/coord/leader_debug_image`
  - `/coord/leader_visual_control_debug_image`
- `record_profile:=step2_light` records only the compact Step 2/3/estimation-upgrade proof topics:
  - `/clock`
  - `/coord/events`
  - `/coord/leader_detection_status`
  - `/coord/leader_selected_target`
  - `/coord/leader_selected_target_filtered`
  - `/coord/leader_selected_target_filtered_status`
  - `/coord/leader_visual_target_estimate`
  - `/coord/leader_visual_target_estimate_status`
  - `/coord/leader_follow_point`
  - `/coord/leader_follow_point_status`
  - `/coord/leader_planned_target`
  - `/coord/leader_planned_target_status`
  - `/coord/leader_visual_control`
  - `/coord/leader_visual_control_status`
  - `/coord/leader_visual_actuation_bridge_status`
  - `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
  - `/<uav>/pose_cmd`
  - `/<uav>/pose`

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
    - `ugv_amcl_to_platform_odom` (`pose_cov_to_odom`)
    - `ugv_amcl_to_platform_filtered_odom` (`pose_cov_to_odom`)
    - `ugv_platform_odom_to_tf` (`odom_to_tf`)
    - `follow_uav_odom` or `follow_uav`
    - `camera_tracker`
    - optional `visual_follow_controller`
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
  - `follow_point_generator.py`
  - `visual_follow_controller.py`
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
- `src/lrs_halmstad/lrs_halmstad/sim/odom_to_tf.py`
- `src/lrs_halmstad/lrs_halmstad/follow/follow_uav_odom.py`
- `src/lrs_halmstad/lrs_halmstad/nav/ugv_nav2_driver.py`
- `src/lrs_halmstad/lrs_halmstad/dataset/sim_dataset_capture.py`

Current Nav2 fallback for sim startup:
- the active follow path now publishes fallback `/<ugv>/platform/odom` and `/<ugv>/platform/odom/filtered` from `/<ugv>/amcl_pose`
- the active follow path now also publishes fallback `odom -> base_link` TF from that AMCL-derived filtered odom stream
- this keeps Nav2 local costmaps and the UGV motion driver alive even when the Clearpath Gazebo model TF bridge is late or absent

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
  - `startup_reposition_enable:=true`
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
- visual-follow test controller in parallel with the current follow stack:
  - `./run.sh 1to1_yolo warehouse start_visual_follow_controller:=true`
  - `./run.sh tmux_1to1 warehouse gui:=false mode:=yolo start_visual_follow_controller:=true`
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
  - publishes `/coord/leader_selected_target`
- `selected_target_filter`
  - subscribes to `/coord/leader_selected_target`
  - publishes `/coord/leader_selected_target_filtered`
  - publishes `/coord/leader_selected_target_filtered_status`
- `follow_uav`
  - subscribes to `/coord/leader_estimate`
  - may also use shared AMCL heading if enabled
  - can publish legacy `/<uav>/pose_cmd*` helper topics when enabled
  - can publish `/<uav>/follow/{target,actual,error,debug}/*` topics when enabled

Visual-follow test path:
- `selected_target_filter`
  - subscribes to `/coord/leader_selected_target`
  - publishes `/coord/leader_selected_target_filtered`
  - publishes `/coord/leader_selected_target_filtered_status`
- `visual_follow_controller`
  - subscribes to `/coord/leader_selected_target_filtered`
  - subscribes to `/<uav>/camera0/camera_info`
  - optionally subscribes to `/<uav>/camera0/image_raw` for controller overlay
  - publishes `/coord/leader_visual_control`
  - publishes `/coord/leader_visual_control_status`
  - publishes `/coord/leader_visual_control_debug_image`

Recorder coverage for Step 2 validation:
- `./run.sh tmux_1to1 warehouse mode:=yolo record:=true`
  - captures the raw selected target, filtered selected target, upgraded visual target estimate, follow-point outputs, and visual-control outputs
- `record_profile:=step2_light`
  - smallest useful Step 2/3/estimation-upgrade/follow-point proof bag; omits camera/debug image streams and general follow telemetry
- `record_profile:=vision`
  - adds the camera and debug-image streams needed for post-run proof

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

Optional Step 2 test path:
- `leader_estimator`
  - also publishes clean selected-target state on `/coord/leader_selected_target`
- `selected_target_filter`
  - consumes `/coord/leader_selected_target`
  - publishes `/coord/leader_selected_target_filtered`
  - publishes `/coord/leader_selected_target_filtered_status`
  - applies trust, continuity, short-loss hold, and lightweight smoothing before control
- `visual_target_estimator`
  - consumes `/coord/leader_selected_target_filtered`
  - consumes `/<uav>/camera0/camera_info`
  - publishes `/coord/leader_visual_target_estimate`
  - publishes `/coord/leader_visual_target_estimate_status`
  - lifts the filtered image-space target into a camera-relative state and estimates short-horizon target motion
- `visual_follow_controller`
  - consumes `/coord/leader_visual_target_estimate`
  - keeps `/coord/leader_selected_target_filtered` for debug/fallback only
  - optionally uses `/<uav>/camera0/image_raw` for controller debug overlay
  - uses the upgraded camera-relative estimate as the primary control input
  - only falls back to direct filtered-target geometry if the upgraded estimate is missing/stale
  - publishes structured test commands on `/coord/leader_visual_control`
  - publishes readable status on `/coord/leader_visual_control_status`
  - publishes controller overlay on `/coord/leader_visual_control_debug_image`

Follow-point generation path:
- `follow_point_generator`
  - consumes `/coord/leader_visual_target_estimate`
  - consumes `/coord/leader_estimate`
  - consumes `/<uav>/pose`
  - consumes `/<uav>/camera/actual/center_pose`
  - publishes `/coord/leader_follow_point`
  - publishes `/coord/leader_follow_point_status`
  - turns the estimator-backed relative target state into a bounded world-frame spatial follow goal
  - prefers the estimator's world-frame target pose and yaw as the primary behind-target anchor so the UAV shadows the UGV from behind instead of orbiting the camera projection
  - falls back to motion-heading, held heading, and then the current view line only when the target-pose anchor is not fresh
  - briefly holds the last valid follow point across short estimate gaps

Planner path:
- `follow_point_planner`
  - consumes `/coord/leader_follow_point`
  - consumes `/<uav>/pose`
  - publishes `/coord/leader_planned_target`
  - publishes `/coord/leader_planned_target_status`
  - turns the raw follow point into a bounded planned pose target
  - moves only partway toward each new follow point, clamps target jumps, and briefly holds the last planned target on short gaps

Visual-follow bridge path:
- `visual_actuation_bridge`
  - subscribes to `/coord/leader_visual_control`
  - subscribes to `/coord/leader_follow_point`
  - subscribes to `/coord/leader_planned_target`
  - subscribes to `/<uav>/pose`
  - converts direct visual control, the raw follow point, or the planned target into the existing pose-like ENU command interface
  - publishes `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
  - publishes `/<uav>/pose_cmd`
  - publishes `/coord/leader_visual_actuation_bridge_status`
  - is the active motion owner only when `start_visual_actuation_bridge:=true`
- `follow_uav` / `follow_uav_odom`
  - remain the default motion owners when bridge mode is disabled
  - does not command the UAV directly

Current follow-point visual-follow chain:
- `/coord/leader_selected_target`
  -> `/coord/leader_selected_target_filtered`
  -> `/coord/leader_visual_target_estimate`
  -> `/coord/leader_follow_point`
  -> `/coord/leader_planned_target`
  -> `/coord/leader_visual_actuation_bridge_status` + existing UAV command path
- this is the current modular baseline before later harder-case validation and runtime hardening work
- `visual_follow_controller` still exists as the direct reactive lower-level path, but the planner-backed follow-point path is now the main structured motion-objective stage
- the bridge now reports which upstream input it is actively using via `active_input=control|follow_point|planned_target`

Current tuned bridge baseline:
- first tuning/validation should use `start_visual_actuation_bridge:=true` together with `use_estimate:=false`
- that stable mode isolates controller/filter/bridge motion quality before estimate-driven camera behavior is tuned further
- current controller defaults were tightened to reduce area-mode forward aggressiveness, allow smaller yaw corrections through, and decay commands faster on short loss / no-distance cases
- current bridge validation shows meaningful closed-loop improvement in that stable mode
- `leader_estimator` now also consumes live camera tilt/yaw topics when they are available:
  - `/<uav>/follow/actual/tilt_deg`
  - `/<uav>/camera/actual/world_yaw_rad`
- projected-range behavior is now validated in stable bridged mode:
  - recent bridge proof runs entered `range_src=ground`
  - the visual target estimator produced finite camera-relative range
  - the controller consumed that upgraded estimate during bridge mode
  - direct `range <-> area` flapping was not observed in the proof bag
- projected-range behavior is still not fully characterized for harder estimate-driven runs; that remains a later follow-quality / estimation task
- the estimator-backed phase is now minimally validated in the same stable bridged mode:
  - recent proof bags showed the controller primarily in `distance_mode=estimate`
  - fallback to direct filtered-target geometry was rare
  - the bridge remained the active motion owner and commands stayed bounded
- the planner phase is now minimally validated in the same stable bridged mode:
  - `follow_point_generator` stayed active upstream
  - `follow_point_planner` stayed active in the motion path
  - the bridge consumed `active_input=planned_target` for the meaningful motion portion of the proof run
  - raw follow-point updates averaged about `1.51 m` per step while planned targets averaged about `0.36 m` per step and were clamped to `0.40 m`
  - `/coord/leader_planned_target` and `/<uav>/pose_cmd` were published consistently
  - UAV motion remained bounded and recoverable while following the planned target
- the first harder estimate-driven planner probe is now also characterized:
  - command used `use_estimate:=true use_actual_heading:=false` with the planner path still enabled
  - a planner hold bug was fixed so short-loss `HOLD` no longer refreshes itself forever
  - after that fix, planner state now degrades cleanly from `ACTIVE` to bounded `HOLD` and then `INVALID`
  - the remaining weakness in that harder mode was upstream visual acquisition, not planner ownership:
    - detector status was dominated by `NO_DET`
    - filtered target and estimator were mostly `INVALID`
    - the bridge spent most of the run in `HOLD` because no fresh planned target was available
- the first runtime-hardening pass on that harder mode is now also validated:
  - `selected_target_filter` now allows bounded low-confidence reacquire from very recent plausible history
  - `camera_tracker` now keeps the last trackable leader pose alive briefly so pan does not freeze instantly on short estimator dropouts
  - `camera_tracker` now also falls back to an odom-frame UGV pose for camera-only reacquisition after longer estimate loss, which is meant for sim/testing visibility recovery rather than motion ownership
  - recent defaults now give the filter/estimator slightly longer short-gap prediction windows
  - harder proof command:
    - `./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=true obb:=true weights:=warehouse-v1-yolo26n-obb.pt use_estimate:=true use_actual_heading:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true record:=true record_profile:=step2_light record_tag:=robustness_hardening3 delay_s:=12`
  - compared with the earlier harder planner bag:
    - filtered target `VALID`: `64 -> 92`
    - visual-target estimate `MEASURED+PREDICTED`: `73 -> 110`
    - follow-point `ACTIVE+HOLD`: `91 -> 130`
    - planner `ACTIVE+HOLD`: `126 -> 167`
    - bridge `ACTIVE`: `123 -> 167`
    - bridge `active_input=planned_target`: `143 -> 180`
    - bridge `active_input=control`: `378 -> 193`
  - the planner/bridge path now stays active materially longer in the harder estimate-driven regime instead of collapsing as quickly to stale-control hold
- the latest shadow-follow hardening pass is now also validated for the same harder mode:
  - `follow_point_generator` now preserves a recent behind-target heading briefly when the target slows or weakens, instead of flipping to the wrong side of the UGV from a pure view-line fallback
  - follow-point, planner, and bridge defaults are now more conservative so the UAV shadows the UGV more slowly instead of lunging toward large point changes
  - bridge `input_mode=auto` is now safe by design:
    - it consumes `planned_target`, then `follow_point`
    - otherwise it holds position
    - it no longer falls back silently to raw `visual_control`
  - latest harder proof command:
    - `./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=true obb:=true weights:=warehouse-v1-yolo26n-obb.pt camera:=detached use_estimate:=true use_actual_heading:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true record:=true record_profile:=step2_light record_tag:=shadow_follow_fix3 delay_s:=12`
  - compared with the earlier harder hardening bag:
    - bridge `active_input=control`: `193 -> 0`
    - bridge `active_input=planned_target`: `180 -> 126`
    - bridge mean `step_xy_m`: `0.1097 -> 0.0649`
    - UAV XY path length over the proof window: `41.26 m -> 15.11 m`
  - this does not mean the no-odom-style regime is fully solved, but it does mean the planner-backed path now fails safer when visual continuity degrades instead of dropping into an unsafe lower-level chase mode
- the stable planner-backed path needed two runtime fixes after the shadow-follow hardening:
  - `uav_simulator` now keeps publishing `/<uav>/pose` even if detached-camera `SetEntityPose` futures are still pending, and it clears stuck futures after a short timeout instead of starving the pose topic
  - `leader_estimator` now uses external-detection receive time for freshness gating while keeping the original detection timestamp for latency reporting
  - with those fixes, the stable command `use_estimate:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true` again produces:
    - fresh UAV pose
    - active follow-point generation
    - active planned target output
    - active bridge output
    - visible UAV motion in the planner-backed path
- the latest stable shadow-follow refinement now anchors the follow point more explicitly behind the estimated UGV pose:
  - `follow_point_generator` prefers `/coord/leader_estimate` XY and yaw as the primary behind-target reference
  - when that target-pose anchor is fresh, status reports `policy_mode=target_pose_heading`
  - planner and bridge still stay on the same planned-target path above it
  - short-loss `HOLD` / `INVALID` can still happen if the upstream target estimate drops out, so this change tightens the behind-target policy without claiming the upstream hard case is solved
- the remaining weakness after this hardening pass is now more clearly genuine visual visibility / geometry loss:
  - close-range and under-UAV scenes can still drive the detector into real `NO_DET`
  - those segments still cascade into filtered-target / estimate invalid states even though short-loss behavior is better than before
- the next major focus after this is continued harder estimate-driven validation and runtime hardening, not more bridge/planner architecture work

### Current Active Debug Target

Current live focus in YOLO detect/track follow:
- estimator/follow behavior is much better than earlier, but close-range behavior is still the main problem
- the UAV can struggle when the UGV passes underneath or tries to cross the UAV path
- the estimator can still end up holding the fallback/const range at the wrong moment
- camera/body centering and target-distance recovery are the active runtime tuning/debug areas

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
- `/coord/leader_selected_target`
- `/coord/leader_selected_target_filtered`
- `/coord/leader_selected_target_filtered_status`
- `/coord/leader_visual_target_estimate`
- `/coord/leader_visual_target_estimate_status`
- `/coord/leader_follow_point`
- `/coord/leader_follow_point_status`
- `/coord/leader_planned_target`
- `/coord/leader_planned_target_status`
- `/coord/leader_debug_image`
- `/coord/leader_visual_control`
- `/coord/leader_visual_control_status`
- `/coord/leader_visual_actuation_bridge_status`
- `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- `/<uav>/pose_cmd`
- `/<uav>/update_pan`
- `/<uav>/update_tilt`

Important current assumption:
- this does not look like a missing detector/tracker wiring problem anymore
- it looks more like a runtime interaction between estimate state, body yaw/XY commands, and camera pan/tilt commands

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
- `follow_yaw_rate_gain: 3.0`
- `yaw_deadband_rad: 0.02`
- `yaw_update_xy_gate_m: 0.0`
- `leader_actual_heading_enable: false` in YAML
- `estimate_heading_from_motion_enable: true`
- `traj_max_speed_mps: 5.0`
- `traj_max_accel_mps2: 8.0`

Camera tracker:
- `tick_hz: 20.0`
- `default_pan_deg: 0.0`
- `default_tilt_deg: -45.0`
- `pan_enable: true`
- `tilt_enable: true`
- `image_center_correction_enable: true`
- `pan_image_center_gain: 0.5`
- `tilt_image_center_gain: 0.8`

Estimator:
- `est_hz: 20.0`
- `range_mode: auto`
- `constant_range_m: 7.0`
- `use_depth_range: true`
- `external_detection_timeout_s: 1.0`

Detector:
- `predict_hz: 20.0`
- `conf_threshold: 0.3`
- `bbox_continuity_weight: 0.10`
- `bbox_continuity_max_px: 180.0`

Tracker:
- `predict_hz: 20.0`
- `tracker_config: bytetrack.yaml`
- `conf_threshold: 0.12`
- `iou_threshold: 0.45`

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

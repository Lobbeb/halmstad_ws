## Current State

Assumption:
- start in the workspace root

### Active Baseline

Recommended start/stop flow:
1. `./run_tmux_1to1.sh warehouse`
2. `./stop_tmux_1to1.sh warehouse`

Current tmux default:
- `layout:=panes`
- row 1: `gazebo | spawn`
- row 2: `localization | nav2`
- row 3: `follow`

Current tmux default delays:
- `gui:=false` -> `spawn=9s`, `localization/nav2=11s`, `follow=13s`
- `gui:=true` -> `spawn=7s`, `localization/nav2=9s`, `follow=11s`

Equivalent manual run order:
1. `./run_gazebo_sim.sh warehouse`
2. `./run_spawn_uav.sh warehouse`
3. `./run_localization.sh warehouse`
4. `./run_nav2.sh`
5. `./run_1to1_follow.sh warehouse`

YOLO variant:
- `./run_1to1_yolo.sh warehouse`
- default runtime behavior: estimate-only follow/camera path with no UGV truth shared into the controller
- estimate-mode YOLO now keeps the normal startup pose instead of forcing a farther reposition
- estimate-mode YOLO still computes `/coord/leader_estimate_error` against `/<ugv>/amcl_pose_odom` by default
- `./run_follow_control.sh` works against the same `/follow_uav` node in both odom and estimate modes
- testing override: `./run_1to1_yolo.sh warehouse use_estimate:=false`

### What Works Now

The important follow fix is in place:
- the follow stack still runs with `leader_mode:=odom`
- but the actual leader topic is now `/<ugv>/amcl_pose_odom`
- `/<ugv>/amcl_pose_odom` is synthesized from `/<ugv>/amcl_pose`
- this is the source of truth going forward

Important interpretation:
- `leader_mode:=odom` now means "consume an `Odometry`-shaped leader topic"
- it does **not** mean "use raw `/platform/odom`"
- do **not** switch follow or dataset capture back to `/platform/odom` or `/platform/odom/filtered`
- the YOLO path now defaults to `leader_mode:=estimate`; pass `use_estimate:=false` on `run_1to1_yolo.sh` to test the same stack with shared UGV pose
- estimator held-estimate reuse is now disabled by default, so reject/miss states surface directly instead of reusing the last target for `*_HOLD`

Relevant files:
- `src/lrs_halmstad/launch/run_follow_motion.launch.py`
- `src/lrs_halmstad/lrs_halmstad/pose_cov_to_odom.py`
- `src/lrs_halmstad/lrs_halmstad/follow_uav_odom.py`
- `src/lrs_halmstad/lrs_halmstad/camera_tracker.py`
- `src/lrs_halmstad/lrs_halmstad/sim_dataset_capture.py`

### Run Script Map

`run_tmux_1to1.sh`
- recommended top-level launcher
- starts Gazebo, spawn, localization, Nav2, and either follow or YOLO-follow in tmux
- defaults to pane layout
- accepts common follow/spawn overrides like `camera:=detached`, `camera:=attached`, `height:=...`, `mount_pitch_deg:=...`, `uav_name:=...`
- accepts `mode:=follow|yolo` and `yolo:=true|false`; `mode:=yolo` switches the last pane/window to `run_1to1_yolo.sh`
- accepts `record:=true|false` to start an experiment rosbag recorder in the same tmux session

`run_record_experiment.sh`
- lightweight rosbag wrapper for experiments
- writes `runs/experiments/<world>/<timestamp>_<mode>[_tag]/`
- stores `metadata.json`, `topics.txt`, and the bag under `bag/`
- default profile is intentionally small and excludes image topics; `profile:=vision` opts into image recording

`stop_tmux_1to1.sh`
- recommended stop path
- sends `Ctrl-C` to `follow`, `localization`, and `nav2` together
- waits `5s`
- then stops `spawn`
- then stops Gazebo
- then kills the tmux session and does fallback cleanup for leftover ROS/Gazebo processes and stale `/tmp/halmstad_ws` state files

`run_gazebo_sim.sh`
- builds the workspace, sources ROS, and launches `managed_clearpath_sim.launch.py`
- writes `/tmp/halmstad_ws/gazebo_sim.pid` and `/tmp/halmstad_ws/gazebo_sim.world`
- default GUI is `true` unless overridden

`run_spawn_uav.sh`
- spawns the UAV for the current world
- if Gazebo is already running, it auto-detects the world from `/tmp/halmstad_ws/gazebo_sim.world`
- default camera path is detached because the underlying launch default is `uav_camera_mode:=detached_model`
- if Gazebo exits, this wrapper shuts its launch down too

`run_localization.sh`
- launches localization against the chosen map
- for `warehouse`, it defaults to Clearpath's warehouse map

`run_nav2.sh`
- launches the Nav2 stack for the UGV

`run_1to1_follow.sh`
- launches `run_1to1_follow.launch.py`
- uses `ugv_mode:=nav2`
- uses `leader_mode:=odom`
- important: the downstream launch default for that odom topic is `/<ugv>/amcl_pose_odom`, not raw platform odom
- camera overrides like `camera:=detached` and `camera:=attached` map to `uav_camera_mode:=...`

`run_capture_dataset.sh`
- launches `sim_dataset_capture`
- explicitly sets `target_pose_topic:=/a201_0000/amcl_pose_odom`
- this wrapper is already aligned with the AMCL-based follow fix

`run_follow_control.sh`
- runtime helper for live follow tuning
- default mode is keyboard control of `d_target` / `z_alt`
- `--mode params` is kept as an alias for the same keyboard tuning path
- random sweep mode is still available with `--mode random`

`run_record_pose_alignment.sh` / `run_analyze_pose_alignment.py`
- one-off diagnostics for comparing raw odom against AMCL
- useful for analysis, not part of the normal follow launch path

### Launch Call Chain

High-level wrapper chain:
- `run_tmux_1to1.sh`
  - runs `run_gazebo_sim.sh`
  - runs `run_spawn_uav.sh`
  - runs `run_localization.sh`
  - runs `run_nav2.sh`
  - runs `run_1to1_follow.sh` or `run_1to1_yolo.sh` depending on `mode:=follow|yolo`

Follow launch chain:
- `run_1to1_follow.sh`
  - launches `src/lrs_halmstad/launch/run_1to1_follow.launch.py`
- `run_1to1_follow.launch.py`
  - mostly acts as a world-specific wrapper
  - forwards arguments into `src/lrs_halmstad/launch/run_follow_motion.launch.py`
- `run_follow_motion.launch.py`
  - starts the actual runtime nodes:
    - `uav_simulator`
    - `ugv_amcl_to_odom` (`pose_cov_to_odom`)
    - `follow_uav` (`follow_uav_odom` when `leader_mode:=odom`)
    - `camera_tracker`
    - `ugv_nav2_driver`
    - optionally `leader_estimator` for non-odom / perception flows

Spawn chain:
- `run_spawn_uav.sh`
  - launches `src/lrs_halmstad/launch/spawn_uav_1to1.launch.py`
- `spawn_uav_1to1.launch.py`
  - always spawns the UAV body through `spawn_robot.launch.py`
  - if `uav_camera_mode:=detached_model`, also spawns the detached camera/gimbal through `spawn_gimbal.launch.py`
  - starts the Gazebo `set_pose` bridge
  - starts the detached camera image/info bridge after a short delay

Important separation:
- `run_spawn_uav.sh` creates the visible Gazebo entities
- `run_1to1_follow.sh` starts the logic that moves the UAV and points the camera during the run
- the UAV does **not** actually "follow" anything until the follow launch is running

### Follow Data Flow

UGV side:
- `run_localization.sh` starts AMCL and publishes `/<ugv>/amcl_pose`
- `ugv_amcl_to_odom` converts `/<ugv>/amcl_pose` into `/<ugv>/amcl_pose_odom`
- `ugv_nav2_driver` sends NavigateToPose goals and keeps the UGV moving along the configured route

UAV follow side:
- `follow_uav_odom`
  - subscribes to `/<ugv>/amcl_pose_odom`
  - subscribes to `/<uav>/pose`
  - computes the next desired UAV pose from leader position, `d_target`, `z_alt`, yaw logic, and current limits
  - publishes `/<uav>/pose_cmd` and `/<uav>/pose_cmd/odom`

Camera side:
- `camera_tracker`
  - subscribes to the same AMCL-derived leader topic
  - subscribes to `/<uav>/pose`
  - also listens to `/<uav>/pose_cmd` so pan/tilt can react to commanded motion immediately
  - computes the desired look-at point and publishes:
    - `/<uav>/update_tilt`
    - `/<uav>/update_pan`
    - `/<uav>/camera/target/*`

Simulator side:
- `uav_simulator`
  - subscribes to `/<uav>/pose_cmd`
  - subscribes to `/<uav>/update_tilt` and `/<uav>/update_pan`
  - updates the UAV body pose in Gazebo
  - if camera mode is detached, also updates the detached camera model pose in Gazebo
  - publishes `/<uav>/pose`, `/<uav>/camera/actual/*`, and follow/camera debug topics

Key takeaway:
- the AMCL topic drives both UAV following and camera targeting
- raw `/platform/odom` is no longer part of the validated follow path
- if the UAV seems numerically correct but visually wrong, check whether the leader source is still `/<ugv>/amcl_pose_odom`

### Camera Baseline

Current validated baseline:
- detached camera model
- `uav_camera_mode:=detached_model`
- `pan_enable: true`
- `tilt_enable: true`
- `default_pan_deg: 0.0`
- `default_tilt_deg: -45.0`
- `run_1to1_yolo.sh` uses the same `-45.0` fallback tilt baseline, but in estimate mode it seeds/repositions the UAV farther back before the first estimate arrives

Current shared look-target offsets:
- `leader_look_target_x_m: 0.0`
- `leader_look_target_y_m: 0.0`
- `camera_look_target_z_m: 0.3`

Important consequence:
- `camera:=attached` is now an override path, not the baseline
- detached is the default because it visually tracks pan/tilt correctly in Gazebo

### Clock / Sim Time

Current Gazebo sim launch uses the guarded clock path:
- Gazebo `/clock` -> ROS `/clock_raw`
- `clock_guard.py` republishes monotonic ROS `/clock`

Runtime check:
- `ros2 topic info /clock -v`
- expected publisher count: `1`
- expected publisher: `clock_guard`

### Current Tuning Defaults

From `src/lrs_halmstad/config/run_follow_defaults.yaml`:
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
- `smooth_alpha: 1.0`

Camera tracker defaults:
- `tick_hz: 20.0`
- `default_pan_deg: 0.0`
- `default_tilt_deg: -45.0`
- `pan_enable: true`
- `tilt_enable: true`
- `tilt_deadband_deg: 0.5`

### Dataset State

Old dataset warning:
- older captures made against raw `/platform/odom` are suspect
- pose-alignment analysis showed the odom -> AMCL correction is meaningfully time-varying
- that means the old raw-odom captures are not safely fixable with one static 2D transform

Current rule:
- new dataset capture must use `/<ugv>/amcl_pose_odom`
- `run_capture_dataset.sh` already enforces this

Current dataset note:
- `datasets/warehouse_v1/run1` was manually QA-pruned after removing bad overlays
- corresponding `images`, `labels`, and `metadata` were deleted too, so that run is internally consistent again

### Debug / Verification

Useful runtime checks:
- `/dji0/follow/target/*`
- `/dji0/follow/actual/*`
- `/dji0/follow/error/*`
- `/dji0/camera/target/*`
- `/dji0/camera/actual/*`

Pose-source validation:
- compare `/a201_0000/platform/odom`, `/a201_0000/platform/odom/filtered`, and `/a201_0000/amcl_pose`
- if the image drifts while the debug numbers look self-consistent, validate the pose source before retuning yaw or camera math

### Guardrails

Things that should not be "fixed back":
- do not switch follow back to raw `/platform/odom`
- do not switch dataset capture back to raw `/platform/odom`
- do not remove `clock_guard` unless the backward-jump problem is proven fixed
- do not treat attached camera mode as the default baseline

### Build / Validation State

Recent validation status:
- `colcon build --packages-select lrs_halmstad` passed for the core package during the follow-camera work
- recent wrapper/script changes were checked with `bash -n` and `python3 -m py_compile`

If a new chat continues from here, start by reading:
- this file
- `RUNNING_SIM.md`



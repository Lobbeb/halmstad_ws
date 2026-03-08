## Current State

Assumption:
- start in `~/halmstad_ws`

### Active Run Sequence
Use these wrappers:

1. `./run_gazebo_sim.sh warehouse`
2. `./run_spawn_uav.sh warehouse uav_name:=dji0`
3. `./run_localization.sh warehouse`
4. `./run_nav2.sh`
5. `./run_1to1_follow.sh`

YOLO variant:
- `./run_1to1_yolo.sh`

Notes:
- `./run_1to1_follow.sh` defaults to `leader_mode:=odom`
- `./run_1to1_yolo.sh` defaults to `leader_mode:=estimate`
- `./run_spawn_uav.sh` now defaults to attached/integrated camera mode
- wrapper scripts source ROS/workspace automatically

### Current Architecture

#### Odom Follow
- `leader_mode:=odom` uses:
  - [follow_uav_odom.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow_uav_odom.py)

Responsibilities:
- reads UGV odom from `/a201_0000/platform/odom/...`
- reads actual UAV pose from `/<uav>/pose`
- computes anchor behind the UGV
- publishes UAV command on:
  - `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`

#### Estimate/YOLO Follow
- `leader_mode:=estimate` / pose-like path uses:
  - [follow_uav.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow_uav.py)

This file was already cleaned somewhat and is now estimate/pose-oriented.

#### Camera Control
- camera control is separate:
  - [camera_tracker.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/camera_tracker.py)

Current behavior:
- `pan` is disabled by default
- `tilt` is enabled by default
- default pan/tilt are still published at startup
- in integrated mode, the startup command is `default_tilt_deg: 0.0` and the attached camera mount provides the initial `-45 deg` downward look
- in integrated mode, the attached gimbal pitch command is relative to the spawned camera mount angle

#### Shared Helpers
- math helpers:
  - [follow_math.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow_math.py)
- grouped debug publishers:
  - [follow_debug.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow_debug.py)

#### Simulator
- [simulator.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/simulator.py)

Responsibilities:
- subscribes to UAV command topic
- moves UAV in Gazebo
- applies camera pan/tilt for detached camera mode
- publishes grouped camera actual/error topics

### Current Odom Tuning Model

Odom path is being simplified around a clean separation:

#### XY
- proportional with cap
- current law:
  - `effective_follow_speed_mps = min(follow_speed_mps, follow_speed_gain * anchor_distance_error)`

Relevant params in [run_round_follow_defaults.yaml](/home/ruben/halmstad_ws/src/lrs_halmstad/config/run_round_follow_defaults.yaml):
- `follow_speed_mps`
- `follow_speed_gain`

#### Body Yaw
- proportional with cap
- current law:
  - `effective_yaw_rate_rad_s = min(follow_yaw_rate_rad_s, follow_yaw_rate_gain * abs(yaw_error))`

Relevant params:
- `follow_yaw_rate_rad_s`
- `follow_yaw_rate_gain`

Current defaults:
- `follow_speed_mps: 5.0`
- `follow_speed_gain: 2.0`
- `follow_yaw_rate_rad_s: 3.0`
- `follow_yaw_rate_gain: 2.0`

### Debug Topics To Monitor

Grouped follow topics:
- `/<uav>/follow/target/*`
- `/<uav>/follow/actual/*`
- `/<uav>/follow/error/*`

Grouped camera topics:
- `/<uav>/camera/target/*`
- `/<uav>/camera/actual/*`
- `/<uav>/camera/error/*`

Important examples for `dji0`:
- `/dji0/follow/error/anchor_distance_m`
- `/dji0/follow/error/anchor_along_m`
- `/dji0/follow/error/anchor_cross_m`
- `/dji0/follow/error/yaw_rad`
- `/dji0/follow/debug/yaw_target_raw_rad`
- `/dji0/follow/debug/yaw_target_unwrapped_rad`
- `/dji0/follow/debug/yaw_actual_unwrapped_rad`
- `/dji0/follow/debug/yaw_error_raw_rad`
- `/dji0/follow/debug/yaw_wrap_correction_rad`
- `/dji0/follow/debug/yaw_wrap_active`
- `/dji0/follow/debug/yaw_step_limit_rad`
- `/dji0/follow/debug/yaw_cmd_delta_rad`
- `/dji0/follow/debug/yaw_mode`
- `/dji0/camera/target/look_at_point`
- `/dji0/camera/target/world_yaw_rad`
- `/dji0/camera/actual/world_yaw_rad`
- `/dji0/camera/error/world_yaw_rad`

Legacy/noisy topics were muted by default:
- `/coord/events`
- `/coord/follow_dist_cmd`
- `/coord/follow_tracking_error_cmd`
- `/dji0/debug_camera_pose`
- `/dji0/debug_yaw`

### Spawn / Camera Model Work

Detached camera sensor-frame offsets were exposed through spawn:
- `sensor_roll_deg:=...`
- `sensor_pitch_deg:=...`
- `sensor_yaw_deg:=...`

Path:
- [run_spawn_uav.sh](/home/ruben/halmstad_ws/run_spawn_uav.sh)
- [spawn_uav_1to1.launch.py](/home/ruben/halmstad_ws/src/lrs_halmstad/launch/spawn_uav_1to1.launch.py)
- [spawn_gimbal.launch.py](/home/ruben/halmstad_ws/src/lrs_halmstad/launch/spawn_gimbal.launch.py)
- [generate_sdf.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/generate_sdf.py)
- [lrs_camera_gimbal.sdf.xacro](/home/ruben/halmstad_ws/src/lrs_halmstad/xacro/lrs_camera_gimbal.sdf.xacro)

The typing bug in `generate_sdf.py` for `camera_sensor_*_deg` was already fixed:
- integer or float launch args now work

### Removed / Simplified

Removed from active follow architecture:
- scripted UGV backend from active 1-to-1/follow path
- `run_round_motion.sh`
- `run_round_motion.launch.py`
- `run_sweep_motion.launch.py`
- `run_sweep_motion.yaml`

Current UGV modes:
- `nav2`
- `external`

### Main Remaining Problem

The remaining issue is:
- after sharp turns, the UAV body yaw still seems to overshoot / drift
- visually, the UAV can stop centering the UGV after turning

Important findings already established:
- some earlier “good” debug values were from self-consistent commanded camera state, not independent Gazebo sensor truth
- `look_at_point` matched UGV odom
- camera target/actual world yaw often matched
- image could still look wrong
- pan/body-yaw coupling was suspected, so pan was disabled again by default

Current hypothesis:
- the remaining problem is primarily in the body-yaw control path of `follow_uav_odom.py`
- not in the basic UGV target point

### Recommended Next Step

Continue debugging only the odom path:
- [follow_uav_odom.py](/home/ruben/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow_uav_odom.py)

Do not add more control layers until body yaw is proven stable.

Suggested next checks:
1. verify `pan_enable:=false` and `tilt_enable:=true` at runtime
2. monitor:
   - `/dji0/follow/error/yaw_rad`
   - `/dji0/follow/error/anchor_cross_m`
   - `/dji0/camera/error/world_yaw_rad`
3. if overshoot remains, tune only:
   - `follow_yaw_rate_rad_s`
   - `follow_yaw_rate_gain`

### Build State

Recent builds passed after the latest changes:
- `colcon build --packages-select lrs_halmstad`

If a new chat continues from here, start by reading:
- this file
- [RUNNING_SIM.md](/home/ruben/halmstad_ws/RUNNING_SIM.md)

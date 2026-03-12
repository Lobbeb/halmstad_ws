# Running Sim

Default tested path: `warehouse`.

Assumption:
- you already start in the workspace root
- wrapper files now live under `scripts/`
- use `./run.sh <name>` and `./stop.sh <name>` from the workspace root
- optional bash startup hook for completion:
  - `source "$HOME/halmstad_ws/scripts/bashrc_halmstad_ws.bash"`
- quiet YOLO defaults in this file refer to `./run.sh 1to1_yolo ...`
- a bare `ros2 launch lrs_halmstad run_follow.launch.py ...` still inherits the launch-argument defaults unless you override them explicitly

Recommended tmux workflow:
- start with `./run.sh tmux_1to1 warehouse`
- start YOLO mode with `./run.sh tmux_1to1 warehouse mode:=yolo`
- start YOLO tracker mode with `./run.sh tmux_1to1 warehouse mode:=yolo tracker:=true`
- record a YOLO run with `./run.sh tmux_1to1 warehouse mode:=yolo record:=true`
- record a YOLO vision run with `./run.sh tmux_1to1 warehouse mode:=yolo record:=true record_profile:=vision`
- speed up sim time with `./run.sh tmux_1to1 warehouse rtf:=2.0`
- pass a custom YOLO weight with `./run.sh tmux_1to1 warehouse mode:=yolo weights:=yolo26v2.pt`
- enable truth-assisted YOLO testing with `./run.sh tmux_1to1 warehouse mode:=yolo use_estimate:=false`
- stop with `./stop.sh tmux_1to1 warehouse`
- default tmux layout is panes:
  - row 1: `gazebo | spawn`
  - row 2: `localization | nav2`
  - row 3: `follow`
- current default delays:
  - `gui:=false` -> `spawn=8`, `localization/nav2=10`, `follow=12`
  - `gui:=true` -> `spawn=6`, `localization/nav2=8`, `follow=10`

## Start 1-to-1 Sim With Nav2 And AMCL Follow

Run these in order in separate terminals:

1. Start Gazebo:

```bash
./run.sh gazebo_sim warehouse
```

Without GUI:

```bash
./run.sh gazebo_sim warehouse false
```

2. Spawn one UAV:

```bash
./run.sh spawn_uav warehouse uav_name:=dji0
```

3. Start localization:

```bash
./run.sh localization warehouse
```

If you want a different saved map:

```bash
./run.sh localization warehouse maps/warehouse_manual.yaml
```

4. Start Nav2:

```bash
./run.sh nav2
```

5. Start the AMCL-based odom follow stack:

```bash
./run.sh 1to1_follow warehouse
```

#### If you want the same stack started automatically in tmux, use:

```bash
./run.sh tmux_1to1 warehouse
```

YOLO variant in tmux:

```bash
./run.sh tmux_1to1 warehouse mode:=yolo
```

Default tmux layout is panes:
- Gazebo
- UAV spawn
- localization
- Nav2
- follow

Useful overrides:

```bash
./run.sh tmux_1to1 warehouse gui:=false
./run.sh tmux_1to1 warehouse delay_s:=9
./run.sh tmux_1to1 warehouse spawn_delay_s:=12 follow_delay_s:=18
./run.sh tmux_1to1 warehouse layout:=windows
./run.sh tmux_1to1 warehouse mode:=yolo
./run.sh tmux_1to1 warehouse mode:=yolo use_actual_heading:=false
./run.sh tmux_1to1 warehouse mode:=yolo follow_yaw:=false use_tilt:=false
./run.sh tmux_1to1 warehouse mode:=yolo record:=true record_profile:=vision
./run.sh tmux_1to1 warehouse rtf:=2.0
./run.sh tmux_1to1 warehouse tmux_attach:=false
./run.sh tmux_1to1 warehouse dry_run:=true
```

Default startup is staggered with short delays so the later launches do not all fire at the same instant.

Default delays depend on `gui:=true|false`:
- `gui:=false` uses `spawn=8`, `localization/nav2=10`, and `follow=12`
- `gui:=true` uses `spawn=6`, `localization/nav2=8`, and `follow=10`

If your machine is slower, increase the delay args:
- `delay_s:=...`
- `spawn_delay_s:=...`
- `localization_delay_s:=...`
- `nav2_delay_s:=...`
- `follow_delay_s:=...`

If you want separate tmux windows instead of the default panes:

```bash
./run.sh tmux_1to1 warehouse layout:=windows
```

Alias:

```bash
./run.sh tmux_1to1 warehouse panes:=true
```

Pane layout is:
- row 1: Gazebo | spawn
- row 2: localization | Nav2
- row 3: follow

To stop the tmux-managed stack cleanly, use:

```bash
./stop.sh tmux_1to1 warehouse
```

This sends `Ctrl-c` in this order:
- follow, localization, and Nav2 together
- wait `5s`
- Gazebo
- spawn is expected to exit automatically when Gazebo goes down

Then it waits a few seconds, kills the tmux session, performs a safety cleanup pass for leftover Gazebo / launch processes, and clears stale helper state files under `/tmp/halmstad_ws`.
If recording was enabled, the stop flow also stops the recorder pane and cleans up matching `ros2 bag record` processes as fallback.

Useful stop overrides:

```bash
./stop.sh tmux_1to1 warehouse group_grace_s:=2
./stop.sh tmux_1to1 warehouse final_grace_s:=8
./stop.sh tmux_1to1 warehouse kill_session:=false
./stop.sh tmux_1to1 session:=halmstad-warehouse-1to1
```

Current baseline:
- this odom-follow path now uses `/<ugv>/amcl_pose_odom`, not raw `/platform/odom`
- `/<ugv>/amcl_pose_odom` is synthesized from `/<ugv>/amcl_pose` by [pose_cov_to_odom.py](src/lrs_halmstad/lrs_halmstad/sim/pose_cov_to_odom.py)
- launch `leader_odom_topic` / `ugv_odom_topic` defaults are intentionally pointed at that AMCL-derived topic
- current UAV camera mode is detached: `uav_camera_mode:=detached_model`
- spawned UAV camera is now RGBD-capable while keeping the existing RGB ROS topics
- additional UAV depth topic is available at `/<uav>/camera0/depth_image`
- current camera defaults are `pan_enable: true` and `tilt_enable: true`
- attached mode is still available as an override with `camera:=attached`
- if you override the mount pitch for attached mode, pass the same value to follow:

```bash
./run.sh spawn_uav warehouse uav_name:=dji0 camera:=attached mount_pitch_deg:=35
./run.sh 1to1_follow warehouse camera:=attached mount_pitch_deg:=35
```

To test the older attached camera path instead of the detached default:

```bash
./run.sh spawn_uav warehouse camera:=attached
./run.sh 1to1_follow warehouse camera:=attached
```

Important:
- pass the same camera mode to both spawn and follow
- detached mode does not rely on the attached `45 deg` mount pitch baseline
- the wrapper alias also accepts `camera:=attached`

Important runtime note:
- Gazebo sim time is guarded through [clock_guard.py](src/lrs_halmstad/lrs_halmstad/sim/clock_guard.py)
- expected `/clock` publisher is `clock_guard`
- if Gazebo is restarted or the world is reset, restart localization, Nav2, and follow

## Start The Camera Views

Saved multi-camera / debug layout:

```bash
./run.sh rqt_perspective
```

Then under `Perspectives`, choose one from the `perspectives/` folder if needed.

UAV camera: ``/dji0/camera0/image_raw``

UAV depth image: ``/dji0/camera0/depth_image``

UGV camera: ``/a201_0000/sensors/camera_0/color/image``

YOLO debug image: ``/coord/leader_debug_image``

Overlay fields:
- left block:
  - `est: (...)`
  - `actual: (...)`
  - `est_d: ...`
  - `est_xy: ...`
  - `real_d: ...`
  - `real_xy: ...`
  - `real_z: ...`
  - `range_src: ...`
  - `bearing: ...`
  - `est_heading: forward|reverse|left|right`
  - `heading_src: ...`
- right block:
  - `task: detection|obb`
  - `state: ...`
  - `reason: ...`
  - `conf: ...`
  - `src: detector|tracker|none`
  - `id: ...`
  - `hits: ...`
  - `age: ...s`
- bottom-left:
  - `latency_ms: ...`


Only use the YOLO debug image when you started the YOLO flow from the section below.

## Control Height And Distance During Runtime

These work while `./run.sh 1to1_follow` or `./run.sh 1to1_yolo` is running.

Recommended live control:
- change `d_target` during runtime
- this updates the derived `xy_target` from the current `z_min`
- no restart is needed for this control
- the direct `ros2` commands below assume the terminal is already sourced

Change the 3D UAV-to-UGV follow distance:

```bash
ros2 param set /follow_uav d_target x.xx
```

Examples:

```bash
ros2 param set /follow_uav d_target 9.0
ros2 param set /follow_uav xy_min 5.0
ros2 param set /follow_uav z_min 9.0
```

Use `xy_min`, `xy_max`, `z_min`, and `z_max` if you want to constrain the working range around the shared `d_target`.

Runtime follow control helper:

The moved helper wrappers are available through `./run.sh <name>` and still
exist under `./scripts/run_<name>.sh` if you want direct file completion.

```bash
./run.sh follow_control
```

Default keyboard mode:
- this is the live tuning path for `d_target` and `z_min`
- `w` raises `z_min`
- `s` lowers `z_min`
- `d` increases `d_target`
- `a` decreases `d_target`
- `r` refreshes the current values
- `h` shows the help text
- `q` quits

Random sweep mode:
- pass `--mode random`
- current defaults:
- updates every `10s`
- `z_min` sampled uniformly in `[2, 40]`
- `d_target` sampled uniformly in `[1, 20]`
- sampling is biased toward the `5-15` band by default
- runs until you stop it with `Ctrl-c`

Useful examples:

```bash
./run.sh follow_control
./run.sh follow_control --step-z-alt 2 --step-d-target 2
./run.sh follow_control --mode params --step-z-alt 2 --step-d-target 2
./run.sh follow_control --mode random --interval 8
./run.sh follow_control --mode random --count 20
./run.sh follow_control --mode random --seed 42
./run.sh follow_control --mode random --interval 8 --count 20 --seed 42
./run.sh follow_control --mode random --focus-weight 0.8
```

## Record A Run

Recommended recorder path through tmux:

```bash
./run.sh tmux_1to1 warehouse mode:=yolo record:=true
```

Add image topics too:

```bash
./run.sh tmux_1to1 warehouse mode:=yolo record:=true record_profile:=vision
```

Direct recorder wrapper:

```bash
./run.sh record_experiment warehouse mode:=yolo profile:=vision
```

Default recorder output:
- `bags/experiments/<world>/<timestamp>_<mode>/`

Recorder topic groups:
- default:
  - `/clock`
  - `/coord/events`
  - `/<ugv>/amcl_pose_odom`
  - `/<uav>/pose`
  - `/<uav>/pose_cmd`
  - `/<uav>/pose_cmd/odom`
  - `/<uav>/camera/actual/center_pose`
  - `/<uav>/camera/target/center_pose`
  - `/<uav>/follow/target/anchor_pose`
  - `/<uav>/follow/error/xy_distance_m`
  - `/<uav>/follow/error/yaw_rad`
- YOLO mode adds:
  - `/coord/leader_estimate`
  - `/coord/leader_estimate_status`
  - `/coord/leader_estimate_error`
- `profile:=vision` also adds:
  - `/<uav>/camera0/image_raw`
  - `/<uav>/camera0/camera_info`

## Capture Images For Datasets

Do this while the follow stack is already running.

Default output:
- `datasets/warehouse_auto`

Recommended workflow:
1. start the sim
2. start the UAV and follow stack
3. start dataset capture
4. change `d_target` during runtime
5. let Nav2 drive the UGV through the route
6. once happy, switch world / map campaign and restart all terminals if needed

This gives:
- multiple camera heights / distances
- multiple view angles
- multiple relative poses between UGV and UAV
- saved images and labels in `datasets/`

Wrapper:

```bash
./run.sh capture_dataset warehouse
```

Current capture-topic baseline:
- image topic default is `/<uav>/camera0/image_raw`
- camera info default is `/<uav>/camera0/camera_info`
- camera pose default is `/<uav>/camera/actual/center_pose`
- target pose default is `/<ugv>/amcl_pose_odom`, not raw `/platform/odom`
- older legacy `/<uav>/debug_camera_pose` is not published unless the simulator is started with legacy debug topics enabled

Important:
- dataset capture must use the AMCL-derived UGV pose topic for target geometry
- older captures made against raw `/platform/odom` should be treated as mislabelled for training

Examples:

```bash
./run.sh capture_dataset warehouse class:=car
./run.sh capture_dataset warehouse hz:=1.0
./run.sh capture_dataset warehouse out:=datasets/warehouse_auto_v2
```

If you want to keep dataset campaigns separate:

```bash
./run.sh capture_dataset warehouse out:=datasets/warehouse_run2
```

## Run The Sim With A YOLO Model

Start the normal sim first:

```bash
./run.sh gazebo_sim warehouse
./run.sh spawn_uav warehouse
./run.sh localization warehouse
./run.sh nav2
```

Then start the YOLO follow flow:

```bash
./run.sh 1to1_yolo warehouse
```

Tracker variant:

```bash
./run.sh 1to1_yolo warehouse tracker:=true
```

Tracker variant with explicit config:

```bash
./run.sh 1to1_yolo warehouse tracker:=true tracker_config:=botsort.yaml
```

Live follow control works here too:

```bash
./run.sh follow_control
```

Runtime command logger:

```bash
./run.sh uav_command_logger
```

This is now the preferred live logger for follow/camera behavior. It prints the actual published UAV body commands, camera pan/tilt deltas, speed, and direction labels, and logs under `debug_logs/uav_commands/`.

For YOLO mode, detailed follow-debug topics are now muted by default. Re-enable them if needed:

```bash
./run.sh 1to1_yolo warehouse publish_follow_debug_topics:=true publish_pose_cmd_topics:=true publish_camera_debug_topics:=true
```

Default `run.sh follow_control` mode is keyboard tuning for `d_target` and `z_min`. If you want the same parameter path explicitly, use:

```bash
./run.sh follow_control --mode params
```

Default behavior:
- Nav2 still drives the UGV through the current `ugv_nav2_driver` path
- the UAV/camera runtime uses `leader_estimate` rather than shared UGV pose
- estimate-mode YOLO repositions the UAV to the seeded startup pose before the first estimate arrives
- through `scripts/run_1to1_yolo.sh`, that startup pose defaults to `uav_start_x:=-7.0` and `uav_start_z:=7.0` in non-solar worlds
- the shared follow/camera fallback tilt baseline stays at `-45.0`
- held-estimate reuse is disabled by default, so rejected/missing detections now surface directly as `REJECT` / `NO_DET` instead of `*_HOLD`
- `./run.sh 1to1_yolo ...` now defaults to `obb:=true`
- with no explicit weights override, the current default weight is:
  - `obb/mymodels/warehouse-v1-yolo26n-obb.pt`
- wrapper launch defaults also force:
  - `leader_range_mode:=auto`
- truth/error and legacy debug topics are muted by default in the YOLO wrapper:
  - `leader_actual_pose_enable:=false`
  - `publish_follow_debug_topics:=false`
  - `publish_pose_cmd_topics:=false`
  - `publish_camera_debug_topics:=false`
- estimate-mode YOLO also enables shared actual heading by default unless you override it:
  - `leader_actual_heading_enable:=true`
  - use `use_actual_heading:=false` if you want motion/estimate heading behavior instead
- direct `ros2 launch lrs_halmstad run_follow.launch.py ...` does not inherit that quiet wrapper policy automatically
- bare custom weight names resolve like this:
  - `tracker:=false` -> under `models/detection/...`
  - `tracker:=true` -> under `models/obb/...`

Current live issue under investigation:
- distance holding is much better than before
- the active problems are body yaw on sharp turns / reversing and jumpy camera pan/tilt
- detector / tracker stability is better than the body/camera behavior right now

First topics to watch during that event:
- `/coord/leader_estimate_status`
- `/coord/leader_debug_image`
- `/<uav>/pose`
- `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- `/<uav>/update_pan`
- `/<uav>/update_tilt`

Truth-assisted test mode:

```bash
./run.sh 1to1_yolo warehouse use_estimate:=false
```

This keeps the YOLO estimator running, but switches the UAV/camera path back to shared UGV pose for comparison/testing.

Warehouse `car` filter:

```bash
./run.sh 1to1_yolo warehouse target:=car
```

Different YOLO weights:

```bash
./run.sh 1to1_yolo warehouse weights:=yolo26v2.pt
```

Detection-family shorthand:

```bash
./run.sh 1to1_yolo warehouse folder:=yolo26 weights:=yolo26s.pt
```

OBB-family default:

```bash
./run.sh 1to1_yolo warehouse obb:=true
```

Return to the plain detection-family default:

```bash
./run.sh 1to1_yolo warehouse obb:=false
```

OBB-family with explicit subfolder:

```bash
./run.sh 1to1_yolo warehouse obb:=true folder:=yolo26 weights:=yolo26s-obb.pt
```

Estimate error topic:

```bash
ros2 topic echo /coord/leader_estimate_error
```

This topic is muted by default in the current YOLO path. Re-enable it with `leader_actual_pose_enable:=true` if you want estimate-vs-truth diagnostics against `/<ugv>/amcl_pose_odom`.

Available YOLO models:

These can be passed to `weights:=...` as relative paths under `<workspace_root>/models`.

Detection models:
- `detection/yolo26/yolo26n.pt`
- `detection/yolo26/yolo26s.pt`
- `detection/yolo26/yolo26m.pt`
- `detection/yolo26/yolo26l.pt`
- `detection/yolo26/yolo26x.pt`

Custom models:
- `mymodels/yolo26v1.pt`
- `mymodels/yolo26v2.pt`

OBB models:
- `obb/yolo26/yolo26n-obb.pt`
- `obb/yolo26/yolo26s-obb.pt`
- `obb/yolo26/yolo26m-obb.pt`
- `obb/yolo26/yolo26l-obb.pt`
- `obb/yolo26/yolo26x-obb.pt`

## Scan A New Map With RViz

Use this when the current world does not already have a good Nav2 map.

1. Start Gazebo:

```bash
./run.sh gazebo_sim warehouse
```

2. Start SLAM:

```bash
./run.sh slam
```

3. Drive the UGV:

If the Gazebo GUI drive controls work, use them.

Keyboard option:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  -r cmd_vel:=/a201_0000/cmd_vel
```

4. Start RViz:

```bash
./run.sh nav2_rviz
```

5. Save the map:

Example save path for `warehouse_manual`:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli \
  -t /a201_0000/map \
  -f maps/warehouse_manual
```

This creates:
- `maps/warehouse_manual.yaml`
- `maps/warehouse_manual.pgm`

## Available Arguments And Values

### `./run.sh gazebo_sim`

- `WORLD`
  - any world name resolvable through Gazebo resource paths
  - `warehouse`, `orchard`, `solar_farm`, `pipeline`, `office`, `construction`, `walls`, `baylands`
- `GUI`
  - `true`
  - `false`
- extra forwarded launch arguments
  - `x:=...`
  - `y:=...`
  - `z:=...`
  - `yaw:=...`
  - `rviz:=true|false`
  - `auto_start:=true|false`
  - `use_sim_time:=true|false`
  - `rtf:=...`
  - `real_time_factor:=...`

### `./run.sh spawn_uav`

- positional `WORLD`
  - common example: `warehouse`
  - if omitted, it follows the active Gazebo world when available
- `name:=...`
  - UAV namespace / model name
  - common examples: `dji0`, `dji1`, `dji2`
- `height:=...`
  - UAV spawn height in meters
- `camera:=attached|detached`
  - shorthand for `uav_camera_mode:=integrated_joint|detached_model`
- `mount_pitch_deg:=...`
  - attached camera mount pitch in degrees
  - common examples: `35`, `45`
- `sensor_roll_deg:=...`
  - camera sensor roll offset in degrees
- `sensor_pitch_deg:=...`
  - camera sensor pitch offset in degrees
- `sensor_yaw_deg:=...`
  - camera sensor yaw offset in degrees
- extra forwarded launch arguments
  - anything else is passed through to `spawn_uav_1to1.launch.py`

### `./run.sh localization`

- positional `WORLD`
  - current tested example: `warehouse`
- optional second positional argument: map path
  - path to the `.yaml` occupancy map
  - example: `maps/warehouse_manual.yaml`
- extra forwarded launch arguments
  - anything else is passed through to `localization.launch.py`

### `./run.sh nav2`

- extra forwarded launch arguments
  - anything else is passed through to `nav2.launch.py`

### `./run.sh 1to1_follow`

- positional `WORLD`
  - current tested example: `warehouse`
- `camera:=attached|detached`
  - shorthand for `uav_camera_mode:=integrated_joint|detached_model`
- `height:=...`
  - maps to `uav_start_z:=...`
- `mount_pitch_deg:=...`
  - maps to `camera_mount_pitch_deg:=...`
- `use_tilt:=true|false`
  - maps to `tilt_enable:=true|false` for `camera_tracker`
- `follow_yaw:=true|false`
  - enable or disable UAV body yaw following
  - default is `true`
- extra forwarded launch arguments
  - anything else is passed through to `run_follow.launch.py`
- common forwarded launch arguments:
  - `uav_name:=dji0`
  - `uav_camera_mode:=integrated_joint|detached_model`
  - `ugv_mode:=nav2|external|none`
  - `ugv_set_initial_pose:=true|false`
  - `ugv_goal_sequence_csv:='x1,y1,yaw1;x2,y2,yaw2'`
  - `start_uav_simulator:=true|false`
  - `ugv_start_delay_s:=...`
- common forwarded values in the underlying launch:
  - `leader_mode:=odom|estimate|pose`
  - `start_leader_estimator:=auto|true|false`
  - `leader_perception_enable:=true|false`
  - `leader_range_mode:=ground|const|auto|depth`
  - `target_class_name:=...`
  - `target_class_id:=-1|<non-negative>`
  - `yolo_weights:=...`
  - `models_root:=...`
  - `yolo_device:=cpu|cuda|cuda:0`
  - `ugv_initial_pose_x:=...`
  - `ugv_initial_pose_y:=...`
  - `ugv_initial_pose_yaw_deg:=...`
  - `uav_start_x:=...`
  - `uav_start_y:=...`
  - `uav_start_yaw_deg:=...`

### `./run.sh 1to1_yolo`

- positional `WORLD`
  - current tested example: `warehouse`
- `camera:=attached|detached`
  - shorthand for `uav_camera_mode:=integrated_joint|detached_model`
- `weights:=...`
  - maps to `yolo_weights:=...`
  - bare filenames resolve under `detection/...` when `tracker:=false`
  - bare filenames resolve under `obb/...` when `tracker:=true`
  - example: `weights:=yolo26v2.pt`
- `folder:=...`
  - selects a subfolder under `detection/` or `obb/`
  - example: `folder:=yolo26`
- `obb:=true|false`
  - default `true`
  - with no explicit `weights:=...`, default weight is `obb/mymodels/warehouse-v1-yolo26n-obb.pt`
  - `false`: default weights switch to `detection/mymodels/warehouse_v1-v2-yolo26n.pt`
- `height:=...`
  - maps to `uav_start_z:=...`
- `mount_pitch_deg:=...`
  - maps to `camera_mount_pitch_deg:=...`
- `use_tilt:=true|false`
  - maps to `tilt_enable:=true|false` for `camera_tracker`
- `follow_yaw:=true|false`
  - enable or disable UAV body yaw following
  - default is `true`
- `camera_default_tilt_deg:=...`
  - forwarded through to launch if you want to override the shared fallback tilt
- `target:=...`
  - maps to `target_class_name:=...`
  - example: `target:=car`
- `use_estimate:=true|false`
  - default `true`: estimate-only YOLO follow path
  - `false`: truth-assisted test mode; still runs the estimator, but shares UGV pose into the follow/camera path
- `leader_actual_pose_enable:=true|false`
  - default `false` in `./run.sh 1to1_yolo`
  - enable `/coord/leader_estimate_error` truth diagnostics
- `use_actual_heading:=true|false`
  - default effective wrapper behavior in estimate mode is `true`
  - pass `false` if you want to avoid using shared AMCL heading in follow
- `publish_follow_debug_topics:=true|false`
  - default `false` in `./run.sh 1to1_yolo`
  - enable `/<uav>/follow/{target,actual,error,debug}/*`
- `publish_pose_cmd_topics:=true|false`
  - default `false` in `./run.sh 1to1_yolo`
  - enable legacy `/<uav>/pose_cmd` and `/<uav>/pose_cmd/odom`
- `publish_camera_debug_topics:=true|false`
  - default `false` in `./run.sh 1to1_yolo`
  - enable `/<uav>/camera/target/*`
- `tracker:=true|false`
  - default `false`
  - `true` switches `external_detection_node:=tracker`
- `tracker_config:=...`
  - current default from `run_follow.launch.py` is `botsort.yaml`
  - bare filenames resolve under `src/lrs_halmstad/config/trackers`
- extra forwarded launch arguments
  - anything else is passed through to `run_follow.launch.py`


### `./run.sh capture_dataset`

- `WORLD`
  - current tested example: `warehouse`
- default output folder
  - `datasets/<WORLD>_auto`
- `class:=...`
  - object class label to save in the dataset
  - current useful example: `car`
- `id:=...`
  - numeric class id to save instead of a class name
- `hz:=...`
  - capture frequency in Hz
  - example: `1.0`
- `negatives:=true|false`
  - whether to save negative examples
- `out:=...`
  - output directory for images and labels
  - examples: `datasets/warehouse_auto`, `datasets/warehouse_auto_v2`
- extra forwarded dataset parameters
  - any other `name:=value` is passed as a ROS parameter to `sim_dataset_capture`

Extra useful commands:

- inspect the map or set your own initial pose / Nav2 goal:

```bash
./run.sh nav2_rviz
```

- open the saved multi-camera / debug `rqt` layout:

```bash
./run.sh rqt_perspective
```

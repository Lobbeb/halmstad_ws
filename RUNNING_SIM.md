# Running Sim

Default tested path: `warehouse`.

Assumption:
- you already start in the workspace root

Recommended tmux workflow:
- start with `./run_tmux_1to1.sh warehouse`
- start YOLO mode with `./run_tmux_1to1.sh warehouse mode:=yolo`
- pass a custom YOLO weight with `./run_tmux_1to1.sh warehouse mode:=yolo weights:=yolo26v2.pt`
- enable truth-assisted YOLO testing with `./run_tmux_1to1.sh warehouse mode:=yolo use_estimate:=false`
- stop with `./stop_tmux_1to1.sh warehouse`
- default tmux layout is panes:
  - row 1: `gazebo | spawn`
  - row 2: `localization | nav2`
  - row 3: `follow`
- current default delays:
  - `gui:=false` -> `spawn=9`, `localization/nav2=11`, `follow=13`
  - `gui:=true` -> `spawn=7`, `localization/nav2=9`, `follow=11`

## Start 1-to-1 Sim With Nav2 And AMCL Follow

Run these in order in separate terminals:

1. Start Gazebo:

```bash
./run_gazebo_sim.sh warehouse
```

Without GUI:

```bash
./run_gazebo_sim.sh warehouse false
```

2. Spawn one UAV:

```bash
./run_spawn_uav.sh warehouse uav_name:=dji0
```

3. Start localization:

```bash
./run_localization.sh warehouse
```

If you want a different saved map:

```bash
./run_localization.sh warehouse maps/warehouse_manual.yaml
```

4. Start Nav2:

```bash
./run_nav2.sh
```

5. Start the AMCL-based odom follow stack:

```bash
./run_1to1_follow.sh warehouse
```

#### If you want the same stack started automatically in tmux, use:

```bash
./run_tmux_1to1.sh warehouse
```

YOLO variant in tmux:

```bash
./run_tmux_1to1.sh warehouse mode:=yolo
```

Default tmux layout is panes:
- Gazebo
- UAV spawn
- localization
- Nav2
- follow

Useful overrides:

```bash
./run_tmux_1to1.sh warehouse gui:=false
./run_tmux_1to1.sh warehouse delay_s:=9
./run_tmux_1to1.sh warehouse spawn_delay_s:=12 follow_delay_s:=18
./run_tmux_1to1.sh warehouse layout:=windows
./run_tmux_1to1.sh warehouse mode:=yolo
./run_tmux_1to1.sh warehouse mode:=yolo follow_yaw:=false use_tilt:=false
./run_tmux_1to1.sh warehouse tmux_attach:=false
./run_tmux_1to1.sh warehouse dry_run:=true
```

Default startup is staggered with short delays so the later launches do not all fire at the same instant.

Default delays depend on `gui:=true|false`:
- `gui:=false` uses `spawn=9`, `localization/nav2=11`, and `follow=13`
- `gui:=true` uses `spawn=7`, `localization/nav2=9`, and `follow=11`

If your machine is slower, increase the delay args:
- `delay_s:=...`
- `spawn_delay_s:=...`
- `localization_delay_s:=...`
- `nav2_delay_s:=...`
- `follow_delay_s:=...`

If you want separate tmux windows instead of the default panes:

```bash
./run_tmux_1to1.sh warehouse layout:=windows
```

Alias:

```bash
./run_tmux_1to1.sh warehouse panes:=true
```

Pane layout is:
- row 1: Gazebo | spawn
- row 2: localization | Nav2
- row 3: follow

To stop the tmux-managed stack cleanly, use:

```bash
./stop_tmux_1to1.sh warehouse
```

This sends `Ctrl-c` in this order:
- follow, localization, and Nav2 together
- wait `5s`
- Gazebo
- spawn is expected to exit automatically when Gazebo goes down

Then it waits a few seconds, kills the tmux session, performs a safety cleanup pass for leftover Gazebo / launch processes, and clears stale helper state files under `/tmp/halmstad_ws`.

Useful stop overrides:

```bash
./stop_tmux_1to1.sh warehouse group_grace_s:=2
./stop_tmux_1to1.sh warehouse final_grace_s:=8
./stop_tmux_1to1.sh warehouse kill_session:=false
./stop_tmux_1to1.sh session:=halmstad-warehouse-1to1
```

Current baseline:
- this odom-follow path now uses `/<ugv>/amcl_pose_odom`, not raw `/platform/odom`
- `/<ugv>/amcl_pose_odom` is synthesized from `/<ugv>/amcl_pose` by [pose_cov_to_odom.py](src/lrs_halmstad/lrs_halmstad/pose_cov_to_odom.py)
- launch `leader_odom_topic` / `ugv_odom_topic` defaults are intentionally pointed at that AMCL-derived topic
- current UAV camera mode is detached: `uav_camera_mode:=detached_model`
- current camera defaults are `pan_enable: true` and `tilt_enable: true`
- attached mode is still available as an override with `camera:=attached`
- if you override the mount pitch for attached mode, pass the same value to follow:

```bash
./run_spawn_uav.sh warehouse uav_name:=dji0 camera:=attached mount_pitch_deg:=35
./run_1to1_follow.sh warehouse camera:=attached mount_pitch_deg:=35
```

To test the older attached camera path instead of the detached default:

```bash
./run_spawn_uav.sh warehouse camera:=attached
./run_1to1_follow.sh warehouse camera:=attached
```

Important:
- pass the same camera mode to both spawn and follow
- detached mode does not rely on the attached `45 deg` mount pitch baseline
- the wrapper alias also accepts `camera:=attached`

Important runtime note:
- Gazebo sim time is guarded through [clock_guard.py](src/lrs_halmstad/lrs_halmstad/clock_guard.py)
- expected `/clock` publisher is `clock_guard`
- if Gazebo is restarted or the world is reset, restart localization, Nav2, and follow

## Start The Camera Views

Saved multi-camera / debug layout:

```bash
./run_rqt_perspective.sh
```

Then under `Perspectives`, choose one from the `perspectives/` folder if needed.

UAV camera: ``/dji0/camera0/image_raw``

UGV camera: ``/a201_0000/sensors/camera_0/color/image``

YOLO debug image: ``/coord/leader_debug_image``


Only use the YOLO debug image when you started the YOLO flow from the section below.

## Control Height And Distance During Runtime

These work while `run_1to1_follow.sh` or `run_1to1_yolo.sh` is running.

Recommended live control:
- change `d_euclidean` during runtime
- this updates the derived `d_target` and `z_alt` together
- no restart is needed for this control
- the direct `ros2` commands below assume the terminal is already sourced

Change the 3D UAV-to-UGV follow distance:

```bash
ros2 param set /follow_uav d_euclidean x.xx
```

Examples:

```bash
ros2 param set /follow_uav d_euclidean 5.0
ros2 param set /follow_uav d_target 9.0
ros2 param set /follow_uav z_alt 9.0
```

Use `d_target` and `z_alt` only if you explicitly want to control horizontal distance and altitude separately.

Runtime follow control helper:

```bash
./run_follow_control.sh
```

Default keyboard mode:
- this is the live tuning path for `d_target` and `z_alt`
- `w` raises `z_alt`
- `s` lowers `z_alt`
- `d` increases `d_target`
- `a` decreases `d_target`
- `r` refreshes the current values
- `h` shows the help text
- `q` quits

Random sweep mode:
- pass `--mode random`
- current defaults:
- updates every `10s`
- `z_alt` sampled uniformly in `[2, 40]`
- `d_target` sampled uniformly in `[1, 20]`
- sampling is biased toward the `5-15` band by default
- runs until you stop it with `Ctrl-c`

Useful examples:

```bash
./run_follow_control.sh
./run_follow_control.sh --step-z-alt 2 --step-d-target 2
./run_follow_control.sh --mode params --step-z-alt 2 --step-d-target 2
./run_follow_control.sh --mode random --interval 8
./run_follow_control.sh --mode random --count 20
./run_follow_control.sh --mode random --seed 42
./run_follow_control.sh --mode random --interval 8 --count 20 --seed 42
./run_follow_control.sh --mode random --focus-weight 0.8
```

## Record Pose Alignment For AMCL Analysis

Use this when you want to measure how raw `platform/odom` differs from the AMCL-derived pose during a real sim run.

1. Start the normal sim stack:

```bash
./run_gazebo_sim.sh warehouse
./run_spawn_uav.sh warehouse
./run_localization.sh warehouse
./run_nav2.sh
./run_1to1_follow.sh warehouse
```

2. In one extra terminal, start recording:

```bash
./run_record_pose_alignment.sh
```

Default recorded topics:
- `/clock`
- `/<ugv>/platform/odom`
- `/<ugv>/platform/odom/filtered`
- `/<ugv>/amcl_pose`
- `/<ugv>/amcl_pose_odom`
- `/<ugv>/tf`
- `/<ugv>/tf_static`
- `/<uav>/pose`
- `/<uav>/pose_cmd`
- `/<uav>/camera/actual/center_pose`

3. Let the UGV drive for a while, especially through turns and longer paths, then stop the recorder with `Ctrl-c`.

4. Analyze the bag:

```bash
source /opt/ros/jazzy/setup.bash
python3 ./run_analyze_pose_alignment.py runs/pose_alignment/<bag_dir>
```

Example with CSV export:

```bash
source /opt/ros/jazzy/setup.bash
python3 ./run_analyze_pose_alignment.py runs/pose_alignment/<bag_dir> \
  --csv runs/pose_alignment/alignment_report.csv
```

Read the result like this:
- small rigid-fit residuals mean a single 2D correction might approximate the run
- large rigid-fit residuals mean the correction is time-varying, so old dataset boxes are not safely recoverable with one static transform

## Capture Images For Datasets

Do this while the follow stack is already running.

Default output:
- `datasets/warehouse_auto`

Recommended workflow:
1. start the sim
2. start the UAV and follow stack
3. start dataset capture
4. change `d_euclidean` during runtime
5. let Nav2 drive the UGV through the route
6. once happy, switch world / map campaign and restart all terminals if needed

This gives:
- multiple camera heights / distances
- multiple view angles
- multiple relative poses between UGV and UAV
- saved images and labels in `datasets/`

Wrapper:

```bash
./run_capture_dataset.sh warehouse
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
./run_capture_dataset.sh warehouse class:=car
./run_capture_dataset.sh warehouse hz:=1.0
./run_capture_dataset.sh warehouse out:=datasets/warehouse_auto_v2
```

If you want to keep dataset campaigns separate:

```bash
./run_capture_dataset.sh warehouse out:=datasets/warehouse_run2
```

## Run The Sim With A YOLO Model

Start the normal sim first:

```bash
./run_gazebo_sim.sh warehouse
./run_spawn_uav.sh warehouse
./run_localization.sh warehouse
./run_nav2.sh
```

Then start the YOLO follow flow:

```bash
./run_1to1_yolo.sh warehouse
```

Live follow control works here too:

```bash
./run_follow_control.sh
```

Runtime turn analysis:

```bash
./run_follow_debug.sh
```

This subscribes to the follow, estimator, and camera topics and prints an interpreted diagnosis such as:
- camera-only pan is moving
- UAV body yaw command is moving
- both camera and body are moving
- suspicious motion while estimator state is `NO_DET` / `REJECT`
- logs are also saved by default under `debug_logs/follow_debug/`

That default mode is now direct manual UAV steering. If you want the old runtime `d_target` / `z_alt` tuning path instead, use:

```bash
./run_follow_control.sh --mode params
```

Default behavior:
- Nav2 still drives the UGV through the current `ugv_nav2_driver` path
- the UAV/camera runtime uses `leader_estimate` rather than shared UGV pose
- estimate-mode YOLO repositions the UAV to the seeded startup pose before the first estimate arrives
- through `run_1to1_yolo.sh`, that startup pose defaults to `uav_start_x:=-7.0` and `uav_start_z:=7.0` in non-solar worlds
- the shared follow/camera fallback tilt baseline stays at `-45.0`
- held-estimate reuse is disabled by default, so rejected/missing detections now surface directly as `REJECT` / `NO_DET` instead of `*_HOLD`
- estimator error is still computed against `/<ugv>/amcl_pose_odom` by default
- bare custom weight names resolve under `models/detection/mymodels/`

Truth-assisted test mode:

```bash
./run_1to1_yolo.sh warehouse use_estimate:=false
```

This keeps the YOLO estimator running, but switches the UAV/camera path back to shared UGV pose for comparison/testing.

Warehouse `car` filter:

```bash
./run_1to1_yolo.sh warehouse target:=car
```

Different YOLO weights:

```bash
./run_1to1_yolo.sh warehouse weights:=yolo26v2.pt
```

Detection-family shorthand:

```bash
./run_1to1_yolo.sh warehouse folder:=yolo26 weights:=yolo26s.pt
```

OBB-family default:

```bash
./run_1to1_yolo.sh warehouse obb:=true
```

OBB-family with explicit subfolder:

```bash
./run_1to1_yolo.sh warehouse obb:=true folder:=yolo26 weights:=yolo26s-obb.pt
```

Estimate error topic:

```bash
ros2 topic echo /coord/leader_estimate_error
```

This topic is populated by default in the current YOLO path and is computed against `/<ugv>/amcl_pose_odom`. Disable it only if you explicitly pass `leader_actual_pose_enable:=false`.

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

YOLOv5 models:
- `detection/yolo5/yolov5n.pt`
- `detection/yolo5/yolov5nu.pt`
- `detection/yolo5/yolov5s.pt`
- `detection/yolo5/yolov5su.pt`

## Scan A New Map With RViz

Use this when the current world does not already have a good Nav2 map.

1. Start Gazebo:

```bash
./run_gazebo_sim.sh warehouse
```

2. Start SLAM:

```bash
./run_slam.sh
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
./run_nav2_rviz.sh
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

### `./run_gazebo_sim.sh`

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

### `./run_spawn_uav.sh`

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

### `./run_localization.sh`

- positional `WORLD`
  - current tested example: `warehouse`
- optional second positional argument: map path
  - path to the `.yaml` occupancy map
  - example: `maps/warehouse_manual.yaml`
- extra forwarded launch arguments
  - anything else is passed through to `localization.launch.py`

### `./run_nav2.sh`

- extra forwarded launch arguments
  - anything else is passed through to `nav2.launch.py`

### `./run_1to1_follow.sh`

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
  - default is `false`
- extra forwarded launch arguments
  - anything else is passed through to `run_1to1_follow.launch.py`
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
  - `leader_constant_range_m:=...`
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

### `./run_1to1_yolo.sh`

- positional `WORLD`
  - current tested example: `warehouse`
- `camera:=attached|detached`
  - shorthand for `uav_camera_mode:=integrated_joint|detached_model`
- `weights:=...`
  - maps to `yolo_weights:=...`
  - bare filenames default to `detection/mymodels/`
  - example: `weights:=yolo26v2.pt`
- `folder:=...`
  - selects a subfolder under `detection/` or `obb/`
  - example: `folder:=yolo26`
- `obb:=true|false`
  - default `false`: unresolved `weights:=...` paths are resolved under `detection/`
  - with no `folder:=...`, bare filenames resolve under `detection/mymodels/`
  - `true`: default weights switch to the OBB family, and unresolved `weights:=...` paths are resolved under `obb/`
- `height:=...`
  - maps to `uav_start_z:=...`
- `mount_pitch_deg:=...`
  - maps to `camera_mount_pitch_deg:=...`
- `use_tilt:=true|false`
  - maps to `tilt_enable:=true|false` for `camera_tracker`
- `follow_yaw:=true|false`
  - enable or disable UAV body yaw following
  - default is `false`
- `camera_default_tilt_deg:=...`
  - forwarded through to launch if you want to override the shared fallback tilt
- `target:=...`
  - maps to `target_class_name:=...`
  - example: `target:=car`
- `use_estimate:=true|false`
  - default `true`: estimate-only YOLO follow path
  - `false`: truth-assisted test mode; still runs the estimator, but shares UGV pose into the follow/camera path
- extra forwarded launch arguments
  - anything else is passed through to `run_1to1_follow.launch.py`


### `./run_capture_dataset.sh`

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
./run_nav2_rviz.sh
```

- open the saved multi-camera / debug `rqt` layout:

```bash
./run_rqt_perspective.sh
```


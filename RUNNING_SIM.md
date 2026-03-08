# Running Sim

Default tested path: `warehouse`.

The commands below are written in execution order. If a step has alternatives, they are listed directly under that step.

Assumption: you already start in `~/halmstad_ws`.

For the ROS launch commands in this file, keep `world:=...` as the last argument so it is easy to swap worlds later.

## Current Goal

Current focus:
- run the 1-to-1 UGV/UAV sim in `warehouse`
- change UAV follow distance and height while the sim is running
- record auto-labeled images into `datasets/`
- use those datasets later to train a YOLO model

Current working idea:
- `leader_mode:=odom` is the control-debug baseline
- `leader_mode:=estimate` is the YOLO path
- dataset capture should usually be done while the sim is moving and while live follow-distance changes are being tested

## Main Flow

### Step 1: Start Gazebo

With GUI:

```bash
./run_gazebo_sim.sh warehouse
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_gazebo_sim.sh warehouse
```

Without GUI:

```bash
./run_gazebo_sim.sh warehouse false
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_gazebo_sim.sh warehouse false
```

### Step 2: Spawn One UAV

GUI option:

1. Wait for Gazebo to finish loading.
2. Open the UAV spawner plugin.
3. Spawn one UAV.
4. Use `dji0`.

Terminal option:

```bash
./run_spawn_uav.sh warehouse uav_name:=dji0
```

Current default:
- spawns the UAV with the camera attached to the UAV model
- attached mount pitch defaults to `45 deg`
- `uav_camera_mode:=integrated_joint` is now the default follow/simulator mode
- if you want the old detached camera path explicitly, pass `uav_camera_mode:=detached_model`

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad spawn_uav_1to1.launch.py \
  uav_name:=dji0 \
  world:=warehouse
```

### Step 3: Start Localization

Default `warehouse` map:

```bash
./run_localization.sh warehouse
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
ros2 launch clearpath_nav2_demos localization.launch.py \
  use_sim_time:=true \
  setup_path:=/home/ruben/clearpath \
  scan_topic:=/a201_0000/sensors/lidar2d_0/scan \
  map:=/opt/ros/jazzy/share/clearpath_nav2_demos/maps/warehouse.yaml
```

If you have your own saved map for another world:

```bash
./run_localization.sh warehouse maps/warehouse_manual.yaml
```
or
```bash

ros2 launch clearpath_nav2_demos localization.launch.py \
  ...
  map:=maps/warehouse_manual.yaml
```

### Step 4: Start Nav2

```bash
./run_nav2.sh
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
ros2 launch clearpath_nav2_demos nav2.launch.py \
  use_sim_time:=true \
  setup_path:=/home/ruben/clearpath \
  scan_topic:=/a201_0000/sensors/lidar2d_0/scan
```

### Step 5: Start The 1-to-1 Follow Stack

Odom follow option:

```bash
./run_1to1_follow.sh warehouse
```

Current default:
- uses `uav_camera_mode:=integrated_joint`
- uses attached-gimbal pitch tracking by default
- if you override the spawn mount angle, pass the same value to follow, for example `./run_1to1_follow.sh warehouse mount_pitch_deg:=35`

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_1to1_follow.launch.py \
  ugv_mode:=nav2 \
  ugv_set_initial_pose:=true \
  leader_mode:=odom \
  world:=warehouse
```

YOLO estimate follow option:

```bash
./run_1to1_yolo.sh warehouse
```

Warehouse `car` filter:

```bash
./run_1to1_yolo.sh warehouse target:=car
```

Different YOLO weights:

```bash
./run_1to1_yolo.sh warehouse weights:=detection/yolo26/yolo26s.pt
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_1to1_follow.launch.py \
  ugv_mode:=nav2 \
  ugv_set_initial_pose:=true \
  leader_mode:=estimate \
  start_leader_estimator:=true \
  leader_range_mode:=ground \
  yolo_weights:=detection/yolo26/yolo26l.pt \
  world:=warehouse
```

Raw command with the warehouse `car` filter:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_1to1_follow.launch.py \
  ugv_mode:=nav2 \
  ugv_set_initial_pose:=true \
  leader_mode:=estimate \
  start_leader_estimator:=true \
  leader_range_mode:=ground \
  target_class_name:=car \
  yolo_weights:=detection/yolo26/yolo26l.pt \
  world:=warehouse
```
See available YOLO models at bottom of page.

### Step 5A: Live Controls During Follow

These work while `run_1to1_follow.sh` or `run_1to1_yolo.sh` is running.

Recommended control:
- change `d_euclidean` during runtime
- this updates the derived `d_target` and `z_alt` together
- use this while recording datasets so the model sees multiple heights and follow distances in one run

Change the 3D UAV-to-UGV follow distance:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 param set /follow_uav d_euclidean 12.0
```

Notes:
- this is applied during runtime
- this is meant for active sim runs and dataset generation
- restart is not needed for this control
- map changes still require restarting localization and Nav2

Advanced direct controls:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 param set /follow_uav d_target 9.0
ros2 param set /follow_uav z_alt 9.0
```

Use these only if you explicitly want to control horizontal distance and altitude separately.

### Step 6: View The Camera Topics

Saved multi-camera / debug layout:

```bash
./run_rqt_perspective.sh
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rqt_gui rqt_gui --perspective-file perspectives/monitor_UGVUAV_with_YOLO
```

UAV camera:

```bash
./run_rqt_image_view.sh /dji0/camera0/image_raw
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_rqt_image_view.sh /dji0/camera0/image_raw
```

### Step 7: Capture Auto-Labeled Sim Images

Do this while the follow stack is already running.

Default output:
- `datasets/warehouse_auto`

Recommended workflow:
1. start the sim
2. start the UAV and follow stack
3. start dataset capture
4. change `d_euclidean` during runtime
5. let Nav2 drive the UGV through the route

This gives:
- multiple camera heights / distances
- multiple view angles
- multiple relative poses between UGV and UAV
- saved images and labels in `datasets/`

Wrapper:

```bash
./run_capture_dataset.sh warehouse
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run lrs_halmstad sim_dataset_capture --ros-args \
  -p use_sim_time:=true \
  -p output_dir:=datasets/warehouse_auto \
  -p dataset_name:=warehouse
```

Examples:

```bash
./run_capture_dataset.sh warehouse class:=car
./run_capture_dataset.sh warehouse hz:=1.0
./run_capture_dataset.sh warehouse out:=datasets/warehouse_auto_v2
```

Examples while changing geometry during the same run:

```bash
ros2 param set /follow_uav d_euclidean 8.0
ros2 param set /follow_uav d_euclidean 10.0
ros2 param set /follow_uav d_euclidean 12.0
```

If you switch to a different world or want to keep dataset campaigns separate, use a different output folder:

```bash
./run_capture_dataset.sh warehouse out:=datasets/warehouse_run2
```

UGV camera:

```bash
./run_rqt_image_view.sh /a201_0000/sensors/camera_0/color/image
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_rqt_image_view.sh /a201_0000/sensors/camera_0/color/image
```

YOLO debug image:

```bash
./run_rqt_image_view.sh /coord/leader_debug_image
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_rqt_image_view.sh /coord/leader_debug_image
```

Only use the YOLO debug image when you started the YOLO estimate follow option in Step 5.

Estimate error topic:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /coord/leader_estimate_error
```

## Scan A New Map

Use this when the current world does not already have a good Nav2 map.

### Step 1: Start Gazebo

```bash
./run_gazebo_sim.sh warehouse
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_gazebo_sim.sh warehouse
```

### Step 2: Start SLAM

```bash
./run_slam.sh
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
ros2 launch clearpath_nav2_demos slam.launch.py \
  use_sim_time:=true \
  setup_path:=/home/ruben/clearpath \
  scan_topic:=/a201_0000/sensors/lidar2d_0/scan
```

### Step 3: Drive The UGV

If the Gazebo GUI drive controls work, use them.

Keyboard option:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  -r cmd_vel:=/a201_0000/cmd_vel
```

### Step 4: Start RViz

```bash
./run_nav2_rviz.sh
```

Raw command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nav2_bringup rviz_launch.py \
  namespace:=a201_0000 \
  use_namespace:=true \
  use_sim_time:=true \
  rviz_config:=/home/ruben/halmstad_ws/src/lrs_halmstad/config/nav2_namespaced_waypoints.rviz
```

### Step 5: Save The Map

Example save path for `warehouse_manual`:

```bash
mkdir -p maps
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli \
  -t /a201_0000/map \
  -f maps/warehouse_manual
```

This creates:
- `maps/warehouse_manual.yaml`
- `maps/warehouse_manual.pgm`

## Notes

- The built-in Nav2 initial pose and route defaults now include `warehouse`.
- `run_1to1_follow.sh` defaults to `leader_mode:=odom`.
- `run_1to1_yolo.sh` defaults to `leader_mode:=estimate`.
- `run_spawn_uav.sh` defaults to `uav_name:=dji0`.
- If you switch to another world and it does not have calibrated defaults yet, either set the initial pose in RViz or pass explicit launch overrides.
- To override the built-in Nav2 route, pass:

```bash
ugv_goal_sequence_csv:='x1,y1,yaw1;x2,y2,yaw2;x3,y3,yaw3'
```

- If you want manual Nav2 goals instead of the built-in route, use:

```bash
ugv_mode:=external
```

## Argument Reference

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


### `spawn_uav_1to1.launch.py`

- `uav_name`
  - any UAV namespace / model name
  - common examples: `dji0`, `dji1`, `dji2`
- `uav_mode`
  - `teleport` : moves UAV by adjusting pose
  - `physics` : activates gravity for UAVs (not implemented)
- `camera_name`
  - default: `camera0`
  - can be changed if you want a different camera topic prefix
- `x`, `y`, `z`
  - UAV spawn coordinates
- `world`
  - any Gazebo world name


### `clearpath_nav2_demos localization.launch.py`

- `map`
  - path to the `.yaml` occupancy map

### `clearpath_nav2_demos nav2.launch.py`

### `run_1to1_follow.launch.py`

- `world`
  - any Gazebo world name
- `uav_name`
  - common examples: `dji0`, `dji1`, `dji2`
- `leader_mode`: selects what the UAV follows
  - `odom`: follow real UGV odometry
  - `estimate`: follow `/coord/leader_estimate` from `leader_estimator` / YOLO
  - `pose`: same estimator-driven path, mainly for direct pose-input workflows
- `start_leader_estimator`
  - `auto`
  - `true`
  - `false`
- `leader_perception_enable`
  - `true`
  - `false`
- `ugv_mode`: selects how the UGV moves
  - `nav2`: built-in Nav2 route driver sends goals automatically
  - `external` or `none`: you provide Nav2 goals yourself from RViz or another node
- `ugv_set_initial_pose`
  - `true`
  - `false`
- `ugv_initial_pose_x`, `ugv_initial_pose_y`, `ugv_initial_pose_yaw_deg`
  - explicit map-frame initial pose overrides
- `ugv_goal_sequence_csv`
  - semicolon-separated list of `x,y,yaw_deg`
  - example:
    - `'0.0,0.0,0.0;5.0,0.0,0.0;5.0,5.0,90.0'`
- `uav_camera_mode`: selects how the UAV camera is simulated
  - `detached_model` or `detached`: separate camera model moved by the simulator
  - `integrated_joint` or `integrated`: camera stays on the UAV and uses gimbal joint commands
- `leader_range_mode`: selects how distance to the detected target is estimated
  - `ground`: project the detection ray onto the ground plane
  - `const`: use `leader_constant_range_m`
  - `auto`: prefer depth, then ground, then constant fallback
  - `depth`: require a depth image topic
- `leader_constant_range_m`
  - numeric range in meters, used when `leader_range_mode:=const`
- `target_class_name`: selects which detected object class to accept
  - empty string: accept any detected class
  - `car`: useful for the current warehouse Husky tests
  - any other class name must match the YOLO class label exactly
- `target_class_id`: numeric class filter
  - `-1`: disabled
  - non-negative integer: accept only that class id
- `yolo_weights`
  - path to a YOLO weights file
  - absolute paths work
  - relative paths are resolved under `/home/ruben/halmstad_ws/models`
  - empty string disables explicit weights override
- `models_root`
  - default: `/home/ruben/halmstad_ws/models`
  - used when `yolo_weights` is a relative path
- `yolo_device`
  - free-form device string
  - common examples: `cpu`, `cuda`, `cuda:0`
- `uav_start_x`, `uav_start_y`, `uav_start_yaw_deg`
  - explicit UAV start pose overrides
- `start_uav_simulator`
  - `true`
  - `false`
- `ugv_start_delay_s`
  - numeric delay before UGV motion starts

### `run_1to1_yolo.sh`

- `weights:=...`
  - maps to `yolo_weights:=...`
  - example: `weights:=detection/yolo26/yolo26s.pt`
- `target:=...`
  - maps to `target_class_name:=...`
  - example: `target:=car`

### `run_yolo.sh`

- `weights:=...`
  - maps to `yolo_version:=...`
  - example: `weights:=detection/yolo26/yolo26s.pt`
- `target:=...`
  - maps to `target_class_name:=...`
  - example: `target:=car`

- If you want to inspect the map, set your own initial pose, or send your own `Nav2 Goal`, start RViz with:

```bash
./run_nav2_rviz.sh
```

- If you want the saved multi-camera / debug `rqt` layout, use:

```bash
./run_rqt_perspective.sh
```

## Available YOLO Models

These can be passed to `yolo_weights:=...` as relative paths because the default model root is `/home/ruben/halmstad_ws/models`.

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

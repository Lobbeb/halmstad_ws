# lrs_halmstad

Reference for the current 1-to-1 Gazebo/ROS 2 workflow.

## Topic contract

### Husky UGV (`a201_0000`)

- `/a201_0000/cmd_vel`
- `/a201_0000/sensors/camera_0/color/image`
- `/a201_0000/sensors/camera_0/color/compressed`
- `/a201_0000/sensors/camera_0/color/camera_info`
- `/a201_0000/sensors/camera_0/depth/image`
- `/a201_0000/sensors/camera_0/depth/compressedDepth`
- `/a201_0000/sensors/lidar2d_0/scan`

### UAV (`dji0` by default)

- `/dji0/camera0/image_raw`
- `/dji0/camera0/camera_info`
- `/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- `/dji0/update_pan`
- `/dji0/update_tilt`
- `/dji0/pose`
- `/dji0/pose_cmd`
- `/dji0/pose_cmd/odom`

## Recommended 1-to-1 bring-up

### 1. Start Gazebo + Husky

```bash
cd <workspace_root>
./run_gazebo_sim.sh warehouse
```

### 2. Spawn one UAV

Use either the GUI spawn plugin or the dedicated detached-camera launch:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad spawn_uav_1to1.launch.py
```

### 3. Start the 1-to-1 UAV follow stack

This starts the UAV simulator adapter plus the follow stack for `dji0` and `a201_0000`.

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_1to1_follow.launch.py
```

Useful overrides:

```bash
ros2 launch lrs_halmstad run_1to1_follow.launch.py leader_mode:=odom
ros2 launch lrs_halmstad run_1to1_follow.launch.py leader_mode:=estimate start_leader_estimator:=true
ros2 launch lrs_halmstad run_1to1_follow.launch.py leader_mode:=estimate start_leader_estimator:=true leader_range_mode:=ground yolo_weights:=detection/yolo26/yolo26l.pt
ros2 launch lrs_halmstad run_1to1_follow.launch.py ugv_mode:=nav2
ros2 launch lrs_halmstad run_1to1_follow.launch.py ugv_mode:=nav2 ugv_initial_pose_x:=1.0 ugv_initial_pose_y:=2.0 ugv_initial_pose_yaw_deg:=90.0
```

For `ugv_mode:=nav2`, start Clearpath localization and Nav2 separately first. The follow launch will then drive the Husky through `/a201_0000/navigate_to_pose` using the configured waypoint route.

If you want to test Nav2 goals from a separate node, use `ugv_mode:=external` in the follow launch so it does not also start its own UGV driver.

The Nav2 UGV driver now also auto-publishes `/a201_0000/initialpose` before sending goals. The `ugv_initial_pose_*` values are in the saved map frame, not Gazebo world coordinates.

For `world:=orchard`, the launch defaults now auto-fill the current verified orchard map-frame initial pose, so you do not need to manually publish `/a201_0000/initialpose` for the normal Nav2 bring-up. The recommended orchard Nav2 map is `maps/orchard_nav.yaml`, which keeps black occupied cells and frees gray terrain/unknown cells from the raw SLAM map. Other worlds still default to `0,0,0` unless overridden.

For standalone Nav2 testing without RViz goals:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run lrs_halmstad ugv_nav2_goal_tester --ros-args \
  -r __ns:=/a201_0000 \
  -p pattern:=square \
  -p pattern_size_m:=2.0
```

This waits for `/a201_0000/amcl_pose`, then sends a simple square of `NavigateToPose` goals and publishes the planned test path on `/a201_0000/test_goal_path`.

### 4. View the UAV camera

```bash
cd <workspace_root>
./run_rqt_image_view.sh /dji0/camera0/image_raw
```

## Manual smoke test path

Use this when testing the legacy UAV control topics directly.

### Spawn only the UAV

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad spawn_uav_1to1.launch.py
```

### Start the Gazebo adapter

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run lrs_halmstad simulator --ros-args -p uav_name:=dji0 -p camera_mode:=detached_model
```

### Publish manual test motion

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run lrs_halmstad controller --ros-args -p uav_name:=dji0
```

## Notes

- `follow_uav` no longer talks to Gazebo directly. It publishes to `/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw` and leaves Gazebo actuation to `simulator`.
- In simulation, `simulator` interprets `/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw` as an absolute ENU pose setpoint: `x`, `y`, `z`, `yaw`.
- The detached camera model is the default backend for visible camera motion in Gazebo, but the ROS image topics remain `/dji0/camera0/*`.
- `leader_estimator` now defaults to the actual simulated UAV pose topic `/dji0/pose`, not `/dji0/pose_cmd`.
- The default perception range mode is `ground`, not fixed range. `leader_range_mode:=const` is still available as a fallback for debugging.
- The Husky now uses `lidar2d_0` as its only active range sensor in this workspace. The old temporary `lidar3d_0` path is no longer part of the active bring-up.
- UGV mobility now runs in two supported modes: `ugv_mode:=nav2` sends sequential Nav2 `NavigateToPose` goals derived from the configured route, and `ugv_mode:=external` leaves UGV motion to an external Nav2 goal source.

## Contract checks

Base Gazebo + UGV + UAV camera contract:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run lrs_halmstad contract_check orchard dji0
```

Include the UAV simulator adapter topics:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
REQUIRE_UAV_ADAPTER=1 ros2 run lrs_halmstad contract_check orchard dji0
```

Include the follow-stack topics as well:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
REQUIRE_UAV_ADAPTER=1 REQUIRE_FOLLOW_STACK=1 ros2 run lrs_halmstad contract_check orchard dji0
```

Include estimator topics too when running `leader_mode:=estimate`:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
REQUIRE_UAV_ADAPTER=1 REQUIRE_FOLLOW_STACK=1 REQUIRE_ESTIMATOR=1 ros2 run lrs_halmstad contract_check orchard dji0
```

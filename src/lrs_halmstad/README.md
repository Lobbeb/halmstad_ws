# lrs_halmstad

Reference for the current 1-to-1 Gazebo/ROS 2 workflow.

Current real follow launch:
- `run_follow.launch.py`

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
- `/dji0/camera0/depth_image`
- `/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- `/dji0/update_pan`
- `/dji0/update_tilt`
- `/dji0/pose`

Legacy optional debug topics:
- `/dji0/pose_cmd`
- `/dji0/pose_cmd/odom`

Perception / estimate topics:
- `/coord/leader_detection`
- `/coord/leader_estimate`
- `/coord/leader_estimate_status`
- `/coord/leader_estimate_fault`
- `/coord/leader_debug_image`
- `/coord/leader_distance_debug`

OMNeT++ network metrics topics (published by `omnet_metrics_bridge` when OMNeT is running,
requires `start_omnet_bridge:=true` in `run_follow.launch.py`; all `std_msgs/Float64`, ~10 Hz):

- `/omnet/sim_time`          — OMNeT simulation time (s)
- `/omnet/link_distance`     — geometric UAV–UGV distance from Gazebo positions (m)
- `/omnet/rssi_dbm`          — received signal strength (dBm, free-space path-loss model)
- `/omnet/snir_db`           — signal-to-noise-plus-interference ratio (dB)
- `/omnet/packet_error_rate` — sliding-window PER estimate (0–1)
- `/omnet/radio_distance`    — range estimate from RSSI inversion only, no Gazebo positions (m)

## Recommended 1-to-1 bring-up

### 1. Start Gazebo + Husky

```bash
cd <workspace_root>
./run.sh gazebo_sim warehouse
```

### 2. Spawn one UAV

Use the GUI spawn plugin or the integrated-camera launch:

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
ros2 launch lrs_halmstad run_follow.launch.py
```

Useful overrides:

```bash
ros2 launch lrs_halmstad run_follow.launch.py leader_mode:=odom
ros2 launch lrs_halmstad run_follow.launch.py leader_mode:=estimate start_leader_estimator:=true
ros2 launch lrs_halmstad run_follow.launch.py ugv_mode:=nav2
ros2 launch lrs_halmstad run_follow.launch.py ugv_mode:=nav2 ugv_initial_pose_x:=1.0 ugv_initial_pose_y:=2.0 ugv_initial_pose_yaw_deg:=90.0
```

Important launch-default nuance:
- `run_follow.launch.py` still declares `leader_actual_pose_enable`, `publish_follow_debug_topics`, `publish_pose_cmd_topics`, and `publish_camera_debug_topics` with launch-default `true`
- `./run.sh 1to1_yolo ...` overrides those to a quieter runtime

For `ugv_mode:=nav2`, start Clearpath localization and Nav2 separately first. The follow launch will then drive the Husky through `/a201_0000/navigate_to_pose` using the configured waypoint route.

For file-based waypoint routes such as `config/warehouse_waypoints.yaml`, the Nav2 UGV driver now constrains the first selected waypoint to one that lies forward of the current UGV heading when possible, so the route does not start with an immediate backward segment. After that initial choice, the remaining file-waypoint order keeps the usual randomization behavior.

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
./run.sh rqt_image_view /dji0/camera0/image_raw
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
ros2 run lrs_halmstad simulator --ros-args -p uav_name:=dji0 -p camera_mode:=integrated_joint
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
- The attached gimbal path is now the default camera backend in Gazebo, while the ROS image topics remain `/dji0/camera0/*`.
- Attached-camera teleport spawns now use a non-static UAV model with a kinematic `base_link`, so the gimbal joints visibly actuate while the body remains pose-driven by `simulator`.
- `uav_simulator` boots **relaxed**: no gimbal joint commands are published until the first `update_pan` or `update_tilt` message arrives. The joint rests at its SDF-default until the follow stack or a manual command arms it. Motion is then rate-limited at `pan_rate_deg_s: 45.0` and `tilt_rate_deg_s: 60.0`, configurable in `config/run_follow_defaults.yaml` under the `uav_simulator` section (uses `yaml_param`, so the values **must** be present in the YAML — there is no hardcoded fallback). Pan was reduced from 90 °/s to 45 °/s to prevent large visible jumps during UAV yaw changes.
- Camera image bridge (`/image`, `/camera_info`, `/depth_image`) is now Gazebo→ROS only, reducing stale-frame buffering in image viewers.
- `camera_update_rate` spawn arg is now correctly wired through to the SDF (was declared but not passed to xacro). The spawn default is `10` Hz, which is the recommended value for WSL2 — Ogre2 rendering through WSLg is the primary RTF bottleneck. Increase only if your host can sustain RTF ≥ 1.0 at higher rates.
- `leader_estimator` now defaults to the actual simulated UAV pose topic `/dji0/pose`, not `/dji0/pose_cmd`.
- The active perception range mode is `auto` (depth → radio → const). Available explicit modes: `depth`, `radio`, `const`. Set via `range_mode` in `run_follow_defaults.yaml` under `leader_estimator`. The `ground` mode has been removed.
- When running with OMNeT++, `leader_estimator` subscribes to `/omnet/radio_distance` and uses it as the middle tier in `auto` mode (between depth and constant-range fallback). The raw FSPL-inverted Euclidean range is projected to horizontal distance using the current UAV altitude. Configure via `radio_range_topic` and `radio_range_timeout_s`. Set `radio_range_topic: ''` to disable entirely.
- Depth range sampling uses the **inner 50 % of the detection bounding box** (25 % margin on each edge) instead of a fixed 5 × 5 pixel patch. This scales correctly at all distances and avoids edge pixels that land on background or drone body. Requires at least 10 valid pixels (`depth_patch_min_valid_px`).
- YOLO `conf_threshold` is 0.3 for both `leader_detector` and `leader_tracker`. The previous value of 0.4 was above ByteTrack's internal `track_high_thresh: 0.25`, which caused borderline detections (0.30–0.39 confidence) to be dropped before the tracker could see them.
- YOLO inference runs **CPU-only** (`device: 'cpu'` in `run_follow_defaults.yaml`). GPU inference is not available on WSL2 with AMD hardware: ROCm requires `/dev/dri/renderD*` which WSL2 does not expose, and `torch-directml` is incompatible with YOLO OBB operations. Expected detection rate on CPU is 3–10 Hz depending on image resolution and model size.
- Image center correction (`image_center_correction_enable`) is **enabled** at `tick_hz: 10.0` (matched to the CPU YOLO detection rate). Running the tracker faster than the detection rate caused corrections to flicker on stale bounding boxes, producing gimbal jitter that disrupted ByteTrack. Keep `tick_hz` ≤ the expected YOLO rate, or disable correction if running at a higher tick rate without GPU inference.
- All world SDF files use consistent physics parameters: `max_step_size: 0.004`, `real_time_update_rate: 250` (targeting RTF ≤ 1.0).
- The Husky now uses `lidar2d_0` as its only active range sensor in this workspace. The old temporary `lidar3d_0` path is no longer part of the active bring-up.
- UGV mobility now runs in two supported modes: `ugv_mode:=nav2` sends sequential Nav2 `NavigateToPose` goals derived from the configured route, and `ugv_mode:=external` leaves UGV motion to an external Nav2 goal source.

### Reverting the current attached-gimbal kinematic workaround

If you need to back out the current visible-gimbal fix, revert these files together:

- `src/lrs_halmstad/launch/spawn_robot.launch.py`
  - restore teleport spawns to `model_static:=true`
  - remove the `base_link_kinematic_for_mode` wiring
- `src/lrs_halmstad/lrs_halmstad/generate_sdf.py`
  - remove the `base_link_kinematic` parameter and mapping
- `src/lrs_halmstad/xacro/lrs_model.xacro`
  - remove the `base_link_kinematic` arg and passthrough into `lrs_m100_macro`
- `src/lrs_halmstad/xacro/lrs_m100_base.sdf.xacro`
  - remove `<kinematic>${base_link_kinematic}</kinematic>`
  - if you want the exact old behavior, also remove `<gravity>false</gravity>` from `base_link`

Expected rollback result:
- the old static-body teleport behavior returns
- attached gimbal joints may stop producing visible camera motion in Gazebo again

## Contract checks

Base Gazebo + AMCL-derived UGV pose + UAV camera contract:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run lrs_halmstad contract_check orchard dji0
```

Include the UAV simulator topics:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
REQUIRE_UAV_ADAPTER=1 ros2 run lrs_halmstad contract_check orchard dji0
```

Include the odom-follow outputs as well:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
REQUIRE_UAV_ADAPTER=1 REQUIRE_FOLLOW_STACK=1 ros2 run lrs_halmstad contract_check orchard dji0
```

Include detector plus estimator topics when running the YOLO estimate path:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
REQUIRE_UAV_ADAPTER=1 REQUIRE_FOLLOW_STACK=1 REQUIRE_DETECTION=1 REQUIRE_ESTIMATOR=1 ros2 run lrs_halmstad contract_check orchard dji0
```

Useful overrides:

- `UGV_NAMESPACE=<name>` changes the default UGV namespace from `a201_0000`
- `REQUIRED_FLOW_TOPICS=<csv>` overrides the default flow check topic list
- `UGV_CMD_TOPICS=<csv>` overrides the cmd-vel subscriber check topics

Current defaults:

- base UGV odom flow check uses `/<ugv>/amcl_pose_odom`
- follow-stack checks no longer require `/<uav>/pose_cmd/odom`
- detector and estimator are checked separately with `REQUIRE_DETECTION=1` and `REQUIRE_ESTIMATOR=1`

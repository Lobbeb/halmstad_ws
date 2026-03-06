
# Stage 3 Checklist (Code-Confirmed)

This document answers the Stage 3 checklist by checking the current codebase (launch files, `leader_estimator.py`, `follow_uav.py`, and `scripts/run_round.sh`).

Scope:
- Current non-PX4 thesis stack (`clearpath_gz` + `lrs_halmstad`)
- Stage 3 perception/follow behavior in the current repo
- Both launch-based (`run_round_follow_motion.launch.py`) and script-based (`scripts/run_round.sh`) Stage 3 paths

## 1. Stage 3 Node Inventory

There are two relevant Stage 3 execution paths.

### A. Launch-based Stage 3 (`ros2 launch lrs_halmstad run_round_follow_motion.launch.py`)

Started by `src/lrs_halmstad/launch/run_round_follow_motion.launch.py`:

- `leader_estimator` (`lrs_halmstad` / `leader_estimator`) — conditional start
- `follow_uav` (`lrs_halmstad` / `follow_uav`)
- `ugv_motion_driver` (`lrs_halmstad` / `ugv_motion_driver`) — delayed via `TimerAction`

References:
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:62`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:83`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:100`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:118`

Estimator auto-start (`start_leader_estimator=auto`) starts the estimator when any of these are true:
- `leader_mode in (pose, estimate)`
- `leader_perception_enable=true`
- `yolo_weights` is non-empty

References:
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:9`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:19`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:20`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:22`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:24`

### B. Scripted Stage 3 (`scripts/run_round.sh`)

This is the standard recorded run path (rosbag + `meta.yaml` + logs).

Started by `scripts/run_round.sh` (for `CONDITION=follow`):

- `leader_estimator_<run_suffix>` via `ros2 run lrs_halmstad leader_estimator` (conditional)
- `follow_uav_<run_suffix>` via `ros2 run lrs_halmstad follow_uav`
- inline Python `rclpy` UGV driver node named `run_round_ugv_driver`
- `ros2 bag record` background process

References:
- `scripts/run_round.sh:429`
- `scripts/run_round.sh:445`
- `scripts/run_round.sh:483`
- `scripts/run_round.sh:695`
- `scripts/run_round.sh:412`

### C. Camera source prerequisite (spawn path)

`spawn_uavs.launch.py` starts a `ros_gz_bridge` camera bridge in `uav_mode:=physics` to expose `/dji*/camera0/image_raw` and `/camera_info`.

References:
- `src/lrs_halmstad/launch/spawn_uavs.launch.py:130`
- `src/lrs_halmstad/launch/spawn_uavs.launch.py:134`
- `src/lrs_halmstad/launch/spawn_uavs.launch.py:135`
- `src/lrs_halmstad/launch/spawn_uavs.launch.py:142`

## 2. YOLO Pipeline Contract

### Inputs (`leader_estimator`)

`leader_estimator` subscribes to:
- image topic: `sensor_msgs/msg/Image`
- camera info topic: `sensor_msgs/msg/CameraInfo`
- optional depth topic: `sensor_msgs/msg/Image` (if configured)
- UAV pose topic: `geometry_msgs/msg/PoseStamped`

Code references:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:317`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:318`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:320`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:323`

Default topics in code:
- image: `/<uav_name>/camera0/image_raw`
- camera_info: `/<uav_name>/camera0/camera_info`
- uav_pose: `/<uav_name>/pose_cmd`

Code references:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:159`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:160`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:162`

Launch defaults match this:
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:51`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:52`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:54`

### Detections output topic/message type

There is currently **no ROS detections output topic**.

- Detections are internal only (`Detection2D` dataclass)
- The node does **not** publish `vision_msgs` (or any detections ROS message)

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:50`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:502`

### Leader estimate and status outputs

Published outputs:
- `/coord/leader_estimate` — `geometry_msgs/msg/PoseStamped`
- `/coord/leader_estimate_status` — `std_msgs/msg/String`
- `/coord/events` (configurable `event_topic`) — `std_msgs/msg/String`

References:
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml:4`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml:5`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:324`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:325`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:326`

## 3. Leader Estimate Definition

### What `leader_estimate` contains

`leader_estimate` is a `geometry_msgs/msg/PoseStamped` with:
- `header.frame_id = "map"`
- `header.stamp = now` (estimator publish time)
- `pose.position.x/y/z`
- `pose.orientation` (yaw-only quaternion)

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:765`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:767`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:768`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:770`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:774`

### Frame / units / fields

- Frame: `map`
- Position units: meters
- Orientation: yaw (internally radians before quaternion conversion)
- `z` is set to `target_ground_z_m`

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:140`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:762`

### What it does **not** include

The published `PoseStamped` does **not** include:
- velocity
- confidence
- detection metadata (bbox/class)

Those are internal / exposed in `leader_estimate_status` (string) instead.

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:280`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:281`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:786`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:805`

### How it is computed from detections

High-level pipeline:
1. YOLO inference runs on latest image (`_pick_detection`)
2. One best detection is selected (optional class filter + continuity bias)
3. Pixel center `(u,v)` + camera intrinsics produce bearing
4. Range chosen by `range_mode` (`auto|depth|ground|const`)
5. Bearing/range smoothed (EMA)
6. World XY projected using UAV pose + camera offsets
7. XY smoothed
8. Yaw estimated from motion when possible
9. Tracker + latency compensation refine result
10. Publish `PoseStamped` on `/coord/leader_estimate`

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:502`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:527`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:688`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:693`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:702`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:721`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:735`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:741`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:751`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:969`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:972`

## 4. Failure Handling and Fallbacks

The estimator publishes a status string frequently and only publishes estimates when prerequisites / detections pass checks.

### A. Startup/readiness states (`on_status_tick`)

Status states from `_current_state()`:
- `waiting_for_image`
- `waiting_for_camera_info`
- `waiting_for_uav_pose`
- `stale_image`
- `running`

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:855`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:866`

### B. Stale prerequisites (camera/image/pose)

If camera info, image, or UAV pose is stale/missing during `on_tick()`:
- publishes status `state=STALE`
- no estimate published
- emits `ESTIMATE_STALE` on transition

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:876`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:884`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:888`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:890`

### C. Image decode failure

If image decode fails:
- status `DECODE_FAIL`
- no estimate
- emits `ESTIMATE_STALE` on transition

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:897`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:902`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:905`

### D. YOLO unavailable / disabled / failed

`yolo_error` can contain (examples):
- `ultralytics_not_installed`
- `yolo_weights_not_set`
- `file_not_found:<path>`
- `yolo_load_failed:<exception>`
- `infer_failed:<exception>`

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:351`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:353`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:356`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:359`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:367`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:515`

Status string includes `yolo=...` and `yolo_reason=...`.
No estimate is published if there is no detection and no hold estimate (`YOLO_DISABLED` branch).

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:795`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:796`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:925`

### E. No detections

If no usable detection is found:
- hold path: `DEBOUNCE_HOLD` or `HOLD` and publishes held estimate
- otherwise: `NO_DET` (YOLO ready) or `YOLO_DISABLED` (YOLO not ready)

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:909`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:915`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:916`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:925`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:833`

### F. Rejected estimates (sanity filters)

Rejection reasons include:
- `jump_xy`
- `speed`
- `bearing_jump`

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:837`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:845`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:847`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:851`

Behavior:
- may publish held estimate (`REJECT_DEBOUNCE_HOLD` / `REJECT_HOLD`)
- or publish no estimate (`REJECT`)

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:935`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:942`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:951`

### G. Reacquire debounce

When detections return, estimator can enter `REACQUIRE` before latching to `OK`.
It may publish held estimates during reacquire.

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:974`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:978`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:989`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:996`

### H. Range fallbacks (`range_mode`)

For `range_mode=auto`, fallback order is:
1. depth (if valid)
2. ground projection (if valid)
3. constant range

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:713`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:714`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:717`

### I. UAV pose bootstrap fallback

To avoid startup deadlock before first real UAV pose, a bootstrap UAV pose can be used and is later replaced by real `/pose_cmd` data.

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:265`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:267`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:445`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:447`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:425`

## 5. Rate / Latency Logging

### What is implemented now

Implemented in `leader_estimator`:
- `last_latency_ms = now - image_stamp` (stored internally)
- `latency_ms` included in `/coord/leader_estimate_status` (string)
- status publishing at 1 Hz
- estimator tick at `est_hz`

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:281`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:328`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:329`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:805`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:967`

### What is not implemented yet

Not implemented as explicit structured metrics:
- inference rate (Hz)
- detection rate
- rolling latency stats/histograms
- dedicated estimator metrics topic

`run_round.sh` logs estimator stdout to `leader_estimator.log`, but that is not structured performance telemetry.

Reference:
- `scripts/run_round.sh:476`

### Best hook points to add metrics

Suggested hook points in `leader_estimator.py`:
- `on_image()` for image input rate
- before/after `_pick_detection()` for inference timing
- `det is None` / `det is not None` branches in `on_tick()` for detection rate
- `_publish_estimate()` for estimate publish rate
- `on_status_tick()` for rolling metric summary output

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:384`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:502`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:909`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:765`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:866`

## 6. Stage 3 Run Configuration

### A. Launch-based Stage 3 (`run_round_follow_motion.launch.py`)

Main launch args controlling Stage 3:
- `params_file`
- `leader_mode`
- `leader_perception_enable`
- `start_leader_estimator` (`auto|true|false`)
- `leader_image_topic`
- `leader_camera_info_topic`
- `leader_depth_topic`
- `leader_uav_pose_topic`
- `yolo_weights`
- `yolo_device`

References:
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:35`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:42`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:43`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:44`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:51`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:52`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:53`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:54`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:55`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:56`

### B. YAML defaults (`run_round_follow_defaults.yaml`)

Shared defaults file:
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`

Defines defaults for:
- `leader_estimator` (`out_topic`, `status_topic`, `est_hz`, range/smoothing, `device`, etc.)
- `follow_uav` (`leader_input_type`, quality shaping, trajectory shaping, etc.)
- `ugv_motion_driver`

Examples:
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml:4`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml:5`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml:7`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml:24`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml:31`

### C. YOLO thresholds and selection (actual inference behavior)

Important: `run_round_follow_defaults.yaml` does **not** currently define the core YOLO thresholds. Those come from code defaults in `leader_estimator.py` unless overridden.

Code defaults:
- `conf_threshold = 0.25`
- `iou_threshold = 0.45`
- `imgsz = 640`
- `target_class_id` / `target_class_name` optional filters

References:
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:107`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:108`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:109`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:110`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:111`

### D. Scripted Stage 3 (`scripts/run_round.sh`) env knobs

Key Stage 3 env vars:
- `LEADER_MODE` (default `odom`)
- `LEADER_PERCEPTION_ENABLE` (default `false`)
- `YOLO_WEIGHTS`
- `YOLO_DEVICE`
- `LEADER_IMAGE_TOPIC`
- `LEADER_CAMERA_INFO_TOPIC`
- `LEADER_DEPTH_TOPIC`
- `LEADER_UAV_POSE_TOPIC`
- many estimator tuning vars (`LEADER_EST_*`)
- many follow shaping vars (`FOLLOW_*`)

References:
- `scripts/run_round.sh:40`
- `scripts/run_round.sh:41`
- `scripts/run_round.sh:49`
- `scripts/run_round.sh:50`
- `scripts/run_round.sh:51`
- `scripts/run_round.sh:52`
- `scripts/run_round.sh:53`
- `scripts/run_round.sh:54`
- `scripts/run_round.sh:55`
- `scripts/run_round.sh:64`

#### YOLO model choice (`v5n` / `v5s`)

- The actual model used is selected by the `YOLO_WEIGHTS` file path loaded by `leader_estimator`.
- `YOLO_VARIANT` in `run_round.sh` is a metadata label inferred from the weights filename (`v5n`, `v5s`, `custom`) and written to `meta.yaml`.
- `YOLO_VARIANT` does **not** directly change inference behavior in the node.

References:
- `scripts/run_round.sh:150`
- `scripts/run_round.sh:152`
- `scripts/run_round.sh:153`
- `scripts/run_round.sh:154`
- `scripts/run_round.sh:156`
- `scripts/run_round.sh:318`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:171`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py:363`

## 7. Rosbag Topic Set for Stage 3

### Where defined

In the standard scripted Stage 3 run, the bag topic list is defined as the `TOPICS` array in `scripts/run_round.sh`.

Reference:
- `scripts/run_round.sh:269`

### Base topics recorded (standard follow run)

Always included:
- `/clock`
- `$UGV_ODOM_TOPIC`
- `/a201_0000/platform/odom/filtered`
- `/a201_0000/platform/cmd_vel`
- `$UGV_CMD_VEL_TOPIC`
- `/a201_0000/tf`
- `/a201_0000/tf_static`
- `$EVENT_TOPIC`
- `/${UAV}/pose_cmd`
- `/coord/leader_estimate`
- `/coord/leader_estimate_status`
- `/coord/follow_dist_cmd`
- `/coord/follow_tracking_error_cmd`

References:
- `scripts/run_round.sh:268`
- `scripts/run_round.sh:270`
- `scripts/run_round.sh:279`
- `scripts/run_round.sh:280`
- `scripts/run_round.sh:281`
- `scripts/run_round.sh:282`

### Conditional additions

If `EVENT_TOPIC != /coord/events`:
- `/coord/events` is also recorded

References:
- `scripts/run_round.sh:285`
- `scripts/run_round.sh:287`

If `WITH_CAMERAS=true` (`--with-cameras`):
- `/${UAV}/camera0/image_raw`
- `/${UAV}/camera0/camera_info`
- optional `$LEADER_DEPTH_TOPIC`
- UGV color/depth topics

References:
- `scripts/run_round.sh:290`
- `scripts/run_round.sh:291`
- `scripts/run_round.sh:292`
- `scripts/run_round.sh:293`
- `scripts/run_round.sh:296`
- `scripts/run_round.sh:299`

### Rosbag record call

`ros2 bag record` is started with QoS overrides from `config/rosbag_qos.yaml`.

References:
- `scripts/run_round.sh:19`
- `scripts/run_round.sh:412`
- `scripts/run_round.sh:413`
- `scripts/run_round.sh:414`

Note:
- `run_round_follow_motion.launch.py` itself does **not** start rosbag recording.
- The topic list above applies to the script-based Stage 3 run (`scripts/run_round.sh`).

## 8. What Is Actually Used for Control Today

### Default control path: odom-based follow

`follow_uav` is odom-driven by default.

Defaults:
- code default: `leader_input_type = "odom"`
- YAML default: `leader_input_type: odom`
- launch default: `leader_mode:=odom` (passed to `follow_uav`)
- script default: `LEADER_MODE=odom`

References:
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:98`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:156`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml:31`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:42`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:93`
- `scripts/run_round.sh:40`
- `scripts/run_round.sh:489`

### Can `follow_uav` consume `leader_estimate` for steering?

Yes.

- `leader_input_type=pose` makes `follow_uav` subscribe to `leader_pose_topic` (`/coord/leader_estimate`)
- `leader_input_type=estimate` is accepted as alias and normalized to `pose`

References:
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:100`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:160`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:161`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:297`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:306`

### How to toggle control source

#### Launch path (`run_round_follow_motion.launch.py`)

- `leader_mode:=odom` (default; authoritative today)
- `leader_mode:=pose` or `leader_mode:=estimate` (uses `/coord/leader_estimate`)
- `leader_perception_enable:=true` can start estimator while follow still uses odom (`leader_mode:=odom`)

References:
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:42`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:43`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py:93`

#### Script path (`scripts/run_round.sh`)

- `LEADER_MODE=odom|pose|estimate`
- `LEADER_PERCEPTION_ENABLE=true` starts estimator in perception-only mode when `LEADER_MODE=odom`

References:
- `scripts/run_round.sh:40`
- `scripts/run_round.sh:41`
- `scripts/run_round.sh:429`
- `scripts/run_round.sh:432`
- `scripts/run_round.sh:434`
- `scripts/run_round.sh:489`

### How `leader_estimate_status` affects control

`follow_uav` subscribes to `leader_estimate_status` in all modes, but quality shaping / state machine logic is only active in `pose` mode.

- quality scaling disabled unless `leader_input_type == "pose"`
- follow state-machine debounce only in `pose` mode
- command shaping / hold behavior from quality only in `pose` mode

References:
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:313`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:412`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:462`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:487`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:581`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py:873`

## Practical summary (today)

- Control is **odom-only by default**.
- `leader_estimate` steering is implemented and can be enabled (`pose` / `estimate` mode), but is not the default.
- Stage 3 perception can run in perception-only/smoke mode while `follow_uav` still uses odom for control.

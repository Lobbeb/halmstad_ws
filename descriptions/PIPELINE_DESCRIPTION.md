# UAV–UGV Follow System — Research Narrative and Pipeline Description

*Structured for thesis writing. Follows the development chronology from simulation setup to visual follow.*

---

## 1. Problem Statement

The goal of this work is to develop a system in which an Unmanned Aerial Vehicle (UAV) autonomously follows a ground vehicle (UGV) in an indoor warehouse environment, maintaining a desired spatial separation and keeping the UGV continuously in its camera's field of view — without relying on GPS or external localisation infrastructure.

The development followed an iterative path: first establishing a simulation baseline with known ground-truth position, then introducing realistic wireless network effects, then replacing ground-truth control with vision-based detection, and finally closing the full perception-to-actuation loop with a visual follow pipeline.

---

## 2. Simulation Environment Setup

### 2.1 Gazebo Harmonic and ROS 2 Jazzy

The simulation environment uses **Gazebo Harmonic** for physics and rendering, integrated with **ROS 2 Jazzy** for all runtime communication, node management, and sensor data. The warehouse world is a closed indoor environment with shelving, walls, and a textured floor, designed to be representative of a realistic logistics scenario.

The UAV is a simulated DJI M100-class quadrotor equipped with an RGB-D camera mounted on a two-axis pan-tilt gimbal. The UGV is a Clearpath Husky differential-drive ground robot. Both are spawned using custom `xacro`/SDF model descriptions and managed through ROS 2 launch files.

Sensor data flows from Gazebo to ROS 2 through the `ros_gz_bridge` package. All simulation time is driven by Gazebo's internal clock, and all ROS 2 nodes use `use_sim_time: true` to stay synchronised.

The full simulation stack is orchestrated through a `tmux` session manager that starts and sequences five stages:

| Stage | Description |
| ----- | ----------- |
| `gazebo` | Gazebo Harmonic physics + rendering |
| `spawn` | UAV body + camera/gimbal model spawner |
| `localization` | AMCL localisation against a pre-built warehouse map |
| `nav2` | Nav2 autonomous ground navigation stack |
| `follow` | Full ROS 2 follow pipeline |

Standard launch:

```bash
./run.sh tmux_1to1 warehouse
```

### 2.2 UGV Localisation and Navigation

The UGV localises using **Adaptive Monte Carlo Localisation (AMCL)** against a known 2-D warehouse map, providing a reliable world-frame pose estimate via `/<ugv>/amcl_pose` (a `PoseWithCovarianceStamped` in the `map` frame). Because many downstream components require an `Odometry` message, a lightweight converter node (`pose_cov_to_odom`) republishes the AMCL pose as `/<ugv>/amcl_pose_odom`.

The UGV navigates autonomously using **ROS 2 Nav2**, following waypoints loaded from a YAML file. Waypoint order is randomised each run, and the first waypoint is constrained to lie in front of the UGV's initial heading to prevent immediate U-turns.

### 2.3 Coordinate Convention

All follow and estimation computations use the **world (`map`) frame** in **East-North-Up (ENU)** convention — X East, Y North, Z Up — with angles measured from the East axis, increasing counter-clockwise. The helper `wrap_pi(a) = ((a + π) mod 2π) − π` is used throughout to keep angles in `[−π, π]`.

---

## 3. OMNeT++ Network Simulation Integration

### 3.1 Motivation

In a real deployment the UAV and a ground control station communicate over a wireless channel. To simulate realistic network behaviour (signal attenuation, range-dependent link quality, packet loss), the Gazebo simulation was coupled with **OMNeT++**, a discrete-event network simulator.

### 3.2 Architecture

Three ROS 2 nodes form the bridge between ROS 2/Gazebo and OMNeT++:

**`gazebo_pose_tcp_bridge`** acts as a TCP server (default port 5555). OMNeT's `GazeboPositionScheduler` polls it at each simulation step by sending a `GET` command. The bridge responds with a snapshot of all tracked entity positions formatted as:

```
N  name x y z yaw  name x y z yaw …
```

It subscribes to:

- `/<ugv>/platform/odom` — UGV raw simulator odometry for OMNeT coupling
- `/<uav>/pose/odom` — UAV world-frame position (converted from `/<uav>/pose` by `omnet_uav_pose_to_odom`)

OMNeT applies a coordinate mapping (+50 m offset on both axes, Y-axis flip) internally to convert from ENU to its own grid.

**`omnet_metrics_bridge`** is the return path: a TCP client (port 5556) that connects to OMNeT's `OmnetMetricsServer` and republishes live network metrics as ROS 2 topics, including `/omnet/radio_distance` — the Free-Space Path Loss (FSPL)-derived Euclidean distance between UAV and UGV.

This radio distance is later used by the pose estimator as a fallback range source when depth sensing is unreliable.

Launch with OMNeT enabled:

```bash
./run.sh tmux_1to1 warehouse start_omnet_bridge:=true omnet_bridge_port:=5555
```

---

## 4. Early Follow — Odometry-Based Control

### 4.1 Baseline: Follow with Ground-Truth Position

Before any vision was introduced, the follow pipeline was validated using the UGV's AMCL odometry directly. The node `follow_uav_odom.py` subscribes to `/<ugv>/amcl_pose_odom` (known ground-truth position in simulation) and to the UAV's own pose `/<uav>/pose`, computing a target position that maintains a desired 3-D separation and heading.

This baseline confirmed:

- The UAV simulator's position stepping and yaw rate limiting
- The camera gimbal control loop (pan/tilt)
- The Nav2 UGV waypoint driver
- The overall tmux orchestration and ROS 2 topic wiring

The control law is a direct geometric computation: the desired follow point is placed at distance `d` behind the UGV along its current heading, at altitude `z_follow`. The UAV's commanded position steps toward this point with rate limiting to avoid abrupt motion.

### 4.2 UAV Simulator

`simulator.py` bridges the high-level ENU position command to Gazebo's `SetEntityPose` service. Each 20 Hz tick it:

1. Steps the UAV XY position toward the commanded target (scaled step, not instantaneous teleport)
2. Rate-limits the yaw rotation
3. Holds altitude at the commanded Z
4. Applies pan/tilt commands to the gimbal joints via joint position control
5. Publishes `/<uav>/pose` for downstream consumption

This design means the flight controller interface is the same whether the commands come from the odometry follow node, the visual follow pipeline, or a manual override — only the command source changes.

---

## 5. Dataset Creation

### 5.1 Motivation

To train a YOLO model that can detect the Clearpath Husky UGV from a downward-facing airborne camera, a labelled image dataset was required. Because the UGV's appearance from altitude is distinctive (top-down or near-top-down view of a rectangular chassis) and real-world data collection at altitude is impractical, the dataset was collected entirely in simulation.

### 5.2 Capture Process

The simulation dataset capture node (`sim_dataset_capture.py`) saves synchronised pairs of:

- RGB images from `/<uav>/camera0/image_raw`
- UGV world-frame pose from `/<ugv>/amcl_pose_odom`
- UAV world-frame pose from `/<uav>/pose`
- Camera intrinsics from `/<uav>/camera0/camera_info`

The UAV is flown to multiple positions and altitudes around the UGV while the UGV navigates its waypoint path, ensuring coverage of diverse viewing angles, distances, and lighting conditions within the warehouse.

### 5.3 Label Generation

**Axis-Aligned Bounding Boxes (detection labels):** For each saved frame, the UGV's 3-D bounding box corners are projected into the image using the pinhole camera model and the known relative pose, producing pixel-space bounding boxes in YOLO format (`labels/`).

**Oriented Bounding Boxes (OBB labels):** The dataset was later extended with OBB labels (`labels_obb/`) by projecting the physical length and width of the UGV chassis onto the image plane and computing the minimum-area enclosing rotated rectangle. OBB labels encode both the spatial extent and the heading of the vehicle as it appears in the image.

Dataset location: `datasets/warehouse_v1/run1/` and `datasets/warehouse_v1/run2/`.

---

## 6. YOLO Detection

### 6.1 Model Training

Two YOLO model variants were trained on the warehouse dataset:

**Plain detection model** (`warehouse_v1-v2-yolo26n.pt`): A standard YOLO detection head predicting axis-aligned bounding boxes. Used for straightforward presence and location detection.

**OBB model** (`warehouse-v1-yolo26n-obb.pt`): A YOLO variant with an oriented bounding box head, trained on the OBB labels. Returns four corner points and a rotation angle alongside the standard confidence and class predictions. This is the default in the current pipeline.

Both models are based on a lightweight YOLO architecture (yolo26n) appropriate for the low-compute inference rate required on an embedded UAV processor.

### 6.2 LeaderDetector

`leader_detector.py` runs YOLO inference at 60 Hz on the camera image stream. For each frame it selects the highest-confidence detection and publishes:

```
/coord/leader_detection        — bounding box centroid (u, v), dimensions, OBB corners, confidence
/coord/leader_detection_status — human-readable diagnostics
```

### 6.3 LeaderTracker

An alternative front-end, `leader_tracker.py`, wraps the YOLO model with an Ultralytics multi-object tracker (BoT-SORT or ByteTrack by default). The tracker assigns persistent track IDs across frames, making the output more robust to momentary occlusion and detection gaps. It publishes the same message format as the detector, so all downstream nodes are agnostic to which front-end is active.

Selection is controlled at launch time:

```bash
./run.sh tmux_1to1 warehouse mode:=yolo tracker:=true
```

---

## 7. Pose Estimation from Detection

### 7.1 Motivation

A 2-D pixel bounding box identifies *where in the image* the UGV appears, but the follow pipeline requires a world-frame 3-D position to compute where the UAV should fly. The `LeaderEstimator` node converts the 2-D detection into a world-frame pose using camera geometry, depth sensing, and the OBB heading.

### 7.2 Camera Model

The camera uses a standard pinhole model with intrinsics `(fx, fy, cx, cy)` received from `camera_info`. The camera is mounted at a fixed body-frame offset from the UAV centre, transformed to world frame as:

```
cam_x = uav_x + off_x·cos(uav_yaw) − off_y·sin(uav_yaw)
cam_y = uav_y + off_x·sin(uav_yaw) + off_y·cos(uav_yaw)
```

### 7.3 Bearing Computation

The normalised horizontal image coordinate of the detection centroid `(u, v)` gives the horizontal bearing from the camera:

```
x_n           = (u − cx) / fx
bearing_world = uav_yaw + cam_yaw_offset − atan2(x_n, 1.0)
```

### 7.4 Depth Range Estimation

A patch of the depth image is extracted from the inner 50% of the detection bounding box to exclude background pixels near the edges. Valid pixels within `[depth_min_m, depth_max_m]` are collected and the Nth percentile is used:

```
range_m = percentile(depth_patch, depth_percentile)
```

The default `depth_percentile = 10` deliberately biases toward the *near side* of the depth distribution. At high elevation angles (~45° tilt, 7 m altitude), the lower portion of the bounding box can look past the UGV roof onto the warehouse floor behind it. Using the median (50th percentile) would return floor depth (~11 m) rather than UGV depth (~7–8 m); the 10th percentile selects UGV surface pixels instead.

This parameter is declared with `dynamic_typing=True` so it can be tuned live:

```bash
ros2 param set /leader_estimator depth_percentile 5.0
```

**Range mode fallback chain** (`range_mode: auto`):

1. `depth` — depth image percentile (preferred)
2. `radio` — OMNeT FSPL distance → converted to horizontal range via `sqrt(r² − Δz²)`
3. `const` — constant seeded from the configured target follow distance

### 7.5 World-Frame Position

```
ugv_x = cam_x + range_m · cos(bearing_world)
ugv_y = cam_y + range_m · sin(bearing_world)
```

### 7.6 UGV Heading from OBB

The four OBB corner points are paired to find the midpoints of the two longest sides. Each midpoint is projected to the ground plane using the pinhole model and the known UAV altitude:

```
x_n      = (u − cx) / fx
y_n      = (v − cy) / fy
bearing  = cam_yaw_offset − atan2(x_n, 1.0)
elev     = cam_pitch_offset + atan2(−y_n, sqrt(1 + x_n²))
range_h  = cam_z / tan(π/2 + elev)
gx       = cam_x + range_h · cos(uav_yaw + bearing)
gy       = cam_y + range_h · sin(uav_yaw + bearing)
```

The heading `atan2(gy2 − gy1, gx2 − gx1)` between the two ground midpoints gives the UGV's longitudinal axis, with the 180° ambiguity resolved by comparing to the previous estimate.

The estimator publishes:

- `/coord/leader_estimate` — PoseStamped (world frame)
- `/coord/leader_selected_target` — structured detection state

---

## 8. Visual Follow Pipeline

With a world-frame UGV estimate available, the final stage closes the loop from perception to actuation using a multi-stage motion planning chain.

### 8.1 Pipeline Overview

```
/coord/leader_detection
        │
        ▼
  LeaderEstimator          → /coord/leader_estimate
        │                     /coord/leader_selected_target
        ▼
  SelectedTargetFilter     → /coord/leader_selected_target_filtered
        │
        ▼
  VisualTargetEstimator    → /coord/leader_visual_target_estimate
        │
        ▼
  FollowPointGenerator     → /coord/leader_follow_point
        │
        ▼
  FollowPointPlanner       → /coord/leader_planned_target
        │
        ▼
  VisualActuationBridge    → /<uav>/psdk_ros2/…ENUposition_yaw  (Joy)
        │
        ▼
   UAV flight controller (simulator)

CameraTracker ← /coord/leader_estimate
        ├─ /<uav>/update_pan
        └─ /<uav>/update_tilt
```

### 8.2 SelectedTargetFilter

Applies temporal trust and continuity checks to suppress transient false positives, low-confidence detections, and geometrically implausible position jumps. A configurable short-gap hold prevents downstream nodes from reacting to single missed frames.

### 8.3 VisualTargetEstimator

Lifts the filtered 2-D detection state into a camera-relative bearing-and-range representation and estimates the target's short-horizon motion. This enriched state is the primary input to the follow point generator.

### 8.4 FollowPointGenerator

Converts the world-frame UGV estimate into a *follow point* — the world-frame position the UAV should occupy.

**Horizontal distance** for the desired 3-D separation `d`:

```
h = sqrt(d² − Δz²)     where Δz = uav_altitude − ugv_z
```

**Velocity estimate (EMA):**

```
vx = (1 − α_v)·vx + α_v·(Δx / dt)     α_v = 0.35
```

**Lookahead (T = 0.25 s):**

```
pred_x = target_x + vx · T
pred_y = target_y + vy · T
```

**Follow direction priority:** OBB heading → motion heading → held heading → anchor direction → view line.

**Follow point:**

```
follow_x = pred_x − h · dir_x
follow_y = pred_y − h · dir_y
```

Smoothed with EMA (α = 0.55) and a hard jump clamp.

### 8.5 FollowPointPlanner

Smooths the raw follow point into a planned target at 20 Hz. An *adaptive alpha* reduces the interpolation weight when the follow point is approximately stationary (cached upstream estimate):

```
α_eff = α_xy · stale_alpha_scale    if ||fp − fp_prev|| < 0.05 m
```

Per-tick step clamps:

- XY: `max_planned_step_m = 0.40 m/tick` (→ max 8 m/s at 20 Hz)
- Yaw: `max_planned_yaw_step_rad = 0.12 rad/tick`

### 8.6 VisualActuationBridge

Converts the planned target into a bounded per-tick ENU position command:

```
Δxy = plan_xy − uav_xy
if ||Δxy|| > 0.25 m:
    Δxy = Δxy · (0.25 / ||Δxy||)

Δyaw = clamp(wrap_pi(plan_yaw − uav_yaw), ±0.20 rad)
```

Published as `sensor_msgs/Joy` with axes `[x, y, z, yaw]` to the PSDK-compatible topic. The fixed altitude `z = 7.0 m` is commanded unconditionally; Z is not derived from the estimator to avoid noise amplification.

### 8.7 Camera Gimbal Control

`camera_tracker.py` runs in parallel and continuously aims the gimbal at the estimated UGV position.

**Pan** = residual heading from UAV body yaw to look-at direction:

```
pan_cmd = degrees(wrap_pi(atan2(dy, dx) − uav_yaw))
```

**Tilt** = depression angle to look-at point:

```
tilt_cmd = −degrees(atan2(vertical_drop, horiz_dist))   ∈ [−89°, +89°]
```

**Image-centre correction** (visual servo, applied on top):

```
err_x_deg          = degrees(atan2((u − cx) / fx, 1.0))
pan_correction     = clamp(−K_pan · err_x_deg,  ±3°)
tilt_correction    = clamp(−K_tilt · err_y_deg, ±12°)
```

This inner feedback loop corrects calibration offsets and the latency-induced lag of the outer estimation chain.

### 8.8 Key Default Parameters

| Parameter | Value | Stage |
| --------- | ----- | ----- |
| `follow_distance_m` | 7.0 m | FollowPointGenerator |
| `follow_altitude_m` | 7.0 m | FollowPointGenerator |
| `lookahead_horizon_s` | 0.25 s | FollowPointGenerator |
| `depth_percentile` | 10 | LeaderEstimator |
| `range_mode` | auto | LeaderEstimator |
| `max_planned_step_m` | 0.40 m/tick | FollowPointPlanner |
| `max_planned_yaw_step_rad` | 0.12 rad/tick | FollowPointPlanner |
| `max_xy_step_m` | 0.25 m/tick | VisualActuationBridge |
| `max_yaw_step_rad` | 0.20 rad/tick | VisualActuationBridge |
| `fixed_z_m` | 7.0 m | VisualActuationBridge |
| `pan_image_center_gain` | 0.25 | CameraTracker |
| `tilt_image_center_gain` | 0.50 | CameraTracker |

---

## 9. Summary of Development Stages

| Stage | Key outcome |
| ----- | ----------- |
| Gazebo + ROS 2 setup | Simulation environment, UAV/UGV models, launch orchestration |
| OMNeT++ integration | Realistic wireless range feedback via FSPL radio distance |
| Odometry follow | Validated follow geometry, simulator, camera gimbal |
| Dataset creation | Labelled RGB + OBB images from simulation, diverse viewing angles |
| YOLO training | Plain detection model + OBB model for heading estimation |
| YOLO detection node | LeaderDetector / LeaderTracker, 60 Hz inference |
| Pose estimation | Bearing + depth → world-frame UGV pose, OBB heading extraction |
| Visual follow | Closed-loop pipeline: filter → estimator → planner → bridge → UAV |

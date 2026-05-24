# Visual Follow Pipeline — Mathematics

All computation runs in the **world (odom) frame**: ENU convention, Z up.
Angles are in **radians** unless noted otherwise.
`wrap_pi(a) = ((a + π) mod 2π) - π`

---

## Pipeline Overview

```
camera image
    │
    ▼
LeaderTracker          YOLO OBB detection (pixel bbox + corners)
    │  /coord/leader_detection_status
    ▼
LeaderEstimator        pixel + depth → world-frame UGV pose
    │  /coord/leader_estimate          (PoseStamped)
    │  /coord/leader_visual_target_estimate  (Odometry, bearing+range)
    ▼
SelectedTargetFilter   temporal filter on raw detections
    ▼
VisualTargetEstimator  produces the Odometry used by FollowPointGenerator
    │  /coord/leader_visual_target_estimate
    ▼
FollowPointGenerator   UGV estimate → UAV follow point
    │  /coord/leader_follow_point      (PoseStamped)
    ▼
FollowPointPlanner     rate-limited, smoothed planned target
    │  /coord/leader_planned_target    (PoseStamped)
    ▼
VisualActuationBridge  planned target → bounded position command
    │  /<uav>/psdk_ros2/…ENUposition_yaw  (Joy [x, y, z, yaw])
    ▼
UAV flight controller

                    ↑ /coord/leader_estimate
CameraTracker  ─────┘  gimbal pan + tilt commands
```

---

## 1 — LeaderEstimator

Converts a 2-D detection in image space into a world-frame UGV pose using camera intrinsics, depth, and OBB heading.

### 1.1 Camera model

Pinhole intrinsics received from `camera_info`:

```
fx, fy   — focal lengths (pixels)
cx, cy   — principal point (pixels)
```

Camera position in world frame (static offset from UAV body):

```
cam_x = uav_x + cam_x_offset·cos(uav_yaw) - cam_y_offset·sin(uav_yaw)
cam_y = uav_y + cam_x_offset·sin(uav_yaw) + cam_y_offset·cos(uav_yaw)
```

### 1.2 Bearing from detection centroid

The detection centroid `(u, v)` gives the horizontal angular offset from the camera optical axis:

```
x_n = (u - cx) / fx          # normalised horizontal pixel offset

bearing_cam = cam_yaw_offset - atan2(x_n, 1.0)
```

`cam_yaw_offset` is a calibrated static rotation between UAV body and camera boresight.
The world-frame bearing from the camera to the target is:

```
bearing_world = uav_yaw + bearing_cam
```

### 1.3 Depth range

A patch of the depth image is sampled at the inner 50% of the detection bounding box (shrunk by 25% on each side) to avoid edges and background.
Invalid pixels and values outside `[depth_min_m, depth_max_m]` are discarded.
The range used is the **median** of the remaining valid pixels:

```
range_m = median(depth_patch)   (mode: depth | radio | const | auto)
```

In `auto` mode: depth is preferred, radio FSPL distance is the fallback, constant range is the last resort.

### 1.4 World-frame UGV position

```
ugv_x = cam_x + range_m · cos(bearing_world)
ugv_y = cam_y + range_m · sin(bearing_world)
```

### 1.5 UGV heading from OBB

The OBB detector returns 4 corner points in pixel space.
The two midpoints of the **longest axis** are projected onto the ground plane using the pinhole model and the known UAV altitude:

```
ground_point(u, v):
    x_n = (u - cx) / fx
    y_n = (v - cy) / fy
    bearing  = cam_yaw_offset - atan2(x_n, 1.0)
    elev     = cam_pitch_offset + atan2(-y_n, sqrt(1 + x_n²))
    range_horiz = cam_z / tan(π/2 + elev)    # project to ground
    gx = cam_x + range_horiz · cos(uav_yaw + bearing)
    gy = cam_y + range_horiz · sin(uav_yaw + bearing)
```

Heading from the two midpoints `(gx1,gy1)` and `(gx2,gy2)`:

```
yaw_a = atan2(gy2 - gy1, gx2 - gx1)
yaw_b = wrap_pi(yaw_a + π)      # 180° ambiguity
```

The candidate closest to the previous heading estimate is kept.

---

## 2 — FollowPointGenerator

Converts the world-frame UGV estimate into a world-frame follow point for the UAV.

### 2.1 Horizontal distance for a Euclidean follow distance

The desired 3-D distance between UAV and UGV is `follow_distance_m = d`.
Given the vertical separation `Δz = follow_altitude_m - ugv_z`, the required horizontal distance is:

```
h = sqrt(d² - Δz²)    (returns 0 if |Δz| ≥ d)
```

### 2.2 UGV velocity (EMA)

```
meas_vx = (target_x - prev_target_x) / dt
meas_vy = (target_y - prev_target_y) / dt
     (clamped to max_target_meas_speed_mps)

vx = (1 - α_v)·vx + α_v·meas_vx      α_v = target_velocity_alpha = 0.35
vy = (1 - α_v)·vy + α_v·meas_vy
```

### 2.3 Lookahead prediction

```
pred_x = target_x + vx · T_horizon
pred_y = target_y + vy · T_horizon     T_horizon = lookahead_horizon_s = 0.25 s
```

### 2.4 Follow direction (heading)

Priority order:
1. **target_pose_heading** — UGV yaw from OBB (smoothed by EMA, α = `heading_dir_alpha = 0.25`)
2. **motion_heading** — normalised velocity `(vx, vy)` when `speed ≥ min_target_speed_mps`
3. **held_heading** — last valid heading within `heading_hold_timeout_s`
4. **anchor_heading** — direction from last published follow point to current predicted target
5. **view_line** — direction from UAV to predicted target (fallback)

### 2.5 Follow point

```
perp = (-dir_y, dir_x)         # perpendicular to heading (left of travel)

raw_follow_x = pred_x - h · dir_x + lateral_offset_m · perp_x
raw_follow_y = pred_y - h · dir_y + lateral_offset_m · perp_y
```

Smoothing (EMA + jump clamp):

```
smooth_x = (1 - α_p)·prev_x + α_p·raw_x      α_p = point_alpha = 0.55
smooth_y = (1 - α_p)·prev_y + α_p·raw_y

if ||smooth - prev|| > max_jump:
    smooth = prev + (smooth - prev) · max_jump / ||smooth - prev||
```

### 2.6 UAV yaw command

The UAV should face the predicted UGV position.
If the camera has a body offset, the yaw is solved iteratively (3 Newton steps):

```
yaw = atan2(pred_y - follow_y, pred_x - follow_x)
for i in range(3):
    cam_x, cam_y = follow_x + cam_offset rotated by yaw
    yaw = atan2(pred_y - cam_y, pred_x - cam_x)

follow_yaw = wrap_pi(yaw)
```

---

## 3 — FollowPointPlanner

Rate-limits and smooths the follow point into a planned target at 20 Hz.

### 3.1 Adaptive interpolation alpha

If the follow point has barely moved since the last tick (upstream estimate is cached):

```
fp_delta = ||fp - fp_prev||
α_eff = α_xy                          if fp_delta ≥ stale_fp_thresh_m (0.05 m)
α_eff = α_xy · stale_alpha_scale       if fp_delta < stale_fp_thresh_m
                                        (stale_alpha_scale = 0.3)
```

### 3.2 XY interpolation + step clamp

```
interp = prev_planned + α_eff · (fp - prev_planned)

if ||interp - prev_planned|| > max_planned_step_m:
    interp = prev_planned + (interp - prev_planned) · max_planned_step_m / ||…||
```

`max_planned_step_m = 0.40 m` per tick (at 20 Hz → max ~8 m/s).

### 3.3 Yaw interpolation + step clamp

```
Δyaw = wrap_pi(fp_yaw - prev_yaw)
step_yaw = clamp(α_yaw · Δyaw, ±max_planned_yaw_step_rad)

planned_yaw = wrap_pi(prev_yaw + step_yaw)
```

`α_yaw = 0.55`, `max_planned_yaw_step_rad = 0.12 rad/tick` (~1.38°/tick).

### 3.4 Altitude interpolation

```
step_z = clamp(raw_z - planned_z, ±max_planned_z_step_m)
planned_z = planned_z + α_z · step_z      α_z = 0.45
```

---

## 4 — VisualActuationBridge

Converts the planned target into bounded per-tick UAV position commands.

### 4.1 Position step

```
Δx = plan_x - uav_x
Δy = plan_y - uav_y
dist_xy = sqrt(Δx² + Δy²)

if dist_xy > max_xy_step_m:
    Δx, Δy = Δx, Δy · (max_xy_step_m / dist_xy)

target_x = uav_x + Δx
target_y = uav_y + Δy
```

`max_xy_step_m = 0.25 m` per tick.

### 4.2 Yaw step

```
Δyaw = wrap_pi(plan_yaw - uav_yaw) · yaw_cmd_sign
step_yaw = clamp(Δyaw, ±max_yaw_step_rad)

target_yaw = wrap_pi(uav_yaw + step_yaw)
```

`max_yaw_step_rad = 0.20 rad/tick`.

### 4.3 Altitude

```
target_z = fixed_z_m = 7.0 m    (use_current_altitude = false)
```

When `use_current_altitude = true` the bridge would instead pass `uav_z` which ignores the planner altitude — **this was a bug that has been fixed**.

### 4.4 Output

Published as `sensor_msgs/Joy` with axes `[target_x, target_y, target_z, target_yaw]`
and mirrored as a `PoseStamped` on `/<uav>/pose_cmd`.

---

## 5 — CameraTracker

Controls the gimbal pan and tilt to keep the UGV look-at point in frame.

### 5.1 Look-at point

An optional body-frame offset on the UGV (e.g. pointing at the top of the vehicle) is rotated into world frame by the UGV heading:

```
look_at_x = ugv_x + look_offset_x·cos(ugv_yaw) - look_offset_y·sin(ugv_yaw)
look_at_y = ugv_y + look_offset_x·sin(ugv_yaw) + look_offset_y·cos(ugv_yaw)
look_at_z = ugv_z + look_offset_z
```

### 5.2 Camera position

```
cam_x = uav_x + cam_x_offset·cos(uav_yaw) - cam_y_offset·sin(uav_yaw)
cam_y = uav_y + cam_x_offset·sin(uav_yaw) + cam_y_offset·cos(uav_yaw)
cam_z = uav_z - cam_z_offset
```

### 5.3 Pan command

The pan is the residual yaw from UAV heading to look-at direction:

```
dx = look_at_x - cam_x
dy = look_at_y - cam_y
target_yaw = atan2(dy, dx)
pan_cmd_deg = degrees(wrap_pi(target_yaw - uav_yaw))
```

`camera_pan_sign` and `camera_yaw_offset_deg` are applied by the simulator when converting this to the rendered camera pose.

### 5.4 Tilt command

```
horiz_dist = sqrt(dx² + dy²)
vertical_drop = max(0, cam_z - look_at_z)
tilt_cmd_deg = -degrees(atan2(vertical_drop, horiz_dist))
               clamped to [-89°, +89°]
```

Negative = camera tilts down. At `horiz_dist = 7 m`, `vertical_drop = 7 m` → `tilt ≈ -45°`.

### 5.5 Image centre correction (visual servo loop)

When `image_center_correction_enable = true` and a fresh detection is available, the pixel error from centre is converted to an angular additive correction applied on top of the model-based command:

```
err_x_deg = degrees(atan2((u_det - cx) / fx, 1.0))
err_y_deg = degrees(atan2((v_det - cy) / fy, 1.0))

pan_correction_deg  = clamp(-K_pan  · err_x_deg,  ±pan_image_center_max_deg)
tilt_correction_deg = clamp(-K_tilt · err_y_deg, ±tilt_image_center_max_deg)

pan_cmd_final  = pan_cmd  + pan_correction_deg
tilt_cmd_final = tilt_cmd + tilt_correction_deg
```

`K_pan = pan_image_center_gain`, `K_tilt = tilt_image_center_gain` (default 0.5).
This loop closes a visual feedback path that corrects calibration errors and latency-induced lag.

### 5.6 Reacquire fallback

When the estimator goes `NO_DET` and `actual_pose_reacquire_enable = true`, the camera falls back to the UGV odom pose (`leader_actual_pose_topic`) to keep the gimbal aimed at the last known UGV location while waiting for detections to recover.

---

## Key Parameters (defaults)

| Parameter | Value | Where |
|---|---|---|
| `follow_distance_m` | 7.0 m | FollowPointGenerator, FollowPointPlanner |
| `follow_altitude_m` | 7.0 m | FollowPointGenerator |
| `lookahead_horizon_s` | 0.25 s | FollowPointGenerator |
| `point_alpha` | 0.55 | FollowPointGenerator (follow point EMA) |
| `heading_dir_alpha` | 0.25 | FollowPointGenerator (heading EMA) |
| `target_velocity_alpha` | 0.35 | FollowPointGenerator (velocity EMA) |
| `xy_alpha` | 0.45 | FollowPointPlanner |
| `yaw_alpha` | 0.55 | FollowPointPlanner |
| `max_planned_step_m` | 0.40 m | FollowPointPlanner |
| `max_planned_yaw_step_rad` | 0.12 rad | FollowPointPlanner |
| `stale_alpha_scale` | 0.3 | FollowPointPlanner |
| `max_xy_step_m` | 0.25 m | VisualActuationBridge |
| `max_yaw_step_rad` | 0.20 rad | VisualActuationBridge |
| `fixed_z_m` | 7.0 m | VisualActuationBridge |

# Follow UAV Odom — Mathematics

Odometry-based UAV follow controller (`follow_uav_odom.py`).
All computation runs in the **world (`map`) frame**: ENU convention, Z up.
Angles are in **radians** unless noted otherwise.
`wrap_pi(a) = ((a + π) mod 2π) - π`

---

## Pipeline Overview

```
/<ugv>/amcl_pose_odom   (Odometry)
    │
    ▼
 on_leader_odom         extracts UGV world pose, body-velocity → world heading
    │
    ▼
 on_tick (20 Hz)
    ├─ distance geometry    d_target → xy_target, z_target
    ├─ anchor target        ugv_pos − xy_target · heading_dir  (clamped)
    ├─ XY command           rate/speed limited step toward anchor
    ├─ Z command            rate-limited altitude step
    └─ yaw command          solve camera to look-at, rate-limited step
    │
    ▼
/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw  (Joy [x, y, z, yaw])
/<uav>/pose_cmd                                            (PoseStamped)
/<uav>/pose_cmd/odom                                       (Odometry)
```

Input source: ground-truth AMCL odometry directly (no vision path).
Used for: odometry-follow validation, baseline testing, dataset collection.

---

## 1 — Leader Pose from Odometry

The UGV heading is derived from its body-frame velocity rotated to world frame:

```
vx_world = cos(yaw)·vx_body - sin(yaw)·vy_body
vy_world = sin(yaw)·vx_body + cos(yaw)·vy_body
speed    = hypot(vx_world, vy_world)

ugv_follow_heading = atan2(vy_world, vx_world)    if speed > 0.05 m/s
                   = yaw                           otherwise (pose yaw fallback)
```

This follow heading is used as the anchor direction for placing the follow point.

---

## 2 — Distance Geometry

### 2.1 Euclidean standoff decomposition

The desired 3-D standoff is `d_target`. Given start altitude `uav_start_z` and the constraint that `z ≥ z_min`, the nominal altitude target is:

```
z_cap          = leader_z + sqrt(max(0, d_target² - xy_min²))
z_cap          = min(z_cap, z_max)         if z_max > 0
z_target       = clamp(uav_start_z, z_min, z_cap)
```

The corresponding horizontal distance:

```
xy_target = sqrt(max(0, d_target² - (z_target - leader_z)²))
xy_target = clamp(xy_target, xy_min, xy_cap)
```

where `xy_cap = sqrt(max(0, d_target² - (z_min - leader_z)²))` (bounded by z_min constraint).

If the clamping changes `xy_target`, z is re-derived to stay on the d_target sphere:

```
z_target = leader_z + sqrt(max(0, d_target² - xy_target²))
z_target = clamp(z_target, z_min, z_cap)
```

### 2.2 Look-at target

An optional body-frame offset on the UGV is rotated to world frame to offset where the camera points:

```
offset_x, offset_y = rotate_body(leader_look_target_x_m, leader_look_target_y_m, ugv_yaw)
look_at_x = ugv_x + offset_x
look_at_y = ugv_y + offset_y
```

---

## 3 — Anchor Target

The UAV should sit `xy_target` behind the UGV along its current heading direction:

```
xt_raw = ugv_x - xy_target · cos(ugv_follow_heading)
yt_raw = ugv_y - xy_target · sin(ugv_follow_heading)
```

**Leash clamp** — keeps the anchor within `xy_anchor_max` of the UGV regardless of geometry:

```
d = hypot(xt_raw - ugv_x, yt_raw - ugv_y)
if d > xy_anchor_max:
    xt = ugv_x + (xt_raw - ugv_x) · xy_anchor_max / d
    yt = ugv_y + (yt_raw - ugv_y) · xy_anchor_max / d
```

**Yaw at anchor** — solved iteratively (3 Newton steps) so the camera boresight points at the look-at target despite the camera body offset:

```
yaw = atan2(look_at_y - yt, look_at_x - xt)
for i in range(3):
    cam_x = xt + cam_x_offset·cos(yaw) - cam_y_offset·sin(yaw)
    cam_y = yt + cam_x_offset·sin(yaw) + cam_y_offset·cos(yaw)
    yaw   = atan2(look_at_y - cam_y, look_at_x - cam_x)

anchor_yaw = yaw                  (follow_yaw=true)
anchor_yaw = current_uav.yaw      (follow_yaw=false)
```

---

## 4 — XY Command

The XY command is computed from the current UAV pose to the anchor target,
rate-limited to `follow_speed_mps`:

```
anchor_error = hypot(uav_x - xt, uav_y - yt)
effective_speed = min(follow_speed_mps, follow_speed_gain · anchor_error)
```

Two command modes (`uav_xy_command_mode`):

**`direct`** (default) — one proportional step toward the anchor:

```
step = effective_speed / tick_hz
d    = hypot(xt - uav_x, yt - uav_y)
if d <= step:
    cmd_x, cmd_y = xt, yt
else:
    cmd_x = uav_x + (xt - uav_x) · step / d
    cmd_y = uav_y + (yt - uav_y) · step / d
```

**`controller_step`** — same formula but `step` is scaled by `uav_xy_command_step_scale` (< 1), producing smaller incremental steps for smoother motion:

```
step = effective_speed · uav_xy_command_step_scale / tick_hz
```

The control pose source switches between the last commanded pose and the last actual Gazebo feedback pose: the commanded state is preferred when it is newer than the most recent odometry (prevents oscillation during command latency).

---

## 5 — Z Command

Rate-limited altitude step toward `z_target`:

```
error          = z_target - current_z
speed_per_s    = min(follow_z_speed_mps, follow_z_speed_gain · |error|)
step_limit     = speed_per_s / tick_hz

if |error| <= 0.01 m:
    z_cmd = z_target                     (snap)
elif |error| <= step_limit:
    z_cmd = z_target
else:
    z_cmd = current_z + sign(error) · step_limit
```

---

## 6 — Yaw Command

Yaw tracks the look-at direction with a configurable rate limit:

```
yaw_target = solve_yaw_to_target(cmd_x, cmd_y, look_at_x, look_at_y, cam_x_offset, cam_y_offset)
yaw_error  = wrap_pi(yaw_target - current_yaw)

yaw_rate_rad_s = min(follow_yaw_rate_rad_s, follow_yaw_rate_gain · |yaw_error|)
yaw_step_limit = yaw_rate_rad_s / tick_hz
```

If the error exceeds the step limit the command advances by one step; otherwise it snaps to target:

```
if yaw_step_limit > 0 and |yaw_error| > yaw_step_limit:
    yaw_cmd = wrap_pi(current_yaw + sign(yaw_error) · yaw_step_limit)
    mode    = "rate_limited"
else:
    yaw_cmd = yaw_target
    mode    = "direct_target"
```

Note: `yaw_cmd` is always solved from the **new XY command position** `(cmd_x, cmd_y)`,
not from the current actual pose, so large XY steps on sharp turns do not leave a stale yaw.

---

## 7 — Output

All three values are published together:

```
Joy.axes = [cmd_x, cmd_y, z_cmd, yaw_cmd]     → /<uav>/psdk_ros2/…ENUposition_yaw
PoseStamped(cmd_x, cmd_y, z_cmd, yaw_cmd)     → /<uav>/pose_cmd
Odometry(cmd_x, cmd_y, z_cmd, yaw_cmd)        → /<uav>/pose_cmd/odom
```

---

## Key Parameters (defaults)

| Parameter | Value | Role |
|---|---|---|
| `tick_hz` | 20 Hz | Control loop rate |
| `d_target` | 8.0 m | 3-D Euclidean standoff sphere radius |
| `xy_min` | 2.0 m | Minimum horizontal follow distance |
| `z_min` | 5.0 m | Minimum UAV altitude |
| `xy_anchor_max` | 10.0 m | Leash radius around the UGV |
| `follow_speed_mps` | 3.0 m/s | Hard cap on XY command speed |
| `follow_speed_gain` | 1.2 | Proportional gain: anchor error → speed |
| `uav_xy_command_mode` | `direct` | Step mode: `direct` or `controller_step` |
| `uav_xy_command_step_scale` | 0.6 | Step scale for `controller_step` mode |
| `follow_z_speed_mps` | 3.0 m/s | Hard cap on Z command speed |
| `follow_z_speed_gain` | 1.0 | Proportional gain: altitude error → speed |
| `follow_yaw_rate_rad_s` | 0.6 rad/s | Maximum yaw rate |
| `follow_yaw_rate_gain` | 1.2 | Proportional gain: yaw error → rate |
| `pose_timeout_s` | 3.0 s | Staleness threshold for leader/UAV pose |
| `min_cmd_period_s` | 0.05 s | Minimum interval between commands |
| `camera_x_offset_m` | 0.0 m | Camera forward offset from UAV body |
| `camera_y_offset_m` | 0.0 m | Camera lateral offset from UAV body |

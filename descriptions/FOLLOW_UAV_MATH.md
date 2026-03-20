# Follow UAV (Estimate Mode) — Mathematics

Estimate-based UAV follow controller (`follow_uav.py`).
All computation runs in the **world (`map`) frame**: ENU convention, Z up.
Angles are in **radians** unless noted otherwise.
`wrap_pi(a) = ((a + π) mod 2π) - π`

This node consumes the world-frame leader pose published by `LeaderEstimator`
(`/coord/leader_estimate`, a `PoseStamped`) and is therefore the top-level
motion controller for the full visual follow pipeline. It shares the same
distance geometry and anchor-target logic as `follow_uav_odom` but adds:

- Quality-scaled motion from the estimator status stream
- A debounced follow-state machine (TRACK / REACQUIRE / HOLD / DEGRADED)
- Trajectory shaping with velocity and acceleration limits
- Leader-heading resolution from multiple sources
- A search-climb behaviour when detection is lost

---

## Pipeline Overview

```
/coord/leader_estimate          (PoseStamped, from LeaderEstimator)
/coord/leader_estimate_status   (String, estimator state/confidence/latency)
    │
    ▼
 on_leader_pose         stores UGV world pose; feeds motion model
 on_leader_status       parses state, conf, latency_ms, heading_src
    │
    ▼
 on_tick (20 Hz)
    ├─ quality_scale_from_status    → quality_scale ∈ [0,1]
    ├─ classify/update follow state → TRACK | REACQUIRE | HOLD | DEGRADED
    ├─ distance geometry            → xy_target, z_target
    ├─ anchor target                → ugv_pos − xy_target · heading_dir  (clamped)
    ├─ trajectory shaper            → velocity/accel-limited step toward anchor
    ├─ XY command (quality-scaled speed)
    ├─ Z command (rate-limited, search-scaled when searching)
    └─ yaw command (accel-limited rate)
    │
    ▼
/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw  (Joy [x, y, z, yaw])
/<uav>/pose_cmd                                            (PoseStamped)
/<uav>/pose_cmd/odom                                       (Odometry)
```

---

## 1 — Distance Geometry

Identical to `follow_uav_odom`. Given `d_target`, `z_min`, `xy_min`, and `uav_start_z`:

```
z_cap    = leader_z + sqrt(max(0, d_target² - xy_min²))
z_cap    = min(z_cap, z_max)                   if z_max > 0
z_target = clamp(uav_start_z, z_min, z_cap)

xy_target = sqrt(max(0, d_target² - (z_target - leader_z)²))
xy_target = clamp(xy_target, xy_min, xy_cap)
xy_cap    = sqrt(max(0, d_target² - (z_min - leader_z)²))
```

If the xy clamp is active, z is re-derived to stay on the `d_target` sphere.

**Search mode** uses the same formula but forces `z_target` to its effective cap:

```
z_target_search  = effective_z_cap(leader_z)
xy_target_search = horizontal_distance_for_euclidean(d_target, z_target_search - leader_z)
```

This climbs the UAV until the UGV re-enters camera range.

---

## 2 — Leader Heading Resolution

The anchor target requires a heading direction for the UGV. Four sources are
tried in descending priority order:

| Priority | Source | When active |
|---|---|---|
| 1 | `actual_heading` | `leader_actual_heading_enable=true` and fresh AMCL odom |
| 2 | `motion_heading` | `estimate_heading_from_motion_enable=true` and `speed ≥ estimate_heading_min_speed_mps` |
| 3 | `estimate_<src>` | OBB/motion heading from estimator `heading_src` field (not `none`/`fallback`) |
| 4 | `motion_heading_fallback` | Estimator heading unavailable but motion heading available |
| 5 | `estimate_pose_yaw` | Quaternion yaw from `/coord/leader_estimate` (lowest trust) |

**Motion heading model** — updated only when leader pose gap `dt ≥ 1.0 s` (coarse interval to avoid jitter):

```
dx = ugv_x_new - ugv_x_prev
dy = ugv_y_new - ugv_y_prev
speed = hypot(dx, dy) / dt

if speed ≥ estimate_heading_min_speed_mps:
    leader_motion_heading_yaw = atan2(dy, dx)
```

Gaps larger than `estimate_heading_max_dt_s` reset the motion model without updating heading.

---

## 3 — Look-at Target

Same as `follow_uav_odom`: a body-frame offset on the UGV is rotated to world frame:

```
offset_x, offset_y = rotate_body(leader_look_target_x_m, leader_look_target_y_m, ugv_heading_yaw)
look_at_x = ugv_x + offset_x
look_at_y = ugv_y + offset_y
```

---

## 4 — Anchor Target

The desired follow point sits `xy_target` behind the UGV along its resolved heading:

```
leader_for_follow = Pose2D(ugv_x, ugv_y, resolved_heading)

xt_raw = ugv_x - xy_target · cos(resolved_heading)
yt_raw = ugv_y - xy_target · sin(resolved_heading)
```

**Leash clamp** (radius `xy_anchor_max` around the UGV):

```
d = hypot(xt_raw - ugv_x, yt_raw - ugv_y)
if d > xy_anchor_max:
    xt = ugv_x + (xt_raw - ugv_x) · xy_anchor_max / d
    yt = ugv_y + (yt_raw - ugv_y) · xy_anchor_max / d
```

**Yaw at anchor** — iterative solve (3 Newton steps) accounting for camera body offset:

```
yaw = atan2(look_at_y - yt, look_at_x - xt)
for i in range(3):
    cam_x = xt + cam_x_offset·cos(yaw) - cam_y_offset·sin(yaw)
    cam_y = yt + cam_x_offset·sin(yaw) + cam_y_offset·cos(yaw)
    yaw   = atan2(look_at_y - cam_y, look_at_x - cam_x)
```

---

## 5 — Quality Scaling

The estimator publishes `state`, `conf`, and `latency_ms` on the status topic.
These are combined into a scalar `quality_scale ∈ [0, 1]` that gates motion:

```
hard_bad = {STALE, DECODE_FAIL, NO_DET, YOLO_DISABLED, waiting_*}
soft     = {HOLD, DEBOUNCE_HOLD, REJECT_HOLD, REJECT_DEBOUNCE_HOLD, REACQUIRE}

quality_scale = 0.0           if state ∈ hard_bad

q_conf = (conf - quality_conf_min) / (quality_conf_good - quality_conf_min)
q_conf = clamp(q_conf, 0, 1)           # if conf is known
q_conf = 0.6                           # soft state without conf
q_conf = 1.0                           # no conf available, normal state

q_lat  = 1 / (1 + latency_ms / quality_latency_ref_ms)
q_lat  = 1.0                           # if latency unavailable

q_state = 0.45                         if state ∈ soft
          1.0                          otherwise

q = q_conf · q_lat · q_state
q = max(quality_min_motion_scale, q)   if q > 0
quality_scale = clamp(q, 0, 1)
```

When `quality_scale_enable=false` all scaling is bypassed (`quality_scale = 1.0`).

---

## 6 — Follow State Machine

A debounced state machine categorises each tick as one of:

| State | Condition |
|---|---|
| `TRACK` | quality_scale > quality_hold_step_scale and state not hard-bad |
| `REACQUIRE` | Soft estimator state |
| `DEGRADED` | Hard-bad estimator state but quality > hold threshold |
| `HOLD` | quality_scale ≤ quality_hold_step_scale |

With `state_machine_enable=true`, transitions require `state_debounce_ticks`
consecutive identical desired states before committing:

```
if desired_state == previous_desired:
    good_ticks += 1   (TRACK/REACQUIRE)  or  bad_ticks += 1
    if ticks ≥ state_debounce_ticks:
        follow_state = desired_state
else:
    reset counters, start accumulating toward new state
```

**State effects on control:**

- `HOLD` — XY command is frozen to current UAV position; yaw is frozen.
- `HOLD` → search — if search conditions met, XY and Z motion resume at search scales.
- `TRACK`/`REACQUIRE` — normal control with quality-scaled speed.

---

## 7 — Trajectory Shaping (optional, `traj_enable=true`)

Before the XY command step, the anchor target is filtered through a velocity- and acceleration-limited virtual trajectory. This prevents the UAV from making abrupt position jumps when the estimated leader position steps discontinuously.

When `traj_rel_frame_enable=true`, the error and velocity are computed in the leader-relative frame (rotated by `resolved_heading`) and then rotated back to world frame. This keeps trajectory dynamics consistent even when the UGV turns.

```
cur_rel = to_leader_frame(leader_for_follow, uav_xy)
des_rel = to_leader_frame(leader_for_follow, anchor_xy)
error   = des_rel - cur_rel

q_scale = max(quality_min_motion_scale, quality_scale)    if quality_scale_enable
q_scale = max(q_scale, search_min_motion_scale)            if search_active

v_desired = traj_pos_gain · error
v_desired = clamp_magnitude(v_desired, traj_max_speed_mps · q_scale)

if traj_max_accel_mps2 > 0:
    delta_v = clamp_magnitude(v_desired - traj_vel, traj_max_accel_mps2 · dt)
    traj_vel += delta_v
else:
    traj_vel = v_desired

step = traj_vel · dt
```

**Overshoot guard** — if the step would cross the desired target, snap to target and zero velocity:

```
if (error · (error - step)) < 0:
    nxt = des_rel
    traj_vel = (0, 0)
else:
    nxt = cur_rel + step
```

**Yaw-jump reset** — when `|wrap_pi(leader_yaw - prev_leader_yaw)| > traj_reset_on_yaw_jump_rad`, trajectory velocity is zeroed to prevent the wrong-frame carry-over.

---

## 8 — XY Command

After trajectory shaping (or directly from anchor if shaping disabled):

```
anchor_error   = hypot(uav_xy - anchor_xy)
effective_speed = min(follow_speed_mps, follow_speed_gain · anchor_error)

if quality_scale_enable:
    effective_speed *= max(0, min(1, motion_quality_scale))

step = effective_speed / tick_hz   (direct mode)
step = effective_speed · uav_xy_command_step_scale / tick_hz   (controller_step mode)

d = hypot(target - uav_xy)
if d <= step:
    cmd_xy = target
else:
    cmd_xy = uav_xy + (target - uav_xy) · step / d
```

---

## 9 — Z Command

Standard rate-limited step toward `z_target`.
During a search climb, speed and gain are scaled down:

```
speed_scale = search_z_speed_scale   if search_climbing else 1.0
gain_scale  = search_z_gain_scale    if search_climbing else 1.0

error       = z_target - current_z
speed_per_s = min(follow_z_speed_mps · speed_scale, follow_z_speed_gain · gain_scale · |error|)
step_limit  = speed_per_s / tick_hz

z_cmd = current_z + sign(error) · min(|error|, step_limit)
```

When follow_state is HOLD (and not search_active), `z_cmd = current_z` (altitude frozen).

---

## 10 — Yaw Command

Yaw uses an **acceleration-limited rate** (unlike `follow_uav_odom` which uses direct rate limiting):

```
yaw_target = solve_yaw_to_target(cmd_x, cmd_y, look_at_x, look_at_y, cam_x_offset, cam_y_offset)
yaw_error  = wrap_pi(yaw_target - current_yaw)

yaw_rate_target = min(follow_yaw_rate_rad_s, follow_yaw_rate_gain · |yaw_error|)
if quality_scale_enable:
    yaw_rate_target *= max(0, min(1, quality_scale))
yaw_rate_target = copysign(yaw_rate_target, yaw_error)

if follow_yaw_accel_rad_s2 > 0:
    max_delta = follow_yaw_accel_rad_s2 · control_dt
    rate_delta = clamp(yaw_rate_target - yaw_rate_cmd, ±max_delta)
    yaw_rate_cmd += rate_delta
else:
    yaw_rate_cmd = yaw_rate_target

yaw_step_limit = |yaw_rate_cmd| · control_dt
```

Priority table for the yaw command issued each tick:

| Condition | yaw_cmd | mode |
|---|---|---|
| search_active | current_yaw | `search_hold` |
| follow_state == HOLD | current_yaw | `hold_state` |
| REJECT_HOLD / REJECT_DEBOUNCE_HOLD | current_yaw | `status_hold` |
| follow_yaw = false | current_yaw | `follow_yaw_disabled` |
| |yaw_error| > yaw_step_limit | current_yaw + sign · step | `accel_rate_limited` |
| else | yaw_target | `direct_target` |

---

## Key Parameters (defaults)

| Parameter | Value | Role |
|---|---|---|
| `tick_hz` | 20 Hz | Control loop rate |
| `d_target` | 8.0 m | 3-D Euclidean standoff sphere radius |
| `xy_min` | 2.0 m | Minimum horizontal standoff |
| `z_min` | 5.0 m | Minimum UAV altitude |
| `xy_anchor_max` | 10.0 m | Leash radius around UGV |
| `follow_speed_mps` | 3.0 m/s | XY speed cap |
| `follow_speed_gain` | 1.2 | Proportional gain: anchor error → speed |
| `follow_z_speed_mps` | 3.0 m/s | Altitude speed cap |
| `follow_z_speed_gain` | 1.0 | Proportional gain: altitude error → speed |
| `follow_yaw_rate_rad_s` | 0.6 rad/s | Maximum yaw rate |
| `follow_yaw_rate_gain` | 1.2 | Proportional gain: yaw error → rate |
| `follow_yaw_accel_rad_s2` | 6.0 rad/s² | Maximum yaw acceleration |
| `pose_timeout_s` | 3.0 s | Leader/UAV pose staleness threshold |
| `state_debounce_ticks` | 3 | Consecutive ticks before state transition |
| `quality_conf_min` | 0.4 | Confidence floor for motion scaling |
| `quality_conf_good` | 0.7 | Confidence treated as full quality |
| `quality_latency_ref_ms` | 300 ms | Reference latency for quality decay |
| `quality_min_motion_scale` | 0.25 | Minimum motion scale after gating |
| `quality_hold_step_scale` | 0.08 | Quality threshold below which HOLD triggers |
| `traj_enable` | true | Trajectory shaping active |
| `traj_rel_frame_enable` | true | Shape trajectory in leader-relative frame |
| `traj_pos_gain` | 1.0 | Proportional gain: position error → velocity |
| `traj_max_speed_mps` | 5.0 m/s | Max trajectory speed |
| `traj_max_accel_mps2` | 2.5 m/s² | Max trajectory acceleration |
| `traj_reset_on_yaw_jump_rad` | 0.8 rad | Heading jump threshold for traj reset |
| `estimate_heading_from_motion_enable` | true | Use motion heading for anchor direction |
| `estimate_heading_min_speed_mps` | 0.5 m/s | Minimum speed for motion heading trust |
| `search_delay_s` | 1.5 s | Wait after losing target before search climb |
| `search_z_speed_scale` | 0.25 | Vertical speed scale during search |
| `search_z_gain_scale` | 0.05 | Altitude gain scale during search |
| `search_min_motion_scale` | 0.05 | Minimum lateral motion scale during search |

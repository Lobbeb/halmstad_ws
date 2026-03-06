# IMPLEMENTATION LOG

## 2026-03-06 - Vertical search overshoot guard (local tilt span in reacquire)

### What changed
Added a focused control safeguard to prevent large unnecessary vertical panning during structured reacquire.

1. `follow_uav.py`
- Added new parameter:
  - `reacquire_tilt_local_span_deg`
- Reacquire sequence now stores loss-time tilt center (`reacquire_seq_tilt_center_deg`).
- `TILT` phase sweep is now limited to a local band around that center:
  - `[tilt_center - reacquire_tilt_local_span_deg, tilt_center + reacquire_tilt_local_span_deg]`
  - still clamped to global gimbal tilt limits.
- This replaces prior full-range tilt sweeping that could run from near min to near max tilt and destabilize retention.

2. Launch/config wiring
- Added launch arg + follow-node mapping:
  - `reacquire_tilt_local_span_deg`
- Added defaults:
  - `run_round_follow_defaults.yaml`: `10.0`
  - `run_round_follow_observe_strict.yaml`: `8.0`

### Why this change was needed
Runtime feedback showed major improvement overall, but occasional unnecessary vertical pan/overshoot could still “break” tracking despite target being in frame.

Root cause: reacquire `TILT` phase previously used broad/global sweep bounds, which was too aggressive for short-loss/noisy windows.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Static/build (completed):
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py
python3 -m py_compile src/lrs_halmstad/launch/run_round_follow_motion.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```

2. Launch arg visibility (completed):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py --show-args | rg reacquire_tilt_local_span_deg
```

3. Runtime expectation:
- During short loss/reacquire, tilt stays near loss-time angle instead of sweeping full vertical range.
- Fewer sudden “bad” vertical swings that previously broke lock.

### Files changed in this implementation block
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `src/lrs_halmstad/config/run_round_follow_observe_strict.yaml`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-06 - Proxy multiclass stability pass (multi-box lock + motorcycle class)

### What changed
Addressed the remaining proxy regression where selection became unstable when multiple allowed boxes appear in frame or multiple proxy boxes overlap the UGV.

1. `leader_estimator.py`
- Added sticky local lock parameters for proxy mode:
  - `proxy_lock_on_last_enable`
  - `proxy_lock_max_px`
  - `proxy_lock_min_conf`
  - `proxy_lock_dist_weight`
  - `proxy_lock_class_bonus`
- Updated target selection in `_pick_yolo_detection()`:
  - keep existing score-based global candidate selection,
  - then (in proxy mode, with recent tracking) apply a local lock chooser around last selected center to keep continuity through multi-box clutter,
  - still fall back to existing switch guard logic for large jumps / weak-confidence switches.
- Goal: reduce class/box hopping when UGV is simultaneously labeled as several COCO classes.

2. Proxy class defaults updated
- Added `motorcycle` to default proxy class allowlists in:
  - `run_round_follow_defaults.yaml`
  - `run_round_follow_observe_strict.yaml`

3. Estimator continuity tuning defaults updated (both profiles)
- `bbox_continuity_weight: 0.30`
- `bbox_continuity_class_bonus: 0.10`
- `bbox_continuity_max_px: 350.0`
- `proxy_lock_on_last_enable: true`
- `proxy_lock_max_px: 240.0`
- `proxy_lock_min_conf: 0.03`
- `proxy_lock_dist_weight: 0.50`
- `proxy_lock_class_bonus: 0.12`

### Why this change was needed
Runtime behavior improved, but target control still degraded when:
- many allowed proxy classes appeared in one frame, or
- several proxy boxes overlapped the UGV.

The detector was still seeing the target, but downstream selection jitter caused lock instability.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Static/build (completed):
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/leader_estimator.py
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py
python3 -m py_compile src/lrs_halmstad/launch/run_round_follow_motion.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```

2. Runtime expectation:
- with multi-box clutter, selected proxy target should switch less and stay committed to one local UGV box longer.
- debug image should still show multiple boxes, but selection should be less erratic.

### Files changed in this implementation block
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `src/lrs_halmstad/config/run_round_follow_observe_strict.yaml`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-06 - First-lock sequencing gate + camera-first reacquire assist staging

### What changed
Implemented the approved control-sequencing pass so startup/loss behavior is gated by lock readiness instead of immediate search/motion competition.

1. `follow_uav.py`
- Added new parameters:
  - `reacquire_first_lock_gate_enable`
  - `reacquire_first_lock_min_visible_frames`
  - `reacquire_relock_min_visible_frames`
  - `reacquire_yaw_assist_delay_s`
  - `reacquire_xy_assist_delay_s`
- Added visibility streak tracking from `sel_visible` status and a first-lock latch:
  - emits `REACQUIRE_FIRST_LOCK_READY` when first lock is stable.
- Enforced first-lock gate in structured reacquire:
  - no PAN/TILT/YAW search starts before first stable lock window.
- Staged assist behavior during loss:
  - camera-first by default (`hold_xy=1`, `hold_yaw=1`),
  - yaw assist only after `reacquire_yaw_assist_delay_s`,
  - XY assist only after `reacquire_xy_assist_delay_s`,
  - fallback HOLD behavior remains safety terminal path.
- Added relock hysteresis:
  - `sel_visible=1` returns `TRACK` only after `reacquire_relock_min_visible_frames`; otherwise stays `REACQUIRE`.
- Tightened motion gating:
  - `REACQUIRE` now held by default, and motion is only released when structured-search assist is explicitly armed.
- Extended `/coord/follow_debug_status` with:
  - `first_lock_ready`
  - `sel_visible_streak`
  - `relock_ready`
  - `reacquire_yaw_assist`
  - `reacquire_xy_assist`

2. Launch/config wiring
- Added launch args and follow-node parameter wiring in:
  - `src/lrs_halmstad/launch/run_round_follow_motion.launch.py`
- Added defaults in both profiles:
  - `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
  - `src/lrs_halmstad/config/run_round_follow_observe_strict.yaml`

### Why this change was needed
Observed behavior matched sequencing conflict:
- startup/loss could trigger search/motion too early,
- camera and UAV could diverge instead of lock-follow cooperation,
- reacquire oscillation then passive phases appeared in runs.

This pass makes first lock a prerequisite, keeps short-loss reacquire camera-first, and only escalates UAV assist with explicit persistence delays.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Syntax/build (completed):
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py
python3 -m py_compile src/lrs_halmstad/launch/run_round_follow_motion.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```

2. Launch args visible (completed):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py --show-args | \
  rg -e reacquire_first_lock_gate_enable \
     -e reacquire_first_lock_min_visible_frames \
     -e reacquire_relock_min_visible_frames \
     -e reacquire_yaw_assist_delay_s \
     -e reacquire_xy_assist_delay_s
```

### Files changed in this implementation block
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `src/lrs_halmstad/config/run_round_follow_observe_strict.yaml`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-06 - Regression fix: allow XY motion during active reacquire search in DEGRADED

### What changed
Applied a targeted fix to address spin-only / passive behavior in latest proxy runs.

1. `follow_uav.py`
- Updated motion gating so `DEGRADED` does **not** force `track_motion_hold` when structured reacquire is already actively searching (`PAN/TILT/YAW`) and not in fallback hold.
- Updated search hold application so active reacquire search in `DEGRADED` does not re-freeze XY (`reacquire_hold_xy` overridden to false in this specific case).
- Fallback behavior remains unchanged:
  - long-loss fallback hold (`reacquire_seq_fallback_hold`) is still terminal hold behavior.

### Why this change was needed
Regression analysis of `run_proxy_reacquire_holdloop` vs earlier better runs showed:
- search/camera activity present, but
- XY command movement collapsed to near-zero,
- behavior became mostly spin/search and then passive.

Most likely cause was interaction between:
- `DEGRADED -> track_motion_hold` gating and
- reacquire search hold path,
which froze translation during active search.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Syntax/build:
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```

2. Runtime evidence:
- During active loss with `search_phase=PAN|TILT|YAW`, `/coord/follow_debug_status` should no longer show persistent translation freeze from `DEGRADED` gating.
- Expect more nonzero `cmd_dx/cmd_dy` during active search windows (before fallback hold).

## 2026-03-06 - Reacquire HOLD made non-terminal (loop search, delayed true fallback)

### What changed
Implemented a targeted regression fix in the reacquire state machine so it does not freeze in terminal `HOLD` right after one PAN/TILT/YAW pass.

1. `follow_uav.py`
- Added new reacquire timing parameters:
  - `reacquire_hold_phase_s` (short HOLD dwell before restarting PAN)
  - `reacquire_fallback_hold_s` (long total search timeout before true fallback HOLD)
- Updated sequence behavior:
  - `PAN -> TILT -> YAW -> HOLD(dwell) -> PAN -> ...`
  - Only after total uninterrupted search age exceeds `reacquire_fallback_hold_s` does it enter true fallback hold.
- Added explicit fallback-hold events:
  - `REACQUIRE_SEQ_FALLBACK_HOLD_ENTER`
  - `REACQUIRE_SEQ_FALLBACK_HOLD_EXIT`
- Extended `/coord/follow_debug_status` with:
  - `reacquire_fallback_hold=0|1`

2. Launch/config wiring
- Added launch args + follow parameter wiring:
  - `reacquire_hold_phase_s`
  - `reacquire_fallback_hold_s`
- Added defaults in both profiles:
  - `config/run_round_follow_defaults.yaml`
  - `config/run_round_follow_observe_strict.yaml`

### Why this change was needed
Latest run analysis showed a concrete regression pattern:
- startup spazz reduced compared to earlier, but
- after first search cycle it entered `search_phase=HOLD` and stayed there,
- resulting in near-zero camera/UAV recovery motion.

Root cause was terminal HOLD behavior in reacquire sequence, not detector/proxy selection.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Syntax/build:
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py src/lrs_halmstad/launch/run_round_follow_motion.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```

2. Launch args visible:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py --show-args | \
  rg "reacquire_hold_phase_s|reacquire_fallback_hold_s|reacquire_"
```

3. Runtime evidence:
- During loss, `search_phase` should cycle (`PAN/TILT/YAW/HOLD/PAN`) instead of sticking in `HOLD` immediately.
- `/coord/follow_debug_status` should include `reacquire_fallback_hold`.
- `REACQUIRE_SEQ_FALLBACK_HOLD_ENTER` should only appear after long uninterrupted loss windows.

## 2026-03-06 - Reacquire commit window + startup guard (anti-spazz and anti-freeze)

### What changed
Implemented the next control-gating pass after structured search:

1. `follow_uav.py`
- Added **reacquire commit window** so short visibility hits are usable:
  - On `sel_visible=1`, start `reacquire_commit_s` grace window.
  - During grace, quality is floored (`reacquire_commit_quality_floor`) to prevent immediate HOLD freeze.
  - Search sequence does not re-enter during grace (`commit_grace`), reducing ping-pong behavior.
- Added **startup guard** for structured search:
  - `reacquire_startup_guard_enable=true` prevents PAN/TILT/YAW search before first estimator status is received.
  - Stops early startup “spazz” camera motion before perception is alive.
- Kept `sel_visible=0` as true-loss input for lock/search decisions outside commit grace.
- Extended `/coord/follow_debug_status` with:
  - `reacquire_commit=0|1`

2. Launch/config wiring
- Added launch args + follow parameter wiring:
  - `reacquire_commit_enable`
  - `reacquire_commit_s`
  - `reacquire_commit_quality_floor`
  - `reacquire_startup_guard_enable`
- Added defaults in both profiles:
  - `config/run_round_follow_defaults.yaml`
  - `config/run_round_follow_observe_strict.yaml`

### Why this change was needed
Latest run showed:
- startup search oscillation before stable perception,
- brief `sel_visible=1` moments,
- but follow stayed motion-held and quickly fell back to search/HOLD.

So the bottleneck moved to **transition gating** between loss/search and short reacquire hits.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Syntax/build:
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py src/lrs_halmstad/launch/run_round_follow_motion.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```

2. Launch args visible:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py --show-args | \
  rg "reacquire_commit|reacquire_startup_guard|reacquire_"
```

3. Runtime evidence:
- `/coord/follow_debug_status` should show `reacquire_commit=1` shortly after `sel_visible=1`.
- `search_phase` should remain `IDLE` during commit grace, then re-enter PAN/TILT/YAW only if lock is not recovered.

## 2026-03-06 - Structured reacquire pass (PAN->TILT->YAW->HOLD) + sel_visible loss gating

### What changed
Implemented the requested control-path fix pass focused on structured search/reacquire, without detector redesign:

1. `follow_uav.py`:
- Added strict loss gating from estimator status:
  - `sel_visible=0` is now treated as true target loss for camera-lock/search quality decisions.
  - This prevents periodic `state=running` heartbeats from being treated as valid target lock when no target is visible.
- Added structured reacquire sequence with explicit phases and events:
  - `PAN` (local search around last pan center)
  - `TILT` (vertical sweep)
  - `YAW` (wide/360-style yaw scan while sweeping pan)
  - `HOLD` fallback
- Sequence is active in both:
  - normal tracking ticks
  - stale hold ticks (`hold:pose_stale`) so search continues during dropout periods.
- Added events:
  - `REACQUIRE_SEQ_ENTER`
  - `REACQUIRE_SEQ_PHASE:<PAN|TILT|YAW|HOLD>`
  - `REACQUIRE_SEQ_EXIT:<reason>`
- Extended `/coord/follow_debug_status` with:
  - `search_phase=...`
- Added configurable params:
  - `reacquire_sequence_enable`
  - `reacquire_pan_phase_s`
  - `reacquire_tilt_phase_s`
  - `reacquire_yaw_phase_s`
  - `reacquire_pan_local_span_deg`
  - `reacquire_pan_rate_deg_s`
  - `reacquire_tilt_rate_deg_s`
  - `reacquire_yaw_rate_deg_s`
  - `reacquire_hold_after_search`
  - `reacquire_search_hold_xy`
  - `reacquire_search_hold_yaw_before_yaw_phase`

2. Launch/config wiring:
- Exposed new reacquire args in `run_round_follow_motion.launch.py`.
- Added defaults in:
  - `config/run_round_follow_defaults.yaml`
  - `config/run_round_follow_observe_strict.yaml`

### Why this change was needed
Latest run evidence showed long `NO_DET` periods, mostly `HOLD` in follow, and oscillating search enter/exit behavior.  
The bottleneck was reacquire/search control behavior and loss gating, not another detector tweak.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Syntax/build:
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py src/lrs_halmstad/launch/run_round_follow_motion.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```

2. Launch args visible:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py --show-args | \
  rg "reacquire_|camera_lock_tilt|under_target"
```

Expected output:
- New `reacquire_*` args visible.
- Build succeeds.

### If topics/services changed
- No topic/service renames.
- `/coord/follow_debug_status` now includes `search_phase`.

## 2026-03-06 - Vertical retention fix (tilt lock + under-target guard + dynamic tilt compensation)

### What changed
Implemented the vertical-retention control path so proxy tracking can keep the UGV in-frame longer when it moves low in the image or near-under UAV geometry:

1. `follow_uav.py`:
- Activated vertical camera lock with existing tilt params:
  - `camera_lock_tilt_enable`
  - `camera_lock_tilt_gain_per_px`
  - `camera_lock_tilt_deadband_px`
  - `camera_lock_tilt_min_deg` / `camera_lock_tilt_max_deg`
  - `camera_lock_tilt_max_step_deg`
  - `camera_lock_tilt_hold_on_bad_state`
- Added runtime tilt command state (`camera_tilt_cmd_deg`) into both camera control paths:
  - separate camera model `set_pose` path
  - gimbal topic path (`/<uav>/update_tilt`)
- Added under-target guard (`FOLLOW_UNDER_TARGET_GUARD_ENTER/EXIT`) using status `err_v_px`, `img_h`, and `range_m`:
  - if target is low in frame and close in range, optionally hold XY and yaw to avoid overshoot/loss.
- Extended `/coord/follow_debug_status` with:
  - `tilt_cmd_deg`
  - `under_target_hold`

2. `leader_estimator.py`:
- Added `camera_tilt_topic` subscription (default `/<uav>/update_tilt`).
- Added dynamic tilt compensation in ground-range projection:
  - uses live tilt when available, otherwise static `cam_pitch_offset_deg`.
- Extended status line with `tilt_deg=...` for visibility.

3. Launch/config wiring:
- `run_round_follow_motion.launch.py` now exposes:
  - `leader_camera_tilt_topic`
  - `camera_lock_tilt_*` args
  - `under_target_*` args
- Added defaults for new parameters in:
  - `config/run_round_follow_defaults.yaml`
  - `config/run_round_follow_observe_strict.yaml`

### Why this change was needed
Current proxy runs improved horizontally but still lost target vertically after ~10–20s.  
Root issue was: horizontal lock existed, but vertical lock and near-under-target handling were incomplete, and estimator range projection still used static pitch.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Syntax/build:
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py src/lrs_halmstad/lrs_halmstad/leader_estimator.py src/lrs_halmstad/launch/run_round_follow_motion.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```

2. Launch arg exposure:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py --show-args | \
  rg "leader_camera_tilt_topic|camera_lock_tilt|under_target"
```

Expected output:
- Tilt/under-target launch args are visible.
- Build succeeds.

### If topics/services changed
- No topic/service renames.
- New estimator parameter:
  - `camera_tilt_topic`
- New launch-exposed follow parameters:
  - `camera_lock_tilt_enable`
  - `camera_lock_tilt_gain_per_px`
  - `camera_lock_tilt_deadband_px`
  - `camera_lock_tilt_min_deg`
  - `camera_lock_tilt_max_deg`
  - `camera_lock_tilt_max_step_deg`
  - `under_target_guard_enable`
  - `under_target_err_v_ratio`
  - `under_target_range_m`
  - `under_target_hold_xy`
  - `under_target_hold_yaw`

## 2026-03-06 - Proxy retention hotfix + camera-control path safety + debug bagging

### What changed
Applied targeted proxy/runtime stability fixes based on Phase 2/3/4 bag analysis (no dataset/autolabel work yet):

1. `leader_estimator.py` proxy reacquire fix:
- Relaxed `proxy_switch_guard_local` behavior during active loss/reacquire.
- Before: far candidate switch could be hard-rejected repeatedly, causing prolonged `NO_DET`.
- Now: local switch guard is only enforced while not already in missed-detection streak (`bad_det_streak <= 0`), allowing reacquire after loss.

2. `follow_uav.py` camera control path safety (mode lock + failover):
- Added `camera_pose_control_mode`: `auto|separate_model|gimbal_only`.
- Added `camera_pose_failover_failures` threshold.
- In `auto` + `setpose` backend, repeated camera `set_pose` failures trigger one-way failover to `gimbal_only` path for that run.
- Keeps UAV `set_pose` active while avoiding repeated conflicting camera pose control in integrated camera/gimbal setups.
- Added camera-path info to events/debug status (`CAMERA_CONTROL_PATH:*`, `CAMERA_SETPOSE_FAILOVER_GIMBAL_ONLY`, `cam_path=*`).

3. Launch/config wiring:
- Exposed launch args:
  - `camera_pose_control_mode`
  - `camera_pose_failover_failures`
- Added defaults in both profiles.

4. Proxy class allowlist expansion:
- Added `remote` and `kite` (kept existing `traffic light,cell phone,book,suitcase,laptop`).

5. Bagging/debug observability:
- `run_follow_with_bag.sh` now records `/coord/follow_debug_status` by default for post-run diagnosis.

### Why this change was needed
Phase bag evidence showed high `NO_DET/HOLD` share and frequent `proxy_switch_guard_local` rejects during loss periods, while command output remained active.  
That indicates retention/reacquire blocking in control/perception gating, not just missing command publication.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Syntax/build:
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py src/lrs_halmstad/lrs_halmstad/leader_estimator.py
bash -n run_follow_with_bag.sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```

2. Launch arg exposure:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_round_follow_motion.launch.py --show-args | \
  rg "camera_pose_control_mode|camera_pose_failover_failures|camera_lock_enable|xy_motion_enable|follow_debug_status_topic"
```

3. Runtime check (combined proxy):
```bash
FOLLOW_PROFILE=strict bash run_follow_with_bag.sh phase_combined_recheck orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu \
  conf_threshold:=0.03 \
  iou_threshold:=0.50 \
  target_mode:=proxy_coco \
  target_coco_class_names:='traffic light,cell phone,book,suitcase,laptop,remote,kite' \
  target_coco_class_id:=-1 \
  proxy_use_geom_filters:=false \
  proxy_switch_force_local:=true \
  camera_pose_control_mode:=auto \
  camera_pose_failover_failures:=12 \
  shutdown_when_ugv_done:=true
```
Expected output:
- `/coord/follow_debug_status` is present in bag topics.
- Fewer prolonged `NO_DET` stretches caused by `proxy_switch_guard_local` lockout.
- In integrated-camera setups, camera pose failover event appears and pan/tilt control continues without repeated camera `set_pose` conflicts.

### If topics/services changed
- No topic/service renames.
- Added default bag topic recording:
  - `/coord/follow_debug_status`
- New follow parameters:
  - `camera_pose_control_mode`
  - `camera_pose_failover_failures`

## 2026-03-06 - Reverted flat-world experiment path

### What changed
Removed the temporary flat-world simulation additions used for YOLO/debug experiments.  
Deleted `run_gazebo_flat.sh`, deleted `simulation_flat.launch.py`, deleted `worlds/flat.sdf`, and removed custom world install wiring from `setup.py`.  
Kept the normal orchard flow and follow stack unchanged.

### Affected mode(s)
- Mode 1 (set_pose): affected (flat-world path removed)
- Mode 2 (controller backend): affected (flat-world path removed)

### How to verify
```bash
cd ~/halmstad_ws
test ! -f run_gazebo_flat.sh
test ! -f src/lrs_halmstad/launch/simulation_flat.launch.py
test ! -f src/lrs_halmstad/worlds/flat.sdf
python3 -m py_compile src/lrs_halmstad/setup.py
```
Expected output:
- The three flat-world files are absent.
- `setup.py` remains syntactically valid.

### If topics/services changed
- No topic/service/interface changes.

## 2026-03-06 - Wrapper world override fix (`world:=...`) for preflight consistency

### What changed
Fixed `run_follow_with_bag.sh` so `world:=...` in extra launch args now overrides positional world consistently for all wrapper stages.  
Preflight, metadata, reset service call, and final launch now all use the same effective world value (`EFFECTIVE_WORLD`) to avoid wrapper world mismatches.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
```bash
cd ~/halmstad_ws
bash -n run_follow_with_bag.sh
```
Expected output:
- Script syntax check passes.

### If topics/services changed
- No topic/service/interface changes.

## 2026-03-06 - Shell robustness fix for ROS setup sourcing (nounset)

### What changed
Patched `run_sim_orchard.sh` to temporarily disable nounset (`set +u`) while sourcing ROS setup scripts, then restore `set -u` afterwards.  
This prevents startup failure `AMENT_TRACE_SETUP_FILES: unbound variable` on environments where ROS setup scripts read unset variables under strict nounset mode.

### Affected mode(s)
- Mode 1 (set_pose): affected (launcher robustness)
- Mode 2 (controller backend): affected (launcher robustness)

### How to verify
```bash
cd ~/halmstad_ws
bash -n run_sim_orchard.sh
```
Expected output:
- No `AMENT_TRACE_SETUP_FILES: unbound variable` error from setup sourcing path.

### If topics/services changed
- No topic/service/interface changes.

## 2026-03-06 - Proxy multi-class allowlist + REACQUIRE motion gate relaxation

### What changed
Extended proxy mode so COCO proxy targeting can use a class allowlist (`target_coco_class_names`) instead of only a single class name/id; this supports mixed detections like `traffic light`, `cell phone`, and `book` in the same run.  
Added follow control parameter `track_only_allow_reacquire_motion` so UAV XY motion can continue in `REACQUIRE` state (camera lock still active), while HOLD/DEGRADED still block XY commands.  
Wired both parameters through launch and default/strict YAML profiles for direct CLI control during debugging runs.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```
2. Run proxy multi-class test:
```bash
FOLLOW_PROFILE=strict bash run_follow_with_bag.sh run_proxy_multiclass orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu \
  conf_threshold:=0.03 \
  iou_threshold:=0.50 \
  target_mode:=proxy_coco \
  target_coco_class_names:='traffic light,cell phone,book' \
  target_coco_class_id:=-1 \
  track_only_allow_reacquire_motion:=true \
  shutdown_when_ugv_done:=true
```
Expected output:
- Debug image shows labels for the allowlisted classes.
- `/coord/leader_estimate` and `/dji0/pose_cmd` should continue during REACQUIRE phases more often than before.

### If topics/services changed
- No topic/service renames.
- New parameters:
  - `target_coco_class_names`
  - `track_only_allow_reacquire_motion`

## 2026-03-06 - Debug image now shows YOLO class/conf boxes (including rejected candidates)

### What changed
Extended `leader_estimator` debug overlay so `/coord/leader_debug_image` can show YOLO detection boxes with class+confidence text for both accepted and rejected candidates, not only the final selected target box.  
Added parameters `debug_draw_all_boxes`, `debug_show_class_labels`, and `debug_max_boxes` to control overlay behavior and prevent clutter.  
Default and strict follow profiles now enable this debug overlay to make class identity debugging fast during 1.4 runs.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```
2. Run any follow test with debug image enabled (default):
```bash
FOLLOW_PROFILE=strict bash run_follow_with_bag.sh run_debug_boxes orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu \
  shutdown_when_ugv_done:=true
```
3. Open debug image:
```bash
ros2 run rqt_image_view rqt_image_view /coord/leader_debug_image
```
Expected output:
- You see per-box labels like `cell phone:0.62` or `car:0.31/class_filter` on the image.
- Selected target still appears with the thicker green box.

### If topics/services changed
- No topic/service changes.
- New estimator parameters:
  - `debug_draw_all_boxes`
  - `debug_show_class_labels`
  - `debug_max_boxes`

## 2026-03-05 - Proxy COCO target mode (cell-phone-as-UGV debug path)

### What changed
Added a proxy targeting mode in `leader_estimator` (`target_mode:=proxy_coco`) so a stable COCO class can be treated as the UGV target during observation-chain debugging.  
Proxy mode now supports dedicated class selectors (`target_coco_class_id` / `target_coco_class_name`) plus center-biased scoring and a switch-guard (`proxy_switch_max_px` + `proxy_switch_min_conf`) to reduce jumps between objects.  
Wired all new proxy parameters into `run_round_follow_motion.launch.py` and both default parameter profiles so runs can switch mode from CLI without code edits.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```

2. Run proxy-COCO path (debug assumption: `cell phone` maps to UGV in this sim):
```bash
FOLLOW_PROFILE=strict bash run_follow_with_bag.sh run_proxy_coco_cellphone orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu \
  target_mode:=proxy_coco \
  target_coco_class_name:='cell phone' \
  target_coco_class_id:=-1 \
  conf_threshold:=0.10 \
  iou_threshold:=0.50 \
  shutdown_when_ugv_done:=true
```
Expected output:
- `leader_estimator` status remains active with fewer class-filter misses in proxy mode.
- `/coord/leader_estimate` and `/dji0/pose_cmd` stream during lock periods.
- Debug image/overlay shows continuous proxy-target selection with HOLD/reacquire behavior rather than frequent hard jumps.

### If topics/services changed
- No topic/service renames.
- New launch/estimator parameters:
  - `target_mode`
  - `target_coco_class_id`
  - `target_coco_class_name`
  - `proxy_center_weight`
  - `proxy_center_max_px`
  - `proxy_switch_max_px`
  - `proxy_switch_min_conf`

## 2026-03-05 - 1.4 debug + robustness patch (truth-test tooling + HOLD command continuity)

### What changed
Added a hard truth-test helper script `scripts/yolo_truth_test.py` that captures one frame directly from ROS camera topic and runs Ultralytics inference on it (optionally comparing two models), so we can quickly separate model-quality issues from ROS-pipeline issues.  
Added launch arguments `conf_threshold` and `iou_threshold` in `run_round_follow_motion.launch.py` so YOLO thresholds can be tuned per run without editing YAML files.  
`follow_uav` now publishes explicit HOLD commands when leader pose is stale (instead of returning early): command stream stays alive, and camera lock/search continues sweeping for reacquire in `uav_only` runs.
Added preflight duplicate-node guard (`leader_estimator`/`follow_uav`/`ugv_motion_driver`) to fail fast if stale nodes are already running.
Added timeout protection for duplicate-node check (`ros2 node list`) so preflight cannot hang indefinitely; now it exits with a clear recovery hint if node listing stalls.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```

2. Truth-test custom model vs baseline on exact ROS frame:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/yolo_truth_test.py \
  --topic /dji0/camera0/image_raw \
  --model /home/william/halmstad_ws/models/yolo_26_trained.pt \
  --compare-model /home/william/halmstad_ws/models/yolov5n.pt \
  --conf 0.05 --iou 0.50 --out /tmp/yolo_truth_frame.png
```
Expected output:
- Frame is saved.
- Box counts printed for each model on the same image.

3. Run low-threshold observation test:
```bash
FOLLOW_PROFILE=strict bash run_follow_with_bag.sh run_truth_lowconf orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolo_26_trained.pt \
  yolo_device:=cpu \
  conf_threshold:=0.05 \
  iou_threshold:=0.50 \
  shutdown_when_ugv_done:=true
```
Expected output:
- `leader_estimator` stays alive.
- In no-detection periods, follow node still publishes HOLD ticks/commands (continuous `/dji0/pose_cmd` stream), while camera search can continue.

### If topics/services changed
- No topic/service renames.
- New helper script:
  - `scripts/yolo_truth_test.py`
- New launch parameters:
  - `conf_threshold`
  - `iou_threshold`
- New preflight toggle:
  - `PREFLIGHT_FORBID_EXISTING_FOLLOW_NODES`
  - `PREFLIGHT_NODE_LIST_TIMEOUT_S`

## 2026-03-05 - Hotfix: numeric target class args no longer crash leader_estimator

### What changed
`leader_estimator` now declares `target_class_name` and `target_class_names` with dynamic typing enabled, so launch overrides like `target_class_name:=0` are accepted instead of crashing with `InvalidParameterTypeException`.  
This is a robustness fix only; filtering behavior is unchanged (the values are still parsed to strings internally).

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```
2. Run with numeric class override:
```bash
FOLLOW_PROFILE=strict bash run_follow_with_bag.sh run_y26_numclass_check orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolo_26_trained.pt \
  yolo_device:=cpu \
  target_class_name:=0
```
Expected output:
- `leader_estimator` stays alive (no parameter type exception in launch log).
- `/coord/leader_estimate_status` keeps publishing.

### If topics/services changed
- No topic or service changes.

## 2026-03-05 - uav_only startup lock gate (wait for first leader estimate flow)

### What changed
Added an explicit readiness gate in `ugv_motion_driver` so motion startup can require first `/coord/leader_estimate` message flow before UGV begins moving.  
This is wired through launch/run wrappers (`ugv_ready_require_leader_flow`, `ugv_ready_timeout_s`) and auto-enabled by default for `leader_dependency_mode:=uav_only`, reducing early target-loss when camera/YOLO has not locked yet.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```

2. Run one `uav_only` follow run:
```bash
FOLLOW_PROFILE=strict bash run_follow_with_bag.sh run_lock_gate_check orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/huskyrun1.pt \
  yolo_device:=cpu \
  target_class_name:=robot
```

Expected output:
- `ugv_motion_driver` log includes `leader_flow=` in readiness status.
- UGV waits for lock (or until ready timeout) before motion starts.
- `meta.yaml` includes:
  - `ugv_ready_require_leader_flow`
  - `ugv_ready_timeout_s`
  - `uav_only_auto_ready_gate_applied`

### If topics/services changed
- No topic/service renames.
- New launch/run parameters:
  - `ugv_ready_require_leader_flow`
  - `ugv_ready_leader_topic`
  - `ugv_ready_timeout_s`

## 2026-03-05 - Stability gate patch (strict profile + stable-track motion + full-frame reject)

### What changed
Implemented a stability-gate patch to stop the "detect briefly, then drift/lose target" loop before moving to later substeps.  
`leader_estimator` now supports a maximum bbox area reject (`target_max_bbox_area_ratio`) so full-frame/huge false detections are filtered out; this complements existing min-area and center gating.  
`follow_uav` now enforces a track gate in perception mode: when estimator state is not stable `TRACK`, XY motion is held (camera lock/search keeps running) and full motion resumes only after debounced stable detections.  
Added a frozen strict profile file (`run_round_follow_observe_strict.yaml`) and wrapper support (`FOLLOW_PROFILE=strict`) so A/B runs are reproducible with one switch.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```
Expected output:
- Build completes.

2. Reset runtime processes:
```bash
cd ~/halmstad_ws
bash scripts/kill_everything.sh
```
Expected output:
- Ends with `[kill_everything] Done.`

3. Run strict A/B (same profile, only weights differ):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
FOLLOW_PROFILE=strict bash run_follow_with_bag.sh run_ab_v5n orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
FOLLOW_PROFILE=strict bash run_follow_with_bag.sh run_ab_husky orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/huskyrun1.pt \
  yolo_device:=cpu \
  target_class_name:=robot
```
Expected output:
- `meta.yaml` records `follow_profile: "strict"` and strict params file path.
- In status stream, fewer full-frame false locks; reject reasons visible when filtered.
- UAV does not make large unstable jumps during reacquire windows.

4. Quick continuity checks:
```bash
ros2 topic hz /coord/leader_estimate
ros2 topic hz /dji0/pose_cmd
timeout 10s ros2 topic echo /coord/leader_estimate_status
```
Expected output:
- Continuous estimate/command activity for most of run.
- No long streaks dominated by `NO_DET/class_filter/geom_filter`.

### If topics/services changed
- No topic/service renames.
- New/used parameters:
  - `leader_estimator.target_max_bbox_area_ratio`
  - `follow_uav.track_only_motion_enable`
  - `follow_uav.track_only_hold_yaw_enable`
- New strict profile file:
  - `src/lrs_halmstad/config/run_round_follow_observe_strict.yaml`
- New wrapper switch:
  - `FOLLOW_PROFILE=default|strict` (default unchanged).

## 2026-03-05 - 1.4 target-selection hardening + startup QoL patch

### What changed
Hardened `leader_estimator` target selection so it is less likely to lock onto non-UGV objects when multiple detections exist: added optional geometric gates (`target_min_center_v_ratio`, `target_min_bbox_area_ratio`) and a bottom-of-image preference bias (`target_bottom_bias`) on candidate scoring.  
Also added clearer reject reasons in status (`class_filter`, `geom_filter`, etc.) when detections are dropped, which makes debugging "UAV stands still" cases faster.  
For runtime usability, reduced default `uav_only` delayed UGV start (`UAV_ONLY_UGV_START_DELAY_S`) and made preflight camera checks auto-mode aware (required for `uav_only`, skipped for `ugv_state`), plus restored compatibility launcher `run_gazebo_sim.sh`.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```
Expected output:
- Build completes.

2. Clean reset before run:
```bash
cd ~/halmstad_ws
bash scripts/kill_everything.sh
```
Expected output:
- Script ends with `[kill_everything] Done.`

3. Terminal A (sim):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
bash run_gazebo_sim.sh orchard
```
Expected output:
- Gazebo starts normally.

4. Terminal B (UAV):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
ros2 launch lrs_halmstad spawn1m100gimbal.launch.py world:=orchard name:=dji0
```
Expected output:
- UAV and camera entity spawn without `PackageNotFoundError`.

5. Terminal C (old generic YOLO baseline, no hard class lock):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
bash run_follow_with_bag.sh run_obs_oldweights orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- Preflight passes.
- `leader_estimator` status includes reject reasons when it drops candidates.
- `/coord/leader_estimate` and `/dji0/pose_cmd` should publish when target is detected.

6. Terminal C (custom Husky weights, class-locked):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
bash run_follow_with_bag.sh run_obs_huskyweights orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/huskyrun1.pt \
  yolo_device:=cpu \
  target_class_name:=robot
```
Expected output:
- More stable target identity than generic weights.
- `leader_estimator` startup shows active class filter.

7. Terminal D (visual debug):
```bash
cd ~/halmstad_ws
bash run_rqt_image_view.sh dji0 raw
```
```bash
cd ~/halmstad_ws
bash run_rqt_image_view.sh dji0 debug
```
Expected output:
- Raw camera in one window and YOLO overlay/status in another (`rqt_image_view`, not `showimage`).

### If topics/services changed
- No renamed topics/services.
- New estimator params (optional, defaults set in YAML):
  - `target_min_center_v_ratio`
  - `target_min_bbox_area_ratio`
  - `target_bottom_bias`
- New compatibility script:
  - `run_gazebo_sim.sh` (wrapper to `run_sim_orchard.sh`)

## 2026-03-05 - Launch wiring fix for class-filtered YOLO runs

### What changed
Fixed launch argument wiring so `target_class_id`, `target_class_name`, and `target_class_names` are now declared in `run_round_follow_motion.launch.py` and forwarded into `leader_estimator`.  
Before this fix, passing `target_class_name:=robot` in run commands did not affect estimator filtering (`<any>` stayed active), which could make the UAV lock onto non-target objects.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```
Expected output:
- Build completes.

2. Run one follow test with explicit class filter:
```bash
bash run_follow_with_bag.sh run_cls_filter_check orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu \
  target_class_name:=robot
```
Expected output:
- `leader_estimator` startup line shows `target_class_name=robot` (not `<any>`).

### If topics/services changed
- No topics/services changed.
- New/active launch args in this flow: `target_class_id`, `target_class_name`, `target_class_names`.

## 2026-03-04 - 1.3/1.4 robustness pass: active camera lock + marker aid + fast-exit diagnostics

### What changed
Implemented active camera lock in `follow_uav` using estimator status bearing: pan is updated continuously, bounded and rate-limited, then applied in both backends (`setpose` camera yaw offset and controller `update_pan`).  
Added camera-pan compensation in `leader_estimator` (subscribes to `/\<uav\>/update_pan`) so world estimate math stays consistent when camera pans; also added optional marker-first detection (ArUco) with range estimate from marker pixel size, plus stronger class-continuity options (`target_class_names`, prefer-last-class).  
Improved wrapper diagnostics for short runs: `run_follow_with_bag.sh` now prints quick-exit hints (simulator failfast, set_pose failures, YOLO init issues, preflight conflicts) instead of opaque exits.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```
Expected output:
- Build completes.

2. Terminal A:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
ros2 launch clearpath_gz simulation.launch.py world:=orchard use_sim_time:=true gui:=true
```

3. Terminal B:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
ros2 launch lrs_halmstad spawn1m100gimbal.launch.py world:=orchard name:=dji0
```

4. Terminal C (setpose backend):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
bash run_follow_with_bag.sh run_obs_lock_setpose orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- `follow_uav` startup line includes `camera_lock_enable=True`.
- Events can include `CAMERA_LOCK_ENTER/EXIT`.
- Camera keeps tracking longer before drifting out of view.

5. Terminal C (controller backend):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
bash run_follow_with_bag.sh run_obs_lock_controller orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- Same observation chain, with active pan on controller topics.

6. Terminal D (visual checks):
```bash
cd ~/halmstad_ws
bash run_rqt_image_view.sh dji0 raw
```
```bash
cd ~/halmstad_ws
bash run_rqt_image_view.sh dji0 debug
```
Expected output:
- Raw feed on `/dji0/camera0/image_raw`
- Overlay on `/coord/leader_debug_image`

7. Stream continuity checks:
```bash
ros2 topic hz /coord/leader_estimate
ros2 topic hz /dji0/pose_cmd
timeout 10s ros2 topic echo /coord/leader_estimate_status
```
Expected output:
- Non-zero estimate and pose command activity.
- Status includes `bearing_deg=` and `pan_deg=` fields.

### Topics/services changed (required/optional updates)
- No renamed topics/services.
- New/used interfaces:
  - `leader_estimator` now consumes camera pan topic (default `/\<uav\>/update_pan`) for geometry compensation.
  - Optional marker mode via parameters:
    - `marker_enable`, `marker_dictionary`, `marker_id`, `marker_size_m`
  - Optional class allowlist:
    - `target_class_names` (comma-separated)
- New diagnostics behavior:
  - `run_follow_with_bag.sh` prints quick-exit hints for short/failed runs.

## 2026-03-04 - 1.4 stability patch: dropout recovery + camera-flow preflight + uav_only YOLO guard

### What changed
Added a short-dropout fallback in `follow_uav` for `leader_input_type:=pose`: if estimates go stale briefly, the node now predicts leader motion from the last valid estimator-driven velocity and keeps sending conservative commands instead of hard-freezing immediately.  
Strengthened observation defaults for robustness (`conf_threshold`, hold horizon, altitude/camera geometry, quality timeout) so runs do not stop after a few `NO_DET` frames.  
Hardened run orchestration with two fail-fast checks: preflight now verifies camera topics are actively publishing messages (not only listed), and `uav_only` runs now require a valid `yolo_weights` path by default.
Added a dedicated helper script (`run_rqt_image_view.sh`) to open either raw UAV camera feed or estimator debug feed in a separate terminal without conflicting with follow mode locks.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```
Expected output:
- Build completes without errors.

2. Run observation-only setpose:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_obs_setpose_stability orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- No immediate freeze after first short detection drop.
- `follow_uav` startup log includes `stale_predict_enable=True`.
- During dropouts, events may include `POSE_STALE_PREDICT_ENTER` / `POSE_STALE_PREDICT_EXIT`.

3. Runtime continuity checks during run:
```bash
ros2 topic hz /coord/leader_estimate
ros2 topic hz /dji0/pose_cmd
timeout 10s ros2 topic echo /coord/leader_estimate_status
```
Expected output:
- `/dji0/pose_cmd` stays active (does not flatline to zero immediately on short `NO_DET`).
- Status may show `NO_DET/HOLD/REACQUIRE`, but command stream should recover.

4. Preflight negative check (camera flow):
```bash
cd ~/halmstad_ws
PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW=true \
bash run_follow_preflight.sh /a201_0000/platform/odom /a201_0000/controller_manager platform_velocity_controller orchard dji0 setpose uav_only
```
Expected output:
- If camera is not publishing, preflight fails with camera-flow message before launch.

5. Terminal D camera/debug windows:
```bash
cd ~/halmstad_ws
bash run_rqt_image_view.sh dji0 raw
# or
bash run_rqt_image_view.sh dji0 debug
```
Expected output:
- `raw`: `/dji0/camera0/image_raw` visible
- `debug`: `/coord/leader_debug_image` visible when estimator is running

### Topics/services changed (required/optional updates)
- No topic/service name changes.
- New/updated behavior:
  - `follow_uav` may emit `POSE_STALE_PREDICT_ENTER/EXIT` events during short estimate dropouts.
  - `run_follow_preflight.sh` enforces camera message flow when `PREFLIGHT_REQUIRE_UAV_CAMERA_FLOW=true` (default true).
  - `run_follow_with_bag.sh` enforces YOLO weights for `leader_dependency_mode:=uav_only` when `UAV_ONLY_REQUIRE_YOLO=true` (default true).

## 2026-03-04 - 1.4 hotfix: follow camera params scope + setpose startup seed

### What changed
Fixed a parameter scope bug where `camera_pitch_deg` accidentally landed under `ugv_motion_driver` instead of `follow_uav`, causing follow camera commands to stay at the default `-45` tilt.  
Added a one-time setpose startup seed in `follow_uav` so observation-only (`uav_only`) runs publish an initial UAV command pose and camera pose before first detection, reducing startup deadlock (`NO_DET` forever).  
Aligned setpose camera orientation command to include UAV yaw (pitch + yaw quaternion), matching simulator/controller behavior more closely.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): unaffected by startup seed logic

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```
Expected output:
- build completes

2. Run observation-only setpose:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_obs_setpose_hotfix orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- `follow_uav` startup log shows `camera_pitch_deg=-70.0`
- events include `FOLLOW_STARTUP_SEED_SETPOSE`

3. During run:
```bash
ros2 topic hz /coord/leader_estimate
ros2 topic hz /dji0/pose_cmd
timeout 10s ros2 topic echo /coord/leader_estimate_status
```
Expected output:
- `/dji0/pose_cmd` no longer stays at zero-count for entire run
- status should move out of persistent `NO_DET` when UGV enters view

### Topics/services changed (required/optional updates)
- No topic/service name changes.
- Behavior changes:
  - setpose backend emits startup seed pose/camera command once on node start.

### Files changed in this implementation block
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-04 - Camera/estimator geometry alignment patch for observation runs

### What changed
Aligned camera geometry defaults between follow actuation and estimator math to avoid “camera looks wrong / YOLO sees wrong plane” behavior.  
`follow_uav` now defaults to a steeper down-tilt (`camera_pitch_deg=-70`) and explicit camera z offset, and `leader_estimator` now uses matching pitch and camera z offset assumptions (`cam_pitch_offset_deg=-70`, `cam_z_offset_m=-0.27`).

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Rebuild:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_halmstad --symlink-install
```
Expected output:
- build completes successfully

2. Run one observation-only test:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_obs_setpose_geomfix orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- `follow_uav` startup log includes `camera_pitch_deg=-70.0`
- `leader_estimator` startup log includes `cam_pitch_deg=-70.0`

3. Visual checks while run is active:
```bash
ros2 run rqt_image_view rqt_image_view /dji0/camera0/image_raw
ros2 run rqt_image_view rqt_image_view /coord/leader_debug_image
```
Expected output:
- raw camera view shows a steeper downward scene (ground/UGV coverage improved)
- debug image topic active with state/overlay text when estimator is running

### Topics/services changed (required/optional updates)
- No topic/service name changes.
- Parameter-only alignment update.

### Files changed in this implementation block
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-04 - 1.4 patch: debug image stream + uav_only startup stabilization

### What changed
Added a live estimator debug image publisher (`/coord/leader_debug_image`) so you can directly inspect what YOLO/estimator sees during runs.  
Stabilized observation-only startup by auto-applying a UGV start delay in `uav_only` runs (unless explicitly overridden), so the estimator has time to load before the UGV starts moving.  
Applied conservative estimator defaults for sparse-detection scenarios: lower confidence threshold and longer hold/reacquire window.

### Affected mode(s)
- Mode 1 (set_pose): affected
- Mode 2 (controller backend): affected

### How to verify
1. Static checks:
```bash
cd ~/halmstad_ws
bash -n run_follow_with_bag.sh
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/leader_estimator.py
```
Expected output:
- no syntax errors

2. Runtime check (setpose backend, observation-only):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_obs_setpose_stable orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- launch shows `leader_mode=pose dependency=uav_only`
- launch command includes `ugv_start_delay_s:=...` when not manually set
- `/coord/leader_debug_image` is available during run

3. Live camera/debug windows during the same run:
```bash
ros2 run rqt_image_view rqt_image_view /dji0/camera0/image_raw
ros2 run rqt_image_view rqt_image_view /coord/leader_debug_image
```
Expected output:
- raw camera feed visible
- debug feed visible with state text and detection overlay when detections exist

4. Stream continuity quick checks:
```bash
ros2 topic hz /coord/leader_estimate
ros2 topic hz /dji0/pose_cmd
timeout 10s ros2 topic echo /coord/leader_estimate_status
```
Expected output:
- non-zero estimate + pose_cmd activity
- status should not be dominated by long `STALE` spans after startup

### Topics/services changed (required/optional updates)
- Added estimator debug topic:
  - `/coord/leader_debug_image` (`sensor_msgs/msg/Image`)
- No service changes.
- Run metadata added:
  - `ugv_start_delay_s`
  - `uav_only_auto_start_delay_applied`

### Files changed in this implementation block
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `run_follow_with_bag.sh`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-04 - 1.4 Observation-chain stabilization (YOLO -> estimate -> follow in `uav_only`)

### What changed
Stabilized the perception pipeline so `leader_dependency_mode:=uav_only` has a continuous estimate stream instead of sparse bursts.  
Implemented longer and safer estimate hold/reacquire behavior, relaxed overly strict reject thresholds, and added explicit estimated velocity output from the estimator tracker path.  
Extended run bag capture to include estimator odometry output for thesis comparability and post-run diagnosis.

### Affected mode(s)
- Mode 1 (set_pose): affected (better estimate continuity and hold/reacquire behavior in `leader_mode:=pose`)
- Mode 2 (controller backend): affected (same estimate continuity improvements; backend behavior unchanged)

### How to verify
1. Static checks (already run after implementation):
```bash
cd ~/halmstad_ws
bash -n run_follow_with_bag.sh
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/leader_estimator.py
```
Expected output:
- no syntax errors

2. Required runtime check: observation-only setpose
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_obs_setpose_stable orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- launch control-chain line includes `leader_mode=pose dependency=uav_only`
- run completes cleanly
- bag contains non-trivial `/coord/leader_estimate` and `/dji0/pose_cmd` counts (not near-zero)
- bag contains `/coord/leader_estimate_odom`

3. Required runtime check: observation-only controller backend
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_obs_controller_stable orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- same observation-chain behavior as setpose run
- `/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw` active in bag
- `/coord/leader_estimate_odom` present in bag

4. Bag quick checks:
```bash
ros2 bag info runs/run_obs_setpose_stable/bag | rg 'leader_estimate|pose_cmd|leader_estimate_odom'
ros2 bag info runs/run_obs_controller_stable/bag | rg 'leader_estimate|pose_cmd|leader_estimate_odom|flight_control_setpoint'
```
Expected output:
- both runs show `leader_estimate` and `leader_estimate_odom`
- both runs show `pose_cmd` activity
- controller run shows controller setpoint topic activity

### Topics/services changed (required/optional updates)
- Added estimator output topic:
  - `/coord/leader_estimate_odom` (`nav_msgs/msg/Odometry`) with tracker-derived velocity
- Existing topic unchanged:
  - `/coord/leader_estimate` (`geometry_msgs/msg/PoseStamped`)
  - `/coord/leader_estimate_status` (`std_msgs/msg/String`)
- Run wrapper bag allowlist updated:
  - added `/coord/leader_estimate_odom`

### Parameter tuning applied (default profile)
- `leader_estimator.max_hold_frames: 3 -> 0` (time-governed hold)
- `leader_estimator.max_hold_s: 0.5 -> 2.5`
- `leader_estimator.image_timeout_s: 1.0 -> 1.2`
- `leader_estimator.conf_threshold: (implicit 0.25) -> 0.20`
- `leader_estimator.max_xy_jump_m: 6.0 -> 10.0`
- `leader_estimator.max_target_speed_mps: 8.0 -> 14.0`
- `leader_estimator.max_bearing_jump_deg: 40.0 -> 80.0`
- `leader_estimator.ok_debounce_frames: (implicit 2) -> 1`
- `leader_estimator.bad_debounce_frames: (implicit 2) -> 3`
- `follow_uav.pose_timeout_s: 0.75 -> 1.0`
- `follow_uav.quality_status_timeout_s: 1.5 -> 2.0`
- `follow_uav.state_bad_debounce_ticks: 2 -> 3`

### Files changed in this implementation block
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `run_follow_with_bag.sh`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-04 - Leader dependency lock (`uav_only|ugv_state`) for no-UGV-state runs

### What changed
Implemented an explicit dependency policy switch for follow runs: `leader_dependency_mode:=uav_only|ugv_state` in the run wrapper path.  
`uav_only` enforces observation/estimate-driven follow input by requiring `leader_mode:=pose` (or `estimate` alias) and disabling preflight's mandatory UGV odom-flow check, while preserving the same run harness and logging pipeline.  
Added launch visibility and metadata traceability for dependency mode so each run explicitly records whether it was observation-only or UGV-state-assisted debug.

### Affected mode(s)
- Mode 1 (set_pose): affected (policy guard + metadata + launch visibility)
- Mode 2 (controller backend): affected (policy guard + metadata + launch visibility)

### How to verify
1. Static checks (already run):
```bash
cd ~/halmstad_ws
bash -n run_follow_preflight.sh
bash -n run_follow_with_bag.sh
python3 -m py_compile src/lrs_halmstad/launch/run_round_follow_motion.launch.py
```
Expected output:
- no syntax errors

2. Enforced policy check (should fail):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_dep_fail orchard dji0 odom \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose
```
Expected output:
- wrapper exits early with message:
  `leader_dependency_mode=uav_only requires leader_mode:=pose (or estimate alias)`

3. Observation-only run (setpose backend):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_dep_uav_only_setpose orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- launch control-chain log includes `dependency=uav_only`
- run completes and `meta.yaml` includes:
  - `leader_dependency_mode: "uav_only"`
  - `leader_mode: "pose"`
  - `control_chain.leader_dependency_mode: "uav_only"`

4. Observation-only run (controller backend):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_dep_uav_only_controller orchard dji0 pose \
  leader_dependency_mode:=uav_only \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- same dependency log + metadata behavior as above
- controller command topics active in bag as usual

### Topics/services changed (required/optional updates)
- No message type changes.
- New launch/run argument:
  - `leader_dependency_mode:=uav_only|ugv_state` (default: `ugv_state`)
- Policy behavior:
  - `uav_only` requires `leader_mode:=pose|estimate`
  - preflight odom-flow requirement is set to false by default in `uav_only`

### Files changed in this implementation block
- `run_follow_with_bag.sh`
- `run_follow_preflight.sh`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-04 - Follow+leash stabilization micro-tuning (5 params)

### What changed
Applied a small, conservative stability tuning pass to reduce zigzag/overshoot while preserving the same follow+leash architecture and control chain.  
Tweaks were limited to five parameters in `run_round_follow_defaults.yaml`: lower trajectory aggressiveness and slightly stronger controller deadband/rate limiting.

Changed parameters:
- `traj_pos_gain`: `2.0 -> 1.7`
- `traj_max_speed_mps`: `1.5 -> 1.2`
- `traj_max_accel_mps2`: `3.0 -> 2.0`
- `controller_profile_cmd_xy_deadband_m`: `0.05 -> 0.08`
- `controller_profile_min_cmd_period_s`: `0.12 -> 0.15`

### Affected mode(s)
- Mode 1 (set_pose): affected (slightly calmer trajectory shaping)
- Mode 2 (controller backend): affected (slightly calmer trajectory shaping + stronger anti-jitter profile)

### How to verify
1. Static checks (already run):
```bash
cd ~/halmstad_ws
bash -n run_follow_preflight.sh
bash -n run_follow_with_bag.sh
python3 -m py_compile src/lrs_halmstad/launch/run_round_follow_motion.launch.py
```
Expected output:
- no syntax errors

2. Quick runtime sanity check (optional):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_tune_setpose orchard dji0 odom \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu

bash run_follow_with_bag.sh run_tune_controller orchard dji0 odom \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- Same command chain behavior as before.
- Slightly reduced command jitter/oscillation, especially in controller backend.

### Topics/services changed (required/optional updates)
- No topic/service/interface changes.
- Parameter-only tuning update.

### Files changed in this implementation block
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-03 - Control-chain freeze (planner intent -> backend adapter, no manual run path)

### What changed
Froze the run harness control chain so experiment runs are explicitly planner-driven: leader input goes into `follow_uav`, which publishes canonical intent on `/<uav>/pose_cmd`, and backend adapters execute that intent via either `set_pose` service or controller command topics.  
Added preflight guardrails to reject conflicting publishers on controller command topics and on `/<uav>/pose_cmd`, preventing mixed/manual command sources from silently affecting runs.  
Extended `meta.yaml` run metadata with a `control_chain` block and fixed effective launch-arg capture so recorded metadata reflects actual runtime values (including extra launch overrides).

### Affected mode(s)
- Mode 1 (set_pose): affected (explicit control-chain metadata + intent conflict guard)
- Mode 2 (controller backend): affected (explicit control-chain metadata + stronger publisher conflict guard)

### How to verify
1. Static checks (already run):
```bash
cd ~/halmstad_ws
bash -n run_follow_preflight.sh
bash -n run_follow_with_bag.sh
python3 -m py_compile src/lrs_halmstad/launch/run_round_follow_motion.launch.py
```
Expected output:
- no syntax errors

2. Runtime smoke check (setpose backend):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_chain_setpose orchard dji0 odom \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- launch log prints control-chain line from `run_round_follow_motion.launch.py`
- run completes with artifacts in `runs/run_chain_setpose/`
- `meta.yaml` contains:
  - `control_chain.profile: planner_to_backend_v1`
  - `control_chain.intent_topic: /dji0/pose_cmd`
  - `control_chain.backend: setpose`

3. Runtime smoke check (controller backend):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_chain_controller orchard dji0 odom \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- launch log prints same control-chain line with `backend=controller`
- preflight blocks start if controller command topics already have publishers
- run artifacts in `runs/run_chain_controller/` include `meta.yaml` with controller adapter topics in `control_chain`

4. Conflict guard check (optional):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

# start a temporary publisher on /dji0/pose_cmd in another terminal, then:
bash run_follow_preflight.sh /a201_0000/platform/odom /a201_0000/controller_manager platform_velocity_controller orchard dji0 setpose
```
Expected output:
- preflight fails with `intent conflict: topic already has publishers: /dji0/pose_cmd`

### Topics/services changed (required/optional updates)
- No message type changes.
- New enforced run contract checks:
  - `/<uav>/pose_cmd` must not already have external publishers when follow run starts
  - in controller backend mode, these must not already have external publishers:
    - `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
    - `/<uav>/update_pan`
    - `/<uav>/update_tilt`
- Metadata schema addition:
  - `control_chain` block in run `meta.yaml`

### Files changed in this implementation block
- `run_follow_preflight.sh`
- `run_follow_with_bag.sh`
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-03 - Robust bag shutdown + controller backend smoothing guard

### What changed
Hardened `run_follow_with_bag.sh` shutdown so rosbag recorder stop is now deterministic: `SIGINT` with timeout, then `SIGTERM`, then `SIGKILL` as final fallback. This prevents Terminal C from hanging when rosbag ignores the first signal.  
Added conservative controller-backend tuning defaults in `follow_uav` that only apply when `uav_backend=controller` and the values still look like baseline defaults; explicit user overrides are preserved.

### Affected mode(s)
- Mode 1 (set_pose): affected only by safer run teardown behavior
- Mode 2 (controller backend): affected by safer teardown + smoother command profile at default settings

### How to verify
1. Static checks (already run):
```bash
cd ~/halmstad_ws
bash -n run_follow_with_bag.sh
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py
python3 -m py_compile src/lrs_halmstad/launch/run_round_follow_motion.launch.py
```
Expected output:
- `OK:run_follow_with_bag.sh`
- `OK:follow_uav.py`
- `OK:run_round_follow_motion.launch.py`

2. Runtime check for non-hanging shutdown:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_mode2_shutdown_check orchard dji0 odom \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- Run exits back to shell without hanging after launch shutdown.
- If recorder is slow to stop, wrapper prints escalation messages:
  - `rosbag still running after SIGINT; escalating to SIGTERM`
  - `rosbag still running after SIGTERM; escalating to SIGKILL`
- `runs/run_mode2_shutdown_check/bag_info.txt` is written.

3. Optional process check after run:
```bash
pgrep -af "run_follow_with_bag.sh|ros2 bag record" || true
```
Expected output:
- No leftover process from the completed run.

### Topics/services changed (required/optional updates)
- No topic/service contract change.
- New optional env controls in `run_follow_with_bag.sh`:
  - `BAG_STOP_INT_TIMEOUT_S` (default `4`)
  - `BAG_STOP_TERM_TIMEOUT_S` (default `3`)
  - `BAG_STOP_KILL_TIMEOUT_S` (default `2`)
- New optional `follow_uav` params (controller profile):
  - `controller_profile_enable`
  - `controller_profile_smooth_alpha`
  - `controller_profile_max_step_m_per_tick`
  - `controller_profile_cmd_xy_deadband_m`
  - `controller_profile_yaw_deadband_rad`
  - `controller_profile_min_cmd_period_s`

### Files changed in this implementation block
- `run_follow_with_bag.sh`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `Markdowns/Implementation_Documentation/IMPLEMENTATION_LOG.md`

## 2026-03-03 - Two-mode UAV backend integration (`setpose|controller`)

### What changed
Added a neutral backend switch (`uav_backend:=setpose|controller`) to the follow launch path and follow node so one follow pipeline can drive two actuation backends without renaming the rest of the stack.  
Mode 1 (`setpose`) keeps the previous behavior as default. Mode 2 (`controller`) now starts the `simulator` backend node from `run_round_follow_motion.launch.py`, and `follow_uav` publishes controller-interface commands (`psdk_ros2/...`, `update_pan`, `update_tilt`) instead of calling `SetEntityPose` directly.  
Controller seed state in `follow_uav` is synchronized from launch backend initial pose arguments (`backend_initial_x/y/yaw_deg`) to avoid startup mismatch.

Also updated run wrappers and preflight so backend mode is explicit, validated, and recorded (`meta.yaml`), and so mode-specific conflicts are detected earlier.

### Affected mode(s)
- Mode 1 (set_pose): affected (default preserved, now explicit via `uav_backend=setpose`)
- Mode 2 (controller backend): affected (new integrated path via same follow entrypoint)

### How to verify
1. Static syntax/compile checks (already run):
```bash
cd ~/halmstad_ws
bash -n run_follow_with_bag.sh
bash -n run_follow_preflight.sh
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/follow_uav.py src/lrs_halmstad/launch/run_round_follow_motion.launch.py
```
Expected output:
- No syntax errors.
- `OK:run_follow_with_bag.sh`
- `OK:run_follow_preflight.sh`
- `OK:python_compile`

2. Runtime smoke test, Mode 1 (baseline set_pose):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_mode1_test orchard dji0 odom \
  uav_backend:=setpose \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- Preflight prints `backend=setpose` and ends with `[run_follow_preflight] ok`
- Follow node startup log includes `backend=setpose`
- UGV and UAV move, run ends cleanly if `shutdown_when_ugv_done:=true`
- Run artifacts exist in `runs/run_mode1_test/` with `meta.yaml`, `launch.log`, `rosbag.log`, `bag_info.txt`, bag directory

3. Runtime smoke test, Mode 2 (controller backend):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_mode2_test orchard dji0 odom \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- Preflight prints `backend=controller` and ends with `[run_follow_preflight] ok`
- Launch output includes simulator process start (controller backend path)
- Follow node startup log includes `backend=controller`
- UAV command topics become active:
  - `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
  - `/<uav>/update_pan`
  - `/<uav>/update_tilt`
- Run artifacts exist in `runs/run_mode2_test/` with same contract as Mode 1

### Topics/services changed (required/optional updates)
- Required launch arg (new):
  - `uav_backend:=setpose|controller` (default: `setpose`)
- Required service dependency (unchanged, still required):
  - `/world/<world>/set_pose` (used by setpose backend and by simulator backend)
- Optional/conditional topics (controller mode):
  - `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
  - `/<uav>/update_pan`
  - `/<uav>/update_tilt`
  - `/<uav>/pose` (simulator-published pose stream)
- Metadata update:
  - `run_follow_with_bag.sh` now writes `uav_backend` into `meta.yaml`

### Files changed in this implementation block
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py`
- `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
- `src/lrs_halmstad/config/run_round_follow_defaults.yaml`
- `run_follow_with_bag.sh`
- `run_follow_preflight.sh`

## 2026-03-03 - Mode 2 runtime fix (simulator import crash)

### What changed
Mode 2 UAV non-movement was traced to `simulator` crashing at startup with `ModuleNotFoundError: lrs_halmstad.world_names`.  
Added missing module `src/lrs_halmstad/lrs_halmstad/world_names.py` with `gazebo_world_name()` so both `simulator.py` and `command.py` imports resolve at runtime.

Also added controller-mode fail-fast in `run_round_follow_motion.launch.py`: if simulator backend exits, launch now shuts down immediately instead of silently continuing with a non-moving UAV.

### Affected mode(s)
- Mode 1 (set_pose): not functionally changed
- Mode 2 (controller backend): fixed (simulator now starts instead of crashing)

### How to verify
1. Build after fix (required for runtime import path):
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lrs_halmstad
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```
Expected output:
- Build succeeds with no Python module install errors.

2. Static checks (already run):
```bash
cd ~/halmstad_ws
python3 -m py_compile src/lrs_halmstad/lrs_halmstad/world_names.py \
  src/lrs_halmstad/lrs_halmstad/simulator.py \
  src/lrs_halmstad/lrs_halmstad/command.py \
  src/lrs_halmstad/launch/run_round_follow_motion.launch.py
```
Expected output:
- No syntax/compile errors.

3. Runtime check, Mode 2:
```bash
cd ~/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash

bash run_follow_with_bag.sh run_mode2_fixcheck orchard dji0 odom \
  uav_backend:=controller \
  leader_perception_enable:=true \
  start_leader_estimator:=true \
  yolo_weights:=/home/william/halmstad_ws/models/yolov5n.pt \
  yolo_device:=cpu
```
Expected output:
- `simulator` starts without `ModuleNotFoundError`.
- Follow node reports `backend=controller`.
- UAV moves instead of staying at spawn.
- If simulator crashes for any reason, launch terminates quickly with message:
  `[run_round_follow_motion] simulator backend exited in controller mode; shutting down launch`

### Topics/services changed (required/optional updates)
- No topic/service contract changes in this fix.
- Added internal module dependency:
  - `lrs_halmstad.world_names` (now present in package source)

### Files changed in this implementation block
- `src/lrs_halmstad/lrs_halmstad/world_names.py` (new)
- `src/lrs_halmstad/launch/run_round_follow_motion.launch.py` (controller-mode fail-fast on simulator exit)

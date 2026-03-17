# Running Sim

Default tested path: `warehouse`.

Assumption:
- you already start in the workspace root
- wrapper files now live under `scripts/`
- use `./run.sh <name>` and `./stop.sh <name>` from the workspace root
- optional bash startup hook for completion:
  - `source "$HOME/halmstad_ws/scripts/bashrc_halmstad_ws.bash"`
- quiet YOLO defaults in this file refer to `./run.sh 1to1_yolo ...`
- a bare `ros2 launch lrs_halmstad run_follow.launch.py ...` still inherits the launch-argument defaults unless you override them explicitly

Recommended tmux workflow:
- start with `./run.sh tmux_1to1 warehouse`
- for an on-screen Gazebo test, use `./run.sh tmux_1to1 warehouse gui:=true`
- start YOLO mode with `./run.sh tmux_1to1 warehouse mode:=yolo`
- start YOLO tracker mode with `./run.sh tmux_1to1 warehouse mode:=yolo tracker:=true`
- record a YOLO run with `./run.sh tmux_1to1 warehouse mode:=yolo record:=true`
- record a lightweight YOLO Step 2/3 run with `./run.sh tmux_1to1 warehouse mode:=yolo record:=true record_profile:=step2_light`
  - this is the recommended small proof bag for Step 2, Step 3, the estimator phase, the follow-point phase, the planner phase, and the harder-case hardening phase because it records the raw selected-target, filtered selected-target, estimator, follow-point, planned-target, bridge, and downstream actuation topics without images
- record a YOLO vision run with `./run.sh tmux_1to1 warehouse mode:=yolo record:=true record_profile:=vision`
  - use this when you also need camera/debug image proof
- speed up sim time with `./run.sh tmux_1to1 warehouse rtf:=2.0`
- pass a custom YOLO weight with `./run.sh tmux_1to1 warehouse mode:=yolo weights:=yolo26v2.pt`
- enable truth-assisted YOLO testing with `./run.sh tmux_1to1 warehouse mode:=yolo use_estimate:=false`
- stop with `./stop.sh tmux_1to1 warehouse`
- default tmux layout is panes:
  - row 1: `gazebo | spawn`
  - row 2: `localization | nav2`
  - row 3: `follow`
- current default delays:
  - `gui:=false` -> `spawn=8`, `localization/nav2=10`, `follow=12`
  - `gui:=true` -> `spawn=6`, `localization/nav2=8`, `follow=10`
- in tmux mode, the follow pane now starts after its staged delay without an extra Nav2-localization wait gate
- Nav2 readiness is now enforced inside `ugv_nav2_driver`, which avoids circular startup waits during AMCL initial-pose seeding
- `ugv_nav2_driver` now also waits for those required Nav2 lifecycle nodes before its first goal send and before retrying a rejected goal, so the UGV side stays guarded even if launch timing changes
- on WSL, `./run.sh gazebo_sim ... true` now defaults to software GL rendering for GUI stability unless you override it
- `camera_tracker` now uses an odom-frame UGV pose for camera-only reacquisition by default, which helps detached-camera YOLO runs recover from bad startup framing without mixing `map` and `odom` frames while keeping UAV motion ownership on the visual pipeline

## Start 1-to-1 Sim With Nav2 And AMCL Follow

Run these in order in separate terminals:

1. Start Gazebo:

```bash
./run.sh gazebo_sim warehouse
```

Without GUI:

```bash
./run.sh gazebo_sim warehouse false
```

2. Spawn one UAV:

```bash
./run.sh spawn_uav warehouse uav_name:=dji0
```

3. Start localization:

```bash
./run.sh localization warehouse
```

If you want a different saved map:

```bash
./run.sh localization warehouse maps/warehouse_manual.yaml
```

4. Start Nav2:

```bash
./run.sh nav2
```

5. Start the AMCL-based odom follow stack:

```bash
./run.sh 1to1_follow warehouse
```

#### If you want the same stack started automatically in tmux, use:

```bash
./run.sh tmux_1to1 warehouse
```

Recommended visible test run:

```bash
./run.sh tmux_1to1 warehouse gui:=true
```

YOLO variant in tmux:

```bash
./run.sh tmux_1to1 warehouse mode:=yolo
```

Default tmux layout is panes:
- Gazebo
- UAV spawn
- localization
- Nav2
- follow

Useful overrides:

```bash
./run.sh tmux_1to1 warehouse gui:=false
./run.sh tmux_1to1 warehouse delay_s:=9
./run.sh tmux_1to1 warehouse spawn_delay_s:=12 follow_delay_s:=18
./run.sh tmux_1to1 warehouse layout:=windows
./run.sh tmux_1to1 warehouse mode:=yolo
./run.sh tmux_1to1 warehouse mode:=yolo use_actual_heading:=false
./run.sh tmux_1to1 warehouse mode:=yolo follow_yaw:=false use_tilt:=false
./run.sh tmux_1to1 warehouse mode:=yolo record:=true record_profile:=vision
./run.sh tmux_1to1 warehouse rtf:=2.0
./run.sh tmux_1to1 warehouse tmux_attach:=false
./run.sh tmux_1to1 warehouse dry_run:=true
```

Default startup is staggered with short delays so the later launches do not all fire at the same instant.

Default delays depend on `gui:=true|false`:
- `gui:=false` uses `spawn=8`, `localization/nav2=10`, and `follow=12`
- `gui:=true` uses `spawn=6`, `localization/nav2=8`, and `follow=10`

If your machine is slower, increase the delay args:
- `delay_s:=...`
- `spawn_delay_s:=...`
- `localization_delay_s:=...`
- `nav2_delay_s:=...`
- `follow_delay_s:=...`

If you want separate tmux windows instead of the default panes:

```bash
./run.sh tmux_1to1 warehouse layout:=windows
```

Alias:

```bash
./run.sh tmux_1to1 warehouse panes:=true
```

Pane layout is:
- row 1: Gazebo | spawn
- row 2: localization | Nav2
- row 3: follow

To stop the tmux-managed stack cleanly, use:

```bash
./stop.sh tmux_1to1 warehouse
```

This sends `Ctrl-c` in this order:
- follow, localization, and Nav2 together
- wait `5s`
- Gazebo
- spawn is expected to exit automatically when Gazebo goes down

Then it waits a few seconds, kills the tmux session, performs a safety cleanup pass for leftover Gazebo, launch, and child stack node processes, and clears stale helper state files under `/tmp/halmstad_ws`.
If recording was enabled, the stop flow also stops the recorder pane and cleans up matching `ros2 bag record` processes as fallback.

Useful stop overrides:

```bash
./stop.sh tmux_1to1 warehouse group_grace_s:=2
./stop.sh tmux_1to1 warehouse final_grace_s:=8
./stop.sh tmux_1to1 warehouse kill_session:=false
./stop.sh tmux_1to1 session:=halmstad-warehouse-1to1
```

Current baseline:
- this odom-follow path now uses `/<ugv>/amcl_pose_odom`, not raw `/platform/odom`
- `/<ugv>/amcl_pose_odom` is synthesized from `/<ugv>/amcl_pose` by [pose_cov_to_odom.py](src/lrs_halmstad/lrs_halmstad/sim/pose_cov_to_odom.py)
- when the Clearpath sim odom / TF path is late during startup, the active follow launch also synthesizes `/<ugv>/platform/odom`, `/<ugv>/platform/odom/filtered`, and fallback `odom -> base_link` TF from `/<ugv>/amcl_pose`
- launch `leader_odom_topic` / `ugv_odom_topic` defaults are intentionally pointed at that AMCL-derived topic
- current UAV camera mode is detached: `uav_camera_mode:=detached_model`
- current camera defaults are `pan_enable: true` and `tilt_enable: true`
- attached mode is still available as an override with `camera:=attached`
- if you override the mount pitch for attached mode, pass the same value to follow:

```bash
./run.sh spawn_uav warehouse uav_name:=dji0 camera:=attached mount_pitch_deg:=35
./run.sh 1to1_follow warehouse camera:=attached mount_pitch_deg:=35
```

To test the older attached camera path instead of the detached default:

```bash
./run.sh spawn_uav warehouse camera:=attached
./run.sh 1to1_follow warehouse camera:=attached
```

Important:
- pass the same camera mode to both spawn and follow
- detached mode does not rely on the attached `45 deg` mount pitch baseline
- the wrapper alias also accepts `camera:=attached`

Important runtime note:
- Gazebo sim time is guarded through [clock_guard.py](src/lrs_halmstad/lrs_halmstad/sim/clock_guard.py)
- expected `/clock` publisher is `clock_guard`
- if Gazebo is restarted or the world is reset, restart localization, Nav2, and follow

## Start The Camera Views

Saved multi-camera / debug layout:

```bash
./run.sh rqt_perspective
```

Then under `Perspectives`, choose one from the `perspectives/` folder if needed.

UAV camera: ``/dji0/camera0/image_raw``

UGV camera: ``/a201_0000/sensors/camera_0/color/image``

YOLO debug image: ``/coord/leader_debug_image``

Overlay fields:
- `det_state=<state> reason=<reason> src=<detector|tracker|none> conf=<...> cls=<...>`
- `track_id=<id|none> state=<raw|reacquire|tracked|na> hits=<...> age_s=<...> switched=<...>`
- `est_range=<...> src=<depth|ground|const|none> bearing=<...>`
- `heading_src=<...>`
- `est=(x,y,yaw)`
- `err=(dx,dy,planar)`

If truth comparison is disabled or stale, `err` shows `na`.


Only use the YOLO debug image when you started the YOLO flow from the section below.

## Control Height And Distance During Runtime

These work while `./run.sh 1to1_follow` or `./run.sh 1to1_yolo` is running.

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

The moved helper wrappers are available through `./run.sh <name>` and still
exist under `./scripts/run_<name>.sh` if you want direct file completion.

```bash
./run.sh follow_control
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
./run.sh follow_control
./run.sh follow_control --step-z-alt 2 --step-d-target 2
./run.sh follow_control --mode params --step-z-alt 2 --step-d-target 2
./run.sh follow_control --mode random --interval 8
./run.sh follow_control --mode random --count 20
./run.sh follow_control --mode random --seed 42
./run.sh follow_control --mode random --interval 8 --count 20 --seed 42
./run.sh follow_control --mode random --focus-weight 0.8
```

## Record A Run

Recommended recorder path through tmux:

```bash
./run.sh tmux_1to1 warehouse mode:=yolo record:=true
```

Add image topics too:

```bash
./run.sh tmux_1to1 warehouse mode:=yolo record:=true record_profile:=vision
```

Small Step 2/3 proof bag:

```bash
./run.sh tmux_1to1 warehouse mode:=yolo record:=true record_profile:=step2_light
```

Recommended Step 2 proof run:

```bash
./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=true obb:=false weights:=warehouse_v1-v2-yolo26n.pt use_estimate:=false start_visual_follow_controller:=true record:=true record_profile:=vision delay_s:=12
```

Recommended planner-enabled bridge run:

```bash
./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=true obb:=true weights:=warehouse-v1-yolo26n-obb.pt use_estimate:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true record:=true record_profile:=step2_light record_tag:=planner_phase delay_s:=12
```

Use this stable bridge mode first when validating planner behavior:
- `use_estimate:=false` keeps the planner + bridge audit focused on the filter/estimator/follow-point/planner/bridge path instead of mixing in extra estimate-driven camera instability too early
- current tuned defaults reduce area-mode forward aggressiveness, allow smaller yaw corrections to reach the body, and decay commands faster on short loss / no-distance cases
- `leader_estimator` now uses live camera tilt/yaw topics when available:
  - `/<uav>/follow/actual/tilt_deg`
  - `/<uav>/camera/actual/world_yaw_rad`
- recent stable bridge proof runs meaningfully exercised projected range in this mode, so this remains the recommended projected-range validation command
- recent stable planner proof runs showed:
  - `follow_point_generator` active and publishing consistently
  - `follow_point_planner` active and publishing consistently
  - `visual_actuation_bridge` consuming `active_input=planned_target`
  - bounded motion through the planner-backed world-frame target

Direct recorder wrapper:

```bash
./run.sh record_experiment warehouse mode:=yolo profile:=vision
./run.sh record_experiment warehouse mode:=yolo profile:=step2_light
```

Default recorder output:
- `bags/experiments/<world>/<timestamp>_<mode>/`

Recorder topic groups:
- default:
  - `/clock`
  - `/coord/events`
  - `/<ugv>/amcl_pose_odom`
  - `/<uav>/pose`
  - `/<uav>/pose_cmd`
  - `/<uav>/pose_cmd/odom`
  - `/<uav>/camera/actual/center_pose`
  - `/<uav>/camera/target/center_pose`
  - `/<uav>/follow/target/anchor_pose`
  - `/<uav>/follow/error/xy_distance_m`
  - `/<uav>/follow/error/yaw_rad`
- YOLO mode adds:
  - `/coord/leader_detection`
  - `/coord/leader_detection_status`
  - `/coord/leader_estimate`
  - `/coord/leader_estimate_status`
  - `/coord/leader_estimate_error`
  - `/coord/leader_selected_target`
  - `/coord/leader_selected_target_filtered`
  - `/coord/leader_selected_target_filtered_status`
  - `/coord/leader_visual_target_estimate`
  - `/coord/leader_visual_target_estimate_status`
  - `/coord/leader_follow_point`
  - `/coord/leader_follow_point_status`
  - `/coord/leader_planned_target`
  - `/coord/leader_planned_target_status`
  - `/coord/leader_visual_control`
  - `/coord/leader_visual_control_status`
- `profile:=step2_light` records only:
  - `/clock`
  - `/coord/events`
  - `/coord/leader_detection_status`
  - `/coord/leader_selected_target`
  - `/coord/leader_selected_target_filtered`
  - `/coord/leader_selected_target_filtered_status`
  - `/coord/leader_visual_target_estimate`
  - `/coord/leader_visual_target_estimate_status`
  - `/coord/leader_follow_point`
  - `/coord/leader_follow_point_status`
  - `/coord/leader_planned_target`
  - `/coord/leader_planned_target_status`
  - `/coord/leader_visual_control`
  - `/coord/leader_visual_control_status`
  - `/coord/leader_visual_actuation_bridge_status`
  - `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
  - `/<uav>/pose_cmd`
  - `/<uav>/pose`
- `profile:=vision` also adds:
  - `/<uav>/camera0/image_raw`
  - `/<uav>/camera0/camera_info`
  - `/coord/leader_debug_image`
  - `/coord/leader_visual_control_debug_image`

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
./run.sh capture_dataset warehouse
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
./run.sh capture_dataset warehouse class:=car
./run.sh capture_dataset warehouse hz:=1.0
./run.sh capture_dataset warehouse out:=datasets/warehouse_auto_v2
```

If you want to keep dataset campaigns separate:

```bash
./run.sh capture_dataset warehouse out:=datasets/warehouse_run2
```

## Run The Sim With A YOLO Model

Start the normal sim first:

```bash
./run.sh gazebo_sim warehouse
./run.sh spawn_uav warehouse
./run.sh localization warehouse
./run.sh nav2
```

Then start the YOLO follow flow:

```bash
./run.sh 1to1_yolo warehouse
```

Tracker variant:

```bash
./run.sh 1to1_yolo warehouse tracker:=true
```

Tracker variant with explicit config:

```bash
./run.sh 1to1_yolo warehouse tracker:=true tracker_config:=botsort.yaml
```

Live follow control works here too:

```bash
./run.sh follow_control
```

Optional Step 2 visual-follow test output:

```bash
./run.sh 1to1_yolo warehouse start_visual_follow_controller:=true
./run.sh tmux_1to1 warehouse gui:=false mode:=yolo start_visual_follow_controller:=true
```

This keeps the current estimate-follow path untouched and additionally publishes a structured test command on `/coord/leader_visual_control`.
The estimator also publishes the clean controller-facing selected target on `/coord/leader_selected_target`.
The Step 3 trust/smoothing layer republishes the controller input on `/coord/leader_selected_target_filtered`.
The Sensors-inspired estimator-upgrade stage publishes the camera-relative control state on `/coord/leader_visual_target_estimate`.
Readable controller status is mirrored on `/coord/leader_visual_control_status`.
The visual test controller:
- prefers `/coord/leader_visual_target_estimate` as the controller-facing input
- keeps `/coord/leader_selected_target_filtered` for debug/fallback only
- consumes a camera-relative relative-state estimate built from projected range when valid, otherwise area-derived pseudo-range
- does not command the UAV directly

Planner-enabled visual-follow ownership:

```bash
./run.sh 1to1_yolo warehouse tracker:=true obb:=true weights:=warehouse-v1-yolo26n-obb.pt use_estimate:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true
./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=true obb:=true weights:=warehouse-v1-yolo26n-obb.pt use_estimate:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true
```

When `start_visual_follow_planner:=true` together with `start_visual_actuation_bridge:=true`:
- the filtered-target + visual-target-estimator + follow-point pipeline starts automatically
- the planner stage starts automatically on top of that follow-point pipeline
- the old `follow_uav` / `follow_uav_odom` motion owner is disabled
- `visual_actuation_bridge` becomes the active motion owner
- `camera_tracker` may still use an odom-frame UGV pose to re-aim the camera when the estimator is unavailable; this is camera-only reacquisition support for sim/testing and does not make the bridge chase truth pose
- `follow_point_generator` publishes a world-frame spatial goal on `/coord/leader_follow_point`
- `follow_point_generator` now prefers `/coord/leader_estimate` XY and yaw as the primary behind-target anchor, so the UAV shadows the UGV pose instead of circling the camera-relative target projection
- `follow_point_planner` publishes a planned world-frame pose target on `/coord/leader_planned_target`
- the bridge consumes that planned target as the active upstream motion objective
- the bridge converts the planned target into the existing pose-like ENU command interface on `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- the bridge also mirrors `/<uav>/pose_cmd` for existing camera/debug consumers

Direct reactive controller mode remains available:
- `start_visual_follow_controller:=true` without `start_visual_follow_point_generator:=true`
- in that mode the bridge consumes `/coord/leader_visual_control` directly

Direct follow-point mode remains available:
- `start_visual_follow_point_generator:=true start_visual_actuation_bridge:=true`
- in that mode the bridge consumes `/coord/leader_follow_point` directly without the planner stage

Current planner validation note:
- the stable `use_estimate:=false` bridge path is now the recommended tuning baseline
- recent tuned proof runs improved closed-loop stability and reduced forward overshoot in that mode
- recent stable-mode projected-range proof runs entered `range_src=ground` and produced finite upgraded range estimates
- direct `range <-> area` flapping was not observed in the projected-range proof bag
- projected-range behavior is still not fully characterized for harder estimate-driven runs, so keep that distinction clear
- the current estimator-backed phase is also validated in that same stable mode:
  - `visual_target_estimator` stayed alive in the active control path
  - `visual_follow_controller` spent most of the run in `distance_mode=estimate`
  - fallback to the old direct filtered-target path remained rare
  - bridge-enabled motion stayed bounded and recoverable
- the current planner phase is also minimally validated in that same stable mode:
  - `follow_point_generator` stayed active upstream
  - `follow_point_planner` stayed active
  - bridge status reported `active_input=planned_target` in the meaningful motion portion of the proof run
  - `/coord/leader_planned_target` and `/<uav>/pose_cmd` were published consistently
  - raw follow points moved in larger jumps while the planner clamped planned-target jumps to a smaller bounded step
  - UAV motion stayed bounded while the bridge followed the planned spatial goal

Useful checks:

```bash
ros2 topic echo /coord/leader_selected_target
ros2 topic echo /coord/leader_selected_target_filtered_status
ros2 topic echo /coord/leader_selected_target_filtered
ros2 topic echo /coord/leader_visual_target_estimate_status
ros2 topic echo /coord/leader_visual_target_estimate
ros2 topic echo /coord/leader_follow_point_status
ros2 topic echo /coord/leader_follow_point
ros2 topic echo /coord/leader_planned_target_status
ros2 topic echo /coord/leader_planned_target
ros2 topic echo /coord/leader_visual_control_status
ros2 topic echo /coord/leader_visual_control
ros2 topic echo /coord/leader_visual_actuation_bridge_status
ros2 topic echo /dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw
ros2 topic echo /dji0/pose_cmd
```

Current stable proof command for the planner phase:

```bash
./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=true obb:=true weights:=warehouse-v1-yolo26n-obb.pt use_estimate:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true record:=true record_profile:=step2_light record_tag:=planner_phase delay_s:=12
```

Harder estimate-driven planner probe:

```bash
./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=true obb:=true weights:=warehouse-v1-yolo26n-obb.pt use_estimate:=true use_actual_heading:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true record:=true record_profile:=step2_light record_tag:=planner_estimate_fix delay_s:=12
```

What that harder probe currently means:
- the planner no longer holds stale planned targets forever; short-loss `HOLD` now times out cleanly into `INVALID`
- in this harder mode, the main remaining weakness is upstream perception/estimate availability rather than planner ownership
- recent proof stats showed detector `NO_DET` dominating, with filtered target / visual-target estimate mostly `INVALID`
- so the planner path itself is behaving more honestly now, but the less-assisted visual regime still needs runtime hardening

Current harder-case hardening proof:

```bash
./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=true obb:=true weights:=warehouse-v1-yolo26n-obb.pt use_estimate:=true use_actual_heading:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true record:=true record_profile:=step2_light record_tag:=robustness_hardening3 delay_s:=12
```

What changed in that hardening pass:
- `selected_target_filter` now allows bounded low-confidence reacquire from very recent plausible history instead of always dropping to invalid on those weak returns
- `camera_tracker` now keeps the last trackable leader pose alive briefly so camera pan does not freeze instantly on short estimator dropouts
- the tuned defaults now give the filter/estimator slightly longer short-gap prediction windows

What that harder proof showed relative to the earlier `planner_estimate_fix` bag:
- filtered target `VALID`: `64 -> 92`
- visual-target estimate `MEASURED+PREDICTED`: `73 -> 110`
- follow-point `ACTIVE+HOLD`: `91 -> 130`
- planner `ACTIVE+HOLD`: `126 -> 167`
- bridge `ACTIVE`: `123 -> 167`
- bridge `active_input=planned_target`: `143 -> 180`
- bridge `active_input=control`: `378 -> 193`

What that means:
- the current visual stack now survives short ugly segments better in the harder estimate-driven mode
- the planner-backed path stays active materially longer before falling back to stale-control hold
- the remaining weakness is now more clearly real visibility / geometry loss in close-range or under-UAV scenes, not a stale planner hold bug or immediate short-loss collapse

Latest shadow-follow hardening proof:

```bash
./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=true obb:=true weights:=warehouse-v1-yolo26n-obb.pt camera:=detached use_estimate:=true use_actual_heading:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true record:=true record_profile:=step2_light record_tag:=shadow_follow_fix3 delay_s:=12
```

What changed in that shadow-follow pass:
- `follow_point_generator` now keeps a recent behind-target heading briefly when motion weakens, instead of relying only on the UAV-to-target view line
- `follow_point_generator` also prefers the estimator's world-frame target pose and yaw as the primary behind-target anchor in the stable planner-backed path
- follow-point, planner, and bridge progression are all more conservative so the UAV shadows more slowly
- bridge `input_mode=auto` is now safe:
  - it uses `planned_target`, then `follow_point`
  - otherwise it holds
  - it no longer falls back silently to raw controller commands

What that proof showed:
- bridge `active_input=control`: `193 -> 0`
- bridge `active_input=planned_target`: `180 -> 126`
- bridge mean `step_xy_m`: `0.1097 -> 0.0649`
- UAV XY path length over the proof window: `41.26 m -> 15.11 m`

What that means:
- the hard no-odom-style path is still not finished, because visibility / close-range geometry loss still exists upstream
- but the planner-backed path now degrades much more safely instead of flying off on a lower-level fallback when the visual chain weakens

Current stable behind-target note:
- the stable `use_estimate:=false` planner-backed path now prefers the estimated UGV world pose as the follow-point anchor
- when that anchor is fresh, `/coord/leader_follow_point_status` reports `policy_mode=target_pose_heading`
- planner and bridge still stay on the same planned-target pipeline above it
- if the upstream target estimate drops out, the follow-point stage can still go through short `HOLD` / `INVALID`, so the remaining problem is still upstream availability rather than the behind-target policy itself

Stable planner-backed path recovery:

- after the shadow-follow safety hardening, the stable `use_estimate:=false` path briefly regressed because:
  - `uav_simulator` could starve `/<uav>/pose` while detached-camera pose-service futures stayed pending
  - `leader_estimator` was treating detections as stale based on source-frame stamp rather than receive freshness
- both were fixed:
  - `uav_simulator` now keeps publishing `/<uav>/pose` and clears stuck detached-camera pose futures after a short timeout
  - `leader_estimator` now gates detection freshness on receive time while still reporting source latency
- with those fixes, the stable planner command again produces active:
  - selected-target / estimate flow
  - follow-point generation
  - planner output
  - bridge output
  - visible UAV motion

Runtime turn analysis:

```bash
./run.sh follow_debug
```

This subscribes to the follow, estimator, and camera topics and prints an interpreted diagnosis such as:
- camera-only pan is moving
- UAV body yaw command is moving
- both camera and body are moving
- suspicious motion while estimator state is `NO_DET` / `REJECT`
- logs are also saved by default under `debug_logs/follow_debug/`

For YOLO mode, detailed follow-debug topics are now muted by default. Re-enable them if needed:

```bash
./run.sh 1to1_yolo warehouse publish_follow_debug_topics:=true publish_pose_cmd_topics:=true publish_camera_debug_topics:=true
```

Default `run.sh follow_control` mode is keyboard tuning for `d_target` and `z_alt`. If you want the same parameter path explicitly, use:

```bash
./run.sh follow_control --mode params
```

Default behavior:
- Nav2 still drives the UGV through the current `ugv_nav2_driver` path
- the UAV/camera runtime uses `leader_estimate` rather than shared UGV pose
- estimate-mode YOLO repositions the UAV to the seeded startup pose before the first estimate arrives
- through `scripts/run_1to1_yolo.sh`, that startup pose defaults to `uav_start_x:=-7.0` and `uav_start_z:=7.0` in non-solar worlds
- the shared follow/camera fallback tilt baseline stays at `-45.0`
- held-estimate reuse is disabled by default, so rejected/missing detections now surface directly as `REJECT` / `NO_DET` instead of `*_HOLD`
- `./run.sh 1to1_yolo ...` now defaults to `obb:=true`
- with no explicit weights override, the current default weight is:
  - `obb/mymodels/warehouse-v1-yolo26n-obb.pt`
- wrapper launch defaults also force:
  - `leader_range_mode:=auto`
- truth/error and legacy debug topics are muted by default in the YOLO wrapper:
  - `leader_actual_pose_enable:=false`
  - `publish_follow_debug_topics:=false`
  - `publish_pose_cmd_topics:=false`
  - `publish_camera_debug_topics:=false`
- estimate-mode YOLO also enables shared actual heading by default unless you override it:
  - `leader_actual_heading_enable:=true`
  - use `use_actual_heading:=false` if you want motion/estimate heading behavior instead
- direct `ros2 launch lrs_halmstad run_follow.launch.py ...` does not inherit that quiet wrapper policy automatically
- bare custom weight names resolve like this:
  - `tracker:=false` -> under `models/detection/...`
  - `tracker:=true` -> under `models/obb/...`

Current live issue under investigation:
- estimate-follow is much better than before, but the close-range case is still the active problem
- when the UGV tries to pass underneath or cross the UAV path, the estimator/follow stack can still hold the wrong range or recover too slowly
- camera/body centering and target-distance recovery are the main things to watch

First topics to watch during that event:
- `/coord/leader_estimate_status`
- `/coord/leader_debug_image`
- `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- `/<uav>/update_pan`
- `/<uav>/update_tilt`

Truth-assisted test mode:

```bash
./run.sh 1to1_yolo warehouse use_estimate:=false
```

This keeps the YOLO estimator running, but switches the UAV/camera path back to shared UGV pose for comparison/testing.

Warehouse `car` filter:

```bash
./run.sh 1to1_yolo warehouse target:=car
```

Different YOLO weights:

```bash
./run.sh 1to1_yolo warehouse weights:=yolo26v2.pt
```

Detection-family shorthand:

```bash
./run.sh 1to1_yolo warehouse folder:=yolo26 weights:=yolo26s.pt
```

OBB-family default:

```bash
./run.sh 1to1_yolo warehouse obb:=true
```

Return to the plain detection-family default:

```bash
./run.sh 1to1_yolo warehouse obb:=false
```

OBB-family with explicit subfolder:

```bash
./run.sh 1to1_yolo warehouse obb:=true folder:=yolo26 weights:=yolo26s-obb.pt
```

Estimate error topic:

```bash
ros2 topic echo /coord/leader_estimate_error
```

This topic is muted by default in the current YOLO path. Re-enable it with `leader_actual_pose_enable:=true` if you want estimate-vs-truth diagnostics against `/<ugv>/amcl_pose_odom`.

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

## Scan A New Map With RViz

Use this when the current world does not already have a good Nav2 map.

1. Start Gazebo:

```bash
./run.sh gazebo_sim warehouse
```

2. Start SLAM:

```bash
./run.sh slam
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
./run.sh nav2_rviz
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

### `./run.sh gazebo_sim`

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
  - `rtf:=...`
  - `real_time_factor:=...`

### `./run.sh spawn_uav`

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

### `./run.sh localization`

- positional `WORLD`
  - current tested example: `warehouse`
- optional second positional argument: map path
  - path to the `.yaml` occupancy map
  - example: `maps/warehouse_manual.yaml`
- extra forwarded launch arguments
  - anything else is passed through to `localization.launch.py`

### `./run.sh nav2`

- extra forwarded launch arguments
  - anything else is passed through to `nav2.launch.py`

### `./run.sh 1to1_follow`

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
  - default is `true`
- extra forwarded launch arguments
  - anything else is passed through to `run_follow.launch.py`
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

### `./run.sh 1to1_yolo`

- positional `WORLD`
  - current tested example: `warehouse`
- `camera:=attached|detached`
  - shorthand for `uav_camera_mode:=integrated_joint|detached_model`
- `weights:=...`
  - maps to `yolo_weights:=...`
  - bare filenames resolve under `detection/...` when `tracker:=false`
  - bare filenames resolve under `obb/...` when `tracker:=true`
  - example: `weights:=yolo26v2.pt`
- `folder:=...`
  - selects a subfolder under `detection/` or `obb/`
  - example: `folder:=yolo26`
- `obb:=true|false`
  - default `true`
  - with no explicit `weights:=...`, default weight is `obb/mymodels/warehouse-v1-yolo26n-obb.pt`
  - `false`: default weights switch to `detection/mymodels/warehouse_v1-v2-yolo26n.pt`
- `height:=...`
  - maps to `uav_start_z:=...`
- `mount_pitch_deg:=...`
  - maps to `camera_mount_pitch_deg:=...`
- `use_tilt:=true|false`
  - maps to `tilt_enable:=true|false` for `camera_tracker`
- `follow_yaw:=true|false`
  - enable or disable UAV body yaw following
  - default is `true`
- `camera_default_tilt_deg:=...`
  - forwarded through to launch if you want to override the shared fallback tilt
- `target:=...`
  - maps to `target_class_name:=...`
  - example: `target:=car`
- `use_estimate:=true|false`
  - default `true`: estimate-only YOLO follow path
  - `false`: truth-assisted test mode; still runs the estimator, but shares UGV pose into the follow/camera path
- `leader_actual_pose_enable:=true|false`
  - default `false` in `./run.sh 1to1_yolo`
  - enable `/coord/leader_estimate_error` truth diagnostics
- `use_actual_heading:=true|false`
  - default effective wrapper behavior in estimate mode is `true`
  - pass `false` if you want to avoid using shared AMCL heading in follow
- `publish_follow_debug_topics:=true|false`
  - default `false` in `./run.sh 1to1_yolo`
  - enable `/<uav>/follow/{target,actual,error,debug}/*`
- `publish_pose_cmd_topics:=true|false`
  - default `false` in `./run.sh 1to1_yolo`
  - enable legacy `/<uav>/pose_cmd` and `/<uav>/pose_cmd/odom`
- `publish_camera_debug_topics:=true|false`
  - default `false` in `./run.sh 1to1_yolo`
  - enable `/<uav>/camera/target/*`
- `start_visual_follow_controller:=true|false`
  - default `false`
  - enable the optional `/coord/leader_visual_control*` Step 2 test output without changing UAV actuation
- `start_visual_actuation_bridge:=true|false`
  - default `false`
  - enable the bridge that makes the visual-follow path the active real UAV motion owner
- `leader_selected_target_topic:=...`
  - default `/coord/leader_selected_target`
  - override the raw selected-target topic published by the estimator if needed
- `leader_selected_target_filtered_topic:=...`
  - default `/coord/leader_selected_target_filtered`
  - override the Step 3 filtered selected-target topic if needed
- `leader_selected_target_filtered_status_topic:=...`
  - default `/coord/leader_selected_target_filtered_status`
  - override the Step 3 filtered-target status topic if needed
- `leader_visual_actuation_bridge_status_topic:=...`
  - default `/coord/leader_visual_actuation_bridge_status`
  - override the bridge status topic if needed
- `tracker:=true|false`
  - default `false`
  - `true` switches `external_detection_node:=tracker`
- `tracker_config:=...`
  - current default from `run_follow.launch.py` is `botsort.yaml`
  - bare filenames resolve under `src/lrs_halmstad/config/trackers`
- extra forwarded launch arguments
  - anything else is passed through to `run_follow.launch.py`


### `./run.sh capture_dataset`

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
./run.sh nav2_rviz
```

- open the saved multi-camera / debug `rqt` layout:

```bash
./run.sh rqt_perspective
```

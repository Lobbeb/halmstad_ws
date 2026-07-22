# lrs_halmstad

Reference for the current 1-to-1 Gazebo/ROS 2 workflow.

For repo-level conventions and handoff notes, also read `../../AGENTS.md`.

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
- `/dji0/update_tilt` - external tilt command in degrees
- `/dji0/update_pan` - external pan command in degrees
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

Optional typed support-hazard topics:
- `/coord/support/dji1/aerial_hazards`
- `/coord/support/dji2/aerial_hazards`
- `/coord/dji0/aerial_hazards`
- `/coord/ugv/aerial_hazards`

The typed path is disabled by default. Enable its map-frame association/fusion and UGV forwarding explicitly when running the support overlay:

```bash
./run.sh support_observation baylands hazard_fusion_enable:=true hazard_forward_enable:=true
```

The path validates age, TTL, dimensions, confidence, and covariance, associates class-compatible observations in time and XY, and assigns deterministic dji0 track IDs. First evidence is tentative; repeated single-UAV evidence or consistent two-UAV evidence confirms a track; incompatible overlapping cross-UAV evidence is marked conflict. Estimate selection remains conservative: one fresh acceptable source is forwarded without averaging or covariance reduction. Typed dji2 evidence is disabled unless `hazard_fusion_dji2_enable:=true` is supplied. The Baylands global costmap can consume the typed UGV output only when explicitly enabled below.

## Task 4: interactive synthetic hazard validation

This is an operator-run simulation workflow. It does not establish runtime success until its evidence has been reviewed. The existing C1-C4 behavior remains unchanged because both the typed chain and the aerial layer default to disabled.

Start a Baylands stack with the typed chain, aerial layer, and the no-image hazard rosbag profile enabled, but without a synthetic source. This gives a stable baseline before injecting a hazard:

```bash
./run.sh tmux_support_chain baylands \
  hazard_chain_enable:=true aerial_support_layer_enable:=true \
  record:=true record_profile:=support_hazard record_tag:=aerial_baseline
```

After the Nav2 route is visible, capture the baseline in another sourced terminal:

```bash
./run.sh verify_aerial_support_chain phase:=baseline
```

For an outside-route control, start a second support-chain session only when the first has been stopped, or use this as the first command instead of the baseline command. The configured point is outside the documented `parkinglot_west` route corridor:

```bash
./run.sh tmux_support_chain baylands \
  hazard_chain_enable:=true hazard_synthetic_enable:=true \
  aerial_support_layer_enable:=true record:=true record_profile:=support_hazard \
  record_tag:=hazard_outside \
  hazard_synthetic_x:=-20.0 hazard_synthetic_y:=230.0 \
  hazard_synthetic_active_duration_s:=15.0 hazard_synthetic_ttl_s:=4.0
```

For the route-intersection case, use the same command with the documented `parkinglot_west` route and the map-frame point near its segment between waypoints 1 and 2:

```bash
./run.sh tmux_support_chain baylands \
  nav2_goals:=parkinglot_west hazard_chain_enable:=true \
  hazard_synthetic_enable:=true aerial_support_layer_enable:=true \
  record:=true record_profile:=support_hazard record_tag:=hazard_on_route \
  hazard_synthetic_x:=-72.0 hazard_synthetic_y:=195.5 \
  hazard_synthetic_size_x:=2.0 hazard_synthetic_size_y:=2.0 \
  hazard_synthetic_active_duration_s:=15.0 hazard_synthetic_ttl_s:=4.0
```

For stale-message rejection, keep the chain and layer enabled but use a timestamp that is two seconds behind simulation time. The typed fusion should reject it, so this is a rejection check rather than a costmap-marking test:

```bash
./run.sh tmux_support_chain baylands \
  hazard_chain_enable:=true hazard_synthetic_enable:=true \
  aerial_support_layer_enable:=true \
  hazard_synthetic_stamp_offset_s:=-2.0 hazard_synthetic_active_duration_s:=15.0
```

For the on-route scenario, run the bounded observer while the hazard is active, then after the publisher changes to an empty array:

```bash
./run.sh verify_aerial_support_chain phase:=hazard
./run.sh verify_aerial_support_chain phase:=expiry timeout_s:=20
```

The `support_hazard` rosbag profile records the four typed hazard topics, global costmap, global plan, AMCL pose, NavigateToPose action status, and `/coord/events`; it does not add raw image topics. Return the three verifier reports under `/tmp/halmstad_ws/aerial_support_validation/`, terminal logs, the rosbag run directory, a global-costmap screenshot, and before/after plan snapshots.

## Task 5: dji1 RGB-D hazard projection

The real projector is opt-in and dji1-only. The tmux wrapper enables its required simulation-localization TF, Gazebo gimbal-command bridge, and typed dji1→dji0→UGV chain when `hazard_projector_enable:=true`, and disables the legacy dji2 support slot for this validation; the synthetic source remains available but cannot be enabled at the same time. The aerial costmap layer remains disabled unless `aerial_support_layer_enable:=true` is supplied.

The verified simulation contract is:

- RGB `/dji1/camera0/image_raw`, `sensor_msgs/Image`, `rgb8`;
- aligned depth `/dji1/camera0/depth_image`, `sensor_msgs/Image`, `32FC1` metres, with `+inf` for invalid pixels;
- CameraInfo `/dji1/camera0/camera_info`;
- optical frame `dji1/camera0/image_optical_frame`, using ROS +x-right, +y-down, +z-forward axes;
- timestamped Gazebo-world poses on `/dji1/pose` and `/dji1/camera0/actual/center_pose`;
- TF chain `map → dji1/base_link → dji1/camera0/image_optical_frame` at the source acquisition timestamp.

The Baylands support-observation wrapper defaults to the available Baylands checkpoint `models/obb/mymodels/baylands-leader-v4-3.pt` with the Ultralytics backend. Its verified class is `ugv`; this is a geometry and transport fixture, not a validated environmental-hazard detector.

World→map is a simulation-only planar calibration fitted from the existing `parkinglot_west` world/AMCL waypoint pairs. `parkinglot_west_0` is held out: the current fit has 1.152 m maximum fit residual and 0.578 m held-out XY error, below the configurable 1.25 m and 0.75 m gates. This calibration is not a claim that the full Baylands raster is globally rigid, and it must not be reused outside its calibrated route without new evidence.

Start the geometry and typed-chain validation without changing the global costmap:

```bash
./run.sh tmux_support_chain baylands mode:=follow \
  hazard_projector_enable:=true \
  record:=true record_profile:=support_hazard record_tag:=task5_rgbd
```

For an explicit downstream-layer observation, add `aerial_support_layer_enable:=true`. Use that only after confirming the selected class is appropriate: the currently verified existing weights detect class `ugv`, so this first proof validates RGB-D geometry and transport, not a trained environmental-hazard detector. No detection accuracy, precision, recall, or mAP claim is made.

Useful evidence commands from a second sourced terminal are:

```bash
ros2 topic echo --once /coord/support/dji1/hazard_detection
ros2 topic echo --once /coord/support/dji1/aerial_hazards
ros2 topic echo --once /coord/dji0/aerial_hazards
ros2 topic echo --once /coord/ugv/aerial_hazards
ros2 run tf2_ros tf2_echo map dji1/camera0/image_optical_frame -t <acquisition_stamp_seconds>
```

The `support_hazard` bag remains image-free but now also records dji1 hazard detector metadata, dji1 simulation pose contracts, and `/tf`. Use a separate, short diagnostic bag for RGB/depth/CameraInfo only when timestamp or depth debugging is required.

## Task 6: multi-source association and confirmation

Task 6 upgrades only the typed dji0 fusion stage. It keeps at most one latest observation per source and a bounded number of fused tracks; it does not retain unbounded observation history. Association requires a compatible class and timestamp window, then uses an XY Mahalanobis gate when the covariance is usable or a Euclidean fallback when it is singular. `hazard_fusion_association_require_footprint_overlap:=true` adds an oriented XY-footprint overlap requirement.

The initial parameters are:

- `hazard_fusion_dji2_enable:=false`
- `hazard_fusion_association_time_window_s:=0.75`
- `hazard_fusion_association_chi2_xy:=5.991`
- `hazard_fusion_association_max_distance_m:=1.5`
- `hazard_fusion_association_require_footprint_overlap:=false`
- `hazard_fusion_confirmation_window_s:=2.0`
- `hazard_fusion_single_source_confirm_hits:=2`
- `hazard_fusion_track_timeout_s:=2.0`
- `hazard_fusion_max_track_count:=256`
- `hazard_fusion_conflict_max_dimension_ratio:=3.0`
- quality weights: confidence `0.35`, freshness `0.25`, uncertainty `0.25`, view quality `0.15`; the four weights must sum to one.

There is no communication-quality term in Task 6. The selected source covariance is copied unchanged; agreement affects confirmation state and source attribution, not numerical precision.

For a bounded, non-simulation two-source typed check, use four sourced terminals. This uses deterministic publishers and does not require a live dji2 vehicle:

```bash
# Terminal 1: fusion, with typed dji2 evidence explicitly enabled
ros2 run lrs_halmstad support_hazard_fusion --ros-args \
  -p use_sim_time:=false -p dji2_enable:=true

# Terminal 2: dji1 evidence
ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args \
  -p use_sim_time:=false \
  -p topic:=/coord/support/dji1/aerial_hazards -p source_uav:=dji1 \
  -p stable_track_id:=dji1_fixture -p class_name:=hazard \
  -p center_x:=4.0 -p center_y:=5.0

# Terminal 3: consistent dji2 evidence for the same object
ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args \
  -p use_sim_time:=false \
  -p topic:=/coord/support/dji2/aerial_hazards -p source_uav:=dji2 \
  -p stable_track_id:=dji2_fixture -p class_name:=hazard \
  -p center_x:=4.2 -p center_y:=5.1

# Terminal 4: expect one stable CONFIRMED dji0 track with both source_uavs
ros2 topic echo /coord/dji0/aerial_hazards --once
```

An optional/heavy live two-UAV Baylands run must explicitly opt in to both the legacy support slot and typed fusion with `dji2_enable:=true hazard_fusion_dji2_enable:=true`. Task 6 does not require that run and does not add a real dji2 RGB-D projector. No detector-accuracy, environmental-hazard perception, or closed-loop navigation-success claim follows from the synthetic association check.

## Task 7: communication-aware freshness and source selection

Task 7 preserves Task 6 association, stable dji0 IDs, confirmation/conflict handling, bounded track storage, complete historical `source_uavs`, and conservative selection of one source estimate. It scores each fresh candidate as:

`w_confidence * confidence + w_freshness * freshness + w_uncertainty * uncertainty + w_view * support_quality + w_communication * configured_communication_quality`

Freshness is based on the source acquisition stamp in `Detection3D.header.stamp`, not dji0 receipt or publication time. Evidence older than `hazard_fusion_max_source_age_s` is rejected. Cached evidence is also removed when its source has not delivered a callback within `hazard_fusion_source_timeout_s`; another fresh source can keep the track alive, while stale evidence cannot retain CONFIRMED or CONFLICT state. Tracks still expire using the Task 6 track timeout.

The existing `/omnet/*` bridge describes one aggregate dji0-to-UGV link and does not identify or timestamp dji1/dji2 source links. Fusion therefore does not subscribe to it or invent live network awareness. The communication term is the explicit simulation/configuration input `clamp(source_quality - source_penalty, 0, 1)`. Its default weight is zero, so the default behavior is Task 6's ideal/no-metrics mode.

New parameters and defaults are:

- `hazard_fusion_max_source_age_s:=0.75`
- `hazard_fusion_source_timeout_s:=0.75`
- `hazard_fusion_quality_weight_communication:=0.0`
- `hazard_fusion_selection_score_epsilon:=0.01`
- `hazard_fusion_dji1_communication_quality:=1.0`
- `hazard_fusion_dji2_communication_quality:=1.0`
- `hazard_fusion_dji1_communication_penalty:=0.0`
- `hazard_fusion_dji2_communication_penalty:=0.0`
- `hazard_fusion_diagnostic_period_s:=5.0`

All five quality weights must sum to one. Scores within `hazard_fusion_selection_score_epsilon` use stable source order (`dji1`, then `dji2`) and output tracks remain sorted by deterministic dji0 ID. The selected source covariance is deep-copied verbatim; neither multi-source agreement nor communication quality shrinks it. Compact periodic logs report selected-source counts, stale rejection, dropped/expired evidence, source timeout, no-fresh-source transitions, expired tracks, and configured communication values.

For a bounded non-simulation source-selection check, use the Task 6 four-terminal fixture but start fusion with typed dji2 explicitly enabled and nonzero configured communication weight:

```bash
# Terminal 1: weights sum to 1.0; dji2 is intentionally penalized
ros2 run lrs_halmstad support_hazard_fusion --ros-args \
  -p use_sim_time:=false -p dji2_enable:=true \
  -p quality_weight_confidence:=0.25 -p quality_weight_freshness:=0.25 \
  -p quality_weight_uncertainty:=0.20 -p quality_weight_view:=0.15 \
  -p quality_weight_communication:=0.15 \
  -p dji1_communication_quality:=1.0 -p dji2_communication_quality:=0.2

# Terminals 2 and 3: use the Task 6 dji1/dji2 synthetic publishers.
# Give dji1 center_x:=4.0 and dji2 center_x:=4.2, then inspect the representative.

# Terminal 4
ros2 topic echo /coord/dji0/aerial_hazards --once
```

Repeat after swapping the two configured communication-quality values to verify that the representative center/covariance changes source without averaging. Source timestamps can be aged deterministically with the synthetic publisher's `stamp_offset_s` parameter to verify rejection and fallback. This test uses replayable typed inputs and does not require live dji2. An optional/heavy Baylands run remains explicitly opt-in with `dji2_enable:=true hazard_fusion_dji2_enable:=true`; it is not a Task 7 pass requirement.

## Task 8: bounded validation and evidence packaging

Task 8 is a post-thesis/future-work prototype harness around the completed Task 5–7 implementation. It supports future closed-loop experiments, but synthetic or typed-flow evidence is not part of the original thesis results and does not establish navigation benefit.

The harness has two entry points:

- `./run.sh validate_support_hazards scenario:=...` starts bounded non-simulation fusion fixtures, applies a timeout, and terminates every node it starts.
- `ros2 run lrs_halmstad support_hazard_evidence live|bag ...` validates live topics or an existing `support_hazard` bag and emits `summary.json`, `summary.csv`, `timeline.csv`, and `timeline.svg`.

Each synthetic run creates a new `bags/validation/support_hazards/<timestamp>_<scenario>/` directory containing the verifier products, process logs, and `manifest.json`. The manifest records every command, scenario parameter, full Git commit SHA, branch, dirty state, dji2 opt-in state, and costmap state. Add `record:=true` to create `recording/bag/`, `recording/topics.txt`, and `recording/metadata.json` through the existing image-free `support_hazard` profile. Existing evidence is never overwritten. `bags/`, logs, rosbag databases, datasets, and Python caches are ignored by Git.

### 1. Task 5 single-UAV RGB-D typed flow — optional live Baylands

This is user-operated and requires the existing Baylands simulation. It keeps dji2 and the aerial costmap off:

```bash
# Terminal 1
./run.sh tmux_support_chain baylands mode:=follow \
  hazard_projector_enable:=true \
  record:=true record_profile:=support_hazard record_tag:=task8_task5_rgbd \
  gui:=false tmux_attach:=true

# Terminal 2, after sourcing ROS and install/setup.bash
EVIDENCE="bags/validation/$(date +%Y%m%dT%H%M%S)_task5_live"
ros2 run lrs_halmstad support_hazard_evidence live \
  --timeout-s 30 --max-age-s 1.0 \
  --expected-sources dji1 --expected-selected-source dji1 \
  --output "$EVIDENCE"
```

Expected evidence: non-empty dji1, dji0, and UGV typed topics; exact dji0→UGV forwarding; a source-matched covariance; acquisition age; selected source; state history; JSON/CSV/timeline files; and the separate `support_hazard` bag. This validates typed transport and the already implemented geometry contract only. Runtime success must be judged from the actual terminal output and bag.

A bounded headless Baylands run on 2026-07-22 recorded detector output, but the sampled records remained `valid:false`. The projector received detector messages but produced zero successful projections and therefore published no dji1 typed hazards; dji0 and the UGV published only empty arrays. This is retained as an optional live-validation limitation, not a Task 8 implementation failure and not evidence of end-to-end runtime success. The generated report and bag remain under ignored `bags/validation/` and `bags/experiments/` paths. Do not infer detector accuracy from this run, and do not change projector geometry solely to force a positive result.

Task 8 completion is based on the deterministic synthetic/replayed typed-message checks below for Task 6 association/confirmation/conflict and Task 7 freshness/source selection/expiry. Optional live Task 5, dji2, and costmap runs provide additional runtime evidence only when their own verifier reports pass.

### 2. Task 6 multi-source association, confirmation, and conflict — deterministic non-simulation

```bash
./run.sh validate_support_hazards \
  scenario:=task6_confirmation record:=true

./run.sh validate_support_hazards \
  scenario:=task6_conflict record:=true
```

Expected confirmation evidence: initial TENTATIVE followed by CONFIRMED, both `source_uavs`, one conservative selected geometry/covariance, and unchanged dji0→UGV forwarding. Expected conflict evidence: two overlapping incompatible tracks in CONFLICT without a crash. These commands use synthetic typed inputs; they do not start Gazebo or require a live dji2 vehicle.

Analyze an existing confirmation bag without replaying or modifying it:

```bash
ros2 run lrs_halmstad support_hazard_evidence bag \
  --bag <confirmation_evidence_dir>/recording/bag \
  --require-dji2 --expected-state CONFIRMED \
  --expected-sources dji1,dji2 --require-confirmation-promotion \
  --output "bags/validation/$(date +%Y%m%dT%H%M%S)_task6_replay"
```

### 3. Task 7 freshness and configured source quality — deterministic non-simulation

```bash
./run.sh validate_support_hazards scenario:=task7_quality record:=true
./run.sh validate_support_hazards scenario:=task7_stale record:=true
./run.sh validate_support_hazards scenario:=task7_expiry record:=true
```

`task7_quality` gives dji1 configured quality `1.0`, dji2 quality `0.0`, and communication weight `0.6`; the expected representative is dji1 while both contributors remain recorded. `task7_stale` supplies dji2 with a two-second-old acquisition stamp and expects dji1-only accepted evidence. `task7_expiry` requires a non-empty UGV hazard followed by an empty array. These values are explicit simulation/configuration inputs, not measured live network metrics.

Offline analysis for an existing quality-selection bag is:

```bash
ros2 run lrs_halmstad support_hazard_evidence bag \
  --bag <quality_evidence_dir>/recording/bag \
  --require-dji2 --expected-state CONFIRMED \
  --expected-sources dji1,dji2 --expected-selected-source dji1 \
  --output "bags/validation/$(date +%Y%m%dT%H%M%S)_task7_replay"
```

### 4. Optional/heavy dji2 Baylands integration

There is no real dji2 RGB-D projector in the completed Task 5 scope. The following is therefore only an opt-in two-vehicle transport/integration run; provide typed dji1/dji2 fixtures separately and do not describe it as two-UAV perception validation:

```bash
./run.sh tmux_support_chain baylands mode:=follow \
  dji2_enable:=true hazard_chain_enable:=true \
  hazard_fusion_dji2_enable:=true \
  record:=true record_profile:=support_hazard record_tag:=task8_dji2_optional \
  gui:=false tmux_attach:=true

# Terminal 2: deterministic dji1 typed fixture
ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args \
  -p use_sim_time:=true -p topic:=/coord/support/dji1/aerial_hazards \
  -p source_uav:=dji1 -p stable_track_id:=task8_live_dji1 \
  -p class_name:=hazard -p center_x:=4.0 -p center_y:=5.0

# Terminal 3: deterministic dji2 typed fixture
ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args \
  -p use_sim_time:=true -p topic:=/coord/support/dji2/aerial_hazards \
  -p source_uav:=dji2 -p stable_track_id:=task8_live_dji2 \
  -p class_name:=hazard -p center_x:=4.2 -p center_y:=5.1

# Terminal 4
ros2 run lrs_halmstad support_hazard_evidence live \
  --timeout-s 30 --require-dji2 --expected-state CONFIRMED \
  --expected-sources dji1,dji2 \
  --output "bags/validation/$(date +%Y%m%dT%H%M%S)_dji2_optional"
```

Expected evidence: dji2 is present only because both opt-in arguments were supplied, typed source arrays reach dji0 and the UGV, and the bag contains the four hazard topics. This command is optional/heavy and is not run by the Task 8 harness.

### 5. Optional aerial-costmap validation

Use the existing synthetic route workflow; the layer must be explicitly enabled:

```bash
# Terminal 1: start the layer and recorder without a hazard source
./run.sh tmux_support_chain baylands nav2_goals:=parkinglot_west \
  hazard_chain_enable:=true \
  aerial_support_layer_enable:=true \
  record:=true record_profile:=support_hazard record_tag:=task8_costmap_optional

# Terminal 2: capture the no-hazard baseline first
./run.sh verify_aerial_support_chain phase:=baseline

# Terminal 3: then inject one bounded on-route source
ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args \
  -p use_sim_time:=true -p topic:=/coord/support/dji1/aerial_hazards \
  -p source_uav:=dji1 -p stable_track_id:=task8_costmap_hazard \
  -p class_name:=hazard -p center_x:=-72.0 -p center_y:=195.5 \
  -p dimension_x:=2.0 -p dimension_y:=2.0 \
  -p active_duration_s:=15.0 -p ttl_s:=4.0

# Terminal 2: capture while active, then after the empty array/TTL clearing
./run.sh verify_aerial_support_chain phase:=hazard
./run.sh verify_aerial_support_chain phase:=expiry timeout_s:=20
```

Expected evidence: layer/plugin parameters, typed receipt, baseline/hazard/expiry costmap hashes, optional plan-change hash, clearing evidence, reports under `/tmp/halmstad_ws/aerial_support_validation/`, and the timestamped bag directory. Topic presence or a changed hash alone is not a closed-loop navigation-success result; inspect the actual run, path, costmap, action status, and UGV outcome.

### Validation boundaries and non-claims

Validated implementation behavior consists of message validation, bounded storage, deterministic association/source selection, state transitions, expiry, source retention, covariance copying, typed forwarding, and optional layer unit tests. Synthetic/replayed validation checks those contracts reproducibly. Optional live validation may provide runtime evidence, but only after its logs and bags are reviewed.

This harness does not claim detector accuracy, general environmental-hazard perception, full SLAM, communication-aware UAV motion planning, closed-loop navigation success, or quantitative safety improvement. It does not create a statistical conclusion before repeated experimental data exists. The existing C1–C4 campaign runner remains unchanged so future-work evidence cannot silently become an original thesis result.

OMNeT++ network metrics topics (published by `omnet_metrics_bridge` when OMNeT is running,
requires `start_omnet_bridge:=true` in `run_follow.launch.py`; all `std_msgs/Float64`, ~10 Hz):

- `/omnet/sim_time`          â€” OMNeT simulation time (s)
- `/omnet/link_distance`     â€” geometric UAVâ€“UGV distance from Gazebo positions (m)
- `/omnet/rssi_dbm`          â€” received signal strength (dBm, free-space path-loss model)
- `/omnet/snir_db`           â€” signal-to-noise-plus-interference ratio (dB)
- `/omnet/packet_error_rate` â€” sliding-window PER estimate (0â€“1)
- `/omnet/radio_distance`    â€” range estimate from RSSI inversion only, no Gazebo positions (m)

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

For `world:=orchard`, the launch defaults now auto-fill the current verified orchard map-frame initial pose, so you do not need to manually publish `/a201_0000/initialpose` for the normal Nav2 bring-up. The recommended orchard Nav2 map is `maps/orchard_nav.yaml`, which keeps black occupied cells and frees gray terrain/unknown cells from the raw SLAM map.

For `world:=baylands`, the launch defaults now auto-fill the current verified Baylands AMCL startup pose and packaged Baylands waypoint route (`config/baylands_waypoints.yaml`). Baylands follow defaults also switch the UAV leader source to `/<ugv>/ground_truth/odom`, which is bridged directly from the Gazebo world pose so the UAV and UGV stay in the same coordinate frame. The actively supported Baylands Nav2 maps are `maps/baylands.yaml` (the current default in `./run.sh localization baylands`) and `maps/baylands_merged.yaml` (pose-compatible test variant). Other Baylands map files in `maps/` should be treated as experimental and are not part of the normal bring-up path.

Current Baylands follow shortcuts:

```bash
cd /home/ruben/halmstad_ws
./run.sh tmux_1to1 baylands
./run.sh tmux_1to1 baylands waypoint:=parkinglot_east_0 mode:=follow \
  nav2_goals:=parkinglot_east
```

Direct Baylands stack:

```bash
cd /home/ruben/halmstad_ws
./run.sh gazebo_sim baylands true waypoint:=parkinglot_east_0
./run.sh spawn_uav baylands
./run.sh localization baylands lidar:=3d
./run.sh nav2 lidar:=3d
./run.sh 1to1_follow baylands \
  nav2_goals:=parkinglot_east
```

## Baylands Nav2 map maintenance

The active Baylands Nav2 maps are:
- `maps/baylands_finished_v3_nav_20cm.yaml`
- `maps/baylands_finished_v3_nav_20cm_merged.yaml`

Both are intended to stay pose-compatible. Keep these values unchanged unless you are intentionally rebuilding the Baylands map frame:
- `resolution: 0.200`
- `origin: [-227, -444.5, 0.0]`
- `mode: trinary`
- `occupied_thresh: 0.65`
- `free_thresh: 0.196`

Current Baylands map workflow:
- Start from the pose-compatible Baylands base map, not a fresh arbitrary export.
- Edit the PGM in GIMP at the same canvas size so the map frame stays aligned with the saved AMCL poses and Baylands waypoint files.
- Keep the image trinary: white for free road, black for blocked/occupied areas, and gray for unknown or "do not rely on this" terrain.
- Add black where the UGV should not drive, especially on narrow, bumpy, or misleading side paths.
- Export the final image as a full-resolution PGM and keep the YAML metadata matched to the same pose-compatible base.
- Test in RViz with localization running and compare the Baylands map against AMCL plus the live scan overlay before changing defaults in scripts.

## Baylands waypoint authoring

Useful Baylands waypoint sources:
- `maps/waypoints_baylands.csv` for the flat list.
- `maps/waypoints_baylands_groups.csv` for grouped Baylands areas used by `waypoint:=...` and grouped route launches.
- `config/baylands_waypoints/` for Nav2-ready YAML routes such as `baylands_waypoints_strip.yaml` and `baylands_waypoints_parkinglot_east.yaml`.

Helpful scripts:
- `./run.sh save_waypoint_csv <name> output:=waypoints_baylands_groups.csv group:=strip`
- `./run.sh save_waypoint_yaml <name> output:=src/lrs_halmstad/config/baylands_waypoints/baylands_waypoints_strip.yaml group:=strip`

These helpers read the current Gazebo pose plus the current AMCL pose, then save them in the formats used by the Baylands spawn and Nav2 pipelines.

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
- `uav_simulator` boots **relaxed**: no gimbal joint commands are published until the first `update_pan` or `update_tilt` message arrives. The joint rests at its SDF-default until the follow stack or a manual command arms it. Motion is then rate-limited at `pan_rate_deg_s: 45.0` and `tilt_rate_deg_s: 60.0`, configurable in `config/run_follow_defaults.yaml` under the `uav_simulator` section (uses `yaml_param`, so the values **must** be present in the YAML â€” there is no hardcoded fallback). Pan was reduced from 90 Â°/s to 45 Â°/s to prevent large visible jumps during UAV yaw changes.
- Camera image bridge (`/image`, `/camera_info`, `/depth_image`) is now Gazeboâ†’ROS only, reducing stale-frame buffering in image viewers.
- `camera_update_rate` spawn arg is now correctly wired through to the SDF (was declared but not passed to xacro). The spawn default is `10` Hz, which is the recommended value for WSL2 â€” Ogre2 rendering through WSLg is the primary RTF bottleneck. Increase only if your host can sustain RTF â‰¥ 1.0 at higher rates.
- `leader_estimator` now defaults to the actual simulated UAV pose topic `/dji0/pose`, not `/dji0/pose_cmd`.
- The active perception range mode is `auto` (depth â†’ radio â†’ const). Available explicit modes: `depth`, `radio`, `const`. Set via `range_mode` in `run_follow_defaults.yaml` under `leader_estimator`. The `ground` mode has been removed.
- When running with OMNeT++, `leader_estimator` subscribes to `/omnet/radio_distance` and uses it as the middle tier in `auto` mode (between depth and constant-range fallback). The raw FSPL-inverted Euclidean range is projected to horizontal distance using the current UAV altitude. Configure via `radio_range_topic` and `radio_range_timeout_s`. Set `radio_range_topic: ''` to disable entirely.
- Depth range sampling uses the **inner 50 % of the detection bounding box** (25 % margin on each edge) instead of a fixed 5 Ã— 5 pixel patch. This scales correctly at all distances and avoids edge pixels that land on background or drone body. Requires at least 10 valid pixels (`depth_patch_min_valid_px`).
- YOLO `conf_threshold` is 0.15 for both `leader_detector` and `leader_tracker`. Lowered from 0.3 to improve detection recall with the newer OBB models; `min_confidence_threshold` (absolute floor in `leader_estimator`) is 0.08.
- YOLO inference now defaults to `device: 'auto'`. On CUDA-capable hosts (for example HH GPU-LAB) the detector/tracker will use GPU 0 automatically; on non-CUDA hosts such as the current WSL2 AMD setup they fall back to CPU. Expected CPU detection rate is still roughly 3â€“10 Hz depending on image resolution and model size.
- The live detector/tracker runtime now defaults to asynchronous newest-frame-only processing. `leader_detector` and `leader_tracker` keep only one pending image, use `image_qos_depth: 1`, stamp every published detection with timing metadata, and can write per-frame benchmark CSV rows via `benchmark_csv_path`.
- Runtime backend selection is controlled by `detector_backend:=ultralytics|onnx_cpu|onnx_directml` plus optional `detector_onnx_model:=<path>`. On Windows + AMD, install `onnxruntime-directml` in the runtime environment before using `detector_backend:=onnx_directml`.
- Export the current trained Ultralytics model to ONNX with `ros2 run lrs_halmstad export_yolo_onnx --weights <model.pt> --out <model.onnx> --imgsz 640`. The command also writes a `.manifest.json` next to the exported ONNX file for reproducible deployment.
- Compare baseline vs improved runtime with launch overrides, for example:
  - baseline sync Ultralytics: `ros2 launch lrs_halmstad run_follow.launch.py detector_backend:=ultralytics detector_async_inference:=false detector_benchmark_csv_path:=/tmp/baseline.csv`
  - async Ultralytics: `ros2 launch lrs_halmstad run_follow.launch.py detector_backend:=ultralytics detector_async_inference:=true detector_benchmark_csv_path:=/tmp/async.csv`
  - async ONNX CPU: `ros2 launch lrs_halmstad run_follow.launch.py detector_backend:=onnx_cpu detector_onnx_model:=/abs/model.onnx detector_benchmark_csv_path:=/tmp/onnx_cpu.csv`
  - async ONNX DirectML: `ros2 launch lrs_halmstad run_follow.launch.py detector_backend:=onnx_directml detector_onnx_model:=C:/path/model.onnx detector_benchmark_csv_path:=C:/temp/onnx_dml.csv`
- Summarize benchmark CSVs with `ros2 run lrs_halmstad run_perception_benchmark_summary /tmp/baseline.csv /tmp/async.csv /tmp/onnx_cpu.csv`.
- Image center correction (`image_center_correction_enable`) is **enabled** at `tick_hz: 10.0` (matched to the CPU YOLO detection rate). Running the tracker faster than the detection rate caused corrections to flicker on stale bounding boxes, producing gimbal jitter that disrupted ByteTrack. Keep `tick_hz` â‰¤ the expected YOLO rate, or disable correction if running at a higher tick rate without GPU inference.
- All world SDF files use consistent physics parameters: `max_step_size: 0.004`, `real_time_update_rate: 250` (targeting RTF â‰¤ 1.0).
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

## Gimbal control

`follow_control` commands the simulator gimbal directly. It publishes
`std_msgs/Float64` degrees to:

- `/{uav_name}/update_pan`
- `/{uav_name}/update_tilt`

This path does not depend on `camera_tracker` override topics.

### follow_control keyboard mode

```bash
ros2 run lrs_halmstad run_follow_control keyboard --uav-name dji0
```

Keys:

- `w/s`: increase/decrease `d_target`
- `a/d`: decrease/increase `leader_heading_offset_deg`
- `j/l`: pan left/right
- `i/k`: tilt up/down
- `c`: center gimbal
- `p`: print current values
- `q`: quit

### follow_control gimbal sweep (random dataset collection)

`follow_control` in `random` mode can simultaneously sweep the gimbal alongside
follow distance randomisation:

```bash
# Sweep d_target and gimbal together
ros2 run lrs_halmstad run_follow_control random --gimbal --uav-name dji0

# Only sweep gimbal (skip d_target param changes)
ros2 run lrs_halmstad run_follow_control random --gimbal-only --uav-name dji0

# Custom sweep ranges and cadence
ros2 run lrs_halmstad run_follow_control random --gimbal \
  --tilt-center -45 --tilt-amplitude 20 --tilt-min -75 --tilt-max -15 \
  --pan-center 0   --pan-amplitude 25  --pan-min -45 --pan-max 45 \
  --gimbal-interval 8 --interval 10 --uav-name dji0
```

Relevant `follow_control random` gimbal args:

| Arg | Default | Description |
| --- | --- | --- |
| `--gimbal` | off | Also sweep pan/tilt alongside d_target |
| `--gimbal-only` | off | Only sweep pan/tilt; skip d_target |
| `--gimbal-interval` | `--interval` | Seconds between gimbal updates |
| `--tilt-center` | -45Â° | Centre of the tilt random distribution |
| `--tilt-amplitude` | 15Â° | Â±amplitude around centre |
| `--tilt-min` / `--tilt-max` | -75Â° / -15Â° | Hard limits |
| `--pan-center` | 0Â° | Centre of the pan random distribution |
| `--pan-amplitude` | 20Â° | Â±amplitude around centre |
| `--pan-min` / `--pan-max` | -45Â° / 45Â° | Hard limits |
| `--uav-name` | `dji0` | UAV namespace for update topics |

Set `gimbal_override_hold_s` in `run_follow_defaults.yaml` to a value slightly larger than `--gimbal-interval` (e.g. `10.0` with `--gimbal-interval 8`).

## Dataset tools

All dataset tools resolve paths relative to `~/halmstad_ws/datasets/` unless an absolute path is given.

### make_obb â€” generate OBB labels

```bash
ros2 run lrs_halmstad make_obb warehouse_v3
ros2 run lrs_halmstad make_obb warehouse_v3 --overlay          # also write overlay_obb/ images
ros2 run lrs_halmstad make_obb warehouse_v3 --overlay --overwrite  # regenerate all
```

- Reads `metadata/<split>/<stem>.json` for `projected_points` and camera intrinsics.
- Writes Ultralytics 8-point OBB labels to `labels/<split>/`.
- `--overlay`: draws yellow projected-point dots + green OBB polygon on each image into `overlay_obb/<split>/`.
- `--overwrite`: regenerates existing label/overlay files instead of skipping them.

### prune_negatives â€” delete frames where the UGV is not visible

Labels are generated geometrically (without occlusion checking), so some frames have valid-looking labels but the UGV is actually behind a wall or out of frame. Use this tool to prune them:

```bash
# Dry run â€” shows what would be deleted
ros2 run lrs_halmstad prune_negatives warehouse_v3 --dry-run

# Actually delete (removes matching files from images/, labels/, labels_det/, metadata/, overlays/)
ros2 run lrs_halmstad prune_negatives warehouse_v3

# Stricter threshold (default 8)
ros2 run lrs_halmstad prune_negatives warehouse_v3 --min-visible-points 12
```

A frame is pruned when:
1. The label file is empty, **or**
2. Fewer than `--min-visible-points` (default `8`) of the 33 projected cuboid corners fall within the image bounds.

Files are deleted from all sibling directories with the same stem (`images/`, `labels/`, `labels_det/`, `metadata/`, `overlay/`, `overlay_detection/`, `overlay_obb/`).

### Zipping for Ultralytics upload

```bash
cd ~/halmstad_ws/datasets/warehouse_v3
zip -r ~/warehouse_v3_obb.zip images/ labels/ dataset.yaml
```

Ensure `dataset.yaml` has `path: .` (not an absolute path) so Ultralytics can resolve it after upload.

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

- base UGV odom flow check uses `/<ugv>/amcl_pose_odom` (Baylands follow defaults use `/<ugv>/ground_truth/odom`)
- follow-stack checks no longer require `/<uav>/pose_cmd/odom`
- detector and estimator are checked separately with `REQUIRE_DETECTION=1` and `REQUIRE_ESTIMATOR=1`

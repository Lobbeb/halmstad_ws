# Visual Follow Pipeline Reference

## Title

Visual-follow pipeline for 1-to-1 UAV-UGV cooperation:

- full architecture overview
- exact role of each main node
- topic flow through the stack
- what was adapted from `ByteTrack`
- what was adapted from the `Sensors` paper
- what is our own engineering contribution
- what was **not** implemented from the papers

This document is written as a thesis-friendly reference for explaining the implemented system without overstating paper reproduction.

## 1. Purpose of the System

The implemented system is a modular ROS2 pipeline for one UAV following one UGV in simulation.

The core goal is:

- the UGV moves using Nav2 and AMCL
- the UAV detects or tracks the UGV visually
- the UAV estimates the UGV state from perception
- the UAV converts that estimate into a follow objective
- the UAV keeps the camera aimed at the UGV while moving

The broader research direction is:

- UGV shared pose can be used for validation and debugging
- but the long-term target behavior is based on visual detection, tracking, and estimation rather than direct ground-truth sharing

## 2. Design Philosophy

The implemented system keeps the architecture modular on purpose.

It does **not** collapse everything into one monolithic controller.

The main separation is:

1. detection/tracking
2. world-frame estimation
3. selected-target filtering
4. controller-facing relative-state estimation
5. follow-point generation
6. planning/smoothing
7. camera tracking
8. command/actuation bridge

That separation matters for:

- debugging
- runtime validation
- future multi-UAV work
- future OMNeT/network integration
- keeping perception, estimation, and control conceptually separated

## 3. Active Launch Graph

The main active launch entrypoint is:

- [run_follow.launch.py](/home/william/halmstad_ws/src/lrs_halmstad/launch/run_follow.launch.py)

That launch file conditionally assembles the runtime graph.

It can enable:

- `leader_detector`
- `leader_tracker`
- `leader_estimator`
- `selected_target_filter`
- `visual_target_estimator`
- `visual_follow_controller`
- `follow_point_generator`
- `follow_point_planner`
- `camera_tracker`
- `visual_actuation_bridge`
- `ugv_nav2_driver`

So the active visual-follow pipeline is not one node. It is a launch-assembled chain of cooperating nodes.

## 4. End-to-End Pipeline Overview

The implemented visual-follow stack can be described as:

1. The UAV camera produces images.
2. A detector or tracker extracts a UGV candidate.
3. A world-frame estimator converts image detection into UGV world pose.
4. A selected-target filter decides whether the candidate should be trusted, held, smoothed, or rejected.
5. A controller-facing visual target estimator converts the filtered target into a relative target state suitable for control.
6. A follow-point generator places a spatial target behind the UGV.
7. A planner smooths that target into a bounded planned target.
8. A camera tracker keeps the target in view.
9. A visual actuation bridge converts the chosen target/control input into the existing UAV command interface.

That means the stack includes both:

- perception-side state handling
- control-side spatial behavior

## 5. Detailed Node-by-Node Description

### 5.1 `leader_detector.py`

File:

- [leader_detector.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_detector.py)

Role:

- pure image-based detection front-end
- wraps YOLO inference
- publishes detection output in the project detection protocol

Main inputs:

- `/<uav>/camera0/image_raw`

Main outputs:

- `/coord/leader_detection`
- `/coord/leader_detection_status`

Key behavior:

- loads YOLO weights
- runs inference on each incoming frame
- selects the target class of interest
- keeps a small notion of continuity through bbox center/class scoring
- publishes a single structured detection payload

Purpose in the pipeline:

- this is the detector-only front end
- it is useful when we want raw visual detection without the external tracker path

### 5.2 `leader_tracker.py`

File:

- [leader_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_tracker.py)

Role:

- optional track-mode front-end
- uses Ultralytics track mode instead of pure detect mode

Main inputs:

- `/<uav>/camera0/image_raw`

Main outputs:

- `/coord/leader_detection`
- `/coord/leader_detection_status`

Key behavior:

- runs tracker-backed YOLO inference
- attaches track metadata such as:
  - `track_id`
  - `track_hits`
  - `track_age_s`
  - `track_state`
  - `track_switched`

Purpose in the pipeline:

- this is the tracker-backed alternative to `leader_detector`
- it provides stronger short-term continuity when tracking works well

### 5.3 `leader_estimator.py`

File:

- [leader_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_estimator.py)

Role:

- converts external visual detections into an estimated UGV world pose
- acts as the main perception-to-world-state bridge

Main inputs:

- image
- camera info
- depth if available
- UAV pose
- camera tilt
- camera world yaw
- external detection topic
- external detection status
- optional actual UGV pose for evaluation/debug

Main outputs:

- `/coord/leader_estimate`
- `/coord/leader_estimate_status`
- `/coord/leader_selected_target`
- `/coord/leader_debug_image`
- optional estimate error topic

Key behavior:

- decodes the project detection payload
- uses camera geometry and UAV pose to convert image detections into world estimates
- supports multiple range modes:
  - `depth`
  - `ground`
  - `const`
  - `auto`
- publishes a world-frame UGV estimate
- publishes a controller-facing selected-target message for downstream filtering
- publishes debug image overlays and readable status text

Purpose in the pipeline:

- this is the main world-frame estimator
- it separates perception interpretation from downstream vehicle-control decisions

### 5.4 `selected_target_filter.py`

File:

- [selected_target_filter.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/selected_target_filter.py)

Role:

- trust, smoothing, hold, and short-gap reacquire layer between raw selected target and controller-facing estimation

Main input:

- `/coord/leader_selected_target`

Main outputs:

- `/coord/leader_selected_target_filtered`
- `/coord/leader_selected_target_filtered_status`

Key behavior:

- confidence-bucket reasoning:
  - strong
  - weak
  - low
  - reject

- continuity checks:
  - same-track center jump
  - track-switch center jump
  - area ratio
  - range jump

- hold behavior:
  - preserves the target briefly through short losses

- prediction behavior:
  - short predictive image-space hold

- low-confidence reacquire:
  - allows a recent plausible weak candidate to reacquire

- confidence-dependent smoothing:
  - stronger measurements update faster
  - weaker ones are smoothed more conservatively

Purpose in the pipeline:

- this layer reduces premature target loss
- it prevents downstream controller-facing state from becoming unstable on every weak frame

### 5.5 `visual_target_estimator.py`

File:

- [visual_target_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/visual_target_estimator.py)

Role:

- transforms the filtered selected target into a controller-facing relative target estimate

Main inputs:

- `/coord/leader_selected_target_filtered`
- `/<uav>/camera0/camera_info`

Main outputs:

- `/coord/leader_visual_target_estimate`
- `/coord/leader_visual_target_estimate_status`

Key behavior:

- estimates relative target state suitable for control
- prefers camera intrinsics normalization when available
- falls back to image-size normalization otherwise

- supports multiple range signal modes:
  - `projected`
  - `area`
  - `auto`

- maintains short-gap prediction through brief target loss
- uses bounded alpha-beta-like correction:
  - position correction
  - velocity correction

- constrains the estimate by:
  - valid projected-range windows
  - minimum valid area
  - maximum target velocity

Purpose in the pipeline:

- this node produces the cleanest controller-facing visual target state
- it is more suitable for control than the raw selected-target stream

### 5.6 `visual_follow_controller.py`

File:

- [visual_follow_controller.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/visual_follow_controller.py)

Role:

- optional test-mode visual control node
- converts visual error directly into bounded forward/yaw commands

Main inputs:

- `/coord/leader_selected_target_filtered`
- `/coord/leader_visual_target_estimate`
- camera info
- image

Main outputs:

- `/coord/leader_visual_control`
- `/coord/leader_visual_control_status`
- optional debug image
- optional debug numeric topics

Key behavior:

- computes yaw command from image-space error
- computes forward command from range or area error
- can prefer the controller-facing target estimate over raw filtered target geometry
- supports short hold on brief loss
- limits command magnitude and slew rate

Purpose in the pipeline:

- this is an optional direct visual-control path
- in the more structured follow-point stack, it is not the only control path

### 5.7 `follow_point_generator.py`

File:

- [follow_point_generator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py)

Role:

- converts controller-facing target state into a world-frame spatial follow objective

Main inputs:

- `/coord/leader_visual_target_estimate`
- `/coord/leader_estimate`
- UAV pose
- camera pose

Main outputs:

- `/coord/leader_follow_point`
- `/coord/leader_follow_point_status`

Key behavior:

- keeps the UAV behind the UGV rather than simply chasing image projections
- uses target pose position when available
- uses target heading when available
- can estimate heading direction from motion if needed
- smooths target world velocity
- smooths the follow point
- limits follow-point jumps
- can hold the last valid follow point briefly across short estimate gaps

Purpose in the pipeline:

- this is the main spatial-shadow-follow layer
- it transforms visual target state into a meaningful world-frame UAV objective

### 5.8 `follow_point_planner.py`

File:

- [follow_point_planner.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_planner.py)

Role:

- planner/smoother between the raw follow point and the final actuation bridge

Main inputs:

- `/coord/leader_follow_point`
- UAV pose

Main outputs:

- `/coord/leader_planned_target`
- `/coord/leader_planned_target_status`

Key behavior:

- smooths XY and yaw changes
- limits step size in position and yaw
- can seed from the current UAV pose
- can briefly hold the last planned target when the raw follow point disappears

Purpose in the pipeline:

- prevents abrupt jumps from the raw follow-point layer
- makes the final actuation target more stable and safer

### 5.9 `camera_tracker.py`

File:

- [camera_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/camera_tracker.py)

Role:

- keeps the camera aimed at the UGV

Main inputs:

- UGV pose estimate or odometry
- leader status
- image-space detection
- camera info
- UAV actual pose
- UAV commanded pose

Main outputs:

- `/<uav>/update_pan`
- `/<uav>/update_tilt`
- multiple camera debug topics

Key behavior:

- solves geometry-based pan/tilt
- can use actual or commanded UAV pose depending on freshness
- supports image-center correction on top of geometry
- uses deadbands to reduce chatter
- holds the last trackable leader pose briefly across short estimate loss

Purpose in the pipeline:

- keeps the target inside the camera field of view
- helps preserve usable perception instead of treating camera pointing as a separate cosmetic issue

### 5.10 `visual_actuation_bridge.py`

File:

- [visual_actuation_bridge.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/visual_actuation_bridge.py)

Role:

- converts visual-follow outputs into the existing UAV pose-command interface

Main inputs:

- `/coord/leader_visual_control`
- `/coord/leader_follow_point`
- `/coord/leader_planned_target`
- UAV pose

Main outputs:

- `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- optional `/<uav>/pose_cmd`
- `/coord/leader_visual_actuation_bridge_status`

Key behavior:

- supports multiple input modes:
  - `control`
  - `follow_point`
  - `planned_target`
  - `auto`

- enforces bounded XY and yaw steps
- can mirror the chosen target into `pose_cmd` for compatibility/debug
- chooses current altitude or fixed altitude depending on configuration

Purpose in the pipeline:

- this is the final connection between the visual-follow stack and the existing UAV command interface
- it lets the new stack reuse the existing simulator/controller path rather than replacing it completely

### 5.11 `follow_uav.py`

File:

- [follow_uav.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_uav.py)

Role:

- compatibility and estimate-follow controller path
- older direct follow controller that still exists in the codebase

Why it matters in the explanation:

- it is part of the project history and active codebase
- but the structured visual-follow pipeline described above is the more modular perception-to-follow chain
- this file should not be confused with the whole visual pipeline by itself

## 6. Topic Flow Through the Stack

The main visual-follow topic flow is:

1. `/<uav>/camera0/image_raw`
2. `/coord/leader_detection`
3. `/coord/leader_estimate`
4. `/coord/leader_selected_target`
5. `/coord/leader_selected_target_filtered`
6. `/coord/leader_visual_target_estimate`
7. `/coord/leader_follow_point`
8. `/coord/leader_planned_target`
9. `/coord/leader_visual_actuation_bridge_status`
10. `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`

In parallel, the camera path uses:

- `/<uav>/camera0/camera_info`
- `/<uav>/update_pan`
- `/<uav>/update_tilt`
- `/<uav>/camera/actual/*`

And the readable status chain uses:

- `/coord/leader_detection_status`
- `/coord/leader_estimate_status`
- `/coord/leader_selected_target_filtered_status`
- `/coord/leader_visual_target_estimate_status`
- `/coord/leader_follow_point_status`
- `/coord/leader_planned_target_status`
- `/coord/leader_visual_control_status`
- `/coord/leader_visual_actuation_bridge_status`

That readable status chain is important in the thesis because it shows the system was designed to be inspectable stage by stage.

## 7. What Was Adapted From ByteTrack

We did **not** implement ByteTrack as a full faithful port.

We adapted its main idea:

- preserve weak but plausible evidence longer
- use continuity, not confidence alone
- tolerate short losses
- improve reacquire after brief dropouts

### ByteTrack-style influence in our system

#### `leader_tracker.py`

- tracker-backed continuity is available at the detection front end
- track metadata is preserved and forwarded

#### `selected_target_filter.py`

This is the clearest ByteTrack-inspired layer.

Implemented ideas:

- low-confidence evidence is not discarded immediately
- continuity matters:
  - center jump
  - area ratio
  - range jump
  - same-track vs track-switch logic

- short-gap hold and prediction are allowed
- low-confidence reacquire is allowed if continuity is plausible
- confidence-dependent smoothing stabilizes the target seen by the controller

### Correct thesis wording for ByteTrack

Use wording like:

- "We adapted the continuity-preserving principle of ByteTrack."
- "Weak but plausible visual evidence was preserved longer instead of being discarded immediately."
- "We did not port the full ByteTrack algorithm."

### What we did **not** implement from ByteTrack

Do **not** claim:

- full two-stage association
- full MOT pipeline
- full ByteTrack tracker internals
- ReID / appearance-based association

That would be inaccurate.

## 8. What Was Adapted From the Sensors Paper

We also did **not** reproduce the Sensors paper in full.

We adapted its control-oriented visual-target-state principle:

- estimate a controller-facing relative target state
- keep that estimate alive across short losses
- use image geometry and camera intrinsics
- keep the target visible in the camera instead of controlling only by rigid geometry

### Sensors-style influence in our system

#### `visual_target_estimator.py`

Implemented ideas:

- controller-facing target state
- image/intrinsics normalization
- range-source switching
- short-gap prediction
- bounded relative-state estimation
- alpha-beta-like state correction

#### `camera_tracker.py`

Implemented ideas:

- camera-centered control support
- image-centering correction
- explicit attempt to keep the target in view
- hold of last trackable leader pose over short loss

#### `follow_point_generator.py`

Implemented ideas:

- convert visual target state into a spatial world-frame objective
- preserve behind-target geometry
- use target pose/heading when available instead of orbiting noisy detections

### Correct thesis wording for Sensors

Use wording like:

- "We adapted a controller-facing visual target-state idea from the Sensors paper."
- "The visual target estimate was designed to remain useful across short losses and to support visibility-preserving behavior."
- "We did not reproduce the full original controller or optimization formulation."

### What we did **not** implement from the Sensors paper

Do **not** claim:

- full paper controller reproduction
- exact dynamics/control-law reproduction
- full formal FOV-constrained optimization from the paper

That would overstate the work.

## 9. Our Own Engineering Contribution

The most important contribution is not merely reading the papers.

The actual system contribution is the integration of these ideas into a modular ROS2 stack.

That includes:

- choosing stable topic contracts
- splitting the system into inspectable layers
- converting raw visual detections into world-frame state
- introducing a controller-facing filtered target state
- introducing a controller-facing relative target estimator
- generating a world-frame follow point behind the UGV
- smoothing that follow point into a planner-friendly target
- connecting the new visual-follow stack into the existing UAV command interface through the bridge
- keeping camera tracking as an explicit layer instead of burying it inside a monolithic controller

So the clean thesis claim is:

- the main contribution is the integrated visual-follow architecture
- the papers provided conceptual direction for some layers
- but the final stack is our own ROS2 system design

## 10. Bridge and Final Command Path

This part is important and should not be skipped in the thesis explanation.

The final command path is **not**:

- detector directly commanding the UAV
- estimator directly commanding the UAV

Instead, it is:

1. perception produces target information
2. target information becomes a spatial or control objective
3. the bridge converts that objective into the existing UAV command interface

That final bridge layer is implemented in:

- [visual_actuation_bridge.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/visual_actuation_bridge.py)

Its importance is:

- it preserves modularity
- it allows the visual stack to plug into the simulator/controller path without rewriting the entire low-level UAV path
- it keeps the actuation interface stable

That is a useful thesis point because it shows system engineering discipline, not just algorithm experimentation.

## 11. What the Stack Includes at Thesis Level

If you want to describe the system as a single pipeline in the thesis, a good concise version is:

1. visual detection/tracking front-end
2. world-frame UGV estimator
3. continuity-aware selected-target filter
4. controller-facing visual target-state estimator
5. spatial follow-point generator
6. follow-point planner
7. camera tracker
8. visual actuation bridge into the UAV command interface

That is the safest single-paragraph architecture summary.

## 12. Honest Boundaries

This system is **not**:

- a full ByteTrack reimplementation
- a full Sensors-controller reproduction
- a single monolithic visual servo controller
- a finished perception-perfect product system

This system **is**:

- a modular ROS2 visual-follow architecture
- inspired by ByteTrack-style continuity handling
- inspired by Sensors-style controller-facing visual target estimation and visibility preservation
- engineered to be debuggable stage by stage

## 13. Recommended Thesis Wording

### Short formal wording

"The implemented system is a modular ROS2 visual-follow pipeline for UAV-UGV cooperation. Visual detections or tracked detections are converted into a world-frame UGV estimate, then passed through a continuity-aware selected-target filtering layer and a controller-facing visual target-state estimator. This state is converted into a world-frame follow point, smoothed into a planned target, and bridged into the existing UAV command interface while the camera tracker maintains target visibility."

### ByteTrack wording

"Rather than reproducing ByteTrack in full, the work adapts its key principle of preserving weak but plausible evidence and tolerating short target loss. This influenced the selected-target filtering layer, where low-confidence reacquire, continuity checks, and short-gap hold were introduced."

### Sensors wording

"Rather than reproducing the full controller from the Sensors paper, the work adapts its core idea of maintaining a controller-facing visual target state and supporting visibility-preserving behavior. This influenced the visual target estimator, camera tracker, and follow-point generation stages."

### Contribution wording

"The main contribution is the integration of these ideas into a modular ROS2 architecture rather than the direct reproduction of either source paper."

## 14. What Browser GPT Should and Should Not Assume

Browser GPT should assume:

- the pipeline is modular
- the bridge is part of the real system
- the planner is part of the real system
- the camera tracker is part of the real system
- paper influence is conceptual and partial

Browser GPT should **not** assume:

- full paper-faithful ByteTrack
- full paper-faithful Sensors controller
- that every runtime experiment or later branch change is part of the stable `v1` system description

## 15. Bottom Line

The cleanest truthful summary is:

- `ByteTrack` influenced how we preserve weak evidence, continuity, and short-gap reacquire.
- `Sensors` influenced how we estimate a controller-facing visual target state and keep the target visible in the camera.
- Our main work was integrating these ideas into a modular ROS2 UAV-UGV visual-follow pipeline that includes estimation, filtering, follow-point generation, planning, camera tracking, and a final actuation bridge.

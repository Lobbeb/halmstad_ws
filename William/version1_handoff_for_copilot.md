# Version 1 Handoff For Copilot

## Purpose

This file explains what `version 1` actually is in this workspace, what we implemented, what came from the two main paper directions, what is our own engineering, and what the current live problem is.

This is meant as a handoff note for Copilot or any helper model so it can work from the right mental model instead of guessing.

## What `Version 1` Means Here

`Version 1` is the first full modular visual-follow stack that we built around the active ROS2 launch/runtime path.

It is **not**:

- a paper-faithful reproduction of ByteTrack
- a paper-faithful reproduction of the Sensors controller
- a single giant controller file
- a fully solved hard-regime product system

It **is**:

- a modular ROS2 UAV-UGV visual-follow architecture
- built on top of the existing simulator/controller path
- split into detection, estimation, filtering, controller-facing target estimation, follow-point generation, planning, camera tracking, and a final actuation bridge

## Main Goal of V1

The goal of `v1` was:

- UGV moves with AMCL + Nav2
- UAV visually detects or tracks the UGV
- UAV estimates the UGV state from perception
- UAV converts that into a follow objective
- UAV keeps the camera aimed at the UGV
- UAV follows from a useful behind/shadow geometry

The design direction was:

- keep the architecture modular
- keep topic contracts stable
- make the visual-follow system inspectable stage by stage

## Active Runtime Path

The real active launch graph is:

- [run_follow.launch.py](/home/william/halmstad_ws/src/lrs_halmstad/launch/run_follow.launch.py)

The real wrapper path is:

- `./run.sh tmux_1to1 ...`

The baseline `v1` style command we have been using is:

```bash
cd ~/halmstad_ws
./stop.sh tmux_1to1 warehouse
./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=false obb:=true weights:=warehouse-v1-yolo26n-obb.pt camera:=detached use_estimate:=true use_actual_heading:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true delay_s:=12 tmux_attach:=true
```

In that mode, the intended active stack is roughly:

- `leader_detector`
- `leader_estimator`
- `selected_target_filter`
- `visual_target_estimator`
- `follow_point_generator`
- `follow_point_planner`
- `camera_tracker`
- `visual_actuation_bridge`
- UGV Nav2 driver on the ground side

## High-Level Pipeline

The implemented `v1` visual-follow pipeline is:

1. UAV camera image arrives.
2. Detector or tracker produces a candidate UGV detection.
3. `leader_estimator` converts that into a world-frame UGV estimate.
4. A selected-target filter keeps weak but plausible target evidence alive longer.
5. A controller-facing visual target estimator turns the filtered target into a relative target state suitable for control.
6. A follow-point generator creates a world-frame spatial target behind the UGV.
7. A follow-point planner smooths that target.
8. A camera tracker keeps the UGV in view.
9. A visual actuation bridge converts the chosen target into the existing UAV command interface.

That means `v1` already includes:

- perception
- continuity handling
- controller-facing estimation
- spatial follow behavior
- planning
- camera control
- bridge/actuation integration

## What We Implemented In V1

### 1. Visual Front End

Files:

- [leader_detector.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_detector.py)
- [leader_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_tracker.py)

Implemented:

- YOLO-based detection
- optional tracker-backed detection path
- publishing of `/coord/leader_detection`
- publishing of `/coord/leader_detection_status`
- continuity-aware candidate selection at the front end

### 2. World-Frame Perception Estimate

File:

- [leader_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_estimator.py)

Implemented:

- conversion of image detections into world-frame UGV estimate
- use of camera geometry, UAV pose, and camera orientation
- support for multiple range modes:
  - `depth`
  - `ground`
  - `const`
  - `auto`
- publishing:
  - `/coord/leader_estimate`
  - `/coord/leader_estimate_status`
  - `/coord/leader_selected_target`
  - `/coord/leader_debug_image`

### 3. Continuity-Aware Target Filtering

File:

- [selected_target_filter.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/selected_target_filter.py)

Implemented:

- confidence buckets
- continuity checks across frames
- same-track vs track-switch logic
- short-gap hold
- short predictive hold
- low-confidence reacquire
- confidence-weighted smoothing

This layer is important because it reduces the raw:

- `NO_DET`
- `raw_invalid`
- `target_missing`

cascade that would otherwise kill the downstream controller-facing state too early.

### 4. Controller-Facing Visual Target Estimation

File:

- [visual_target_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/visual_target_estimator.py)

Implemented:

- controller-facing relative target estimate
- camera-intrinsics-aware normalization
- range-source selection:
  - `projected`
  - `area`
  - `auto`
- short-gap prediction
- bounded alpha-beta-like update behavior
- publishing:
  - `/coord/leader_visual_target_estimate`
  - `/coord/leader_visual_target_estimate_status`

### 5. Spatial Shadow-Follow Objective

File:

- [follow_point_generator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py)

Implemented:

- conversion of controller-facing target estimate into a world-frame follow point
- behind-target geometry
- use of target pose and heading when available
- motion-aware target direction handling
- smoothing and bounded jumps
- brief hold through short estimate loss
- publishing:
  - `/coord/leader_follow_point`
  - `/coord/leader_follow_point_status`

### 6. Planned Target Smoothing

File:

- [follow_point_planner.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_planner.py)

Implemented:

- smoothing from raw follow point to planned target
- bounded XY step
- bounded yaw step
- seed from current UAV pose
- brief hold across short follow-point loss
- publishing:
  - `/coord/leader_planned_target`
  - `/coord/leader_planned_target_status`

### 7. Camera Tracking

File:

- [camera_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/camera_tracker.py)

Implemented:

- geometry-based pan/tilt aiming
- image-center correction
- use of actual vs commanded UAV pose depending on freshness
- hold of last trackable leader pose
- publishing of pan/tilt updates and camera debug topics

### 8. Visual Actuation Bridge

File:

- [visual_actuation_bridge.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/visual_actuation_bridge.py)

Implemented:

- final bridge between visual-follow stack and existing UAV command interface
- support for multiple input modes:
  - `control`
  - `follow_point`
  - `planned_target`
  - `auto`
- bounded XY and yaw step output
- optional `pose_cmd` mirror
- publishing into:
  - `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
  - optional `/<uav>/pose_cmd`

This bridge is part of `v1`. It is not something external to the system.

## The Two Main Paper Influences

## A. ByteTrack Influence

Important:

- we did **not** implement full ByteTrack
- we adapted a small set of its most useful ideas

What we took conceptually:

- weak evidence should not be dropped too early
- continuity matters, not confidence alone
- short target loss should be tolerated
- reacquire should be easier after brief loss

Where that shows up:

- [leader_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_tracker.py)
- [selected_target_filter.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/selected_target_filter.py)

Concrete adapted mechanisms:

- weak/low-confidence target acceptance when continuity is plausible
- short-gap hold instead of instant invalidation
- low-confidence reacquire logic
- geometry/continuity checks before switching or rejecting

Correct thesis wording:

- "We adapted ByteTrack-style continuity preservation and weak-evidence handling."
- not:
  - "We implemented ByteTrack."

## B. Sensors Paper Influence

Important:

- we did **not** implement the full controller from the paper
- we adapted the controller-facing visual target-state idea

What we took conceptually:

- maintain a controller-facing relative target state
- estimate it continuously across short gaps
- use camera/image geometry meaningfully
- support visibility-preserving behavior rather than only rigid geometry

Where that shows up:

- [visual_target_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/visual_target_estimator.py)
- [camera_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/camera_tracker.py)
- [follow_point_generator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py)

Concrete adapted mechanisms:

- controller-facing relative target estimate
- projected-vs-area range reasoning
- short-gap prediction
- bounded state update
- image-center correction
- camera tracking as part of target-visibility preservation
- follow-point generation based on estimated target state rather than raw image measurements alone

Correct thesis wording:

- "We adapted the Sensors-style controller-facing visual target state and visibility-preserving support behavior."
- not:
  - "We reproduced the Sensors controller."

## What Is Our Own Engineering Contribution

The biggest contribution is not just reading the papers.

The actual contribution is integrating these ideas into one modular ROS2 system with clear boundaries:

- visual front end
- world-frame estimation
- selected-target continuity layer
- controller-facing relative target estimation
- follow-point generation
- planning
- camera control
- actuation bridge

That integrated pipeline is our own engineering contribution.

So the safest thesis claim is:

- we adapted ideas from the two paper directions
- but the implemented pipeline itself is our own ROS2 architecture

## Important Boundaries: What V1 Is Not

Do **not** let Copilot or GPT assume:

- full paper-faithful ByteTrack
- full paper-faithful Sensors controller
- a fully solved hard-regime visual-only follow system
- that `leader_estimator` directly controls the UAV
- that the bridge/planner/camera path are optional side extras rather than part of the real stack

## Current Live Problem

Current practical symptom in the live sim:

- the sim is running
- the UGV side is alive enough to move or attempt route execution
- the UAV spawns
- but the UAV is not properly following the UGV as a coherent shadow-follow behavior
- it appears to have "its own life"

That means the problem is **not**:

- "what is v1?"
- or "what did we implement?"

It is now more likely a runtime issue in the active follow/bridge/control path.

## Most Relevant Files For The Current Bug

If Copilot is trying to fix the live `v1` follow behavior, inspect these first:

- [run_follow.launch.py](/home/william/halmstad_ws/src/lrs_halmstad/launch/run_follow.launch.py)
- [follow_point_generator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py)
- [follow_point_planner.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_planner.py)
- [camera_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/camera_tracker.py)
- [visual_actuation_bridge.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/visual_actuation_bridge.py)
- [follow_uav.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_uav.py)
- [simulator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/sim/simulator.py)

## Topics To Check First For The Current Bug

If the UAV is acting independently instead of actually following the UGV, inspect:

- `/coord/leader_estimate`
- `/coord/leader_estimate_status`
- `/coord/leader_selected_target_filtered`
- `/coord/leader_selected_target_filtered_status`
- `/coord/leader_visual_target_estimate`
- `/coord/leader_visual_target_estimate_status`
- `/coord/leader_follow_point`
- `/coord/leader_follow_point_status`
- `/coord/leader_planned_target`
- `/coord/leader_planned_target_status`
- `/coord/leader_visual_actuation_bridge_status`
- `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- `/<uav>/pose_cmd`
- `/<uav>/update_pan`
- `/<uav>/update_tilt`

The most important debugging question is:

- is the visual stack producing sensible follow-point / planned-target / bridge output, and is the simulator/controller path actually executing it?

## Instructions To Copilot

If Copilot is taking over from here, the safest framing is:

1. Treat the architecture above as the intended `v1` system.
2. Do **not** redesign the architecture.
3. Do **not** claim we implemented full ByteTrack or the full Sensors controller.
4. Assume the current problem is in the live follow/bridge/control execution path, not in the existence of the architecture itself.
5. Start from the status topics and actual UAV command topics, not from random rewrites.

## One-Paragraph Thesis Summary

`Version 1` is a modular ROS2 visual-follow pipeline for UAV-UGV cooperation. A visual detector or tracker produces a UGV candidate, a world-frame estimator converts this into an estimated UGV pose, a continuity-aware selected-target filter preserves plausible target state across weak detections, and a controller-facing visual target estimator produces a relative target state for control. This state is converted into a spatial follow point, smoothed into a planned target, and bridged into the existing UAV command interface, while a camera tracker maintains visibility of the UGV. The continuity-handling direction was inspired by ByteTrack, and the controller-facing visual target-state and visibility-preserving direction was inspired by the Sensors paper, but the integrated ROS2 pipeline itself is our own engineering contribution.

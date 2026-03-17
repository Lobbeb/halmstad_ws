# Claude Opus Handoff: V1 Visual-Follow Fix Pass

## What This Is

This is a single combined handoff for fixing and understanding `Version 1` of the visual-follow stack in this workspace.

Use this as the main context document.

It combines:

- what `v1` is
- what was implemented
- what was adapted from the two paper directions
- what we are observing in the live run now
- what should and should not be changed
- where the active tools, files, and runtime commands are

## Core Goal

We want `v1` to be stable enough to:

- run the 1-to-1 simulation reliably
- show the intended modular visual-follow pipeline working
- be explainable and defensible for the halftime report

We do **not** want a broad redesign right now.

The near-term goal is:

- fix `v1`
- understand `v1`
- write `v1`

Then later:

- build `v2` from a stable `v1` baseline

## What Version 1 Is

`v1` is a modular ROS2 visual-follow stack for one UAV following one UGV.

It includes:

1. visual detection / optional tracking
2. world-frame UGV estimation
3. selected-target filtering
4. controller-facing relative visual target estimation
5. follow-point generation
6. follow-point planning / smoothing
7. camera tracking
8. final actuation bridge into the existing UAV command path

It is **not**:

- a full ByteTrack reimplementation
- a full Sensors-paper controller reproduction
- a monolithic controller
- a fully solved hard-regime product system

## Main Architecture

The active launch graph is:

- [run_follow.launch.py](/home/william/halmstad_ws/src/lrs_halmstad/launch/run_follow.launch.py)

The main live wrapper path is:

- `./run.sh tmux_1to1 ...`

Important nodes in the active stack:

- [leader_detector.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_detector.py)
- [leader_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_tracker.py)
- [leader_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_estimator.py)
- [selected_target_filter.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/selected_target_filter.py)
- [visual_target_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/visual_target_estimator.py)
- [follow_point_generator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py)
- [follow_point_planner.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_planner.py)
- [camera_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/camera_tracker.py)
- [visual_actuation_bridge.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/visual_actuation_bridge.py)
- [follow_uav.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_uav.py)
- [simulator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/sim/simulator.py)

The intended pipeline is:

1. UAV camera image arrives
2. detector or tracker produces `/coord/leader_detection`
3. `leader_estimator` converts that into `/coord/leader_estimate` and `/coord/leader_selected_target`
4. `selected_target_filter` publishes `/coord/leader_selected_target_filtered`
5. `visual_target_estimator` publishes `/coord/leader_visual_target_estimate`
6. `follow_point_generator` publishes `/coord/leader_follow_point`
7. `follow_point_planner` publishes `/coord/leader_planned_target`
8. `camera_tracker` keeps the target in view
9. `visual_actuation_bridge` converts the active follow input into UAV command output

## What Came From The Two Paper Directions

### 1. ByteTrack Direction

We did **not** implement full ByteTrack.

We adapted the idea:

- preserve weak but plausible evidence longer
- use continuity, not confidence alone
- tolerate short loss
- make reacquire easier after brief dropouts

Main places this shows up:

- [leader_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_tracker.py)
- [selected_target_filter.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/selected_target_filter.py)

Concrete adapted behavior:

- confidence buckets
- same-track vs track-switch logic
- hold over short loss
- short predictive hold
- low-confidence reacquire
- continuity-based rejection of implausible jumps

Correct interpretation:

- "ByteTrack-inspired continuity handling"
- not "we implemented ByteTrack"

### 2. Sensors Direction

We did **not** implement the full Sensors controller.

We adapted the idea:

- maintain a controller-facing visual target state
- keep that state alive through short measurement gaps
- use visual geometry meaningfully in control
- keep the target visible in the camera

Main places this shows up:

- [visual_target_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/visual_target_estimator.py)
- [camera_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/camera_tracker.py)
- [follow_point_generator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py)

Concrete adapted behavior:

- controller-facing relative target estimate
- projected-vs-area range handling
- short-gap prediction
- bounded target-state update
- image-center correction
- visibility-preserving camera behavior
- follow-point generation from estimated target state rather than only raw image observations

Correct interpretation:

- "Sensors-inspired controller-facing target-state and visibility-preserving behavior"
- not "we reproduced the full Sensors controller"

## What Is Our Own Engineering

The main contribution is the integration of these ideas into a modular ROS2 architecture.

That includes:

- splitting the stack into inspectable layers
- maintaining stable topic contracts
- converting raw perception into world-frame and controller-facing state
- combining target filtering, target-state estimation, follow-point generation, planning, camera tracking, and the bridge
- connecting the new visual stack into the existing UAV command interface instead of rewriting the whole low-level path

So the accurate thesis claim is:

- the papers inspired some of the logic directions
- but the integrated pipeline itself is our own ROS2 system design

## Existing Reference Files

Read these first:

- [version1_handoff_for_copilot.md](/home/william/halmstad_ws/William/version1_handoff_for_copilot.md)
- [visual_follow_pipeline_bytetrack_sensors.md](/home/william/halmstad_ws/William/visual_follow_pipeline_bytetrack_sensors.md)

These already explain:

- the `v1` pipeline
- node responsibilities
- paper mapping
- boundaries on what was and was not implemented

This current file is the combined "fix and understand" handoff.

## Current Live Runtime Situation

The sim now runs far enough that we can inspect the actual follow behavior.

Important recent finding:

- YOLO is working
- but it is very sparse on CPU
- detection cadence is on the order of several seconds between successful outputs
- the original downstream hold/timeout settings were too short for that cadence

This means:

- the system was not only failing because the detector was "bad"
- it was also failing because the pipeline timed out too quickly between sparse detections

That is an important debugging boundary.

## Very Important Current Observation

We observed a strong live symptom:

- the UGV is still near its spawn/start area
- the UAV camera still clearly has the UGV in view
- yet the UAV body drifts away instead of holding a proper behind/shadow-follow position

This matters a lot.

It means the bug is likely **not only**:

- YOLO visibility
- raw target detection

It strongly suggests a problem in the motion/follow path such as:

- follow point generation
- follow point planning
- bridge behavior
- frame/sign interpretation
- stale target state being used for motion
- camera tracking working while body-follow is wrong

In other words:

- camera tracking may still be working
- while UAV motion behavior is wrong

That is currently one of the highest-value clues.

## Current Likely Root Causes

At the moment, the likely active issues are:

1. sparse CPU YOLO cadence vs downstream hold time mismatch
2. body-follow path not respecting intended shadow-follow geometry
3. potential stale estimate / stale follow-point / stale planned-target usage
4. potential bridge/frame/sign issue causing the UAV body to drift or act independently
5. camera tracking not being tightly coupled enough to the body-follow objective

## What We Want Fixed In V1

We want `v1` to:

- run reliably enough to demonstrate the architecture
- keep the camera on the UGV
- keep the UAV body in a sensible behind/shadow-follow position
- avoid drifting away while the target is still visible
- avoid spazzing / overreacting between sparse detections

In short:

- calm, coherent shadow-follow
- not random drifting
- not uncontrolled overshoot
- not an architecture rewrite

## What We Do NOT Want Right Now

Do **not**:

- redesign the architecture
- import or recreate a large external GitHub system
- claim we implemented full ByteTrack or the full Sensors controller
- jump straight to `v2`
- do broad random tuning without stage-by-stage reasoning
- collapse the modular stack into one controller

This pass should still be a `v1` stabilization and understanding pass.

## Good Direction For V1 Fixing

A good `v1` fix pass should focus on:

1. making the stack survive sparse detections
2. making the UAV follow more calmly
3. keeping the UGV in view
4. ensuring body-follow and camera-follow are consistent
5. preserving the modular structure

The right kind of changes are:

- timeout/hold adjustments where justified
- calmer follow-point / planner behavior
- bridge behavior fixes
- camera/follow consistency fixes
- stage-by-stage debugging using status topics

The wrong kind of changes are:

- giant new mode systems
- full v2 paper-driven redesign
- replacing the whole stack with a different repo

## Useful Paper Guidance For Fixing V1

Use the papers only as conceptual guidance.

### ByteTrack-style guidance

- preserve weak evidence longer
- allow short-loss continuity
- avoid dropping the target too early

### Sensors-style guidance

- keep the target in view
- make controller behavior visibility-aware
- use the camera/control coupling sensibly

Important:

- these ideas should guide small `v1` fixes
- not trigger a broad architectural rewrite

## When To Use The Papers More Deeply

If `v1` becomes stable and trustworthy, then `v2` can go deeper into the Sensors direction.

That would make sense because the Sensors direction helps with:

- visibility-aware following
- camera/control coupling
- controller-facing visual target-state handling

But that should be a later controlled `v2` step, not mixed into this `v1` fix pass.

## Files To Inspect First

If the current symptom is:

- camera sees the UGV
- UAV body drifts away

inspect these first:

- [follow_point_generator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py)
- [follow_point_planner.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/follow_point_planner.py)
- [visual_actuation_bridge.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/visual_actuation_bridge.py)
- [camera_tracker.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/camera_tracker.py)
- [leader_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/leader_estimator.py)
- [selected_target_filter.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/follow/selected_target_filter.py)
- [visual_target_estimator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/perception/visual_target_estimator.py)
- [run_follow_defaults.yaml](/home/william/halmstad_ws/src/lrs_halmstad/config/run_follow_defaults.yaml)
- [run_follow.launch.py](/home/william/halmstad_ws/src/lrs_halmstad/launch/run_follow.launch.py)
- [simulator.py](/home/william/halmstad_ws/src/lrs_halmstad/lrs_halmstad/sim/simulator.py)

## Topics To Inspect First

Use these topics to determine where the live behavior diverges:

- `/coord/leader_detection`
- `/coord/leader_detection_status`
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

The key debugging question is:

- is the visual stack producing sensible motion objectives, and is the UAV execution path following them correctly?

## Runtime Tools In This Workspace

Main run command:

```bash
cd ~/halmstad_ws
./stop.sh tmux_1to1 warehouse
./run.sh tmux_1to1 warehouse gui:=true mode:=yolo tracker:=false obb:=true weights:=warehouse-v1-yolo26n-obb.pt camera:=detached use_estimate:=true use_actual_heading:=false start_visual_follow_planner:=true start_visual_actuation_bridge:=true delay_s:=12 tmux_attach:=true
```

YOLO/camera perspective:

```bash
cd ~/halmstad_ws
./run.sh rqt_perspective YOLO
```

Raw camera view:

```bash
cd ~/halmstad_ws
./run.sh rqt_image_view /dji0/camera0/image_raw
```

Estimator/debug image:

```bash
cd ~/halmstad_ws
./run.sh rqt_image_view /coord/leader_debug_image
```

Workspace root:

- `/home/william/halmstad_ws`

## Instructions For Claude

Please do the following:

1. Use this file plus the two existing markdown files as the source of truth for architecture.
2. Treat this as a `v1` stabilization/debugging task, not a `v2` redesign task.
3. Use the papers only as conceptual guidance:
   - ByteTrack for short-loss/weak-evidence continuity
   - Sensors for visibility-preserving, camera-aware behavior
4. Start from the live symptom that the camera sees the UGV but the UAV body drifts away.
5. Inspect the follow-point, planner, bridge, and camera path first.
6. Return with:
   - the most likely root cause
   - the smallest high-value `v1` fix path
   - exact files/parameters to change
   - why the change stays inside `v1`

## One-Paragraph Summary

`v1` is a modular ROS2 visual-follow stack for UAV-UGV cooperation. It includes visual detection/tracking, world-frame estimation, a ByteTrack-inspired continuity-aware selected-target filter, a Sensors-inspired controller-facing visual target estimator, follow-point generation, planner smoothing, camera tracking, and a final actuation bridge into the existing UAV command interface. The current live issue is not only sparse CPU YOLO cadence, but also that the UAV camera can still see the UGV while the UAV body drifts away instead of holding a stable behind-follow geometry. This suggests the next `v1` fix should focus on the follow-point/planner/bridge/camera execution path, not on a broad redesign.

# Agent Guidelines

## Project Goal

This workspace is right now for 1-to-1 UAV-UGV cooperation:
- the UGV navigates with localization and Nav2
- the UAV follows and predicts future UGV positions
- the long-term goal is for the UAV to rely only on visual UGV detection/tracking for prediction and path planning

The intended direction is:
- UGV position sharing is useful for testing and validation
- production behavior should move toward estimation based on perception alone

## Future Scope

This project is intended to support multiple UAVs later.

Because of that:
- keep hardcoding to a minimum
- avoid single-UAV assumptions unless explicitly required
- prefer parameters, namespaces, and topic indirection over fixed names
- do not add world-specific or user-specific hacks unless explicitly requested

## Omnet / Network Layer

There is an intended OMNeT integration layer for simulating communication/network effects during runtime.

Current status:
- this is not fully implemented yet
- do not assume the network simulation layer is complete
- when changing runtime architecture, keep future OMNeT cooperation possible

That means:
- keep message flow clear
- keep detection, tracking, estimation, and control separated
- avoid tightly coupling nodes in ways that make network simulation hard later

## Core Structure To Preserve

Keep the internal structure of the workspace intact.

Important files and roles:
- `controller.py`
  - keep this as the controller path for future real-world testing
- `simulator.py`
  - keep this as the main sim-side execution path
  - keep topic names stable unless explicitly asked to change them
- `leader_detector.py`
  - detection only
  - publishes `/coord/leader_detection`
  - publishes `/coord/leader_detection_status`
- `leader_tracker.py`
  - tracking only
  - publishes `/coord/leader_detection`
  - publishes `/coord/leader_detection_status`
- `leader_estimator.py`
  - estimation only
  - consumes detection inputs
  - publishes estimate outputs and `leader_debug_image`

General rule:
- do not collapse separate concerns back into one giant file

## Active Entry Points

Current user-facing wrappers:
- scripts live under `scripts/`
- use `./run.sh <name>` and `./stop.sh <name>` from the workspace root

Current follow launch naming:
- `src/lrs_halmstad/launch/run_follow.launch.py`
  - this is the real active follow launch graph
- `run_follow_motion.launch.py`
  - compatibility shim
- `run_1to1_follow.launch.py`
  - compatibility shim

Important:
- keep the wrapper interface stable unless explicitly asked to change it
- do not document compatibility shims as the primary runtime path
- if defaults differ between YAML and wrapper-passed launch args, document the effective runtime behavior, not just the YAML file

## Documentation Rules

These two documents must stay current:
- `CURRENT_STATE.md`
- `RUNNING_SIM.md`

Rules:
- keep `CURRENT_STATE.md` up to date when architecture, defaults, active scripts, or validated workflows change
- make sure `RUNNING_SIM.md` always contains the latest correct way to run the simulation
- remove obsolete instructions when they are no longer true
- do not leave dead workflows documented as active paths

## Coding Rules

Keep code clean and minimal.

Preferred direction:
- smaller scripts with clear responsibilities
- fewer hidden side effects
- import existing helpers instead of redefining the same logic in multiple places
- reduce dependency on giant files
- if we are cleaning and something adds more code than it removes, it might not be worth it

Avoid:
- adding helper/debug functionality unless explicitly asked for it
- adding convenience behavior that changes runtime semantics without request
- hardcoded local paths
- hardcoded world-specific behavior
- hardcoded user-specific defaults

When possible:
- reuse shared math/util functions
- keep topic contracts stable
- keep node responsibilities narrow

## Runtime / Architecture Rules

The current architecture should stay modular:
- detection/prediction
- tracking
- estimation
- follow control
- camera control
- simulation/control bridge

Do not make estimator logic control the vehicle directly.

`leader_estimator.py` should:
- consume detections/tracks
- consume detection status only for debug/visibility
- estimate pose / heading
- publish estimate outputs
- stay independent of vehicle-control decisions

## Active Debugging Target

The current open issue is not the package split anymore.

The current debugging focus is the live YOLO follow behavior:
- distance holding is much better than before
- the current main issues are body yaw on sharp turns / reversing and jumpy camera pan/tilt
- detector / tracker stability is better than the body/camera behavior right now
- prefer simplifying or removing over-restrictive logic before adding new control features

Start investigation from:
- `lrs_halmstad/follow/follow_uav.py`
- `lrs_halmstad/follow/camera_tracker.py`
- `lrs_halmstad/sim/simulator.py`
- `lrs_halmstad/perception/leader_estimator.py`

Relevant runtime topics:
- `/coord/leader_detection`
- `/coord/leader_detection_status`
- `/coord/leader_estimate`
- `/coord/leader_estimate_status`
- `/coord/leader_debug_image`
- `/<uav>/psdk_ros2/flight_control_setpoint_ENUposition_yaw`
- `/<uav>/update_pan`
- `/<uav>/update_tilt`
- `/<uav>/pose`

Do not start a new refactor before understanding that runtime behavior.

## Change Discipline

Before adding something new:
- ask whether it is really required for the requested task
- prefer modifying the active path instead of creating parallel dead-end paths

If you find something that:
- is not used
- has no clear function
- lacks purpose

then remove it, but ask first.

Exception:
- obviously dead references or stale documentation can be cleaned up directly when the change is low risk

## Topic / Interface Stability

Be careful with ROS topic names and message flow.

Especially:
- keep simulator topics stable
- keep controller-facing topics stable
- do not rename live topics casually
- prefer wrappers/launch args over changing internal contracts when possible

## Genericity Rules

Code should be generic and dynamic by default.

That means:
- no `/home/<user>` paths in committed code
- avoid assumptions about one machine
- avoid assumptions about one world
- avoid assumptions about one UAV unless explicitly asked
- use workspace-relative or package-relative resolution where possible

## Validation Rules

When changing active runtime code:
- syntax-check touched shell scripts
- compile-check touched Python/launch files
- mention clearly if full end-to-end runtime validation was not performed

When changing entrypoints, docs, or defaults:
- reflect that in `CURRENT_STATE.md`
- reflect the actual run commands in `RUNNING_SIM.md`

## Current Project Direction

The current preferred direction is:
- Nav2 + AMCL on the UGV side
- detached camera as the default sim baseline
- modular perception stack
- cleaner, smaller files
- less hidden behavior
- fewer local hacks

If in doubt:
- preserve structure
- keep behavior explicit
- keep docs current
- ask before removing uncertain code

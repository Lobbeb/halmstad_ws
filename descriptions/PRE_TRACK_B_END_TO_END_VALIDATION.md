# Pre-Track-B end-to-end validation

## Scope

- Branch: `support-chain`
- Combined starting HEAD: `6e927704e4099b5df3c8891b8b07ddb24dbef7a5`
- Incorporated main: `fd1f7029ee6c2c54d93caf2bc62deaa40c4f3757`
- Track A base: `931fbffd476c5005d98fc71c9251e1f0436e02d7`
- World: Baylands
- Resource policy: sequential, headless, minimum UAV/sensor composition
- Track B and EiraX: outside scope

Validation classes: A = static/config, B = unit/integration, C = ROS node
smoke, D = simulation smoke, E = full bounded scenario.

## Acceptance matrix

| ID | Runtime path / contract | Class | Evidence | Status |
|---|---|---:|---|---|
| S1 | Git/branch/history/worktree/process ownership | A | SHA/status/process audit | PASS |
| S2 | Python, shell and YAML syntax | A | compile/parse checks | PASS |
| S3 | Launch arguments, topics, frames, executables and dependencies | A/B | focused contract tests | PASS |
| S4 | ROS packages and typed interfaces | B | colcon build | PASS |
| S5 | AerialSupportLayer global/local semantics | B | plugin GTests | PASS |
| S6 | Machine paths, ROS domain, GPU and cleanup defaults | A/B | audit and regression tests | PASS |
| R1 | Baylands world, UGV spawn, sensors and clock | D | base headless runtime | PASS |
| R2 | Localization and map/odom/base_link TF | D | base headless runtime | PASS |
| R3 | Nav2 lifecycle, mission driver, controller and UGV motion | E | Track A baseline | PASS |
| C1 | Odometry-follow baseline | D | bounded headless campaign | PASS |
| C2 | Direct visual estimate follow | D | bounded headless campaign | PASS |
| C3 | Full visual bridge pipeline | D | bounded headless campaign | PASS |
| C4 | Support observation/forwarding baseline | D | bounded headless campaign | PASS |
| U1 | Single UAV spawn and pose runtime | D | one-UAV runtime evidence | PASS |
| U2a | RGB, CameraInfo and depth contracts | C/D | C2-C4 runtime evidence | PASS |
| U2b | Optional UAV laser contract | C/D | launch/SDF/bridge tests and live attempt | N/A: environment |
| U3 | Follow, gimbal, estimator and perception nodes | C/D | C1-C3 runtime evidence | PASS |
| U4 | UAV SLAM entrypoint | A/C | bounded node smoke | PASS |
| N1a | Network parser and bridge node | A/C | unit and missing-server smoke | PASS |
| N1b | External UAV_UGV OMNeT runtime | C | external simulator availability | N/A: unavailable |
| E1 | Recording and evidence subscriptions | B/D | tests and scenario artifacts | PASS |
| A1 | Track A baseline | E | fresh full-runtime scenario | PASS |
| A2 | Track A valid hazard | E | fresh full-runtime scenario | PASS |
| A3 | Track A explicit clearing | E | fresh full-runtime scenario | PASS |
| X1 | Main Nav2 with support layers and current localization | E | A2/A3 evidence | PASS |
| X2 | Clean task-owned stop and restart | D/E | stop, process audit, restart | PASS |

## Runtime evidence

- Track A baseline: `evidence/pre_track_b_e2e/track_a/baseline_v4_tf_margin/analysis`.
  The mission completed without aerial marking.
- Track A valid hazard: `evidence/pre_track_b_e2e/track_a/valid_v20_delayed/analysis`.
  The dji1 -> dji0 -> UGV typed path was observed; the layer marked the lethal
  core, the global plan changed without crossing it, the physical UGV detoured,
  and NavigateToPose completed successfully.
- Track A clearing: `evidence/pre_track_b_e2e/track_a/clearing_v20_delayed/analysis`.
  An ordered explicit-empty update removed aerial costs, planning returned to
  the cleared geometry, motion continued, and the mission completed.
- C1 odometry follow: `evidence/pre_track_b_e2e/c1_c4_smoke/C1_odom/r06`.
  Valid summary, mean follow error 0.016 m and no stuck event.
- C2 direct visual estimate: `evidence/pre_track_b_e2e/c1_c4_smoke/C2_direct/r03`.
  ONNX CPU detection and fresh estimates drove follow commands; mean follow
  error was 0.019 m and no stuck event occurred.
- C3 full visual bridge: `evidence/pre_track_b_e2e/c1_c4_smoke/C3_bridge/r02`.
  Valid summary, detector health 0.953, estimate freshness 0.993, bridge health
  1.0, and about 10.7 Hz follow/planned-target updates.
- C4 support runtime: `evidence/pre_track_b_e2e/c1_c4_smoke/C4_support/r03`.
  Both support detector statuses, dji1/dji2 selection, UGV awareness/advisory,
  typed forwarding and follow motion were recorded in the 120 s gate.

These results also cover Baylands spawn, clock, localization, TF, Nav2
lifecycle/controller, current single- and three-UAV composition, camera/depth,
gimbal/follow/perception, recording, repeated sequential startup, and
task-owned shutdown. They do not prove a real environmental-hazard detector,
EiraX integration, or Track B.

## Defects corrected during validation

- Deferred aerial-layer TF handling, repeated-snapshot deduplication and
  current-state reporting were corrected and covered by plugin tests.
- Nav2 readiness and mission evidence were made deterministic without changing
  planner/controller tolerances to manufacture a PASS.
- Campaign argument forwarding, ROS-sourced summarization, ONNX model selection,
  C4 recording topics, and support-pane lookup were repaired.
- Simulation-only C1 pose evidence now uses a task-owned Gazebo topic bridge;
  it remains evaluation input and is not operational UAV tracking input.
- Stop scripts now signal only the named task session/process groups, including
  the spawn pane; no broad ROS/Gazebo/tmux cleanup was introduced.
- Low-level UAV laser arguments now reach generated SDF and the ROS bridge.
- ROS nodes used in bounded timeout smokes guard shutdown to avoid duplicate
  `rclpy.shutdown()` exceptions.

## Cleanup and retained compatibility

Cleanup removed temporary plugin diagnostics, an unused SDF-generator field,
and two machine-specific README checkout paths. Existing public topics,
arguments, numerical configuration, C1-C4 semantics and compatibility branches
were retained. Older compatibility paths and large runtime modules were not
rewritten because their removal or restructuring lacked evidence of safe
benefit.

## Limitations

- The optional UAV laser SDF and LaserScan bridge contracts pass static tests,
  and a live publisher was created, but Gazebo reproducibly aborted in an Ogre
  render thread while creating the UAV visual before scan data arrived. Live
  laser data is therefore `N/A: environment`, not PASS.
- `/home/william/omnet_workspace/UAV_UGV` is absent. Parser tests and the bridge
  missing-server smoke pass; external OMNeT composition is `N/A: unavailable`.
- UAV SLAM configured and activated in a bounded node smoke. No sensor-connected
  mapping-quality claim is made.
- Evidence directories are generated artifacts and remain ignored by Git.

## Post-cleanup regression

- Shell syntax: PASS for all repository shell scripts and root entrypoints.
- Python compilation: PASS for scripts and ROS Python packages.
- YAML parsing: PASS for 61 files.
- Focused pytest: 171 passed, 1 skipped.
- `colcon build --symlink-install`: all five packages passed.
- `colcon test`: 224 tests, 0 errors, 0 failures, 7 skipped.
- `AerialSupportLayer` GTests: 21 passed.
- Launch argument resolution: PASS for UAV spawn, Nav2 and follow launches;
  all laser arguments appear in both low-level spawn entrypoints.
- `git diff --check`, active machine-path audit and task-process audit: PASS.

No already-passing runtime scenario was repeated after cleanup because the
cleanup changed only documentation and removed diagnostics/dead state. Runtime
changes had already been exercised by their affected C1-C4, Track A, node and
laser-attempt gates before cleanup.

# Track A full-runtime closure handoff

## Status

- Branch: `support-chain`.
- Repository HEAD from which the final closure work began:
  `dd08e358518f02791ccc511beeca218969390658`.
- Lightweight Track A checkpoint: `4d1953fc2403e9e3601a6362ad843b255f741a92`.
- The authoritative desktop Baylands downstream sequence is complete:
  `baseline = PASS`, `valid = PASS`, `clearing = PASS`.
- Track A downstream of perception is complete for the bounded synthetic
  map-frame hazard contract. Phase 4 Track B real perception is unblocked.
- Generated evidence is ignored and must not be committed.

## Authoritative profile and evidence

The passing runtime profile is the minimal headless downstream composition:

```text
Baylands UGV + localization + Nav2 mission/controller
+ synthetic dji1 typed hazard
+ dji0 association/fusion
+ dji0-to-UGV forwarding
+ global and local AerialSupportLayer
+ passive runtime evidence
```

It does not spawn rendered UAVs, cameras, YOLO, gimbals, support follow, RViz,
or an operational ground-truth bridge.

Authoritative evidence roots:

- Baseline:
  `evidence/support_runtime/final_v6/desktop/baseline_reanalyzed_v3/analysis`
- Valid:
  `evidence/support_runtime/final_v6/desktop/valid_downstream_v1/analysis`
- Clearing recording/live result:
  `evidence/support_runtime/final_v6/desktop/clearing_downstream_v1/`
- Corrected clearing replay of the unchanged bag:
  `evidence/support_runtime/final_v6/desktop/clearing_downstream_v1/analysis_clearing_gate_v2`

All named runtime sessions were stopped after their scenario. Use task-owned
cleanup only; never use a broad ROS, Gazebo, Nav2, or tmux kill command.

## Footprint investigation and correction

The earlier `valid_lifecycle_v1` recording contained five genuine estimated
UGV-footprint penetrations into the active virtual lethal core. They were not a
timestamp mismatch or polygon-calculation artifact.

The exact recorded events were at simulation receipt times 49.732, 50.220,
50.856, 51.712, and 52.568 s. Pose source stamps were only 4-32 ms earlier.
Every event had a fresh confirmed UGV hazard and a comparable costmap snapshot
with 400 lethal core cells and 438 graded halo cells. Reprocessing with exact
Nav2 padding increased the maximum intersection from the preliminary estimate
to 0.010863 m2.

Nav2 1.3.10 semantics used by the analyzer and runtime are:

- raw local/global footprint:
  `[[0.55, 0.45], [0.55, -0.45], [-0.55, -0.45], [-0.55, 0.45]]`;
- default `footprint_padding`: 0.01 m;
- padded footprint vertices: (+/-0.56, +/-0.46) m in `base_link`;
- each recorded polygon uses the AMCL map-frame position and yaw at that pose;
- global costmap: `map`, 0.20 m resolution, 0.95 m inflation radius, scaling 3.0;
- local rolling costmap: `odom`, 0.05 m resolution, 0.80 m inflation radius,
  scaling 4.0;
- MPPI `CostCritic.consider_footprint: true` evaluates the local costmap.

The root cause was that the virtual hazard existed only in the global costmap.
The global planner kept its centerline out of lethal cells, but MPPI's local
footprint check could not see the virtual hazard and cut the corner relative to
the global plan.

The bounded correction:

1. adds the existing `AerialSupportLayer` before inflation in the local costmap;
2. uses `target_frame: odom` there and preserves `target_frame: map` globally;
3. keeps both layer instances disabled by default and lets the existing
   `aerial_support_layer_enable` argument enable both for Track A;
4. synchronizes the plugin's internal origin with a rolling master costmap and
   rerasterizes retained tracks after an origin shift;
5. records raw/padded local and global footprints plus inflation provenance;
6. makes any active-interval padded-footprint intersection a runtime failure.

No controller tuning, inflation retuning, goal-tolerance change, hazard
shrinking, C1-C4 default change, or new UGV motion path was introduced. Nav2
remains the only autonomous UGV motion authority.

## Scenario results

### Baseline: PASS

The recorded baseline proves a single matching NavigateToPose mission,
19.693 m estimated motion, 117 active-goal plans, a stable pre-hazard costmap,
and `SUCCEEDED` with no operational hazard.

Its localization limitation remains explicit. At success the independent
current map-frame TF/AMCL estimates did not reproduce the controller's 1.0 m
XY acceptance. The installed stateful goal checker evaluated the retained
plan-stamped goal in the controller/local frame. Track A accepts the matching
action `SUCCEEDED` plus physical motion and does not claim independent absolute
map-frame accuracy.

### Valid: PASS

Fresh minimal evidence reports no failures:

- typed dji1 -> dji0 -> UGV flow complete;
- global aerial mark: 400 lethal core cells and 438 graded inflation cells;
- both local (`odom`) and global (`map`) support layers initialized enabled;
- passive same-goal automatic replan: 2.959 m material change, 0.328 s after
  first mark, one unambiguous NavigateToPose goal, zero analyzer ComputePath
  requests;
- estimated trajectory: 21.984 m travel and 4.122 m Hausdorff departure from
  baseline;
- 131 active-interval padded footprints: zero intersections, minimum polygon
  clearance 0.443 m;
- mission terminal status: `SUCCEEDED`.

The current valid run did not observe an unintended aerial clear. The
historical lifecycle defect remains covered by regression tests: rejected
nonempty input preserves still-fresh evidence, valid explicit empty input
clears immediately, and accepted tracks otherwise persist through brief gaps
until TTL.

### Clearing: PASS

The live clearing analysis initially failed because the centerline condition
was applied to the entire mission. The UGV crossed the old hazard region only
after it had been explicitly cleared. First center entry was 62.720 s, whereas
costmap restoration occurred at 40.564 s.

The analyzer now evaluates both centerline and configured-footprint avoidance
over the same active mark-to-clear interval. A regression test proves that a
post-clear return through the former region is allowed while an active-period
crossing still fails. Replay of the unchanged recording reports:

- source, fusion, and forwarded explicit-empty snapshots: 40.000 s;
- exact observed costmap restoration: 40.564 s;
- clearing mechanism: `explicit_empty_snapshot`, not TTL or silence;
- active hazard plan avoided the core; post-clear plan returned toward the
  baseline path through the former hazard region;
- no active-interval centerline or padded-footprint intersection;
- navigation continued 18.580 m after clear;
- one goal remained active and the mission `SUCCEEDED`;
- final status: **PASS**, no failures or inconclusive reasons.

## Validation completed

- `colcon build --symlink-install --packages-select
  lrs_halmstad_interfaces lrs_halmstad_nav_plugins lrs_halmstad`: PASS.
- `colcon test --packages-select lrs_halmstad_nav_plugins`: PASS; 18 plugin
  tests, including rolling-origin synchronization.
- `pytest src/lrs_halmstad/test/test_support_hazard_evidence.py`: PASS; 56
  tests, including exact padded-footprint overlap and post-clear interval
  semantics.
- Baylands local/global layer configuration and target-frame contract check:
  PASS.
- Runtime parameter inspection in valid confirmed both local and global layer
  instances enabled.

The final closure checkpoint requires the focused Python, plugin, shell, build,
configuration-contract, and `git diff --check` validations to pass together.

## Proof boundary and next phase

Track A proves the downstream support architecture for this bounded synthetic
Baylands experiment: typed propagation, global/local costmap influence,
automatic same-goal replanning, estimated route deviation, configured padded-
footprint avoidance while active, explicit clearing, continued navigation,
and mission completion.

It does not prove:

- a real environmental-hazard detector or its accuracy;
- full three-UAV composition with cameras and perception;
- absolute Gazebo-world hazard clearance or localization accuracy;
- general runtime safety outside the recorded Baylands scenarios;
- EiraX integration.

Phase 4 Track B is next. Define the real hazard classes, dataset split and
annotation policy, model evaluation gates, and projection inputs. The real
detector must plug into the validated typed projector/fusion/forwarding path.
Do not redesign the downstream interface or begin EiraX changes without an
explicit task.

# Master implementation plan

## Purpose and authority

This is the canonical roadmap and status source for `halmstad_ws`. Use it to
recover project direction after a new chat, workstation switch, or long pause.
`AGENTS.md` defines working rules; current phase handoffs provide detailed
evidence and operator context. If older documentation conflicts with this plan
or current code, verify the implementation and update this plan rather than
silently following stale material.

Historical documents may use different phase numbers. The order below is the
canonical 0-to-100 project sequence.

## Current state

- Active branch: `support-chain`.
- Current Track A world: Baylands. Warehouse material is legacy/reference
  unless needed to preserve compatibility.
- Lightweight Track A: PASS.
- Downstream Track A: **DONE / PASS**. The authoritative desktop sequence
  `baseline -> valid -> clearing` passed. The valid gate includes configured
  padded-footprint avoidance while the virtual lethal core is active; clearing
  proves ordered explicit empty propagation, exact cost restoration, continued
  navigation, and mission completion.
- Laptop full and reduced Baylands runtime attempts were environment-limited
  and inconclusive, not demonstrated functional failures.
- Pre-Track-B shared-runtime synchronization: PASS. Remote `main` at
  `fd1f7029ee6c2c54d93caf2bc62deaa40c4f3757` was merged into
  `support-chain` from the completed Track A checkpoint
  `931fbffd476c5005d98fc71c9251e1f0436e02d7` using normal merge history.

### Current next gate

Phase 4 Track B real perception is now unblocked and is the next implementation
phase. Define the real environmental-hazard classes and dataset/evaluation
contract before training. The detector must publish through the already
validated typed projector/fusion/forwarding interface; it must not redesign the
Nav2 integration.

## Final system

The intended operational path is:

```text
real support-UAV perception
-> metric map-frame hazard observation
-> typed AerialHazardArray
-> dji0 association/fusion
-> dji0-to-UGV forwarding
-> /coord/ugv/aerial_hazards
-> AerialSupportLayer
-> Nav2 global and local costmaps
-> automatic Nav2 replanning
-> existing UGV navigation/controller
-> safe UGV traversal
```

Nav2 remains the UGV motion authority. Support UAVs provide hazard awareness,
not UGV localization or direct UGV `cmd_vel`. Existing UGV localization,
odometry, mission, planner, and controller paths remain authoritative.
Synthetic hazards are temporary Track A substitutes for a future real
environmental-hazard detector. Simulation/debug ground truth must not become an
operational navigation or tracking input.

## Phase order

### Phase 0 - Shared-runtime reconciliation

**Status: DONE**

- Relevant Ruben/team `main` changes were selectively reconciled into
  `support-chain` at `783b2ef`.
- Existing C1-C4 and shared runtime paths were preserved.
- No blind replacement from `main` was used.

### Phase 1 - Typed hazard/support-chain foundation

**Status: DONE**

- `AerialHazardArray` carries metric detections plus contributing UAVs, track
  state, first/last seen times, TTL, support quality, and provenance.
- dji1 publishes to `/coord/support/dji1/aerial_hazards`.
- dji0 associates and conservatively selects/fuses evidence, publishing
  `/coord/dji0/aerial_hazards`.
- The dji0-to-UGV forwarder publishes `/coord/ugv/aerial_hazards`.
- `AerialSupportLayer` consumes the UGV topic in the Nav2 global and local
  costmaps when explicitly enabled; both instances remain disabled by default.
- Typed dji2 input exists at `/coord/support/dji2/aerial_hazards` but remains
  opt-in and disabled by default.

### Phase 2 - Lightweight Track A validation

**Status: DONE / PASS**

Validated scenarios:

- `baseline`
- `valid`
- `clearing`
- `off_route`
- `low_confidence`
- `stale`
- `layer_disabled`

This proves the bounded Baylands typed flow, configured fixed-global-costmap
marking/clearing behavior, planner response, and negative controls. It does not
prove a live NavigateToPose replanning cycle, physical UGV motion or detour,
goal completion, runtime safety, or detector accuracy. Checkpoint: `4d1953f`.

### Phase 3 - Full Baylands Track A runtime

**Status: DONE / PASS**

The authoritative desktop sequence `baseline -> valid -> clearing` passed using
the minimal headless downstream profile: real Baylands UGV, localization, Nav2
mission/controller, synthetic dji1 typed source, dji0 fusion, forwarding,
AerialSupportLayer, and passive evidence. No rendered UAVs, cameras, YOLO,
gimbals, support follow, RViz, or operational ground-truth input were used.

Baseline evidence is under
`evidence/support_runtime/final_v6/desktop/baseline_reanalyzed_v3/analysis`.
It proves one matching NavigateToPose mission, 19.693 m estimated motion,
117 observed active-goal plans, stable pre-hazard costmap, and `SUCCEEDED`.
Its historical localization discrepancy remains visible: success-time map-frame
TF and AMCL estimates were outside the configured 1.0 m XY tolerance even
though Jazzy's stateful controller-frame/timestamp goal check succeeded. Track A
does not claim independent absolute map-frame localization accuracy.

The earlier valid recording exposed five genuine estimated footprint
penetrations into the active virtual lethal core. The analyzer now reproduces
Nav2 1.3.10 footprint semantics exactly: the configured 1.10 x 0.90 m polygon
is expanded by the default 0.01 m `footprint_padding`, producing vertices at
(+/-0.56, +/-0.46), then transformed by each map-frame AMCL pose and yaw. The
five samples were fresh and aligned with an active 400-cell lethal core; maximum
intersection was 0.010863 m2. The cause was architectural: the support layer
existed only in the global costmap. The global planner avoided lethal centerline
cells, while MPPI's configured footprint collision check used the local costmap
and could cut the corner without seeing the virtual hazard.

The bounded correction adds the same opt-in support layer before inflation in
the local rolling `odom` costmap and synchronizes the plugin's internal origin
with the rolling master costmap. Both local and global instances remain disabled
by default and are enabled together only through the existing Baylands support
argument. No controller weights, inflation radii, goal tolerances, C1-C4
defaults, hazard geometry, or motion authority changed.

Fresh minimal valid evidence is under
`evidence/support_runtime/final_v6/desktop/valid_downstream_v1/analysis` and is
**PASS** with no failures. Both support layers were observed enabled. Typed flow
completed; the global map contained 400 lethal-core cells and 438 graded halo
cells; a passive same-goal plan changed 2.959 m 0.328 s after marking; the
evidence process issued zero ComputePath requests; the estimated trajectory
deviated 4.122 m from baseline; all 131 active-interval padded footprints had
zero intersection and at least 0.443 m polygon clearance; and NavigateToPose
returned `SUCCEEDED` after 21.984 m estimated travel.

Fresh minimal clearing evidence is under
`evidence/support_runtime/final_v6/desktop/clearing_downstream_v1/`. The live
analysis initially applied the active-hazard centerline rule to the full mission
and incorrectly failed when the UGV traversed the former region after clearing.
A regression-tested analyzer correction now evaluates centerline and footprint
safety over the same mark-to-clear interval. Offline replay of the unchanged bag
at `analysis_clearing_gate_v2/` is **PASS**: explicit empty snapshots propagated
dji1 -> dji0 -> UGV at 40.000 s; exact observed costmap restoration occurred at
40.564 s; the post-clear plan returned toward baseline through the former
hazard; navigation continued 18.580 m; and the mission `SUCCEEDED`. No active-
interval centerline or padded-footprint intersection occurred.

The result proves Track A downstream of perception for the synthetic map-frame
hazard contract: typed propagation, costmap marking and inflation, automatic
same-goal replanning, estimated physical route deviation, configured-footprint
avoidance while active, explicit clearing, continued navigation, and mission
completion. It does not prove real detector accuracy, three-UAV composition,
absolute Gazebo-world clearance, or general runtime safety outside this bounded
Baylands experiment. Optional world-registration diagnostics remain
inconclusive and evaluation-only.

### Phase 4 - Track B real perception

**Status: NOT STARTED / UNBLOCKED; NEXT PHASE**

Phase 3 has passed. Next:

1. Define the environmental hazard classes and operational scope.
2. Select or collect representative data.
3. Annotate with a documented policy and quality checks.
4. Train candidate models reproducibly.
5. Validate accuracy and failure modes on held-out data.
6. Integrate bounded runtime inference.
7. Project detections using synchronized RGB, depth, and CameraInfo.
8. Resolve acquisition-time TF into the required metric/map frame.
9. Publish the same typed hazard contract already validated downstream.

The repository already contains a dji1 RGB-D projection/transport fixture using
an existing `ugv` detector class. That validates geometry integration only; it
is not a trained environmental-hazard detector or a Track B PASS. Track B must
replace the synthetic source, not redesign the fusion, forwarding, costmap, or
Nav2 architecture.

### Phase 5 - Combined real-perception support-chain validation

**Status: FUTURE**

Validate the complete path:

```text
real detector -> projection/localization -> fusion -> UGV
-> costmap -> replanning -> physical UGV motion
```

Measure detector accuracy, false positives/negatives, localization error,
confidence behavior, end-to-end latency, observation freshness, and downstream
navigation effects. Keep detector claims separate from integration claims.

### Phase 6 - EiraX/Basuedo integration

**Status: FUTURE**

- EiraX is the downstream integration target.
- Keep William's typed support-chain interface stable unless evidence reveals a
  genuine generic defect; adapt EiraX rather than redesigning the chain around
  it.
- Preserve Basuedo's localization/SLAM, Nav2, controller, and mission
  architecture. UAVs must not gain UGV `cmd_vel` authority.
- William's Baylands hazard frame currently uses `map`; EiraX navigation uses a
  start-relative `odom`. Establish and validate the map-to-odom/datum
  registration before transforming hazards.
- Do not confuse local SLAM correction with cross-system datum registration.
- Preserve acquisition timestamps, track IDs, state, TTL, geometry, and
  covariance through transforms.
- Rotate/reproject covariance correctly where required.
- The generic AerialSupportLayer rolling-origin correction is implemented and
  unit-tested; validate its frame/topic configuration in EiraX before enabling it there.

These are future integration requirements, not claims that EiraX integration
or registration is already implemented.

### Phase 7 - Final shared-runtime reconciliation

**Status: FUTURE**

The intermediate pre-Track-B synchronization is complete. It incorporated
remote `main` `fd1f7029ee6c2c54d93caf2bc62deaa40c4f3757` on `support-chain`.
That exact main head had already received a selective runtime audit in the
earlier `783b2ef` checkpoint. During the normal-history merge, overlapping
startup, follow, localization, Nav2, simulation and support files retained the
validated Track A versions. Compatible main-only plotting, network monitoring,
UAV SLAM, spawn, camera/laser, model, world, dataset and team-documentation
changes were retained.

The merge adaptations made new environment/debug files repository-portable,
kept GPU adapter selection opt-in, prevented a new unconditional ROS domain 3
override, and made the YOLO debug matrix valid YAML. Broad cleanup, removal of
the guarded simulation clock, collision-monitor changes, unrelated Nav2 tuning
and localization changes were not reintroduced. Validation passed shell,
Python and YAML checks; 161 Python tests (1 skipped); builds for
`lrs_halmstad_interfaces`, `lrs_halmstad_nav_plugins` and `lrs_halmstad`; and
all 18 `AerialSupportLayer` GTests. No Gazebo smoke was needed because the
validated Track A core remained byte-identical to the Track A checkpoint. The
integration delta passes `git diff --check`; inherited main dataset/result and
mesh assets retain pre-existing whitespace warnings and were not mass-rewritten.

- Fetch the latest `origin/main` only when this phase is explicitly authorized.
- Audit new Ruben/team changes for relevant behavior and dependencies.
- Reconcile selectively; never blindly overwrite `support-chain`.
- Rerun affected unit, build, launch-contract, and runtime validation.

### Phase 8 - Final validation and thesis closure

**Status: FUTURE**

- Run the final combined, reproducible validation matrix.
- Preserve self-contained evidence, configuration provenance, and limitations.
- Build thesis figures, tables, and results only from actual evidence.
- Make no claims beyond the validated world, inputs, metrics, and runtime scope.
- Separate demonstrated limitations and future work from completed results.

## Non-negotiable invariants

- Baylands is the current Track A runtime.
- Do not silently alter C1-C4 or shared runtime behavior.
- Nav2 owns autonomous UGV movement.
- No support-UAV-to-UGV `cmd_vel` path.
- No simulation truth as operational navigation or tracking input.
- Typed support hazards remain opt-in unless explicitly changed and validated.
- Typed dji2 fusion remains disabled by default.
- `AerialSupportLayer` remains disabled by default outside explicit validation.
- Complete Track A before starting Track B.
- Complete Track B before or alongside final real-perception integration.
- Modify EiraX only when its phase is explicitly authorized.
- Prefer small evidence-driven fixes over speculative retuning or redesign.
- Planner evidence is not physical-motion evidence.
- Lightweight PASS is not full-runtime PASS.
- Downstream Track A evidence with a synthetic source is not three-UAV
  composition or real-perception evidence.
- Generated evidence, bags, build/install/log outputs, and runtime artifacts
  must not be silently committed.
- Do not commit, push, merge, or rebase without explicit authorization.

## Checkpoints

- `783b2ef` - shared-runtime reconciliation.
- `4d1953f` - validated lightweight Track A.
- `e76b99e` - success-time TF evidence correction.
- `bcc83771b25bd15f7f573a56af380e6e2998660a` - last committed Track A
  runtime-code checkpoint before the evidence-policy correction.
- `dd08e358518f02791ccc511beeca218969390658` - documentation checkpoint from
  which the final Track A closure work began.
- `931fbffd476c5005d98fc71c9251e1f0436e02d7` - completed Track A downstream
  support-chain validation, used as the pre-Track-B synchronization base.

## Maintaining this plan

When a gate passes or evidence demonstrates a defect, update the relevant phase
status, checkpoint, proof boundary, and next gate here. Keep detailed run output
in phase handoffs or ignored evidence directories, not in this roadmap.

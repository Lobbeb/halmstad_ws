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
- Current implementation baseline: `bcc83771b25bd15f7f573a56af380e6e2998660a`.
- Current Track A world: Baylands. Warehouse material is legacy/reference
  unless needed to preserve compatibility.
- Lightweight Track A: PASS.
- Full-runtime Track A: implementation complete; authoritative desktop runtime
  pending.
- Laptop full and reduced Baylands runtime attempts were environment-limited
  and inconclusive, not demonstrated functional failures.

### Current next gate

Run authoritative desktop Baylands full-runtime validation in this order:

1. `baseline`
2. `valid`, only if baseline passes
3. `clearing`, only if valid passes

Use the exact commands and evidence criteria in
`descriptions/TRACK_A_FULL_RUNTIME_WIP_HANDOFF.md`. Do not expand Track A
implementation unless these runs reveal a demonstrated defect. After all three
authoritative scenarios pass, begin Phase 4 Track B perception work.

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
-> Nav2 global costmap
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
- `AerialSupportLayer` consumes the UGV topic in the Nav2 global costmap.
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

**Status: IMPLEMENTATION COMPLETE / AUTHORITATIVE DESKTOP RUNTIME PENDING**

Required authoritative sequence: `baseline -> valid -> clearing`.

Baseline must prove:

- one real NavigateToPose mission;
- actual UGV motion;
- runtime goal-checker parameters and correct success semantics;
- a hazard-relevant baseline plan and trajectory without operational hazard.

Valid must additionally prove:

- synthetic dji1 hazard publication, dji0 fusion, and UGV forwarding;
- aerial costmap lethal marking plus configured inflation;
- a materially changed automatic plan under the same unambiguous active goal,
  with no evidence-side ComputePath request;
- actual post-replan physical UGV deviation from the baseline trajectory;
- hazard avoidance, replanned-path tracking, and goal completion.

Clearing must additionally prove:

- the hazard and aerial cost were marked first;
- ordered explicit empty propagation from dji1 to dji0 to UGV;
- exact restoration of the observed pre-hazard costmap region;
- continued healthy navigation, a post-clear plan, and goal completion.

The success-time common-timestamp TF evidence is corrected at `e76b99e`.
Goal semantics, passive automatic-replanning attribution, physical-trajectory
proof, explicit clearing, and inconclusive classifications are implemented in
the evidence analyzer. The complete runtime-closure implementation is at
`bcc83771b25bd15f7f573a56af380e6e2998660a`.

Full three-UAV and reduced-resource Baylands attempts on the laptop were
environment-limited. The reduced profile is diagnostic only and cannot replace
authoritative three-UAV evidence. Run the heavy sequence on the stronger
desktop without speculative Nav2/controller retuning.

### Phase 4 - Track B real perception

**Status: NOT STARTED / DEFERRED UNTIL TRACK A GATE**

After Phase 3 passes:

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
- Correct and validate the known generic AerialSupportLayer rolling-costmap
  origin issue before using it with EiraX's rolling global costmap.

These are future integration requirements, not claims that EiraX integration
or registration is already implemented.

### Phase 7 - Final shared-runtime reconciliation

**Status: FUTURE**

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
- Reduced-resource diagnostics are not authoritative three-UAV evidence.
- Generated evidence, bags, build/install/log outputs, and runtime artifacts
  must not be silently committed.
- Do not commit, push, merge, or rebase without explicit authorization.

## Checkpoints

- `783b2ef` - shared-runtime reconciliation.
- `4d1953f` - validated lightweight Track A.
- `e76b99e` - success-time TF evidence correction.
- `bcc83771b25bd15f7f573a56af380e6e2998660a` - latest pushed Track A
  runtime-closure state and current implementation baseline.

## Maintaining this plan

When a gate passes or evidence demonstrates a defect, update the relevant phase
status, checkpoint, proof boundary, and next gate here. Keep detailed run output
in phase handoffs or ignored evidence directories, not in this roadmap.

# Visual Follow Current Plan

## Current Status Note

This file is still useful as historical context, but it is **not** the active top priority right now.

Current active work has shifted toward:

- capturing training data for the leader UAV (`dji0`)
- capturing training data for the two support UAVs (`dji1`, `dji2`)
- making the existing Baylands capture workflows reliable

Current next-session problem to solve:

- Baylands **AMCL pose / Nav2 waypoint pose alignment**

So for the next session, do not default into more visual-follow redesign work unless the user explicitly asks for it. Reuse the current code paths and debugging tools first.

This note captures the current agreed direction for the visual-follow thesis work on branch `live-yolo-runtime-refactor`.

## Current Safe Base

- Current safe pushed checkpoint: `681d7ef`
- This is the chosen best checkpoint from the current Chen / ByteTrack / Falanga-guided tuning lane.
- Keep this as the baseline unless a future change proves better in the same clean validation flow.

## What We Already Improved

- `onnx_cpu + async + ByteTrack` improved the old runtime/staleness problem a lot.
- The pipeline is less binary than before.
- Recovery/camera behavior is better than the old baseline.
- Launcher/tmux cleanup is more trustworthy now.
- Weak detector status can help camera-only recentering without promoting weak detections into the full follow pipeline.

## Main Remaining Problem

The dominant remaining long-run failure is still:

- upstream `NO_DET`
- especially `no_candidates_above_conf`
- followed by `LOST -> INVALID/HOLD`

So the main remaining issue is more upstream continuity/recall collapse than controller architecture.

The honest current judgment is:

- this branch is clearly better than the older baseline
- but repeatability is still variable across long runs
- and the last focused implementation passes after `681d7ef` did not produce a better checkpoint

## Main Blueprints

- `Chen / Sensors paper` = main blueprint
  - measurements, target state, prediction, graceful degradation
- `ByteTrack paper` = continuity companion
  - preserve weak-but-consistent target evidence
- `Falanga / PAMPC ideas` = perception-aware companion
  - move in ways that help keep the target visible

## What We Are Not Doing

- no GPU bridge for now
- no redesign / new architecture
- no full controller rewrite
- no full Chen / PAMPC / NMPC copy
- no random threshold thrashing

## Current Disposition

The original immediate-next-step plan in this file was executed:

1. inspect the lead-up to `no_candidates_above_conf`
2. prove whether weak-but-real UGV candidates still exist just before collapse
3. test narrow continuity-aware / low-confidence rescue ideas
4. keep only changes that improved the same clean validation path

Outcome:

- weak-but-real target-class cues did exist before some collapses
- the best kept improvement from that line of work is the pushed checkpoint `681d7ef`
- later follow-up passes after `681d7ef` were mixed or worse and were rolled back
- the current recommendation is to stop adding more implementation passes in this same narrow lane

## Final Validation Snapshot

Representative kept / final artifacts:

- best short strong run:
  - `validation_results/upstream_leadup_audit_v5_weak_status_center/summary.json`
- headless repeatability references:
  - `validation_results/repeatability_headless_v1_120s/summary.json`
  - `validation_results/repeatability_headless_v2_120s/summary.json`
- GUI repeatability reference:
  - `validation_results/repeatability_gui_v1_120s/summary.json`
- final wrap-up reruns on the chosen base:
  - `validation_results/final_headless_base_120s/summary.json`
  - `validation_results/final_gui_base_120s/summary.json`

High-level conclusion:

- `681d7ef` remains the best chosen checkpoint from this lane
- GUI on the chosen base is still clearly better than the old GUI baseline
- headless long-run repeatability is still variable
- further micro-tuning in the same lane did not reliably improve the chosen base

## Later Idea To Keep In Mind

Visual-only motion prediction is a valid later refinement, but not a new branch of work right now.

Important constraints:

- simplex / vision-only only
- no duplex coordination
- no UGV runtime state feed
- no hidden ground-truth dependency

Current judgment:

- the stack already has meaningful visual-only prediction and continuity
- do **not** add a brand-new Kalman/UKF subsystem right now
- if needed later, strengthen the existing predictor/filter layers inside the current stack instead

Relevant existing files:

- `src/lrs_halmstad/lrs_halmstad/follow/selected_target_filter.py`
- `src/lrs_halmstad/lrs_halmstad/perception/visual_target_estimator.py`
- `src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py`
- `src/lrs_halmstad/lrs_halmstad/follow/camera_tracker.py`

## If More Improvement Is Needed

If more improvement is required beyond `681d7ef`, prefer a bigger-category change instead of more micro-tuning in this same lane.

Recommended order:

1. camera / view geometry changes
   - altitude
   - standoff distance
   - mount pitch
   - keeping the UGV larger / better framed
2. model / data improvement
   - improve training data for weak / long-range / warehouse views
   - improve model robustness where confidence currently collapses
3. only after that consider bigger architecture changes

## Validation Discipline

For each pass:

1. make one focused change area
2. run the same clean validation path
3. compare:
   - `OK / NO_DET`
   - `TRACKED / LOST`
   - `ACTIVE / INVALID`
   - longest dead stretches
4. keep only changes that clearly improve the clean result

# Visual Follow Current Plan

This note captures the current agreed direction for the visual-follow thesis work on branch `live-yolo-runtime-refactor`.

## Current Safe Base

- Current safe pushed checkpoint: `90e1165`
- Keep this as the baseline unless a new change proves better in the same clean validation flow.

## What We Already Improved

- `onnx_cpu + async + ByteTrack` improved the old runtime/staleness problem a lot.
- The pipeline is less binary than before.
- Recovery/camera behavior is better than the old baseline.
- Launcher/tmux cleanup is more trustworthy now.

## Main Remaining Problem

The dominant remaining long-run failure is still:

- upstream `NO_DET`
- especially `no_candidates_above_conf`
- followed by `LOST -> INVALID/HOLD`

So the main remaining issue is more upstream continuity/recall collapse than controller architecture.

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

## Immediate Next Step

Focus upstream, not downstream.

Main files:

- `src/lrs_halmstad/lrs_halmstad/perception/leader_tracker.py`
- `src/lrs_halmstad/lrs_halmstad/perception/onnx_backend.py`
- `src/lrs_halmstad/lrs_halmstad/follow/selected_target_filter.py`

Goal:

1. inspect the lead-up to `no_candidates_above_conf`
2. prove whether weak-but-real UGV candidates still exist just before collapse
3. if they do, add a narrow continuity-aware low-confidence rescue there
4. validate on the same clean soak path and only keep it if metrics improve

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

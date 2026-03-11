# `william_updated` Merge Notes

Updated: 2026-03-11

## Branch refs

- Current working branch: `splitting_everything` at `4461e74`
- `main`: local `4402712`
- Local stale remote-tracking ref: `origin/william_updated` at `e43cd5d`
- Live GitHub ref fetched separately: `github_live/william_updated` at `eb5eb47`

## Current graph snapshot

- `splitting_everything` vs `main`: `0 behind / 6 ahead`
- Live `william_updated` vs `main`: `3 behind / 3 ahead`
- `splitting_everything` vs live `william_updated`: `9 ahead / 3 behind`

## What the live `william_updated` branch adds

### 1. Recorder and tmux orchestration

Useful, mostly additive:

- Adds `run_record_experiment.sh`
- Extends `run_tmux_1to1.sh` with:
  - `record:=true|false`
  - `record_profile:=default|vision`
  - `record_tag:=...`
  - `record_out:=...`
  - `record_delay_s:=...`
- Extends `stop_tmux_1to1.sh` so it also stops the recorder and kills matching `ros2 bag record` processes as fallback

Important layout difference:

- William branch keeps these as root-level scripts
- Current branch moved orchestration into `scripts/` and dispatches through `./run.sh`

Conclusion:

- The recorder/tmux logic is worth porting
- It should be adapted into the current `scripts/` layout, not merged as-is

### 2. Perception stack refactor

William live branch includes a newer estimator-related refactor:

- rewrites `leader_estimator.py`
- adds `leader_projection.py`
- adds `leader_tracking.py`
- adds `leader_types.py`
- adds launch/config args:
  - `tracker_enable`
  - `tracker_name`
  - `tracker_yaml`

High-level design on William branch:

- tracking is performed inside the estimator-side flow
- estimator consumes image/camera/depth/UAV pose directly
- `leader_tracking.py` wraps Ultralytics tracking
- `leader_projection.py` handles depth/ground projection and sanity checks

## How this differs from the current branch

Current branch already has most of the same responsibilities, but split differently:

- `perception/leader_detector.py`
  - raw detection
- `perception/leader_tracker.py`
  - Ultralytics track-mode wrapper
- `perception/leader_estimator.py`
  - consumes external detections and projects them into world coordinates
- `perception/detection_protocol.py`
  - transport format between detector/tracker and estimator
- `perception/yolo_common.py`
  - shared Ultralytics helpers

Launch architecture is also different:

- Current branch uses unified `src/lrs_halmstad/launch/run_follow.launch.py`
- William branch still routes behavior through the older launch/script shape

## Important behavioral gap

This is the main reason not to merge wholesale.

Current branch has OBB-aware detection payloads:

- `obb_corners`
- `obb_heading_yaw`
- `source`

Those are used by the current estimator for ground projection and heading inference.

William live branch `leader_types.Detection2D` keeps:

- bbox
- class info
- track metadata

But it drops the current branch OBB fields entirely.

If William's perception code is merged wholesale, there is a real risk of regressing:

- OBB-based ground point selection
- OBB-derived heading estimation
- detector/tracker source information carried through the protocol

## Safe ports that look worth doing

### A. Port directly

- `run_record_experiment.sh`
- recorder pane/state logic from `run_tmux_1to1.sh`
- recorder cleanup logic from `stop_tmux_1to1.sh`

### B. Port as additive changes into current perception split

From William tracker/types:

- `track_hits`
- `track_age_s`
- `track_state`
- `track_switched`

These fit well into the current split architecture if added to:

- `perception/detection_protocol.py`
- `perception/leader_tracker.py`
- optionally estimator status/debug output

### C. Port selectively, not wholesale

Potentially useful as internal refactors only:

- `leader_projection.py`
  - depth sampling
  - range selection
  - smoothing
  - jump/speed/bearing sanity checks
- parts of `leader_tracking.py`
  - cleaner tracker config resolution
  - tracker state bookkeeping

But these should be transplanted into the current split pipeline, not used to replace it.

## Things to avoid merging wholesale

- Do not replace current `run_follow.launch.py` with William's older launch flow
- Do not collapse the current `detector -> tracker -> estimator` split back into one estimator-driven pipeline without a deliberate redesign
- Do not drop current OBB payload support from `detection_protocol.py`
- Do not move current `scripts/` back to root-level runner scripts

## Suggested merge strategy for another agent

1. Port recorder support first
   - adapt William root script into `scripts/run_record_experiment.sh`
   - update `scripts/run_tmux_1to1.sh`
   - update `scripts/stop_tmux_1to1.sh`

2. Port additive tracker metadata
   - extend `perception/detection_protocol.py`
   - populate the new fields in `perception/leader_tracker.py`
   - optionally expose them in estimator status/debug text

3. Treat William estimator/tracker/projection files as a reference implementation
   - mine them for helper extraction and parameter naming
   - keep the current split architecture unless a larger redesign is explicitly intended

4. Preserve current OBB path
   - any port from William perception code must keep:
     - `obb_corners`
     - `obb_heading_yaw`
     - OBB-based heading/ground projection behavior

## Most relevant files to compare

Current branch:

- `run.sh`
- `scripts/run_tmux_1to1.sh`
- `scripts/stop_tmux_1to1.sh`
- `scripts/run_1to1_yolo.sh`
- `src/lrs_halmstad/launch/run_follow.launch.py`
- `src/lrs_halmstad/lrs_halmstad/perception/leader_detector.py`
- `src/lrs_halmstad/lrs_halmstad/perception/leader_tracker.py`
- `src/lrs_halmstad/lrs_halmstad/perception/leader_estimator.py`
- `src/lrs_halmstad/lrs_halmstad/perception/detection_protocol.py`
- `src/lrs_halmstad/lrs_halmstad/perception/yolo_common.py`

Live William branch reference:

- `run_record_experiment.sh`
- `run_tmux_1to1.sh`
- `stop_tmux_1to1.sh`
- `src/lrs_halmstad/lrs_halmstad/leader_estimator.py`
- `src/lrs_halmstad/lrs_halmstad/leader_tracking.py`
- `src/lrs_halmstad/lrs_halmstad/leader_projection.py`
- `src/lrs_halmstad/lrs_halmstad/leader_types.py`


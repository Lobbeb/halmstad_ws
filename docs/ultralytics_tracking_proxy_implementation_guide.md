# Codex Implementation Guide — Add Ultralytics Tracking to Proxy Stage

## Purpose

This guide is for the **current proxy stage**, not full 4.1.

Goal:
Use Ultralytics tracking on top of the existing proxy detection path so the system gets **identity continuity** across frames instead of treating every frame as a fresh guess.

This is the standard next step after detection and before full prediction/Kalman.
Ultralytics officially supports object tracking with **BoT-SORT** and **ByteTrack** on top of Detect/Segment/Pose models, and the tracking output is the normal detection output plus persistent object IDs. Tracking reuses the same detection model and does **not** require a separate tracker-specific training stage.

This guide is **not** the full final architecture and is **not** Substep 4.1 yet.
It is the “finish proxy enough” tracking layer.

## 1) Why do this now?

Current proxy facts:
- The debug image already shows the UGV being detected as proxy classes like `book`, `cell phone`, `suitcase`, `traffic light`, etc.
- The main problem is no longer “YOLO sees nothing.”
- The main problem is retention/reacquire and unstable continuity under motion, trees, and frame-edge drift.

What tracking gives you:
- a persistent ID per object across frames
- reduced jumping between candidate boxes
- a more stable target to camera-lock and follow

Ultralytics docs state:
- tracking output is the standard detection output plus object IDs
- supported trackers are **BoT-SORT** and **ByteTrack**
- BoT-SORT is the default tracker
- you can use a custom-trained detect model or an official model in tracking mode
- tracker behavior is tuned through YAML configs such as `botsort.yaml` and `bytetrack.yaml`

So this is a standard implementation path, not a custom hack.

## 2) What to implement in this repo

### High-level design

Keep the current proxy detector as the **measurement source**.

Add a tracking layer on top of it:

**proxy detect → tracker ID continuity → selected tracked target → leader_estimate / camera lock / follow**

Important:
- Do **not** replace the whole pipeline.
- Do **not** jump to full predictor/Kalman here.
- Do **not** redesign the proxy classes first.

This step is only:
- keep the same proxy detection classes
- add stable object ID continuity
- feed downstream control from the tracked target instead of raw per-frame box guesses

## 3) Tracker choice

### Recommended default: BoT-SORT
Why:
- It is the default tracker in Ultralytics
- It supports ReID if you ever want it later
- It is the most natural first tracker to try in Ultralytics docs

### Fallback / simpler alternative: ByteTrack
Why:
- Also officially supported
- Good if BoT-SORT integration feels too heavy or unstable in your current repo

Ultralytics docs list both:
- `botsort.yaml`
- `bytetrack.yaml`

### Decision for this repo
Start with:
- **BoT-SORT first**
- if integration or behavior is bad, test **ByteTrack**
- do not enable ReID initially unless clearly needed

## 4) How to integrate it (repo-friendly version)

You have two practical integration options.

### Option A — Direct Ultralytics track API (preferred if clean)
Use the Ultralytics model in `track` mode instead of `predict` mode.

Ultralytics docs show:
- `model = YOLO("path/to/model.pt")`
- `results = model.track(source=..., tracker="botsort.yaml")`
- CLI equivalent `yolo track model=... source=... tracker="bytetrack.yaml"`

#### In your repo
Inside the estimator/perception node:
1. Load the current proxy model once
2. For each frame, call tracking instead of plain detection
3. Read the returned tracked boxes and IDs
4. Filter only the allowed proxy classes
5. Pick one tracked target ID and keep it as the active leader
6. Pass that tracked target to the existing `leader_estimate` path

### Option B — Keep current detection call, add internal continuity layer
If direct `model.track(...)` is awkward in your ROS loop, then:
1. Keep the current detection inference
2. Build a lightweight internal association layer that behaves like a simple tracker:
   - associate detections to current target using IoU + center distance + class allowlist
   - keep current target alive for a short timeout
   - assign a local track ID yourself

This option is less “pure Ultralytics track integration” but easier if the repo structure makes direct tracking awkward.

### Recommendation
Try **Option A first** if integration is straightforward.
If not, implement **Option B** using the same tracking logic principles and leave full official tracker integration as later cleanup.

## 5) What the downstream pipeline should consume

Downstream control should no longer consume “best raw box this frame.”

It should consume one **tracked target** with:
- `track_id`
- `class_name`
- `confidence`
- `bbox`
- `bbox_center_u`
- `bbox_center_v`
- `age / frames_since_seen`
- `association_status` (`DETECTED`, `TRACKED`, `LOST`)

This tracked target becomes the input to:
- `leader_estimate`
- camera lock
- follow logic
- reacquire logic

## 6) Target-selection logic on top of tracking

Once tracking is added, the target-selection problem becomes simpler.

### Rule
If there is an active target ID:
- keep that ID unless it is truly lost

Only switch to a new target if:
- the old target is lost beyond timeout, or
- the new target is overwhelmingly better and local continuity rules allow it

### Keep existing proxy class allowlist
Use the same allowed proxy classes you already use.
Tracking should reduce jumping; it should not require redesigning the classes immediately.

## 7) Initial tracker configuration (practical defaults)

Ultralytics tracker configs expose parameters such as:
- `track_high_thresh`
- `track_low_thresh`
- `new_track_thresh`
- `track_buffer`
- `match_thresh`
- `fuse_score`
- `with_reid` (BoT-SORT only)

What they mean, practically:
- `track_high_thresh`: confidence threshold for first association; if too high, tracks won’t update easily
- `track_low_thresh`: second, looser association threshold
- `new_track_thresh`: how easy it is to start a new track
- `track_buffer`: how long lost tracks are kept alive; higher means more tolerance to short occlusion
- `match_thresh`: how lenient the tracker is when matching
- `with_reid`: appearance-based matching; off by default and adds overhead

### Suggested starting profile for proxy stage

- `tracker_type: botsort`
- `track_high_thresh: 0.20`
- `track_low_thresh: 0.05`
- `new_track_thresh: 0.20`
- `track_buffer: 30`
- `match_thresh: 0.80`
- `fuse_score: True`
- `with_reid: False`

Why:
- proxy confidence is low, so high thresholds will kill track updates
- `track_buffer` should tolerate short occlusions
- `with_reid` stays off initially to keep things simpler and lighter

If confidence is even weaker in practice, try:
- `track_high_thresh: 0.10`
- `new_track_thresh: 0.10`

Do not make everything too loose immediately; otherwise you risk sticky wrong tracks.

## 8) What to expose in ROS/debug topics

Add these fields to the estimator debug/status output:

### In `leader_estimate_status`
- `proxy_track_id`
- `proxy_track_state` (`TRACKED`, `LOST`, `NEW`)
- `proxy_track_age`
- `proxy_track_frames_since_seen`
- `proxy_class`
- `proxy_conf`
- `bbox_u`
- `bbox_v`
- `sel_visible`

### In debug image
Draw:
- box
- class
- confidence
- `ID=<track_id>`
- maybe age / lost count for debugging

This is essential so you can tell whether failure is:
- no detection,
- unstable detection,
- or stable track but bad control.

## 9) How tracking interacts with current reacquire logic

Do **not** delete the structured reacquire sequence you just added.

Instead:

### If target is `TRACKED`
- camera lock + follow operate normally

### If target is temporarily missing but still inside `track_buffer`
- treat as softer loss
- tracker continuity still informs reacquire

### If target is fully lost
- existing reacquire sequence (`PAN -> TILT -> YAW -> HOLD`) still runs

So tracking should reduce useless “immediate loss,” not replace the reacquire state machine.

## 10) What not to add yet

Do **not** add all of this now:
- full Kalman filter
- full constant-velocity predictor with world-state integration
- full 4.1 `TRACK -> PREDICT -> SEARCH/HOLD -> REACQUIRE` architecture
- communication-aware leash logic
- OBB / segmentation

Those belong later.

This step is only:
**better continuity on top of detections**

## 11) Phase-based validation after integration

After tracking is added, repeat structured validation.

### Phase 1 — Selection stability
Goal:
- same `track_id` stays on the UGV for longer windows
- less random switching between proxy boxes

Success:
- stable ID most of the time while target remains visible

### Phase 2 — Camera lock only
Goal:
- with UAV XY motion disabled, camera lock should reduce image error while following the tracked target

Success:
- target stays nearer center longer
- less jitter caused by changing boxes

### Phase 3 — UAV motion only / combined
Goal:
- tracked target should survive longer under motion and light occlusion
- less early collapse to `NO_DET` / stale-hold

Success:
- longer retention before loss
- fewer meaningless oscillatory reacquire episodes

## 12) Stop condition for this step

This tracking integration is done when:
- target ID continuity is clearly better than raw per-frame selection
- proxy target stays stable longer in debug image
- camera lock gets a steadier target
- retention is visibly improved enough to justify stopping proxy work and moving to dataset generation

Do **not** keep polishing forever.

Once proxy is “good enough”:
- stop
- move to Gazebo dataset capture/autolabel
- train real detector
- later do full 4.1

## 13) Concrete Codex to-do list

1. Inspect current estimator inference path and determine whether direct `model.track(...)` integration is clean.
2. If clean, add Ultralytics tracking mode using `botsort.yaml`.
3. If not clean, implement a lightweight internal continuity layer using IoU + center proximity + timeout.
4. Keep current proxy class allowlist and switch guards.
5. Expose tracked-target debug fields (`track_id`, track state, age, frames_since_seen).
6. Feed downstream lock/follow from tracked target, not raw best box.
7. Do not add Kalman yet.
8. Re-run the structured phase tests and compare against pre-tracking behavior.
9. Stop when retention is clearly better and proxy is “good enough”.

## 14) Suggested message to Codex after latest test

Use this implementation order:
- add tracker continuity on top of current proxy detections
- prefer Ultralytics BoT-SORT if clean in the repo
- otherwise add a lightweight internal tracker with IoU + center proximity + timeout
- keep current structured reacquire logic
- do not add Kalman yet
- expose track_id / track state in status/debug topics
- validate again with the same phase protocol
- stop once proxy is good enough, then move to dataset capture

## 15) Official source notes behind this guide

Ultralytics Track mode:
- supports BoT-SORT and ByteTrack
- default tracker is BoT-SORT
- works with official or custom detect models
- uses tracker YAML configs like `botsort.yaml` and `bytetrack.yaml`
- tracking arguments include thresholds and `track_buffer`, where higher `track_buffer` increases tolerance for occlusion
- ReID is optional and off by default; it is only supported in BoT-SORT and adds overhead if enabled

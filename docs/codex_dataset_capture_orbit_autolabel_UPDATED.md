# NEXT IMPLEMENTATION SPEC (Codex): Orbit Dataset Capture + Ground-Truth Auto-Label for UGV DETECT (Sim-Only)

Context (read first)
You are working in the existing `halmstad_ws` repo with ROS 2 + Gazebo Sim (orchard world, DJI-like UAV `dji0`, Husky/UGV entity). Our trained YOLO weights do **not** detect the UGV on real Gazebo frames (confirmed by offline Ultralytics predict). This is a **domain shift** problem, not a ROS bug.

Goal of this task
Build a deterministic, reproducible tool that **generates a DETECT dataset** (images + YOLO bbox labels) from the *actual Gazebo camera stream*, using Gazebo ground truth to auto-label. This dataset will be used to train a new single-class detector `robot` that actually works in our sim.
Important: this is **offline dataset generation only**. It does NOT change the runtime constraint that in `uav_only` the UGV does not broadcast state for control.

Why orbit/hemisphere sampling
Capturing only “follow-behind” causes overfitting to one viewpoint. Orbit sampling collects front/side/rear, different distances and heights, partial occlusions, and makes the detector generalize.

Scope boundaries (do not exceed)
- Produce YOLO **DETECT** labels only (`class x_center y_center w h` normalized). No OBB here.
- Do not integrate OMNeT++ here.
- Do not modify the runtime follow/leash pipeline in this task (separate substep later).
- Only touch capture/autolabel + docs + optional preview scripts.

---

## 1) Deliverables (Definition of Done)

DoD-1: A ROS2 executable node exists: `dataset_orbit_capture.py` (package `lrs_halmstad` or equivalent).
DoD-2: Running one command produces:
- `datasets/ugv_sim_orbit/images/{train,val}/...`
- `datasets/ugv_sim_orbit/labels/{train,val}/...`
- `datasets/ugv_sim_orbit/data.yaml` ready for Ultralytics training.
DoD-3: A preview script exists to draw GT boxes on random samples and write `datasets/ugv_sim_orbit/preview/*.png`.
DoD-4: `docs/IMPLEMENTATION_LOG.md` updated with: what changed, how to run capture, how to verify, how to train.

---

## 2) Inputs / Interfaces to use

### Camera (source of images)
- `/dji0/camera0/image_raw`
- `/dji0/camera0/camera_info`
Must use the actual runtime camera topics to match distribution.

### UGV pose (ground truth)
We need UGV pose (position + orientation) by entity name. Use the simplest reliable method in this repo:

Preferred:
- Subscribe to Gazebo Pose_V (names included) and select pose where `name == ugv_entity`.
Fallback:
- If the repo already exposes a ROS topic giving UGV pose: use it.

Note:
Bridging Pose_V to PoseArray may lose names; avoid that unless you already have a named mapping.

### UAV teleport (viewpoint control)
Use the existing Gazebo service you already use for UAV repositioning:
- `/world/<world>/set_pose` or `/world/<world>/set_pose_vector`

Also support the case where the camera is a separate model/entity (you already sometimes set pose for both UAV model and its camera model).

---

## 3) What to implement

### 3.1 Node: `dataset_orbit_capture.py`

Core behavior:
1) Read camera intrinsics from `CameraInfo`.
2) Track UGV ground-truth pose by entity name (continuous).
3) Sample a set of relative viewpoints around the UGV (orbit/hemisphere).
4) For each viewpoint:
   - Teleport UAV (and camera entity if separate) to that pose.
   - Wait `settle_time_s`.
   - Capture `frames_per_pose` images.
   - Auto-label each frame by projecting a 3D UGV bounding box into the image.
   - Save image + label.

Output format:
- Images saved as PNG (or JPEG) with deterministic filenames.
- Labels saved as YOLO DETECT TXT: `0 x y w h` normalized (single class).

### 3.2 Optional helper: `split_dataset.py`
If you generate everything in one folder, split deterministically into train/val.
Alternatively, do the split online: every Nth sample goes to val.

### 3.3 Preview: `render_labels_preview.py`
Randomly choose K images, draw the GT bbox from the label file, write preview images.

---

## 4) Auto-labeling details (must be correct)

We generate axis-aligned boxes from a 3D cuboid.

### 4.1 UGV 3D cuboid definition (params)
Add ROS params:
- `ugv_box_l` (meters)
- `ugv_box_w` (meters)
- `ugv_box_h` (meters)
- `ugv_box_z_offset` (meters, center offset if needed)
This defines a cuboid in UGV local frame.

### 4.2 Projection
For each frame:
1) Compute 8 corners of cuboid in UGV frame.
2) Transform corners to world using UGV pose.
3) Transform world points into camera frame using TF (preferred) or direct math:
   - Use `tf2_ros` to lookup transform from world to camera optical frame.
4) Project 3D camera points to pixels using CameraInfo K (pinhole model).

### 4.3 Label extraction
- Take min/max x,y of projected corners.
- Clip to image bounds.
- Filter out invalid labels:
  - Require at least `min_visible_corners` (e.g. 4) with z>0.
  - Require bbox size >= `min_box_px` (e.g. 20 px).
  - Optionally require bbox area >= `min_area_px2`.

Then normalize:
- x_center = (xmin+xmax)/2 / W
- y_center = (ymin+ymax)/2 / H
- w = (xmax-xmin)/W
- h = (ymax-ymin)/H

Write label line: `0 x_center y_center w h`

---

## 5) Orbit / hemisphere sampling (viewpoint plan)

### 5.1 Params
- `r_min`, `r_max`
- `z_min`, `z_max`
- `theta_samples` (angles around 0..2π)
- `r_samples`
- `z_samples`
- `frames_per_pose`
- `settle_time_s`
- `rate_limit_hz`
- `seed` (for deterministic sampling)
- `total_samples` (optional override to sample randomly instead of grid)
- `min_box_px`, `min_visible_corners`

### 5.2 Pose generation
For each sample:
- radius r
- azimuth theta
- altitude z
Compute offset in world frame:
dx = r*cos(theta), dy = r*sin(theta), dz = z
UAV position = UGV_position + [dx, dy, dz]

Yaw should “look at” the UGV:
yaw = atan2(UGV_y - UAV_y, UGV_x - UAV_x)

Pitch optional; simplest: keep UAV level and rely on camera mounting.
If camera is separate entity and supports pan, you may set camera yaw/pan instead.

---

## 6) Data quality knobs (important)

Add options to improve dataset realism:
- `skip_if_too_clipped` + `max_clipping_fraction` (e.g. skip if >0.5 outside image)
- `skip_if_too_small` (min_box_px)
- `max_images` stop early

Log counts:
- accepted frames
- rejected frames by reason (behind camera, too small, too clipped, missing transforms)

---

## 7) Run instructions (exact)

### 7.1 Terminals
Terminal A: start Gazebo orchard + spawn UGV + UAV as usual.
Terminal B: run capture node.

### 7.2 Command (example)
`ros2 run lrs_halmstad dataset_orbit_capture --ros-args \
  -p world:=orchard \
  -p ugv_entity:=<UGV_NAME> \
  -p uav_entity:=dji0 \
  -p camera_entity:=<CAMERA_ENTITY_OR_EMPTY> \
  -p out_dir:=datasets/ugv_sim_orbit \
  -p theta_samples:=24 -p r_samples:=4 -p z_samples:=3 \
  -p r_min:=6.0 -p r_max:=30.0 -p z_min:=3.0 -p z_max:=15.0 \
  -p frames_per_pose:=2 -p settle_time_s:=0.25 -p min_box_px:=20 \
  -p ugv_box_l:=1.0 -p ugv_box_w:=0.7 -p ugv_box_h:=0.6 -p ugv_box_z_offset:=0.3 \
  -p use_sim_time:=true`

Expected:
- Images + labels created progressively
- Console shows progress + accepted/rejected counts

---

## 8) Acceptance tests

AT-1 Dataset integrity
- number of images == number of label files
- each label line has exactly 5 fields
- normalized values in [0,1]
- no empty label files for accepted images

AT-2 Visual preview
Run:
`python3 scripts/render_labels_preview.py --data datasets/ugv_sim_orbit --n 50`
Expect: preview images show the box tightly around the UGV.

AT-3 Training sanity (performed by William, but document commands)
Train:
`yolo train model=yolo26n.pt data=datasets/ugv_sim_orbit/data.yaml imgsz=640 epochs=30`
Then test:
`yolo predict model=runs/detect/train/weights/best.pt source=<fresh_gazebo_frame>.png conf=0.1`
Expect: the model draws a box on the UGV in new Gazebo frames.

---

## 9) Documentation requirements

Update `docs/IMPLEMENTATION_LOG.md` with:
- What changed (orbit dataset capture + autolabel + preview)
- Which mode(s) affected (offline dataset generation only)
- How to run capture (exact command)
- How to verify (integrity + preview)
- How to train/test model (commands)

---

## 10) Implementation notes (repo-specific)

- Reuse existing code patterns for:
  - calling `/world/<world>/set_pose` or `/set_pose_vector`
  - resolving entity names (UGV, UAV, camera)
  - event/log conventions if present
- Prefer deterministic output structure and stable filenames.
- Keep CPU usage reasonable (limit frame save rate and service call rate).

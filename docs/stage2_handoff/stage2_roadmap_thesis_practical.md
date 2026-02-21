# Practical Roadmap (Stage 2.x → Stage 3) — Thesis + Implementation Plan

This is a “doable in-order” plan that matches what you *actually* built in Stage 2 (follow + leash + harness + logging) and what you still need for the thesis (network effects, perception/YOLO, removing privileged information, scaling).

It also answers your question: **yes — it’s usually better to wait with synthetic degradation and PX4 until OMNeT++ and the comms layer are in place**, because OMNeT++ becomes the *real mechanism* that creates delay/loss/jitter. You can still do tiny pre-work that doesn’t waste time (interfaces + metrics + acceptance tests), but the “real” 2.1 happens once OMNeT++ is wired.

---

## Current state (Stage 2.0) — DONE ✅

You have a clean Stage 2 implementation that is **harness-integrated, measurable, and reproducible**:

- `scripts/run_round.sh` supports `CONDITION=follow`
- `follow_uav.py` implements **Follow + Leash** using `/world/<world>/set_pose` (SetEntityPose)
- Uses **UGV odometry as privileged state source** (now `/a201_0000/platform/odom`)
- Publishes `/${uav}/pose_cmd` and events on `/coord/events`
- Rosbag records: `/coord/events`, `/a201_0000/cmd_vel`, odom topics, tf, `/dji0/pose_cmd`
- Verified by bag counts (nonzero `/dji0/pose_cmd` + FOLLOW_TICK), and on-screen behavior (UAV follows UGV)

**Known operational gotchas:**
- If you see duplicate `/follow_uav` nodes, kill them before runs (`pkill -f "ros2 run lrs_halmstad follow_uav"`).
- If FastDDS SHM errors appear, use `export RMW_FASTRTPS_USE_SHM=0` in the terminals you use for CLI tools (echo/hz/bag play).

---

## Should you “wait” with 2.1 / 2.2 / 2.3? (Answer)

### 2.1 (degraded state source)
**Yes, wait for OMNeT++** *if the degradation you care about is caused by communication.*  
Otherwise you end up building an artificial noise/dropout injector that you will later replace with OMNeT++ anyway.

**What you can do before OMNeT++ without wasting time:**  
Define the *contract* for “state input” and log/metrics, so when OMNeT++ arrives you just swap the source.

### 2.2 (PX4 adapter)
**Yes, wait unless your thesis contribution depends on PX4 realism now.**  
PX4 integration is a high-cost integration step. It’s a Stage 3 “realism/transfer” move, not required to prove Stage 2 network/perception effects.

### 2.3 (multi-UAV coordination)
**Yes, wait until you have at least (a) comm impairment and (b) a leader-estimate source.**  
Scaling without comm/perception tends to become “more of the same” and not thesis-interesting. You want scaling *under impairment*.

---

## Proposed sequence (recommended)

This is the combined plan: it merges your earlier roadmap structure (2.1/2.2/2.3…) with what you now know you must include (OMNeT++, YOLO/perception), and keeps it thesis-friendly.

### Stage 2.1 — OMNeT++ integration as the *real* impairment mechanism (Comms layer)
**Goal:** Introduce controlled latency/loss/jitter **without touching** follow controller logic.

Implementation tasks:
- Decide which topics go through impairment (minimum viable):
  - `/coord/events` (for correctness checks under impairment)
  - the “leader state” feed (future: whatever replaces odom)
- Put event markers on a “raw” topic if needed (ex: `/coord/events_raw`) and bridge to `/coord/events` through OMNeT++/impairment.
- Keep the harness unchanged except for:
  - selecting `EVENT_TOPIC` via env var
  - recording both raw + impaired topics if required

Acceptance tests (objective):
- Bag contains both raw and impaired marker streams (if used).
- Impairment settings (delay/loss) are recorded in `meta.yaml`.
- Event ordering remains correct (no missing stage markers in the impaired stream beyond expected loss).

**Writing tasks (thesis):**
- Explain comm impairment methodology and why events are used as “in-run ground truth segmentation”.
- Define impairment parameters as experimental knobs.

### Stage 2.2 — Perception/YOLO: build a *replaceable* “leader estimate” source
**Goal:** Produce a leader pose estimate that *could* be used instead of privileged odom later.

Minimum viable path (sim-first):
- Use UAV camera topics already available (`/dji0/.../image_raw`)
- Implement a perception node that outputs an estimate topic, e.g.:
  - `leader_estimate` as `geometry_msgs/PoseStamped` (or `PoseWithCovarianceStamped`)
- In the simplest case, start with:
  - AprilTag/marker in sim (if allowed) or
  - a “perfect detector” (ground truth from Gazebo) **but with controlled noise**  
  Then replace it with YOLO when you have the pipeline stable.

Acceptance tests:
- The estimate topic exists and updates at a defined rate.
- Logged in rosbag.
- Error statistics vs odom (mean error, max error) computed offline.

**Writing tasks (thesis):**
- Justify why a two-step perception path is valid:
  - first validate pipeline + metrics in sim,
  - then upgrade estimator (YOLO) while keeping the contract identical.

### Stage 2.3 — Remove privileged information: swap controller input to estimate (+ comm impairment)
**Goal:** Make the follow/leash controller consume the estimate topic instead of `/platform/odom`.

Implementation tasks:
- Add a parameter to `follow_uav.py` already exists (`ugv_odom_topic`) — use it as the injection point:
  - now it points to your estimate topic instead of odom
- Decide what “stale” means under comm impairment:
  - keep `pose_timeout_s`, but now it’s actually meaningful
- Run experiments under impairment:
  - baseline (odom)
  - estimated + no impairment
  - estimated + impairment

Acceptance tests:
- Constraint violations: leash breaches (d > d_max) frequency.
- Tracking error: |d - d_target| distribution.
- Robustness: proportion of time in POSE_STALE_HOLD.

**Writing tasks (thesis):**
- This is the clean story: Stage 2.0 validates controller; Stage 2.3 removes privileged input and quantifies degradation under comm effects.

### Stage 2.4 — Scaling experiments (multi-UAV) under impairment (optional but strong)
**Goal:** Scale to multiple UAVs and measure how comm + estimation affects coordination.

Minimum viable:
- Run multiple follower UAVs (dji0, dji1, dji2) following the same leader at different offsets.
- Keep it simple: parameterize follower IDs and offsets (no “smart task allocation” yet).

Acceptance tests:
- Inter-UAV separation constraints (avoid collisions).
- Aggregate bandwidth / update rates vs impairment.

### Stage 3 — PX4 / real-world transfer plan (only after Stage 2 is publishable)
**Goal:** Replace the “SetEntityPose teleport adapter” with a realistic flight stack and/or hardware.

What to do first:
- Keep the controller outputs identical (setpoints).
- Swap only the adapter/backend.

---

## Table version (copy/paste friendly)

| Stage | Goal | Practical tasks (what you code) | What you measure | Thesis value |
|------|------|----------------------------------|------------------|-------------|
| 2.0 (done) | Follow + leash baseline | `follow_uav.py` + `run_round.sh` integration | `/dji0/pose_cmd`, FOLLOW_TICK, odom, cmd_vel | Establishes reproducible baseline |
| 2.1 | OMNeT++ comm impairment | Bridge/impair selected topics, log impairment params | marker integrity, delay/loss effects | Enables controlled network experiments |
| 2.2 | Perception pipeline (YOLO path) | estimate node → `leader_estimate` topic | estimation error vs odom | Adds sensing realism + basis to remove privileged info |
| 2.3 | Remove privileged odom | feed estimate into controller + impairment | leash violations, tracking error, stale holds | Core “no privileged info” contribution |
| 2.4 | Multi-UAV scaling | N followers, offsets, minimal coordination | separation + stability under impairment | Scaling story with real constraints |
| 3 | PX4 / real-world | adapter swap, safety controls | lag/overshoot vs teleport baseline | Transfer / external validity |

---

## Papers to cite / connect explicitly

From your spec document (development-to-thesis mapping):
- **AuNa** (ROS 2 + Gazebo + OMNeT++ integrated evaluation anchor)
- **Communication-based leashing** (conceptual grounding for leash constraint under comm limits)
- **Collaborative inspection / heterogeneous collaboration** (multi-robot coordination framing and role separation)

Plus the network-simulation / emulation thread you’ve already been building in your related work:
- ROS2+Gazebo+network-sim/emulation works (your Stage 1/2 background section sources)

(Use the exact BibTeX keys you already maintain in the thesis; this file is intentionally key-agnostic.)

---

## Next actions (what to do next week)

1. Merge/push Stage 2.0 code and the stage2 handoff doc (DONE/ONGOING).
2. Ruben/Comms: finish OMNeT++ wiring plan → define which topics are impaired and where the bridge sits.
3. You: add the thesis-methodology writeup for Stage 2.0 + planned 2.1–2.3 (use this roadmap as structure).
4. Once OMNeT++ is in place: implement 2.1 and run a small matrix of impairment configs.
5. Start perception pipeline with the smallest stable estimator; then upgrade to YOLO when the contract is stable.


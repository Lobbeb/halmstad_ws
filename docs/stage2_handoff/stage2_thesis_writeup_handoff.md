# Stage 2 (Follow + Leash) — Thesis Write‑Up Handoff (ROS 2 Jazzy + Gazebo Harmonic)

This document is for thesis writing (methodology + implementation + verification narrative), not for development.
It summarizes what Stage 2 is, why it exists, how it was implemented in the repo, how it was verified, and what you can safely claim in the thesis.

It is designed so you can paste sections directly into your LaTeX draft (e.g., Methodology / Experimental Setup / Scenario Definition / Implementation / Verification / Limitations).

---

## 0) Context (where Stage 2 sits in the thesis)

Stage 1 established a reproducible baseline “round” in simulation: deterministic control, event markers, and rosbag logging for a simple UGV+UAV scenario.

Stage 2 adds a more thesis‑aligned coordination behavior while keeping the same evaluation harness:

1. Keep the same run orchestration (run script, event markers, rosbag, artifacts).
2. Replace the Stage 1 UAV sweep with a Stage 2 follow + leash controller.
3. Preserve repeatability and observability so later stages can add:
   communication impairment (e.g., OMNeT++ / emulation) as conditions
   multi‑UAV scaling

Stage 2 therefore functions as the first “coordination” scenario that is still simple enough to be deterministic and measurable.

---

## 1) Stage 2 scenario (what happens in the experiment)

High‑level scenario:
A Clearpath Husky UGV drives a deterministic motion pattern in the orchard world.
A UAV (dji0) maintains a fixed standoff behind the UGV (distance target) while remaining inside a maximum “leash” radius (communication‑range surrogate).
The experiment is structured into phases and tagged using explicit event markers in /coord/events.

Core behavior definition (B1 Follow + Leash):
At a fixed tick rate, read UGV odometry (x, y, yaw).
Compute a target point behind the UGV:
  (x_t, y_t) = (x_g, y_g) − d_target [cos(ψ_g), sin(ψ_g)]
Apply leash constraint to keep UAV within a maximum radius d_max around the UGV:
  enforce ||(x_t, y_t) − (x_g, y_g)|| ≤ d_max
Command the UAV pose via Gazebo service /world/<world>/set_pose.
Publish the commanded UAV pose to /dji0/pose_cmd for logging and later metrics.
Publish events (FOLLOW_TICK, LEASH_CLAMPED, POSE_STALE_HOLD, etc.) to /coord/events.

Why this scenario is thesis‑friendly:
deterministic inputs and parameters
repeatable behavior (no stochastic path planners)
clean segmentation using event markers
metrics can be computed from bagged topics without extra instrumentation

This matches the Stage 2 specification derived from the four reference papers.

---

## 2) Parameters (what you should present in the thesis)

Stage 2 parameters are intentionally few and interpretable:

d_target (default 5.0 m): desired horizontal standoff behind UGV
d_max (default 15.0 m): maximum allowed UAV–UGV horizontal distance (“leash”)
z_alt (default 10.0 m): constant UAV altitude
tick_hz (default 5 Hz): follow controller update rate
pose_timeout_s (default 0.75 s): odometry freshness threshold; stale → hold commands
min_cmd_period_s (default 0.10 s): prevents command over‑publishing
ugv_odom_topic: /a201_0000/platform/odom (or /filtered if desired)
event_topic: /coord/events

In the thesis you can group these as:
control parameters (d_target, d_max, z_alt, tick_hz)
robustness parameters (pose_timeout_s, min_cmd_period_s)

---

## 3) Reference papers used (how each supports the thesis narrative)

Stage 2 is justified using four papers. This is not “we copied their system”; rather it is “we extracted robust, minimal patterns that match our evaluation goals”.

Paper A — AuNa (Teper et al.)
Use to motivate staged methodology and why spacing/distance error is a valid measurable objective.

Paper B — Collaborative Inspection (Amorim et al., JIRS 2025)
Use to motivate role separation and repeatable mission workflows (phases and fixed initialization).

Paper C — Communication‑based Leashing (Hauert et al., ICRA 2010)
Use to justify the leash constraint as a communication‑range surrogate.

Paper D — Heterogeneous multi‑robot collaboration (de Castro et al., Machines 2024)
Use to motivate coordination event markers and safe fallback logic under stale data.

Thesis phrasing tip:
Write this as design patterns extracted from literature (staged evaluation, phase boundaries, comm‑range maintenance, comm‑loss fallback), rather than claiming reproduction of any single paper.

---

## 4) Implementation summary (what changed in the repo)

Stage 2 was implemented as a new ROS 2 node plus minimal harness changes so Stage 1 remains intact.

### 4.1 New node: follow_uav.py (Stage 2 controller)
Location: src/lrs_halmstad/lrs_halmstad/follow_uav.py

What it does:
subscribes to UGV odometry
checks freshness; stale odom → POSE_STALE_HOLD (no new commands)
computes target behind UGV and clamps it to a circle of radius d_max around the UGV
calls /world/<world>/set_pose using ros_gz_interfaces/srv/SetEntityPose
publishes /dji0/pose_cmd (PoseStamped) to guarantee bagged observability
publishes events to /coord/events (String), including FOLLOW_NODE_START, FOLLOW_TICK, LEASH_CLAMPED

Notes you can mention in thesis:
the logged UAV trajectory for Stage 2 evaluation uses the commanded pose stream (/dji0/pose_cmd)
the controller is intentionally minimal and deterministic

### 4.2 Harness: scripts/run_round.sh (scenario orchestration)
Location: scripts/run_round.sh

What it does:
validates required topics/services via contract_check
starts rosbag record (MCAP)
publishes event markers
UAV phase:
  if CONDITION=follow: start follow_uav in background
  else: run Stage 1 grid sweep unchanged
UGV phase:
  runs deterministic cmd_vel publishing (embedded Python snippet)
stops follow_uav and rosbag cleanly
writes run artifacts (runs/<run_id>/meta.yaml, logs, bag folder)

### 4.3 Guard: contract_check.py
Location: src/lrs_halmstad/lrs_halmstad/contract_check.py

Purpose:
ensures key topics/services exist before starting a run (early failure, easier debugging)
does not require the event topic to exist yet (the harness creates it when publishing the first marker)

---

## 5) What gets logged (and why this is enough for metrics)

Stage 2 logs just what is needed for:
temporal segmentation (events)
UGV motion + state (odom, tf)
UGV commands (cmd_vel)
UAV commanded motion (pose_cmd)

Typical recorded topics:
 /clock
 /a201_0000/platform/odom and /a201_0000/platform/odom/filtered
 /a201_0000/cmd_vel and /a201_0000/platform/cmd_vel
 /a201_0000/tf and /a201_0000/tf_static
 /coord/events
 /dji0/pose_cmd

---

## 6) Verification evidence (what we actually proved in the terminal)

### 6.1 Manual sanity test (fast)
With Gazebo and UAV spawn running, starting the follower manually and driving the UGV forward for ~5 seconds produced:
FOLLOW_TICK events in /coord/events
UAV repositioning consistent with follow behavior

### 6.2 Full run proof (rosbag evidence)
Successful full‑round runs were produced and inspected (e.g., run_0003 and run_0004):
bags contain non‑zero counts for:
 /a201_0000/cmd_vel
 /coord/events
 /dji0/pose_cmd
This is sufficient to claim: Stage 2 follow + leash runs under the experiment harness and produces fully logged trajectories and event markers.

### 6.3 Replay check (optional but strong)
ros2 bag play ... --clock while echoing /coord/events shows FOLLOW_TICK during replay, supporting offline reproducibility.

---

## 7) Known runtime issues and mitigations (write as limitations / engineering notes)

Duplicate follow_uav instances
Symptom: multiple /follow_uav nodes in ros2 node list; unexpected behavior.
Mitigation:
  ps aux | grep follow_uav | grep -v grep
  pkill -f "ros2 run lrs_halmstad follow_uav"

FastDDS shared memory warnings (RTPS_TRANSPORT_SHM)
Observed warnings: Failed init_port ... open_and_lock_file failed.
Mitigation used:
  export RMW_FASTRTPS_USE_SHM=0
Important: avoid typos such as export RMW_FASTRTPS_USE_SHM=0=0.

UGV not moving confusion
Often due to phase timing or confusion between /a201_0000/cmd_vel and /a201_0000/platform/cmd_vel.
In thesis: explain there is a mux/bridge pipeline; the harness publishes /a201_0000/cmd_vel during UGV phase and logs both topics for auditability.

---

## 8) What you can claim (and what you should not claim)

Safe claims:
Stage 2 follow + leash controller is implemented and integrated into the deterministic run harness.
The controller produces a logged commanded UAV trajectory (/dji0/pose_cmd) and event markers (/coord/events) inside MCAP bags.
The scenario is deterministic given fixed parameters and fixed UGV motion primitives.

Do not claim yet:
OMNeT++ impairment is active (unless you have runs showing impairment config + measurable effects).
Perception/YOLO integration (unless a perception node exists and is logged).
True UAV feedback control; current control uses Gazebo SetEntityPose for repeatability.

---

## 9) Thesis‑oriented metrics (enabled by Stage 2)

Standoff distance error
  e(t) = d(t) − d_target
  d(t) = ||(x_u(t), y_u(t)) − (x_g(t), y_g(t))||
Use UAV commanded pose (/dji0/pose_cmd) and UGV odom.

Leash violations
Count/duration where d(t) > d_max + ε.
(Clamp is applied, so violations should be rare; ε covers sampling/async effects.)

Update rate stability
Check /dji0/pose_cmd rate (~tick_hz) and event cadence.

Phase segmentation correctness
Use /coord/events to isolate UAV/UGV phases and compute metrics per phase.

---

## 10) Paste‑ready “run description” paragraph

“In Stage 2 we replace the baseline UAV sweep with a deterministic follow‑and‑leash behavior. The UGV executes a fixed motion primitive sequence in Gazebo (orchard world), while the UAV is commanded via Gazebo’s SetEntityPose service at 5 Hz to maintain a desired standoff distance behind the UGV. A leash constraint clamps the UAV target to a maximum radius around the UGV, representing a communication‑range surrogate. Each experiment run is orchestrated by a single script that validates required ROS interfaces, records an MCAP rosbag of the relevant topics, and publishes explicit phase markers for offline segmentation. The resulting bags contain UGV odometry and command topics as well as the UAV’s commanded trajectory and event markers, enabling repeatable metric computation across runs.”

---

## 11) Roadmap alignment

Stage 2 provides the stable scenario core.
Stage 2.1 adds communication conditions (delay/loss/jitter) while preserving scenario + metrics definitions.
Stage 3 scales to multi‑UAV or more complex mission phases, reusing:
event segmentation
run harness and artifact structure
metric computation pipeline

---

## 12) Appendix: commands (reproducibility)

Terminal A (Gazebo + Husky):
  ros2 launch clearpath_gz simulation.launch.py world:=orchard

Terminal B (spawn UAVs):
  ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard

Terminal C (run one round):
Baseline:
  bash scripts/run_round.sh run_XXXX baseline dji0 orchard
Stage 2:
  bash scripts/run_round.sh run_XXXX follow dji0 orchard

Cleanup:
  ps aux | grep follow_uav | grep -v grep
  pkill -f "ros2 run lrs_halmstad follow_uav"

DDS SHM mitigation (optional):
  export RMW_FASTRTPS_USE_SHM=0

---

## 13) BibTeX reminder

Ensure BibTeX entries exist for:
Teper et al. (AuNa)
Amorim et al. (Collaborative Inspection, JIRS 2025)
Hauert et al. (ICRA 2010 leashing)
de Castro et al. (Machines 2024)

See also:
docs/stage2_handoff/stage2_leashing_follow_handoff.md
Stage2_Scenario_Control_Spec_from_4_papers_v2_repo_mapping.md

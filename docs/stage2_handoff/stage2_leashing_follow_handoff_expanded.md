# Stage 2: Leashing + Follow — Full Implementation Handoff

## Project Context

- **Workspace root:** `~/halmstad_ws`
- **ROS 2 Distribution:** Jazzy
- **Simulation:** Gazebo Harmonic
- **Robots:** Clearpath Husky (UGV), DJI-like UAV(s)
- **World:** orchard
- **User:** William (Sweden)
- **Date:** February 2026

---

## 1. Scientific Foundation & Reference Papers

Stage 2 is built on four key papers, each contributing a core concept:

### 1.1 AuNa (Teper et al.) — Staged Co-Simulation & Distance Error

- **Concept:** Staged methodology, desired spacing, deterministic evaluation.
- **Takeaway:** Compute distance error $e = d - d_r$ (actual vs desired), use fixed update rates, treat network effects as experimental conditions.

### 1.2 Collaborative Inspection (Amorim et al., JIRS 2025)

- **Concept:** Mission workflow, repeatability, phase boundaries.
- **Takeaway:** Separate UGV/UAV roles, fixed initial conditions, explicit phase segmentation (START/END events).

### 1.3 Communication-based Leashing (Hauert et al., ICRA 2010)

- **Concept:** Communication range as leash constraint.
- **Takeaway:** Enforce $distance(UAV, UGV) \leq d_{max}$, target $d_{target}$ for stability, fallback behavior when out of range.

### 1.4 Heterogeneous Multi-Robot Collaboration (de Castro et al., Machines 2024)

- **Concept:** Coordination triggers, comm-loss routines.
- **Takeaway:** Coordination vocabulary (REQUEST, ACK, EXECUTE, DONE, COMM_LOST, RESUME), deterministic triggers, safe fallback under comm loss.

---

## 2. Scenario Definition

### 2.1 Stage 2 Goal

- **Replace Stage 1 grid sweep with follow/leash controller.**
- **UAV maintains fixed standoff behind UGV, obeys leash constraint.**
- **UGV executes deterministic path (square or lawnmower).**
- **All phases and events are logged for offline analysis.**

### 2.2 Parameters

- `d_target`: Desired standoff distance (default: 5.0 m)
- `d_max`: Maximum allowed distance (default: 15.0 m)
- `z_alt`: UAV altitude (default: 10.0 m)
- `tick_hz`: Control loop rate (default: 5 Hz)
- `world`: Simulation world (default: orchard)
- `uav_name`: UAV model name (default: dji0)
- `ugv_odom_topic`: UGV odometry topic (default: /a201_0000/platform/odom/filtered)
- `publish_events`: Whether to publish event markers (default: true)

---

## 3. Implementation Decisions

### 3.1 Node Separation

- **Stage 2 controller implemented as a new node:** `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
- **Stage 1 sweep retained in:** `scripts/uav_setpose_sweep.py`
- **Run harness:** `scripts/run_round.sh` orchestrates both stages.

### 3.2 Deterministic Control

- **UAV pose is tracked as "last commanded pose"** (not actual feedback).
- **UGV odometry is read from `/a201_0000/platform/odom/filtered`.**
- **Leash constraint is enforced every tick.**
- **Events are published to `/coord/events`.**

### 3.3 Logging & Artifacts

- **Rosbag records all relevant topics:** `/clock`, `/odom`, `/cmd_vel`, `/tf`, `/dji0/pose_cmd`, `/coord/events`.
- **Metadata written to:** `runs/<RUN_ID>/meta.yaml`
- **UAV setpoints logged to CSV:** `runs/<RUN_ID>/uav_setpoints.csv`
- **Follow controller logs:** `runs/<RUN_ID>/follow_uav.log`

---

## 4. File Mapping & Entrypoints

| File                                                 | Purpose                                                                     |
| ---------------------------------------------------- | --------------------------------------------------------------------------- |
| `scripts/run_round.sh`                               | Orchestrates experiment, records bag, publishes events, runs UAV/UGV phases |
| `src/lrs_halmstad/lrs_halmstad/follow_uav.py`        | Stage 2 follow/leash controller node                                        |
| `scripts/uav_setpose_sweep.py`                       | Stage 1 grid sweep tool                                                     |
| `src/lrs_halmstad/resource/experiment_defaults.yaml` | Experiment parameters (UAV/UGV, grid, lawnmower)                            |
| `config/rosbag_qos.yaml`                             | QoS settings for rosbag topics                                              |
| `runs/<RUN_ID>/`                                     | Output directory for each run (bag, logs, meta, CSV)                        |

---

## 5. Detailed Behavior (Follow + Leash)

### 5.1 Control Loop (Pseudocode)

```
# tick loop at tick_hz
odom = latest_odom()
xg, yg = odom.pose.pose.position.x, odom.pose.pose.position.y
yaw_g = yaw_from_quaternion(odom.pose.pose.orientation)

# target behind UGV
xt = xg - d_target * cos(yaw_g)
yt = yg - d_target * sin(yaw_g)

# leash constraint
d = hypot(last_uav_x - xg, last_uav_y - yg)
if d > d_max:
    xt = xg - (d_max/2.0) * cos(yaw_g)
    yt = yg - (d_max/2.0) * sin(yaw_g)

set_pose(entity=uav_name, x=xt, y=yt, z=z_alt, yaw=yaw_g)
publish_pose_burst('/dji0/pose_cmd', xt, yt, z_alt, yaw_g, count=5)
publish_event('/coord/events', 'FOLLOW_TICK')
```

### 5.2 Node Implementation (`follow_uav.py`)

- Subscribes to UGV odometry.
- Computes target pose behind UGV.
- Enforces leash constraint.
- Calls Gazebo SetEntityPose for UAV and camera model.
- Publishes PoseStamped to `/dji0/pose_cmd`.
- Publishes events to `/coord/events`.
- Runs at fixed tick rate.
- Terminates cleanly on shutdown.

### 5.3 Run Harness (`run_round.sh`)

- Sources environment.
- Checks contract (topics/services).
- Starts rosbag recording.
- Publishes event markers.
- **UAV phase:**
  - If `CONDITION=follow`, launches `follow_uav` node.
  - Else, runs grid sweep.
- **UGV phase:**
  - Runs deterministic motion (square/lawnmower).
- Cleans up all processes.

---

## 6. Experiment Structure

### 6.1 Run Sequence

1. **Terminal A:** Launches world + UGV
   - `ros2 launch clearpath_gz simulation.launch.py ...`
2. **Terminal B:** Spawns UAV(s)
   - `ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard`
3. **Terminal C:** Runs experiment
   - `bash scripts/run_round.sh <run_id> follow dji0 orchard`

### 6.2 Artifacts

- `runs/<RUN_ID>/bag/` — MCAP rosbag
- `runs/<RUN_ID>/meta.yaml` — run metadata
- `runs/<RUN_ID>/follow_uav.log` — controller log
- `runs/<RUN_ID>/uav_setpoints.csv` — setpoints (if sweep)

---

## 7. Acceptance Criteria

- **Rosbag contains `/dji0/pose_cmd` messages during UAV phase.**
- **Mean standoff error $|d - d_{target}|$ is finite and stable.**
- **Leash constraint respected: $max(distance(UAV, UGV)) \leq d_{max} + \epsilon$.**
- **Round is deterministic: repeated runs produce identical commanded sequence.**
- **All phases and events are logged for offline analysis.**

---

## 8. Known Issues & Fixes

- **UGV spinning:** Caused by extra cmd_vel publishers; must identify and disable.
- **Python dependencies:** PyYAML and transforms3d required; install in correct environment.
- **Twist mux QoS:** RELIABLE publisher vs BEST_EFFORT subscriber; risk for command loss.
- **YAML edits:** Generator rewrites config files; edit generator inputs or launch setup.
- **Terminal environment:** Do not run ROS launches inside OMNeT++ venv.

---

## 9. Next Steps (Stage 2.1)

- **Fix UGV spin:** Identify and stop publisher producing angular.z = 0.5.
- **Verify lawnmower pattern:** Confirm UGV executes lanes, not just spins.
- **Run controlled experiments:** Use fixed parameters, log all outputs.
- **Write up methodology:** Map scenario to thesis structure, reference papers.
- **Attach BibTeX for papers:** Communication-based Leashing (ICRA 2010), Heterogeneous Multi-Robot Collaboration (Machines 2024).

---

## 10. Repo Mapping (for Copilot or new chat)

| Scenario              | File(s)                                | Entrypoint                                                 |
| --------------------- | -------------------------------------- | ---------------------------------------------------------- |
| Stage 2 Follow/Leash  | `follow_uav.py`, `run_round.sh`        | `bash scripts/run_round.sh <run_id> follow dji0 orchard`   |
| Stage 1 Grid Sweep    | `uav_setpose_sweep.py`, `run_round.sh` | `bash scripts/run_round.sh <run_id> baseline dji0 orchard` |
| UGV Motion            | Embedded Python in `run_round.sh`      | UGV phase                                                  |
| Event Markers         | `run_round.sh`                         | publish_event()                                            |
| Experiment Parameters | `experiment_defaults.yaml`             | All phases                                                 |

---

## 11. Coordination Vocabulary (for future extension)

- **Events:** `FOLLOW_TICK`, `LEASH_TRIGGERED`, `ROUND_START`, `UAV_PHASE_START/END`, `UGV_PHASE_START/END`
- **Coordination:** `REQUEST`, `ACK`, `EXECUTE`, `DONE`, `COMM_LOST`, `RESUME` (future)

---

## 12. Summary

This Stage 2 implementation is a deterministic, reproducible follow/leash controller for UAV/UGV in ROS 2 + Gazebo, built on best practices from four reference papers. All code, parameters, and experiment structure are mapped for easy handoff and further development. Use this markdown as the definitive guide for future chats, collaborators, or thesis documentation.

---

**END OF STAGE 2 HANDOFF**

---

## 13. What We Actually Changed (Stage‑2 Change Log)

This section captures the concrete edits done during Stage 2 so a new developer can reproduce the exact working state.

### 13.1 New code added

- `src/lrs_halmstad/lrs_halmstad/follow_uav.py`
  - Implements the follow + leash controller (tick loop, SetEntityPose client, events, /<uav>/pose_cmd publisher).

### 13.2 Existing code modified

- `scripts/run_round.sh`
  - Added:
    - `contract_check` sanity guard before starting the run.
    - `CONDITION=follow` branch that starts `follow_uav` in background and keeps it running during UGV phase.
    - Standardized event topic handling via `EVENT_TOPIC` env var.
    - Extended rosbag topic list to include both `/a201_0000/cmd_vel` and `/a201_0000/platform/cmd_vel` (useful when the stack routes through `twist_mux`).
    - Run artifacts: `meta.yaml`, `follow_uav.log`, `uav_setpoints.csv` (only in sweep mode).
  - Critical param fix:
    - `ugv_odom_topic` passed to follower was switched to `/a201_0000/platform/odom` (not `/filtered`) because `/filtered` intermittently caused `POSE_STALE_HOLD` due to freshness/QoS/visibility issues on some runs.

- `src/lrs_halmstad/lrs_halmstad/contract_check.py`
  - Checks the presence of required topics/services and fails early if something is missing.
  - Important nuance:
    - It does **not** require the event topic to exist yet (because the harness creates it when it first publishes markers).
    - It **does** require `/world/<world>/set_pose`.

- `src/lrs_halmstad/setup.py`
  - Updated to include new module(s) so `ros2 run lrs_halmstad follow_uav` works after build/install.

- `requirements.txt`
  - Updated to a more realistic list of runtime + dev deps.
  - Note: ROS Python dependencies (rclpy etc.) are provided by ROS install, not pip.

### 13.3 Extra files committed (Clearpath SRDF)

- `src/lrs_halmstad/clearpath/robot.srdf`
- `src/lrs_halmstad/clearpath/robot.srdf.xacro`

These were untracked and not present in the repo history at the time of Stage‑2 work. They were added so clean clones reproduce the same Clearpath robot description behavior used during simulation.

### 13.4 Docs added

- `docs/stage2_handoff/stage2_leashing_follow_handoff.md`
  - This handoff doc (expanded here) was committed so it lives with the code.

---

## 14. Runbook (Clean, Reproducible Terminal Procedure)

This is the terminal procedure that was verified working on the desktop environment.

### 14.1 Clean start (recommended)

Before launching anything, open a fresh shell and disable FastDDS shared memory (prevents recurring “open_and_lock_file” failures):

```bash
export RMW_FASTRTPS_USE_SHM=0
```

Do this in **every** terminal you use for ROS 2 commands (A/B/C/D), or put it in your shell profile.

Optional: reset discovery daemon when ROS graph gets “weird”:

```bash
ros2 daemon stop
ros2 daemon start
```

### 14.2 Terminals and commands

Terminal A (Gazebo + UGV):

```bash
export RMW_FASTRTPS_USE_SHM=0
ros2 launch clearpath_gz simulation.launch.py world:=orchard
```

Expected:

- Gazebo opens orchard world.
- Husky spawns.
- Husky should be **still** at start (unless a stray cmd_vel publisher exists).

Terminal B (spawn UAVs):

```bash
export RMW_FASTRTPS_USE_SHM=0
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard
```

Expected:

- Spawns `dji0–dji2` (depending on launch file defaults).
- We control `dji0` only.

Terminal C (optional live monitoring):

Event stream:

```bash
export RMW_FASTRTPS_USE_SHM=0
ros2 topic echo /coord/events
```

Terminal D (the full Stage‑2 run):

```bash
export RMW_FASTRTPS_USE_SHM=0
cd ~/halmstad_ws
bash scripts/run_round.sh run_0004 follow dji0 orchard
```

Expected:

- `contract_check` prints “Contract OK”.
- `/coord/events` shows: `ROUND_START`, `UAV_PHASE_START`, follower events (`FOLLOW_NODE_START`, `FOLLOW_TICK`), then `UGV_PHASE_START/END`, `ROUND_END`.
- UGV performs the deterministic forward/turn cycles.
- UAV follows behind UGV while UGV moves.

---

## 15. Sanity Test Mode (Follower Only, No Harness)

When debugging, run the follower manually first. This isolates SetEntityPose and odometry freshness without involving rosbag or harness logic.

Terminal A + B as above, then:

Terminal C (follower):

```bash
export RMW_FASTRTPS_USE_SHM=0
ros2 run lrs_halmstad follow_uav --ros-args \
  -p use_sim_time:=true \
  -p world:=orchard \
  -p uav_name:=dji0 \
  -p ugv_odom_topic:=/a201_0000/platform/odom \
  -p event_topic:=/coord/events
```

Terminal D (move UGV for 5s):

```bash
export RMW_FASTRTPS_USE_SHM=0
ros2 topic pub -r 10 /a201_0000/cmd_vel geometry_msgs/msg/TwistStamped \
"{twist: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Expected:

- `/coord/events` stops showing `POSE_STALE_HOLD` and starts showing `FOLLOW_TICK`.
- `dji0` visibly repositions and continues to update position (follows behind UGV).

---

## 16. Why “UGV Spins On Start” Sometimes Happens

Observed symptom:

- Starting only Terminal A causes UGV to rotate/spin immediately.

Most likely cause:

- A stale/leftover publisher is still publishing to `/a201_0000/cmd_vel` or `/a201_0000/platform/cmd_vel`.
- This can come from:
  - A previously launched `run_round.sh` that didn’t fully terminate its background publisher process.
  - A manual `ros2 topic pub -r ...` still running in another terminal.
  - Duplicate `follow_uav` nodes are harmless for UGV (they only move UAV), but leftover cmd_vel pubs will move the UGV.

How to confirm:

- Check publishers:

```bash
ros2 topic info /a201_0000/cmd_vel -v
ros2 topic info /a201_0000/platform/cmd_vel -v
```

If you see unexpected publishers beyond what you expect for the simulation stack, you have a stray process.

---

## 17. Hard Cleanup (Kill Leftovers Between Runs)

Use this when you see:

- Duplicate `/follow_uav` nodes.
- UGV moves when it should be still.
- `run_round` appears to “finish” but something keeps publishing.

### 17.1 Kill follower nodes

```bash
pkill -f "ros2 run lrs_halmstad follow_uav" || true
pkill -f "lrs_halmstad/follow_uav" || true
```

Verify:

```bash
ros2 node list | grep follow_uav || true
```

If duplicates remain, kill by PID:

```bash
ps aux | grep follow_uav | grep -v grep
kill <PID1> <PID2>
```

### 17.2 Kill any command publishers

Look for any `ros2 topic pub` processes:

```bash
ps aux | grep "ros2 topic pub" | grep -v grep
```

Kill them:

```bash
pkill -f "ros2 topic pub" || true
```

### 17.3 Kill rosbag record/play leftovers

```bash
pkill -f "ros2 bag record" || true
pkill -f "ros2 bag play" || true
```

### 17.4 Reset ROS graph (optional but often helps)

```bash
ros2 daemon stop
ros2 daemon start
```

---

## 18. The /cmd_vel Topic Mapping (Important Detail)

The Clearpath + Gazebo stack routes velocity commands through multiple topics/nodes.

Observed on the working system:

- `/a201_0000/cmd_vel`
  - Publisher: `cmd_vel_bridge` (namespace `/a201_0000`)
  - Subscriber: `twist_mux` (namespace `/a201_0000`)
- `/a201_0000/platform/cmd_vel`
  - Publisher: `twist_mux`
  - Subscribers: `cmd_vel_bridge`, `platform_velocity_controller`

Practical implication:

- Publishing to `/a201_0000/cmd_vel` should drive the UGV, but depending on mux configuration and QoS, you may see the actual actuator-facing commands on `/a201_0000/platform/cmd_vel`.
- That is why the rosbag topic list includes **both** topics: it makes debugging deterministic motion much easier.

---

## 19. Evidence That Stage 2 Worked (What We Verified)

The “proof” for Stage 2 was:

1. Full run produced non-zero commanded UAV poses:

- `ros2 bag info runs/run_0003/bag` showed `/dji0/pose_cmd` count > 0 (example: 364 messages).
- `ros2 bag info runs/run_0004/bag` showed `/dji0/pose_cmd` count > 0 (example: 373 messages).

2. Event markers present:

- `ros2 bag info ...` showed `/coord/events` count > 0 (example: 376 / 385).

3. UGV command stream present:

- `ros2 bag info ...` showed `/a201_0000/cmd_vel` count > 0 (example: 535 / 523).

4. Live behavior:

- UAV repositions and follows behind a moving UGV.
- `/coord/events` shows `FOLLOW_TICK` continuously during operation.

This meets the Stage‑2 acceptance criteria for “follow/leash integrated into the run harness + logged and reproducible”.

---

## 20. Common Failure Modes and How to Fix

### 20.1 `POSE_STALE_HOLD` keeps repeating

Cause:

- follower is not seeing fresh odometry (topic wrong, QoS mismatch, sim time mismatch, or odom not updating).

Fix sequence:

1. Verify odom exists and updates:

```bash
ros2 topic hz /a201_0000/platform/odom
```

2. Verify follower is subscribed to the same topic you’re checking.
3. Ensure follower has `-p use_sim_time:=true`.
4. Prefer raw odom:

Use `/a201_0000/platform/odom` instead of `/filtered` when debugging.

### 20.2 RTPS shared-memory errors

Symptom:

- Repeated messages like:
  `RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed`

Fix:

- Disable SHM in every terminal:

```bash
export RMW_FASTRTPS_USE_SHM=0
```

If you typed `export RMW_FASTRTPS_USE_SHM=0=0` by mistake, correct it (that sets an invalid env var).

### 20.3 Duplicate node name warning for `/follow_uav`

Symptom:

- `ros2 node list` warns that multiple nodes share exact name `/follow_uav`.

Cause:

- Old follower still running, or you launched follower manually + from run harness.

Fix:

- Kill by PID as shown in Section 17.1.

---

## 21. Git Push Workflow (What Worked)

Remote had newer commits, so pushing required integrating remote changes first.

Working flow:

```bash
cd ~/halmstad_ws
git status
git pull --rebase origin main
git push origin main
```

If GitHub rejects password:

- Use a GitHub Personal Access Token (PAT) instead of password for HTTPS auth.

---

## 22. Minimal “Stage‑2 is Done” Checklist

Run this after a clean run (e.g., `run_0004`):

1. Bag exists and has metadata:

```bash
ls -la ~/halmstad_ws/runs/run_0004/bag
```

Expect:

- `bag_0.mcap`
- `metadata.yaml`

2. Bag contains key topics:

```bash
ros2 bag info ~/halmstad_ws/runs/run_0004/bag | grep -E "/dji0/pose_cmd|/coord/events|/a201_0000/cmd_vel"
```

Expect:

- Non-zero counts for all three.

3. Optional: follower log exists:

```bash
tail -n 80 ~/halmstad_ws/runs/run_0004/follow_uav.log
```

Expect:

- “Started … service … /world/orchard/set_pose …”
- No repeated failures.

---

## 23. Notes for Stage 3 / OMNeT++ Integration (Prep Only)

Stage 2 was designed to be “impairment-ready” via the event topic mechanism.

- Harness publishes to `EVENT_TOPIC` (default `/coord/events`).
- If you later publish to a raw channel (e.g., `/coord/events_raw`) and bridge it through an impairment layer, keep:
  - Publish: `/coord/events_raw`
  - Record: both `/coord/events_raw` and the impaired output `/coord/events`

This allows Stage 2 logic to remain unchanged while comm impairment is swapped in experimentally.

---

2.2 (Estimator interface)
Add a new ROS 2 node (e.g. leader_estimator.py) that subscribes to /a201_0000/platform/odom and publishes /coord/leader_estimate as geometry_msgs/PoseStamped (x, y, yaw). Add simple stale handling (if odom age > X s, stop updating and publish a status/event like ESTIMATE_STALE). Update run_round.sh to record /coord/leader_estimate (and optional /coord/estimator_status). Run one follow trial and confirm ros2 bag info shows nonzero /coord/leader_estimate.

2.3 (Controller consumes estimate)
Update follow_uav.py so the leader input topic is configurable (parameter), letting it use either /a201_0000/platform/odom (baseline) or /coord/leader_estimate (new). For estimate mode, subscribe to PoseStamped instead of Odometry, but keep the same follow+leash logic. Run 5–10 trials using estimate input and reuse the same metrics extraction + plots to compare against the Stage 2.0 baseline.

**APPENDIX END**

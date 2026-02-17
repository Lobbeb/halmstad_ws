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

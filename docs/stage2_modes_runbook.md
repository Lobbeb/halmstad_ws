# Stage 2 Runbook: Teleport and Physics Modes

This document is intentionally strict: each claim is tagged as either:

- `Implemented + Verified in code`
- `Runtime validated`
- `Planned / future work`

## Status Summary

1. UAV spawn mode switch (`uav_mode`)  
   Status: `Implemented + Verified`

2. Helper wrapper (`run_spawn_uavs.sh` with `UAV_MODE`)  
   Status: `Implemented + Verified`

3. `run_round.sh` topic override env vars  
   Status: `Implemented + Verified`

4. Stage 2 behavior (ODOM + POSE runs producing expected topics)  
   Status: `Runtime validated` (runs: `run_0111` ODOM, `run_0113` POSE)

5. Physics-mode actuation backend (non-teleport control)  
   Status: `Planned / future work`

## A) UAV Mode Implementation (Verified)

### `spawn_uavs.launch.py`

- Accepts launch arg `uav_mode` with default `teleport`.
- Launch file does not enforce choices; practical/implemented values are `teleport` and `physics`.
- Routes behavior:
  - `teleport`: includes `spawn_gimbal.launch.py` for separate camera models.
  - `physics`: switches UAV generation to attached-camera dynamic path and enables camera bridge for attached-camera topics.

### `spawn_robot.launch.py` + `generate_sdf.py`

- `spawn_robot.launch.py` passes `uav_mode`.
- `with_camera` is derived from mode:
  - `physics` -> `with_camera=true`
  - `teleport` -> `with_camera=false`
- `generate_sdf.py` path selection:
  - `with_camera=false` (teleport path) -> `lrs_robot.xacro`
  - `with_camera=true` (physics path) -> `lrs_model.xacro`

### Teleport no-fall guarantee

Status: `Implemented + Verified`

- `lrs_robot.xacro` uses `<static>true</static>`.
- This is the teleport path selected when `uav_mode=teleport`.

### Physics dynamics + attached camera

Status: `Implemented + Verified`

- `lrs_model.xacro` uses `<static>false</static>`.
- Camera is attached in-model when `with_camera=true`.

### UAV entity naming

Status: `Implemented + Verified`

- Spawned UAV model names remain `dji0`, `dji1`, `dji2` in both modes.
- `set_pose` should target these names directly.

## B) Helper Script (Verified)

File exists: `run_spawn_uavs.sh`  
Behavior:

- Reads `UAV_MODE` env var (default `teleport`)
- Runs:
  - `ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard uav_mode:="$UAV_MODE"`

Verification command:

```bash
sed -n '1,80p' run_spawn_uavs.sh
```

Usage:

```bash
UAV_MODE=teleport ./run_spawn_uavs.sh
UAV_MODE=physics ./run_spawn_uavs.sh
```

## C) `run_round.sh` Override Env Vars (Verified Names)

In `scripts/run_round.sh`:

- `LEADER_MODE` (default `odom`)
- `UGV_ODOM_TOPIC` (default `/a201_0000/platform/odom`)
- `UGV_CMD_VEL_TOPIC` (default `/a201_0000/cmd_vel`)
- `UGV_CMD_TOPICS` (default `/a201_0000/cmd_vel,/a201_0000/platform/cmd_vel`)
- `REQUIRE_FLOW` (default `1`)

In `contract_check.py`:

- `REQUIRE_FLOW`
- `UGV_CMD_TOPICS`
- `REQUIRED_FLOW_TOPICS` (exported by `run_round.sh` from `UGV_ODOM_TOPIC`)

Verification commands:

```bash
rg -n "UGV_ODOM_TOPIC|UGV_CMD_VEL_TOPIC|UGV_CMD_TOPICS|REQUIRE_FLOW|REQUIRED_FLOW_TOPICS" scripts/run_round.sh src/lrs_halmstad/lrs_halmstad/contract_check.py
```

Example override:

```bash
UGV_ODOM_TOPIC=<actual_odom_topic> \
UGV_CMD_VEL_TOPIC=<actual_cmd_topic> \
UGV_CMD_TOPICS="<cmd_topic_1>,<cmd_topic_2>" \
bash scripts/run_round.sh run_XXXX follow dji0 orchard
```

## Required Terminal Setup

Always in every terminal:

```bash
cd ~/halmstad_ws
source scripts/env.sh
echo "$ROS_DOMAIN_ID $RMW_IMPLEMENTATION"
```

Expected:

- `3 rmw_fastrtps_cpp`

## Standard A/B/C Startup

### Terminal A

```bash
cd ~/halmstad_ws
source scripts/env.sh
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/halmstad_ws/src/lrs_halmstad/clearpath rviz:=false world:=orchard use_sim_time:=true
```

### Terminal B (Teleport mode, default)

```bash
cd ~/halmstad_ws
source scripts/env.sh
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard
```

### Terminal B (Physics mode)

```bash
cd ~/halmstad_ws
source scripts/env.sh
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard uav_mode:=physics
```

### Terminal C (ODOM mode)

```bash
cd ~/halmstad_ws
source scripts/env.sh
export LEADER_MODE=odom
bash scripts/run_round.sh run_XXXX follow dji0 orchard
```

### Terminal C (POSE mode)

```bash
cd ~/halmstad_ws
source scripts/env.sh
export LEADER_MODE=pose
bash scripts/run_round.sh run_XXXX follow dji0 orchard
```

## Uav_mode Verification Checklist

1. Teleport path:

```bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard
```

Expected:

- UAVs should not fall (static path).

2. Physics path:

```bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard uav_mode:=physics
```

Expected:

- UAVs come from attached-camera path (`with_camera=true` path).

3. Entity naming + pose service:

```bash
ros2 service call /world/orchard/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity:{name:'dji0',type:2},pose:{position:{x:0,y:0,z:10},orientation:{x:0,y:0,z:0,w:1}}}"
```

Expected:

- `success: true`
- `dji0` moves.

Code verification commands:

```bash
rg -n "uav_mode|teleport|physics|spawn_gimbal|camera_bridge" src/lrs_halmstad/launch/spawn_uavs.launch.py
rg -n "with_camera_for_mode|uav_mode" src/lrs_halmstad/launch/spawn_robot.launch.py
rg -n "robot_gimbal|lrs_model.xacro|lrs_robot.xacro|with_camera" src/lrs_halmstad/lrs_halmstad/generate_sdf.py
```

## Health Checks

```bash
ros2 topic hz /a201_0000/platform/odom
ros2 topic info /a201_0000/cmd_vel -v
```

Expected:

- Odom frequency > 0.
- `/a201_0000/cmd_vel` has subscribers when control stack is active.

## Cleanup Guarantees (run_round.sh)

Status: `Implemented + Verified`

- Unique node names per run:
  - `follow_uav_<run_id>`
  - `leader_estimator_<run_id>`
- On exit/interruption, stale cmd_vel publishers are killed:
  - `ros2 topic pub ... /a201_0000/cmd_vel`
  - `ros2 topic pub ... /a201_0000/platform/cmd_vel`

## Key Files

- `scripts/env.sh`
- `scripts/run_round.sh`
- `run_spawn_uavs.sh`
- `src/lrs_halmstad/launch/spawn_uavs.launch.py`
- `src/lrs_halmstad/launch/spawn_robot.launch.py`
- `src/lrs_halmstad/lrs_halmstad/generate_sdf.py`
- `src/lrs_halmstad/lrs_halmstad/contract_check.py`

## Label Definitions

- `Implemented + Verified`: confirmed directly in repository code and can be checked with the listed command.
- `Implemented but not yet verified`: code exists, but runtime behavior should be validated in your current environment.
- `Planned`: not implemented yet.

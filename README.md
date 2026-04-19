# Halmstad ROS 2 + Gazebo Testbed — Workspace Snapshot
Current source of truth
-----------------------
- `CURRENT_STATE.md`
- `RUNNING_SIM.md`

Current tested baseline:
1. `./run.sh gazebo_sim warehouse`
2. `./run.sh spawn_uav warehouse uav_name:=dji0`
3. `./run.sh localization warehouse`
4. `./run.sh nav2`
5. `./run.sh 1to1_follow warehouse`

Current Baylands follow baseline:
1. `./run.sh tmux_1to1 baylands`
2. For a grouped route: `./run.sh tmux_1to1 baylands waypoint:=parkinglot_east_0 mode:=follow ugv_goal_sequence_file:=/home/ruben/halmstad_ws/src/lrs_halmstad/config/baylands_waypoints/baylands_waypoints_parkinglot_east.yaml`

Current real follow launch:
- `src/lrs_halmstad/launch/run_follow.launch.py`
- `run_follow_motion.launch.py` and `run_1to1_follow.launch.py` are compatibility shims only

Wrappers now live under `scripts/`. Use `./run.sh <name>` and `./stop.sh <name>`
from the workspace root for the supported helper flow.

For bash completion of `./run.sh` and `./stop.sh`, add this to your own
`~/.bashrc`:

```bash
source "$HOME/halmstad_ws/scripts/bashrc_halmstad_ws.bash"
```

Recommended tmux workflow:
- start the full stack with `./run.sh tmux_1to1 warehouse`
- current default tmux layout is panes:
  - row 1: `gazebo | spawn`
  - row 2: `localization | nav2`
  - row 3: `follow`
- current default delays:
  - `gui:=false` -> `spawn=9`, `localization/nav2=11`, `follow=15`
  - `gui:=true` -> `spawn=7`, `localization/nav2=9`, `follow=13`
- stop the tmux-managed stack with `./stop.sh tmux_1to1 warehouse`

Current important notes:
- the 1-to-1 odom-follow path uses `/<ugv>/amcl_pose_odom` by default, but Baylands now overrides this to `/<ugv>/ground_truth/odom`
- attached/integrated gimbal camera is the only simulation camera path
- attached-camera teleport spawns now use a non-static UAV model with a kinematic base link so the gimbal joints visibly actuate while the UAV body still follows the simulator `set_pose` path
- Gazebo sim time is guarded by `clock_guard`, and `/clock` should have exactly one publisher
- `./run.sh 1to1_yolo ...` is the quiet/default YOLO wrapper; bare `ros2 launch ... run_follow.launch.py ...` does not automatically inherit those quiet overrides
- gimbal override is active (`gimbal_override_hold_s: 10.0`); use `follow_control random --gimbal` to sweep pan/tilt during dataset collection without the camera snapping back

The rest of this README contains older reference material and may be stale compared with the two files above.

Running experiments (current runbook)
-------------------------------------
This section documents the commands currently used to run the Halmstad Gazebo + ROS 2 simulation stack, spawn UAVs, start movement/follow logic, and start the OMNeT TCP bridge.

Terminal setup (run in every terminal or put in your .bashrc)
--------------------------------------
```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/lrs_halmstad/clearpath/setup.bash
```

Build (after code changes)
--------------------------
```bash
colcon build --symlink-install
source install/setup.bash
```

Recommended launch order (full simulation)
------------------------------------------
1. Start Gazebo + Clearpath UGV (GUI):
```bash
ros2 launch clearpath_gz simulation.launch.py world:=warehouse use_sim_time:=true gui:=true
```

2. Spawn UAV(s):
- Single UAV (`dji0`) with integrated camera/gimbal:
```bash
ros2 launch lrs_halmstad spawn1m100gimbal.launch.py
```
- Multiple UAVs (`dji0`, `dji1`, `dji2`) with integrated cameras:
```bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=warehouse uav_mode:=teleport
```

3. Start motion / follow logic:
- Follow stack (UGV motion + UAV follow + optional YOLO leader estimate):
```bash
ros2 launch lrs_halmstad run_follow.launch.py
```

4. Start OMNeT pose bridge (TCP pose snapshots from ROS odom topics):
```bash
ros2 run lrs_halmstad gazebo_pose_tcp_bridge
```

Gazebo + UGV simulation (GUI)
-----------------------------
Direct launch command:
```bash
ros2 launch clearpath_gz simulation.launch.py world:=warehouse use_sim_time:=true gui:=true
```

Common arguments (when calling `clearpath_gz` directly):
- `world:=warehouse` (world name)
- `use_sim_time:=true|false`
- `gui:=true|false`

Spawn UAVs
----------
Direct multi-UAV launch:
```bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=warehouse uav_mode:=teleport
```

`spawn_uavs.launch.py` arguments:
- `world:=orchard|construction|office|pipeline|solar_farm|warehouse`
- `uav_mode:=teleport|physics`

Single UAV + camera/gimbal launch (integrated model)
----------------------------------------------------
```bash
ros2 launch lrs_halmstad spawn1m100gimbal.launch.py world:=warehouse name:=dji0
```

`spawn1m100gimbal.launch.py` arguments:
- `world:=<gazebo_world>` (default `warehouse`)
- `name:=<uav_name>` (default `dji0`)

Low-level UAV spawn (`spawn_robot.launch.py`)
---------------------------------------------
Use this when you want one UAV with full control over camera attachment and spawn pose.

Example (teleport UAV with attached camera and camera bridge; the spawned model is non-static and uses a kinematic base link):
```bash
ros2 launch lrs_halmstad spawn_robot.launch.py \
  world:=warehouse name:=dji0 type:=m100 \
  uav_mode:=teleport with_camera:=true bridge_camera:=true camera_name:=camera0 \
  x:=0.0 y:=0.0 z:=9.0 R:=0.0 P:=0.0 Y:=0.0
```

`spawn_robot.launch.py` arguments:
- `world:=<world>` (default `empty`)
- `name:=<model_name>` (default `m100`)
- `type:=m100|...` (currently `m100` is the tested UAV type)
- `uav_mode:=teleport|physics`
- `with_camera:=true|false`
- `bridge_camera:=true|false` (bridge `/<name>/<camera_name>/(image_raw,camera_info)`)
- `camera_name:=camera0` (default `camera0`)
- `x:=<m>` `y:=<m>` `z:=<m>`
- `R:=<rad>` `P:=<rad>` `Y:=<rad>`
- `model:=...` (legacy arg, currently not used by the generated-SDF spawn path)

Attached gimbal rollback
------------------------
The current attached-gimbal fix depends on a non-static model with a kinematic UAV base link. If you need to back it out, revert these files together:

- `src/lrs_halmstad/launch/spawn_robot.launch.py`
  - restore `model_static_for_mode` to the old teleport/static rule
  - remove `base_link_kinematic_for_mode`
  - stop passing `-p base_link_kinematic:=...` into `generate_sdf`
- `src/lrs_halmstad/lrs_halmstad/generate_sdf.py`
  - remove the `base_link_kinematic` parameter and mapping
- `src/lrs_halmstad/xacro/lrs_model.xacro`
  - remove the `base_link_kinematic` xacro arg and passthrough into `lrs_m100_macro`
- `src/lrs_halmstad/xacro/lrs_m100_base.sdf.xacro`
  - remove `<kinematic>${base_link_kinematic}</kinematic>`
  - if you want the exact old behavior, also remove `<gravity>false</gravity>` from `base_link`

Expected rollback effect:
- UAV teleport mode returns to the old static-body behavior
- attached gimbal joint motion in Gazebo may stop being visibly rendered again

Movement and follow orchestration
---------------------------------
Motion-only launch:
```bash
  world:=warehouse uav_name:=dji0 ugv_cmd_topic:=/a201_0000/cmd_vel
```

- `world:=<world>` (default `warehouse`)
- `uav_name:=<uav_name>` (default `dji0`)
- `ugv_cmd_topic:=<topic>` (default `/a201_0000/cmd_vel`)
- `uav_log_csv:=<path>` (default empty)

Follow launch (UGV motion + `follow_uav` + optional `leader_estimator`):
```bash
ros2 launch lrs_halmstad run_follow.launch.py \
  world:=warehouse uav_name:=dji0 leader_mode:=odom
```

YOLO estimate mode (UAV follows UGV estimate from camera detections):
```bash
ros2 launch lrs_halmstad run_follow.launch.py \
  world:=warehouse uav_name:=dji0 leader_mode:=estimate \
  yolo_weights:=detection/yolo26/yolo26n.pt yolo_device:=cpu
```

`run_follow.launch.py` arguments:
- `params_file:=<yaml>` (default `config/run_follow_defaults.yaml`)
- `world:=<world>` (default `warehouse`)
- `uav_name:=<uav_name>` (default `dji0`)
- `leader_mode:=odom|pose|estimate` (default `odom`)
- `leader_perception_enable:=true|false` (default `false`)
- `start_leader_estimator:=auto|true|false` (default `auto`)
- `ugv_cmd_topic:=<topic>` (default `/a201_0000/cmd_vel`)
- `ugv_odom_topic:=<topic>` (default `/a201_0000/platform/odom`)
- `leader_image_topic:=<topic>` (default `/<uav_name>/camera0/image_raw`)
- `leader_camera_info_topic:=<topic>` (default `/<uav_name>/camera0/camera_info`)
- `leader_depth_topic:=<topic>` (default empty / disabled)
  Current limitation: the simulated UAV camera only bridges `/<uav_name>/camera0/image_raw` and `/<uav_name>/camera0/camera_info` today, so there is no UAV depth topic to wire here yet. Future implementation: add a UAV depth sensor/bridge before enabling estimator depth ranging from the UAV camera.
- `leader_uav_pose_topic:=<topic>` (default `/<uav_name>/pose_cmd`)
- `yolo_weights:=<weights.pt>` (relative paths resolve under `<workspace_root>/models`, for example `detection/yolo26/yolo26n.pt`)
- `yolo_device:=cpu|cuda` (default `cpu`)
- `event_topic:=<topic>` (default `/coord/events`)
- `ugv_start_delay_s:=<seconds>` (default `0.0`; readiness gate handles startup)

YOLO estimate mode notes
------------------------
- `leader_estimator` publishes:
  - `/coord/leader_estimate` (`geometry_msgs/msg/PoseStamped`)
  - `/coord/leader_estimate_status` (`std_msgs/msg/String`)
- If status shows `yolo=disabled yolo_reason=ultralytics_not_installed`, install the Python package:
```bash
python3 -m pip install --user ultralytics
```
- If YOLO is enabled but no detections, status will typically show `NO_DET`, `HOLD`, or `REACQUIRE`.

OMNeT bridge (TCP pose snapshots)
---------------------------------
This workspace currently uses `gazebo_pose_tcp_bridge` (ROS odom -> TCP snapshot server) for the OMNeT side.

Default start:
```bash
ros2 run lrs_halmstad gazebo_pose_tcp_bridge
```

Default behavior:
- Binds TCP server on `127.0.0.1:5555`
- Streams snapshot data from:
  - `/a201_0000/platform/odom` as model `robot`
  - `/dji0/pose/odom` as model `dji0`
- Auto-discovers any additional `/<model>/pose_cmd/odom` topics and exposes them as model `<model>`

Override bridge parameters:
```bash
ros2 run lrs_halmstad gazebo_pose_tcp_bridge --ros-args \
  -p bind_host:=127.0.0.1 \
  -p port:=5555 \
  -p auto_discover_pose_cmd_odom:=true \
  -p odom_topics:="['/a201_0000/platform/odom','/dji0/pose/odom']" \
  -p model_names:="['robot','dji0']"
```

`gazebo_pose_tcp_bridge` ROS parameters:
- `odom_topic:=<topic>` (legacy single-topic fallback)
- `model_name:=<name>` (legacy single-model fallback)
- `odom_topics:=['<topic1>','<topic2>', ...]`
- `model_names:=['<name1>','<name2>', ...]` (same length as `odom_topics`)
- `auto_discover_pose_cmd_odom:=true|false` (default `true`; discovers `/<model>/pose_cmd/odom`)
- `bind_host:=<ip>` (default `127.0.0.1`)
- `port:=<tcp_port>` (default `5555`)
- `stale_timeout_sec:=<seconds>` (default `2.0`)

Optional helper: pose_cmd -> odom converter
-------------------------------------------
`follow_uav` now publishes `/<uav>/pose_cmd/odom` directly, so this is usually not needed for the follow stack.

Use this helper only if another node publishes `PoseStamped` and you need an `Odometry` topic:
```bash
ros2 run lrs_halmstad pose_cmd_to_odom
```

`pose_cmd_to_odom` parameters:
- `pose_topic:=<topic>` (default `/dji0/pose_cmd`)
- `odom_topic:=<topic>` (default `/dji0/pose_cmd/odom`)
- `frame_id:=<frame>` (default empty = copy from pose header)
- `child_frame_id:=<frame>` (default `base_link`)
- `copy_header_stamp:=true|false` (default `true`)

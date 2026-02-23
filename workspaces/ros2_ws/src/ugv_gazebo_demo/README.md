# UGV Gazebo Demo

Launches Gazebo Harmonic with a flat world and spawns the Husky model wrapped with an odometry publisher.

Use `./run_ugv_demo.sh` as the normal way to run this project. Raw `ros2 ...` commands are included below only as equivalent reference commands.

## Run

Recommended (wrapper script):

```bash
cd /home/ruben/ros2_ws
./run_ugv_demo.sh
```

Spawn Bebop and choose a world (common setup):

```bash
cd /home/ruben/ros2_ws
./run_ugv_demo.sh bebop world=walls
```

Equivalent raw ROS 2 command:

```bash
source /opt/ros/<distro>/setup.bash
cd /home/ruben/ros2_ws
colcon build --packages-select ugv_gazebo_demo --build-base build --install-base install --log-base log
source install/setup.bash
ros2 launch ugv_gazebo_demo ugv_flat_world.launch.py
```

## `run_ugv_demo.sh` Arguments

Wrapper script location:

```bash
cd /home/ruben/ros2_ws
./run_ugv_demo.sh [args...]
```

Supported arguments and allowed values:

- `build`
  - No value. Builds `ugv_gazebo_demo` before running.
- `drive`
  - No value. Starts the normal UGV driver (`drive_ugv`).
- `drive_lidar`
  - No value. Starts the lidar-based reactive driver (`drive_lidar`).
- `bridge`
  - No value. Starts the TCP pose bridge for OMNeT.
  - Cannot be combined with `drive` or `drive_lidar`.
- `bebop`
  - No value. Spawns the Bebop drone in Gazebo.
- `camera`
  - No value. Also spawns the Bebop drone (camera panel appears in Gazebo if supported by the world GUI).
- `follow`
  - No value. Starts the Bebop follower when used with `drive` or `drive_lidar`.
- `follow_distance=<m>`
  - Numeric value (integer or decimal), e.g. `10`, `20`, `12.5`
  - Used with `drive` or `drive_lidar`
  - Example: `follow_distance=20`
- `takeoff_duration=<sec>`
  - Numeric value (integer or decimal), e.g. `8`, `15`, `12.5`
  - Used with follower / drive flow
  - Example: `takeoff_duration=8`
- `world=<name|file>`
  - World name without extension (e.g. `walls`, `default`)
  - Or an explicit file path (absolute or relative), e.g. `worlds/walls.sdf`

Rules:

- Use only one drive mode: `drive` or `drive_lidar`
- `follow` requires a drive mode
- `follow_distance=<m>` requires a drive mode
- If no drive mode and no bridge are given, the script launches Gazebo

## Available Worlds

Current files in `~/ros2_ws/worlds`:

- `default` (`default.sdf`)
- `walls` (`walls.sdf`)
- `aruco` (`aruco.sdf`)
- `baylands` (`baylands.sdf`)
- `forest` (`forest.sdf`)
- `kthspacelab` (`kthspacelab.sdf`)

Examples:

```bash
./run_ugv_demo.sh world=default
./run_ugv_demo.sh bebop world=walls
./run_ugv_demo.sh world=forest
./run_ugv_demo.sh world=worlds/kthspacelab.sdf
```

## Topics

- `/x2_ugv/odom` (nav_msgs/msg/Odometry)
- `/x2_ugv/cmd_vel` (geometry_msgs/msg/Twist)
- `/x2_ugv/scan` (sensor_msgs/msg/LaserScan)

Optional Bebop topics (when launched with `spawn_bebop:=true` or `./run_ugv_demo.sh bebop`):

- `/bebop1/cmd_vel` (geometry_msgs/msg/Twist)
- `/bebop1/enable` (std_msgs/msg/Bool)
- `/bebop1/odom` (nav_msgs/msg/Odometry)
- `/bebop1/camera/image` (sensor_msgs/msg/Image)
- `/bebop1/camera/camera_info` (sensor_msgs/msg/CameraInfo)
- `/bebop1/follow_ready` (std_msgs/msg/Bool)

## Drive the UGV

Run a simple velocity pattern (wrapper script):

```bash
cd /home/ruben/ros2_ws
./run_ugv_demo.sh drive
```

Equivalent raw ROS 2 command:

```bash
source /opt/ros/<distro>/setup.bash
cd /home/ruben/ros2_ws
source install/setup.bash
ros2 run ugv_gazebo_demo drive_ugv
```

Override the command topic if needed:

```bash
ros2 run ugv_gazebo_demo drive_ugv --ros-args -p cmd_vel_topic:=/x2_ugv/cmd_vel
```

## Drive with Lidar Avoidance

Run reactive obstacle avoidance based on LaserScan (wrapper script):

```bash
cd /home/ruben/ros2_ws
./run_ugv_demo.sh drive_lidar
```

Equivalent raw ROS 2 command:

```bash
source /opt/ros/<distro>/setup.bash
cd /home/ruben/ros2_ws
source install/setup.bash
ros2 run ugv_gazebo_demo drive_lidar
```

Useful parameter overrides:

```bash
ros2 run ugv_gazebo_demo drive_lidar --ros-args \
  -p scan_topic:=/x2_ugv/scan \
  -p cmd_vel_topic:=/x2_ugv/cmd_vel \
  -p safety_distance:=1.2 \
  -p critical_distance:=0.55
```

## Make Bebop Follow UGV

Launch the follower node by adding `follow` to your drive command:

```bash
./run_ugv_demo.sh drive follow
# or
./run_ugv_demo.sh drive_lidar follow
```

Set follower spacing directly from launcher:

```bash
./run_ugv_demo.sh drive follow follow_distance=20
# or with lidar drive:
./run_ugv_demo.sh drive_lidar follow follow_distance=20
```

When running `drive` / `drive_lidar`, the launcher also auto-starts the follower if it detects live `/bebop1/odom`.
If follower is active, the launcher waits for `/bebop1/follow_ready == true` (takeoff complete) before starting UGV motion.

The follower behavior:
- Enables `/bebop1/enable`
- Takes off first
- Tracks a target approximately `10 m` behind UGV heading
- Holds altitude approximately `4 m` above UGV

Tune offsets if needed:

```bash
ros2 run ugv_gazebo_demo bebop_follow_ugv --ros-args \
  -p follow_distance_m:=10.0 \
  -p hover_height_above_ugv_m:=4.0 \
  -p takeoff_height_above_ugv_m:=10.0
```

Vertical hold is enabled by default (`use_odom_vertical_control:=true`) so UAV maintains elevation.  
If bridged UAV odometry `z` is pinned at 0, set `-p use_odom_vertical_control:=false`.

You can set takeoff duration directly (seconds):

```bash
./run_ugv_demo.sh drive takeoff_duration=15
```

You can combine both:

```bash
./run_ugv_demo.sh drive follow follow_distance=20 takeoff_duration=15
```

## TCP Pose Bridge for OMNeT

Run a TCP bridge that serves pose snapshots from `/x2_ugv/odom` (wrapper script):

```bash
cd /home/ruben/ros2_ws
./run_ugv_demo.sh bridge
```

Equivalent raw ROS 2 command:

```bash
source /opt/ros/<distro>/setup.bash
cd /home/ruben/ros2_ws
source install/setup.bash
ros2 run ugv_gazebo_demo gazebo_pose_tcp_bridge
```

Defaults:

- Host: `127.0.0.1`
- Port: `5555`
- Model names in snapshot: `x2_ugv`, `bebop1` (when available)
- Protocol: send `GET` and receive one line `<count> <name> <x> <y> <z> ...`

Override parameters if needed:

```bash
ros2 run ugv_gazebo_demo gazebo_pose_tcp_bridge --ros-args \
  -p odom_topics:="['/x2_ugv/odom', '/bebop1/odom']" \
  -p model_names:="['x2_ugv', 'bebop1']" \
  -p bind_host:=127.0.0.1 \
  -p port:=5555
```

## Recommended Run Order (Gazebo + OMNeT)

Use separate terminals in this order.

1. Start Gazebo with UGV + Bebop:

```bash
cd /home/ruben/ros2_ws
./run_ugv_demo.sh bebop world=walls
```

2. Start the TCP pose bridge (for OMNeT):

```bash
cd /home/ruben/ros2_ws
./run_ugv_demo.sh bridge
```

3. Start UGV driving + Bebop follow:

```bash
cd /home/ruben/ros2_ws
./run_ugv_demo.sh drive follow follow_distance=20 takeoff_duration=8
```

4. Start OMNeT (Gazebo bridge config):

```bash
cd /home/ruben/omnet_workspace/UAV_UGV
./UAV_UGV -u Qtenv -f omnetpp.ini -c Communication-GazeboBridge -n src:../inet4.5/src
```

Optional terminal-only OMNeT run:

```bash
./UAV_UGV -u Cmdenv -f omnetpp.ini -c Communication-GazeboBridge -n src:../inet4.5/src
```

## Notes

- The launch file adds `~/ros2_ws/models` to `GZ_SIM_RESOURCE_PATH`.
- To spawn one Bebop drone with multicopter control plugins:
  - `ros2 launch ugv_gazebo_demo ugv_flat_world.launch.py spawn_bebop:=true`
  - or `./run_ugv_demo.sh bebop`
  - default spawn is `10 m` behind UGV (`bebop_spawn_x:=-10.0`, `bebop_spawn_y:=0.0`, `bebop_spawn_z:=0.15`)
- Bebop camera is shown inside Gazebo via an `ImageDisplay` GUI panel when worlds from `~/ros2_ws/worlds/*.sdf` are used.
- `./run_ugv_demo.sh bebop` and `./run_ugv_demo.sh camera` both spawn `bebop1` (camera feed appears in Gazebo panel).
- Launch arg `show_bebop_camera` is kept only for backward compatibility and is now a no-op.
- Enable and command the Bebop from ROS 2:

```bash
ros2 topic pub --once /bebop1/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub -r 20 /bebop1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.6, y: 0.0, z: 0.1}, angular: {z: 0.3}}"
```

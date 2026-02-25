# lrs_halmstad

## Plan

- Simulation of Husky in Warehouse environment
- Add moveable camera to warehouse environment
- Add UAVs with camera to warehouse environent

## MISC

See:

- https://docs.clearpathrobotics.com/docs/ros/config/generators/

## Simulate Husky in Gazebo using Warehouse Environment


See: https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install

## Launch Simulator and More

Possible values for the world parameter is:
- construction
- office
- orchard
- pipeline
- solar_farm
- warehouse

### Gazebo Sim

```bash
source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/halmstad_ws/src/lrs_halmstad/clearpath rviz:=false world:=orchard use_sim_time:=true
```

Change topic to: /a201_0000/cmd_vel

If we start the simulation this way then the value of
GZ_SIM_RESOURCE_PATH is ignored and instead "install/repo/share" is added to
the path that is searched.  So in this case
".../halmstad_ws/install/lrs_halmstad/share" is added to the model search path.

Simulated Image topics (Intel Realsense):

- /a201_0000/sensors/camera_0/color/image
- /a201_0000/sensors/camera_0/color/compressed

- /a201_0000/sensors/camera_0/color/camera_info

Simulated Depth topics (Intel Realsense).

- /a201_0000/sensors/camera_0/depth/image
- /a201_0000/sensors/camera_0/depth/compressedDepth

The simulated topics corresponds to the available real Husky A200.

### Rviz2

```bash
source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
rviz2 -d ~/halmstad_ws/src/lrs_halmstad/clearpath/husky.rviz --ros-args -r /tf:=/a201_0000/tf -r /tf_static:=/a201_0000/tf_static -p use_sim_time:=true
```
### Add UAVs with cameras

Add 3 Uav:s:

```bash
source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
ros2 launch lrs_halmstad spawn_uavs.launch.py world:=orchard
```

Service calls:

- /world/orchard/set_pose  (used in command.py examples)

Topics:

- /dji0/camera0/image_raw
- /dji0/camera0/camera_info
- /dji1/camera0/image_raw
- /dji1/camera0/camera_info
- /dji2/camera0/image_raw
- /dji2/camera0/camera_info

View images:

```bash
ros2 run  rqt_image_view rqt_image_view dummy
```

### Move Cameras

Teleport:
```bash
ros2 run lrs_halmstad command --ros-args -p command:=setpose -p x:=3.0 -p z:=15.0 -p pitch:=-65.0 -p yaw:=0.0 -p name:=dji1
```

Move in scan pattern:
```bash
ros2 run lrs_halmstad command --ros-args -p command:=scan -p x:=10.0 -p y:=10.0 -p z:=15.0 -p pitch:=-89.0 -p yaw:=0.0 -p name:=dji0
```


Look at Husky:
```bash
todo
```



# Manual YOLO Follow

Run each section in a separate terminal, in this order. Run `ws` first in every terminal.

## 1. Start Gazebo And Husky

```bash
ws
ros2 launch lrs_halmstad managed_clearpath_sim.launch.py gui:=false
```

## 2. Check Husky Controllers

```bash
ws
ros2 control list_controllers -c /a201_0000/controller_manager
```

Both `joint_state_broadcaster` and `platform_velocity_controller` must be `active`.
If they are not active, run:

```bash
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /a201_0000/controller_manager \
  --controller-manager-timeout 60 \
  --switch-timeout 30 \
  --service-call-timeout 60

ros2 run controller_manager spawner platform_velocity_controller \
  --controller-manager /a201_0000/controller_manager \
  --controller-manager-timeout 60 \
  --switch-timeout 30 \
  --service-call-timeout 60
```

## 3. Spawn UAV

```bash
ws
ros2 launch lrs_halmstad spawn_uav_1to1.launch.py bridge_depth:=true
```

## 4. Drive Husky Manually

```bash
ws
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -p speed:=1.0 \
  -p turn:=0.8 \
  -r cmd_vel:=/a201_0000/cmd_vel
```

## 5. Start YOLO Follow

```bash
ws
export PYTHONNOUSERSITE=1
export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages"
ros2 launch lrs_halmstad run_follow.launch.py \
  world:=baylands \
  ugv_mode:=external \
  leader_mode:=estimate \
  external_detection_enable:=true \
  yolo_weights:=/home/ruben/halmstad_ws/models/obb/mymodels/new/v10-tuned-no-rotundan.pt
```

## 6. View YOLO Output

```bash
ws
ros2 run rqt_image_view rqt_image_view
```

Select `/coord/leader_debug_image`. The raw UAV image is `/dji0/camera0/image_raw`.

## Optional Status Checks

```bash
ws
ros2 topic echo /coord/leader_detection_status
```

```bash
ws
ros2 topic echo /coord/leader_estimate_status
```

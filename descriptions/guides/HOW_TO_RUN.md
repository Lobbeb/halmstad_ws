# Running the shit with everything to capture the data

## Follow mode

För att starta navigation och localization på UGVn behövs:

* Genererade waypoints (routes) för enstaka rutter: **halmstad_ws/src/lrs_halmstad/config/baylands_waypoints**
* Eller den stora routen för hela mappen: **halmstad_ws/src/lrs_halmstad/config/baylands_waypoints.yaml**
* En startpunkt/waypoint (kan egentligen vara vart som helst med lättare för UGVn att klara målet om den är förbestämd.)

Detta sätts automatiskt av olika scripts om du har med i dina args:

```bash
waypoint:=...  (initialposition)        nav2_goals:=...  (route)
```

Exempel:
```bash
waypoint:=parkinglot_west_0          nav2_goals:=parkinglot_west
```

Så för att köra med en UGV och vanlig follow (utan yolo) på UAVn så är bästa sätt:

```bash
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
source ~/halmstad_ws/install/setup.bash
```

> *Tips - lägg till detta i din *.bashrc* fil:*

```bash
build() {
    colcon build --symlink-install --packages-select lrs_halmstad lrs_halmstad_gui_plugins pointcloud_to_laserscan
    source /opt/ros/jazzy/setup.bash
    source ~/halmstad_ws/src/lrs_halmstad/clearpath/setup.bash
    source ~/halmstad_ws/install/setup.bash
}
```

```bash
./run.sh nav2_tuning start nav2_goals:=... waypoint:=... spawn_uav:=true
```

Om du explicit ska skriva varenda argument:

```bash
./run.sh nav2_tuning start|restart|stack_stop|follow|route_stop|stop|attach|status
  world:=baylands
  profile:=standard|minimal
  lidar:=2d|3d
  pause_after_goal_s:=0.0
  gui:=true|false 
  perspective:=NAV2 
  start_rqt:=true|false 
  start_collision_monitor:=true|false 
  map:=maps/baylands.yaml
  clock_mode:=guarded|direct
  start_optional_teleop:=true|false
  spawn_uav:=true|false
  start_camera_tracker:=true|false
  with_route_driver:=true|false 
  follow_start_delay_s:=10.0
  uav_camera_update_rate:=20
  localization_ready_timeout_s:=20
  localization_scan_ready_timeout_s:=20
  nav2_start_delay_s:=10
  spawn_pre_delay_s:=20
  spawn_post_delay_s:=5
  rebuild:=true|false
  session:=name 
  tmux_attach:=true|false
  nav2_goals:=rotundan 
  waypoint:=rotundan_0
  dry_run:=true|false
```

Examples:

* ./run.sh nav2_tuning start
* ./run.sh nav2_tuning start profile:=minimal
* ./run.sh nav2_tuning start waypoint:=rotundan_0
* ./run.sh nav2_tuning start waypoint:=strip_2 nav2_goals:=strip
* ./run.sh nav2_tuning start spawn_uav:=true
* ./run.sh nav2_tuning restart
* ./run.sh nav2_tuning stack_stop
* ./run.sh nav2_tuning follow
* ./run.sh nav2_tuning route_stop
* ./run.sh nav2_tuning stop
* ./run.sh nav2_tuning attach

 Sen attacha med tmux för att kika på vad som händer:

```bash
 tmux attach
```

Första sidan visar Gazebo starten och UAV spawning, nästa sida innehåller Localization, Nav2 och Follow koden. (Ctrl+B, sen klicka på N för att bläddra)

*Nav2 Goals/routes du kan välja på: (I ordern dem är i den stora rutten):*

* rotundan          (finished)
* road_to_west      (finished)
* parkinglot_west   (finished)
* road_to_spawn     (needs tuning)
* spawn             (finished)
* road_to_east      (needs tuning)
* parkinglot_east   (needs tuning)
* road_to_strip     (needs tuning)
* strip             (needs tuning)

*Enstaka rutter inte kopplade till baylands_waypoints rutten:*

* art
* playground

*Waypoints du kan välja på:*

* rotundan_0 \- 8
* road_to_west_0 \- 8
* parkinglot_west_0 \- 21
* road_to_spawn_0 \- 10
* spawn_0 \- 8
* road_to_east_0 \- 11
* parkinglot_east_0 \- 9
* road_to_strip_0 \- 9
* strip_0 \- 16

Medans du kör kan du göra ändringar i koden (yaml filerna eller python scripts) och restarta utan att bygga om:

```bash
./run.sh nav2_tuning restart
```

Då kör den om allt och startar på samma waypoint som förut och med samma rutt.
Vill du börja ifrån en ny waypoint men med samma rutt skriver du:

```bash
./run.sh nav2_tuning restart waypoint:=NEW_WAYPOINT_HERE
```

Vill du byta rutt i samma session, ge både waypoint och route:

```bash
./run.sh nav2_tuning restart waypoint:=strip_0 nav2_goals:=strip
```

Vill du påbörja en helt ny session:

```bash
./run.sh nav2_tuning stop
./run.sh nav2_tuning start nav2_goals:=NEW_ROUTE_HERE
```

Väljs ingen waypoint tas den första i routen automatiskt.

## Active UGV sensor setup

The Clearpath-generated camera and 3D lidar ROS launches are disabled in:

* `src/lrs_halmstad/clearpath/robot.yaml`

Current ownership is:

* UGV RGB camera sensor is still created by `robot.yaml` with `urdf_enabled: true`.
  * Set UGV camera size/rate there with `image_width`, `image_height` and `rgb_camera.profile`.
  * ROS bridging is done by `src/lrs_halmstad/launch/ugv_rgb_camera_bridge.launch.py`.
  * Only the color image and color camera info are bridged.
* UGV 3D lidar sensor is created by `src/lrs_halmstad/clearpath/lidar3d_0_override.urdf.xacro`.
  * Set lidar range, angles and Gazebo update rate there.
  * ROS bridging is done by `src/lrs_halmstad/launch/ugv_lidar3d_points_bridge.launch.py`.
  * Only `/a201_0000/sensors/lidar3d_0/points` is bridged.
  * Native `/a201_0000/sensors/lidar3d_0/scan` is intentionally not bridged.
* Nav2/localization should use pc2ls:
  * `/a201_0000/sensors/lidar3d_0/points`
  * `/a201_0000/sensors/lidar3d_0/scan_from_points`

Do not tune UGV 3D lidar rate/range in the disabled `lidar3d:` block in `robot.yaml`; that block is only kept as a disabled Clearpath entry.
Generated files under `src/lrs_halmstad/clearpath/sensors/config/` may still exist after Clearpath generation, but they are not active when `src/lrs_halmstad/clearpath/sensors/launch/sensors-service.launch.py` is empty.

Lidar intställningar för varje waypoint ligger i ~*src/lrs_halmstad/config/baylands_route_lidar.yaml*
De sätts automatiskt av localization vid start och av nav2-drivern vid waypoint-övergångar.
Manuellt kan det tunas med:

```bash
ros2 param set /a201_0000/pointcloud_to_laserscan min_height -0.4
ros2 param set /a201_0000/pointcloud_to_laserscan max_height 0.5
./run.sh pc2ls_sweep sweep:=min min_start:=-0.7 max:=0.5 step:=0.05 dwell_s:=1
```

Skulle det hänga sig och det ligger massa bakgrunds processer så kör:

```bash
./run.sh kill_all_ros2
```

Det dödar ALLT.

## Visualisation

Kör bara RViz, det är bäst.

```bash
./run.sh nav2_rviz lidar:=3d config:=follow
```

Configs att välja på (ligger i halmstad_ws/src/lrs_halmstad/config/rviz_configs)

* waypoints_testing
* localization_testing
* follow <-- KÖR MED DENNA

Jag la till images för kamerorna i där, men vill du köra med rqt så kan du göra det:

```bash
./run.sh rqt_perspective
```

## Dataset Collection

Alla dataset hamnar under `~/halmstad_ws/datasets/`

### Leader UAV (`dji0`) - route capture + OBB labels

Start Nav2/follow yourself first. The collector does not start or stop
Gazebo, localization, Nav2, follow, or the UAV:

```bash
cd ~/halmstad_ws
./run.sh nav2_tuning start spawn_uav:=true waypoint:=strip_0 nav2_goals:=strip
```

Then run the collector from another terminal:

```bash
cd ~/halmstad_ws
./run.sh collect_leader_dataset baylands route:=strip
```

Output goes to:

```text
datasets/baylands_leader_routes/<route>/v1
datasets/baylands_leader_routes/<route>/v2
...
```

The collector starts the recorder when it sees the external Nav2 route driver,
then keeps recording until you stop it with `Ctrl-C`. The recorder waits
internally for UAV camera, camera pose, and UGV pose messages. It saves only
useful frames: it skips images when the UGV pose is not available or when the
UGV does not project into a valid box. It writes AABB labels to `labels_aabb/`
and metadata to `metadata/`. When you stop the collector, it only stops the
recorder.

Generate Ultralytics OBB labels after capture:

```bash
cd ~/halmstad_ws
./run.sh dataset_make_obb datasets/baylands_leader_routes/strip/v1 --overwrite --overlay
```

That writes OBB labels to `labels/` in Ultralytics format:

```text
class_index x1 y1 x2 y2 x3 y3 x4 y4
```

Coordinates are normalized to `[0, 1]`, matching the Ultralytics OBB dataset
format.

Run one route, or a selected list of routes:

```bash
cd ~/halmstad_ws
./run.sh collect_leader_dataset baylands dry_run:=true
./run.sh collect_leader_dataset baylands route:=strip
./run.sh collect_leader_dataset baylands routes:=parkinglot_west,strip
```

Use `once:=true` if you want the collector to capture one route run and then
exit:

```bash
cd ~/halmstad_ws
./run.sh collect_leader_dataset baylands route:=strip once:=true
```

You can still move the UAV manually while capture is running:

```bash
cd ~/halmstad_ws
./run.sh uav_position angle 30
./run.sh uav_position distance 12
```

For live keyboard control of follow distance, follow angle, and gimbal:

```bash
cd ~/halmstad_ws
./run.sh follow_control keyboard --uav-name dji0 --step-d-target 1 --step-heading 15 --step-pan 5 --step-tilt 5
```

Keys: `w/s` changes follow distance, `a/d` changes follow angle, `j/l`
changes gimbal pan, `i/k` changes gimbal tilt, `c` centers the gimbal, `p`
prints current values, `q` quits.

### Leader UAV (`dji0`) - manuell capture

Starta först Nav2/follow med UAV:

```bash
cd ~/halmstad_ws
./run.sh nav2_tuning start spawn_uav:=true waypoint:=rotundan_0 nav2_goals:=rotundan start_camera_tracker:=true
```

Starta sedan capture:

```bash
cd ~/halmstad_ws
./run.sh capture_dataset baylands out:=datasets/pilot_rotundan uav_name:=dji0 hz:=0.5 save_overlay:=true save_metadata:=true negatives:=true
```

Generate Ultralytics OBB labels from the saved metadata after capture:

```bash
cd ~/halmstad_ws
./run.sh dataset_make_obb datasets/pilot_rotundan --overlay --overwrite
```

Optional gimbal-only variation while capture is running:

```bash
cd ~/halmstad_ws
./run.sh follow_control random --gimbal-only --uav-name dji0 --interval 6 --gimbal-interval 6 --pan-amplitude 25 --tilt-center -45 --tilt-amplitude 12
```

`follow_control` sends gimbal commands directly to `/dji0/update_pan` and
`/dji0/update_tilt`.

Simple live UAV follow-position commands:

```bash
cd ~/halmstad_ws
./run.sh uav_position angle 45
./run.sh uav_position angle -45
./run.sh uav_position distance 12
./run.sh uav_position angle:=45 distance:=12
./run.sh uav_position angle_abs:=0 distance:=8 wait_s:=1
```

`angle` is relative to the current follow angle. `angle_abs` and `distance`
are absolute. Default wait for `/follow_uav` is 3 seconds; use `wait_s:=1` or
`no_wait:=true` if you know it is already active.

### Support UAVs (`dji1` och `dji2`) - capture båda samtidigt

Starta först vanlig follow:

```bash
cd ~/halmstad_ws
./run.sh tmux_1to1 baylands mode:=follow waypoint:=parkinglot_west_0 nav2_goals:=parkinglot_west
```

Starta support-UAV:erna:

```bash
cd ~/halmstad_ws
./run.sh support_follow_odom baylands support_with_camera:=true support_bridge_gimbal:=true
```

Starta kamerasvep:

```bash
cd ~/halmstad_ws
./run.sh support_camera_scan baylands yaw_amplitude_deg:=120 period_s:=12 pan_phase_offsets_deg:=0,180 pitch_deg:=-22
```

Starta capture för båda:

```bash
cd ~/halmstad_ws
./run.sh support_capture_pair baylands out:=datasets/baylands_support_pair hz:=1.0 save_overlay:=false
```

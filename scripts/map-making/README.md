# Map-Making Scripts

This folder holds the scripts that are mainly used for SLAM capture, map
cleanup, and waypoint capture workflows.

Shared helpers such as `slam_state_common.sh` and `lidar_mode_common.sh` stay in
the parent `scripts/` folder because they are also used by non-mapping scripts.

## Core keep

- `run_slam.sh`: Starts the normal SLAM Toolbox mapping session.
- `run_slam_save_state.sh`: Saves a checkpoint with pose graph, occupancy map,
  and resume metadata.

## Support utilities

- `run_slam_rviz.sh`: Opens the normal RViz view used with SLAM/Nav2.
- `run_current_pose.sh`: Prints the current Gazebo pose and a reusable respawn
  command for the robot.
- `run_make_nav_map.sh`: Wrapper around `make_nav_map.py` for generating a
  coarser Nav2 map from a mapping output.
- `make_nav_map.py`: Downsamples a map image/YAML pair into a trinary nav map.
- `run_save_waypoint_csv.sh`: Captures the current Gazebo pose plus AMCL pose
  into a waypoint CSV row.
- `run_save_waypoint_yaml.sh`: Captures the current AMCL pose directly into a
  waypoint YAML file.
- `run_sync_waypoints_csv.sh`: Regenerates the route YAML and RViz waypoint
  files from the grouped waypoint CSV.

## Likely optional

- `run_current_pose.sh`: Optional if you no longer respawn from live Gazebo
  poses.
- `run_save_waypoint_yaml.sh`: Optional if CSV capture plus
  `run_sync_waypoints_csv.sh` is now your only waypoint workflow.

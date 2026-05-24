# Trajectory Plot Handoff

handoff for comparing paths from recorded ROS 2 runs.
It is meant for baseline, visual-follow, or communication comparisons such as
duplex vs simplex.

## Script

Use:

```bash
scripts/plot_trajectory_paths.py
```

The script reads a ROS 2 bag from a run folder and plots x-y trajectories. It can
plot:

- a reference / UGV path
- a UAV / followed path
- optional estimated or predicted paths

It writes both PNG and PDF.

## Input Data

The script expects a recorded run folder with a `bag/` subfolder, for example:

```text
bags/results_c1_batch01/C1_odom/r01/
```

Useful default topics:

```text
/a201_0000/ground_truth/odom   reference / UGV path
/dji0/pose                     UAV path
```

Other useful topics, depending on the run:

```text
/a201_0000/amcl_pose_odom
/a201_0000/platform/odom/filtered
/coord/leader_estimate
/dji0/pose_cmd
/dji0/pose_cmd/odom
```

For communication comparisons, use the same topic choices for each run and save
the plots with names that identify the communication mode, for example
`duplex`, `simplex`, or a specific OMNeT profile.

## Example

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/plot_trajectory_paths.py \
  --run-dir bags/results_c1_batch01/C1_odom/r01 \
  --reference-topic /a201_0000/ground_truth/odom \
  --uav-topic /dji0/pose \
  --estimated-topic "Estimated=/coord/leader_estimate" \
  --warmup 30 \
  --title "Example path comparison" \
  --out /tmp/example_path_comparison
```

This creates:

```text
/tmp/example_path_comparison.png
/tmp/example_path_comparison.pdf
```

If the run does not contain `/coord/leader_estimate`, the script warns and still
plots the available paths.

## Adapting For Duplex vs Simplex

Run the script once per recorded communication run and keep the same topics:

```bash
python3 scripts/plot_trajectory_paths.py \
  --run-dir <duplex-run-folder> \
  --reference-topic /a201_0000/ground_truth/odom \
  --uav-topic /dji0/pose \
  --estimated-topic "Estimated=/coord/leader_estimate" \
  --warmup <warmup-seconds> \
  --title "Duplex path comparison" \
  --out <output-folder>/duplex_path_comparison

python3 scripts/plot_trajectory_paths.py \
  --run-dir <simplex-run-folder> \
  --reference-topic /a201_0000/ground_truth/odom \
  --uav-topic /dji0/pose \
  --estimated-topic "Estimated=/coord/leader_estimate" \
  --warmup <warmup-seconds> \
  --title "Simplex path comparison" \
  --out <output-folder>/simplex_path_comparison
```

For predicted-path comparisons, replace or add the estimated topic:

```bash
--estimated-topic "Predicted=<predicted-path-topic>"
```

The script supports common pose-like message types such as `Odometry`,
`PoseStamped`, `PoseWithCovarianceStamped`, and `Path`.

## Known Limitations

- The script plots x-y paths only.
- It does not compute path error metrics by itself.
- It does not align separate runs against each other; compare separate runs by
  using the same route, warmup, duration, and topic choices.
- If a topic is missing from the bag, it is skipped with a warning.

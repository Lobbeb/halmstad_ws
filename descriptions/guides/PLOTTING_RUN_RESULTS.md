# Plotting Run Results

Use this after a recorded run has finished and the rosbag is closed.

## Environment

Run from the workspace root:

```bash
cd /home/ruben/halmstad_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

If plotting fails with missing Python packages, also activate the same Python/ROS environment used for the simulation.

## Run Folder Layout

Single-run recordings usually look like:

```text
bags/experiments/baylands/follow_0524-002129/
  bag/
  metadata.json
  topics.txt
```

Route sweep recordings usually look like:

```text
bags/results_baylands_lora/
  rep01/
    R01_rotundan/
      bag/
      metadata.json
      topics.txt
      omnet/
      pane_logs/
```

Create plots next to the bag:

```bash
mkdir -p <run-dir>/plots
```

## Trajectory Plot

Use the existing trajectory script:

```bash
python3 scripts/plot_trajectory_paths.py \
  --run-dir <run-dir> \
  --warmup 30 \
  --title "<run label>" \
  --out <run-dir>/plots/trajectory
```

Example:

```bash
python3 scripts/plot_trajectory_paths.py \
  --run-dir bags/results_baylands_lora/rep01/R01_rotundan \
  --warmup 30 \
  --title "R01 rotundan" \
  --out bags/results_baylands_lora/rep01/R01_rotundan/plots/trajectory
```

Outputs:

```text
<run-dir>/plots/trajectory.png
<run-dir>/plots/trajectory.pdf
```

Default plotted topics:

```text
/a201_0000/ground_truth/odom
/dji0/pose
```

For YOLO/estimate runs, add extra path topics if they exist:

```bash
python3 scripts/plot_trajectory_paths.py \
  --run-dir <run-dir> \
  --warmup 30 \
  --estimated-topic "Leader estimate=/coord/leader_estimate" \
  --extra-topic "Follow point=/coord/leader_follow_point" \
  --title "<run label>" \
  --out <run-dir>/plots/trajectory
```

## Inspect Recorded Topics

Before plotting, check what was actually recorded:

```bash
cat <run-dir>/topics.txt
ros2 bag info <run-dir>/bag
```

For interactive replay/inspection:

```bash
./run.sh bag_monitor <run-dir> /omnet/packet_delivery_ratio /omnet/rssi_dbm /omnet/radio_distance
```

## OMNeT Metrics To Plot

For LoRa runs, the useful bag topics are:

```text
/omnet/packet_delivery_ratio
/omnet/packet_error_rate
/omnet/latency_s
/omnet/jitter_s
/omnet/rssi_dbm
/omnet/snir_db
/omnet/radio_distance
```

Recommended per-run figure:

```text
<run-dir>/plots/omnet_metrics.png
<run-dir>/plots/omnet_metrics.pdf
<run-dir>/plots/omnet_metrics.csv
```

Recommended stacked panels:

```text
1. PDR and PER
2. latency_ms and jitter_ms
3. RSSI_dBm and SNIR_dB
4. radio_distance_m
```

Skip the first 30 seconds for thesis summaries unless explicitly studying startup behavior:

```text
warmup_s = 30
```

The route sweep already prints and saves average measured OMNeT metrics at the end when `record:=true`. Use the detailed time-series plot for diagnosing a single route/run, then use the route sweep summary for campaign-level averages.

## Campaign Summary Plots

After a full route sweep, use the summary files first:

```text
bags/results_baylands_lora/<campaign_tag>_summary.txt
/tmp/halmstad_ws/nav2_route_sweep/<run_id>/summary.txt
```

Useful thesis summary plots:

```text
average PDR per route
average PER per route
average latency_ms per route
average jitter_ms per route
RSSI/SNIR distribution per route
radio_distance distribution per route
```

Keep trajectory plots and OMNeT plots separate at first. It makes failures easier to diagnose: route/navigation failures show up in `trajectory.*`; radio/network failures show up in `omnet_metrics.*`.

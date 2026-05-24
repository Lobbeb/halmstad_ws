# Baylands Simulation Test Plan

## Goal

Run repeatable Baylands simulations for thesis network results, using the same general setup as William's short C1-style run but covering all Baylands route groups over several smaller sessions if needed.

The primary output is OMNeT/LoRa network metrics. Gazebo, Nav2, and follow behavior are there to generate realistic UAV/UGV motion and should be kept consistent so the network results are comparable.

## Test Modes

### 1. Short Smoke Run

Use this before long recording sessions.

- One route only, usually `rotundan`.
- Confirm Gazebo, localization, Nav2, YOLO/follow, OMNeT, and recording all start.
- Confirm `/omnet/*` topics publish.
- Confirm the UGV/UAV motion is good enough to produce meaningful network samples.

### 2. Main Baylands Route Sweep

Run each route group once, then repeat later if time allows. It does not need to be one continuous run.

Primary route order:

| Run | Route | Start waypoint |
| --- | --- | --- |
| R01 | `rotundan` | `rotundan_0` |
| R02 | `road_to_west` | `road_to_west_0` |
| R03 | `parkinglot_west` | `parkinglot_west_0` |
| R04 | `road_to_spawn` | `road_to_spawn_0` |
| R05 | `spawn` | `spawn_0` |
| R06 | `road_to_east` | `road_to_east_0` |
| R07 | `parkinglot_east` | `parkinglot_east_0` |
| R08 | `road_to_strip` | `road_to_strip_0` |
| R09 | `strip` | `strip_0` |

If a route fails because of Nav2, Gazebo, or GPU instability, keep the failed bag/logs but mark it as invalid for network summary statistics unless enough stable OMNeT time was recorded.

### 3. Repetitions

Target repetitions:

- Minimum: 1 full pass over all route groups.
- Better: 3 repetitions per route group.
- If time is limited, prioritize routes with representative behavior: `rotundan`, `parkinglot_west`, `parkinglot_east`, and `strip`.

## Standard Run Settings

Use:

- `world`: `baylands`
- `mode`: `follow` for C1-like network baseline runs, or `mode:=yolo` only when testing network metrics under visual follow.
- `gui`: `false`
- `record`: `true`
- `record_profile`: `default`
- `omnet`: `true`
- `omnet_network`: `lora`
- `omnet_ui`: `cmdenv`
- Warmup exclusion for analysis: first 30 seconds.

For LoRa-assisted perception tests, make sure the estimator uses `/omnet/radio_distance`. The pose-derived `link_distance` value stays in OMNeT output only and is not republished as a ROS topic.

## Campaign Command

Dry-run the full C1-style campaign first. It should print 27 route jobs:

```bash
./run.sh nav2_route_sweep baylands \
  routes:=rotundan,road_to_west,parkinglot_west,road_to_spawn,spawn,road_to_east,parkinglot_east,road_to_strip,strip \
  repetitions:=3 \
  route_sim_timeout_s:=300 \
  campaign_tag:=c1_baylands_lora \
  record:=true \
  record_profile:=default \
  record_root:=bags/results_baylands_lora \
  omnet:=true \
  omnet_network:=lora \
  omnet_ui:=cmdenv \
  gui:=false \
  mode:=follow \
  lidar:=3d \
  dry_run:=true
```

Real run is the same command without `dry_run:=true`.

For a short smoke test:

```bash
./run.sh nav2_route_sweep baylands \
  routes:=rotundan,road_to_west \
  repetitions:=1 \
  route_sim_timeout_s:=30 \
  campaign_tag:=c1_smoke \
  record:=true \
  record_profile:=default \
  record_root:=bags/results_baylands_lora_smoke \
  omnet:=true \
  omnet_network:=lora \
  omnet_ui:=cmdenv \
  gui:=false \
  mode:=follow \
  lidar:=3d \
  dry_run:=true
```

## What To Record

Use the tmux wrapper recording support. Keep ROS bags and OMNeT files under the same per-run folder:

```text
bags/results_baylands_lora/
  rep01/
    R01_rotundan/
      bag/
      omnet/
      pane_logs/
  rep02/
    R01_rotundan/
      bag/
      omnet/
  rep03/
    R01_rotundan/
      bag/
      omnet/
```

Treat the raw ROS bag and OMNeT `.vec/.sca/.tlog` files as source data. CSVs and plots are derived outputs.

## Metrics To Compare

Main network metrics:

- Packet delivery ratio.
- Packet error rate.
- Latency.
- Jitter.
- RSSI.
- SNIR.
- Link distance.
- Radio-derived distance.
- Packet sent/received counts from OMNeT files.

Use route/control metrics only as validity checks and context for the network results.

Route and control:

- Route completed or failed.
- Time to completion.
- Where it got stuck, if it failed.
- Nav2 recovery/clear events.
- Collision monitor stop/slowdown events.
- UAV follow stability after warmup, mainly to explain network outliers.
- Follow distance and XY error, mainly to explain network outliers.
- Yaw/gimbal command stability, mainly to explain network outliers.

OMNeT/LoRa:

- `/omnet/rssi_dbm`
- `/omnet/snir_db`
- `/omnet/packet_error_rate`
- `/omnet/packet_delivery_ratio`
- `/omnet/latency_s`
- `/omnet/jitter_s`
- `/omnet/radio_distance`
- `/omnet/sim_time`

From OMNeT files:

- Packets sent/received.
- PDR / delivery ratio.
- RSSI and SNIR over time.
- Latency and jitter.
- Queue or data-rate metrics if available.

## Useful Commands

Inspect live OMNeT metrics:

```bash
./run.sh omnet_monitor once:=true
```

Analyze OMNeT output for one run:

```bash
cd /home/ruben/omnet_workspace/UAV_UGV
python3 network_metrics_analysis.py \
  --results /home/ruben/halmstad_ws/bags/results_baylands_lora/R01_rotundan/omnet \
  --config LoRa \
  --plot 30 \
  --write-csv
```

Optional path-loss export:

```bash
cd /home/ruben/omnet_workspace/UAV_UGV
python3 path_loss_analysis.py \
  --results /home/ruben/halmstad_ws/bags/results_baylands_lora/R01_rotundan/omnet \
  --config LoRa
```

Compare against William's C1 files when available:

```text
bags/results_c1_batch01/export/c1_batch01_runs.csv
bags/results_c1_batch01/export/c1_batch01_aggregate.csv
bags/results_c1_batch01/export/c1_batch01_by_route.csv
```

## Run Notes

Use this table while collecting data.

| Run | Route               | Repetition | Network data valid? | Result / notes |
| --- | ------------------- | ---------- | ------------------- | -------------- |
| R01 | rotundan            | 3          |                     |                |
| R02 | road_to_west        | 3          |                     |                |
| R03 | parkinglot_west     | 3          |                     |                |
| R04 | road_to_spawn       | 3          |                     |                |
| R05 | spawn               | 3          |                     |                |
| R06 | road_to_east        | 3          |                     |                |
| R07 | parkinglot_east     | 3          |                     |                |
| R08 | road_to_strip       | 3          |                     |                |
| R09 | strip               | 3          |                     |                |

Distance sweeps: Start at normal distances (7 m offset), then increase.
Test limits up to 100 m.

## Assumptions

- Use native FLORA LoRa via `omnet_network:=lora`.
- Keep the current Baylands follow height/defaults unless the run notes say otherwise.
- Exclude the first 30 seconds from plots and summary statistics.
- Do not use OMNeT `link_distance` as a perception range source for fair LoRa-assisted visual tests. It is geometric ground truth and stays OMNeT-side.
- No new scripts are required to gather the data.

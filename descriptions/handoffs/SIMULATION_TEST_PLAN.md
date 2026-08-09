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

LoRa perception has two intended modes:

- **Simplex LoRa**: `omnet_network:=lora`. The UAV does not get UGV feedback. `/omnet/radio_distance` is available as an RSSI/path-loss derived metric, but `range_mode:=auto` does not use it unless `radio_fallback_enable:=true`.
- **Duplex LoRa**: `omnet_network:=lora-duplex`. The UGV is allowed to communicate back through OMNeT. The bridge publishes the decoded UGV-to-UAV LoRa distance payload on `/omnet/radio_distance`, but this is saved as a metric only and is not used for coordination.

In duplex, `/omnet/radio_distance` updates only after OMNeT sees a successful reverse-link delivery at the UAV radio; it is not an RSSI estimate.
Use `range_mode:=radio` only for explicit radio-only estimator tests, not for the main C2 runs.

## Current Workflow

C1 and C2 live runs keep ROS bags and OMNeT files in the same route folder. C3 and C4 thesis bags are already recorded without `/omnet/*`, so network metrics are generated afterward with offline OMNeT replay.

Use `replay_scale:=3` for offline replay batches by default. It sets both the bag pose replay speed and OMNeT `realtimescheduler-scaling`, so a 300 s bag takes about 100 s. `replay_scale:=4` has looked fine in spot testing and can be used if completion checks still pass.

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

Network metric plots can be generated directly from the recorded bags. Trajectory-only figures are handled by `plot_trajectory_paths.py`.

```bash
./run.sh plot_network_metrics \
  --run-dir bags/results_baylands_lora_c2_duplex/rep01/R01_rotundan
```

For a whole campaign directory:

```bash
./run.sh plot_network_metrics \
  --results-dir bags/results_baylands_lora_c2 \
  --lora-mode rep-map
```

By default this writes one campaign-level thesis signal/loss figure under `<results-dir>/plots/network/combined/`, plus `network_metrics_summary.csv`. Plot labels use route-order letters `A-I` for `R01-R09`. C2 plot labels use `Simplex`, `Duplex`, and `Distance Sweep` instead of `rep01`, `rep02`, and `rep03`. Use `--per-run-plots` only when debugging a specific route; per-run figures go under `repXX/`.

Offline replay writes:

```text
<run>/offline_omnet/<network>_sf7_bw125kHz/
  network_metrics.csv
  omnet.log
  pose_replay.log
  metrics_capture.log
  omnet/
```

The batch replay manifest is:

```text
bags/ruben_c1_c4_selected_rosbags_2026-05-25/offline_omnet_manifest.csv
```

## Metrics To Compare

Main network metrics to keep in CSV/summary:

- Packet delivery ratio.
- Packet error rate.
- Latency.
- Jitter.
- RSSI.
- SNIR.
- Link distance.
- Radio-derived distance.
- Packet sent/received counts from OMNeT files.

Preferred thesis figures:

- Campaign signal/loss summary: RSSI against true UAV-UGV distance, with PER on the right axis.
- Link distance and radio-derived distance remain as separate raw CSV/summary columns.
- C2 repetition averages: one combined time-series figure each for `simplex`, `duplex`, and `distance sweep`, averaged across routes with `--rep-average-plots`; the distance panel shows true distance, the displayed radio-derived distance, and absolute distance error, while the network panel shows RSSI with PER on the right axis.
- Per-run combined figures only for debugging: distance plus RSSI/PER; duplex figures use `link_distance_m` as the displayed `$d_r$` source.
- Figures are generated without in-plot titles; use LaTeX captions instead.
- Latency, SNIR, PDR, and jitter stay in `network_metrics_summary.csv`, but are usually not plotted because they add little separation in the current runs.

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

- `network_metrics.csv` from offline replay
- `/omnet/rssi_dbm`
- `/omnet/snir_db`
- `/omnet/packet_error_rate`
- `/omnet/packet_delivery_ratio`
- `/omnet/latency_s`
- `/omnet/jitter_s`
- `/omnet/radio_distance`
- `/omnet/sim_time`
- `link_distance_m` from offline replay CSV when OMNeT exports it

From OMNeT files:

- Packets sent/received.
- PDR / delivery ratio.
- RSSI over time.
- PER, especially for distance sweep runs.
- Queue or data-rate metrics if available.

## Useful Commands

Inspect live OMNeT metrics:

```bash
./run.sh omnet_monitor once:=true
```

Run offline OMNeT replay for one selected C3 run:

```bash
./run.sh omnet_bag_replay \
  run_dir:=bags/ruben_c1_c4_selected_rosbags_2026-05-25/C3/C3_RouteA_rotundan_r01__old_selected_r01 \
  network:=lora \
  lora_sf:=7 \
  lora_bw:=125kHz \
  ugv_topic:=/a201_0000/platform/odom/filtered \
  uav_topic:=/dji0/pose \
  replay_scale:=3
```

Run simplex and duplex replay for all C3 selected bags:

```bash
./run.sh omnet_bag_replay_batch \
  root:=bags/ruben_c1_c4_selected_rosbags_2026-05-25 \
  conditions:=C3 \
  networks:=lora,lora-duplex \
  replay_scale:=3
```

Run leader-link replay for C4 selected bags:

```bash
./run.sh omnet_bag_replay_batch \
  root:=bags/ruben_c1_c4_selected_rosbags_2026-05-25 \
  conditions:=C4 \
  networks:=lora-duplex \
  replay_scale:=3 \
  sample_stride:=2
```

Plot offline replay metrics:

```bash
./run.sh plot_network_metrics \
  --results-dir bags/ruben_c1_c4_selected_rosbags_2026-05-25/C3 \
  --offline-omnet \
  --overview-set thesis
```

```bash
./run.sh plot_network_metrics \
  --results-dir bags/ruben_c1_c4_selected_rosbags_2026-05-25/C4 \
  --offline-omnet \
  --networks lora-duplex \
  --overview-set thesis
```

Generate only the summary CSV and best/worst metric tables:

```bash
./run.sh plot_network_metrics --results-dir bags/ruben_c1_c4_selected_rosbags_2026-05-25 --offline-omnet --summary-only --rank-limit 5
```

## Run Notes

### C2 Live Collection

C2 uses manual `tmux_1to1` route launches now. The older `nav2_route_sweep` commands are not the primary path because manual runs were more reliable.

Routes currently used for C2 network work:

```text
R01 rotundan
R02 road_to_west
R03 parkinglot_west
R04 road_to_spawn
R05 spawn
R06 road_to_east
R07 parkinglot_east
```

C2 repetition mapping:

- `rep01`: simplex, fixed geometry.
- `rep02`: duplex, fixed geometry.
- `rep03`: simplex, manual distance sweep.

In thesis figures, C2 is labeled by condition only: `simplex`, `duplex`, and `distance sweep`.

Manual distance sweep command after rep03 launch:

```bash
for d in 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100; do z=7; xy=$(python3 -c "import math; d=$d; z=min($z,d); print(math.sqrt(max(0,d*d-z*z))+2)"); ros2 param set /follow_uav xy_anchor_max $xy; ros2 param set /follow_uav d_target $d; ros2 param set /follow_uav follow_z_offset_m $z; sleep 10; done
```

Use `plot_trajectory_paths.py --plane xz` for rep03 height/distance inspection.
Trajectory plots write under `<results-dir>/plots/trajectory/`: averaged route sheets go under `combined/`, and non-averaged repetition sheets go under `by_rep/`.

Plot one averaged C2 network time-series figure per repetition/condition:

```bash
./run.sh plot_network_metrics \
  --results-dir bags/results_baylands_lora_c2 \
  --lora-mode rep-map \
  --rep-average-plots
```

This writes only the three averaged C2 LoRa-network figures plus `network_metrics_summary.csv`. The averaged network distance panel uses compact thesis notation: true 3D distance $d$, displayed radio-derived distance $d_r$, and absolute radio error $\Delta d = |d_r-d|$. For simplex and distance-sweep plots, `$d_r$` comes from `radio_distance_m`; for duplex plots, `$d_r$` comes from `link_distance_m`. The distance-sweep (`rep03`) figure also shows height delta $\Delta z$; horizontal distance and leader-localization error are omitted from these network figures to keep the thesis plots focused. Distance series use solid coloured lines and delta series dotted lines. Figures default to the thesis text width (312 pt, about 4.32 in), can use LaTeX text rendering with the thesis `newtx` font packages via `--usetex`, and place the distance legend to the right of the top panel; `--width` remains available as an override. Generated artifact paths are printed without prefixes for direct copy/paste.

```bash
d=15 z=10 xy=10

ros2 param set /follow_uav xy_anchor_max $xy 
ros2 param set /follow_uav d_target $d 
ros2 param set /follow_uav follow_z_offset_m $z
```

### C3 Offline OMNeT

C3 is the visual bridge pipeline. The selected thesis bags live under:

```text
bags/ruben_c1_c4_selected_rosbags_2026-05-25/C3
```

Most C3 selected bags do not contain `/a201_0000/ground_truth/odom`; use `/a201_0000/platform/odom/filtered` when replaying. `plot_network_metrics.py --offline-omnet` falls back automatically when plotting.

### C4 Offline OMNeT

C4 selected bags include `dji0`, `dji1`, and `dji2` poses. The current offline OMNeT replay is valid for the leader link only:

```text
UGV <-> dji0
```

It is not yet a full three-UAV ad hoc replay. A true C4 ad hoc analysis needs OMNeT/replay support for `dji1` and `dji2` as additional radio nodes.

## Assumptions

- Use native FLORA LoRa via `omnet_network:=lora`.
- Use `omnet_network:=lora-duplex` only for the allowed feedback comparison where UGV-to-UAV communication is part of the scenario.
- Keep the current Baylands follow height/defaults unless the run notes say otherwise.
- Exclude the first 30 seconds from plots and summary statistics.
- In simplex and distance-sweep plots, `$d_r$` is read from `radio_distance_m`, which is RSSI/path-loss derived.
- In duplex plots, `$d_r$` is read from `link_distance_m`; keep the raw CSV column distinction clear in thesis text.
- Live collection still uses normal tmux scripts. Offline OMNeT replay from saved bags uses `./run.sh omnet_bag_replay`.
- Offline replay batch collection uses `./run.sh omnet_bag_replay_batch`.
- Offline replay plotting uses `./run.sh plot_network_metrics --offline-omnet`; default output is one campaign-level thesis figure, `network_metrics_summary.csv`, and `network_metrics_extremes.md`.
- `replay_scale:=4` has been spot-tested successfully; use `replay_scale:=3` as the safer full-folder batch default unless time pressure is high.

## Campaigns To-Do

- ~~C1: Raw follow, only odom.~~
- C2: Finish rep03 distance-sweep routes and plot rep-separated network metrics.
- C3: Run offline simplex and duplex OMNeT replay for selected C3 bags.
- C4: Run offline leader-link OMNeT replay for selected C4 bags; full support-UAV ad hoc is future work unless OMNeT topology is extended.

## Current State

As of 2026-05-25:

- Primary C1 result root: `bags/results_baylands_lora_c1`.
- Primary live C2 result root: `bags/results_baylands_lora_c2`.
- Selected thesis bag root for offline C3/C4 network replay: `bags/ruben_c1_c4_selected_rosbags_2026-05-25`.
- `run_omnet_bag_replay.sh` now cleans up helper processes, applies wall-time timeout, and supports `replay_scale`.
- `run_omnet_bag_replay_batch.sh` discovers C3/C4 selected bags and records an offline replay manifest.
- `plot_network_metrics.py` supports `--offline-omnet`, `--overview-set thesis|all|both`, optional `--per-run-plots`, and `--summary-only` for CSV/table output.

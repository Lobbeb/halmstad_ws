# LoRa Settings and Distance Sweep Plan

Purpose: collect controlled LoRa network results across radio settings and UAV/UGV separation distances.

This plan is for thesis data collection after the basic Baylands route/recording stack is stable.

## Questions

Measure:

- Which LoRa settings keep acceptable PDR/PER at useful observation distances?
- How latency and jitter change with SF/BW and distance.
- How RSSI/SNIR and `/omnet/radio_distance` change against offline geometric distance.
- Whether the ROS2 TCP bridge preserves useful timing between Gazebo `/clock` and `/omnet/sim_time`.

Do not use geometric distance as a live tracking input. It is only for offline validation.

## Controlled Variables

Keep these fixed unless the run label says otherwise:

```text
world: baylands
mode: follow
record: true
record_profile: default
omnet: true
omnet_network: lora
omnet_ui: cmdenv
LoRa CF: 869.525 MHz
LoRa TP: 27 dBm
LoRa CR: 4
LoRa payload: 32 B
LoRa duty cycle: 10%
analysis warmup: 30 s
```

Current runnable LoRa args:

```text
lora_sf:=7..12
lora_bw:=125kHz|250kHz|500kHz
```

CR and TX power are currently fixed in `omnetpp.ini` for coupled Gazebo runs. Add runnable args for those only after SF/BW results show that they are worth sweeping.

## Metrics

Record and summarize:

```text
/omnet/packet_delivery_ratio
/omnet/packet_error_rate
/omnet/latency_s
/omnet/jitter_s
/omnet/rssi_dbm
/omnet/snir_db
/omnet/radio_distance
/omnet/sim_time
```

Offline validation metrics:

```text
true geometric UAV/UGV distance
radio_distance - true_distance
omnet_sim_time - gazebo_clock
OMNeT topic update interval
```

## Phase 0: Smoke Test

Run one short route with the default profile.

```bash
cd /home/ruben/halmstad_ws

./run.sh nav2_route_sweep baylands \
  routes:=rotundan \
  repetitions:=1 \
  route_sim_timeout_s:=60 \
  campaign_tag:=lora_smoke_sf8_bw125 \
  record:=true \
  record_profile:=default \
  record_root:=bags/results_lora_smoke \
  omnet:=true \
  omnet_network:=lora \
  omnet_ui:=cmdenv \
  lora_sf:=8 \
  lora_bw:=125kHz \
  gui:=false \
  mode:=follow \
  lidar:=3d
```

Pass criteria:

```text
OMNeT does not crash
/omnet/* topics are recorded
PDR stays within 0..1
RSSI/SNIR/radio_distance update after LoRa packets arrive
summary file is written
```

## Phase 1: Distance Sweep at Default LoRa

Use the default thesis profile first:

```text
SF8, BW125
```

Target distances:

```text
10 m
25 m
50 m
75 m
100 m
150 m
200 m
```

Optional stretch distances if the run is stable:

```text
300 m
500 m
```

Recommended method:

1. Start a recorded `follow` run with `lora_sf:=8 lora_bw:=125kHz`.
2. Let the system stabilize for at least 30 s.
3. Change `d_target` at runtime.
4. Hold each distance for at least 90-120 s.
5. Record notes with the exact time each distance change was applied.

Manual distance change:

```bash
ros2 param set /follow_uav d_target 50.0
ros2 param set /follow_uav follow_z_offset_m 7.0
```

Keyboard helper:

```bash
./run.sh run_follow_control --mode keyboard --step-d-target 5 --max-d-target 500
```

Use manual fixed-distance holds for the first dataset. Random/scripted changes are useful later, but fixed holds are easier to analyze.

## Phase 2: SF/BW Sweep at Representative Distances

Do not start with every possible combination. Use representative distances first:

```text
25 m
75 m
150 m
```

Primary setting matrix:

| Label | SF | BW |
| --- | ---: | --- |
| fast_1 | 7 | 125kHz |
| default | 8 | 125kHz |
| mid_1 | 9 | 125kHz |
| robust_1 | 10 | 125kHz |
| robust_fast | 10 | 250kHz |
| robust_2 | 11 | 250kHz |
| max_robust | 12 | 250kHz |

For each setting:

1. Start one run.
2. Hold 25 m, 75 m, and 150 m.
3. Keep each hold long enough for several LoRa packets.
4. Stop the run and move to the next setting.

Example run start:

```bash
./run.sh tmux_1to1 baylands \
  session:=lora_sf10_bw250_dists \
  mode:=follow \
  waypoint:=rotundan_0 \
  nav2_goals:=rotundan \
  record:=true \
  record_profile:=default \
  record_tag:=lora_sf10_bw250_dists \
  record_out:=bags/results_lora_distance_sweep/sf10_bw250 \
  omnet:=true \
  omnet_network:=lora \
  omnet_ui:=cmdenv \
  lora_sf:=10 \
  lora_bw:=250kHz \
  gui:=false \
  lidar:=3d
```

Then set holds:

```bash
ros2 param set /follow_uav d_target 25.0
sleep 120
ros2 param set /follow_uav d_target 75.0
sleep 120
ros2 param set /follow_uav d_target 150.0
sleep 120
```

Stop:

```bash
./stop.sh tmux_1to1 baylands session:=lora_sf10_bw250_dists
```

## Phase 3: Full SF/BW Sweep if Time Allows

Only run this after Phase 2 identifies promising settings.

Full matrix:

```text
SF: 7, 8, 9, 10, 11, 12
BW: 125kHz, 250kHz
Distances: 25 m, 75 m, 150 m
Repetitions: 2
```

This is:

```text
6 * 2 * 3 * 2 = 72 distance holds
```

At 120 s per hold, this is about 2.4 hours of hold time, not counting startup/shutdown.

## Phase 4: Duplex Comparison

Run duplex only on the best 1-2 simplex settings.

Use:

```text
omnet_network:=lora-duplex
```

Compare:

```text
simplex SF8/BW125 vs duplex SF8/BW125
simplex best robust profile vs duplex same profile
```

Focus on whether duplex improves tracking/network behavior enough to justify extra traffic and possible duty-cycle pressure.

## Analysis Outputs

Per run:

```text
plots/trajectory.png
plots/trajectory.pdf
plots/omnet_metrics.png
plots/omnet_metrics.pdf
plots/omnet_metrics.csv
```

Required validation plots:

```text
true_distance vs radio_distance
radio_distance error over time
PDR/PER over time
latency/jitter over time
RSSI/SNIR over time
OMNeT sim_time vs Gazebo /clock
OMNeT metric update interval histogram
```

Campaign summary:

```text
mean PDR per SF/BW/distance
mean PER per SF/BW/distance
mean latency_ms per SF/BW/distance
mean jitter_ms per SF/BW/distance
RSSI/SNIR distribution per SF/BW/distance
radio_distance error distribution per SF/BW/distance
```

## Run Naming

Use labels that include the controlled variables:

```text
lora_sf8_bw125_d025
lora_sf8_bw125_d075
lora_sf10_bw250_d150
```

For multi-distance runs:

```text
lora_sf8_bw125_dists_025_075_150
```

## Notes

- The first 30 s are warmup and should not count in thesis summaries.
- Keep true geometric distance only in offline plots/analysis.
- `/omnet/radio_distance` is a network-derived estimate and can be compared with true distance.
- Do not change SF/BW mid-run unless the run is explicitly testing dynamic reconfiguration. For clean results, restart OMNeT for each LoRa setting.
- Distance can change mid-run through `d_target`; record the change times.

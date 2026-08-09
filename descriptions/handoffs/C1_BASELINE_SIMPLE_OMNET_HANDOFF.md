# C1 Baylands Baseline Handoff for OMNeT++ Communication Runs

This is the simple setup Ruben should mirror for communication/OMNeT++ runs.

## Purpose

Use the same Baylands baseline setup as William's C1 Results batch, then add the communication/OMNeT++ condition on top.

C1 is the odometry/AMCL-driven baseline:

- No YOLO detector.
- No visual bridge.
- No support-chain.
- No OMNeT++ impairment in the original C1 batch.
- Used as the stable reference condition for later comparison.

## Fixed Setup

- World: `baylands`
- Condition: `C1`
- Mode: `follow`
- GUI: `false`
- Duration per run: `300 s`
- Warmup excluded from analysis: `30 s`
- Output used by William: `bags/results_c1_batch01`
- Raw bags: `bags/results_c1_batch01/C1_odom/`
- CSV summaries: `bags/results_c1_batch01/export/`

The UAV baseline was run with the fixed C1 follow configuration. Height was fixed at `7 m` and was not varied as an experimental factor.

## Route Schedule

William's C1 batch used 15 runs:

- 5 Baylands routes
- 3 repetitions per route

Use the same route order if you want direct comparison:

| Run | Route label | `nav2_goals` | `waypoint` |
| --- | --- | --- | --- |
| r01 | Route A | `rotundan` | `rotundan_0` |
| r02 | Route B | `road_to_west` | `road_to_west_0` |
| r03 | Route C | `road_to_spawn` | `road_to_spawn_0` |
| r04 | Route D | `spawn` | `spawn_0` |
| r05 | Route E | `parkinglot_west` | `parkinglot_west_0` |
| r06 | Route A | `rotundan` | `rotundan_0` |
| r07 | Route B | `road_to_west` | `road_to_west_0` |
| r08 | Route C | `road_to_spawn` | `road_to_spawn_0` |
| r09 | Route D | `spawn` | `spawn_0` |
| r10 | Route E | `parkinglot_west` | `parkinglot_west_0` |
| r11 | Route A | `rotundan` | `rotundan_0` |
| r12 | Route B | `road_to_west` | `road_to_west_0` |
| r13 | Route C | `road_to_spawn` | `road_to_spawn_0` |
| r14 | Route D | `spawn` | `spawn_0` |
| r15 | Route E | `parkinglot_west` | `parkinglot_west_0` |

## Command Shape Used for C1

William ran the batch with the Results harness using Baylands only:

```bash
./run.sh results_campaign \
  --condition C1 \
  --runs 15 \
  --duration 300 \
  --warmup 30 \
  --world baylands \
  --gui false \
  --out bags/results_c1_batch01 \
  --route-schedule rotundan,road_to_west,road_to_spawn,spawn,parkinglot_west
```

Summary command:

```bash
python3 scripts/results_summarize_bag.py bags/results_c1_batch01/C1_odom --warmup 30
```

## What Ruben Should Keep the Same

For communication/OMNeT++ comparison, keep these identical:

- `world=baylands`
- same 5 routes
- same route order
- same start waypoints
- same `300 s` duration
- same `30 s` warmup removal
- same route labels, Route A-E
- same output structure, but use a new folder for communication results

Suggested communication output folder:

```text
bags/results_omnet_c1_like/
```

or one folder per impairment profile:

```text
bags/results_omnet/profile_<name>/
```

## What Ruben Should Add

For OMNeT++/communication results, add the communication condition labels and record communication topics/metrics, for example:

- OMNeT++ profile name
- impairment setting
- RSSI
- SNIR
- packet error rate
- delivery ratio
- communication latency/jitter
- route label
- run ID

The important part is that the route schedule and baseline timing stay the same, so the communication results can be compared against William's C1 baseline.

## Source of Truth

The raw MCAP bags are the source of truth:

```text
bags/results_c1_batch01/C1_odom/
```

The CSVs are derived summaries for inspection and thesis plotting:

```text
bags/results_c1_batch01/export/c1_batch01_runs.csv
bags/results_c1_batch01/export/c1_batch01_aggregate.csv
bags/results_c1_batch01/export/c1_batch01_by_route.csv
```


# Ruben Results Bag Handover

Date: 2026-05-25

This is the source-of-truth handover for the ROS 2 bags and locked result packages used for the thesis C1, C2, C3, and C4 figures/results. Use these exact folders when adding the communication/OMNeT++ part. If different bags are used, the plots and route statistics can change.

## Short version: exact selected ROS bags to send

Ruben asked for the ROS bags used for the C1-C4 thesis figures/results. The selected-only package has now been staged here:

```text
docs/handovers/ruben_c1_c4_selected_rosbags_2026-05-25/
docs/handovers/ruben_c1_c4_selected_rosbags_2026-05-25.zip
```

This is the package to send first. It contains only the selected runs used in the thesis plots/results, not every rerun or broad source batch.

Selected-only contents:

| Condition | Selected bag runs | Staged size before zip |
|---|---:|---:|
| C1 | 24 | 0.434 GB |
| C2 | 24 | 0.480 GB |
| C3 | 38 | 1.151 GB |
| C4 | 40 | 0.958 GB |

Total selected set: 126 ROS bag run folders. The zip is about 0.70 GB on this machine.

The staged folder includes:

```text
README.md
selected_rosbags_manifest.csv
selected_rosbags_summary.csv
C1/
C2/
C3/
C4/
```

Each staged run folder preserves the original run folder contents, including its `bag/` directory.

## Source folders used to build the selected package

These are the broader source folders from which the selected runs were copied:

```text
bags/results_c1_batch01/C1_odom/
bags/results_c1_added_routes_batch01/C1_odom/
bags/results_c1_parkinglot_east_cleanup/C1_odom/
bags/results_c2_batch01/C2_direct/
bags/results_c2_batch01_weakroute_rerun_v4_3/C2_direct/
bags/results_c2_added_routes_batch01/C2_direct/
bags/results_c3_final_8routes/C3_bridge/
bags/results_c3_rerun_routes_cdh_batch01/C3_bridge/
bags/results_c3_rerun_routes_dh_extra_batch01/C3_bridge/
bags/results_c4_final_8routes_cpu/C4_support/
```

Do not send the whole broad source folders unless Ruben explicitly wants all reruns. They contain extra runs that were not used in the final figures.

## Result packages and context to send too

Also send Ruben these folders/files so he can reproduce which bags were selected and how the thesis figures/tables were generated:

```text
bags/results_c1_final_8routes_matched/
bags/results_c2_final_8routes/
bags/results_c3_final_8routes_cdh_updated/
bags/results_c4_final_8routes_cpu/
MasterThesis/gfx/Plots/sim_final_comparison/
MasterThesis/gfx/Plots/sim_final_c1_c3_comparison_clean_v3/
MasterThesis/gfx/Plots/Sim_c4/modern/
docs/handovers/For_Ruben_c3.md
docs/handovers/ruben_results_bags_handover.md
models/obb/mymodels/baylands-leader-v4-3.pt
```

Also keep the raw bag source folders listed below. Some final packages are clean result packages and point back to raw bags in the original batch folders.

## Locked thesis result packages

| Condition | Thesis meaning | Locked package | Valid runs used |
|---|---|---|---:|
| C1 | Odometry baseline | `bags/results_c1_final_8routes_matched/` | 24 |
| C2 | Direct visual estimate-follow | `bags/results_c2_final_8routes/` | 24 |
| C3 | Full visual-bridge pipeline | `bags/results_c3_final_8routes_cdh_updated/` | 38 |
| C4 | Support-chain observation/forwarding | `bags/results_c4_final_8routes_cpu/` | 40 |

Validation reference:

```text
MasterThesis/gfx/Plots/sim_final_comparison/SOURCE_PACKAGE_AUDIT.md
MasterThesis/gfx/Plots/sim_final_comparison/csv/sim_conditions_validity_summary.csv
```

Current validity summary:

```text
C1: 8 routes, 24 selected/valid runs, 0 invalid
C2: 8 routes, 24 selected/valid runs, 0 invalid
C3: 8 routes, 38 selected/valid runs, 0 invalid
C4: 8 routes, 42 attempted runs, 40 valid runs, 2 invalid attempts
```

For C4, the two invalid attempts are Route G / `road_to_east`: `r15` with `record_start_timeout`, and `r39` with `stop_failed`. The final valid C4 dataset still has five valid runs per route.

## Raw ROS bag source folders

### C1 raw bag sources

The locked C1 package is:

```text
bags/results_c1_final_8routes_matched/
```

It was built from these raw C1 sources:

```text
bags/results_c1_batch01/C1_odom/
bags/results_c1_added_routes_batch01/C1_odom/
bags/results_c1_parkinglot_east_cleanup/C1_odom/
```

The C1 builder records these source roots in:

```text
bags/results_c1_final_8routes_matched/build_c1_final_8routes_matched.py
bags/results_c1_final_8routes_matched/csv/c1_8route_runs.csv
bags/results_c1_final_8routes_matched/notes/c1_8route_matched_manifest.json
```

Important: do not use older C1 folders such as `results_c1_final_8routes`, `results_c1_final_8routes_corrected`, or `results_c1_final_8routes_final` as the thesis source unless intentionally redoing the C1 figures.

### C2 raw bag sources

The locked C2 package is:

```text
bags/results_c2_final_8routes/
```

It points back to these raw C2 sources:

```text
bags/results_c2_batch01/C2_direct/
bags/results_c2_batch01_weakroute_rerun_v4_3/C2_direct/
bags/results_c2_added_routes_batch01/C2_direct/
```

Selection reference:

```text
bags/results_c2_final_8routes/csv/c2_final_route_run_selection.csv
bags/results_c2_final_8routes/raw_result_pointers/README_RAW_RESULTS.md
```

C2 uses direct estimate-follow:

```text
detector/tracker -> leader_estimator -> follow_uav.py -> UAV command
```

### C3 raw bag sources

The locked C3 package is:

```text
bags/results_c3_final_8routes_cdh_updated/
```

It combines accepted old C3 runs and targeted replacement runs:

```text
bags/results_c3_final_8routes/C3_bridge/
bags/results_c3_rerun_routes_cdh_batch01/C3_bridge/
bags/results_c3_rerun_routes_dh_extra_batch01/C3_bridge/
```

Selection reference:

```text
bags/results_c3_final_8routes_cdh_updated/csv/c3_selected_run_pointers_final.csv
bags/results_c3_final_8routes_cdh_updated/csv/c3_selected_runs_final_cdh_updated.csv
bags/results_c3_final_8routes_cdh_updated/csv/c3_final_route_means.csv
```

C3 uses the full visual bridge chain:

```text
detector/tracker
-> leader_estimator
-> selected_target_filter
-> visual_target_estimator
-> follow_point_generator
-> follow_point_planner
-> visual_actuation_bridge
-> UAV command
```

### C4 raw bag source

The locked C4 package and raw C4 runs are in:

```text
bags/results_c4_final_8routes_cpu/C4_support/
```

Inventory and modern thesis output references:

```text
MasterThesis/gfx/Plots/Sim_c4/chatgpt_redesign_data/c4_run_inventory.csv
MasterThesis/gfx/Plots/Sim_c4/modern/notes/C4_MODERN_PACKAGE_NOTES.md
MasterThesis/gfx/Plots/Sim_c4/modern/tables/main/
MasterThesis/gfx/Plots/Sim_c4/modern/figures/main/
```

C4 validates support-chain information flow:

```text
dji1 support observation
dji2 support observation
-> dji0 support mux / aggregation
-> UGV support summary
-> UGV awareness status + path advisory
```

Important: C4 is monitor/advisory only. It does not perform active UGV replanning, costmap updates, or obstacle avoidance in the current implementation.

## Route mapping

Use the same route labels everywhere:

| Label | Route name | Waypoint |
|---|---|---|
| Route A | `rotundan` | `rotundan_0` |
| Route B | `road_to_west` | `road_to_west_0` |
| Route C | `road_to_spawn` | `road_to_spawn_0` |
| Route D | `spawn` | `spawn_0` |
| Route E | `parkinglot_west` | `parkinglot_west_0` |
| Route F | `parkinglot_east` | `parkinglot_east_0` |
| Route G | `road_to_east` | `road_to_east_0` |
| Route H | `strip` | `strip_0` |

## Detector/model settings to preserve

For C2, C3, and C4 support detector runs, the thesis-side model reference is:

```text
models/obb/mymodels/baylands-leader-v4-3.pt
```

Key C2/C3 settings:

```text
world: baylands
duration: 300 s
warmup: 30 s
detector_backend: ultralytics
tracker: true
external_detection_node: tracker
detector_conf_threshold: 0.11
detector_iou_threshold: 0.3
```

C3-specific:

```text
yolo_control_mode: visual_bridge
visual_follow_logic: follow_core
visual_reacquire_assist_enable: true
visual_reacquire_stale_timeout_s: 2.0
visual_reacquire_return_fresh_s: 1.0
```

C4-specific:

```text
mode: follow
record_profile: support
support_detector_backend: ultralytics
support_yolo_weights: /home/william/halmstad_ws/models/obb/mymodels/baylands-leader-v4-3.pt
support_camera_scan_enable: true
support_bridge_gimbal: true
support_mux_relation_source: odom
support_mux_source_stale_timeout_s: 4.0
```

## Thesis plot/output folders

For the exact figures/tables used or intended for the thesis, use:

```text
MasterThesis/gfx/Plots/sim_final_comparison/
MasterThesis/gfx/Plots/sim_final_c1_c3_comparison_clean_v3/
MasterThesis/gfx/Plots/Sim_c4/modern/
```

Key C1-C3 validation file:

```text
MasterThesis/gfx/Plots/sim_final_c1_c3_comparison_clean_v3/C1_C3_FINAL_COMPARISON_DATA_VALIDATION.md
```

Key C4 interpretation file:

```text
MasterThesis/gfx/Plots/Sim_c4/modern/notes/C4_MODERN_PACKAGE_NOTES.md
```

## Communication/OMNeT note for Ruben

William's C4 result package does not include OMNeT++ or radio metrics. It is a support-chain observation/forwarding package only.

If Ruben adds communication plots, he should keep the C1-C4 bag sources above fixed and add the communication condition/output separately, or clearly label it as an added OMNeT/communication dataset. Otherwise the communication plots may not match the existing thesis result figures.

Current repo convention separates OMNeT/communication as an optional C5-style path in the campaign scripts, not as part of the locked C4 thesis dataset.

## Do not accidentally use

Avoid using these as replacements for the locked thesis sources unless the thesis figures are intentionally being regenerated:

```text
bags/results_smoke/
bags/results/
bags/results_c4_readiness_smoke*/
bags/c3_smoke_trajectory_previews/
older C1 intermediate packages:
  bags/results_c1_final_8routes/
  bags/results_c1_final_8routes_corrected/
  bags/results_c1_final_8routes_final/
```

The locked folders at the top of this file are the ones that keep the thesis plots stable.

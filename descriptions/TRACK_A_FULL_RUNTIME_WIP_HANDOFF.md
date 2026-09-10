# Track A full-runtime WIP handoff

## Checkpoint state

- Branch: `support-chain`.
- The lightweight planner-only Track A suite passed and was checkpointed at
  `4d1953fc2403e9e3601a6362ad843b255f741a92`.
- The current working state extends that checkpoint with the bounded Baylands
  full-runtime harness, recording/evidence support, tests, and documentation.
- Full-runtime Track A validation has **not** passed. Do not infer physical
  detour, runtime replanning, goal completion, or clearing from the lightweight
  result.

## Latest full-runtime result

The latest baseline run is under the ignored directory
`evidence/support_runtime/final_v5/baseline/` on the original workstation.
It completed the Nav2 action with `SUCCEEDED`, but the evidence analyzer marked
the scenario `FAIL` because the reported final UGV pose was outside the runtime
goal tolerance.

Recorded facts from that run:

- Requested goal: map-frame pose near `(-72.5316, 185.3861)`, yaw `0.00662`.
- Runtime goal checker: `nav2_controller::SimpleGoalChecker`, XY tolerance
  `1.0 m`, yaw tolerance `2.5 rad`, stateful.
- Action duration: about `55.1 s`; no recoveries were reported.
- Last action-feedback pose before success was about `1.135 m` from the goal;
  its `distance_remaining` was about `1.174 m`.
- Last AMCL pose before success was about `1.274 m` from the goal.
- No hazard or aerial-layer marking was active in this baseline.
- AMCL repeatedly warned that more than 95 percent of observations were not in
  the map and that the particle filter might have converged to the wrong pose.

## Unresolved evidence defect

The analyzer's current TF-derived final pose is not yet authoritative. Its
map-to-base composition uses the most recently received transforms without
interpolating both links to a common timestamp. In the latest run, the received
`map -> odom` transform could be stamped later than the paired
`odom -> base_link` transform. Fix or explicitly constrain this common-time TF
selection before using the TF result to decide goal-tolerance acceptance.

After correcting that evidence logic, rerun the bounded baseline and compare
the action result, action feedback, AMCL pose, and common-time TF pose. Do not
change Nav2 tolerances merely to make the gate pass. Diagnose the mismatch and
the AMCL warning first.

## Runtime compatibility note

The legacy `gazebo_model_pose_bridge` cannot currently import the Python
`gz.transport13` module in the installed Jazzy environment. ROS/Gazebo C++
transport packages and `ros_gz_bridge` are present, but that Python binding is
not. This bridge is for simulation/debug ground truth used by legacy support;
it must not become an operational navigation or UAV-tracking input. Decide
whether to provide a supported debug-only bridge or disable that optional path
cleanly after the Nav2 evidence mismatch is understood.

## Remaining validation

1. Correct the common-time TF evidence calculation and rerun the baseline.
2. Resolve or explain the Nav2-success versus measured-pose tolerance mismatch.
3. Run the full-runtime valid-hazard scenario and prove typed propagation,
   aerial costmap marking, runtime replanning, physical detour, and goal result.
4. Run the full-runtime clearing scenario and prove TTL/removal plus continued
   or recovered navigation.
5. Preserve task-owned process cleanup only. Do not use broad ROS, Gazebo, or
   tmux cleanup commands.

Generated evidence, `build/`, `install/`, and `log/` remain local/ignored and
are not part of this checkpoint.

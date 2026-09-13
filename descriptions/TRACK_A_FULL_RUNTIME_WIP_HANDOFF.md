# Track A full-runtime WIP handoff

## Checkpoint state

- Branch: `support-chain`.
- The lightweight planner-only Track A suite passed and was checkpointed at
  `4d1953fc2403e9e3601a6362ad843b255f741a92`.
- The current working state extends that checkpoint with the bounded Baylands
  full-runtime harness, recording/evidence support, tests, and documentation.
- Success-time TF resolution was corrected and checkpointed locally at
  `e76b99e` (`Track A: fix success-time TF evidence resolution`).
- Full-runtime Track A validation has **not** passed. Do not infer physical
  detour, runtime replanning, goal completion, or clearing from the lightweight
  result.

## Latest full-runtime results

The 2026-09-10 headless laptop attempt is preserved under
`evidence/support_runtime/final_v6/baseline/`. It is **INCONCLUSIVE** because
Gazebo reached only about 0.003 real-time factor under high host load before
Nav2 or NavigateToPose became ready. Do not retry the full three-UAV profile on
this laptop or tune Nav2 to compensate. Run the authoritative sequence on the
desktop.

The preceding desktop baseline is under the ignored directory
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

## Success and motion evidence

The analyzer now resolves success-time `map -> base_link` at the latest bounded
common source timestamp using exact samples or interpolation. It excludes TF
received after success and rejects stale, unbracketed, or invalid histories.

Goal-checker evidence distinguishes success inside XY tolerance, a stateful XY
latch proven by prior map-frame NavigateToPose feedback, success without proven
XY entry, and insufficient pose evidence. Runtime replanning additionally
requires one unambiguous goal lifetime, a mark before a materially changed plan,
and zero evidence-side ComputePath requests. Physical detour requires an actual
post-replan departure from the baseline trajectory. Explicit clearing requires
ordered dji1, dji0, and UGV empty snapshots before exact costmap restoration.

## Runtime compatibility note

The legacy `gazebo_model_pose_bridge` cannot currently import the Python
`gz.transport13` module in the installed Jazzy environment. ROS/Gazebo C++
transport packages and `ros_gz_bridge` are present, but that Python binding is
not. This bridge is debug/legacy ground truth and is not part of Nav2, the typed
hazard chain, or AerialSupportLayer. Track A runtime profiles explicitly disable
it; generic C1-C4 defaults remain unchanged.

## Reduced-resource diagnostic

`reduced_resource:=true` is a clearly labelled, non-authoritative Baylands
diagnostic. It keeps the real UGV/localization/Nav2/controller and the same
synthetic dji1 -> dji0 fusion -> UGV forwarding -> AerialSupportLayer path, but
omits UAV spawning, cameras, YOLO, gimbals, support follow, and ground-truth
bridging. It does not replace the final desktop three-UAV sequence.

```bash
./run.sh support_chain_full_runtime scenario:=baseline \
  output:=evidence/support_runtime/final_v6/reduced_resource/baseline \
  reduced_resource:=true gui:=false
```

The single 2026-09-12 laptop attempt is preserved at that path and is also
**INCONCLUSIVE**. Gazebo advanced `0.14 s` in `32.46 s` wall time
(`RTF 0.00505`) despite `6.7 GiB` available memory and no UAV workload. Nav2
did not become ready, so the task-owned session was stopped and reduced valid
and clearing were not run. See `environment_limit.json` beside the logs. This
confirms the laptop limitation is the Baylands UGV/Gazebo runtime itself, not
the removed support-UAV perception workload.

## Desktop authoritative continuation

Run each scenario to analyzer completion, inspect its `track_a_evidence` pane,
then stop only its named session before starting the next command.

```bash
./run.sh support_chain_full_runtime scenario:=baseline \
  output:=evidence/support_runtime/final_v6/desktop/baseline
./stop.sh tmux_support_chain baylands \
  session:=halmstad-baylands-track-a-baseline

./run.sh support_chain_full_runtime scenario:=valid \
  output:=evidence/support_runtime/final_v6/desktop/valid \
  baseline_evidence:=evidence/support_runtime/final_v6/desktop/baseline/analysis
./stop.sh tmux_support_chain baylands \
  session:=halmstad-baylands-track-a-valid

./run.sh support_chain_full_runtime scenario:=clearing \
  output:=evidence/support_runtime/final_v6/desktop/clearing \
  baseline_evidence:=evidence/support_runtime/final_v6/desktop/baseline/analysis
./stop.sh tmux_support_chain baylands \
  session:=halmstad-baylands-track-a-clearing
```

## Remaining validation

1. Run the authoritative desktop baseline with the corrected evidence and
   resolve the Nav2-success versus measured-pose discrepancy.
2. Run the full-runtime valid-hazard scenario and prove typed propagation,
   aerial costmap marking, runtime replanning, physical detour, and goal result.
3. Run the full-runtime clearing scenario and prove explicit removal plus continued
   or recovered navigation.
4. Preserve task-owned process cleanup only. Do not use broad ROS, Gazebo, or
   tmux cleanup commands.

Generated evidence, `build/`, `install/`, and `log/` remain local/ignored and
are not part of this checkpoint.

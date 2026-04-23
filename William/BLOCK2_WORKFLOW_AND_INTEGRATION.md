# Block 2 Workflow And Integration Handoff

## Purpose

This file is the quick handoff for future chats and future Codex sessions working in this workspace. It records the practical simulation workflow after work blocks 1 and 2, so the corrected follow baseline is used as the real experiment path and not treated as a side experiment.

Use the thesis execution markdown as the operational plan. This file only summarizes the current implementation state and workflow.

## Current Block Status

Work block 1 is complete enough to treat as the corrected follow baseline.

Work block 2 is the current integration baseline. The corrected follow logic is wired into the practical simulation path used by the run scripts, launch files, shared YAML defaults, and support overlay.

Do not reopen controller design, broad follow cleanup, or support-chain feature work when the goal is only to run or verify the current system.

## What Block 1 Accomplished

Block 1 ported and adapted the corrected follow behavior into the live simulation repo.

Main live files changed by block 1:

- `src/lrs_halmstad/lrs_halmstad/follow/follow_math.py`
- `src/lrs_halmstad/lrs_halmstad/follow/follow_uav_odom.py`
- `src/lrs_halmstad/lrs_halmstad/follow/follow_uav.py`
- `src/lrs_halmstad/lrs_halmstad/follow/follow_core.py`
- `src/lrs_halmstad/lrs_halmstad/follow/follow_point_generator.py`
- `src/lrs_halmstad/lrs_halmstad/perception/leader_estimator.py`
- `src/lrs_halmstad/config/run_follow_defaults.yaml`

Important corrected behavior now in the live baseline:

- Odom follow uses leader body yaw for anchor placement instead of velocity direction, so reverse UGV motion does not flip the anchor side.
- Estimate follow avoids silently falling back to motion heading when that would reintroduce reverse-motion side flips.
- Explicit motion-heading paths are sign-continuous when enabled.
- Planned follow-point heading is sign-continuous.
- Estimator OBB heading ambiguity is resolved against heading history.
- Yaw solving and near-zero geometry handling are more stable.
- `Time.from_msg(...)` fallbacks are narrowed to expected conversion/type errors.
- Command state is not used as closed-loop feedback once measured pose is available.
- Default `estimate_heading_from_motion_enable` is false in the shared run defaults.

## What Block 2 Accomplished

Block 2 verified that the corrected follow baseline is not isolated. It is the code used by the practical simulation workflow.

The practical launch path uses:

- the live package `src/lrs_halmstad`
- the main launch file `src/lrs_halmstad/launch/run_follow.launch.py`
- the shared defaults file `src/lrs_halmstad/config/run_follow_defaults.yaml`
- the corrected follow executables from `src/lrs_halmstad/lrs_halmstad/follow`
- the corrected estimator from `src/lrs_halmstad/lrs_halmstad/perception/leader_estimator.py`

The support overlay uses:

- `src/lrs_halmstad/launch/support_follow_odom.launch.py`
- the same `run_follow_defaults.yaml`
- the corrected `follow_uav_odom` executable
- `/dji0/pose` converted to `/dji0/pose/odom` as the support leader input

The installed package share was refreshed so `support_follow_odom.launch.py` resolves from install to the live source launch file.

## Practical Run Workflow

Recommended full one-to-one workflow:

```bash
./run.sh tmux_1to1 warehouse
```

Recommended dry run to inspect the commands without starting the stack:

```bash
./run.sh tmux_1to1 warehouse dry_run:=true tmux_attach:=false
```

The tmux workflow starts:

- Gazebo simulation
- UAV spawn
- localization
- Nav2
- follow stack

Default odom-follow command used by the tmux workflow:

```bash
./run.sh 1to1_follow warehouse
```

Estimate/visual follow workflow:

```bash
./run.sh tmux_1to1 warehouse mode:=yolo
```

Direct estimate/visual follow wrapper:

```bash
./run.sh 1to1_yolo warehouse
```

Support overlay after the one-to-one baseline is running:

```bash
./run.sh support_follow_odom warehouse
```

If support UAV cameras are needed for later support observation work:

```bash
./run.sh support_follow_odom warehouse support_with_camera:=true
```

That belongs to support-chain/runtime verification, not to follow controller redesign.

## Main Entry Points

Top-level dispatcher:

- `run.sh`

Practical one-to-one orchestration:

- `scripts/run_tmux_1to1.sh`

Odom baseline wrapper:

- `scripts/run_1to1_follow.sh`

Estimate/visual wrapper:

- `scripts/run_1to1_yolo.sh`

Support overlay wrapper:

- `scripts/run_support_follow_odom.sh`

Main launch:

- `src/lrs_halmstad/launch/run_follow.launch.py`

Support overlay launch:

- `src/lrs_halmstad/launch/support_follow_odom.launch.py`

Shared source-of-truth params:

- `src/lrs_halmstad/config/run_follow_defaults.yaml`

## Main Launch Path

`run_1to1_follow.sh` launches:

```bash
ros2 launch lrs_halmstad run_follow.launch.py \
  ugv_mode:=nav2 \
  ugv_set_initial_pose:=true \
  leader_mode:=odom \
  world:=<world>
```

In `leader_mode:=odom`, `run_follow.launch.py` starts `follow_uav_odom`.

`run_1to1_yolo.sh` launches:

```bash
ros2 launch lrs_halmstad run_follow.launch.py \
  ugv_mode:=nav2 \
  ugv_set_initial_pose:=true \
  leader_mode:=estimate \
  start_leader_estimator:=true \
  external_detection_enable:=true \
  start_visual_actuation_bridge:=true \
  start_visual_follow_point_generator:=true \
  start_visual_follow_planner:=true \
  world:=<world>
```

In visual/estimate mode, the visual bridge disables the direct `follow_uav`/`follow_uav_odom` actuation path and uses the visual follow-point/planner/bridge path instead.

## Source-Of-Truth YAML

Use:

```text
src/lrs_halmstad/config/run_follow_defaults.yaml
```

Important follow defaults:

- `follow_uav.ros__parameters.estimate_heading_from_motion_enable: false`
- `follow_uav.ros__parameters.leader_actual_heading_enable: false`
- `follow_point_generator.ros__parameters.prefer_target_pose_heading: false`
- `follow_point_generator.ros__parameters.prefer_target_pose_position: false`

Do not blindly copy YAML from `William/Make_it_work`. That folder is reference-only and contains hardware/test-specific behavior that is not automatically correct for sim.

## Build Safely Right Now

Because the reference repo exists inside `William/Make_it_work`, future builds should avoid discovering it as a second `lrs_halmstad` package.

This handoff folder now has:

```text
William/COLCON_IGNORE
```

That tells colcon to ignore the `William` subtree during workspace discovery. It does not modify the read-only reference repo inside `William/Make_it_work`.

Safe build command:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lrs_halmstad
source install/setup.bash
```

Extra-narrow build command if you want to be explicit:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src/lrs_halmstad --packages-select lrs_halmstad
source install/setup.bash
```

After changing launch/config/source files, rebuild before relying on `./run.sh`, because the wrappers source `install/setup.bash`.

Useful launch visibility checks:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lrs_halmstad run_follow.launch.py --show-args
ros2 launch lrs_halmstad support_follow_odom.launch.py --show-args
```

Useful workflow dry run:

```bash
./run.sh tmux_1to1 warehouse dry_run:=true tmux_attach:=false
```

## Duplicate-Package Caveat

The folder:

```text
William/Make_it_work
```

contains a second `lrs_halmstad` package used only as a read-only reference for earlier block-1 follow fixes.

Do not edit it.

Do not build from it.

Do not copy whole folders from it.

If colcon reports duplicate package names, first check that `William/COLCON_IGNORE` still exists.

## Support-Chain Plug-In Point

The support side currently plugs into the corrected baseline as an overlay.

Expected order:

1. Start the trusted one-to-one baseline.
2. Start `support_follow_odom`.
3. Later, start support observation when working on block 3.

Current support-follow behavior:

- `dji1` and `dji2` are spawned as support followers.
- `/dji0/pose` is converted to `/dji0/pose/odom`.
- `dji1` and `dji2` use `follow_uav_odom` to follow UAV1 with lateral offsets.
- The same shared follow defaults feed the support controllers.

This is enough for block 2 integration. Completing the support information chain belongs to block 3.

## What Future Codex Should Know Immediately

- Treat block 1 as the corrected follow baseline.
- Do not re-port follow logic from scratch unless a concrete correctness issue is proven.
- The practical workflow is `run.sh` and the scripts in `scripts/`, not ad hoc launch commands.
- The main launch file is `run_follow.launch.py`.
- The main defaults file is `run_follow_defaults.yaml`.
- The support overlay launch is `support_follow_odom.launch.py`.
- The reference repo is under `William/Make_it_work` and is read-only.
- `William/COLCON_IGNORE` exists so colcon does not discover the reference repo as a duplicate package.
- Future block-3 work should build on the existing corrected follow baseline, not redesign it.

## Done

Done for block 1:

- corrected odom follow behavior
- corrected estimate follow heading behavior
- corrected planned follow heading continuity
- corrected estimator heading ambiguity handling
- corrected stability defaults tied to follow correctness

Done for block 2:

- practical one-to-one workflow identified
- odom baseline wrapper verified
- estimate/visual wrapper verified
- shared YAML source of truth verified
- support overlay launch path verified
- installed launch visibility refreshed
- duplicate reference package issue guarded with `William/COLCON_IGNORE`
- workflow handoff documented here

## Deferred To Block 3 Or Later

Block 3:

- complete the support-chain information flow
- decide exactly what UAV2/UAV3 detect
- decide how support detections are filtered/selected
- decide what UAV1 forwards toward the UGV side
- make support-chain outputs inspectable and measurable

Block 4:

- runtime stability and repeated-run robustness
- async inference policy
- ONNX optional/default decision
- detector/tracker machine-specific stability
- repeated recorded experiment flow

Block 5:

- cleanup and simplification without behavior changes

Block 6:

- bounded final correctness pass for subtle math/frame/stale-state issues

## What Not To Touch For Current Runs

When the goal is only to run or verify the current system, do not touch:

- controller architecture
- follow anchor semantics
- reverse-motion heading logic
- visual planner/controller design
- detector backend policy
- support-chain feature design
- `William/Make_it_work`
- broad README cleanup
- old bag artifacts

For current runs, use the practical wrappers, rebuild if source changed, and keep the corrected follow baseline stable.

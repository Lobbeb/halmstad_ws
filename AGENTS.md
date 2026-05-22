# AGENTS.md

*Everything in this file applies to the `omnet-workspace` as well*

## Gazebo ROS 2 Workspace Expectations

- ***Reuse existing code first.***.
- **Keep hardcoding to a minimum**.
- **Do not create new files unless they are clearly necessary.**
- **Check with the user on the plan before substantial changes.**
- Prefer **small, targeted edits over broad rewrites**.
- Document recent changes in `descriptions/`.
  - Only if it changes the workflow and is not a temporary fix
- Keep the workspace **clean**, prune code or files if it makes sense.
  - Ask the user first.

## Project Rules

- Using the UGVs topics (pose, odom, etc.) to help the UAV tracking is **not allowed**.
  - Tracking must be **solely** from observational sources or network metrics.
  - Unless its for for debugging purposes **only**.
- Don't run simulations in the background.
  - Let the user run it themselves and provide instructions on how.
- **Avoid adding new topics**.
  - This avoids lag and driver timing issues previously experienced by the user.
- Keep current topic names, parameter names, and launch argument styles stable unless the user explicitly wants them changed.
- **Goals must be updated if the current goal has been reached/changed.**
  - *Completed goals have a line through them.*
  - *The goals must be completed in the order of occurence:*
- Preserve core structure, unless it's broken or the user explicitly wants it.
- Keep script arguments **short** for easy use.
- If you find something that looks wrong, abundant, or is not used anymore in our workflow, **ask the user** if it can be removed.

## Goals

Goals for collecting simulation results for the **Master Thesis**:
*Completed goals have a line through them.*

- ~~UGV navigates using Nav2 and Localization and can complete routes without crashing/failing.~~
  - ~~Baylands is scanned and routes with waypoints documented.~~
  - ~~Precise lidar settings at each waypoint.~~
  ~~Fine-tuned model for optimal OBB detection.~~
  - ~~Dataset relabeling using SAM3 mask-to-obb conversion for better OBB (not using inaccurate ACML pose).~~
    - ~~Fix SSH to the university GPU lab and use it to relabel.~~
  - ~~Train new models on the new relabeled dataset.~~
- Test new fine-tuned model `baylands-leader-v9-tuned-full.pt`, (see `/home/ruben/halmstad_ws/models/obb/experiments/sam3_hybrid_full/scoreboards/full_scoreboard.md` for list of models.)
  - Ensure simulations are working with YOLO detection detecting and that we are estimating pose while also Nav2 with localization is running.
  - Same tests but we add OmNet++ to the loop and test network metrics and topics.
    - Ensure sim-time is pipelined through the bridge so timing lines up correctly.
    - Ensure we are transmitting using the flora package for LoRa.
- Explore `descriptions/C1_BASELINE_SIMPLE_OMNET_HANDOFF.md` to see how we should set up our tests.
  - This is to make sure they align for comparison.
T Test with everything activated
  - Nav2, Localization, Bayland routes, 3D-lidar, cameras, YOLO detection, tracking, following, LoRa transmissions, rosbag recordings.
- Start gathering data for results to the Master Thesis.
- *To be continued...*

## Current Known Issues

- Gazebo is not launching with the right GPU enabled.
  - Experienced driver timing issues, causing crashes.
  - Some scripts has set explicitly to use software rendering and other env variables, causing confusion and bugs.
- Simulations are topic heavy and cause heave workload
  - Topics that are not subscribed to are still being published.
  - ~~UGV has many camera topics, should remove depth camera~~ *Fixed*

## For New Sessions

Read these first at the start of a new session:

Quick orientation:

- Main package: `src/lrs_halmstad`.
- Operator entrypoints: `./run.sh` and `./stop.sh`
- Config files `src/lrs_halmstad/config`
- Baylands maps and waypoint CSVs: `maps/`
- Baylands Nav2 waypoint YAMLs: `src/lrs_halmstad/config/baylands_waypoints/`
- Datasets we used for training `datasets/final`
- Custom models used for YOLO OBB detection `models`
- OmNet++ workspace `/home/ruben/omnet-workspace/UAV_UGV`

## Code Conventions

Stay consistent with the existing codebase:

- Reuse existing functions and classes before adding new ones
- Extend current modules instead of creating parallel replacements
- Keep parameter names, topic names, and namespaces stable unless there is a strong reason to change them
- Follow the existing launch argument style and shell `name:=value` argument style
- Keep YAML defaults and launch-time overrides aligned

When working in specific areas:

- Follow logic: reuse helpers from `lrs_halmstad/follow/follow_core.py`, `follow_math.py`, and related follow modules instead of inlining geometry or control math
- Nav2 logic: extend `lrs_halmstad/nav/ugv_nav2_driver.py` and its helper flow instead of adding a second waypoint-loading path
- Launch files: preserve the current world-specific override style and existing argument names
- Shell scripts: preserve the existing parsing style and default-to-workspace behavior

Avoid:

- Duplicating helper functions that already exist nearby
- Adding "temporary" alternate paths when the current code can be extended cleanly
- Adding new files when an existing file or helper can be extended instead
- Renaming files, topics, or parameters casually in this workspace

## Validation

After code changes, use the smallest useful validation:

- Python-only change: `python3 -m py_compile <file>`
- Package-level change: `colcon build --symlink-install`
- Launch or node contract change: run `ros2 run ... --help` or `ros2 launch ... --show-args`
- Ensure processes are killed after finished with them: run `ros2 node list` to see what's active, can kill with `run_kill_all_ros2.sh`. Tmux and gazebo can be shut down with `stop_tmux_1to1.sh`.

After workflow or operator-facing changes:

- Update `README.md` and `src/lrs_halmstad/README.md`
- If logic changes update `HOW_TO_RUN.md` and `FOLLOW_MODES_GUIDE.md`

Put new descriptions in `/descriptions`

If documentation and code disagree, trust the code paths in:

- `scripts/`
- `src/lrs_halmstad/launch/`
- `src/lrs_halmstad/lrs_halmstad/`
- The **user**

Other paths might be from temporary sessions.

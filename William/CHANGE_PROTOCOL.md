# Codex Change Protocol

Purpose:
- Keep a simple running protocol of significant changes.
- Add an entry after each major fix, implementation, or behavior change.

When to log:
- New feature implementation
- Runtime or launch fix
- Config/default change affecting behavior
- Dependency/environment fix needed to run
- Documentation updates that change run instructions

Entry template:
```md
## YYYY-MM-DD HH:MM (local)
- Type: [feature|fix|config|docs|env]
- Files:
  - path/to/file
- Summary:
  - short explanation of what changed
- Why:
  - reason/problem addressed
- Behavior impact:
  - what users will observe
- Core logic impact:
  - none | minor | significant
- Verification:
  - command(s) run and result
```

---

## 2026-03-10 14:55 (local)
- Type: env
- Files:
  - system package manager (apt)
- Summary:
  - Installed `ros-jazzy-clearpath-nav2-demos`.
- Why:
  - `run_localization.sh` and `run_nav2.sh` require `clearpath_nav2_demos`.
- Behavior impact:
  - Nav2/localization launches can resolve package dependencies.
- Core logic impact:
  - none
- Verification:
  - `ros2 pkg list | grep -Fx clearpath_nav2_demos` returned installed.

## 2026-03-10 15:05 (local)
- Type: docs
- Files:
  - `README.md`
  - `RUNNING_SIM.md`
  - `CURRENT_STATE.md`
  - `src/lrs_halmstad/README.md`
- Summary:
  - Replaced hardcoded `/home/ruben/...` references with portable paths and added explicit dependency notes.
- Why:
  - Docs were partially machine-specific and confusing on this system.
- Behavior impact:
  - Cleaner onboarding and reproducible local run instructions.
- Core logic impact:
  - none
- Verification:
  - Manual review of updated markdowns.

## 2026-03-10 15:10 (local)
- Type: fix
- Files:
  - `run_gazebo_sim.sh`
  - `run_localization.sh`
  - `run_nav2.sh`
  - `run_slam.sh`
  - `src/lrs_halmstad/clearpath/setup.bash`
  - `src/lrs_halmstad/launch/run_round_follow_yolo.launch.py`
- Summary:
  - Removed machine-specific hardcoded paths and fixed script environment/path handling.
- Why:
  - Launch wrappers failed or resolved wrong locations on this machine.
- Behavior impact:
  - Startup scripts now work from this workspace path.
- Core logic impact:
  - none
- Verification:
  - Script syntax checks and runtime launch validation.

## 2026-03-10 15:12 (local)
- Type: fix
- Files:
  - `src/lrs_halmstad/lrs_halmstad/simulator.py`
  - `src/lrs_halmstad/lrs_halmstad/camera_tracker.py`
- Summary:
  - Replaced external `tf_transformations.quaternion_from_euler` import with local helper function.
- Why:
  - Runtime compatibility issue with NumPy 2 + `tf_transformations`.
- Behavior impact:
  - Prevents crash in current environment.
- Core logic impact:
  - minor (implementation-level compatibility change, intended same math behavior)
- Verification:
  - Follow stack executed without that crash.

## 2026-03-10 16:05 (local)
- Type: feature
- Files:
  - upstream `main` from `https://github.com/Lobbeb/halmstad_ws`
  - `run_tmux_1to1.sh`
  - `stop_tmux_1to1.sh`
  - `run_1to1_yolo.sh`
  - `src/lrs_halmstad/launch/run_follow_motion.launch.py`
  - `src/lrs_halmstad/config/run_follow_defaults.yaml`
  - `src/lrs_halmstad/lrs_halmstad/camera_tracker.py`
  - `src/lrs_halmstad/lrs_halmstad/simulator.py`
- Summary:
  - Integrated the latest upstream `main` refactor into this workspace and kept the local NumPy 2 quaternion compatibility patch on top.
- Why:
  - Your friend updated the project on upstream `main` with the unified 1-to-1 / YOLO stack, new tmux entrypoints, tracker configs, and launch/config renames.
- Behavior impact:
  - Main entrypoints are now `./run_tmux_1to1.sh` and `./stop_tmux_1to1.sh`.
  - YOLO now runs through the unified `run_1to1_follow.launch.py` path.
  - Old `run_round_*` launch/config paths were replaced by `run_follow_*` paths upstream.
- Core logic impact:
  - significant upstream project update, plus minor retained local compatibility patch in `camera_tracker.py` and `simulator.py`
- Verification:
  - `bash -n` passed for the updated shell scripts.
  - `python3 -m py_compile` passed for the touched Python runtime files.

## 2026-03-10 17:10 (local)
- Type: feature
- Files:
  - `run_record_experiment.sh`
  - `run_tmux_1to1.sh`
  - `stop_tmux_1to1.sh`
  - `README.md`
  - `CURRENT_STATE.md`
- Summary:
  - Added an opt-in experiment rosbag recorder that fits the existing tmux/wrapper flow and keeps the default topic set small.
- Why:
  - The current codebase needed a clean way to capture experiment runs without embedding bagging logic inside the ROS nodes or recording everything by default.
- Behavior impact:
  - `run_tmux_1to1.sh ... record:=true` now starts a recorder window alongside the normal run.
  - Recorded runs are stored under `runs/experiments/<world>/...` with `metadata.json`, `topics.txt`, and `bag/`.
  - Default bagging excludes image topics; `profile:=vision` opts into image recording.
- Core logic impact:
  - none
- Verification:
  - `bash -n run_record_experiment.sh run_tmux_1to1.sh stop_tmux_1to1.sh` passed.
  - `./run_record_experiment.sh warehouse mode:=yolo profile:=vision dry_run:=true` printed the expected topic set.
  - `./run_tmux_1to1.sh warehouse mode:=yolo record:=true record_profile:=vision dry_run:=true tmux_attach:=false` printed the expected start commands.

## 2026-03-10 18:34 (local)
- Type: fix
- Files:
  - `stop_tmux_1to1.sh`
- Summary:
  - Fixed the tmux stop script after a broken rewrite and restored executable permissions.
- Why:
  - `./stop_tmux_1to1.sh warehouse` ended with `syntax error: unexpected end of file`, which blocked clean shutdown of recorded runs.
- Behavior impact:
  - The stop script now shuts down the tmux-managed stack cleanly again.
  - `dry_run:=true` messaging is clearer and no longer looks like a real session kill.
- Core logic impact:
  - none
- Verification:
  - `bash -n stop_tmux_1to1.sh` passed.
  - `./stop_tmux_1to1.sh warehouse dry_run:=true` now executes cleanly from WSL.



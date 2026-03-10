# Thesis Working Notes (Local Baseline)

## Date
- March 10, 2026

## Goal
- Run partner code on this machine with behavior aligned to the original project flow.

## Current baseline that works
1. `./run_gazebo_sim.sh warehouse`
2. `./run_spawn_uav.sh warehouse uav_name:=dji0`
3. `./run_localization.sh warehouse`
4. `./run_nav2.sh`
5. `./run_1to1_follow.sh warehouse`

Tmux one-command run:
- `./run_tmux_1to1_follow.sh warehouse delay_s:=12`

## Required dependency
- `ros-jazzy-clearpath-nav2-demos`
- Install:
```bash
sudo apt update
sudo apt install -y ros-jazzy-clearpath-nav2-demos
```

## Notes from validation
- Stack worked on this machine with increased startup delay (`delay_s:=12`).
- `run_rqt_perspective.sh` may print Nav2 action introspection errors in WSL, but camera stream can still work.
- For a cleaner camera check, use:
```bash
./run_rqt_image_view.sh /dji0/camera0/image_raw
```

## Scope statement (code parity)
- Core behavior is preserved.
- Local compatibility/path updates were applied to make the project runnable on this machine.
- Two runtime files use an internal quaternion helper for NumPy 2 compatibility:
  - `src/lrs_halmstad/lrs_halmstad/simulator.py`
  - `src/lrs_halmstad/lrs_halmstad/camera_tracker.py`

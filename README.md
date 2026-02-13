# Halmstad ROS 2 + Gazebo Testbed — Workspace Snapshot

Status
------
This repository is a source-only snapshot of the Halmstad Stage-1 robotics testbed (ROS 2 Jazzy + Gazebo). It intentionally excludes build artifacts, runtime outputs, and virtual environments so it stays lightweight and clone-ready.

Table of contents
-----------------
- Overview
- Repository layout
- Prerequisites
- Quickstart (one-command bootstrap)
- Manual setup steps (detailed)
- Running experiments
- Important internal tooling
- Recommended development workflow
- VS Code / Pylance tips
- Branching / pushing notes
- Troubleshooting
- License & contact

Overview
--------
This workspace contains:
- Simulation and experiment orchestration (scripts/run_round.sh)
- A ROS 2 package `lrs_halmstad` with launch files, utilities and small nodes (contract checks, event relay)
- Optional OMNeT++ bridge (if present)
- Helpers and documentation to reproduce experiments on another machine

Goal: clone this repo on a new machine, run a single bootstrap script and be able to build and run experiments with minimal manual setup.

Repository layout
-----------------
- README.md (this file)
- requirements.txt (pip-only Python deps)
- scripts/
  - bootstrap_ws.sh  — automated bootstrap (venv, pip install, rosdep, colcon build)
  - run_round.sh     — experiment automation wrapper (rosbag recording + markers + phases)
- src/
  - lrs_halmstad/    — main package (nodes, launch, utils)
  - lrs_omnet_bridge/ (optional)
- Markdowns/         — notes and handoff docs
- .gitignore         — excludes build/, install/, log/, runs/, .venv, etc.

Prerequisites (host machine)
----------------------------
- Ubuntu or WSL Ubuntu (recommended; instructions assume WSL/Ubuntu)
- Python 3.10+ (system Python)
- ROS 2 Jazzy installed via apt (rclpy and ROS libs come from apt packages)
- colcon build tools (colcon-cli, colcon-common-extensions)
- git, rsync (for export workflows), rosdep
- Optional: Gazebo version compatible with installed ROS 2 distro if running simulations

Quickstart — one command (after cloning)
----------------------------------------
1. Clone:
```bash
git clone https://github.com/Lobbeb/Master_Thesis.git ~/halmstad_ws
cd ~/halmstad_ws
```

2. Bootstrap (creates venv, installs pip deps, rosdep installs system deps, builds workspace):
```bash
bash scripts/bootstrap_ws.sh
```

3. Source overlays (if not already sourced by the bootstrap script):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

4. Run experiments or utilities (examples below).

Notes: `bootstrap_ws.sh` expects ROS 2 Jazzy to be installed at `/opt/ros/jazzy`. If ROS is not installed, install it first per ROS 2 Jazzy docs.

What the bootstrap script does
-----------------------------
- Ensures ROS 2 Jazzy is available (sources /opt/ros/jazzy/setup.bash)
- Creates a Python venv `.venv` and installs `requirements.txt`
- Runs `rosdep update` and `rosdep install --from-paths src --ignore-src -r -y` (if rosdep present)
- Builds the workspace using `colcon build --symlink-install` and sources `install/setup.bash`

requirements.txt (pip-only)
---------------------------
This file contains only pip-installable Python libraries used by scripts (not rclpy).
Minimum recommended:
```
PyYAML>=6.0
```
Add extra pip packages used in analysis scripts (numpy, pandas, matplotlib) if required.

Manual setup steps (detailed)
----------------------------
If prefer to run steps yourself:

1. Install ROS 2 Jazzy (follow ROS docs).  
2. On WSL Ubuntu, ensure Distro and OpenGL setup for Gazebo (if using GPU passthrough).  
3. From workspace root:
```bash
# source ROS system
source /opt/ros/jazzy/setup.bash

# optional: create & activate venv
python3 -m venv .venv
source .venv/bin/activate

# pip deps
python -m pip install -U pip
python -m pip install -r requirements.txt

# ros dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build
colcon build --symlink-install
source install/setup.bash
```

Running experiments
------------------
- Start the reliable event relay before recording (recommended):
```bash
ros2 run lrs_halmstad coord_event_relay
```
- Run the helper script to record a round (example):
```bash
bash scripts/run_round.sh <run_id> <condition> <uav_name> <world> [--with-cameras]
```
Example:
```bash
bash scripts/run_round.sh run001 baseline a201_0000 orchard_world
```

Key nodes and scripts
---------------------
- scripts/run_round.sh — orchestrates rosbag recording, publishes event markers, runs sweeps.
- src/lrs_halmstad/lrs_halmstad/coord_event_relay.py — relay node: publish to /coord/events_cmd, node republishes reliably to /coord/events (prevents missed markers in bag).
- src/lrs_halmstad/lrs_halmstad/contract_check.py — pre-run contract check used by scripts/run_round.sh to assert required topics/services are present.
- scripts/uav_setpose_sweep.py — drives UAV set_pose sequences (verify this script is persistent rclpy process on your machine).

Design recommendations preserved here
------------------------------------
- Keep build artifacts out of Git (build/, install/, log/).
- Keep runtime data out of Git (runs/, bags, large binary outputs).
- Keep the repo source-only and reproducible by rosdep + colcon.

Git & repo notes (how this snapshot was made)
---------------------------------------------
- The repository was created by exporting a clean copy of an existing workspace (no nested .git folders were copied).
- The workspace root here is intended to be the Git root (`~/halmstad_ws` after clone).

If starting from an existing workspace with nested package-level repositories:
- Option A (used here): export a clean copy without nested `.git` and commit as a monorepo snapshot.
- Option B (preserve history): convert inner repos to submodules or use `git subtree` to preserve history (more advanced).

VS Code / Pylance tips
---------------------
If VS Code shows "Import 'rclpy' could not be resolved":

- Start VS Code from an environment with ROS 2 sourced:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
code .
```
- Or add extraPaths in `.vscode/settings.json`:
```json
{
  "python.analysis.extraPaths": [
    "/opt/ros/jazzy/lib/python3.10/site-packages",
    "${workspaceFolder}/install/**/lib/python3.10/site-packages"
  ]
}
```
Adjust Python path version (3.10) to your system Python.

Git push guidance
-----------------
To push to your GitHub repo and overwrite a remote branch safely:
```bash
git remote add origin https://github.com/Lobbeb/Master_Thesis.git
git branch -M main
git push --force-with-lease -u origin main
```
Use `--force-with-lease` rather than `--force` to reduce accidental overwrites.

If the repo should keep additional packages (e.g. `lrs_omnet_bridge`) make sure they exist under `src/` before committing.

Troubleshooting
---------------
- Missing rclpy in editor: see VS Code tips above.
- rosdep failures: ensure `sudo apt update` and ROS 2 apt repos configured.
- colcon build errors: inspect `log/` for failing package build logs; run `colcon build --event-handlers console_cohesion+` for clearer output.

Security & large files
----------------------
- Do not commit `install/`, `build/`, `log/` or binary bags — they bloat the repo.
- Use Git LFS only if very large binary artifacts must be versioned.

Recommended workflow for laptop (recover exact state quickly)
------------------------------------------------------------
1. Clone:
```bash
git clone https://github.com/Lobbeb/Master_Thesis.git ~/halmstad_ws
cd ~/halmstad_ws
```
2. Bootstrap:
```bash
bash scripts/bootstrap_ws.sh
```
3. Source and run experiments:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_round.sh run001 baseline a201_0000 orchard_world
```

License
-------
Add a license file appropriate for the project (MIT suggested if permissive sharing is desired).

Contact / Handoff
-----------------
Use issues or repository PRs for any follow-ups. For urgent local help, run the bootstrap script and attach logs from `colcon` (command + failing package logs).

Change log (snapshot notes)
---------------------------
- This repository is a snapshot intended for portability. It contains source and utilities required to reproduce Stage-1 experiments; build products and runtime outputs are intentionally excluded.

End of file.

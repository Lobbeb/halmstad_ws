#!/usr/bin/env bash

# IMPORTANT:
# - This file is executed (bash scripts/run_round.sh ...), not sourced.
# - We source scripts/env.sh BEFORE enabling "nounset" (-u), because ROS setup.bash
#   is not nounset-safe on Ubuntu 24.04 (PEP 668 etc).

# Load ROS env (must happen before set -u)
source "$(dirname "$0")/env.sh"

# Now enable strict mode safely
set -euo pipefail

FOLLOW_PID=""
EST_PID=""
BAG_PID=""


QOS_FILE="$WS_ROOT/config/rosbag_qos.yaml"
if [ ! -f "$QOS_FILE" ]; then
  echo "ERROR: QoS override file missing: $QOS_FILE"
  exit 2
fi

# RUNBOOK expects: scripts/run_round.sh <run_id> <condition> <uav> <world> [--with-cameras]
if [ $# -lt 4 ]; then
  echo "Usage: $0 <run_id> <condition> <uav_name> <world> [--with-cameras]"
  exit 2
fi

RUN_ID="$1"
CONDITION="$2"
UAV="$3"
WORLD="$4"
WITH_CAMERAS="false"
if [ "${5:-}" = "--with-cameras" ]; then
  WITH_CAMERAS="true"
fi

LEADER_MODE="${LEADER_MODE:-odom}"
UGV_ODOM_TOPIC="${UGV_ODOM_TOPIC:-/a201_0000/platform/odom}"
UGV_CMD_VEL_TOPIC="${UGV_CMD_VEL_TOPIC:-/a201_0000/cmd_vel}"
UGV_CMD_TOPICS="${UGV_CMD_TOPICS:-/a201_0000/cmd_vel,/a201_0000/platform/cmd_vel}"
REQUIRE_FLOW="${REQUIRE_FLOW:-1}"
NODE_SUFFIX="$(echo "$RUN_ID" | tr -c '[:alnum:]_' '_')"

# Ensure run root exists
RUN_ROOT="${RUN_ROOT:-$HOME/halmstad_ws/runs}"

# Put runs into runs/<group>/<run_id> (optional). Default keeps old behavior.
# You can set RUN_GROUP explicitly, or let it auto-pick based on CONDITION + LEADER_MODE.
RUN_GROUP="${RUN_GROUP:-auto}"

if [ "$RUN_GROUP" = "auto" ]; then
  if [ "$CONDITION" = "follow" ]; then
    if [ "$LEADER_MODE" = "pose" ]; then
      RUN_GROUP="estimator_pose"
    else
      RUN_GROUP="estimator"
    fi
  else
    RUN_GROUP="baseline"
  fi
fi

RUN_DIR="$RUN_ROOT/$RUN_GROUP/$RUN_ID"
mkdir -p "$RUN_DIR"
rm -rf "$RUN_DIR/bag"

START_TS="$(date -Iseconds)"

# Event marker topic (default keeps Stage-1 behavior unchanged)
EVENT_TOPIC="${EVENT_TOPIC:-/coord/events}"

pub_evt () {
  local s="$1"
  ros2 topic pub -1 "$EVENT_TOPIC" std_msgs/msg/String "{data: '$s'}" >/dev/null
  sleep 0.2
  ros2 topic pub -1 "$EVENT_TOPIC" std_msgs/msg/String "{data: '$s'}" >/dev/null
}

kill_cmd_vel_publishers () {
  pkill -9 -f "ros2 topic pub.*(/a201_0000/cmd_vel|/a201_0000/platform/cmd_vel)" 2>/dev/null || true
}

export EVENT_TOPIC
export REQUIRE_FLOW
export UGV_CMD_TOPICS
export UGV_CMD_VEL_TOPIC
export REQUIRED_FLOW_TOPICS="$UGV_ODOM_TOPIC"
# Sanity guard (new improvement)
ros2 run lrs_halmstad contract_check "$WORLD" "$UAV" 10

# Topics list aligns with your Stage-1 logging
TOPICS=(
  "/clock"
  "$UGV_ODOM_TOPIC"
  "/a201_0000/platform/odom/filtered"
  "/a201_0000/platform/cmd_vel"
  "$UGV_CMD_VEL_TOPIC"
  "/a201_0000/tf"
  "/a201_0000/tf_static"
  "$EVENT_TOPIC"
  "/${UAV}/pose_cmd"
  "/coord/leader_estimate"
  "/coord/leader_estimate_status"
  "/coord/follow_dist_cmd"
  "/coord/follow_tracking_error_cmd"
)

# If we publish markers to /coord/events_raw, also record the impaired output /coord/events
if [ "$EVENT_TOPIC" != "/coord/events" ]; then
  TOPICS+=("/coord/events")
fi

if [ "$WITH_CAMERAS" = "true" ]; then
  TOPICS+=("/${UAV}/camera0/image_raw")
  TOPICS+=("/${UAV}/camera0/camera_info")
  TOPICS+=("/a201_0000/sensors/camera_0/color/image")
fi


# Write meta.yaml
cat > "$RUN_DIR/meta.yaml" << EOF
run_id: ${RUN_ID}
run_group: ${RUN_GROUP}
timestamp: ${START_TS}
condition: ${CONDITION}
world: ${WORLD}
uav_name: ${UAV}
with_cameras: ${WITH_CAMERAS}
EOF

cleanup() {
  set +e
  kill_cmd_vel_publishers
  if [ -n "$FOLLOW_PID" ]; then
    kill "$FOLLOW_PID" 2>/dev/null || true
    wait "$FOLLOW_PID" 2>/dev/null || true
  fi
  if [ -n "$EST_PID" ]; then
    kill "$EST_PID" 2>/dev/null || true
    wait "$EST_PID" 2>/dev/null || true
  fi
  if [ -n "$BAG_PID" ]; then
    kill "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  kill_cmd_vel_publishers
}

trap cleanup EXIT

echo "[run_round] Recording rosbag to $RUN_DIR/bag ..."
ros2 bag record -o "$RUN_DIR/bag" \
  --qos-profile-overrides-path "$QOS_FILE" \
  --topics "${TOPICS[@]}" >/dev/null 2>&1 &
BAG_PID=$!
sleep 1.0 # Give rosbag a moment to start and create the bag directory
# Event markers
pub_evt "ROUND_START"


# UAV phase (Stage 1 default: grid sweep). Stage 2: follow+leash if CONDITION=follow.
pub_evt "UAV_PHASE_START"

if [ "$CONDITION" = "follow" ]; then
  echo "[run_round] CONDITION=follow -> starting leader_estimator + follow_uav (background) during UGV phase"

  EST_PID=""
  if [ "$LEADER_MODE" = "pose" ]; then
    echo "[run_round] Starting leader_estimator (background)"
    ros2 run lrs_halmstad leader_estimator --ros-args \
      -r __node:=leader_estimator_${NODE_SUFFIX} \
      -p use_sim_time:=true \
      -p ugv_odom_topic:="$UGV_ODOM_TOPIC" \
      -p out_topic:=/coord/leader_estimate \
      -p status_topic:=/coord/leader_estimate_status \
      -p publish_status:=true \
      -p est_hz:=5.0 \
      -p pose_timeout_s:=2.0 \
      -p event_topic:="$EVENT_TOPIC" \
      -p publish_events:=true \
      > "$RUN_DIR/leader_estimator.log" 2>&1 &
    EST_PID=$!
    sleep 0.3
  else
    echo "[run_round] LEADER_MODE=$LEADER_MODE -> not starting leader_estimator"
  fi

  echo "[run_round] Starting follow_uav (leader_mode=$LEADER_MODE)"
  ros2 run lrs_halmstad follow_uav --ros-args \
    -r __node:=follow_uav_${NODE_SUFFIX} \
    -p use_sim_time:=true \
    -p world:="$WORLD" \
    -p uav_name:="$UAV" \
    -p leader_input_type:="$LEADER_MODE" \
    -p leader_odom_topic:="$UGV_ODOM_TOPIC" \
    -p leader_pose_topic:=/coord/leader_estimate \
    -p tick_hz:=5.0 \
    -p d_target:=5.0 \
    -p d_max:=15.0 \
    -p z_alt:=10.0 \
    -p pose_timeout_s:=0.75 \
    -p min_cmd_period_s:=0.10 \
    -p smooth_alpha:=1.0 \
    -p event_topic:="$EVENT_TOPIC" \
    > "$RUN_DIR/follow_uav.log" 2>&1 &
  FOLLOW_PID=$!

  pub_evt "UAV_PHASE_END"

else
  echo "[run_round] CONDITION=$CONDITION -> running Stage 1 UAV sweep"
  export UAV="$UAV"
  export WORLD="$WORLD"
  export RUN_DIR="$RUN_DIR"

  python3 - << 'PY'
import os, yaml, subprocess
from pathlib import Path

defaults_path = Path(os.path.expanduser("~/halmstad_ws/src/lrs_halmstad/resource/experiment_defaults.yaml"))
defaults = yaml.safe_load(defaults_path.read_text()) if defaults_path.exists() else {}

uav = os.environ["UAV"]
world = os.environ["WORLD"]
run_dir = os.environ["RUN_DIR"]

x_min = float(defaults.get("uav_grid_x_min", -15.0))
x_max = float(defaults.get("uav_grid_x_max",  15.0))
x_steps = int(defaults.get("uav_grid_x_steps", 6))
y_min = float(defaults.get("uav_grid_y_min", -25.0))
y_max = float(defaults.get("uav_grid_y_max",  25.0))
y_steps = int(defaults.get("uav_grid_y_steps", 10))
z = float(defaults.get("uav_altitude_m", 10.0))
wait_s = float(defaults.get("uav_wait_s", 1.0))

cmd = [
    "python3", os.path.expanduser("~/halmstad_ws/scripts/uav_setpose_sweep.py"),
    "--world", world,
    "--uav", uav,
    "--x_min", str(x_min), "--x_max", str(x_max), "--x_steps", str(x_steps),
    "--y_min", str(y_min), "--y_max", str(y_max), "--y_steps", str(y_steps),
    "--z", str(z),
    "--yaw", "0.0",
    "--wait", str(wait_s),
    "--log_csv", str(Path(run_dir) / "uav_setpoints.csv"),
]

print("[run_round] UAV sweep:", " ".join(cmd))
rc = subprocess.call(cmd)
raise SystemExit(rc)
PY

  pub_evt "UAV_PHASE_END"
fi



# UGV movement
pub_evt "UGV_PHASE_START"

python3 - << 'PY'
import os, time, yaml, subprocess, signal
from pathlib import Path

defaults_path = Path(os.path.expanduser("~/halmstad_ws/src/lrs_halmstad/resource/experiment_defaults.yaml"))
defaults = {}
if defaults_path.exists():
    defaults = yaml.safe_load(defaults_path.read_text()) or {}

speed_fwd = float(defaults.get("ugv_forward_speed", 0.5))
t_fwd = float(defaults.get("ugv_forward_time_s", 4.0))
speed_turn = float(defaults.get("ugv_turn_speed", 0.5))
t_turn = float(defaults.get("ugv_turn_time_s", 3.14))
cycles = int(defaults.get("ugv_cycles", 10))

topic = os.environ.get("UGV_CMD_VEL_TOPIC", "/a201_0000/cmd_vel")
msg_fwd = (
    "{twist: {linear: {x: " + str(speed_fwd) + ", y: 0.0, z: 0.0}, "
    "angular: {x: 0.0, y: 0.0, z: 0.0}}}"
)
msg_turn = (
    "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, "
    "angular: {x: 0.0, y: 0.0, z: " + str(speed_turn) + "}}}"
)

def run_for_seconds(msg: str, secs: float):
    p = subprocess.Popen(
        ["ros2", "topic", "pub", "-r", "10", topic, "geometry_msgs/msg/TwistStamped", msg],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(secs)
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=2.0)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass

for _ in range(cycles):
    run_for_seconds(msg_fwd, t_fwd)
    run_for_seconds(msg_turn, t_turn)
PY

#
pub_evt "UGV_PHASE_END"

pub_evt "ROUND_END"

# Stop background nodes (also handled by EXIT trap, but we stop here to finish cleanly)
if [ -n "$FOLLOW_PID" ]; then
  echo "[run_round] Stopping follow_uav (pid=$FOLLOW_PID)"
  kill "$FOLLOW_PID" 2>/dev/null || true
  wait "$FOLLOW_PID" 2>/dev/null || true
  FOLLOW_PID=""
fi

if [ -n "$EST_PID" ]; then
  echo "[run_round] Stopping leader_estimator (pid=$EST_PID)"
  kill "$EST_PID" 2>/dev/null || true
  wait "$EST_PID" 2>/dev/null || true
  EST_PID=""
fi

# Stop bag
if [ -n "$BAG_PID" ]; then
  kill "$BAG_PID" 2>/dev/null || true
  wait "$BAG_PID" 2>/dev/null || true
  BAG_PID=""
fi

echo "[run_round] Done. Bag info:"
ros2 bag info "$RUN_DIR/bag" | head -n 60 || true

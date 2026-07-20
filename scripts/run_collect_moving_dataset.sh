#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WORLD="baylands"
ROUTE=""
WAYPOINT=""
OUT=""
DURATION_S="300"
HZ="0.5"
WARMUP_S="20"
MOTION_CHECK_S="30"
MIN_MOTION_M="0.5"
NAV2_DELAY_S="80"
GUI="false"
SESSION="halmstad-baylands-moving-dataset"
UAV_NAME="dji0"
TARGET_POSE_TOPIC="/a201_0000/ground_truth/odom"

usage() {
  cat >&2 <<'EOF'
Usage:
  ./run.sh collect_moving_dataset baylands route:=rotundan waypoint:=rotundan_0 out:=datasets/... duration:=300 hz:=0.5

Arguments:
  route:=NAME       Baylands route stem, e.g. rotundan, road_to_west, road_to_spawn, spawn.
  waypoint:=NAME    Start waypoint. If omitted, resolved as first waypoint in the route YAML.
  out:=DIR          Output dataset directory. Required.
  duration:=SEC     Capture duration after warmup. Default 300.
  hz:=RATE          Capture rate. Default 0.5.
  warmup:=SEC       Startup warmup before capture. Default 20.
  motion_check:=SEC Require target movement over this window before capture. Default 30.
  min_motion_m:=M   Minimum target displacement before capture. Default 0.5.
  nav2_delay_s:=SEC Delay Nav2 start so dataset TF helper is ready first. Default 80.
  gui:=true|false   Gazebo GUI. Default false.
  session:=NAME     tmux session name.
EOF
}

if [ "$#" -gt 0 ] && [[ "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
  usage
  exit 0
fi

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

for arg in "$@"; do
  case "$arg" in
    route:=*|nav2_goals:=*)
      ROUTE="${arg#*:=}"
      ;;
    waypoint:=*)
      WAYPOINT="${arg#waypoint:=}"
      ;;
    out:=*)
      OUT="${arg#out:=}"
      ;;
    duration:=*|capture_duration:=*)
      DURATION_S="${arg#*:=}"
      ;;
    hz:=*|capture_hz:=*)
      HZ="${arg#*:=}"
      ;;
    warmup:=*|warmup_s:=*)
      WARMUP_S="${arg#*:=}"
      ;;
    motion_check:=*|motion_check_s:=*)
      MOTION_CHECK_S="${arg#*:=}"
      ;;
    min_motion_m:=*)
      MIN_MOTION_M="${arg#min_motion_m:=}"
      ;;
    nav2_delay_s:=*)
      NAV2_DELAY_S="${arg#nav2_delay_s:=}"
      ;;
    gui:=*)
      GUI="${arg#gui:=}"
      ;;
    session:=*)
      SESSION="${arg#session:=}"
      ;;
    uav_name:=*)
      UAV_NAME="${arg#uav_name:=}"
      ;;
    target_pose_topic:=*)
      TARGET_POSE_TOPIC="${arg#target_pose_topic:=}"
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage
      exit 2
      ;;
  esac
done

if [ "$WORLD" != "baylands" ]; then
  echo "collect_moving_dataset is Baylands-only; got world='$WORLD'" >&2
  exit 2
fi
if [ -z "$ROUTE" ]; then
  echo "Missing required route:=..." >&2
  usage
  exit 2
fi
if [ -z "$OUT" ]; then
  echo "Missing required out:=..." >&2
  usage
  exit 2
fi

route_yaml="$WS_ROOT/src/lrs_halmstad/config/baylands_waypoints/baylands_waypoints_${ROUTE}.yaml"
if [ ! -f "$route_yaml" ]; then
  echo "Baylands route YAML not found: $route_yaml" >&2
  exit 2
fi

if [ -z "$WAYPOINT" ]; then
  WAYPOINT="$(python3 - "$route_yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}
for waypoint in data.get("waypoints") or []:
    if isinstance(waypoint, dict) and waypoint.get("name"):
        print(str(waypoint["name"]))
        raise SystemExit(0)
raise SystemExit("route YAML has no named waypoints")
PY
)"
fi

case "$OUT" in
  bags/results*|./bags/results*|"$WS_ROOT"/bags/results*)
    echo "Refusing to write dataset output into Results folder: $OUT" >&2
    exit 2
    ;;
esac

mkdir -p "$OUT"
OUT_ABS="$(cd "$OUT" && pwd)"

summary_path="$OUT_ABS/moving_capture_summary_$(date -u +%Y%m%dT%H%M%SZ).json"
capture_pid=""
gt_odom_tf_pid=""

cleanup() {
  if [ -n "${capture_pid:-}" ] && kill -0 "$capture_pid" 2>/dev/null; then
    echo "[collect_moving_dataset] Stopping capture_dataset"
    kill -INT "-$capture_pid" 2>/dev/null || kill -INT "$capture_pid" 2>/dev/null || true
    sleep 3
    kill -TERM "-$capture_pid" 2>/dev/null || kill -TERM "$capture_pid" 2>/dev/null || true
  fi
  if [ -n "${gt_odom_tf_pid:-}" ] && kill -0 "$gt_odom_tf_pid" 2>/dev/null; then
    echo "[collect_moving_dataset] Stopping ground-truth odom TF helper"
    kill -INT "-$gt_odom_tf_pid" 2>/dev/null || kill -INT "$gt_odom_tf_pid" 2>/dev/null || true
    sleep 1
    kill -TERM "-$gt_odom_tf_pid" 2>/dev/null || kill -TERM "$gt_odom_tf_pid" 2>/dev/null || true
  fi
  ./stop.sh tmux_1to1 "$WORLD" "session:=$SESSION" || true
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-120}"
  local deadline=$((SECONDS + timeout_s))
  echo "[collect_moving_dataset] Waiting for topic: $topic"
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ros2 topic echo --no-daemon --once --qos-profile sensor_data "$topic" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for topic: $topic" >&2
  return 1
}

require_target_motion() {
  echo "[collect_moving_dataset] Verifying target motion on $TARGET_POSE_TOPIC for ${MOTION_CHECK_S}s"
  python3 - "$TARGET_POSE_TOPIC" "$MOTION_CHECK_S" "$MIN_MOTION_M" <<'PY'
import math
import subprocess
import sys
import time

import yaml

topic = sys.argv[1]
duration_s = float(sys.argv[2])
min_motion_m = float(sys.argv[3])


def sample_pose():
    try:
        out = subprocess.check_output(
            ["timeout", "6s", "ros2", "topic", "echo", "--no-daemon", "--once", topic],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None
    try:
        msg = yaml.safe_load(out.split("---", 1)[0])
        pos = msg["pose"]["pose"]["position"]
        return float(pos["x"]), float(pos["y"])
    except Exception:
        return None


start = sample_pose()
if start is None:
    raise SystemExit(f"could not sample start pose from {topic}")

deadline = time.monotonic() + duration_s
end = start
max_span = 0.0
while time.monotonic() < deadline:
    time.sleep(2.0)
    pose = sample_pose()
    if pose is None:
        continue
    end = pose
    max_span = max(max_span, math.hypot(pose[0] - start[0], pose[1] - start[1]))
    if max_span >= min_motion_m:
        print(f"motion_ok displacement_m={max_span:.3f}")
        raise SystemExit(0)

print(
    f"motion_failed displacement_m={max_span:.3f} "
    f"start=({start[0]:.3f},{start[1]:.3f}) end=({end[0]:.3f},{end[1]:.3f})",
    file=sys.stderr,
)
raise SystemExit(1)
PY
}

activate_platform_controllers() {
  local deadline=$((SECONDS + 90))
  echo "[collect_moving_dataset] Ensuring Clearpath platform controllers are active"
  while [ "$SECONDS" -lt "$deadline" ]; do
    if ros2 control list_controllers -c /a201_0000/controller_manager 2>/dev/null | grep -Eq '^platform_velocity_controller[[:space:]].*[[:space:]]active$'; then
      return 0
    fi
    ros2 control switch_controllers \
      -c /a201_0000/controller_manager \
      --activate joint_state_broadcaster platform_velocity_controller >/dev/null 2>&1 || true
    sleep 2
  done
  echo "Timed out waiting for platform_velocity_controller to become active" >&2
  return 1
}

write_summary() {
  local status="$1"
  local error="${2:-}"
  python3 - "$summary_path" "$status" "$error" "$WORLD" "$ROUTE" "$WAYPOINT" "$OUT_ABS" "$DURATION_S" "$WARMUP_S" "$HZ" <<'PY'
import json
import sys
from datetime import datetime, timezone

path, status, error, world, route, waypoint, out_dir, duration_s, warmup_s, hz = sys.argv[1:]
data = {
    "updated_at": datetime.now(timezone.utc).isoformat(),
    "status": status,
    "error": error or None,
    "world": world,
    "route": route,
    "waypoint": waypoint,
    "output_dir": out_dir,
    "duration_s": float(duration_s),
    "warmup_s": float(warmup_s),
    "capture_hz": float(hz),
    "mode": "moving_nav2_capture",
}
with open(path, "w", encoding="utf-8") as handle:
    json.dump(data, handle, indent=2, sort_keys=True)
PY
}

write_summary "starting"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "[collect_moving_dataset] Starting Baylands moving capture route=$ROUTE waypoint=$WAYPOINT out=$OUT_ABS hz=$HZ duration=${DURATION_S}s"
./stop.sh tmux_1to1 "$WORLD" "session:=$SESSION" || true
./run.sh tmux_1to1 "$WORLD" \
  "mode:=follow" \
  "gui:=$GUI" \
  "tmux_attach:=false" \
  "session:=$SESSION" \
  "waypoint:=$WAYPOINT" \
  "nav2_goals:=$ROUTE" \
  "nav2_delay_s:=$NAV2_DELAY_S" \
  "ugv_use_amcl_odom_fallback:=true"

wait_for_topic "/${UAV_NAME}/camera0/camera_info" 120
wait_for_topic "/${UAV_NAME}/camera0/image_raw" 120
wait_for_topic "/${UAV_NAME}/camera0/actual/center_pose" 120
wait_for_topic "$TARGET_POSE_TOPIC" 120

# Nav2 listens on the Clearpath namespace TF topics. Provide sim odom->base_link
# from ground truth and let AMCL own map->odom.
setsid ros2 run lrs_halmstad odom_to_tf --ros-args \
  -r __ns:=/a201_0000 \
  -r /tf:=/a201_0000/tf \
  -r /tf_static:=/a201_0000/tf_static \
  -p use_sim_time:=true \
  -p odom_topic:=ground_truth/odom \
  -p frame_id:=odom \
  -p child_frame_id:=base_link \
  >/tmp/halmstad_ws/moving_dataset_ground_truth_odom_to_tf.log 2>&1 &
gt_odom_tf_pid="$!"
sleep 2
activate_platform_controllers

echo "[collect_moving_dataset] Warmup ${WARMUP_S}s"
sleep "$WARMUP_S"
if ! require_target_motion; then
  write_summary "failed" "target did not move before capture"
  exit 1
fi

setsid ./run.sh capture_dataset "$WORLD" \
  "out:=$OUT_ABS" \
  "uav_name:=$UAV_NAME" \
  "target_pose_topic:=$TARGET_POSE_TOPIC" \
  "hz:=$HZ" \
  "save_overlay:=false" \
  "save_metadata:=true" \
  "save_negative_examples:=true" &
capture_pid="$!"

sleep 3
if ! kill -0 "$capture_pid" 2>/dev/null; then
  write_summary "failed" "capture_dataset exited during startup"
  exit 1
fi

write_summary "capturing"
echo "[collect_moving_dataset] Capturing for ${DURATION_S}s"
sleep "$DURATION_S"

write_summary "completed"
echo "[collect_moving_dataset] Completed moving capture: $OUT_ABS"

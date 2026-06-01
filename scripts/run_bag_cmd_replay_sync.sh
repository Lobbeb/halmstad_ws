#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"

source "$SCRIPT_DIR/slam_state_common.sh"

BAG=""
MODE="cmd"
SOURCE_TOPIC="/a201_0000/platform/cmd_vel"
TARGET_TOPIC="/a201_0000/platform/cmd_vel"
UGV_POSE_TOPIC="/a201_0000/ground_truth/odom"
UAV_POSE_TOPIC="/dji0/pose"
UGV_REPLAY_TOPIC="/bag_cmd_replay_sync/ugv_pose"
UAV_REPLAY_TOPIC="/bag_cmd_replay_sync/uav_pose"
POSE_TARGETS="ugv,uav"
WORLD="baylands"
UGV_ENTITY=""
UAV_ENTITY="dji0"
POSE_RATE_HZ="30.0"
START_SET_POSE_BRIDGE=true
CLOCK_TOPIC="/clock"
RATE="1.0"
OFFSET_ADD_S="0.0"
START_OFFSET_S="0.0"
SYNC_TO_CLOCK=false
RETIME_STAMP=true
REPLAY_TOPIC="/bag_cmd_replay_sync/platform_cmd_vel"
TIMEOUT_S="0"
MIN_CLOCK_S="0.001"
MIN_TICK_DELTA_S="0.001"
START_PAUSED=false
DRY_RUN=false

usage() {
  cat <<EOF
Usage:
  ./run.sh bag_cmd_replay_sync bag:=PATH [topic:=/a201_0000/platform/cmd_vel] [target_topic:=TOPIC]

Waits for the live Gazebo ROS /clock to tick, then replays bag data against the
live Gazebo simulation. Do not use bag --clock with live Gazebo.

Options:
  bag:=PATH              Bag directory, or run dir containing bag/
  mode:=cmd|pose         cmd replays UGV cmd_vel; pose teleports UGV/UAV from bag poses
  topic:=TOPIC           Topic recorded in the bag, default $SOURCE_TOPIC
  target_topic:=TOPIC    Topic to publish to, default same as topic
  ugv_pose_topic:=TOPIC  UGV recorded pose/odom for mode:=pose, default $UGV_POSE_TOPIC
  uav_pose_topic:=TOPIC  UAV recorded pose for mode:=pose, default $UAV_POSE_TOPIC
  pose_targets:=ugv,uav  Which entities to update in mode:=pose. Use one target if Gazebo GUI is unstable.
  world:=NAME            Gazebo world key. Default: active sim world, else baylands
  ugv_entity:=NAME       Gazebo UGV entity. Default: repo Clearpath entity
  uav_entity:=NAME       Gazebo UAV entity. Default: dji0
  pose_rate_hz:=N        Max Gazebo set_pose rate per entity. Default: $POSE_RATE_HZ
  start_set_pose_bridge:=true|false  Start temporary set_pose bridge if missing
  clock_topic:=TOPIC     Clock topic to sample, default /clock
  rate:=N                Bag playback rate, default 1.0
  start_offset_s:=N      Bag start offset in seconds, default 0.0
  sync_to_clock:=true    Legacy mode: use sampled /clock as bag start-offset
  offset_add_s:=N        Add seconds to start-offset, or sampled clock in sync_to_clock mode
  retime_stamp:=true     Replay via relay and stamp commands with live sim time
  replay_topic:=TOPIC    Private relay input topic, default $REPLAY_TOPIC
  timeout_s:=N           Timeout while waiting for a live ticking clock; 0 waits forever
  min_clock_s:=N         Require clock to be at least this value, default $MIN_CLOCK_S
  min_tick_delta_s:=N    Require two clock samples to differ by this much, default $MIN_TICK_DELTA_S
  start_paused:=true     Start rosbag paused
  dry_run:=true          Print command only

Example:
  ./run.sh bag_cmd_replay_sync bag:=bags/replay_sources/ugv_cmd_test
  ./run.sh bag_cmd_replay_sync bag:=bags/replay_sources/rotundan_manual mode:=pose world:=baylands
  ./run.sh bag_cmd_replay_sync bag:=bags/replay_sources/rotundan_manual mode:=pose pose_targets:=ugv pose_rate_hz:=10
EOF
}

resolve_path() {
  local path="$1"
  case "$path" in
    /*) printf '%s\n' "$path" ;;
    *) printf '%s\n' "$WS_ROOT/$path" ;;
  esac
}

resolve_bag() {
  local path resolved
  path="$(resolve_path "$1")"
  if [ -d "$path/bag" ]; then
    path="$path/bag"
  fi
  if [ ! -d "$path" ]; then
    echo "Bag path does not exist: $path" >&2
    exit 2
  fi
  if [ ! -f "$path/metadata.yaml" ] && ! compgen -G "$path/*.mcap" >/dev/null; then
    echo "Not a ROS 2 bag directory: $path" >&2
    exit 2
  fi
  resolved="$(readlink -f "$path")"
  printf '%s\n' "$resolved"
}

coerce_bool() {
  case "$1" in
    true|false) printf '%s\n' "$1" ;;
    *) echo "Invalid boolean value: $1" >&2; exit 2 ;;
  esac
}

contains_csv_token() {
  local csv="$1"
  local token="$2"
  case ",$csv," in
    *",$token,"*) return 0 ;;
    *) return 1 ;;
  esac
}

shell_join() {
  local out="" part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

bag_has_topic() {
  local topic="$1"
  if [ -f "$BAG_DIR/metadata.yaml" ]; then
    grep -Fq "name: $topic" "$BAG_DIR/metadata.yaml"
    return $?
  fi
  ros2 bag info "$BAG_DIR" 2>/dev/null | grep -Fq "Topic: $topic "
}

parse_clock_s() {
  local raw="$1"
  CLOCK_RAW="$raw" python3 - <<'PY'
import os
import re

text = os.environ.get("CLOCK_RAW", "")
sec = re.search(r"^\s*sec:\s*(-?\d+)\s*$", text, re.M)
nsec = re.search(r"^\s*nanosec:\s*(\d+)\s*$", text, re.M)
if not sec:
    raise SystemExit(f"Could not parse clock.sec from: {text!r}")
value = int(sec.group(1))
if nsec:
    value += int(nsec.group(1)) * 1e-9
print(f"{value:.6f}")
PY
}

clock_once_s() {
  local raw
  raw="$(timeout 1 ros2 topic echo --once "$CLOCK_TOPIC" --field clock 2>/dev/null || true)"
  if [ -z "$raw" ] || grep -q 'does not appear to be published' <<<"$raw"; then
    return 1
  fi
  parse_clock_s "$raw" 2>/dev/null
}

wait_for_clock_tick() {
  local deadline last current now last_status
  if python3 - "$TIMEOUT_S" <<'PY'
import sys
raise SystemExit(0 if float(sys.argv[1]) > 0 else 1)
PY
  then
    deadline="$(python3 - "$TIMEOUT_S" <<'PY'
import sys
import time
print(f"{time.monotonic() + float(sys.argv[1]):.6f}")
PY
)"
  else
    deadline=""
  fi
  last=""
  last_status=0
  while [ -z "$deadline" ] || python3 - "$deadline" <<'PY'
import sys
import time
raise SystemExit(0 if time.monotonic() < float(sys.argv[1]) else 1)
PY
  do
    current="$(clock_once_s || true)"
    if [ -n "$current" ] && python3 - "$current" "$MIN_CLOCK_S" <<'PY'
import sys
current = float(sys.argv[1])
minimum = float(sys.argv[2])
raise SystemExit(0 if current >= minimum else 1)
PY
    then
      if [ -n "$last" ] && python3 - "$current" "$last" "$MIN_TICK_DELTA_S" <<'PY'
import sys
current = float(sys.argv[1])
last = float(sys.argv[2])
minimum_delta = float(sys.argv[3])
raise SystemExit(0 if current - last >= minimum_delta else 1)
PY
      then
        printf '%s\n' "$current"
        return 0
      fi
      last="$current"
    fi
    now="$(date +%s)"
    if [ "$((now - last_status))" -ge 2 ]; then
      if [ -n "$last" ]; then
        echo "[bag_cmd_replay_sync] waiting for ticking $CLOCK_TOPIC; last=${last}s" >&2
      else
        echo "[bag_cmd_replay_sync] waiting for $CLOCK_TOPIC" >&2
      fi
      last_status="$now"
    fi
    sleep 0.1
  done
  echo "Timed out waiting for live ticking $CLOCK_TOPIC (min_clock_s=$MIN_CLOCK_S, min_tick_delta_s=$MIN_TICK_DELTA_S)" >&2
  exit 3
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    bag:=*)
      BAG="${arg#bag:=}"
      ;;
    mode:=*|replay:=*)
      MODE="${arg#*:=}"
      ;;
    topic:=*)
      SOURCE_TOPIC="${arg#topic:=}"
      ;;
    target_topic:=*)
      TARGET_TOPIC="${arg#target_topic:=}"
      ;;
    ugv_pose_topic:=*)
      UGV_POSE_TOPIC="${arg#ugv_pose_topic:=}"
      ;;
    uav_pose_topic:=*)
      UAV_POSE_TOPIC="${arg#uav_pose_topic:=}"
      ;;
    pose_targets:=*)
      POSE_TARGETS="${arg#pose_targets:=}"
      ;;
    world:=*)
      WORLD="${arg#world:=}"
      ;;
    ugv_entity:=*)
      UGV_ENTITY="${arg#ugv_entity:=}"
      ;;
    uav_entity:=*)
      UAV_ENTITY="${arg#uav_entity:=}"
      ;;
    pose_rate_hz:=*)
      POSE_RATE_HZ="${arg#pose_rate_hz:=}"
      ;;
    start_set_pose_bridge:=*)
      START_SET_POSE_BRIDGE="$(coerce_bool "${arg#start_set_pose_bridge:=}")"
      ;;
    clock_topic:=*)
      CLOCK_TOPIC="${arg#clock_topic:=}"
      ;;
    rate:=*)
      RATE="${arg#rate:=}"
      ;;
    offset_add_s:=*)
      OFFSET_ADD_S="${arg#offset_add_s:=}"
      ;;
    start_offset_s:=*)
      START_OFFSET_S="${arg#start_offset_s:=}"
      ;;
    sync_to_clock:=*)
      SYNC_TO_CLOCK="$(coerce_bool "${arg#sync_to_clock:=}")"
      ;;
    retime_stamp:=*)
      RETIME_STAMP="$(coerce_bool "${arg#retime_stamp:=}")"
      ;;
    replay_topic:=*)
      REPLAY_TOPIC="${arg#replay_topic:=}"
      ;;
    timeout_s:=*)
      TIMEOUT_S="${arg#timeout_s:=}"
      ;;
    min_clock_s:=*)
      MIN_CLOCK_S="${arg#min_clock_s:=}"
      ;;
    min_tick_delta_s:=*)
      MIN_TICK_DELTA_S="${arg#min_tick_delta_s:=}"
      ;;
    start_paused:=*)
      START_PAUSED="$(coerce_bool "${arg#start_paused:=}")"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${arg#dry_run:=}")"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$MODE" in
  cmd|pose)
    ;;
  *)
    echo "Invalid mode: $MODE" >&2
    echo "Use mode:=cmd or mode:=pose" >&2
    exit 2
    ;;
esac

if [ -z "$BAG" ]; then
  echo "Missing bag:=PATH" >&2
  usage >&2
  exit 2
fi

BAG_DIR="$(resolve_bag "$BAG")"
if [ -f "$SIM_WORLD_FILE" ] && [ "$WORLD" = "baylands" ]; then
  sim_world="$(cat "$SIM_WORLD_FILE" 2>/dev/null || true)"
  if [ -n "$sim_world" ]; then
    WORLD="$sim_world"
  fi
fi
if [ -z "$UGV_ENTITY" ]; then
  UGV_ENTITY="$(slam_state_robot_entity_name "$WS_ROOT")"
fi

if [ "$MODE" = "cmd" ] && ! bag_has_topic "$SOURCE_TOPIC"; then
  echo "Bag does not contain topic: $SOURCE_TOPIC" >&2
  echo "Check available command topics with:" >&2
  echo "  grep -n 'name: /a201_0000/.*cmd_vel' '$BAG_DIR/metadata.yaml'" >&2
  exit 4
fi
if [ "$MODE" = "pose" ]; then
  if contains_csv_token "$POSE_TARGETS" ugv && ! bag_has_topic "$UGV_POSE_TOPIC"; then
    echo "Bag does not contain UGV pose topic: $UGV_POSE_TOPIC" >&2
    exit 4
  fi
  if contains_csv_token "$POSE_TARGETS" uav && ! bag_has_topic "$UAV_POSE_TOPIC"; then
    echo "Bag does not contain UAV pose topic: $UAV_POSE_TOPIC" >&2
    exit 4
  fi
fi
if [ "$DRY_RUN" = true ]; then
  LIVE_CLOCK_S="0.000000"
else
  LIVE_CLOCK_S="$(wait_for_clock_tick)"
fi
if [ "$SYNC_TO_CLOCK" = true ]; then
  OFFSET_S="$(python3 - "$LIVE_CLOCK_S" "$OFFSET_ADD_S" <<'PY'
import sys
current = float(sys.argv[1])
offset = float(sys.argv[2])
print(f"{max(current + offset, 0.0):.6f}")
PY
)"
else
  OFFSET_S="$(python3 - "$START_OFFSET_S" "$OFFSET_ADD_S" <<'PY'
import sys
start = float(sys.argv[1])
offset = float(sys.argv[2])
print(f"{max(start + offset, 0.0):.6f}")
PY
)"
fi

cmd=(ros2 bag play "$BAG_DIR" --rate "$RATE" --start-offset "$OFFSET_S")
if [ "$MODE" = "cmd" ]; then
  cmd+=(--topics "$SOURCE_TOPIC")
  if [ "$RETIME_STAMP" = true ]; then
    cmd+=(--remap "$SOURCE_TOPIC:=$REPLAY_TOPIC")
  elif [ "$SOURCE_TOPIC" != "$TARGET_TOPIC" ]; then
    cmd+=(--remap "$SOURCE_TOPIC:=$TARGET_TOPIC")
  fi
else
  TOPICS=()
  if contains_csv_token "$POSE_TARGETS" ugv; then
    TOPICS+=("$UGV_POSE_TOPIC")
    cmd+=(--remap "$UGV_POSE_TOPIC:=$UGV_REPLAY_TOPIC")
  fi
  if contains_csv_token "$POSE_TARGETS" uav; then
    TOPICS+=("$UAV_POSE_TOPIC")
    cmd+=(--remap "$UAV_POSE_TOPIC:=$UAV_REPLAY_TOPIC")
  fi
  cmd+=(--topics "${TOPICS[@]}")
fi
if [ "$START_PAUSED" = true ]; then
  cmd+=(--start-paused)
fi

echo "[bag_cmd_replay_sync] mode=$MODE clock=$CLOCK_TOPIC live_clock=${LIVE_CLOCK_S}s start_offset=${OFFSET_S}s sync_to_clock=$SYNC_TO_CLOCK bag=$BAG_DIR"
if [ "$MODE" = "cmd" ] && [ "$RETIME_STAMP" = true ]; then
  echo "[bag_cmd_replay_sync] retiming $REPLAY_TOPIC -> $TARGET_TOPIC with live sim-time stamps"
elif [ "$MODE" = "pose" ]; then
  echo "[bag_cmd_replay_sync] pose replay world=$WORLD ugv_entity=$UGV_ENTITY uav_entity=$UAV_ENTITY targets=$POSE_TARGETS"
fi
echo "[bag_cmd_replay_sync] $(shell_join "${cmd[@]}")"
if [ "$DRY_RUN" = true ]; then
  exit 0
fi

relay_pids=()
bridge_pid=""
cleanup_relays() {
  local pid=""
  for pid in "${relay_pids[@]:-}"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  if [ -n "$bridge_pid" ] && kill -0 "$bridge_pid" 2>/dev/null; then
    kill "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
}

if [ "$MODE" = "cmd" ] && [ "$RETIME_STAMP" = true ]; then
  python3 - "$REPLAY_TOPIC" "$TARGET_TOPIC" <<'PY' &
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter


def main() -> None:
    input_topic = sys.argv[1]
    output_topic = sys.argv[2]
    rclpy.init(args=None)
    node = rclpy.create_node(
        "bag_cmd_replay_retimer",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    pub = node.create_publisher(TwistStamped, output_topic, 10)

    def on_msg(msg: TwistStamped) -> None:
        out = TwistStamped()
        out.header = msg.header
        out.header.stamp = node.get_clock().now().to_msg()
        out.twist = msg.twist
        pub.publish(out)

    node.create_subscription(TwistStamped, input_topic, on_msg, 10)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
  relay_pids+=("$!")
  trap cleanup_relays EXIT INT TERM
  sleep 0.5
elif [ "$MODE" = "pose" ]; then
  GZ_WORLD="$(slam_state_gazebo_world_name "$WORLD")"
  SET_POSE_SERVICE="/world/${GZ_WORLD}/set_pose"
  if [ "$START_SET_POSE_BRIDGE" = true ] && ! timeout 2s ros2 service list 2>/dev/null | grep -qx "$SET_POSE_SERVICE"; then
    echo "[bag_cmd_replay_sync] starting temporary set_pose bridge for $SET_POSE_SERVICE"
    ros2 run ros_gz_bridge parameter_bridge \
      "${SET_POSE_SERVICE}@ros_gz_interfaces/srv/SetEntityPose" \
      --ros-args -r __node:=bag_pose_set_pose_bridge >/dev/null 2>&1 &
    bridge_pid="$!"
    sleep 1.0
  fi

  UGV_INPUT_TOPIC=""
  UAV_INPUT_TOPIC=""
  if contains_csv_token "$POSE_TARGETS" ugv; then
    UGV_INPUT_TOPIC="$UGV_REPLAY_TOPIC"
  fi
  if contains_csv_token "$POSE_TARGETS" uav; then
    UAV_INPUT_TOPIC="$UAV_REPLAY_TOPIC"
  fi

  python3 - \
    "$SET_POSE_SERVICE" \
    "$UGV_INPUT_TOPIC" "$UGV_ENTITY" \
    "$UAV_INPUT_TOPIC" "$UAV_ENTITY" \
    "$POSE_RATE_HZ" <<'PY' &
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from ros_gz_interfaces.srv import SetEntityPose


def main() -> None:
    service_name = sys.argv[1]
    ugv_topic = sys.argv[2]
    ugv_entity = sys.argv[3]
    uav_topic = sys.argv[4]
    uav_entity = sys.argv[5]
    max_hz = float(sys.argv[6])
    min_period_ns = int(1e9 / max_hz) if max_hz > 0 else 0

    rclpy.init(args=None)
    node = rclpy.create_node(
        "bag_pose_replay_set_pose",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    client = node.create_client(SetEntityPose, service_name)
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"SetEntityPose service unavailable: {service_name}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

    last_sent: dict[str, int] = {}
    pending: object | None = None

    def send_pose(entity_name: str, pose) -> None:
        nonlocal pending
        now_ns = node.get_clock().now().nanoseconds
        if min_period_ns and now_ns - last_sent.get(entity_name, -10**30) < min_period_ns:
            return
        if pending is not None and not pending.done():
            return
        req = SetEntityPose.Request()
        req.entity.name = entity_name
        req.entity.type = 2
        req.pose = pose
        pending = client.call_async(req)
        last_sent[entity_name] = now_ns

    if ugv_topic:
        node.create_subscription(
            Odometry,
            ugv_topic,
            lambda msg: send_pose(ugv_entity, msg.pose.pose),
            10,
        )
        node.get_logger().info(f"UGV pose replay: {ugv_topic} -> {ugv_entity}")

    if uav_topic:
        node.create_subscription(
            PoseStamped,
            uav_topic,
            lambda msg: send_pose(uav_entity, msg.pose),
            10,
        )
        node.get_logger().info(f"UAV pose replay: {uav_topic} -> {uav_entity}")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
  relay_pids+=("$!")
  trap cleanup_relays EXIT INT TERM
  sleep 0.5
fi

set +e
"${cmd[@]}"
status=$?
set -e
cleanup_relays
exit "$status"

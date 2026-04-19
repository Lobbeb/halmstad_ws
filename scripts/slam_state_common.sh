#!/usr/bin/env bash

slam_state_ws_root() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  cd "$script_dir/.." && pwd
}

slam_state_source_env() {
  local ws_root="$1"

  set +u
  source /opt/ros/jazzy/setup.bash
  source "$ws_root/install/setup.bash"
  source "$ws_root/src/lrs_halmstad/clearpath/setup.bash"
  set -u
}

slam_state_default_name() {
  local sim_world_file="/tmp/halmstad_ws/gazebo_sim.world"
  local world="warehouse"
  local sim_world=""

  if [ -f "$sim_world_file" ]; then
    sim_world="$(cat "$sim_world_file" 2>/dev/null || true)"
    if [ -n "$sim_world" ]; then
      world="$sim_world"
    fi
  fi

  printf '%s\n' "$world"
}

slam_state_gazebo_world_name() {
  local world_name="$1"
  case "$world_name" in
    construction)
      printf 'office_construction\n'
      ;;
    *)
      printf '%s\n' "$world_name"
      ;;
  esac
}

slam_state_namespace() {
  local ws_root="$1"
  local robot_yaml="$ws_root/src/lrs_halmstad/clearpath/robot.yaml"
  local namespace=""

  namespace="$(awk '/^[[:space:]]+namespace:[[:space:]]/ { print $2; exit }' "$robot_yaml")"
  if [ -z "$namespace" ]; then
    echo "[slam_state] Failed to read ROS namespace from $robot_yaml" >&2
    return 1
  fi

  printf '%s\n' "$namespace"
}

slam_state_robot_entity_name() {
  local ws_root="$1"
  local namespace=""

  namespace="$(slam_state_namespace "$ws_root")" || return 1
  printf '%s/robot\n' "$namespace"
}

slam_state_dir_root() {
  local ws_root="$1"
  printf '%s/maps/slam_states\n' "$ws_root"
}

slam_state_dir_for_name() {
  local ws_root="$1"
  local state_name="$2"
  printf '%s/%s\n' "$(slam_state_dir_root "$ws_root")" "$state_name"
}

slam_posegraph_path_for_name() {
  local ws_root="$1"
  local state_name="$2"
  printf '%s/%s\n' "$(slam_state_dir_for_name "$ws_root" "$state_name")" "$state_name"
}

slam_metadata_path_for_name() {
  local ws_root="$1"
  local state_name="$2"
  printf '%s/metadata.env\n' "$(slam_state_dir_for_name "$ws_root" "$state_name")"
}

slam_map_base_for_name() {
  local ws_root="$1"
  local state_name="$2"
  printf '%s/maps/%s\n' "$ws_root" "$state_name"
}

slam_posegraph_file_for_base() {
  local base_path="$1"
  printf '%s.posegraph\n' "$base_path"
}

slam_posegraph_data_file_for_base() {
  local base_path="$1"
  printf '%s.data\n' "$base_path"
}

slam_posegraph_base_from_input() {
  local input_path="$1"

  case "$input_path" in
    *.posegraph)
      printf '%s\n' "${input_path%.posegraph}"
      ;;
    *.data)
      printf '%s\n' "${input_path%.data}"
      ;;
    *)
      printf '%s\n' "$input_path"
      ;;
  esac
}

slam_posegraph_artifacts_exist() {
  local base_path="$1"
  local posegraph_file
  local data_file

  posegraph_file="$(slam_posegraph_file_for_base "$base_path")"
  data_file="$(slam_posegraph_data_file_for_base "$base_path")"

  [ -f "$posegraph_file" ] && [ -f "$data_file" ]
}

slam_posegraph_resolve_base() {
  local requested_base="$1"
  local legacy_base="${requested_base}.posegraph"

  if slam_posegraph_artifacts_exist "$requested_base"; then
    printf '%s\n' "$requested_base"
    return 0
  fi

  if slam_posegraph_artifacts_exist "$legacy_base"; then
    printf '%s\n' "$legacy_base"
    return 0
  fi

  return 1
}

slam_service_base() {
  local namespace="$1"
  printf '/%s/slam_toolbox\n' "$namespace"
}

slam_state_capture_gazebo_pose_env() {
  local ws_root="$1"
  local world_name="${2:-$(slam_state_default_name)}"
  local timeout_s="${3:-5}"
  local entity_name=""

  entity_name="$(slam_state_robot_entity_name "$ws_root")" || return 1
  slam_state_capture_gazebo_named_pose_env "$world_name" "$entity_name" "$timeout_s"
}

slam_state_capture_gazebo_named_pose_env() {
  local world_name="${1:-$(slam_state_default_name)}"
  local entity_name="$2"
  local timeout_s="${3:-5}"
  local gz_world=""
  local topic=""
  local pose_json=""
  local gz_bin="/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"

  gz_world="$(slam_state_gazebo_world_name "$world_name")"

  for topic in "/world/${gz_world}/dynamic_pose/info" "/world/${gz_world}/pose/info"; do
    pose_json="$(timeout "$timeout_s" "$gz_bin" topic -e -n 1 -t "$topic" --json-output 2>/dev/null || true)"
    if [ -z "$pose_json" ]; then
      continue
    fi

    if POSE_JSON="$pose_json" python3 - "$entity_name" <<'PY'
import json
import math
import os
import sys

entity_name = sys.argv[1]
text = os.environ.get("POSE_JSON", "").strip()
if not text:
    raise SystemExit(1)

data = None
try:
    data = json.loads(text)
except json.JSONDecodeError:
    for line in reversed([line.strip() for line in text.splitlines() if line.strip()]):
        try:
            data = json.loads(line)
            break
        except json.JSONDecodeError:
            continue
    if data is None:
        raise SystemExit(1)

poses = data.get("pose", [])
match = None
for pose in poses:
    if pose.get("name") == entity_name:
        match = pose
        break

if match is None:
    raise SystemExit(1)

position = match.get("position", {})
orientation = match.get("orientation", {})
x = float(position.get("x", 0.0))
y = float(position.get("y", 0.0))
z = float(position.get("z", 0.0))
qz = float(orientation.get("z", 0.0))
qw = float(orientation.get("w", 1.0))
yaw = 2.0 * math.atan2(qz, qw)

print(f"spawn_x={x:.9f}")
print(f"spawn_y={y:.9f}")
print(f"spawn_z={z:.9f}")
print(f"spawn_yaw={yaw:.9f}")
print(f"spawn_qz={qz:.9f}")
print(f"spawn_qw={qw:.9f}")
PY
    then
      return 0
    fi
  done

  return 1
}

slam_state_capture_uav_spawn_from_ugv_env() {
  local ws_root="$1"
  local world_name="${2:-$(slam_state_default_name)}"
  local body_x_offset="${3:--7.0}"
  local body_y_offset="${4:-0.0}"
  local uav_z="${5:-7.0}"
  local timeout_s="${6:-5}"
  local ugv_pose_env=""

  ugv_pose_env="$(slam_state_capture_gazebo_pose_env "$ws_root" "$world_name" "$timeout_s")" || return 1
  eval "$ugv_pose_env"

  python3 - "$spawn_x" "$spawn_y" "$spawn_z" "$spawn_yaw" "$body_x_offset" "$body_y_offset" "$uav_z" <<'PY'
import math
import sys

ugv_x = float(sys.argv[1])
ugv_y = float(sys.argv[2])
ugv_z = float(sys.argv[3])
ugv_yaw = float(sys.argv[4])
body_x_offset = float(sys.argv[5])
body_y_offset = float(sys.argv[6])
uav_z = float(sys.argv[7])

uav_x = ugv_x + body_x_offset * math.cos(ugv_yaw) - body_y_offset * math.sin(ugv_yaw)
uav_y = ugv_y + body_x_offset * math.sin(ugv_yaw) + body_y_offset * math.cos(ugv_yaw)

print(f"ugv_x={ugv_x:.9f}")
print(f"ugv_y={ugv_y:.9f}")
print(f"ugv_z={ugv_z:.9f}")
print(f"ugv_yaw={ugv_yaw:.9f}")
print(f"uav_x={uav_x:.9f}")
print(f"uav_y={uav_y:.9f}")
print(f"uav_z={uav_z:.9f}")
print(f"uav_yaw={ugv_yaw:.9f}")
print(f"uav_yaw_deg={math.degrees(ugv_yaw):.9f}")
PY
}

slam_state_capture_map_pose_env() {
  local ws_root="$1"
  local timeout_s="${2:-5}"
  local namespace=""

  namespace="$(slam_state_namespace "$ws_root")" || return 1

  python3 - "$namespace" "$timeout_s" <<'PY'
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

namespace = sys.argv[1]
timeout_s = float(sys.argv[2])

rclpy.init(args=None)
node = rclpy.create_node(
    "slam_state_capture_map_pose",
    cli_args=[
        "--ros-args",
        "-r",
        f"/tf:=/{namespace}/tf",
        "-r",
        f"/tf_static:=/{namespace}/tf_static",
    ],
)

buffer = Buffer(cache_time=Duration(seconds=max(timeout_s, 2.0)))
listener = TransformListener(buffer, node, spin_thread=False)

deadline = time.monotonic() + timeout_s
transform = None
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform("map", "base_link", Time())
            break
        except Exception:
            continue
finally:
    listener.unregister()
    node.destroy_node()
    rclpy.shutdown()

if transform is None:
    raise SystemExit(1)

translation = transform.transform.translation
rotation = transform.transform.rotation

yaw = math.atan2(
    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
    1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
)

print(f"map_pose_x={translation.x:.9f}")
print(f"map_pose_y={translation.y:.9f}")
print(f"map_pose_yaw={yaw:.9f}")
print(f"map_pose_qz={rotation.z:.9f}")
print(f"map_pose_qw={rotation.w:.9f}")
PY
}

slam_wait_for_service() {
  local service_name="$1"
  local timeout_s="${2:-60}"
  local waited=0

  while [ "$waited" -lt "$timeout_s" ]; do
    if ros2 service list --no-daemon 2>/dev/null | grep -qx "$service_name"; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done

  return 1
}

slam_wait_for_service_or_process() {
  local service_name="$1"
  local pid="$2"
  local timeout_s="${3:-60}"
  local waited=0

  while [ "$waited" -lt "$timeout_s" ]; do
    if ! kill -0 "$pid" 2>/dev/null; then
      return 2
    fi
    if ros2 service list --no-daemon 2>/dev/null | grep -qx "$service_name"; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done

  return 1
}

slam_state_write_metadata() {
  local metadata_path="$1"
  local state_name="$2"
  local world_name="$3"
  local namespace="$4"
  local posegraph_base="$5"
  local map_base="$6"
  local scan_topic="$7"
  local spawn_x="${8:-}"
  local spawn_y="${9:-}"
  local spawn_z="${10:-}"
  local spawn_yaw="${11:-}"
  local spawn_qz="${12:-}"
  local spawn_qw="${13:-}"
  local map_pose_x="${14:-}"
  local map_pose_y="${15:-}"
  local map_pose_yaw="${16:-}"
  local map_pose_qz="${17:-}"
  local map_pose_qw="${18:-}"

  cat > "$metadata_path" <<EOF
state_name=$state_name
world=$world_name
namespace=$namespace
posegraph_base=$posegraph_base
posegraph_file=$(slam_posegraph_file_for_base "$posegraph_base")
posegraph_data=$(slam_posegraph_data_file_for_base "$posegraph_base")
map_yaml=${map_base}.yaml
map_pgm=${map_base}.pgm
scan_topic=$scan_topic
spawn_x=$spawn_x
spawn_y=$spawn_y
spawn_z=$spawn_z
spawn_yaw=$spawn_yaw
spawn_qz=$spawn_qz
spawn_qw=$spawn_qw
map_pose_x=$map_pose_x
map_pose_y=$map_pose_y
map_pose_yaw=$map_pose_yaw
map_pose_qz=$map_pose_qz
map_pose_qw=$map_pose_qw
saved_at=$(date -Iseconds)
EOF
}

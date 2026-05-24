#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
TMUX_STATE_DIR="$STATE_DIR/tmux_sessions"
WORLD="baylands"
SESSION=""
SESSION_EXPLICIT=false
ACTION="start"
MAP_PATH=""
GUI="false"
TMUX_ATTACH=true
DRY_RUN=false
LAYOUT="panes"
MODE="yolo"
RECORD=false
RECORD_PROFILE="default"
RECORD_TAG=""
RECORD_OUT=""
BASE_DELAY_S=7
BASE_DELAY_SET=false
SPAWN_DELAY_OVERRIDE=""
LOCALIZATION_DELAY_OVERRIDE=""
NAV2_DELAY_OVERRIDE=""
FOLLOW_DELAY_OVERRIDE=""
RECORD_DELAY_OVERRIDE=""
GAZEBO_READY_TIMEOUT_S=180
GAZEBO_READY_SETTLE_S=10
UAV_NAME="dji0"
ROS_DOMAIN_ID_EFFECTIVE="${ROS_DOMAIN_ID:-3}"
RMW_IMPLEMENTATION_EFFECTIVE="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# The ROS Python detector executable uses /usr/bin/python3. Do not let IDE or
# conda settings hide user-site packages such as ultralytics.
unset PYTHONNOUSERSITE
UGV_NAMESPACE="a201_0000"
OMNET_NETWORK="lora"
OMNET_UI="cmdenv"
OMNET_PROJECT=""
OMNET_RESULT_DIR=""
OMNET_LORA_SF=""
OMNET_LORA_BW=""
OMNET_BRIDGE_PORT="5555"
OMNET_START_DELAY_OVERRIDE=""
DEFAULT_OMNET_START_DELAY_S="3.0"
DEFAULT_UAV_START_DELAY_S="0.0"
DEFAULT_UAV_BODY_X_OFFSET="-7.0"
DEFAULT_UAV_BODY_Y_OFFSET="0.0"
DEFAULT_UAV_Z="7.0"
UGV_START_DELAY_OVERRIDE=""
UAV_START_DELAY_OVERRIDE=""
UAV_HEIGHT_OVERRIDE=""
BAYLANDS_DEFAULT_WAYPOINT="road_to_spawn_0"
BAYLANDS_DEFAULT_NAV2_GOALS="road_to_spawn"
BAYLANDS_NAV_SPAWN_X="-14.085738068"
BAYLANDS_NAV_SPAWN_Y="-54.861874768"
BAYLANDS_NAV_SPAWN_Z="0.100975479"
BAYLANDS_NAV_SPAWN_YAW="0.484129496"
BAYLANDS_NAV_LIDAR_MODE="3d"
NAV_LIDAR_MODE=""
HAS_GAZEBO_SPAWN_OVERRIDE="false"
GAZEBO_SPAWN_STATE_NAME=""
GAZEBO_WAYPOINT_NAME=""
WAYPOINT_EXPLICIT="false"
GAZEBO_SPAWN_X_OVERRIDE=""
GAZEBO_SPAWN_Y_OVERRIDE=""
GAZEBO_SPAWN_Z_OVERRIDE=""
GAZEBO_SPAWN_YAW_OVERRIDE=""
HAVE_UGV_GOAL_SEQUENCE="false"
HAVE_RANGE_MODE="false"
FOLLOW_WAIT_TOPICS=""
NAV2_GOALS_FOR_LIDAR=""
SPAWN_ARGS=()
FOLLOW_ARGS=()
GAZEBO_ARGS=()
NAV_LIDAR_ARGS=()
RECORD_CMD=()
OMNET="false"

source "$SCRIPT_DIR/slam_state_common.sh"
source "$SCRIPT_DIR/baylands_waypoint_common.sh"
source "$SCRIPT_DIR/baylands_route_lidar_common.sh"
BAYLANDS_GROUP_WAYPOINT_CSV="$(baylands_group_waypoint_csv)"

is_action() {
  case "$1" in
    start|restart|stack_restart|follow|follow_restart|status|attach)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

print_usage() {
  cat <<'EOF'
Usage:
  ./run.sh tmux_1to1 [world] [action] [options...]
  ./run.sh tmux_1to1 [action] [world] [options...]

Actions:
  [ start | restart | stack_restart | follow | follow_restart | status | attach ]

Common:

  mode:=follow|yolo  |   gui:=true|false    |     waypoint:=name
  session:=name
  tmux_attach:=true|false
  layout:=windows|panes
  dry_run:=true|false

Recording:
  record:=[true|false]
  record_profile:=[default|step2_light|vision]
  record_tag:=[name]
  record_out:=[bags/experiments/... ]

Gazebo spawn / scene:
  state:=checkpoint ( x:=... y:=... z:=... yaw:=...)
  camera:=attached
  camera_update_rate:=20
  height:=7
  mount_pitch_deg:=45
  uav_name:=dji0
  rtf:=1.0

Nav2 / route:
  nav2_goals:=route_name|route.yaml
  goal_sequence_file:=route_name|route.yaml
  goal_sequence_csv:=x,y,yaw;...
  ugv_goal_sequence_file:=route_name|route.yaml
  ugv_goal_sequence_csv:=x,y,yaw;...
  ugv_goal_sequence_randomize:=true|false
  ugv_goal_sequence_random_reverse:=true|false
  ugv_goal_sequence_relative_to_current_pose:=true|false

Localization / lidar:
  lidar:=2d|3d
  scan_topic:=...
  pointcloud_topic:=...
  use_scan_relay:=true|false
  scan_relay_hz:=...
  pc2ls_min_height:=...
  pc2ls_max_height:=...

Follow / camera:
  follow_yaw:=true|false
  pan_enable:=true|false
  use_tilt:=true|false
  tilt_enable:=true|false
  camera_default_tilt_deg:=...
  use_actual_heading:=true|false
  leader_actual_heading_enable:=true|false
  leader_actual_pose_enable:=true|false
  camera_actual_pose_reacquire_enable:=true|false
  range_mode:=auto|depth|radio|const
  params_file:=/path/run_follow_defaults.yaml

YOLO only:
  weights:=...
  target:=...
  obb:=true|false
  tracker:=true|false
  yolo_control_mode:=visual_bridge|follow_uav_estimate
  visual_follow_logic:=legacy|follow_core
  external_detection_node:=detector|tracker
  tracker_config:=botsort.yaml
  detector_backend:=ultralytics|onnx_cpu|onnx_directml
  detector_async_inference:=true|false
  yolo_device:=cpu|auto
  detector_onnx_model:=...
  start_visual_follow_controller:=true|false
  start_visual_follow_point_generator:=true|false
  start_visual_follow_planner:=true|false
  start_visual_actuation_bridge:=true|false

OMNeT:
  omnet:=true|false
  omnet_network:=wifi|5g|lora|lora-sf10-250|lora-duplex|lora-sweep
  lora_sf:=7..12
  lora_bw:=125kHz|250kHz
  omnet_ui:=cmdenv|qtenv
  omnet_project:=/path/UAV_UGV
  omnet_result_dir:=/path
  omnet_bridge_port:=5555
  omnet_start_delay_s:=3.0

Timing:
  delay_s:=9
  spawn_delay_s:=9
  localization_delay_s:=11
  nav2_delay_s:=11
  follow_delay_s:=13
  follow_wait_topics:=/topic_a,/topic_b
  record_delay_s:=13
  ugv_start_delay_s:=3.0
  uav_start_delay_s:=12.0
  gazebo_ready_timeout_s:=180
  gazebo_ready_settle_s:=10

Examples:
  ./run.sh tmux_1to1 baylands mode:=yolo omnet:=true record:=true
  ./run.sh tmux_1to1 baylands mode:=follow waypoint:=rotundan_0 nav2_goals:=rotundan omnet:=true
  ./run.sh tmux_1to1 status session:=halmstad-baylands-1to1
EOF
}

resolve_baylands_waypoint() {
  local waypoint_name="$1"
  python3 - "$waypoint_name" "$BAYLANDS_GROUP_WAYPOINT_CSV" <<'PY'
import csv
import sys

name, *paths = sys.argv[1:]
for path in paths:
    try:
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                waypoint_name = str(row.get("place", "")).strip()
                if waypoint_name != name:
                    continue
                x_val = row.get("x")
                y_val = row.get("y")
                if x_val in (None, "") or y_val in (None, ""):
                    raise SystemExit(f"Waypoint '{name}' is missing world x/y in {path}")
                print(f"spawn_x={float(x_val)}")
                print(f"spawn_y={float(y_val)}")
                z_val = row.get("z")
                if z_val not in (None, ""):
                    print(f"spawn_z={float(z_val)}")
                yaw_val = row.get("yaw")
                if yaw_val not in (None, ""):
                    print(f"spawn_yaw={float(yaw_val)}")
                raise SystemExit(0)
    except FileNotFoundError:
        continue
raise SystemExit(f"Waypoint '{name}' was not found in the Baylands waypoint CSV")
PY
}

compute_uav_spawn_from_ugv_pose_env() {
  local ugv_x="$1"
  local ugv_y="$2"
  local ugv_z="$3"
  local ugv_yaw="$4"
  local body_x_offset="${5:--7.0}"
  local body_y_offset="${6:-0.0}"
  local uav_z="${7:-7.0}"

  python3 - "$ugv_x" "$ugv_y" "$ugv_z" "$ugv_yaw" "$body_x_offset" "$body_y_offset" "$uav_z" <<'PY'
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

print(f"uav_x={uav_x:.9f}")
print(f"uav_y={uav_y:.9f}")
print(f"uav_z={uav_z:.9f}")
print(f"uav_yaw={ugv_yaw:.9f}")
print(f"uav_yaw_deg={math.degrees(ugv_yaw):.9f}")
PY
}

case "${1:-}" in
  help|-h|--help)
    print_usage
    exit 0
    ;;
esac

if [ "$#" -gt 0 ] && is_action "$1"; then
  ACTION="$1"
  shift
fi

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]] && ! is_action "$1"; then
  WORLD="$1"
  shift
fi

if [ "$#" -gt 0 ] && is_action "$1"; then
  ACTION="$1"
  shift
fi

for arg in "$@"; do
  case "$arg" in
    session:=*)
      SESSION="${arg#session:=}"
      SESSION_EXPLICIT=true
      ;;
    map:=*)
      MAP_PATH="${arg#map:=}"
      ;;
    gui:=*)
      GUI="${arg#gui:=}"
      ;;
    rtf:=*|real_time_factor:=*)
      GAZEBO_ARGS+=("$arg")
      ;;
    x:=*)
      HAS_GAZEBO_SPAWN_OVERRIDE="true"
      GAZEBO_SPAWN_X_OVERRIDE="${arg#x:=}"
      GAZEBO_ARGS+=("$arg")
      ;;
    y:=*)
      HAS_GAZEBO_SPAWN_OVERRIDE="true"
      GAZEBO_SPAWN_Y_OVERRIDE="${arg#y:=}"
      GAZEBO_ARGS+=("$arg")
      ;;
    z:=*)
      HAS_GAZEBO_SPAWN_OVERRIDE="true"
      GAZEBO_SPAWN_Z_OVERRIDE="${arg#z:=}"
      GAZEBO_ARGS+=("$arg")
      ;;
    yaw:=*)
      HAS_GAZEBO_SPAWN_OVERRIDE="true"
      GAZEBO_SPAWN_YAW_OVERRIDE="${arg#yaw:=}"
      GAZEBO_ARGS+=("$arg")
      ;;
    state:=*)
      HAS_GAZEBO_SPAWN_OVERRIDE="true"
      GAZEBO_SPAWN_STATE_NAME="${arg#state:=}"
      GAZEBO_ARGS+=("$arg")
      ;;
    spawn_state:=*)
      HAS_GAZEBO_SPAWN_OVERRIDE="true"
      GAZEBO_SPAWN_STATE_NAME="${arg#spawn_state:=}"
      GAZEBO_ARGS+=("$arg")
      ;;
    state_name:=*)
      HAS_GAZEBO_SPAWN_OVERRIDE="true"
      GAZEBO_SPAWN_STATE_NAME="${arg#state_name:=}"
      GAZEBO_ARGS+=("$arg")
      ;;
    waypoint:=*)
      HAS_GAZEBO_SPAWN_OVERRIDE="true"
      GAZEBO_WAYPOINT_NAME="${arg#waypoint:=}"
      WAYPOINT_EXPLICIT="true"
      GAZEBO_ARGS+=("$arg")
      ;;
    tmux_attach:=*|attach:=*)
      TMUX_ATTACH="${arg#*:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    layout:=*)
      LAYOUT="${arg#layout:=}"
      ;;
    mode:=*|stack:=*)
      MODE="${arg#*:=}"
      ;;
    record:=*)
      RECORD="${arg#record:=}"
      ;;
    record_profile:=*)
      RECORD_PROFILE="${arg#record_profile:=}"
      ;;
    record_tag:=*)
      RECORD_TAG="${arg#record_tag:=}"
      ;;
    record_out:=*)
      RECORD_OUT="${arg#record_out:=}"
      ;;
    yolo:=*)
      case "${arg#yolo:=}" in
        true|yes|1)
          MODE="yolo"
          ;;
        false|no|0)
          MODE="follow"
          ;;
        *)
          echo "Invalid yolo option: ${arg#yolo:=}" >&2
          echo "Use yolo:=true or yolo:=false" >&2
          exit 2
          ;;
      esac
      ;;
    panes:=*)
      case "${arg#panes:=}" in
        true|yes|1)
          LAYOUT="panes"
          ;;
        false|no|0)
          LAYOUT="windows"
          ;;
        *)
          echo "Invalid panes option: ${arg#panes:=}" >&2
          echo "Use panes:=true or panes:=false" >&2
          exit 2
          ;;
      esac
      ;;
    delay_s:=*)
      BASE_DELAY_S="${arg#delay_s:=}"
      BASE_DELAY_SET=true
      ;;
    spawn_delay_s:=*)
      SPAWN_DELAY_OVERRIDE="${arg#spawn_delay_s:=}"
      ;;
    gazebo_ready_timeout_s:=*|sim_ready_timeout_s:=*)
      GAZEBO_READY_TIMEOUT_S="${arg#*:=}"
      ;;
    gazebo_ready_settle_s:=*|sim_ready_settle_s:=*)
      GAZEBO_READY_SETTLE_S="${arg#*:=}"
      ;;
    localization_delay_s:=*)
      LOCALIZATION_DELAY_OVERRIDE="${arg#localization_delay_s:=}"
      ;;
    nav2_delay_s:=*)
      NAV2_DELAY_OVERRIDE="${arg#nav2_delay_s:=}"
      ;;
    follow_delay_s:=*)
      FOLLOW_DELAY_OVERRIDE="${arg#follow_delay_s:=}"
      ;;
    follow_wait_topics:=*)
      FOLLOW_WAIT_TOPICS="${arg#follow_wait_topics:=}"
      ;;
    record_delay_s:=*)
      RECORD_DELAY_OVERRIDE="${arg#record_delay_s:=}"
      ;;
    lidar:=2d|scan_sensor:=2d)
      NAV_LIDAR_MODE="2d"
      ;;
    lidar:=3d|scan_sensor:=3d)
      NAV_LIDAR_MODE="3d"
      ;;
    scan_topic:=*|pointcloud_topic:=*|use_scan_relay:=*|scan_relay_hz:=*|scan_relay_max_age_s:=*|scan_relay_restamp:=*|scan_relay_start_delay_s:=*|pc2ls_min_height:=*|pc2ls_max_height:=*|pc2ls_angle_min:=*|pc2ls_angle_max:=*|pc2ls_angle_increment:=*|pc2ls_scan_time:=*|pc2ls_range_min:=*|pc2ls_range_max:=*|pc2ls_queue_size:=*|pc2ls_target_frame:=*|pc2ls_transform_tolerance:=*|pc2ls_use_inf:=*)
      NAV_LIDAR_ARGS+=("$arg")
      ;;
    camera_mode:=*|uav_camera_mode:=*)
      echo "Use camera:=attached with $0." >&2
      exit 2
      ;;
    camera:=*|height:=*|mount_pitch_deg:=*|camera_update_rate:=*|uav_name:=*)

      if [[ "$arg" == uav_name:=* ]]; then
        UAV_NAME="${arg#uav_name:=}"
      elif [[ "$arg" == height:=* ]]; then
        UAV_HEIGHT_OVERRIDE="${arg#height:=}"
      fi
      SPAWN_ARGS+=("$arg")
      FOLLOW_ARGS+=("$arg")
      ;;
    goal_sequence_file:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      NAV2_GOALS_FOR_LIDAR="${arg#goal_sequence_file:=}"
      if [[ "$WORLD" == baylands* ]]; then
        FOLLOW_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "$NAV2_GOALS_FOR_LIDAR")")
      else
        FOLLOW_ARGS+=("nav2_goals:=$NAV2_GOALS_FOR_LIDAR")
      fi
      ;;
    goal_sequence_csv:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      FOLLOW_ARGS+=("ugv_goal_sequence_csv:=${arg#goal_sequence_csv:=}")
      ;;
    nav2_goals:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      NAV2_GOALS_FOR_LIDAR="${arg#nav2_goals:=}"
      if [[ "$WORLD" == baylands* ]]; then
        FOLLOW_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "$NAV2_GOALS_FOR_LIDAR")")
      else
        FOLLOW_ARGS+=("$arg")
      fi
      ;;
    ugv_goal_sequence_file:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      if [[ "$WORLD" == baylands* ]]; then
        FOLLOW_ARGS+=("ugv_goal_sequence_file:=$(baylands_route_yaml_path "${arg#ugv_goal_sequence_file:=}")")
      else
        FOLLOW_ARGS+=("$arg")
      fi
      ;;
    ugv_goal_sequence_csv:=*)
      HAVE_UGV_GOAL_SEQUENCE="true"
      FOLLOW_ARGS+=("$arg")
      ;;
    params_file:=*)
      FOLLOW_ARGS+=("$arg")
      ;;
    follow_yaw:=*|pan_enable:=*|use_tilt:=*|tilt_enable:=*|camera_default_tilt_deg:=*|use_actual_heading:=*|leader_actual_heading_enable:=*|leader_actual_heading_topic:=*|leader_actual_pose_enable:=*|camera_actual_pose_reacquire_enable:=*|ugv_goal_sequence_randomize:=*|ugv_goal_sequence_random_reverse:=*|ugv_goal_sequence_relative_to_current_pose:=*)
      FOLLOW_ARGS+=("$arg")
      ;;
    range_mode:=*)
      HAVE_RANGE_MODE="true"
      FOLLOW_ARGS+=("$arg")
      ;;
    weights:=*|target:=*|use_estimate:=*|yolo_control_mode:=*|visual_follow_logic:=*|obb:=*|folder:=*|dir:=*|subdir:=*|tracker:=*|external_detection_node:=*|tracker_config:=*|yolo_device:=*|device:=*|detector_backend:=*|detector_async_inference:=*|detector_onnx_model:=*|ugv_start_delay_s:=*|start_visual_follow_controller:=*|start_visual_follow_point_generator:=*|start_visual_follow_planner:=*|start_visual_actuation_bridge:=*|follow_point_prefer_target_pose_heading:=*|follow_point_prefer_target_pose_position:=*|leader_selected_target_topic:=*|leader_selected_target_filtered_topic:=*|leader_selected_target_filtered_status_topic:=*|leader_visual_target_estimate_topic:=*|leader_visual_target_estimate_status_topic:=*|leader_follow_point_topic:=*|leader_follow_point_status_topic:=*|leader_planned_target_topic:=*|leader_planned_target_status_topic:=*|leader_visual_control_topic:=*|leader_visual_control_status_topic:=*|leader_visual_actuation_bridge_status_topic:=*)
      FOLLOW_ARGS+=("$arg")
      ;;
    omnet:=*)
      case "${arg#omnet:=}" in
        true|yes|1)
          OMNET="true"
          ;;
        false|no|0)
          OMNET="false"
          ;;
        *)
          echo "Invalid omnet option: ${arg#omnet:=}" >&2
          echo "Use omnet:=true or omnet:=false" >&2
          exit 2
          ;;
      esac
      ;;
    omnet_network:=*|omnet_config:=*)
      OMNET_NETWORK="${arg#*:=}"
      ;;
    omnet_ui:=*|omnet_env:=*)
      OMNET_UI="${arg#*:=}"
      ;;
    omnet_project:=*)
      OMNET_PROJECT="${arg#omnet_project:=}"
      ;;
    omnet_result_dir:=*)
      OMNET_RESULT_DIR="${arg#omnet_result_dir:=}"
      ;;
    lora_sf:=*|omnet_lora_sf:=*|sf:=*)
      OMNET_LORA_SF="${arg#*:=}"
      ;;
    lora_bw:=*|omnet_lora_bw:=*|bw:=*)
      OMNET_LORA_BW="${arg#*:=}"
      ;;
    omnet_bridge_port:=*)
      OMNET_BRIDGE_PORT="${arg#omnet_bridge_port:=}"
      FOLLOW_ARGS+=("$arg")
      ;;
    omnet_start_delay_s:=*|omnet_warmup_s:=*)
      OMNET_START_DELAY_OVERRIDE="${arg#*:=}"
      ;;
    ugv_start_delay_s:=*)
      UGV_START_DELAY_OVERRIDE="${arg#ugv_start_delay_s:=}"
      ;;
    uav_start_delay_s:=*)
      UAV_START_DELAY_OVERRIDE="${arg#uav_start_delay_s:=}"
      ;;
    help|-h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      print_usage >&2
      exit 2
      ;;
  esac
done

if [ -z "$SESSION" ]; then
  SESSION="halmstad-${WORLD}-1to1"
fi

if [[ "$WORLD" == baylands* ]]; then
  baylands_sync_waypoints "$DRY_RUN"
fi

case "$MODE" in
  follow|yolo)
    ;;
  *)
    echo "Invalid mode: $MODE" >&2
    echo "Use mode:=follow or mode:=yolo" >&2
    exit 2
    ;;
esac

if [ "$MODE" != "yolo" ]; then
  for arg in "${FOLLOW_ARGS[@]}"; do
      case "$arg" in
      weights:=*|target:=*|use_estimate:=*|yolo_control_mode:=*|visual_follow_logic:=*|obb:=*|folder:=*|dir:=*|subdir:=*|tracker:=*|external_detection_node:=*|tracker_config:=*|yolo_device:=*|device:=*|detector_backend:=*|detector_async_inference:=*|detector_onnx_model:=*|range_mode:=*|ugv_start_delay_s:=*|start_visual_follow_controller:=*|start_visual_follow_point_generator:=*|start_visual_follow_planner:=*|start_visual_actuation_bridge:=*|follow_point_prefer_target_pose_heading:=*|follow_point_prefer_target_pose_position:=*|leader_selected_target_topic:=*|leader_selected_target_filtered_topic:=*|leader_selected_target_filtered_status_topic:=*|leader_visual_target_estimate_topic:=*|leader_visual_target_estimate_status_topic:=*|leader_follow_point_topic:=*|leader_follow_point_status_topic:=*|leader_planned_target_topic:=*|leader_planned_target_status_topic:=*|leader_visual_control_topic:=*|leader_visual_control_status_topic:=*|leader_visual_actuation_bridge_status_topic:=*)
        echo "Argument '$arg' requires mode:=yolo" >&2
        exit 2
        ;;
    esac
  done
else
  for arg in "${FOLLOW_ARGS[@]}"; do
    case "$arg" in
      use_actual_heading:=*|leader_actual_heading_enable:=*|leader_actual_heading_topic:=*|leader_actual_pose_enable:=*|leader_actual_pose_topic:=*|camera_actual_pose_reacquire_enable:=*|camera_leader_actual_pose_topic:=*|ugv_odom_topic:=*|start_ugv_ground_truth_bridge:=*)
        echo "Argument '$arg' is disabled with mode:=yolo; the YOLO pipeline must not consume UGV odom, AMCL, or ground-truth pose." >&2
        exit 2
        ;;
    esac
  done
fi

if [ -z "$GUI" ]; then
  EFFECTIVE_GUI=true
else
  case "$GUI" in
    true|false)
      EFFECTIVE_GUI="$GUI"
      ;;
    *)
      echo "Invalid gui option: $GUI" >&2
      echo "Use gui:=true or gui:=false" >&2
      exit 2
      ;;
  esac

case "$RECORD" in
  true|false)
    ;;
  *)
    echo "Invalid record option: $RECORD" >&2
    echo "Use record:=true or record:=false" >&2
    exit 2
    ;;
esac

case "$RECORD_PROFILE" in
  default|step2_light|vision)
    ;;
  *)
    echo "Invalid record_profile: $RECORD_PROFILE" >&2
    echo "Use record_profile:=default, step2_light, or vision" >&2
    exit 2
    ;;
esac
fi

case "$LAYOUT" in
  windows|panes)
    ;;
  *)
    echo "Invalid layout: $LAYOUT" >&2
    echo "Use layout:=windows or layout:=panes" >&2
    exit 2
    ;;
esac

SESSION_SAFE="$(printf '%s' "$SESSION" | tr -c 'A-Za-z0-9_.-' '_')"
SESSION_STATE_FILE="$TMUX_STATE_DIR/${SESSION_SAFE}.env"
record_pane=""
omnet_pane=""
GAZEBO_PANE_ID=""
SPAWN_PANE_ID=""
LOCALIZATION_PANE_ID=""
NAV2_PANE_ID=""
FOLLOW_PANE_ID=""
OMNET_PANE_ID=""
RECORD_PANE_ID=""

apply_default_delays() {
  if [ "$EFFECTIVE_GUI" = false ]; then
    [ "$BASE_DELAY_SET" = true ] || BASE_DELAY_S=7
  else
    [ "$BASE_DELAY_SET" = true ] || BASE_DELAY_S=6
  fi

  SPAWN_DELAY_S="$BASE_DELAY_S"
  LOCALIZATION_DELAY_S=$((BASE_DELAY_S + 2))
  NAV2_DELAY_S="$LOCALIZATION_DELAY_S"
  FOLLOW_DELAY_S=$((LOCALIZATION_DELAY_S + 2))
  RECORD_DELAY_S="$FOLLOW_DELAY_S"

  if [[ "$WORLD" == baylands* ]]; then
    # Baylands needs a bit more time for Gazebo startup/spawn settling before we
    # derive the UAV-relative spawn and start the rest of the stack.
    SPAWN_DELAY_S=$((SPAWN_DELAY_S + 8))
    LOCALIZATION_DELAY_S=$((SPAWN_DELAY_S + 4))
    NAV2_DELAY_S=$((LOCALIZATION_DELAY_S + 2))
    FOLLOW_DELAY_S=$((NAV2_DELAY_S + 2))
    RECORD_DELAY_S="$FOLLOW_DELAY_S"
  fi

  if [ -n "$SPAWN_DELAY_OVERRIDE" ]; then
    SPAWN_DELAY_S="$SPAWN_DELAY_OVERRIDE"
  fi
  if [ -n "$LOCALIZATION_DELAY_OVERRIDE" ]; then
    LOCALIZATION_DELAY_S="$LOCALIZATION_DELAY_OVERRIDE"
  fi
  if [ -n "$NAV2_DELAY_OVERRIDE" ]; then
    NAV2_DELAY_S="$NAV2_DELAY_OVERRIDE"
  fi
  if [ -n "$FOLLOW_DELAY_OVERRIDE" ]; then
    FOLLOW_DELAY_S="$FOLLOW_DELAY_OVERRIDE"
  fi
  if [ -n "$RECORD_DELAY_OVERRIDE" ]; then
    RECORD_DELAY_S="$RECORD_DELAY_OVERRIDE"
  fi
}

shell_join() {
  local out=""
  local part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

build_line() {
  local delay_s="$1"
  local wait_for_sim="$2"
  local ready_cmd="${3:-}"
  shift 3
  local line=""
  printf -v line 'cd %q && ' "$WS_ROOT"
  printf -v line '%sexport ROS_DOMAIN_ID=%q && export RMW_IMPLEMENTATION=%q && ' "$line" "$ROS_DOMAIN_ID_EFFECTIVE" "$RMW_IMPLEMENTATION_EFFECTIVE"
  if [ "$wait_for_sim" = true ]; then
    printf -v line '%swhile [ ! -f %q ]; do sleep 1; done && ' "$line" "$SIM_PID_FILE"
  fi
  if [ "$delay_s" != "0" ] && [ "$delay_s" != "0.0" ]; then
    printf -v line '%ssleep %q && ' "$line" "$delay_s"
  fi
  if [ "$wait_for_sim" = true ]; then
    printf -v line '%sbash -lc %q && ' "$line" "$(build_gazebo_ready_cmd)"
  fi
  if [ -n "$ready_cmd" ]; then
    printf -v line '%sbash -lc %q && ' "$line" "$ready_cmd"
  fi
  printf -v line '%s%s' "$line" "$(shell_join "$@")"
  printf '%s' "$line"
}

build_gazebo_ready_cmd() {
  cat <<EOF
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_EFFECTIVE"
export RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION_EFFECTIVE"
set -u
deadline=\$((SECONDS + $GAZEBO_READY_TIMEOUT_S))
wait_for_topic_once() {
  local topic="\$1"
  local label="\$2"
  while (( SECONDS < deadline )); do
    if timeout 4s ros2 topic echo --no-daemon --once "\$topic" >/dev/null 2>&1; then
      echo "[gazebo_ready] \$label ready on \$topic"
      return 0
    fi
    echo "[gazebo_ready] waiting for \$label on \$topic"
    sleep 2
  done
  echo "[gazebo_ready] timed out waiting for \$label on \$topic" >&2
  return 1
}
wait_for_topic_once /clock clock
if [ "$GAZEBO_READY_SETTLE_S" != "0" ] && [ "$GAZEBO_READY_SETTLE_S" != "0.0" ]; then
  echo "[gazebo_ready] settling for $GAZEBO_READY_SETTLE_S seconds"
  sleep "$GAZEBO_READY_SETTLE_S"
fi
EOF
}

build_localization_ready_cmd() {
  cat <<EOF
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_EFFECTIVE"
export RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION_EFFECTIVE"
set -u
lifecycle_state_matches() {
  local node="\$1"
  local cli_regex="\$2"
  local service_regex="\$3"
  ros2 lifecycle get "\$node" 2>/dev/null | grep -Eq "\$cli_regex" && return 0
  timeout 3s ros2 service call "\${node}/get_state" lifecycle_msgs/srv/GetState "{}" 2>/dev/null | grep -Eq "\$service_regex"
}
while ! lifecycle_state_matches /$UGV_NAMESPACE/map_server 'active \[3\]' "id=3|label='active'"; do sleep 1; done
# AMCL can be inactive here until nav2 lifecycle manager transitions it.
while ! lifecycle_state_matches /$UGV_NAMESPACE/amcl '(inactive \[2\]|active \[3\])' "id=[23]|label='(inactive|active)'"; do sleep 1; done
EOF
}

build_nav2_ready_cmd() {
  cat <<EOF
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_EFFECTIVE"
export RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION_EFFECTIVE"
set -u
lifecycle_state_matches() {
  local node="\$1"
  local cli_regex="\$2"
  local service_regex="\$3"
  ros2 lifecycle get "\$node" 2>/dev/null | grep -Eq "\$cli_regex" && return 0
  timeout 3s ros2 service call "\${node}/get_state" lifecycle_msgs/srv/GetState "{}" 2>/dev/null | grep -Eq "\$service_regex"
}
while true; do
  if ! lifecycle_state_matches /$UGV_NAMESPACE/map_server 'active \[3\]' "id=3|label='active'"; then
    sleep 1
    continue
  fi
  if ! lifecycle_state_matches /$UGV_NAMESPACE/amcl 'active \[3\]' "id=3|label='active'"; then
    sleep 1
    continue
  fi
  if ! lifecycle_state_matches /$UGV_NAMESPACE/controller_server 'active \[3\]' "id=3|label='active'"; then
    sleep 1
    continue
  fi
  if ! lifecycle_state_matches /$UGV_NAMESPACE/bt_navigator 'active \[3\]' "id=3|label='active'"; then
    sleep 1
    continue
  fi
  if ! ros2 action list 2>/dev/null | grep -qx '/$UGV_NAMESPACE/navigate_to_pose'; then
    sleep 1
    continue
  fi
  break
done
EOF
}

build_omnet_ready_cmd() {
  cat <<EOF
while ! { exec 3<>/dev/tcp/127.0.0.1/$OMNET_BRIDGE_PORT; } 2>/dev/null; do sleep 1; done
exec 3>&-
exec 3<&-
EOF
}

build_wait_for_topic_message_fn() {
  cat <<'EOF'
wait_for_topic_message() {
  local topic="$1"
  local label="$2"
  local timeout_s="${3:-10}"
  echo "[$label] waiting for message on $topic"
  while true; do
    if timeout "${timeout_s}s" ros2 topic echo --once "$topic" >/dev/null 2>&1; then
      echo "[$label] received message on $topic"
      return 0
    fi
    echo "[$label] still waiting for $topic"
    sleep 1
  done
}
EOF
}

build_follow_ready_cmd() {
  local topic=""
  local topics=()

  build_localization_ready_cmd
  if [ -z "$FOLLOW_WAIT_TOPICS" ]; then
    return 0
  fi

  build_wait_for_topic_message_fn
  IFS=',' read -r -a topics <<< "$FOLLOW_WAIT_TOPICS"
  for topic in "${topics[@]}"; do
    topic="$(printf '%s' "$topic" | xargs)"
    [ -n "$topic" ] || continue
    printf "wait_for_topic_message %q 'follow_ready'\n" "$topic"
  done
}

signal_processes_by_pattern() {
  local pattern="$1"
  local pids=()
  local pid=""
  while IFS= read -r pid; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    pids+=("$pid")
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
  if [ "${#pids[@]}" -eq 0 ]; then
    return 0
  fi
  kill -INT "${pids[@]}" 2>/dev/null || true
  sleep 1
  kill -TERM "${pids[@]}" 2>/dev/null || true
  sleep 1
  kill -KILL "${pids[@]}" 2>/dev/null || true
}

signal_named_nodes() {
  local names_regex="$1"
  signal_processes_by_pattern "__node:=($names_regex)(\\s|$)"
}

pane_target() {
  local name="$1"
  local pane_id=""

  pane_id="$(lookup_saved_pane_id "$name" || true)"
  if pane_exists "$pane_id"; then
    printf '%s\n' "$pane_id"
    return 0
  fi

  pane_id="$(tmux list-panes -a -t "$SESSION" -F '#{pane_id}\t#{pane_title}' 2>/dev/null | awk -F '\t' -v want="$name" '$2 == want { print $1; exit }' || true)"
  if [ -n "$pane_id" ]; then
    printf '%s\n' "$pane_id"
    return 0
  fi

  if tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -Fxq "$name"; then
    tmux display-message -p -t "$SESSION:$name.0" '#{pane_id}' 2>/dev/null
    return 0
  fi

  return 1
}

lookup_saved_pane_id() {
  local name="$1"
  case "$name" in
    gazebo) printf '%s\n' "$GAZEBO_PANE_ID" ;;
    spawn) printf '%s\n' "$SPAWN_PANE_ID" ;;
    localization) printf '%s\n' "$LOCALIZATION_PANE_ID" ;;
    nav2) printf '%s\n' "$NAV2_PANE_ID" ;;
    follow) printf '%s\n' "$FOLLOW_PANE_ID" ;;
    omnet) printf '%s\n' "$OMNET_PANE_ID" ;;
    record) printf '%s\n' "$RECORD_PANE_ID" ;;
    *) return 1 ;;
  esac
}

pane_exists() {
  local pane_id="$1"
  [ -n "$pane_id" ] || return 1
  [ "$(tmux display-message -p -t "$pane_id" '#{pane_id}' 2>/dev/null || true)" = "$pane_id" ]
}

load_session_state_if_present() {
  local key=""
  local value=""

  SESSION_SAFE="$(printf '%s' "$SESSION" | tr -c 'A-Za-z0-9_.-' '_')"
  SESSION_STATE_FILE="$TMUX_STATE_DIR/${SESSION_SAFE}.env"
  TMUX_CMD_DIR="/tmp/${SESSION}_tmux_cmds"
  [ -f "$SESSION_STATE_FILE" ] || return 1

  while IFS='=' read -r key value; do
    case "$key" in
      GAZEBO_PANE_ID|SPAWN_PANE_ID|LOCALIZATION_PANE_ID|NAV2_PANE_ID|FOLLOW_PANE_ID|OMNET_PANE_ID|RECORD_PANE_ID)
        eval "$key=$value"
        ;;
    esac
  done < "$SESSION_STATE_FILE"
}

print_pane_lookup_debug() {
  echo "Active panes in $SESSION:" >&2
  tmux list-panes -a -t "$SESSION" -F '  #{pane_id} #{window_name}.#{pane_index} title=#{pane_title} cmd=#{pane_current_command}' >&2 || true
  if [ -f "$SESSION_STATE_FILE" ]; then
    echo "Saved pane IDs from $SESSION_STATE_FILE:" >&2
    echo "  localization=$LOCALIZATION_PANE_ID nav2=$NAV2_PANE_ID follow=$FOLLOW_PANE_ID" >&2
  else
    echo "No saved pane state file found at $SESSION_STATE_FILE" >&2
  fi
}

tmux_session_exists() {
  local session_name="$1"
  tmux has-session -t "$session_name" 2>/dev/null
}

list_active_tmux_sessions() {
  tmux list-sessions -F '#{session_name}' 2>/dev/null || true
}

session_has_1to1_shape() {
  local session_name="$1"
  local pane_titles=""
  local window_names=""

  case "$session_name" in
    *1to1*)
      return 0
      ;;
  esac

  pane_titles="$(tmux list-panes -a -t "$session_name" -F '#{pane_title}' 2>/dev/null || true)"
  if printf '%s\n' "$pane_titles" | grep -Eq '^(localization|nav2|follow)$'; then
    return 0
  fi

  window_names="$(tmux list-windows -t "$session_name" -F '#{window_name}' 2>/dev/null || true)"
  if printf '%s\n' "$window_names" | grep -Eq '^(localization|nav2|follow)$'; then
    return 0
  fi

  return 1
}

discover_single_likely_session() {
  local active_session=""
  local matches=()

  while IFS= read -r active_session; do
    [ -n "$active_session" ] || continue
    if session_has_1to1_shape "$active_session"; then
      matches+=("$active_session")
    fi
  done < <(list_active_tmux_sessions)

  if [ "${#matches[@]}" -eq 1 ]; then
    printf '%s\n' "${matches[0]}"
    return 0
  fi
  return 1
}

print_missing_session_help() {
  local requested_action="$1"
  local active_sessions=()
  local active_session=""

  echo "tmux session not found: $SESSION" >&2
  while IFS= read -r active_session; do
    [ -n "$active_session" ] || continue
    active_sessions+=("$active_session")
  done < <(list_active_tmux_sessions)

  if [ "${#active_sessions[@]}" -eq 0 ]; then
    echo "No active tmux sessions exist. If you ran './stop.sh tmux_1to1', it kills the tmux session by default." >&2
    echo "Use a cold start now: ./run.sh tmux_1to1 start $WORLD [args...]" >&2
  else
    echo "Active tmux sessions:" >&2
    printf '  - %s\n' "${active_sessions[@]}" >&2
    echo "If one of these is the target, rerun with: session:=<name>" >&2
  fi

  echo "For next time, keep panes reusable with: ./stop.sh tmux_1to1 session:=$SESSION kill_session:=false" >&2
  if [ "$requested_action" = "status" ]; then
    echo "After a full stop, there is no tmux session to inspect; use 'start' first." >&2
  elif [ "$requested_action" = "attach" ]; then
    echo "After a full stop, there is no tmux session to attach to; use 'start' first." >&2
  elif [ "$requested_action" = "follow" ] || [ "$requested_action" = "follow_restart" ]; then
    echo "After a full stop, 'follow' cannot restart anything; use 'start' first." >&2
  else
    echo "After a full stop, '$requested_action' cannot restart anything; use 'start' first." >&2
  fi
}

resolve_existing_session_or_fail() {
  local requested_action="$1"
  local discovered_session=""

  if tmux_session_exists "$SESSION"; then
    return 0
  fi

  if [ "$SESSION_EXPLICIT" != true ]; then
    discovered_session="$(discover_single_likely_session || true)"
    if [ -n "$discovered_session" ]; then
      echo "[run_tmux_1to1] Using active tmux session: $discovered_session"
      SESSION="$discovered_session"
      TMUX_CMD_DIR="/tmp/${SESSION}_tmux_cmds"
      return 0
    fi
  fi

  print_missing_session_help "$requested_action"
  return 1
}

send_ctrl_c_target() {
  local name="$1"
  local pane_id=""
  pane_id="$(pane_target "$name" || true)"
  if [ -n "$pane_id" ]; then
    echo "[run_tmux_1to1] Stopping $name pane ($pane_id)"
    tmux send-keys -t "$pane_id" C-c
  fi
}

stop_stack_processes() {
  send_ctrl_c_target follow
  send_ctrl_c_target nav2
  send_ctrl_c_target localization
  sleep 1

  signal_processes_by_pattern 'scripts/run_localization\.sh'
  signal_processes_by_pattern 'scripts/run_nav2\.sh'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_1to1_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch .*/run_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch clearpath_nav2_demos nav2\.launch\.py'
  signal_processes_by_pattern 'ros2 launch .*/nav2_with_updates\.launch\.py'
  signal_processes_by_pattern '/opt/ros/[^/]+/lib/nav2_'
  signal_processes_by_pattern 'ros2 launch clearpath_nav2_demos localization\.launch\.py'
  signal_processes_by_pattern 'ros2 launch .*/localization_with_params\.launch\.py'
  signal_processes_by_pattern '(^|/)pointcloud_to_laserscan_node($| )'
  signal_processes_by_pattern '(^|/)latest_scan_relay($| )'
  signal_named_nodes 'amcl|map_server|planner_server|controller_server|collision_monitor|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|route_server|docking_server|lifecycle_manager_localization|lifecycle_manager_navigation|pointcloud_to_laserscan|latest_scan_relay|ugv_nav2_driver|ugv_amcl_to_odom|ugv_amcl_to_platform_odom|ugv_amcl_to_platform_filtered_odom|ugv_platform_odom_to_tf|follow_uav|follow_uav_odom|leader_detector|leader_tracker|leader_estimator|selected_target_filter|visual_target_estimator|follow_point_generator|follow_point_planner|visual_actuation_bridge|camera_tracker'
}

prelaunch_safety_cleanup() {
  rm -f "$SIM_PID_FILE"
  signal_processes_by_pattern 'scripts/run_gazebo_sim\.sh'
  signal_processes_by_pattern 'scripts/run_spawn_uav\.sh'
  signal_processes_by_pattern 'scripts/run_localization\.sh'
  signal_processes_by_pattern 'scripts/run_nav2\.sh'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_1to1_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch .*/run_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch clearpath_nav2_demos nav2\.launch\.py'
  signal_processes_by_pattern 'ros2 launch .*/nav2_with_updates\.launch\.py'
  signal_processes_by_pattern '/opt/ros/[^/]+/lib/nav2_'
  signal_processes_by_pattern 'ros2 launch clearpath_nav2_demos localization\.launch\.py'
  signal_processes_by_pattern 'ros2 launch .*/localization_with_params\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad spawn_uav_1to1\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad managed_clearpath_sim\.launch\.py'
  signal_processes_by_pattern 'ros2 launch .*/managed_clearpath_sim\.launch\.py'
  signal_processes_by_pattern '/ros_gz_bridge/(bridge_node|parameter_bridge|image_bridge)(\\s|$)'
  signal_named_nodes 'amcl|map_server|planner_server|controller_server|collision_monitor|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|route_server|docking_server|lifecycle_manager_localization|lifecycle_manager_navigation|ugv_nav2_driver|ugv_amcl_to_odom|ugv_amcl_to_platform_odom|ugv_amcl_to_platform_filtered_odom|ugv_platform_odom_to_tf|uav_simulator|follow_uav|follow_uav_odom|leader_detector|leader_tracker|leader_estimator|selected_target_filter|visual_target_estimator|follow_point_generator|follow_point_planner|visual_actuation_bridge|camera_tracker|clock_bridge|clock_guard|omnet_uav_pose_to_odom|omnet_tcp_bridge|omnet_metrics_bridge'
  signal_processes_by_pattern '(^|/)UAV_UGV($| ).*-c Communication-GazeboBridge-'
  signal_processes_by_pattern '(^|/)gz sim($| )'
}

configure_gazebo_gpu_env() {
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="${LRS_SIM_GPU_ADAPTER:-AMD Radeon RX 7600}"
  export GALLIUM_DRIVER=d3d12

  unset LIBGL_ALWAYS_SOFTWARE
  unset MESA_LOADER_DRIVER_OVERRIDE
  unset GALLIUM_DRIVER_LLVM
  unset vblank_mode
  unset MESA_VK_WSI_PRESENT_MODE
}

TMUX_CMD_DIR="/tmp/${SESSION}_tmux_cmds"

send_script_to_pane() {
  local pane="$1"
  local name="$2"
  local command="$3"
  local script="$TMUX_CMD_DIR/${name}.sh"

  mkdir -p "$TMUX_CMD_DIR"
  cat > "$script" <<EOF
#!/usr/bin/env bash
set -e
cd /home/ruben/halmstad_ws
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset PYTHONNOUSERSITE

$command
EOF

  chmod +x "$script"

  tmux send-keys -t "$pane" C-c
  tmux send-keys -t "$pane" "clear; bash '$script'" C-m
}


write_session_state() {
  mkdir -p "$TMUX_STATE_DIR"
  local _record_cmd_str=""
  if [ "${#RECORD_CMD[@]}" -gt 0 ]; then
    _record_cmd_str="$(shell_join "${RECORD_CMD[@]}")"
  fi
  {
    printf 'SESSION=%q\n' "$SESSION"
    printf 'LAYOUT=%q\n' "$LAYOUT"
    printf 'EFFECTIVE_GUI=%q\n' "$EFFECTIVE_GUI"
    printf 'WORLD=%q\n' "$WORLD"
    printf 'MODE=%q\n' "$MODE"
    printf 'UAV_NAME=%q\n' "$UAV_NAME"
    printf 'UGV_NAMESPACE=%q\n' "$UGV_NAMESPACE"
    printf 'ROS_DOMAIN_ID_EFFECTIVE=%q\n' "$ROS_DOMAIN_ID_EFFECTIVE"
    printf 'RMW_IMPLEMENTATION_EFFECTIVE=%q\n' "$RMW_IMPLEMENTATION_EFFECTIVE"
    printf 'RECORD=%q\n' "$RECORD"
    printf 'OMNET=%q\n' "$OMNET"
    printf 'OMNET_NETWORK=%q\n' "$OMNET_NETWORK"
    printf 'OMNET_UI=%q\n' "$OMNET_UI"
    printf 'OMNET_PROJECT=%q\n' "$OMNET_PROJECT"
    printf 'OMNET_RESULT_DIR=%q\n' "$OMNET_RESULT_DIR"
    printf 'OMNET_LORA_SF=%q\n' "$OMNET_LORA_SF"
    printf 'OMNET_LORA_BW=%q\n' "$OMNET_LORA_BW"
    printf 'FOLLOW_CMD_STR=%q\n' "$(shell_join "${FOLLOW_CMD[@]}")"
    printf 'RECORD_CMD_STR=%q\n' "$_record_cmd_str"
    printf 'GAZEBO_PANE_ID=%q\n' "$gazebo_pane"
    printf 'SPAWN_PANE_ID=%q\n' "$spawn_pane"
    printf 'LOCALIZATION_PANE_ID=%q\n' "$localization_pane"
    printf 'NAV2_PANE_ID=%q\n' "$nav2_pane"
    printf 'FOLLOW_PANE_ID=%q\n' "$follow_pane"
    printf 'OMNET_PANE_ID=%q\n' "$omnet_pane"
    printf 'RECORD_PANE_ID=%q\n' "$record_pane"
  } > "$SESSION_STATE_FILE"
}

apply_default_delays

FOLLOW_ARGS+=("start_omnet_bridge:=$OMNET")
if [ "$HAVE_RANGE_MODE" != true ]; then
  if [ "$OMNET" = true ]; then
    FOLLOW_ARGS+=("range_mode:=radio")
  else
    FOLLOW_ARGS+=("range_mode:=auto")
  fi
fi
shared_start_delay_s="$DEFAULT_OMNET_START_DELAY_S"
if [ -n "$OMNET_START_DELAY_OVERRIDE" ]; then
  shared_start_delay_s="$OMNET_START_DELAY_OVERRIDE"
fi
if [ -n "$UGV_START_DELAY_OVERRIDE" ]; then
  FOLLOW_ARGS+=("ugv_start_delay_s:=$UGV_START_DELAY_OVERRIDE")
elif [ "$OMNET" = true ]; then
  FOLLOW_ARGS+=("ugv_start_delay_s:=$shared_start_delay_s")
fi
if [ -n "$UAV_START_DELAY_OVERRIDE" ]; then
  FOLLOW_ARGS+=("uav_start_delay_s:=$UAV_START_DELAY_OVERRIDE")
else
  FOLLOW_ARGS+=("uav_start_delay_s:=$DEFAULT_UAV_START_DELAY_S")
fi

if [[ "$WORLD" == baylands* ]] && [ "$HAVE_UGV_GOAL_SEQUENCE" = "false" ]; then
  if [ -n "$GAZEBO_WAYPOINT_NAME" ]; then
    inferred_nav2_goals="$(baylands_route_for_waypoint "$GAZEBO_WAYPOINT_NAME" 2>/dev/null || true)"
    if [ -n "$inferred_nav2_goals" ]; then
      FOLLOW_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "$inferred_nav2_goals")")
      NAV2_GOALS_FOR_LIDAR="$inferred_nav2_goals"
      echo "[run_tmux_1to1] Baylands waypoint implies Nav2 goals: waypoint:=$GAZEBO_WAYPOINT_NAME nav2_goals:=$inferred_nav2_goals"
    fi
  fi
  if [ -z "$NAV2_GOALS_FOR_LIDAR" ]; then
    FOLLOW_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "$BAYLANDS_DEFAULT_NAV2_GOALS")")
    NAV2_GOALS_FOR_LIDAR="$BAYLANDS_DEFAULT_NAV2_GOALS"
    echo "[run_tmux_1to1] Baylands default Nav2 goals: nav2_goals:=$BAYLANDS_DEFAULT_NAV2_GOALS"
  fi
fi

if [[ "$WORLD" == baylands* ]] && [ "$HAS_GAZEBO_SPAWN_OVERRIDE" = "false" ]; then
  route_start_waypoint=""
  if [ -n "$NAV2_GOALS_FOR_LIDAR" ]; then
    route_start_waypoint="$(baylands_first_waypoint_for_route "$NAV2_GOALS_FOR_LIDAR" 2>/dev/null || true)"
    if [ -z "$route_start_waypoint" ]; then
      echo "[run_tmux_1to1] Could not resolve first waypoint for nav2_goals:=$NAV2_GOALS_FOR_LIDAR" >&2
      echo "[run_tmux_1to1] Pass waypoint:=... explicitly for this route." >&2
      exit 2
    fi
  fi
  GAZEBO_WAYPOINT_NAME="${route_start_waypoint:-$BAYLANDS_DEFAULT_WAYPOINT}"
  HAS_GAZEBO_SPAWN_OVERRIDE="true"
  GAZEBO_ARGS+=("waypoint:=$GAZEBO_WAYPOINT_NAME")
  echo "[run_tmux_1to1] Baylands UGV spawn waypoint: waypoint:=$GAZEBO_WAYPOINT_NAME"
fi

UGV_SPAWN_X=""
UGV_SPAWN_Y=""
UGV_SPAWN_Z=""
UGV_SPAWN_YAW=""
if [ -n "$GAZEBO_SPAWN_STATE_NAME" ]; then
  METADATA_PATH="$(slam_metadata_path_for_name "$WS_ROOT" "$GAZEBO_SPAWN_STATE_NAME")"
  if [ ! -f "$METADATA_PATH" ]; then
    echo "[run_tmux_1to1] Saved SLAM state metadata not found: $METADATA_PATH" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  source "$METADATA_PATH"
  UGV_SPAWN_X="${spawn_x:-}"
  UGV_SPAWN_Y="${spawn_y:-}"
  UGV_SPAWN_Z="${spawn_z:-}"
  UGV_SPAWN_YAW="${spawn_yaw:-}"
elif [ -n "$GAZEBO_WAYPOINT_NAME" ]; then
  if [[ "$WORLD" != baylands* ]]; then
    echo "[run_tmux_1to1] waypoint:=... is currently supported for Baylands only." >&2
    exit 2
  fi
  WAYPOINT_ENV="$(resolve_baylands_waypoint "$GAZEBO_WAYPOINT_NAME")" || {
    echo "[run_tmux_1to1] Failed to resolve waypoint '$GAZEBO_WAYPOINT_NAME'." >&2
    exit 1
  }
  eval "$WAYPOINT_ENV"
  UGV_SPAWN_X="${spawn_x:-}"
  UGV_SPAWN_Y="${spawn_y:-}"
  UGV_SPAWN_Z="${spawn_z:-}"
  UGV_SPAWN_YAW="${spawn_yaw:-}"
elif [ -n "$GAZEBO_SPAWN_X_OVERRIDE" ] || [ -n "$GAZEBO_SPAWN_Y_OVERRIDE" ] || [ -n "$GAZEBO_SPAWN_YAW_OVERRIDE" ]; then
  UGV_SPAWN_X="$GAZEBO_SPAWN_X_OVERRIDE"
  UGV_SPAWN_Y="$GAZEBO_SPAWN_Y_OVERRIDE"
  UGV_SPAWN_Z="$GAZEBO_SPAWN_Z_OVERRIDE"
  UGV_SPAWN_YAW="$GAZEBO_SPAWN_YAW_OVERRIDE"
elif [[ "$WORLD" == baylands* ]]; then
  UGV_SPAWN_X="$BAYLANDS_NAV_SPAWN_X"
  UGV_SPAWN_Y="$BAYLANDS_NAV_SPAWN_Y"
  UGV_SPAWN_Z="$BAYLANDS_NAV_SPAWN_Z"
  UGV_SPAWN_YAW="$BAYLANDS_NAV_SPAWN_YAW"
fi

if [ -n "$UGV_SPAWN_X" ] && [ -n "$UGV_SPAWN_Y" ] && [ -n "$UGV_SPAWN_YAW" ]; then
  EFFECTIVE_UAV_Z="${UAV_HEIGHT_OVERRIDE:-$DEFAULT_UAV_Z}"
  UAV_SPAWN_ENV="$(compute_uav_spawn_from_ugv_pose_env \
    "$UGV_SPAWN_X" \
    "$UGV_SPAWN_Y" \
    "${UGV_SPAWN_Z:-0.0}" \
    "$UGV_SPAWN_YAW" \
    "$DEFAULT_UAV_BODY_X_OFFSET" \
    "$DEFAULT_UAV_BODY_Y_OFFSET" \
    "$EFFECTIVE_UAV_Z")" || {
    echo "[run_tmux_1to1] Failed to compute UAV-relative spawn from the selected UGV spawn pose." >&2
    exit 1
  }
  eval "$UAV_SPAWN_ENV"
  SPAWN_ARGS+=("x:=$uav_x" "y:=$uav_y" "z:=$uav_z" "yaw:=$uav_yaw")
  if [[ "$WORLD" != baylands* ]] || [ "$MODE" = "yolo" ]; then
    FOLLOW_ARGS+=(
      "uav_start_x:=$uav_x"
      "uav_start_y:=$uav_y"
      "uav_start_z:=$uav_z"
      "uav_start_yaw_deg:=$uav_yaw_deg"
    )
  fi
  echo "[run_tmux_1to1] Using deterministic UAV spawn from UGV spawn x=${UGV_SPAWN_X} y=${UGV_SPAWN_Y} yaw=${UGV_SPAWN_YAW}: uav_x=${uav_x} uav_y=${uav_y} uav_z=${uav_z} uav_yaw_deg=${uav_yaw_deg}"
  if [[ "$WORLD" == baylands* ]]; then
    if [ "$MODE" = "yolo" ]; then
      echo "[run_tmux_1to1] Baylands YOLO follow will use the deterministic UAV spawn as its simulator start pose."
    else
      echo "[run_tmux_1to1] Baylands follow start will be resolved from the live UAV pose by run_1to1_follow."
    fi
  fi
fi

GAZEBO_CMD=(./run.sh gazebo_sim "$WORLD")
if [ -n "$GUI" ]; then
  GAZEBO_CMD+=("$GUI")
fi
if [ "${#GAZEBO_ARGS[@]}" -gt 0 ]; then
  GAZEBO_CMD+=("${GAZEBO_ARGS[@]}")
fi

SPAWN_CMD=(./run.sh spawn_uav "$WORLD" "${SPAWN_ARGS[@]}")
RESTART_REALIGN_CMD=()
if [ "$WAYPOINT_EXPLICIT" = "true" ]; then
  RESTART_REALIGN_CMD=(
    ./run.sh realign_yaw "$WORLD"
    "waypoint:=$GAZEBO_WAYPOINT_NAME"
    "with_uav:=true"
    "uav_name:=$UAV_NAME"
    "height:=${UAV_HEIGHT_OVERRIDE:-$DEFAULT_UAV_Z}"
  )
fi
LOCALIZATION_CMD=(./run.sh localization "$WORLD")
if [ -n "$MAP_PATH" ]; then
  LOCALIZATION_CMD+=("$MAP_PATH")
fi
NAV2_CMD=(./run.sh nav2)
EFFECTIVE_NAV_LIDAR_MODE="$NAV_LIDAR_MODE"
if [ -z "$EFFECTIVE_NAV_LIDAR_MODE" ] && [[ "$WORLD" == baylands* ]]; then
  EFFECTIVE_NAV_LIDAR_MODE="$BAYLANDS_NAV_LIDAR_MODE"
fi
if [ -n "$EFFECTIVE_NAV_LIDAR_MODE" ]; then
  LOCALIZATION_CMD+=("lidar:=$EFFECTIVE_NAV_LIDAR_MODE")
  NAV2_CMD+=("lidar:=$EFFECTIVE_NAV_LIDAR_MODE")
fi
if [ "${#NAV_LIDAR_ARGS[@]}" -gt 0 ]; then
  LOCALIZATION_CMD+=("${NAV_LIDAR_ARGS[@]}")
  for nav_lidar_arg in "${NAV_LIDAR_ARGS[@]}"; do
    case "$nav_lidar_arg" in
      pc2ls_*|pointcloud_topic:=*)
        ;;
      *)
        NAV2_CMD+=("$nav_lidar_arg")
        ;;
    esac
  done
fi
if [[ "$WORLD" == baylands* ]] && [ -n "$NAV2_GOALS_FOR_LIDAR" ]; then
  mapfile -t route_default_lidar_args < <(route_lidar_preset_args "$NAV2_GOALS_FOR_LIDAR" "${EFFECTIVE_NAV_LIDAR_MODE:-}" "${NAV_LIDAR_ARGS[@]}")
  if [ "${#route_default_lidar_args[@]}" -gt 0 ]; then
    LOCALIZATION_CMD+=("${route_default_lidar_args[@]}")
  fi
fi
if [ "$MODE" = "yolo" ]; then
  FOLLOW_CMD=(./run.sh 1to1_yolo "$WORLD" "${FOLLOW_ARGS[@]}")
else
FOLLOW_CMD=(./run.sh 1to1_follow "$WORLD" "${FOLLOW_ARGS[@]}")
fi

LOCALIZATION_READY_CMD="$(build_localization_ready_cmd)"
NAV2_READY_CMD="$(build_nav2_ready_cmd)"
FOLLOW_READY_CMD="$(build_follow_ready_cmd)"
if [ "$RECORD" = true ]; then
  RECORD_CMD=(./run.sh record_experiment "$WORLD" "mode:=$MODE" "uav_name:=$UAV_NAME" "profile:=$RECORD_PROFILE")
  RECORD_CMD+=("omnet:=$OMNET")
  if [ -n "$RECORD_TAG" ]; then
    RECORD_CMD+=("tag:=$RECORD_TAG")
  fi
  if [ -n "$RECORD_OUT" ]; then
    RECORD_CMD+=("out:=$RECORD_OUT")
  fi
fi

if [ "$OMNET" = true ]; then
  OMNET_CMD=(./run.sh omnet "network:=$OMNET_NETWORK" "ui:=$OMNET_UI")
  if [ -n "$OMNET_PROJECT" ]; then
    OMNET_CMD+=("project:=$OMNET_PROJECT")
  fi
  if [ -n "$OMNET_RESULT_DIR" ]; then
    OMNET_CMD+=("result_dir:=$OMNET_RESULT_DIR")
  fi
  if [ -n "$OMNET_LORA_SF" ]; then
    OMNET_CMD+=("lora_sf:=$OMNET_LORA_SF")
  fi
  if [ -n "$OMNET_LORA_BW" ]; then
    OMNET_CMD+=("lora_bw:=$OMNET_LORA_BW")
  fi
  OMNET_READY_CMD="$(build_omnet_ready_cmd)"
fi

GAZEBO_LINE="$(build_line 0 false "" "${GAZEBO_CMD[@]}")"
SPAWN_LINE="$(build_line "$SPAWN_DELAY_S" true "" "${SPAWN_CMD[@]}")"
LOCALIZATION_LINE="$(build_line "$LOCALIZATION_DELAY_S" true "" "${LOCALIZATION_CMD[@]}")"
NAV2_LINE="$(build_line "$NAV2_DELAY_S" true "$LOCALIZATION_READY_CMD" "${NAV2_CMD[@]}")"
FOLLOW_LINE="$(build_line "$FOLLOW_DELAY_S" true "$FOLLOW_READY_CMD" "${FOLLOW_CMD[@]}")"
STACK_LOCALIZATION_LINE="$(build_line 0 false "" "${LOCALIZATION_CMD[@]}")"
STACK_NAV2_LINE="$(build_line 0 false "$LOCALIZATION_READY_CMD" "${NAV2_CMD[@]}")"
STACK_FOLLOW_LINE="$(build_line 0 false "$NAV2_READY_CMD" "${FOLLOW_CMD[@]}")"
RESTART_REALIGN_LINE=""
if [ "${#RESTART_REALIGN_CMD[@]}" -gt 0 ]; then
  RESTART_REALIGN_LINE="$(build_line 0 false "" "${RESTART_REALIGN_CMD[@]}")"
fi
if [ "$OMNET" = true ]; then
  OMNET_LINE="$(build_line "$FOLLOW_DELAY_S" true "$OMNET_READY_CMD" "${OMNET_CMD[@]}")"
fi
if [ "$RECORD" = true ]; then
  RECORD_LINE="$(build_line "$RECORD_DELAY_S" true "$LOCALIZATION_READY_CMD" "${RECORD_CMD[@]}")"
fi

case "$ACTION" in
  restart|stack_restart|follow|follow_restart|status|attach)
    resolve_existing_session_or_fail "$ACTION"
    load_session_state_if_present || true
    ;;
esac

if [ "$DRY_RUN" = true ]; then
  echo "Session: $SESSION"
  echo "Action: $ACTION"
  echo "Mode: $MODE"
  echo "Layout: $LAYOUT"
  echo "GUI: $EFFECTIVE_GUI"
  echo "Lidar: ${EFFECTIVE_NAV_LIDAR_MODE:-default}"
  echo "Record: $RECORD"
  echo "Base delay: $BASE_DELAY_S"
  echo "Overrides: spawn=${SPAWN_DELAY_OVERRIDE:-default} localization=${LOCALIZATION_DELAY_OVERRIDE:-default} nav2=${NAV2_DELAY_OVERRIDE:-default} follow=${FOLLOW_DELAY_OVERRIDE:-default} record=${RECORD_DELAY_OVERRIDE:-default}"
  if [ "$OMNET" = true ] || [ -n "$OMNET_START_DELAY_OVERRIDE" ] || [ -n "$UGV_START_DELAY_OVERRIDE" ] || [ -n "$UAV_START_DELAY_OVERRIDE" ]; then
    echo "Startup holds: shared=${OMNET_START_DELAY_OVERRIDE:-$DEFAULT_OMNET_START_DELAY_S} ugv=${UGV_START_DELAY_OVERRIDE:-${OMNET:+$shared_start_delay_s}} uav=${UAV_START_DELAY_OVERRIDE:-$DEFAULT_UAV_START_DELAY_S}"
  fi
  echo "Delays: spawn=$SPAWN_DELAY_S localization=$LOCALIZATION_DELAY_S nav2=$NAV2_DELAY_S follow=$FOLLOW_DELAY_S record=$RECORD_DELAY_S"
  case "$ACTION" in
    restart|stack_restart)
      if [ -n "$RESTART_REALIGN_LINE" ]; then
        echo "[realign restart]      $RESTART_REALIGN_LINE"
      fi
      echo "[localization restart] $STACK_LOCALIZATION_LINE"
      echo "[nav2 restart]         $STACK_NAV2_LINE"
      echo "[follow restart]       $STACK_FOLLOW_LINE"
      ;;
    follow|follow_restart)
      echo "[follow restart]       $STACK_FOLLOW_LINE"
      ;;
    *)
      echo "[gazebo]       $GAZEBO_LINE"
      echo "[spawn]        $SPAWN_LINE"
      echo "[localization] $LOCALIZATION_LINE"
      echo "[nav2]         $NAV2_LINE"
      echo "[follow]       $FOLLOW_LINE"
      if [ "$OMNET" = true ]; then
        echo "[omnet]        $OMNET_LINE"
      fi
      if [ "$RECORD" = true ]; then
        echo "[record]       $RECORD_LINE"
      fi
      ;;
  esac
  exit 0
fi

configure_gazebo_gpu_env

case "$ACTION" in
  start)
    if tmux has-session -t "$SESSION" 2>/dev/null; then
      echo "tmux session already exists: $SESSION" >&2
      echo "Restart stack with: ./run.sh tmux_1to1 restart session:=$SESSION [new args...]" >&2
      echo "Restart follow with: ./run.sh tmux_1to1 follow session:=$SESSION [new args...]" >&2
      echo "Attach with: tmux attach -t $SESSION" >&2
      exit 1
    fi
    prelaunch_safety_cleanup
    ;;
  restart|stack_restart)
    localization_pane="$(pane_target localization || true)"
    nav2_pane="$(pane_target nav2 || true)"
    follow_pane="$(pane_target follow || true)"
    if [ -z "$localization_pane" ] || [ -z "$nav2_pane" ] || [ -z "$follow_pane" ]; then
      echo "Could not find localization/nav2/follow panes in tmux session: $SESSION" >&2
      print_pane_lookup_debug
      exit 1
    fi
    stop_stack_processes
    if [ "${#RESTART_REALIGN_CMD[@]}" -gt 0 ]; then
      echo "[run_tmux_1to1] Realigning Gazebo models before stack restart: waypoint:=$GAZEBO_WAYPOINT_NAME with_uav:=true"
      "${RESTART_REALIGN_CMD[@]}"
    fi
    send_script_to_pane "$localization_pane" "localization" "$STACK_LOCALIZATION_LINE"
    send_script_to_pane "$nav2_pane" "nav2" "$STACK_NAV2_LINE"
    send_script_to_pane "$follow_pane" "follow" "$STACK_FOLLOW_LINE"
    echo "Restarted localization/nav2/follow in tmux session: $SESSION"
    exit 0
    ;;
  follow|follow_restart)
    follow_pane="$(pane_target follow || true)"
    if [ -z "$follow_pane" ]; then
      echo "Could not find follow pane in tmux session: $SESSION" >&2
      print_pane_lookup_debug
      exit 1
    fi
    send_ctrl_c_target follow
    sleep 1
    signal_processes_by_pattern 'ros2 launch lrs_halmstad run_follow\.launch\.py'
    signal_processes_by_pattern 'ros2 launch lrs_halmstad run_1to1_follow\.launch\.py'
    signal_processes_by_pattern 'ros2 launch .*/run_follow\.launch\.py'
    signal_named_nodes 'ugv_nav2_driver|follow_uav|follow_uav_odom|leader_detector|leader_tracker|leader_estimator|selected_target_filter|visual_target_estimator|follow_point_generator|follow_point_planner|visual_actuation_bridge|camera_tracker'
    send_script_to_pane "$follow_pane" "follow" "$STACK_FOLLOW_LINE"
    echo "Restarted follow in tmux session: $SESSION"
    exit 0
    ;;
  status)
    tmux list-panes -a -t "$SESSION" -F '#{session_name}:#{window_name}.#{pane_index} #{pane_id} #{pane_title} #{pane_current_command}'
    exit 0
    ;;
  attach)
    exec tmux attach -t "$SESSION"
    ;;
  *)
    echo "Invalid action: $ACTION" >&2
    echo "Use: start, restart, follow, status, or attach" >&2
    exit 2
    ;;
esac

if [ "$LAYOUT" = "windows" ]; then
  tmux new-session -d -s "$SESSION" -n gazebo
  tmux set-environment -g MESA_D3D12_DEFAULT_ADAPTER_NAME "$MESA_D3D12_DEFAULT_ADAPTER_NAME"
  tmux set-environment -g GALLIUM_DRIVER "$GALLIUM_DRIVER"
  tmux set-environment -gu LIBGL_ALWAYS_SOFTWARE 2>/dev/null || true
  tmux set-environment -gu MESA_LOADER_DRIVER_OVERRIDE 2>/dev/null || true
  tmux set-environment -gu GALLIUM_DRIVER_LLVM 2>/dev/null || true
  tmux set-environment -gu vblank_mode 2>/dev/null || true
  tmux set-environment -gu MESA_VK_WSI_PRESENT_MODE 2>/dev/null || true
  tmux set-environment -gu PYTHONNOUSERSITE 2>/dev/null || true
  tmux new-window -t "$SESSION" -n spawn
  tmux new-window -t "$SESSION" -n localization
  tmux new-window -t "$SESSION" -n nav2
  tmux new-window -t "$SESSION" -n follow
  if [ "$OMNET" = true ]; then
    tmux new-window -t "$SESSION" -n omnet
  fi
  if [ "$RECORD" = true ]; then
    tmux new-window -t "$SESSION" -n record
  fi

  tmux setw -t "$SESSION" automatic-rename off
  tmux setw -t "$SESSION" allow-rename off

  gazebo_pane="$(tmux display-message -p -t "$SESSION:gazebo" '#{pane_id}')"
  spawn_pane="$(tmux display-message -p -t "$SESSION:spawn" '#{pane_id}')"
  localization_pane="$(tmux display-message -p -t "$SESSION:localization" '#{pane_id}')"
  nav2_pane="$(tmux display-message -p -t "$SESSION:nav2" '#{pane_id}')"
  follow_pane="$(tmux display-message -p -t "$SESSION:follow" '#{pane_id}')"
  if [ "$OMNET" = true ]; then
    omnet_pane="$(tmux display-message -p -t "$SESSION:omnet" '#{pane_id}')"
  fi
  if [ "$RECORD" = true ]; then
    record_pane="$(tmux display-message -p -t "$SESSION:record" '#{pane_id}')"
  fi

  write_session_state

  send_script_to_pane "$SESSION:gazebo" "gazebo" "$GAZEBO_LINE" C-m
  send_script_to_pane "$SESSION:spawn" "spawn" "$SPAWN_LINE" C-m
  send_script_to_pane "$SESSION:localization" "localization" "$LOCALIZATION_LINE" C-m
  send_script_to_pane "$SESSION:nav2" "nav2" "$NAV2_LINE" C-m
  send_script_to_pane "$SESSION:follow" "follow" "$FOLLOW_LINE" C-m
  if [ "$OMNET" = true ]; then
    send_script_to_pane "$SESSION:omnet" "omnet" "$OMNET_LINE" C-m
  fi
  if [ "$RECORD" = true ]; then
    send_script_to_pane "$SESSION:record" "record" "$RECORD_LINE" C-m
  fi
  tmux select-window -t "$SESSION:gazebo"
else
  tmux new-session -d -s "$SESSION" -n sim
  gazebo_pane="$(tmux display-message -p -t "$SESSION:sim.0" '#{pane_id}')"
  follow_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$gazebo_pane" -v -l 34%)"
  localization_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$gazebo_pane" -v -l 50%)"
  spawn_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$gazebo_pane" -h -l 50%)"
  nav2_pane="$(tmux split-window -d -P -F '#{pane_id}' -t "$localization_pane" -h -l 50%)"
  if [ "$OMNET" = true ]; then
    tmux new-window -t "$SESSION" -n omnet
    omnet_pane="$(tmux display-message -p -t "$SESSION:omnet" '#{pane_id}')"
  fi

  tmux select-pane -t "$gazebo_pane" -T gazebo
  tmux select-pane -t "$spawn_pane" -T spawn
  tmux select-pane -t "$localization_pane" -T localization
  tmux select-pane -t "$nav2_pane" -T nav2
  tmux select-pane -t "$follow_pane" -T follow
  if [ "$OMNET" = true ]; then
    tmux select-pane -t "$omnet_pane" -T omnet
  fi
  if [ "$RECORD" = true ]; then
    tmux new-window -t "$SESSION" -n record
    record_pane="$(tmux display-message -p -t "$SESSION:record" '#{pane_id}')"
    tmux select-pane -t "$record_pane" -T record
  fi

  write_session_state

  send_script_to_pane "$gazebo_pane" "gazebo" "$GAZEBO_LINE"
  send_script_to_pane "$spawn_pane" "spawn" "$SPAWN_LINE"
  send_script_to_pane "$localization_pane" "localization" "$LOCALIZATION_LINE"
  send_script_to_pane "$nav2_pane" "nav2" "$NAV2_LINE"
  send_script_to_pane "$follow_pane" "follow" "$FOLLOW_LINE"
  if [ "$OMNET" = true ]; then
    send_script_to_pane "$omnet_pane" "omnet" "$OMNET_LINE" C-m
  fi
  if [ "$RECORD" = true ]; then
    send_script_to_pane "$record_pane" "record" "$RECORD_LINE" C-m
  fi
  tmux select-window -t "$SESSION:sim"
  tmux select-pane -t "$gazebo_pane"
fi

if [ "$TMUX_ATTACH" = true ]; then
  exec tmux attach -t "$SESSION"
fi

echo "Started tmux session: $SESSION"
echo "Attach with: tmux attach -t $SESSION"

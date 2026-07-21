#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
TMUX_STATE_DIR="$STATE_DIR/tmux_sessions"
WORLD="baylands"
SESSION="halmstad-1to1"
MAP_PATH=""
GUI="false"
TMUX_ATTACH=true
DRY_RUN=false
LAYOUT="panes"
MODE="follow"
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
UGV_NAMESPACE="a201_0000"
OMNET_NETWORK="wifi"
OMNET_UI="cmdenv"
OMNET_PROJECT=""
OMNET_RESULT_DIR=""
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
BAYLANDS_DEFAULT_WAYPOINT="parkinglot_west_0"
BAYLANDS_DEFAULT_NAV2_GOALS="parkinglot_west"
BAYLANDS_NAV_SPAWN_X="-14.085738068"
BAYLANDS_NAV_SPAWN_Y="-54.861874768"
BAYLANDS_NAV_SPAWN_Z="0.100975479"
BAYLANDS_NAV_SPAWN_YAW="0.484129496"
BAYLANDS_NAV_LIDAR_MODE="3d"
NAV_LIDAR_MODE=""
HAS_GAZEBO_SPAWN_OVERRIDE="false"
GAZEBO_SPAWN_STATE_NAME=""
GAZEBO_WAYPOINT_NAME=""
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
NAV2_EXTRA_ARGS=()
RECORD_CMD=()
OMNET="false"

source "$SCRIPT_DIR/slam_state_common.sh"
source "$SCRIPT_DIR/baylands_waypoint_common.sh"
source "$SCRIPT_DIR/baylands_route_lidar_common.sh"
BAYLANDS_GROUP_WAYPOINT_CSV="$(baylands_group_waypoint_csv)"

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

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  SESSION="halmstad-${WORLD}-1to1"
  shift
fi

for arg in "$@"; do
  case "$arg" in
    session:=*)
      SESSION="${arg#session:=}"
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
    aerial_support_layer_enable:=*)
      NAV2_EXTRA_ARGS+=("$arg")
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
    camera:=*|height:=*|mount_pitch_deg:=*|uav_name:=*)
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
    *)
      echo "Unknown argument: $arg" >&2
      echo "Usage: $0 [world] [mode:=follow|yolo] [record:=true|false] [record_profile:=default|step2_light|vision|support_hazard] [record_tag:=name] [record_out:=bags/experiments/...] [aerial_support_layer_enable:=true|false] [camera:=attached] [follow_yaw:=true|false] [pan_enable:=true|false] [use_tilt:=true|false] [height:=7] [mount_pitch_deg:=45] [uav_name:=dji0] [weights:=...] [target:=...] [yolo_control_mode:=visual_bridge|follow_uav_estimate] [visual_follow_logic:=legacy|follow_core] [obb:=true|false] [tracker:=true|false] [external_detection_node:=detector|tracker] [tracker_config:=botsort.yaml] [detector_backend:=ultralytics|onnx_cpu|onnx_directml] [detector_async_inference:=true|false] [yolo_device:=cpu|auto] [detector_onnx_model:=...] [params_file:=/path/run_follow_defaults.yaml] [ugv_start_delay_s:=12.0] [follow_point_prefer_target_pose_heading:=true|false] [follow_point_prefer_target_pose_position:=true|false] [start_visual_actuation_bridge:=true|false] [start_visual_follow_point_generator:=true|false] [start_visual_follow_planner:=true|false] [start_visual_follow_controller:=true|false] [nav2_goals:=parkinglot_east|route.yaml] [ugv_goal_sequence_csv:=x,y,yaw;...] [ugv_goal_sequence_randomize:=true|false] [ugv_goal_sequence_random_reverse:=true|false] [ugv_goal_sequence_relative_to_current_pose:=true|false] [folder:=...] [map:=/path/map.yaml] [lidar:=2d|3d] [pc2ls_min_height:=...] [pc2ls_max_height:=...] [scan_relay_hz:=...] [gui:=true|false] [rtf:=1.0] [x:=...] [y:=...] [z:=...] [yaw:=...] [state:=checkpoint] [waypoint:=name] [delay_s:=9] [spawn_delay_s:=9] [localization_delay_s:=11] [nav2_delay_s:=11] [follow_delay_s:=13] [follow_wait_topics:=/topic_a,/topic_b] [record_delay_s:=13] [session:=name] [tmux_attach:=true|false] [dry_run:=true|false] [layout:=windows|panes] [omnet:=true|false] [omnet_network:=wifi|5g|lora] [omnet_ui:=cmdenv|qtenv] [omnet_project:=/path/UAV_UGV] [omnet_result_dir:=/path] [omnet_bridge_port:=5555] [omnet_start_delay_s:=3.0] [uav_start_delay_s:=12.0]" >&2
      exit 2
      ;;
  esac
done

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
  default|step2_light|vision|support_hazard)
    ;;
  *)
    echo "Invalid record_profile: $RECORD_PROFILE" >&2
    echo "Use record_profile:=default, step2_light, vision, or support_hazard" >&2
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
  FOLLOW_ARGS+=("nav2_goals:=$(baylands_route_yaml_path "$BAYLANDS_DEFAULT_NAV2_GOALS")")
  NAV2_GOALS_FOR_LIDAR="$BAYLANDS_DEFAULT_NAV2_GOALS"
  echo "[run_tmux_1to1] Baylands default Nav2 goals: nav2_goals:=$BAYLANDS_DEFAULT_NAV2_GOALS"
fi

if [[ "$WORLD" == baylands* ]] && [ "$HAS_GAZEBO_SPAWN_OVERRIDE" = "false" ]; then
  GAZEBO_WAYPOINT_NAME="$BAYLANDS_DEFAULT_WAYPOINT"
  HAS_GAZEBO_SPAWN_OVERRIDE="true"
  GAZEBO_ARGS+=("waypoint:=$GAZEBO_WAYPOINT_NAME")
  echo "[run_tmux_1to1] Baylands default UGV spawn waypoint: waypoint:=$GAZEBO_WAYPOINT_NAME"
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
  if [[ "$WORLD" != baylands* ]]; then
    FOLLOW_ARGS+=(
      "uav_start_x:=$uav_x"
      "uav_start_y:=$uav_y"
      "uav_start_z:=$uav_z"
      "uav_start_yaw_deg:=$uav_yaw_deg"
    )
  fi
  echo "[run_tmux_1to1] Using deterministic UAV spawn from UGV spawn x=${UGV_SPAWN_X} y=${UGV_SPAWN_Y} yaw=${UGV_SPAWN_YAW}: uav_x=${uav_x} uav_y=${uav_y} uav_z=${uav_z} uav_yaw_deg=${uav_yaw_deg}"
  if [[ "$WORLD" == baylands* ]]; then
    echo "[run_tmux_1to1] Baylands follow start will be resolved from the live UAV pose by run_1to1_follow."
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
if [ "${#NAV2_EXTRA_ARGS[@]}" -gt 0 ]; then
  NAV2_CMD+=("${NAV2_EXTRA_ARGS[@]}")
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
  OMNET_READY_CMD="$(build_omnet_ready_cmd)"
fi

GAZEBO_LINE="$(build_line 0 false "" "${GAZEBO_CMD[@]}")"
SPAWN_LINE="$(build_line "$SPAWN_DELAY_S" true "" "${SPAWN_CMD[@]}")"
LOCALIZATION_LINE="$(build_line "$LOCALIZATION_DELAY_S" true "" "${LOCALIZATION_CMD[@]}")"
NAV2_LINE="$(build_line "$NAV2_DELAY_S" true "$LOCALIZATION_READY_CMD" "${NAV2_CMD[@]}")"
FOLLOW_LINE="$(build_line "$FOLLOW_DELAY_S" true "$FOLLOW_READY_CMD" "${FOLLOW_CMD[@]}")"
if [ "$OMNET" = true ]; then
  OMNET_LINE="$(build_line "$FOLLOW_DELAY_S" true "$OMNET_READY_CMD" "${OMNET_CMD[@]}")"
fi
if [ "$RECORD" = true ]; then
  RECORD_LINE="$(build_line "$RECORD_DELAY_S" true "$LOCALIZATION_READY_CMD" "${RECORD_CMD[@]}")"
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  echo "Attach with: tmux attach -t $SESSION" >&2
  exit 1
fi

prelaunch_safety_cleanup

if [ "$DRY_RUN" = true ]; then
  echo "Session: $SESSION"
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
  exit 0
fi

if [ "$LAYOUT" = "windows" ]; then
  tmux new-session -d -s "$SESSION" -n gazebo
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

  tmux send-keys -t "$SESSION:gazebo" "$GAZEBO_LINE" C-m
  tmux send-keys -t "$SESSION:spawn" "$SPAWN_LINE" C-m
  tmux send-keys -t "$SESSION:localization" "$LOCALIZATION_LINE" C-m
  tmux send-keys -t "$SESSION:nav2" "$NAV2_LINE" C-m
  tmux send-keys -t "$SESSION:follow" "$FOLLOW_LINE" C-m
  if [ "$OMNET" = true ]; then
    tmux send-keys -t "$SESSION:omnet" "$OMNET_LINE" C-m
  fi
  if [ "$RECORD" = true ]; then
    tmux send-keys -t "$SESSION:record" "$RECORD_LINE" C-m
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

  tmux send-keys -t "$gazebo_pane" "$GAZEBO_LINE" C-m
  tmux send-keys -t "$spawn_pane" "$SPAWN_LINE" C-m
  tmux send-keys -t "$localization_pane" "$LOCALIZATION_LINE" C-m
  tmux send-keys -t "$nav2_pane" "$NAV2_LINE" C-m
  tmux send-keys -t "$follow_pane" "$FOLLOW_LINE" C-m
  if [ "$OMNET" = true ]; then
    tmux send-keys -t "$omnet_pane" "$OMNET_LINE" C-m
  fi
  if [ "$RECORD" = true ]; then
    tmux send-keys -t "$record_pane" "$RECORD_LINE" C-m
  fi
  tmux select-window -t "$SESSION:sim"
  tmux select-pane -t "$gazebo_pane"
fi

if [ "$TMUX_ATTACH" = true ]; then
  exec tmux attach -t "$SESSION"
fi

echo "Started tmux session: $SESSION"
echo "Attach with: tmux attach -t $SESSION"

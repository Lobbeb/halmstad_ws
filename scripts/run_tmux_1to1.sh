#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
TMUX_STATE_DIR="$STATE_DIR/tmux_sessions"
WORLD="warehouse"
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
UGV_START_DELAY_OVERRIDE=""
UAV_START_DELAY_OVERRIDE=""
SPAWN_ARGS=()
FOLLOW_ARGS=()
GAZEBO_ARGS=()
RECORD_CMD=()
OMNET="false"

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
    localization_delay_s:=*)
      LOCALIZATION_DELAY_OVERRIDE="${arg#localization_delay_s:=}"
      ;;
    nav2_delay_s:=*)
      NAV2_DELAY_OVERRIDE="${arg#nav2_delay_s:=}"
      ;;
    follow_delay_s:=*)
      FOLLOW_DELAY_OVERRIDE="${arg#follow_delay_s:=}"
      ;;
    record_delay_s:=*)
      RECORD_DELAY_OVERRIDE="${arg#record_delay_s:=}"
      ;;
    camera_mode:=*|uav_camera_mode:=*)
      echo "Use camera:=attached with $0." >&2
      exit 2
      ;;
    camera:=*|height:=*|mount_pitch_deg:=*|uav_name:=*)
      if [[ "$arg" == uav_name:=* ]]; then
        UAV_NAME="${arg#uav_name:=}"
      fi
      SPAWN_ARGS+=("$arg")
      FOLLOW_ARGS+=("$arg")
      ;;
    follow_yaw:=*|pan_enable:=*|use_tilt:=*|tilt_enable:=*|camera_default_tilt_deg:=*|use_actual_heading:=*|leader_actual_heading_enable:=*|leader_actual_heading_topic:=*|leader_actual_pose_enable:=*|publish_follow_debug_topics:=*|publish_pose_cmd_topics:=*|publish_camera_debug_topics:=*)
      FOLLOW_ARGS+=("$arg")
      ;;
    weights:=*|target:=*|use_estimate:=*|obb:=*|folder:=*|dir:=*|subdir:=*|tracker:=*|external_detection_node:=*|tracker_config:=*|range_mode:=*|leader_range_mode:=*|start_visual_follow_controller:=*|start_visual_follow_point_generator:=*|start_visual_follow_planner:=*|start_visual_actuation_bridge:=*|leader_selected_target_topic:=*|leader_selected_target_filtered_topic:=*|leader_selected_target_filtered_status_topic:=*|leader_visual_target_estimate_topic:=*|leader_visual_target_estimate_status_topic:=*|leader_follow_point_topic:=*|leader_follow_point_status_topic:=*|leader_planned_target_topic:=*|leader_planned_target_status_topic:=*|leader_visual_control_topic:=*|leader_visual_control_status_topic:=*|leader_visual_actuation_bridge_status_topic:=*)
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
      echo "Usage: $0 [world] [mode:=follow|yolo] [record:=true|false] [record_profile:=default|step2_light|vision] [record_tag:=name] [record_out:=bags/experiments/...] [camera:=attached] [follow_yaw:=true|false] [pan_enable:=true|false] [use_tilt:=true|false] [use_actual_heading:=true|false] [leader_actual_pose_enable:=true|false] [publish_follow_debug_topics:=true|false] [publish_pose_cmd_topics:=true|false] [publish_camera_debug_topics:=true|false] [height:=7] [mount_pitch_deg:=45] [uav_name:=dji0] [weights:=...] [target:=...] [use_estimate:=true|false] [obb:=true|false] [tracker:=true|false] [external_detection_node:=detector|tracker] [tracker_config:=botsort.yaml] [start_visual_actuation_bridge:=true|false] [start_visual_follow_point_generator:=true|false] [start_visual_follow_planner:=true|false] [start_visual_follow_controller:=true|false] [folder:=...] [map:=/path/map.yaml] [gui:=true|false] [rtf:=1.0] [delay_s:=9] [spawn_delay_s:=9] [localization_delay_s:=11] [nav2_delay_s:=11] [follow_delay_s:=13] [record_delay_s:=13] [session:=name] [tmux_attach:=true|false] [dry_run:=true|false] [layout:=windows|panes] [omnet:=true|false] [omnet_network:=wifi|5g|lora] [omnet_ui:=cmdenv|qtenv] [omnet_project:=/path/UAV_UGV] [omnet_result_dir:=/path] [omnet_bridge_port:=5555] [omnet_start_delay_s:=3.0] [ugv_start_delay_s:=3.0] [uav_start_delay_s:=3.0]" >&2
      exit 2
      ;;
  esac
done

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
      weights:=*|target:=*|use_estimate:=*|obb:=*|folder:=*|dir:=*|subdir:=*|tracker:=*|external_detection_node:=*|tracker_config:=*|range_mode:=*|leader_range_mode:=*|start_visual_follow_controller:=*|start_visual_follow_point_generator:=*|start_visual_follow_planner:=*|start_visual_actuation_bridge:=*|leader_selected_target_topic:=*|leader_selected_target_filtered_topic:=*|leader_selected_target_filtered_status_topic:=*|leader_visual_target_estimate_topic:=*|leader_visual_target_estimate_status_topic:=*|leader_follow_point_topic:=*|leader_follow_point_status_topic:=*|leader_planned_target_topic:=*|leader_planned_target_status_topic:=*|leader_visual_control_topic:=*|leader_visual_control_status_topic:=*|leader_visual_actuation_bridge_status_topic:=*)
        echo "Argument '$arg' requires mode:=yolo" >&2
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
  if [ -n "$ready_cmd" ]; then
    printf -v line '%sbash -lc %q && ' "$line" "$ready_cmd"
  fi
  printf -v line '%s%s' "$line" "$(shell_join "$@")"
  printf '%s' "$line"
}

build_localization_ready_cmd() {
  cat <<EOF
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_EFFECTIVE"
export RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION_EFFECTIVE"
set -u
while ! ros2 lifecycle get /$UGV_NAMESPACE/map_server 2>/dev/null | grep -q 'active \[3\]'; do sleep 1; done
# AMCL can be inactive here until nav2 lifecycle manager transitions it.
while ! ros2 lifecycle get /$UGV_NAMESPACE/amcl 2>/dev/null | grep -Eq '(inactive \[2\]|active \[3\])'; do sleep 1; done
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
while true; do
  if ! ros2 lifecycle get /$UGV_NAMESPACE/map_server 2>/dev/null | grep -q 'active \[3\]'; then
    sleep 1
    continue
  fi
  if ! ros2 lifecycle get /$UGV_NAMESPACE/amcl 2>/dev/null | grep -q 'active \[3\]'; then
    sleep 1
    continue
  fi
  if ! ros2 lifecycle get /$UGV_NAMESPACE/controller_server 2>/dev/null | grep -q 'active \[3\]'; then
    sleep 1
    continue
  fi
  if ! ros2 lifecycle get /$UGV_NAMESPACE/bt_navigator 2>/dev/null | grep -q 'active \[3\]'; then
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

signal_processes_by_pattern() {
  local pattern="$1"
  if pkill -INT -f "$pattern" 2>/dev/null; then
    sleep 1
    pkill -TERM -f "$pattern" 2>/dev/null || true
    sleep 1
    pkill -KILL -f "$pattern" 2>/dev/null || true
  fi
}

signal_named_nodes() {
  local names_regex="$1"
  signal_processes_by_pattern "__node:=($names_regex)(\\s|$)"
}

prelaunch_safety_cleanup() {
  rm -f "$SIM_PID_FILE"
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_1to1_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch clearpath_nav2_demos nav2\.launch\.py'
  signal_processes_by_pattern 'ros2 launch clearpath_nav2_demos localization\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad spawn_uav_1to1\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad managed_clearpath_sim\.launch\.py'
  signal_named_nodes 'amcl|map_server|planner_server|controller_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|smoother_server|route_server|lifecycle_manager_localization|lifecycle_manager_navigation|ugv_nav2_driver|ugv_amcl_to_odom|ugv_amcl_to_platform_odom|ugv_amcl_to_platform_filtered_odom|ugv_platform_odom_to_tf|uav_simulator|leader_detector|leader_estimator|selected_target_filter|visual_target_estimator|follow_point_generator|follow_point_planner|visual_actuation_bridge|camera_tracker'
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
elif [ "$OMNET" = true ]; then
  FOLLOW_ARGS+=("uav_start_delay_s:=$shared_start_delay_s")
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
if [ "$MODE" = "yolo" ]; then
  FOLLOW_CMD=(./run.sh 1to1_yolo "$WORLD" "${FOLLOW_ARGS[@]}")
else
FOLLOW_CMD=(./run.sh 1to1_follow "$WORLD" "${FOLLOW_ARGS[@]}")
fi

LOCALIZATION_READY_CMD="$(build_localization_ready_cmd)"
NAV2_READY_CMD="$(build_nav2_ready_cmd)"

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
NAV2_LINE="$(build_line "$NAV2_DELAY_S" true "" "${NAV2_CMD[@]}")"
FOLLOW_LINE="$(build_line "$FOLLOW_DELAY_S" true "" "${FOLLOW_CMD[@]}")"
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
  echo "Record: $RECORD"
  echo "Base delay: $BASE_DELAY_S"
  echo "Overrides: spawn=${SPAWN_DELAY_OVERRIDE:-default} localization=${LOCALIZATION_DELAY_OVERRIDE:-default} nav2=${NAV2_DELAY_OVERRIDE:-default} follow=${FOLLOW_DELAY_OVERRIDE:-default} record=${RECORD_DELAY_OVERRIDE:-default}"
  if [ "$OMNET" = true ] || [ -n "$OMNET_START_DELAY_OVERRIDE" ] || [ -n "$UGV_START_DELAY_OVERRIDE" ] || [ -n "$UAV_START_DELAY_OVERRIDE" ]; then
    echo "Startup holds: shared=${OMNET_START_DELAY_OVERRIDE:-$DEFAULT_OMNET_START_DELAY_S} ugv=${UGV_START_DELAY_OVERRIDE:-${OMNET:+$shared_start_delay_s}} uav=${UAV_START_DELAY_OVERRIDE:-${OMNET:+$shared_start_delay_s}}"
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

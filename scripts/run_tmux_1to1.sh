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
SPAWN_ARGS=()
FOLLOW_ARGS=()
GAZEBO_ARGS=()
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
    *)
      echo "Unknown argument: $arg" >&2
      echo "Usage: $0 [world] [mode:=follow|yolo] [record:=true|false] [record_profile:=default|step2_light|vision] [record_tag:=name] [record_out:=bags/experiments/...] [camera:=attached] [follow_yaw:=true|false] [pan_enable:=true|false] [use_tilt:=true|false] [use_actual_heading:=true|false] [leader_actual_pose_enable:=true|false] [publish_follow_debug_topics:=true|false] [publish_pose_cmd_topics:=true|false] [publish_camera_debug_topics:=true|false] [height:=7] [mount_pitch_deg:=45] [uav_name:=dji0] [weights:=...] [target:=...] [use_estimate:=true|false] [obb:=true|false] [tracker:=true|false] [external_detection_node:=detector|tracker] [tracker_config:=botsort.yaml] [start_visual_actuation_bridge:=true|false] [start_visual_follow_point_generator:=true|false] [start_visual_follow_planner:=true|false] [start_visual_follow_controller:=true|false] [folder:=...] [map:=/path/map.yaml] [gui:=true|false] [rtf:=1.0] [delay_s:=9] [spawn_delay_s:=9] [localization_delay_s:=11] [nav2_delay_s:=11] [follow_delay_s:=13] [record_delay_s:=13] [session:=name] [tmux_attach:=true|false] [dry_run:=true|false] [layout:=windows|panes] [omnet:=true|false]" >&2
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
  default|vision)
    ;;
  *)
    echo "Invalid record_profile: $RECORD_PROFILE" >&2
    echo "Use record_profile:=default or record_profile:=vision" >&2
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
  shift 2
  local line=""
  printf -v line 'cd %q && ' "$WS_ROOT"
  if [ "$wait_for_sim" = true ]; then
    printf -v line '%swhile [ ! -f %q ]; do sleep 1; done && ' "$line" "$SIM_PID_FILE"
  fi
  if [ "$delay_s" != "0" ] && [ "$delay_s" != "0.0" ]; then
    printf -v line '%ssleep %q && ' "$line" "$delay_s"
  fi
  printf -v line '%s%s' "$line" "$(shell_join "$@")"
  printf '%s' "$line"
}

write_session_state() {
  mkdir -p "$TMUX_STATE_DIR"
  {
    printf 'SESSION=%q\n' "$SESSION"
    printf 'LAYOUT=%q\n' "$LAYOUT"
    printf 'EFFECTIVE_GUI=%q\n' "$EFFECTIVE_GUI"
    printf 'GAZEBO_PANE_ID=%q\n' "$gazebo_pane"
    printf 'SPAWN_PANE_ID=%q\n' "$spawn_pane"
    printf 'LOCALIZATION_PANE_ID=%q\n' "$localization_pane"
    printf 'NAV2_PANE_ID=%q\n' "$nav2_pane"
    printf 'FOLLOW_PANE_ID=%q\n' "$follow_pane"
    printf 'RECORD_PANE_ID=%q\n' "$record_pane"
  } > "$SESSION_STATE_FILE"
}

apply_default_delays

FOLLOW_ARGS+=("start_omnet_bridge:=$OMNET")

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

if [ "$RECORD" = true ]; then
  RECORD_CMD=(./run.sh record_experiment "$WORLD" "mode:=$MODE" "uav_name:=$UAV_NAME" "profile:=$RECORD_PROFILE")
  if [ -n "$RECORD_TAG" ]; then
    RECORD_CMD+=("tag:=$RECORD_TAG")
  fi
  if [ -n "$RECORD_OUT" ]; then
    RECORD_CMD+=("out:=$RECORD_OUT")
  fi
fi

GAZEBO_LINE="$(build_line 0 false "${GAZEBO_CMD[@]}")"
SPAWN_LINE="$(build_line "$SPAWN_DELAY_S" true "${SPAWN_CMD[@]}")"
LOCALIZATION_LINE="$(build_line "$LOCALIZATION_DELAY_S" true "${LOCALIZATION_CMD[@]}")"
NAV2_LINE="$(build_line "$NAV2_DELAY_S" true "${NAV2_CMD[@]}")"
FOLLOW_LINE="$(build_line "$FOLLOW_DELAY_S" true "${FOLLOW_CMD[@]}")"
if [ "$RECORD" = true ]; then
  RECORD_LINE="$(build_line "$RECORD_DELAY_S" true "${RECORD_CMD[@]}")"
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session already exists: $SESSION" >&2
  echo "Attach with: tmux attach -t $SESSION" >&2
  exit 1
fi

if [ "$DRY_RUN" = true ]; then
  echo "Session: $SESSION"
  echo "Mode: $MODE"
  echo "Layout: $LAYOUT"
  echo "GUI: $EFFECTIVE_GUI"
  echo "Record: $RECORD"
  echo "Base delay: $BASE_DELAY_S"
  echo "Overrides: spawn=${SPAWN_DELAY_OVERRIDE:-default} localization=${LOCALIZATION_DELAY_OVERRIDE:-default} nav2=${NAV2_DELAY_OVERRIDE:-default} follow=${FOLLOW_DELAY_OVERRIDE:-default} record=${RECORD_DELAY_OVERRIDE:-default}"
  echo "Delays: spawn=$SPAWN_DELAY_S localization=$LOCALIZATION_DELAY_S nav2=$NAV2_DELAY_S follow=$FOLLOW_DELAY_S record=$RECORD_DELAY_S"
  echo "[gazebo]       $GAZEBO_LINE"
  echo "[spawn]        $SPAWN_LINE"
  echo "[localization] $LOCALIZATION_LINE"
  echo "[nav2]         $NAV2_LINE"
  echo "[follow]       $FOLLOW_LINE"
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
  if [ "$RECORD" = true ]; then
    record_pane="$(tmux display-message -p -t "$SESSION:record" '#{pane_id}')"
  fi

  write_session_state

  tmux send-keys -t "$SESSION:gazebo" "$GAZEBO_LINE" C-m
  tmux send-keys -t "$SESSION:spawn" "$SPAWN_LINE" C-m
  tmux send-keys -t "$SESSION:localization" "$LOCALIZATION_LINE" C-m
  tmux send-keys -t "$SESSION:nav2" "$NAV2_LINE" C-m
  tmux send-keys -t "$SESSION:follow" "$FOLLOW_LINE" C-m
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

  tmux select-pane -t "$gazebo_pane" -T gazebo
  tmux select-pane -t "$spawn_pane" -T spawn
  tmux select-pane -t "$localization_pane" -T localization
  tmux select-pane -t "$nav2_pane" -T nav2
  tmux select-pane -t "$follow_pane" -T follow
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

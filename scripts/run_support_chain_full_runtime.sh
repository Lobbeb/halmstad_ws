#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SCENARIO="baseline"
OUTPUT=""
BASELINE_EVIDENCE=""
SESSION=""
GUI="true"
TMUX_ATTACH="true"
DRY_RUN="false"
REDUCED_RESOURCE="false"
TIMEOUT_S="300.0"

START_WAYPOINT="parkinglot_west_1"
START_X="-71.39979517989994"
START_Y="205.6352521524901"
START_YAW="-1.6133110777"
GOAL_X="-72.53159610937462"
GOAL_Y="185.3861158209806"
GOAL_YAW_RAD="0.0066198909"
GOAL_YAW_DEG="0.37926327105935376"
HAZARD_X="-72.0"
HAZARD_Y="195.5"
HAZARD_SIZE_X="2.0"
HAZARD_SIZE_Y="2.0"
MAP="$WS_ROOT/maps/baylands.yaml"
NAV2_CONFIG="$WS_ROOT/src/lrs_halmstad/config/nav2_baylands_large_map.yaml"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh support_chain_full_runtime scenario:=baseline|valid|clearing [option:=value ...]

Options:
  output:=DIR             Evidence root. Default: evidence/support_runtime/<scenario>.
  baseline_evidence:=DIR  Baseline analysis directory used by valid/clearing.
  session:=NAME           Exact task-owned tmux session name.
  gui:=true|false         Gazebo GUI selection. Default: true.
  tmux_attach:=true|false Attach after startup. Default: true.
  timeout_s:=S            Evidence wall-time limit. Default: 300.
  dry_run:=true           Print all resolved commands without starting ROS or tmux.
  reduced_resource:=true  Non-authoritative UGV/Nav2/typed-chain diagnostic without UAV rendering.

This fixed Baylands profile starts at parkinglot_west_1 and sends one existing
NavigateToPose driver goal to parkinglot_west_2. valid and clearing use a
synthetic confirmed dji1 hazard at (-72.0, 195.5). The source activates only
after an active Nav2 goal is observed. Three UAVs are composed, while typed
dji2 fusion remains disabled. Stop only this run with the printed session command.
EOF
}

validate_boolean() {
  local label="$1"
  local value="$2"
  case "$value" in
    true|false) ;;
    *) echo "$label must be true or false." >&2; exit 2 ;;
  esac
}

shell_join() {
  local out=""
  local part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help) usage; exit 0 ;;
    scenario:=*) SCENARIO="${arg#scenario:=}" ;;
    output:=*) OUTPUT="${arg#output:=}" ;;
    baseline_evidence:=*) BASELINE_EVIDENCE="${arg#baseline_evidence:=}" ;;
    session:=*) SESSION="${arg#session:=}" ;;
    gui:=*) GUI="${arg#gui:=}" ;;
    tmux_attach:=*|attach:=*) TMUX_ATTACH="${arg#*:=}" ;;
    timeout_s:=*) TIMEOUT_S="${arg#timeout_s:=}" ;;
    dry_run:=*) DRY_RUN="${arg#dry_run:=}" ;;
    reduced_resource:=*) REDUCED_RESOURCE="${arg#reduced_resource:=}" ;;
    *) echo "Unknown argument: $arg" >&2; usage >&2; exit 2 ;;
  esac
done

case "$SCENARIO" in
  baseline|valid|clearing) ;;
  *) echo "scenario must be baseline, valid, or clearing." >&2; exit 2 ;;
esac
validate_boolean gui "$GUI"
validate_boolean tmux_attach "$TMUX_ATTACH"
validate_boolean dry_run "$DRY_RUN"
validate_boolean reduced_resource "$REDUCED_RESOURCE"

if [ -z "$OUTPUT" ]; then
  OUTPUT="$WS_ROOT/evidence/support_runtime/$SCENARIO"
elif [[ "$OUTPUT" != /* ]]; then
  OUTPUT="$WS_ROOT/$OUTPUT"
fi
if [ -z "$BASELINE_EVIDENCE" ] && [ "$REDUCED_RESOURCE" = true ]; then
  BASELINE_EVIDENCE="$(dirname "$OUTPUT")/baseline/analysis"
elif [ -z "$BASELINE_EVIDENCE" ]; then
  BASELINE_EVIDENCE="$WS_ROOT/evidence/support_runtime/baseline/analysis"
elif [[ "$BASELINE_EVIDENCE" != /* ]]; then
  BASELINE_EVIDENCE="$WS_ROOT/$BASELINE_EVIDENCE"
fi
if [ -z "$SESSION" ]; then
  if [ "$REDUCED_RESOURCE" = true ]; then
    SESSION="halmstad-baylands-track-a-reduced-$SCENARIO"
  else
    SESSION="halmstad-baylands-track-a-$SCENARIO"
  fi
fi

ANALYSIS_DIR="$OUTPUT/analysis"
RECORD_DIR="$OUTPUT/recording"
LOG_DIR="$OUTPUT/logs"
HAZARD_ENABLE="false"
LAYER_ENABLE="false"
ACTIVE_DURATION_S="0.0"
if [ "$SCENARIO" = "valid" ]; then
  HAZARD_ENABLE="true"
  LAYER_ENABLE="true"
elif [ "$SCENARIO" = "clearing" ]; then
  HAZARD_ENABLE="true"
  LAYER_ENABLE="true"
  ACTIVE_DURATION_S="4.0"
fi

SUPPORT_CMD=(
  "$WS_ROOT/run.sh" tmux_support_chain baylands
  "session:=$SESSION"
  mode:=follow
  "gui:=$GUI"
  tmux_attach:=false
  layout:=panes
  waypoint:="$START_WAYPOINT"
  ugv_goal_sequence_csv:="$GOAL_X,$GOAL_Y,$GOAL_YAW_DEG"
  ugv_goal_sequence_randomize:=false
  ugv_goal_sequence_random_reverse:=false
  ugv_goal_sequence_relative_to_current_pose:=false
  follow_delay_s:=30
  record_delay_s:=0
  record:=true
  record_profile:=support_hazard
  record_tag:="track_a_full_runtime_$SCENARIO"
  record_out:="$RECORD_DIR"
  dji2_enable:=true
  hazard_chain_enable:="$HAZARD_ENABLE"
  aerial_support_layer_enable:="$LAYER_ENABLE"
  hazard_synthetic_enable:="$HAZARD_ENABLE"
  hazard_synthetic_x:="$HAZARD_X"
  hazard_synthetic_y:="$HAZARD_Y"
  hazard_synthetic_z:=0.5
  hazard_synthetic_size_x:="$HAZARD_SIZE_X"
  hazard_synthetic_size_y:="$HAZARD_SIZE_Y"
  hazard_synthetic_size_z:=1.0
  hazard_synthetic_confidence:=0.9
  hazard_synthetic_state:=1
  hazard_synthetic_publish_rate_hz:=2.0
  hazard_synthetic_ttl_s:=4.0
  hazard_synthetic_start_delay_s:=1.0
  hazard_synthetic_active_duration_s:="$ACTIVE_DURATION_S"
  hazard_synthetic_publish_empty_after_active_duration:=true
  hazard_synthetic_activation_status_topic:=/a201_0000/navigate_to_pose/_action/status
  hazard_synthetic_provenance:="synthetic_track_a_full_runtime:$SCENARIO"
)
if [ "$REDUCED_RESOURCE" = true ]; then
  SUPPORT_CMD+=(reduced_track_a:=true)
else
  # Track A does not consume legacy Gazebo ground truth, and Jazzy lacks its Python binding.
  SUPPORT_CMD+=(start_ugv_ground_truth_bridge:=false)
fi

EVIDENCE_ROS_CMD=(
  ros2 run lrs_halmstad support_hazard_evidence runtime-live
  --scenario "$SCENARIO"
  --namespace a201_0000
  --map "$MAP"
  --nav2-config "$NAV2_CONFIG"
  --start-x "$START_X" --start-y "$START_Y" --start-yaw "$START_YAW"
  --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --goal-yaw "$GOAL_YAW_RAD"
  --hazard-x "$HAZARD_X" --hazard-y "$HAZARD_Y"
  --hazard-size-x "$HAZARD_SIZE_X" --hazard-size-y "$HAZARD_SIZE_Y"
  --variance-x 0.25 --variance-y 0.25 --covariance-sigma-scale 2.0
  --timeout-s "$TIMEOUT_S"
  --output "$ANALYSIS_DIR"
  --runtime-profile "$(
    if [ "$REDUCED_RESOURCE" = true ]; then
      printf '%s' reduced_resource_diagnostic
    else
      printf '%s' authoritative_full
    fi
  )"
)
if [ "$SCENARIO" != "baseline" ]; then
  EVIDENCE_ROS_CMD+=(--baseline-evidence "$BASELINE_EVIDENCE")
fi
EVIDENCE_LINE="cd $(printf '%q' "$WS_ROOT") && unset VIRTUAL_ENV PYTHONHOME PYTHONPATH && export PATH=/usr/bin:/bin:/usr/sbin:/sbin:\$PATH && source /opt/ros/jazzy/setup.bash && source $(printf '%q' "$WS_ROOT/install/setup.bash") && exec $(shell_join "${EVIDENCE_ROS_CMD[@]}") --ros-args -p use_sim_time:=true"

echo "[support_chain_full_runtime] scenario=$SCENARIO"
echo "[support_chain_full_runtime] output=$OUTPUT"
echo "[support_chain_full_runtime] session=$SESSION"
echo "[support_chain_full_runtime] fixed global costmap config=$NAV2_CONFIG"
if [ "$REDUCED_RESOURCE" = true ]; then
  echo "[support_chain_full_runtime] NON-AUTHORITATIVE REDUCED-RESOURCE TRACK A DIAGNOSTIC"
fi
if [ "$DRY_RUN" = "true" ]; then
  "${SUPPORT_CMD[@]}" dry_run:=true
  echo "[runtime_evidence] $EVIDENCE_LINE"
  echo "[stop] ./stop.sh tmux_support_chain baylands session:=$SESSION"
  exit 0
fi

if [ -e "$OUTPUT" ]; then
  echo "Refusing to overwrite full-runtime evidence root: $OUTPUT" >&2
  exit 1
fi
if [ "$SCENARIO" != "baseline" ]; then
  for required in summary.json plans.json trajectory.csv; do
    if [ ! -f "$BASELINE_EVIDENCE/$required" ]; then
      echo "Baseline evidence is missing $required: $BASELINE_EVIDENCE" >&2
      exit 1
    fi
  done
fi
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Task session already exists: $SESSION" >&2
  exit 1
fi

mkdir -p "$LOG_DIR"
started=false
cleanup_failed_start() {
  if [ "$started" = true ] && tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
  fi
}
trap cleanup_failed_start ERR

"${SUPPORT_CMD[@]}"
started=true
tmux new-window -d -t "$SESSION" -n track_a_evidence
tmux set-option -w -t "$SESSION:track_a_evidence" remain-on-exit on
tmux send-keys -t "$SESSION:track_a_evidence.0" "$EVIDENCE_LINE" C-m

while IFS='|' read -r pane_id window_name pane_index; do
  safe_window="${window_name//[^A-Za-z0-9_.-]/_}"
  log_path="$LOG_DIR/${safe_window}_${pane_index}.log"
  tmux capture-pane -p -S - -t "$pane_id" > "$log_path"
  tmux pipe-pane -o -t "$pane_id" "cat >> $(printf '%q' "$log_path")"
done < <(tmux list-panes -s -t "$SESSION" -F '#{pane_id}|#{window_name}|#{pane_index}')

trap - ERR
echo "[support_chain_full_runtime] live logs: $LOG_DIR"
echo "[support_chain_full_runtime] analysis: $ANALYSIS_DIR"
echo "[support_chain_full_runtime] rosbag: $RECORD_DIR"
echo "[support_chain_full_runtime] stop only this run with:"
echo "  ./stop.sh tmux_support_chain baylands session:=$SESSION"

if [ "$TMUX_ATTACH" = "true" ]; then
  exec tmux attach -t "$SESSION"
fi
echo "Attach with: tmux attach -t $SESSION"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SUPPORT_STATE_DIR="$STATE_DIR/support_sessions"
WORLD="baylands"
SESSION=""
GROUP_GRACE_S=4
FINAL_GRACE_S=4
SUPPORT_GRACE_S=2
KILL_SESSION=true
DRY_RUN=false
BROAD_CLEANUP=false

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

SESSION="halmstad-${WORLD}-support-chain"

for arg in "$@"; do
  case "$arg" in
    session:=*)
      SESSION="${arg#session:=}"
      ;;
    group_grace_s:=*)
      GROUP_GRACE_S="${arg#group_grace_s:=}"
      ;;
    final_grace_s:=*)
      FINAL_GRACE_S="${arg#final_grace_s:=}"
      ;;
    support_grace_s:=*)
      SUPPORT_GRACE_S="${arg#support_grace_s:=}"
      ;;
    kill_session:=*)
      KILL_SESSION="${arg#kill_session:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    broad_cleanup:=*)
      BROAD_CLEANUP="${arg#broad_cleanup:=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      echo "Usage: $0 [world] [session:=name] [group_grace_s:=4] [final_grace_s:=4] [support_grace_s:=2] [kill_session:=true|false] [dry_run:=true|false] [broad_cleanup:=true|false]" >&2
      exit 2
      ;;
  esac
done

session_safe="${SESSION//[^A-Za-z0-9_.-]/_}"
SUPPORT_OBSERVATION_PID_FILE="$SUPPORT_STATE_DIR/${session_safe}.observation.pid"

tmux_has_session() {
  tmux has-session -t "$SESSION" 2>/dev/null
}

find_pane_by_title() {
  local name="$1"
  tmux list-panes -a -t "$SESSION" -F '#{pane_id}|#{pane_title}' 2>/dev/null | awk -F '|' -v want="$name" '$2 == want { print $1; exit }'
}

window_exists() {
  local name="$1"
  tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -Fxq "$name"
}

send_ctrl_c_to_target() {
  local name="$1"
  local pane_id=""

  pane_id="$(find_pane_by_title "$name" || true)"
  if [ -n "$pane_id" ]; then
    echo "Sending Ctrl-C to support target: $name ($pane_id)"
    if [ "$DRY_RUN" != true ]; then
      tmux send-keys -t "$pane_id" C-c
    fi
    return 0
  fi

  if window_exists "$name"; then
    echo "Sending Ctrl-C to support window: $name"
    if [ "$DRY_RUN" != true ]; then
      tmux send-keys -t "$SESSION:$name" C-c
    fi
    return 0
  fi

  echo "Skipping missing support target: $name"
  return 1
}

signal_processes_by_pattern() {
  local label="$1"
  local pattern="$2"
  local matched=""

  matched="$(pgrep -a -f "$pattern" 2>/dev/null || true)"
  [ -n "$matched" ] || return 1

  echo "Safety cleanup: matched $label"
  printf '%s\n' "$matched"

  if [ "$DRY_RUN" != true ]; then
    pkill -INT -f "$pattern" 2>/dev/null || true
    sleep 1
    pkill -TERM -f "$pattern" 2>/dev/null || true
    sleep 1
    pkill -KILL -f "$pattern" 2>/dev/null || true
  fi
  return 0
}

signal_named_nodes() {
  local label="$1"
  local names_regex="$2"
  signal_processes_by_pattern "$label" "__node:=($names_regex)(\\s|$)"
}

signal_process_group_from_pid_file() {
  local pid_file="$1"
  local label="$2"
  local pid=""
  local pgid=""

  [ -f "$pid_file" ] || return 1
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  [ -n "$pid" ] || return 1
  kill -0 "$pid" 2>/dev/null || {
    rm -f "$pid_file"
    return 1
  }
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
  [ -n "$pgid" ] || return 1

  echo "Safety cleanup: signaling task-owned $label process group $pgid"
  if [ "$DRY_RUN" != true ]; then
    /bin/kill -INT -- "-$pgid" 2>/dev/null || true
    sleep 2
    /bin/kill -TERM -- "-$pgid" 2>/dev/null || true
    sleep 2
    /bin/kill -KILL -- "-$pgid" 2>/dev/null || true
    rm -f "$pid_file"
  fi
  return 0
}

run_support_fallback_cleanup() {
  signal_process_group_from_pid_file \
    "$SUPPORT_OBSERVATION_PID_FILE" "support observation" || true
  if [ "$BROAD_CLEANUP" != true ]; then
    return 0
  fi

  echo "Explicit broad support cleanup enabled for session: $SESSION"
  signal_processes_by_pattern "support follow helper" 'scripts/run_support_follow_odom\.sh' || true
  signal_processes_by_pattern "support observation helper" 'scripts/run_support_observation\.sh' || true
  signal_processes_by_pattern "support follow launch" 'ros2 launch lrs_halmstad support_follow_odom\.launch\.py' || true
  signal_processes_by_pattern "support observation launch" 'ros2 launch lrs_halmstad support_observation\.launch\.py' || true
  signal_processes_by_pattern "support camera scanner" 'support_camera_scanner' || true
  signal_processes_by_pattern "support dataset capture" 'sim_dataset_capture' || true
  signal_named_nodes \
    "support chain child nodes" \
    'support_follow_dji0_pose_to_odom|support_follow_dji1_simulator|support_follow_dji1_odom_controller|support_follow_dji2_simulator|support_follow_dji2_odom_controller|support_dji1_leader_detector|support_dji2_leader_detector|support_detection_mux|dji0_to_ugv_forwarder|support_camera_scanner' || true
}

if tmux_has_session; then
  echo "Stopping support-chain tmux session: $SESSION"
  send_ctrl_c_to_target support_observation || true
  send_ctrl_c_to_target support_follow || true

  if [ "$SUPPORT_GRACE_S" != "0" ] && [ "$SUPPORT_GRACE_S" != "0.0" ]; then
    echo "Waiting ${SUPPORT_GRACE_S}s before stopping the base 1-to-1 stack"
    sleep "$SUPPORT_GRACE_S"
  fi
else
  echo "tmux support-chain session not found: $SESSION"
fi

"$SCRIPT_DIR/stop_tmux_1to1.sh" \
  "$WORLD" \
  "session:=$SESSION" \
  "group_grace_s:=$GROUP_GRACE_S" \
  "final_grace_s:=$FINAL_GRACE_S" \
  "kill_session:=$KILL_SESSION" \
  "dry_run:=$DRY_RUN" \
  "broad_cleanup:=$BROAD_CLEANUP"

run_support_fallback_cleanup

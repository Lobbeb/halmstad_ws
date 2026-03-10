#!/usr/bin/env bash
set -euo pipefail

STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
TMUX_STATE_DIR="$STATE_DIR/tmux_sessions"
WORLD="warehouse"
SESSION=""
GROUP_GRACE_S=5
FINAL_GRACE_S=5
KILL_SESSION=true
DRY_RUN=false
CTRL_C_GROUP=(record follow localization nav2)

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

SESSION="halmstad-${WORLD}-1to1"

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
    kill_session:=*)
      KILL_SESSION="${arg#kill_session:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      echo "Usage: $0 [world] [session:=name] [group_grace_s:=5] [final_grace_s:=5] [kill_session:=true|false] [dry_run:=true|false]" >&2
      exit 2
      ;;
  esac
done

SESSION_SAFE="$(printf '%s' "$SESSION" | tr -c 'A-Za-z0-9_.-' '_')"
SESSION_STATE_FILE="$TMUX_STATE_DIR/${SESSION_SAFE}.env"

GAZEBO_PANE_ID=""
SPAWN_PANE_ID=""
LOCALIZATION_PANE_ID=""
NAV2_PANE_ID=""
FOLLOW_PANE_ID=""
RECORD_PANE_ID=""

if [ -f "$SESSION_STATE_FILE" ]; then
  # State file is written by run_tmux_1to1.sh for robust shutdown targeting.
  # shellcheck disable=SC1090
  source "$SESSION_STATE_FILE"
fi

tmux_has_session() {
  tmux has-session -t "$SESSION" 2>/dev/null
}

window_exists() {
  local name="$1"
  tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -Fxq "$name"
}

find_pane_by_title() {
  local name="$1"
  tmux list-panes -a -t "$SESSION" -F '#{pane_id}\t#{pane_title}' 2>/dev/null | awk -F '\t' -v want="$name" '$2 == want { print $1; exit }'
}

pane_exists() {
  local pane_id="$1"
  [ -n "$pane_id" ] || return 1
  tmux list-panes -a -t "$SESSION" -F '#{pane_id}' 2>/dev/null | grep -Fxq "$pane_id"
}

lookup_saved_pane_id() {
  local name="$1"
  case "$name" in
    gazebo)
      printf '%s\n' "$GAZEBO_PANE_ID"
      ;;
    spawn)
      printf '%s\n' "$SPAWN_PANE_ID"
      ;;
    localization)
      printf '%s\n' "$LOCALIZATION_PANE_ID"
      ;;
    nav2)
      printf '%s\n' "$NAV2_PANE_ID"
      ;;
    follow)
      printf '%s\n' "$FOLLOW_PANE_ID"
      ;;
    record)
      printf '%s\n' "$RECORD_PANE_ID"
      ;;
    *)
      return 1
      ;;
  esac
}

cleanup_state_files() {
  rm -f "$SESSION_STATE_FILE" "$SIM_PID_FILE" "$SIM_WORLD_FILE"
}

signal_process_group_from_pid_file() {
  local pid_file="$1"
  local label="$2"
  local pid=""
  local pgid=""

  [ -f "$pid_file" ] || return 1
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  [ -n "$pid" ] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
  [ -n "$pgid" ] || return 1

  echo "Safety cleanup: signaling $label process group $pgid"
  if [ "$DRY_RUN" != true ]; then
    /bin/kill -INT -- "-$pgid" 2>/dev/null || true
    sleep 2
    /bin/kill -TERM -- "-$pgid" 2>/dev/null || true
    sleep 2
    /bin/kill -KILL -- "-$pgid" 2>/dev/null || true
  fi
  return 0
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

run_fallback_cleanup() {
  signal_process_group_from_pid_file "$SIM_PID_FILE" "Gazebo helper" || true
  signal_processes_by_pattern "experiment recorder" 'ros2 bag record .*runs/experiments/.*/bag' || true
  signal_processes_by_pattern "follow launch" 'ros2 launch lrs_halmstad run_1to1_follow\.launch\.py' || true
  signal_processes_by_pattern "Nav2 launch" 'ros2 launch clearpath_nav2_demos nav2\.launch\.py' || true
  signal_processes_by_pattern "localization launch" 'ros2 launch clearpath_nav2_demos localization\.launch\.py' || true
  signal_processes_by_pattern "spawn launch" 'ros2 launch lrs_halmstad spawn_uav_1to1\.launch\.py' || true
  signal_processes_by_pattern "Gazebo launch" 'ros2 launch lrs_halmstad managed_clearpath_sim\.launch\.py' || true
  signal_processes_by_pattern "Gazebo sim" '(^|/)gz sim($| )' || true
}

send_ctrl_c() {
  local name="$1"
  local pane_id=""

  pane_id="$(lookup_saved_pane_id "$name" || true)"
  if pane_exists "$pane_id"; then
    echo "Sending Ctrl-C to target: $name ($pane_id)"
    if [ "$DRY_RUN" != true ]; then
      tmux send-keys -t "$pane_id" C-c
    fi
    return 0
  fi

  if window_exists "$name"; then
    echo "Sending Ctrl-C to window: $name"
    if [ "$DRY_RUN" != true ]; then
      tmux send-keys -t "$SESSION:$name" C-c
    fi
    return 0
  fi

  pane_id="$(find_pane_by_title "$name" || true)"
  if [ -n "$pane_id" ]; then
    echo "Sending Ctrl-C to pane: $name ($pane_id)"
    if [ "$DRY_RUN" != true ]; then
      tmux send-keys -t "$pane_id" C-c
    fi
    return 0
  fi

  return 1
}

if ! tmux_has_session; then
  cleanup_state_files
  if [ "$DRY_RUN" = true ]; then
    echo "tmux session not found: $SESSION"
    echo "Dry run only. Planned Ctrl-C group: ${CTRL_C_GROUP[*]}"
    echo "Dry run only. Planned Gazebo stop after ${GROUP_GRACE_S}s"
    echo "Planned final action: kill_session=$KILL_SESSION after ${FINAL_GRACE_S}s"
    exit 0
  fi
  echo "tmux session not found: $SESSION" >&2
  exit 1
fi

echo "Stopping tmux 1-to-1 session: $SESSION"

for target in "${CTRL_C_GROUP[@]}"; do
  if send_ctrl_c "$target"; then
    :
  else
    echo "Skipping missing target: $target"
  fi
done

if [ "$GROUP_GRACE_S" != "0" ] && [ "$GROUP_GRACE_S" != "0.0" ]; then
  echo "Waiting ${GROUP_GRACE_S}s before stopping Gazebo"
  sleep "$GROUP_GRACE_S"
fi

if send_ctrl_c gazebo; then
  :
else
  echo "Skipping missing target: gazebo"
fi

if [ "$FINAL_GRACE_S" != "0" ] && [ "$FINAL_GRACE_S" != "0.0" ]; then
  echo "Waiting ${FINAL_GRACE_S}s before final session action"
  sleep "$FINAL_GRACE_S"
fi

if [ "$KILL_SESSION" = true ]; then
  if [ "$DRY_RUN" = true ]; then
    echo "Dry run only. Would kill tmux session: $SESSION"
  else
    echo "Killing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
  fi
  run_fallback_cleanup
  cleanup_state_files
else
  echo "Leaving tmux session open after Ctrl-C sweep"
fi

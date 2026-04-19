#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DRY_RUN=false
ONCE=false
TOPICS=()
ECHO_PIDS=()

DEFAULT_TOPICS=(
  /omnet/sim_time
  /omnet/link_distance
  /omnet/rssi_dbm
  /omnet/snir_db
  /omnet/packet_error_rate
  /omnet/radio_distance
)

usage() {
  cat <<'EOF'
Usage: ./run.sh omnet_monitor [topic1 topic2 ...] [once:=true|false] [dry_run:=true|false]

Examples:
  ./run.sh omnet_monitor
  ./run.sh omnet_monitor radio_distance
  ./run.sh omnet_monitor /omnet/rssi_dbm /omnet/snir_db
  ./run.sh omnet_monitor radio_distance once:=true
EOF
}

coerce_bool() {
  case "$1" in
    true|false)
      printf '%s\n' "$1"
      ;;
    *)
      echo "Invalid boolean value: $1" >&2
      exit 2
      ;;
  esac
}

normalize_topic() {
  local value="$1"
  case "$value" in
    all)
      printf '%s\n' "__ALL__"
      ;;
    /omnet/*)
      printf '%s\n' "$value"
      ;;
    sim_time|link_distance|rssi_dbm|snir_db|packet_error_rate|radio_distance)
      printf '/omnet/%s\n' "$value"
      ;;
    *)
      echo "Unknown OMNeT topic token: $value" >&2
      echo "Use one of: sim_time, link_distance, rssi_dbm, snir_db, packet_error_rate, radio_distance" >&2
      echo "or pass a full topic path such as /omnet/radio_distance" >&2
      exit 2
      ;;
  esac
}

append_unique() {
  local value="$1"
  local existing=""
  for existing in "${TOPICS[@]:-}"; do
    if [ "$existing" = "$value" ]; then
      return 0
    fi
  done
  TOPICS+=("$value")
}

cleanup() {
  local pid=""
  for pid in "${ECHO_PIDS[@]:-}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  wait "${ECHO_PIDS[@]:-}" 2>/dev/null || true
}

start_topic_echo() {
  local topic="$1"
  local prefix="[$topic] "

  (
    ros2 topic echo --no-daemon "$topic" 2>&1 | stdbuf -oL awk -v prefix="$prefix" '{ print prefix $0; fflush(); }'
  ) &
  ECHO_PIDS+=("$!")
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    once:=*)
      ONCE="$(coerce_bool "${arg#once:=}")"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${arg#dry_run:=}")"
      ;;
    *)
      normalized="$(normalize_topic "$arg")"
      if [ "$normalized" = "__ALL__" ]; then
        for default_topic in "${DEFAULT_TOPICS[@]}"; do
          append_unique "$default_topic"
        done
      else
        append_unique "$normalized"
      fi
      ;;
  esac
done

if [ "${#TOPICS[@]}" -eq 0 ]; then
  TOPICS=("${DEFAULT_TOPICS[@]}")
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

if [ "$DRY_RUN" = true ]; then
  echo "Topics:"
  printf '  %s\n' "${TOPICS[@]}"
  echo "Mode: $( [ "$ONCE" = true ] && printf 'once' || printf 'stream' )"
  exit 0
fi

if [ "$ONCE" = true ]; then
  topic=""
  for topic in "${TOPICS[@]}"; do
    echo "=== $topic ==="
    ros2 topic echo --no-daemon --once "$topic"
    echo
  done
  exit 0
fi

trap cleanup EXIT INT TERM

topic=""
for topic in "${TOPICS[@]}"; do
  start_topic_echo "$topic"
done

wait "${ECHO_PIDS[@]}"

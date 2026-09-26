#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DRY_RUN=false
ONCE=false
RATE_HZ="2.0"
TOP_PADDING_LINES=3
TOPICS=()
ECHO_PIDS=()
TOPIC_STATE_DIR=""

DEFAULT_TOPICS=(
  /omnet/sim_time
  /omnet/rssi_dbm
  /omnet/snir_db
  /omnet/packet_error_rate
  /omnet/packet_delivery_ratio
  /omnet/latency_s
  /omnet/jitter_s
  /omnet/radio_distance
)

usage() {
  cat <<'EOF'
Usage: ./run.sh omnet_monitor [topic1 topic2 ...] [rate:=Hz] [top:=lines] [once:=true|false] [dry_run:=true|false]

Examples:
  ./run.sh omnet_monitor
  ./run.sh omnet_monitor radio_distance
  ./run.sh omnet_monitor /omnet/rssi_dbm /omnet/snir_db
  ./run.sh omnet_monitor all rate:=4
  ./run.sh omnet_monitor all top:=5
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

validate_rate() {
  local value="$1"
  awk -v value="$value" 'BEGIN { exit !(value + 0 > 0) }' || {
    echo "Invalid rate value: $value" >&2
    echo "Use a positive refresh rate in Hz, for example rate:=2 or rate:=0.5" >&2
    exit 2
  }
  printf '%s\n' "$value"
}

validate_nonnegative_int() {
  local value="$1"
  case "$value" in
    ''|*[!0-9]*)
      echo "Invalid top padding value: $value" >&2
      echo "Use a non-negative integer, for example top:=3" >&2
      exit 2
      ;;
  esac
  printf '%s\n' "$value"
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
    sim_time|rssi_dbm|snir_db|packet_error_rate|packet_delivery_ratio|latency_s|jitter_s|radio_distance)
      printf '/omnet/%s\n' "$value"
      ;;
    *)
      echo "Unknown OMNeT topic token: $value" >&2
      echo "Use one of: sim_time, rssi_dbm, snir_db, packet_error_rate, packet_delivery_ratio, latency_s, jitter_s, radio_distance" >&2
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
  if [ -n "${TOPIC_STATE_DIR:-}" ] && [ -d "$TOPIC_STATE_DIR" ]; then
    rm -rf "$TOPIC_STATE_DIR"
  fi
}

topic_state_file() {
  local topic="$1"
  local safe="${topic#/}"
  safe="${safe//\//__}"
  printf '%s/%s.value\n' "$TOPIC_STATE_DIR" "$safe"
}

format_topic_label() {
  local topic="$1"
  printf '%s\n' "${topic#/omnet/}"
}

format_value() {
  local raw="$1"
  case "$raw" in
    ""|waiting)
      printf '%s\n' "--"
      ;;
    nan|NaN|NAN)
      printf '%s\n' "NaN"
      ;;
    *)
      awk -v value="$raw" 'BEGIN {
        v = value + 0
        if (value ~ /^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$/)
          printf "%.6g\n", v
        else
          print value
      }'
      ;;
  esac
}

topic_note() {
  local topic="$1"
  local value="$2"
  case "$value" in
    ""|waiting)
      printf '%s\n' "waiting"
      return
      ;;
    nan|NaN|NAN)
      case "$topic" in
        /omnet/rssi_dbm|/omnet/snir_db|/omnet/radio_distance)
          printf '%s\n' "waiting for LoRa packet"
          ;;
        *)
          printf '%s\n' "no sample"
          ;;
      esac
      return
      ;;
  esac

  case "$topic" in
    /omnet/packet_error_rate)
      awk -v value="$value" 'BEGIN { printf "PER %.1f%%\n", (value + 0) * 100.0 }'
      ;;
    /omnet/packet_delivery_ratio)
      awk -v value="$value" 'BEGIN { printf "PDR %.1f%%\n", (value + 0) * 100.0 }'
      ;;
    /omnet/rssi_dbm)
      printf '%s\n' "dBm"
      ;;
    /omnet/snir_db)
      printf '%s\n' "dB"
      ;;
    /omnet/radio_distance)
      printf '%s\n' "RSSI m"
      ;;
    /omnet/latency_s|/omnet/jitter_s|/omnet/sim_time)
      printf '%s\n' "s"
      ;;
    *)
      printf '%s\n' ""
      ;;
  esac
}

start_topic_capture() {
  local topic="$1"
  local state_file="$2"

  (
    ros2 topic echo --no-daemon "$topic" 2>&1 |
      stdbuf -oL awk -v out="$state_file" '
        /^[[:space:]]*data:/ {
          sub(/^[[:space:]]*data:[[:space:]]*/, "", $0)
          print $0 > out
          close(out)
          next
        }
        /^ERROR|^Error|^WARNING|^Warning/ {
          print $0 > out ".err"
          close(out ".err")
        }
      '
  ) &
  ECHO_PIDS+=("$!")
}

render_dashboard_once() {
  local topic=""
  local state_file=""
  local raw=""
  local value=""
  local note=""
  local age="-"
  local now=""
  local mtime=""
  local line=""

  now="$(date +%s)"
  printf '\033[H\033[2J'
  if [ "$TOP_PADDING_LINES" -gt 0 ]; then
    printf '%*s' "$TOP_PADDING_LINES" '' | tr ' ' '\n'
  fi
  line="----------------------------------------------------------------------------------------------------"
  printf '%s\n' "$line"
  printf '  OMNeT monitor        topics: %-3d    refresh: %-8s Hz    Ctrl-C to stop\n' "${#TOPICS[@]}" "$RATE_HZ"
  printf '%s\n' "$line"
  printf '  %-34s  %-18s  %-8s  %s\n' "topic" "value" "age" "note"
  printf '  %-34s  %-18s  %-8s  %s\n' "----------------------------------" "------------------" "--------" "----------------------------------"

  for topic in "${TOPICS[@]}"; do
    state_file="$(topic_state_file "$topic")"
    raw="waiting"
    age="-"
    if [ -s "$state_file" ]; then
      raw="$(tail -n 1 "$state_file")"
      if mtime="$(stat -c %Y "$state_file" 2>/dev/null)"; then
        age="$((now - mtime))s"
      fi
    fi
    value="$(format_value "$raw")"
    note="$(topic_note "$topic" "$raw")"
    printf '  %-34s  %-18s  %-8s  %s\n' "$(format_topic_label "$topic")" "$value" "$age" "$note"
  done
  printf '%s\n' "$line"
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
    rate:=*|hz:=*)
      RATE_HZ="$(validate_rate "${arg#*:=}")"
      ;;
    top:=*|pad:=*|padding:=*)
      TOP_PADDING_LINES="$(validate_nonnegative_int "${arg#*:=}")"
      ;;
    refresh:=*|refresh_s:=*)
      refresh_s="${arg#*:=}"
      awk -v value="$refresh_s" 'BEGIN { exit !(value + 0 > 0) }' || {
        echo "Invalid refresh value: $refresh_s" >&2
        echo "Use a positive refresh interval in seconds, for example refresh:=0.5" >&2
        exit 2
      }
      RATE_HZ="$(awk -v value="$refresh_s" 'BEGIN { printf "%.6g\n", 1.0 / value }')"
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
  echo "Mode: $( [ "$ONCE" = true ] && printf 'once' || printf 'dashboard' )"
  echo "Rate: $RATE_HZ Hz"
  echo "Top padding: $TOP_PADDING_LINES lines"
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
TOPIC_STATE_DIR="$(mktemp -d /tmp/omnet_monitor.XXXXXX)"

topic=""
for topic in "${TOPICS[@]}"; do
  start_topic_capture "$topic" "$(topic_state_file "$topic")"
done

sleep_s="$(awk -v rate="$RATE_HZ" 'BEGIN { printf "%.6f\n", 1.0 / rate }')"
while true; do
  render_dashboard_once
  sleep "$sleep_s"
done

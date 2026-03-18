#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEFAULT_BAG_ROOT="$WS_ROOT/bags/experiments"
PLAY_START_DELAY_S="1.0"
RENDER_REFRESH_S="0.25"
BLOCK_MAX_LINES=20
LEADER_FIELDS_PER_LINE=3
LEADER_KEY_WIDTH=15
LEADER_VALUE_WIDTH=12

BAG_INPUT=""
DRY_RUN=false
LOOP=false
RATE=""
TOPICS=()
PLAY_PID=""
TOPIC_STATE_DIR=""
USE_DASHBOARD=false

usage() {
  cat <<EOF
Usage: $0 <bag> [topic1 topic2 ...] [loop:=true|false] [rate:=1.0] [dry_run:=true|false]

Bag resolution:
- absolute bag path
- workspace-relative path
- path relative to bags/experiments/
- experiment run directory containing a bag/ subdirectory
- short run token such as const_test or depth_test (resolves to the latest matching experiment bag)

Examples:
  ./run.sh bag_monitor const_test
  ./run.sh bag_monitor const_test /coord/leader_estimate_status
  ./run.sh bag_monitor warehouse/const_test_yolo_0313-015514 /coord/leader_estimate_status
  ./run.sh bag_monitor warehouse/depth_test_yolo_0313-015917 /coord/leader_estimate_status /coord/leader_distance_debug
  ./run.sh bag_monitor warehouse/depth_test_yolo_0313-015917/bag /dji0/follow/actual/distance_3d_m loop:=true

When stdout is an interactive terminal, topic playback is shown as a live dashboard
with one block per topic instead of raw scrolling echo output.
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

resolve_existing_dir() {
  local path="$1"
  if [ -d "$path" ]; then
    readlink -f "$path"
    return 0
  fi
  return 1
}

bag_dir_from_candidate() {
  local candidate="$1"
  local resolved=""
  local mcaps=()

  if [ -f "$candidate" ] && [ "$(basename "$candidate")" = "metadata.yaml" ]; then
    candidate="$(dirname "$candidate")"
  fi

  if ! resolved="$(resolve_existing_dir "$candidate")"; then
    return 1
  fi

  if [ -d "$resolved/bag" ]; then
    resolved="$(readlink -f "$resolved/bag")"
  fi

  shopt -s nullglob
  mcaps=("$resolved"/bag_*.mcap)
  shopt -u nullglob
  if [ -f "$resolved/metadata.yaml" ] || [ "${#mcaps[@]}" -gt 0 ]; then
    printf '%s\n' "$resolved"
    return 0
  fi

  return 1
}

resolve_bag_dir() {
  local input="$1"
  local candidate=""
  local resolved=""
  local -a candidates=()

  if [[ "$input" = /* ]]; then
    candidates+=("$input")
  else
    candidates+=("$input" "$WS_ROOT/$input" "$DEFAULT_BAG_ROOT/$input")
  fi

  for candidate in "${candidates[@]}"; do
    if resolved="$(bag_dir_from_candidate "$candidate")"; then
      printf '%s\n' "$resolved"
      return 0
    fi
  done

  if resolved="$(resolve_bag_dir_by_token "$input")"; then
    printf '%s\n' "$resolved"
    return 0
  fi

  return 1
}

append_unique() {
  local value="$1"
  local existing=""
  for existing in "${UNIQUE_MATCHES[@]:-}"; do
    if [ "$existing" = "$value" ]; then
      return 0
    fi
  done
  UNIQUE_MATCHES+=("$value")
}

resolve_bag_dir_by_token() {
  local input="$1"
  local candidate=""
  local resolved=""
  local latest_path=""
  local latest_mtime=-1
  local candidate_mtime=0
  local -a raw_matches=()

  if [[ "$input" = /* ]]; then
    return 1
  fi

  UNIQUE_MATCHES=()

  shopt -s nullglob
  raw_matches+=("$DEFAULT_BAG_ROOT"/"$input"*)
  raw_matches+=("$DEFAULT_BAG_ROOT"/*/"$input"*)
  raw_matches+=("$DEFAULT_BAG_ROOT"/*/"$input"*/bag)
  shopt -u nullglob

  for candidate in "${raw_matches[@]}"; do
    if resolved="$(bag_dir_from_candidate "$candidate")"; then
      append_unique "$resolved"
    fi
  done

  if [ "${#UNIQUE_MATCHES[@]}" -eq 0 ]; then
    return 1
  fi

  for candidate in "${UNIQUE_MATCHES[@]}"; do
    candidate_mtime="$(stat -c '%Y' "$candidate" 2>/dev/null || printf '0')"
    if [ "$candidate_mtime" -gt "$latest_mtime" ]; then
      latest_mtime="$candidate_mtime"
      latest_path="$candidate"
    fi
  done

  if [ "${#UNIQUE_MATCHES[@]}" -gt 1 ]; then
    echo "[bag_monitor] Multiple bags matched '$input'; using latest: $latest_path" >&2
  fi

  printf '%s\n' "$latest_path"
}

start_topic_echo() {
  local topic="$1"
  local prefix="[$topic] "

  (
    ros2 topic echo --no-daemon "$topic" 2>&1 | stdbuf -oL awk -v prefix="$prefix" '{ print prefix $0; fflush(); }'
  ) &
  ECHO_PIDS+=("$!")
}

topic_state_file() {
  local topic="$1"
  local safe_name=""
  safe_name="$(printf '%s' "$topic" | tr -c 'A-Za-z0-9_.-' '_')"
  printf '%s/%s.txt\n' "$TOPIC_STATE_DIR" "$safe_name"
}

write_topic_state() {
  local file="$1"
  local updated_at="$2"
  local message_count="$3"
  shift 3
  local tmp_file="${file}.tmp.$$"

  {
    printf 'Updated: %s\n' "$updated_at"
    printf 'Messages: %s\n' "$message_count"
    printf '\n'
    if [ "$#" -gt 0 ]; then
      printf '%s\n' "$@"
    else
      printf '(empty message)\n'
    fi
  } > "$tmp_file"
  mv "$tmp_file" "$file"
}

format_topic_block() {
  local topic="$1"
  local leader_payload=""
  shift

  if [[ "$topic" == *"/leader_follow_point" || "$topic" == *"/leader_planned_target" ]]; then
    format_pose_payload "$@"
    return 0
  fi

  if leader_payload="$(extract_leader_payload "$@")"; then
    if [[ "$topic" == *"/leader_"* ]]; then
      if [[ "$topic" == *"/leader_distance_debug" ]]; then
        format_leader_distance_payload "$leader_payload"
        return 0
      fi
      format_leader_payload "$leader_payload"
      return 0
    fi
  fi

  if [ "$#" -gt 0 ]; then
    printf '%s\n' "$@"
  fi
}

extract_leader_payload() {
  local first_line="${1:-}"
  local payload=""

  if [[ "$first_line" != data:* ]]; then
    return 1
  fi

  payload="${first_line#data:}"
  payload="${payload# }"

  if [[ "$payload" == \'*\' ]]; then
    payload="${payload#\'}"
    payload="${payload%\'}"
  elif [[ "$payload" == \"*\" ]]; then
    payload="${payload#\"}"
    payload="${payload%\"}"
  fi

  if [[ "$payload" != *=* ]]; then
    return 1
  fi

  printf '%s\n' "$payload"
}

format_leader_payload() {
  local payload="$1"
  local token=""
  local cell=""
  local line=""
  local count=0
  local -a tokens=()

  read -r -a tokens <<< "$payload"

  for token in "${tokens[@]}"; do
    cell="$(format_leader_cell "${token%%=*}" "${token#*=}")"
    if [ -z "$line" ]; then
      line="$cell"
    else
      line="$line | $cell"
    fi
    count=$((count + 1))
    if [ "$count" -ge "$LEADER_FIELDS_PER_LINE" ]; then
      printf '%s\n' "$line"
      line=""
      count=0
    fi
  done

  if [ -n "$line" ]; then
    printf '%s\n' "$line"
  fi
}

format_leader_distance_payload() {
  local payload="$1"
  local range_src=""
  local range_m=""
  local est_d_m=""
  local est_xy_m=""
  local avg_est_d_m=""
  local real_d_m=""
  local real_xy_m=""
  local avg_real_d_m=""
  local err_d_m=""
  local err_xy_m=""
  local avg_err_d_m=""

  range_src="$(payload_value "$payload" "range_src")"
  range_m="$(payload_value "$payload" "range_m")"
  est_d_m="$(payload_value "$payload" "est_d_m")"
  est_xy_m="$(payload_value "$payload" "est_xy_m")"
  avg_est_d_m="$(running_metric_avg "est_d_m")"
  real_d_m="$(payload_value "$payload" "real_d_m")"
  real_xy_m="$(payload_value "$payload" "real_xy_m")"
  avg_real_d_m="$(running_metric_avg "real_d_m")"
  err_d_m="$(payload_value "$payload" "err_d_m")"
  err_xy_m="$(payload_value "$payload" "err_xy_m")"
  avg_err_d_m="$(running_metric_avg "err_d_m")"

  format_leader_cells_line "range_src" "$range_src" "range_m" "$range_m"
  format_leader_cells_line "est_d_m" "$est_d_m" "est_xy_m" "$est_xy_m" "est_avg" "$avg_est_d_m"
  format_leader_cells_line "real_d_m" "$real_d_m" "real_xy_m" "$real_xy_m" "real_avg" "$avg_real_d_m"
  format_leader_cells_line "err_d_m" "$err_d_m" "err_xy_m" "$err_xy_m" "err_avg" "$avg_err_d_m"
}

format_pose_payload() {
  local line=""
  local fp_in_pos=false
  local fp_in_ori=false
  local fp_pos_x="na" fp_pos_y="na" fp_pos_z="na"
  local fp_ori_x="0" fp_ori_y="0" fp_ori_z="0" fp_ori_w="1"
  local fp_trimmed=""

  for line in "$@"; do
    fp_trimmed="${line#"${line%%[![:space:]]*}"}"
    case "$fp_trimmed" in
      "position:")    fp_in_pos=true;  fp_in_ori=false ;;
      "orientation:") fp_in_ori=true;  fp_in_pos=false ;;
      "pose:" | "header:" | "stamp:") fp_in_pos=false; fp_in_ori=false ;;
    esac
    if $fp_in_pos; then
      case "$fp_trimmed" in
        "x: "*) fp_pos_x="${fp_trimmed#x: }" ;;
        "y: "*) fp_pos_y="${fp_trimmed#y: }" ;;
        "z: "*) fp_pos_z="${fp_trimmed#z: }" ;;
      esac
    elif $fp_in_ori; then
      case "$fp_trimmed" in
        "x: "*) fp_ori_x="${fp_trimmed#x: }" ;;
        "y: "*) fp_ori_y="${fp_trimmed#y: }" ;;
        "z: "*) fp_ori_z="${fp_trimmed#z: }" ;;
        "w: "*) fp_ori_w="${fp_trimmed#w: }" ;;
      esac
    fi
  done

  local fp_x_fmt fp_y_fmt fp_z_fmt fp_yaw
  IFS='|' read -r fp_x_fmt fp_y_fmt fp_z_fmt fp_yaw <<< \
    "$(awk -v px="$fp_pos_x" -v py="$fp_pos_y" -v pz="$fp_pos_z" \
         -v ox="$fp_ori_x" -v oy="$fp_ori_y" -v oz="$fp_ori_z" -v ow="$fp_ori_w" \
      'BEGIN {
        pi = 3.14159265358979
        s = 2*(ow*oz + ox*oy); c = 1 - 2*(oy*oy + oz*oz)
        yaw_deg = atan2(s, c) * 180 / pi
        xf = (px == "na") ? "na" : sprintf("%.3f", px+0)
        yf = (py == "na") ? "na" : sprintf("%.3f", py+0)
        zf = (pz == "na") ? "na" : sprintf("%.3f", pz+0)
        printf "%s|%s|%s|%.1f", xf, yf, zf, yaw_deg
      }')"

  format_leader_cells_line "x_m" "$fp_x_fmt" "y_m" "$fp_y_fmt" "z_m" "$fp_z_fmt"
  format_leader_cells_line "yaw_deg" "$fp_yaw"
}

format_leader_cells_line() {
  local key=""
  local value=""
  local cell=""
  local line=""

  while [ "$#" -gt 1 ]; do
    key="$1"
    value="$2"
    shift 2
    cell="$(format_leader_cell "$key" "$value")"
    if [ -z "$line" ]; then
      line="$cell"
    else
      line="$line | $cell"
    fi
  done

  printf '%s\n' "$line"
}

format_leader_cell() {
  local key="$1"
  local value="$2"
  local key_text=""
  local value_text=""

  if [ -z "$value" ]; then
    value="na"
  fi

  key_text="$(fit_text "${key}=" "$LEADER_KEY_WIDTH")"
  value_text="$(fit_text "$value" "$LEADER_VALUE_WIDTH")"
  printf '%-*s %-*s' \
    "$LEADER_KEY_WIDTH" "$key_text" \
    "$LEADER_VALUE_WIDTH" "$value_text"
}

payload_value() {
  local payload="$1"
  local wanted_key="$2"
  local token=""
  local -a tokens=()

  read -r -a tokens <<< "$payload"

  for token in "${tokens[@]}"; do
    if [ "${token%%=*}" = "$wanted_key" ]; then
      printf '%s\n' "${token#*=}"
      return 0
    fi
  done

  printf 'na\n'
}

is_numeric_value() {
  [[ "$1" =~ ^-?[0-9]+([.][0-9]+)?$ ]]
}

float_add() {
  awk -v a="$1" -v b="$2" 'BEGIN { printf "%.6f\n", a + b }'
}

float_avg() {
  awk -v total="$1" -v count="$2" 'BEGIN { if (count <= 0) { print "na" } else { printf "%.2f\n", total / count } }'
}

update_running_metric() {
  local metric="$1"
  local value="$2"

  if ! is_numeric_value "$value"; then
    return 0
  fi

  RUNNING_SUM["$metric"]="$(float_add "${RUNNING_SUM[$metric]:-0}" "$value")"
  RUNNING_COUNT["$metric"]=$(( ${RUNNING_COUNT[$metric]:-0} + 1 ))
}

running_metric_avg() {
  local metric="$1"
  float_avg "${RUNNING_SUM[$metric]:-0}" "${RUNNING_COUNT[$metric]:-0}"
}

update_distance_running_stats() {
  local payload="$1"
  local metric=""

  for metric in est_d_m est_xy_m real_d_m real_xy_m err_d_m err_xy_m; do
    update_running_metric "$metric" "$(payload_value "$payload" "$metric")"
  done
}

fit_text() {
  local text="$1"
  local width="$2"
  if [ "${#text}" -le "$width" ]; then
    printf '%s\n' "$text"
    return 0
  fi
  if [ "$width" -le 1 ]; then
    printf '%.*s\n' "$width" "$text"
    return 0
  fi
  printf '%s~\n' "${text:0:$((width - 1))}"
}

start_topic_dashboard_capture() {
  local topic="$1"
  local file="$2"

  write_topic_state "$file" "waiting" 0 "(waiting for first message...)"

  (
    local line=""
    local leader_payload=""
    local message_count=0
    local line_count=0
    local truncated=false
    local -a block_lines=()
    declare -A RUNNING_SUM=()
    declare -A RUNNING_COUNT=()

    while IFS= read -r line || [ -n "$line" ]; do
      if [ "$line" = "---" ]; then
        message_count=$((message_count + 1))
        if [ "$truncated" = true ]; then
          block_lines+=("... [truncated]")
        fi
        if [[ "$topic" == *"/leader_distance_debug" ]]; then
          leader_payload="$(extract_leader_payload "${block_lines[@]}" || true)"
          if [ -n "$leader_payload" ]; then
            update_distance_running_stats "$leader_payload"
          fi
        fi
        mapfile -t formatted_lines < <(format_topic_block "$topic" "${block_lines[@]}")
        write_topic_state "$file" "$(date +%H:%M:%S)" "$message_count" "${formatted_lines[@]}"
        block_lines=()
        line_count=0
        truncated=false
        continue
      fi

      if [ "$line_count" -lt "$BLOCK_MAX_LINES" ]; then
        block_lines+=("$line")
      else
        truncated=true
      fi
      line_count=$((line_count + 1))
    done < <(ros2 topic echo --no-daemon "$topic" 2>&1)

    if [ "${#block_lines[@]}" -gt 0 ] || [ "$truncated" = true ]; then
      message_count=$((message_count + 1))
      if [ "$truncated" = true ]; then
        block_lines+=("... [truncated]")
      fi
      if [[ "$topic" == *"/leader_distance_debug" ]]; then
        leader_payload="$(extract_leader_payload "${block_lines[@]}" || true)"
        if [ -n "$leader_payload" ]; then
          update_distance_running_stats "$leader_payload"
        fi
      fi
      mapfile -t formatted_lines < <(format_topic_block "$topic" "${block_lines[@]}")
      write_topic_state "$file" "$(date +%H:%M:%S)" "$message_count" "${formatted_lines[@]}"
    fi
  ) &
  ECHO_PIDS+=("$!")
}

render_dashboard_once() {
  local cols=100
  local rule=""
  local idx=0

  if command -v tput >/dev/null 2>&1; then
    cols="$(tput cols 2>/dev/null || printf '100')"
  fi
  if [ "$cols" -lt 60 ]; then
    cols=60
  fi
  printf -v rule '%*s' "$cols" ''
  rule="${rule// /=}"

  printf '\033[H\033[2J'
  printf '[bag_monitor] Bag: %s\n' "$BAG_DIR"
  printf '[bag_monitor] Loop: %s  Rate: %s  Refresh: %ss\n' "$LOOP" "${RATE:-default}" "$RENDER_REFRESH_S"
  printf '[bag_monitor] Ctrl-C to stop.\n\n'

  for idx in "${!TOPICS[@]}"; do
    printf '%s\n' "$rule"
    printf '%s\n' "${TOPICS[$idx]}"
    printf '%s\n' "$rule"
    cat "${TOPIC_FILES[$idx]}"
    printf '\n'
  done
}

cleanup() {
  local pid=""
  if [ -n "${PLAY_PID:-}" ]; then
    kill "$PLAY_PID" 2>/dev/null || true
  fi
  for pid in "${ECHO_PIDS[@]:-}"; do
    pkill -TERM -P "$pid" 2>/dev/null || true
    kill "$pid" 2>/dev/null || true
  done
  if [ -n "${TOPIC_STATE_DIR:-}" ] && [ -d "$TOPIC_STATE_DIR" ]; then
    rm -rf "$TOPIC_STATE_DIR"
  fi
  if [ "${USE_DASHBOARD:-false}" = true ] && [ -t 1 ]; then
    printf '\033[0m\033[?25h'
  fi
}

print_recorded_topics_if_available() {
  local topics_file
  topics_file="$(dirname "$BAG_DIR")/topics.txt"
  if [ -f "$topics_file" ]; then
    echo
    echo "[bag_monitor] Recorded topics from $topics_file:"
    cat "$topics_file"
  fi
}

all_topics_visible() {
  local visible_topics="$1"
  local topic=""
  for topic in "${TOPICS[@]}"; do
    if ! grep -Fqx "$topic" <<<"$visible_topics"; then
      return 1
    fi
  done
  return 0
}

wait_for_topics_on_graph() {
  local attempt=""
  local visible_topics=""

  for attempt in $(seq 1 40); do
    if ! kill -0 "$PLAY_PID" 2>/dev/null; then
      return 1
    fi
    visible_topics="$(ros2 topic list --no-daemon 2>/dev/null || true)"
    if all_topics_visible "$visible_topics"; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

if [ "$#" -eq 0 ]; then
  usage
  exit 1
fi

for arg in "$@"; do
  case "$arg" in
    -h|--help|help)
      usage
      exit 0
      ;;
    bag:=*)
      BAG_INPUT="${arg#bag:=}"
      ;;
    loop:=*)
      LOOP="$(coerce_bool "${arg#loop:=}")"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${arg#dry_run:=}")"
      ;;
    rate:=*)
      RATE="${arg#rate:=}"
      ;;
    *)
      if [ -z "$BAG_INPUT" ]; then
        BAG_INPUT="$arg"
      else
        TOPICS+=("$arg")
      fi
      ;;
  esac
done

if [ -z "$BAG_INPUT" ]; then
  echo "Bag path is required." >&2
  usage >&2
  exit 2
fi

if [ -n "$RATE" ]; then
  case "$RATE" in
    ''|*[!0-9.]*)
      echo "Invalid rate value: $RATE" >&2
      exit 2
      ;;
  esac
fi

if ! BAG_DIR="$(resolve_bag_dir "$BAG_INPUT")"; then
  echo "Could not resolve bag path: $BAG_INPUT" >&2
  echo "Tried relative to: $WS_ROOT and $DEFAULT_BAG_ROOT" >&2
  exit 1
fi

if [ "$DRY_RUN" = true ]; then
  echo "Bag dir: $BAG_DIR"
  if [ "${#TOPICS[@]}" -eq 0 ]; then
    echo "Action: info"
  else
    echo "Action: play"
  fi
  echo "Loop: $LOOP"
  echo "Rate: ${RATE:-default}"
  echo "Topics: ${#TOPICS[@]}"
  if [ "${#TOPICS[@]}" -gt 0 ]; then
    printf '%s\n' "${TOPICS[@]}"
  fi
  exit 0
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

if [ -t 1 ]; then
  USE_DASHBOARD=true
fi

if [ "${#TOPICS[@]}" -eq 0 ]; then
  echo "[bag_monitor] Bag: $BAG_DIR"
  ros2 bag info "$BAG_DIR"
  print_recorded_topics_if_available
  exit 0
fi

ECHO_PIDS=()
TOPIC_FILES=()
trap cleanup EXIT INT TERM

echo "[bag_monitor] Bag: $BAG_DIR"
echo "[bag_monitor] Topics: ${#TOPICS[@]}"

PLAY_CMD=(ros2 bag play "$BAG_DIR" --delay "$PLAY_START_DELAY_S")
if [ "$LOOP" = true ]; then
  PLAY_CMD+=(--loop)
fi
if [ -n "$RATE" ]; then
  PLAY_CMD+=(--rate "$RATE")
fi
PLAY_CMD+=(--topics "${TOPICS[@]}")

"${PLAY_CMD[@]}" &
PLAY_PID="$!"

if ! wait_for_topics_on_graph; then
  echo "[bag_monitor] Warning: requested topics did not appear before echo startup." >&2
fi

if [ "$USE_DASHBOARD" = true ]; then
  TOPIC_STATE_DIR="$(mktemp -d /tmp/bag_monitor.XXXXXX)"
  printf '\033[?25l'
  for topic in "${TOPICS[@]}"; do
    topic_file="$(topic_state_file "$topic")"
    TOPIC_FILES+=("$topic_file")
    start_topic_dashboard_capture "$topic" "$topic_file"
  done

  play_status=0
  while kill -0 "$PLAY_PID" 2>/dev/null; do
    render_dashboard_once
    sleep "$RENDER_REFRESH_S"
  done
  wait "$PLAY_PID" || play_status=$?
  render_dashboard_once
  exit "$play_status"
else
  for topic in "${TOPICS[@]}"; do
    start_topic_echo "$topic"
  done
  wait "$PLAY_PID"
fi

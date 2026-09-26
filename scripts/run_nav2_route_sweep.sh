#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws/nav2_route_sweep"
TMUX_STATE_DIR="/tmp/halmstad_ws/tmux_sessions"

WORLD="baylands"
MODE="yolo"
LIDAR="3d"
ROUTES_RAW="all"
EXCLUDE_ROUTES_RAW="${EXCLUDE_ROUTES_RAW:-art,playground_1,playground_2,playground_3}"
SWEEP_MODE="waypoints"
SESSION_PREFIX="halmstad-nav2-route-sweep"
TMUX_ATTACH="true"
DRY_RUN="false"
REPETITIONS=1
REPETITION_INDICES_RAW=""
STOP_BEFORE_EACH="true"
STOP_AFTER_EACH="true"
STOP_GROUP_GRACE_S="3"
STOP_FINAL_GRACE_S="3"
START_TIMEOUT_S=300
ROUTE_TIMEOUT_S=2000
ROUTE_SIM_TIMEOUT_S=""
BETWEEN_S=3
CHECK_UAV_FOLLOW="true"
UAV_NAME="dji0"
UAV_CHECK_TIMEOUT_S=120
UAV_CHECK_TOPIC_TIMEOUT_S=10
UAV_CHECK_MIN_Z=5.0
UAV_CHECK_HOLD_UGV_S=0
UAV_CHECK_REQUIRE_COMMAND="false"
SKIP_REMAINING_ON_COMPLETE="true"
CHAIN_ROUTE_STARTS="false"
FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE="false"
FALLBACK_WAYPOINTS_EXPLICIT="false"
RECORD="true"
RECORD_PROFILE="step2_light"
RECORD_ROOT=""
CAMPAIGN_TAG=""
SKIP_EXISTING_RECORD="false"
OVERWRITE_EXISTING_RECORD="false"
METRIC_WARMUP_S=30
MEASURED_METRICS_SUMMARY=""
DISTANCE_SWEEP="false"
DISTANCE_SWEEP_VALUES="auto"
DISTANCE_SWEEP_INTERVAL_S="10"
DISTANCE_SWEEP_START_DELAY_S="0"
DISTANCE_SWEEP_Z_OFFSET_M="7"
DISTANCE_SWEEP_XY_ANCHOR_MARGIN_M="2"
DISTANCE_SWEEP_NODE="/follow_uav"
DISTANCE_SWEEP_WAIT_TIMEOUT_S="30"
DISTANCE_SWEEP_ADAPT_DETECTION="false"
DISTANCE_SWEEP_STATUS_TOPIC="/coord/leader_estimate_status"
DISTANCE_SWEEP_NO_DET_WINDOW_S="5"
DISTANCE_SWEEP_Z_STEP_M="1"
DISTANCE_SWEEP_MIN_Z_OFFSET_M="2"
DISTANCE_SWEEP_RESTORE_HEIGHT="false"
CURRENT_DISTANCE_SWEEP_PID=""
EXTRA_TMUX_ARGS=()
DETECTOR_BACKEND_ARG=""
DETECTOR_ONNX_MODEL_ARG=""
HAVE_UGV_START_DELAY_ARG="false"
SWEEP_MODE_EXPLICIT="false"
CAMPAIGN_ARGS_SEEN="false"

source "$SCRIPT_DIR/baylands_route_lidar_common.sh"
source "$SCRIPT_DIR/baylands_waypoint_common.sh"
WAYPOINT_CONFIG_DIR="$(baylands_waypoint_config_dir)"
GROUP_CSV="$(baylands_group_waypoint_csv)"

usage() {
  cat <<EOF
Usage:
  ./run.sh nav2_route_sweep [world] [routes:=all|route_a,route_b] [args...]

Examples:
  ./run.sh nav2_route_sweep routes:=rotundan
  ./run.sh nav2_route_sweep routes:=rotundan,parkinglot_west
  ./run.sh nav2_route_sweep sweep:=routes
  ./run.sh nav2_route_sweep routes:=strip gui:=false
  ./run.sh nav2_route_sweep dry_run:=true

What it does:
  In sweep:=waypoints mode, start tmux_1to1 at each waypoint in a route,
  give Nav2 the remaining sliced route from that waypoint onward, wait for
  completion/failure/timeout, stop the stack, then teleport to the next waypoint.

  In sweep:=routes mode, run each route once from its first waypoint.

Options:
  routes:=all|a,b             Route YAML stems, e.g. rotundan,strip
  exclude_routes:=a,b          Routes skipped when routes:=all, default art/playground routes
  sweep:=waypoints|routes     Default waypoints
  mode:=follow|yolo           tmux_1to1 mode, default follow
  lidar:=3d                Nav2/localization lidar mode, default 3d
  session_prefix:=name        tmux session prefix
  start_timeout_s:=seconds    Time to wait for ugv_nav2_driver to start
  route_timeout_s:=seconds    Max time per route after driver starts, 0/off disables
  route_sim_timeout_s:=seconds Max simulation time per route, preferred for campaigns
  repetitions:=count          Repeat the selected route list, default 1
  repetition_indices:=1,3      Run only selected repetition numbers
  replace_repetition:=2|all    Archive and rerun selected repetition(s); accepts rep02
  campaign_tag:=name          Prefix for per-route record tags
  record:=true|false          Enable per-route recording
  record_root:=path           Root folder for per-route bags and OMNeT files
  record_profile:=profile     Recording profile passed to tmux_1to1
  skip_existing:=true|false    Skip route folders that already contain data
  overwrite_existing:=true|false Archive existing route folder before rerun
  replace_existing:=true|false Alias for overwrite_existing
  distance_sweep:=true|false Step follow distance during each route
  distance_sweep_values:=auto|a,b  auto: 10,15,...,100 for one selected sweep run
  distance_sweep_interval_s:=seconds Seconds between d_target steps, default 10
  distance_sweep_z_offset_m:=meters follow_z_offset_m during sweep, default 7
  distance_sweep_xy_anchor_margin_m:=meters Added to computed XY radius, default 2
  distance_sweep_adapt_detection:=true|false Lower sweep height on sustained NO_DET/STALE
  distance_sweep_status_topic:=topic Estimator status topic for adaptive sweep
  distance_sweep_no_det_window_s:=seconds Bad status duration before lowering height
  distance_sweep_z_step_m:=meters Height reduction step for adaptive sweep
  distance_sweep_min_z_offset_m:=meters Minimum adaptive sweep height
  distance_sweep_restore_height:=true|false Restore height after OK; default false
  lora_sf:=7..12              LoRa spreading factor forwarded to tmux_1to1/OMNeT
  lora_bw:=125kHz|250kHz      LoRa bandwidth forwarded to tmux_1to1/OMNeT
  metric_warmup_s:=seconds    Warmup skipped for measured summary metrics, default 30
  between_s:=seconds          Pause between routes
  stop_before_each:=true|false
  stop_after_each:=true|false
  check_uav_follow:=true|false
  uav_check_require_command:=true|false  Require UAV cmd topic before route, default false
  uav_check_timeout_s:=seconds
  uav_check_hold_ugv_s:=seconds  Optional UGV start delay while checking
  skip_remaining_on_complete:=true|false
  chain_route_starts:=true|false  In sweep:=routes, start each route at the previous route's last waypoint
  fallback_waypoints_on_route_failure:=true|false  In sweep:=routes, try sliced waypoint starts if the full route fails
  dry_run:=true|false

Any other tmux_1to1 argument is forwarded, such as gui:=false, rtf:=1.0,
map:=/path/map.yaml, delay_s:=20, layout:=windows, or omnet:=false.
EOF
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

sanitize_name() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
}

pad2() {
  printf '%02d' "$1"
}

normalize_repetition_index() {
  local value="$1"
  value="$(trim "$value")"
  if [[ "$value" =~ ^[Rr][Ee][Pp]([0-9]+)$ ]]; then
    value="${BASH_REMATCH[1]}"
  fi
  if ! [[ "$value" =~ ^[0-9]+$ ]]; then
    return 1
  fi
  value="$((10#$value))"
  [ "$value" -ge 1 ] || return 1
  printf '%s\n' "$value"
}

repetition_indices_are_all() {
  local raw="$1"
  raw="$(trim "$raw")"
  case "$raw" in
    ""|all|ALL|All)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

route_number_for_route() {
  local route="$1"
  local fallback="$2"
  case "$route" in
    rotundan) printf '1\n' ;;
    road_to_west) printf '2\n' ;;
    parkinglot_west) printf '3\n' ;;
    road_to_spawn) printf '4\n' ;;
    spawn) printf '5\n' ;;
    road_to_east) printf '6\n' ;;
    parkinglot_east) printf '7\n' ;;
    road_to_strip) printf '8\n' ;;
    strip) printf '9\n' ;;
    *) printf '%s\n' "$fallback" ;;
  esac
}

workspace_path() {
  local path="$1"
  if [[ "$path" = /* ]]; then
    printf '%s\n' "$path"
  else
    printf '%s/%s\n' "$WS_ROOT" "$path"
  fi
}

record_path_has_data() {
  local record_out="$1"
  local abs_path
  [ -n "$record_out" ] || return 1
  abs_path="$(workspace_path "$record_out")"

  [ -d "$abs_path" ] || return 1
  [ -f "$abs_path/metadata.json" ] && return 0
  [ -d "$abs_path/bag" ] && find "$abs_path/bag" -mindepth 1 -print -quit 2>/dev/null | grep -q . && return 0
  [ -d "$abs_path/omnet" ] && find "$abs_path/omnet" -mindepth 1 -print -quit 2>/dev/null | grep -q . && return 0
  [ -d "$abs_path/pane_logs" ] && find "$abs_path/pane_logs" -mindepth 1 -print -quit 2>/dev/null | grep -q . && return 0
  return 1
}

archive_existing_record_path() {
  local record_out="$1"
  local abs_path archive_path suffix
  [ -n "$record_out" ] || return 0
  abs_path="$(workspace_path "$record_out")"
  [ -e "$abs_path" ] || return 0

  suffix="replaced_${RUN_ID}"
  archive_path="${abs_path}.${suffix}"
  while [ -e "$archive_path" ]; do
    suffix="${suffix}_$RANDOM"
    archive_path="${abs_path}.${suffix}"
  done

  mkdir -p "$(dirname "$archive_path")"
  mv "$abs_path" "$archive_path"
  echo "Archived existing record output: $archive_path"
}

sync_job_logs() {
  local record_out="$1"
  local log_path log_dir
  shift || true
  [ "$DRY_RUN" != true ] || return 0
  [ -n "$record_out" ] || return 0

  log_dir="$(workspace_path "$record_out")/pane_logs"
  mkdir -p "$log_dir"
  for log_path in "$@"; do
    [ -f "$log_path" ] || continue
    cp "$log_path" "$log_dir/$(basename "$log_path")"
  done
}

write_record_root_summary() {
  local state="$1"
  local suffix="$2"
  local summary_dir summary_path
  [ "$DRY_RUN" != true ] || return 0
  [ "$RECORD" = true ] || return 0
  [ -n "$RECORD_ROOT" ] || return 0

  summary_dir="$(workspace_path "$RECORD_ROOT")"
  mkdir -p "$summary_dir"
  summary_path="$summary_dir/${CAMPAIGN_TAG}_${suffix}.txt"
  write_summary_file "$summary_path" "$state"
  echo "Saved campaign summary: $summary_path"
}

print_cmd() {
  printf '  '
  printf '%q ' "$@"
  printf '\n'
}

arg_value_or_default() {
  local prefix="$1"
  local default_value="$2"
  local arg=""
  shift 2
  for arg in "$@"; do
    if [[ "$arg" == "$prefix"* ]]; then
      printf '%s\n' "${arg#"$prefix"}"
      return 0
    fi
  done
  printf '%s\n' "$default_value"
}

result_context() {
  local -a lidar_args=("$@")
  local pc2ls_min pc2ls_max pc2ls_range_min pc2ls_range_max
  pc2ls_min="$(arg_value_or_default "pc2ls_min_height:=" "default" "${EXTRA_TMUX_ARGS[@]}" "${lidar_args[@]}")"
  pc2ls_max="$(arg_value_or_default "pc2ls_max_height:=" "default" "${EXTRA_TMUX_ARGS[@]}" "${lidar_args[@]}")"
  pc2ls_range_min="$(arg_value_or_default "pc2ls_range_min:=" "default" "${EXTRA_TMUX_ARGS[@]}" "${lidar_args[@]}")"
  pc2ls_range_max="$(arg_value_or_default "pc2ls_range_max:=" "default" "${EXTRA_TMUX_ARGS[@]}" "${lidar_args[@]}")"
  printf 'lidar=%s pc2ls_min_height=%s pc2ls_max_height=%s pc2ls_range_min=%s pc2ls_range_max=%s' \
    "$LIDAR" "$pc2ls_min" "$pc2ls_max" "$pc2ls_range_min" "$pc2ls_range_max"
}

generate_measured_metrics_summary() {
  [ "$DRY_RUN" != true ] || return 0
  [ "$RECORD" = true ] || return 0
  [ "${#CAMPAIGN_RESULTS[@]}" -gt 0 ] || return 0

  local input_csv="$RUN_DIR/campaign_results.csv"
  {
    printf 'repetition,route_index,route,job_label,start_waypoint,status,network_data_valid,record_out,omnet_result_dir,context\n'
    printf '%s\n' "${CAMPAIGN_RESULTS[@]}"
  } > "$input_csv"

  python3 - "$WS_ROOT" "$METRIC_WARMUP_S" "$input_csv" <<'PY'
import csv
import math
import statistics
import sys
from pathlib import Path

ws_root = Path(sys.argv[1])
warmup_s = float(sys.argv[2])
input_csv = Path(sys.argv[3])

topics = {
    "pdr": "/omnet/packet_delivery_ratio",
    "per": "/omnet/packet_error_rate",
    "lat_ms": "/omnet/latency_s",
    "jit_ms": "/omnet/jitter_s",
    "rssi": "/omnet/rssi_dbm",
    "snir": "/omnet/snir_db",
    "radio_m": "/omnet/radio_distance",
}

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:
    print(f"Measured OMNeT metrics unavailable: ROS bag reader is not available ({exc})")
    raise SystemExit(0)


def fmt(value, digits=3):
    if value is None or not math.isfinite(value):
        return "-"
    return f"{value:.{digits}f}"


def resolve_bag(record_out):
    if not record_out:
        return None
    run_dir = Path(record_out)
    if not run_dir.is_absolute():
        run_dir = ws_root / run_dir
    bag_dir = run_dir / "bag"
    if not bag_dir.exists():
        return None
    return bag_dir


def read_bag_metrics(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    wanted = {path for path in topics.values() if path in type_map}
    if not wanted:
        return None

    type_cache = {}
    values = {name: [] for name in topics}
    start_ns = None
    total_samples = 0

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if start_ns is None:
            start_ns = int(timestamp_ns)
        if topic not in wanted:
            continue
        rel_t = (int(timestamp_ns) - start_ns) * 1e-9
        if rel_t < warmup_s:
            continue
        msg_type_name = type_map.get(topic)
        if not msg_type_name:
            continue
        try:
            if msg_type_name not in type_cache:
                type_cache[msg_type_name] = get_message(msg_type_name)
            msg = deserialize_message(data, type_cache[msg_type_name])
            value = float(getattr(msg, "data"))
        except Exception:
            continue
        if not math.isfinite(value):
            continue
        if topic == "/omnet/latency_s" or topic == "/omnet/jitter_s":
            value *= 1000.0
        for name, path in topics.items():
            if path == topic:
                values[name].append(value)
                total_samples += 1
                break

    out = {"samples": total_samples}
    for name, samples in values.items():
        out[name] = statistics.fmean(samples) if samples else None
    return out


rows = list(csv.DictReader(input_csv.open("r", encoding="utf-8")))
summaries = []
for row in rows:
    bag_dir = resolve_bag(row.get("record_out", ""))
    metrics = None
    if bag_dir is not None:
        try:
            metrics = read_bag_metrics(bag_dir)
        except Exception as exc:
            metrics = {"error": str(exc), "samples": 0}
    summaries.append((row, metrics))

print(f"Measured OMNeT metrics after {warmup_s:g}s warmup:")
print("  job                         status        samples  pdr     per     lat_ms  jit_ms  rssi    snir    link_m  radio_m")

valid = []
for row, metrics in summaries:
    job = row.get("job_label", "")
    status = row.get("status", "")
    if not metrics:
        print(f"  {job[:27]:27s} {status[:12]:12s} {'-':>7s}  {'-':>6s} {'-':>6s} {'-':>7s} {'-':>7s} {'-':>7s} {'-':>7s} {'-':>7s} {'-':>8s}")
        continue
    if "error" in metrics:
        print(f"  {job[:27]:27s} {status[:12]:12s} {'err':>7s}  {metrics['error'][:60]}")
        continue
    if int(metrics.get("samples", 0)) > 0 and row.get("network_data_valid") == "true":
        valid.append(metrics)
    print(
        f"  {job[:27]:27s} {status[:12]:12s} {int(metrics.get('samples', 0)):7d}  "
        f"{fmt(metrics.get('pdr')):>6s} {fmt(metrics.get('per')):>6s} "
        f"{fmt(metrics.get('lat_ms')):>7s} {fmt(metrics.get('jit_ms')):>7s} "
        f"{fmt(metrics.get('rssi')):>7s} {fmt(metrics.get('snir')):>7s} "
        f"{fmt(metrics.get('link_m')):>7s} {fmt(metrics.get('radio_m')):>8s}"
    )

if valid:
    avg = {}
    for name in topics:
        vals = [m[name] for m in valid if m.get(name) is not None and math.isfinite(m[name])]
        avg[name] = statistics.fmean(vals) if vals else None
    print(
        "  avg valid runs: "
        f"n={len(valid)} pdr={fmt(avg.get('pdr'))} per={fmt(avg.get('per'))} "
        f"lat_ms={fmt(avg.get('lat_ms'))} jit_ms={fmt(avg.get('jit_ms'))} "
        f"rssi={fmt(avg.get('rssi'))} snir={fmt(avg.get('snir'))} "
        f"link_m={fmt(avg.get('link_m'))} radio_m={fmt(avg.get('radio_m'))}"
    )
else:
    print("  avg valid runs: none")
PY
}

discover_routes() {
  python3 - "$WAYPOINT_CONFIG_DIR" "$GROUP_CSV" <<'PY'
import csv
import os
import sys

waypoint_config_dir, group_csv = sys.argv[1:]
seen = set()

def emit(route: str) -> None:
    route = str(route).strip()
    if not route or route in {"none", "rviz"} or route in seen:
        return
    if not os.path.exists(os.path.join(waypoint_config_dir, f"baylands_waypoints_{route}.yaml")):
        return
    seen.add(route)
    print(route)

try:
    with open(group_csv, "r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            emit(row.get("group", ""))
except FileNotFoundError:
    pass

PY
}

split_routes() {
  local raw="$1"
  local part route
  if [ "$raw" = "all" ]; then
    discover_routes
    return
  fi
  IFS=',' read -ra parts <<< "$raw"
  for part in "${parts[@]}"; do
    route="$(trim "$part")"
    [ -n "$route" ] && printf '%s\n' "$route"
  done
}

route_is_excluded() {
  local route="$1"
  local part excluded
  IFS=',' read -ra parts <<< "$EXCLUDE_ROUTES_RAW"
  for part in "${parts[@]}"; do
    excluded="$(trim "$part")"
    if [ "$route" = "$excluded" ]; then
      return 0
    fi
  done
  return 1
}

first_waypoint_for_route() {
  local route="$1"
  python3 - "$route" "$WAYPOINT_CONFIG_DIR" "$GROUP_CSV" <<'PY'
import csv
import os
import sys

route, waypoint_config_dir, group_csv = sys.argv[1:]

def emit(value: str) -> None:
    value = str(value).strip()
    if value:
        print(value)
        raise SystemExit(0)

try:
    import yaml
except Exception:
    yaml = None

route_yaml = os.path.join(waypoint_config_dir, f"baylands_waypoints_{route}.yaml")
if yaml is not None and os.path.exists(route_yaml):
    with open(route_yaml, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    for waypoint in data.get("waypoints") or []:
        if isinstance(waypoint, dict):
            emit(waypoint.get("name", ""))

rows = []
try:
    with open(group_csv, "r", encoding="utf-8", newline="") as handle:
        rows.extend(csv.DictReader(handle))
except FileNotFoundError:
    pass

for row in rows:
    if str(row.get("group", "")).strip() == route:
        emit(row.get("place", ""))

preferred = f"{route}_0"
for row in rows:
    if str(row.get("place", "")).strip() == preferred:
        emit(preferred)

prefix = f"{route}_"
for row in rows:
    name = str(row.get("place", "")).strip()
    if name.startswith(prefix):
        emit(name)

raise SystemExit(f"No first waypoint found for route '{route}'")
PY
}

route_yaml_path() {
  baylands_route_yaml_path "$1"
}

route_waypoint_names() {
  local route="$1"
  local route_yaml
  route_yaml="$(route_yaml_path "$route")"
  python3 - "$route" "$route_yaml" <<'PY'
import sys

route, route_yaml = sys.argv[1:]
try:
    import yaml
except Exception as exc:
    raise SystemExit(f"PyYAML is required for waypoint sweep mode: {exc}")

with open(route_yaml, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}

for waypoint in data.get("waypoints") or []:
    if not isinstance(waypoint, dict):
        continue
    name = str(waypoint.get("name", "")).strip()
    if name:
        print(name)
PY
}

last_waypoint_for_route() {
  local route="$1"
  local last=""
  while IFS= read -r waypoint_name; do
    [ -n "$waypoint_name" ] && last="$waypoint_name"
  done < <(route_waypoint_names "$route")

  if [ -n "$last" ]; then
    printf '%s\n' "$last"
    return 0
  fi

  echo "No last waypoint found for route '$route'" >&2
  return 1
}

write_route_slice() {
  local route="$1"
  local start_index="$2"
  local out_path="$3"
  local route_yaml
  route_yaml="$(route_yaml_path "$route")"
  python3 - "$route_yaml" "$start_index" "$out_path" <<'PY'
import os
import sys

route_yaml, start_index, out_path = sys.argv[1:]
start_index = int(start_index)

import yaml

with open(route_yaml, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}

waypoints = data.get("waypoints") or []
if start_index < 0 or start_index >= len(waypoints):
    raise SystemExit(f"Waypoint slice index {start_index} is out of range for {route_yaml}")

data["waypoints"] = waypoints[start_index:]
os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, "w", encoding="utf-8") as handle:
    yaml.safe_dump(data, handle, sort_keys=False)
PY
}

append_waypoint_jobs() {
  local route="$1"
  local label_prefix="$2"
  local start_index_min="${3:-0}"
  local route_file waypoint_name slice_path index
  local -a waypoint_names
  local appended=0

  route_file="$(route_yaml_path "$route")"
  if [ ! -f "$route_file" ]; then
    echo "Skipping route '$route': missing route file $route_file" >&2
    return 1
  fi

  mapfile -t waypoint_names < <(route_waypoint_names "$route")
  if [ "${#waypoint_names[@]}" -eq 0 ]; then
    echo "Skipping route '$route': no named waypoints in $route_file" >&2
    return 1
  fi

  for index in "${!waypoint_names[@]}"; do
    if [ "$index" -lt "$start_index_min" ]; then
      continue
    fi
    waypoint_name="${waypoint_names[$index]}"
    slice_path="$RUN_DIR/slices/$(sanitize_name "${label_prefix}${route}")_${index}_$(sanitize_name "$waypoint_name").yaml"
    if [ "$DRY_RUN" != true ]; then
      write_route_slice "$route" "$index" "$slice_path"
    fi
    JOB_START_WAYPOINTS+=("$waypoint_name")
    JOB_NAV2_GOALS+=("$slice_path")
    JOB_LABELS+=("${label_prefix}${route}_${index}_${waypoint_name}")
    appended=$((appended + 1))
  done

  [ "$appended" -gt 0 ]
}

session_state_file() {
  local session_safe
  session_safe="$(sanitize_name "$1")"
  printf '%s/%s.env\n' "$TMUX_STATE_DIR" "$session_safe"
}

capture_follow_pane() {
  local session="$1"
  local out_file="$2"
  local state_file pane_id
  state_file="$(session_state_file "$session")"
  pane_id=""
  if [ -f "$state_file" ]; then
    pane_id="$(
      # Keep the tmux state file variables from leaking into this sweep script.
      # shellcheck disable=SC1090
      source "$state_file"
      printf '%s' "${FOLLOW_PANE_ID:-}"
    )"
  fi
  if [ -n "$pane_id" ]; then
    tmux capture-pane -p -S -20000 -t "$pane_id" > "$out_file" 2>/dev/null || true
  else
    tmux capture-pane -p -S -20000 -t "$session" > "$out_file" 2>/dev/null || true
  fi
}

route_driver_running() {
  pgrep -f '[u]gv_nav2_driver' >/dev/null 2>&1
}

route_timeout_disabled() {
  case "${1:-}" in
    ""|0|0.0|false|False|FALSE|off|Off|OFF|none|None|NONE|disabled|Disabled|DISABLED)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

distance_sweep_enabled() {
  case "${DISTANCE_SWEEP:-false}" in
    true|True|TRUE|1|yes|Yes|YES|on|On|ON)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

distance_sweep_adapt_enabled() {
  case "${DISTANCE_SWEEP_ADAPT_DETECTION:-false}" in
    true|True|TRUE|1|yes|Yes|YES|on|On|ON)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

calc_xy_anchor_max() {
  local d_target="$1"
  local z_offset="$2"
  local margin="$3"
  python3 - "$d_target" "$z_offset" "$margin" <<'PY'
import math
import sys

d_target = max(0.0, float(sys.argv[1]))
z_offset = max(0.0, min(float(sys.argv[2]), d_target))
margin = max(0.0, float(sys.argv[3]))
xy = math.sqrt(max(0.0, d_target * d_target - z_offset * z_offset))
print(f"{xy + margin:.6f}")
PY
}

distance_sweep_values_for_repetition() {
  local repetition="$1"
  case "$DISTANCE_SWEEP_VALUES" in
    ""|auto|Auto|AUTO)
      printf '10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100\n'
      ;;
    *)
      printf '%s\n' "$DISTANCE_SWEEP_VALUES"
      ;;
  esac
}

start_distance_sweep() {
  local job_label="$1"
  local log_path="$2"
  local values="$3"
  local interval_s="$DISTANCE_SWEEP_INTERVAL_S"
  local start_delay_s="$DISTANCE_SWEEP_START_DELAY_S"
  local z_offset="$DISTANCE_SWEEP_Z_OFFSET_M"
  local xy_margin="$DISTANCE_SWEEP_XY_ANCHOR_MARGIN_M"
  local node="$DISTANCE_SWEEP_NODE"
  local wait_timeout_s="$DISTANCE_SWEEP_WAIT_TIMEOUT_S"
  local adapt_detection="$DISTANCE_SWEEP_ADAPT_DETECTION"
  local status_topic="$DISTANCE_SWEEP_STATUS_TOPIC"
  local no_det_window_s="$DISTANCE_SWEEP_NO_DET_WINDOW_S"
  local z_step_m="$DISTANCE_SWEEP_Z_STEP_M"
  local min_z_offset_m="$DISTANCE_SWEEP_MIN_Z_OFFSET_M"
  local restore_height="$DISTANCE_SWEEP_RESTORE_HEIGHT"

  mkdir -p "$(dirname "$log_path")"
  (
    set +u
    source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
    source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    set -u

    adapt_enabled() {
      case "${adapt_detection:-false}" in
        true|True|TRUE|1|yes|Yes|YES|on|On|ON)
          return 0
          ;;
        *)
          return 1
          ;;
      esac
    }

    restore_enabled() {
      case "${restore_height:-false}" in
        true|True|TRUE|1|yes|Yes|YES|on|On|ON)
          return 0
          ;;
        *)
          return 1
          ;;
      esac
    }

    clamp_sweep_z() {
      python3 - "$1" "$2" <<'PY'
import sys
d = max(0.0, float(sys.argv[1]))
z = max(0.0, float(sys.argv[2]))
print(f"{min(z, d):.6f}")
PY
    }

    lower_sweep_z() {
      python3 - "$1" "$2" "$3" <<'PY'
import sys
current = float(sys.argv[1])
step = max(0.0, float(sys.argv[2]))
min_z = max(0.0, float(sys.argv[3]))
print(f"{max(min_z, current - step):.6f}")
PY
    }

    float_gt() {
      awk -v a="$1" -v b="$2" 'BEGIN { exit !(a > b) }'
    }

    elapsed_reached() {
      awk -v start="$1" -v window="$2" 'BEGIN { exit !((systime() - start) >= window) }'
    }

    parse_status_state() {
      local file="$1"
      local raw token
      [ -s "$file" ] || {
        printf 'UNKNOWN\n'
        return 0
      }
      raw="$(tail -n 1 "$file")"
      raw="${raw#*|}"
      for token in $raw; do
        case "$token" in
          state=*)
            printf '%s\n' "${token#state=}" | tr '[:lower:]' '[:upper:]'
            return 0
            ;;
        esac
      done
      printf 'UNKNOWN\n'
    }

    apply_sweep_params() {
      local d_target="$1"
      local step_z="$2"
      local reason="$3"
      local xy_anchor
      xy_anchor="$(calc_xy_anchor_max "$d_target" "$step_z" "$xy_margin")"
      echo "[distance_sweep] d_target=$d_target follow_z_offset_m=$step_z xy_anchor_max=$xy_anchor reason=$reason"
      ros2 param set --no-daemon "$node" follow_z_offset_m "$step_z" || true
      ros2 param set --no-daemon "$node" xy_anchor_max "$xy_anchor" || true
      ros2 param set --no-daemon "$node" d_target "$d_target" || true
      # Visual-follow helpers use the same d_target/z pair when present.
      ros2 param set --no-daemon /follow_point_generator follow_z_offset_m "$step_z" >/dev/null 2>&1 || true
      ros2 param set --no-daemon /follow_point_generator d_target "$d_target" >/dev/null 2>&1 || true
    }

    status_file=""
    status_pid=""
    if adapt_enabled; then
      status_file="${log_path}.status"
      rm -f "$status_file"
      echo "[distance_sweep] adaptive detection enabled status_topic=$status_topic no_det_window_s=$no_det_window_s z_step_m=$z_step_m min_z_offset_m=$min_z_offset_m restore_height=$restore_height"
      (
        ros2 topic echo --no-daemon "$status_topic" std_msgs/msg/String --field data 2>/dev/null \
          | stdbuf -oL awk -v out="$status_file" '/(^|[[:space:]])state=/{ print systime() "|" $0 > out; fflush(out) }'
      ) &
      status_pid="$!"
      trap 'if [ -n "${status_pid:-}" ]; then kill "$status_pid" >/dev/null 2>&1 || true; wait "$status_pid" >/dev/null 2>&1 || true; fi' EXIT
    fi

    echo "[distance_sweep] job=$job_label node=$node values=$values interval_s=$interval_s z_offset_m=$z_offset xy_margin_m=$xy_margin"
    deadline="$(awk -v now="$(date +%s)" -v timeout="$wait_timeout_s" 'BEGIN { printf "%.0f\n", now + timeout }')"
    while [ "$(date +%s)" -le "$deadline" ]; do
      if ros2 param get --no-daemon "$node" d_target >/dev/null 2>&1; then
        break
      fi
      echo "[distance_sweep] waiting for $node.d_target ..."
      sleep 1
    done
    if ! ros2 param get --no-daemon "$node" d_target >/dev/null 2>&1; then
      echo "[distance_sweep] $node is not ready after ${wait_timeout_s}s" >&2
      exit 1
    fi

    if ! awk -v value="$start_delay_s" 'BEGIN { exit !(value > 0.0) }'; then
      :
    else
      echo "[distance_sweep] start delay ${start_delay_s}s"
      sleep "$start_delay_s"
    fi

    current_max_z="$z_offset"
    bad_since=""
    while true; do
      IFS=',' read -ra sweep_parts <<< "$values"
      for raw_value in "${sweep_parts[@]}"; do
        value="$(trim "$raw_value")"
        [ -n "$value" ] || continue
        step_z="$(clamp_sweep_z "$value" "$current_max_z")"
        apply_sweep_params "$value" "$step_z" "distance_step"
        step_start_s="$(date +%s)"
        while ! elapsed_reached "$step_start_s" "$interval_s"; do
          sleep 1
          adapt_enabled || continue
          state="$(parse_status_state "$status_file")"
          case "$state" in
            NO_DET|STALE)
              if [ -z "$bad_since" ]; then
                bad_since="$(date +%s)"
                echo "[distance_sweep] status_bad_start state=$state d_target=$value current_max_z=$current_max_z"
              elif elapsed_reached "$bad_since" "$no_det_window_s" && float_gt "$current_max_z" "$min_z_offset_m"; then
                old_max_z="$current_max_z"
                current_max_z="$(lower_sweep_z "$current_max_z" "$z_step_m" "$min_z_offset_m")"
                step_z="$(clamp_sweep_z "$value" "$current_max_z")"
                echo "[distance_sweep] adaptive_height_reduce state=$state old_max_z=$old_max_z new_max_z=$current_max_z d_target=$value"
                apply_sweep_params "$value" "$step_z" "adaptive_${state}"
                bad_since="$(date +%s)"
              fi
              ;;
            OK)
              if [ -n "$bad_since" ]; then
                echo "[distance_sweep] status_ok current_max_z=$current_max_z restore_height=$restore_height"
              fi
              bad_since=""
              if restore_enabled && float_gt "$z_offset" "$current_max_z"; then
                current_max_z="$z_offset"
                step_z="$(clamp_sweep_z "$value" "$current_max_z")"
                echo "[distance_sweep] adaptive_height_restore max_z=$current_max_z d_target=$value"
                apply_sweep_params "$value" "$step_z" "adaptive_restore"
              fi
              ;;
            *)
              :
              ;;
          esac
        done
      done
    done
  ) > "$log_path" 2>&1 &
  CURRENT_DISTANCE_SWEEP_PID="$!"
  echo "Started distance sweep for '$job_label' (pid $CURRENT_DISTANCE_SWEEP_PID, log $log_path)"
}

stop_distance_sweep() {
  if [ -n "${CURRENT_DISTANCE_SWEEP_PID:-}" ]; then
    if kill -0 "$CURRENT_DISTANCE_SWEEP_PID" >/dev/null 2>&1; then
      kill "$CURRENT_DISTANCE_SWEEP_PID" >/dev/null 2>&1 || true
      wait "$CURRENT_DISTANCE_SWEEP_PID" >/dev/null 2>&1 || true
    fi
    CURRENT_DISTANCE_SWEEP_PID=""
  fi
}

current_sim_time_s() {
  timeout 4s bash -lc "
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source '$WS_ROOT/install/setup.bash' >/dev/null 2>&1
export ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-3}'
export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}'
set -u
ros2 topic echo --no-daemon --once /clock
" 2>/dev/null | awk '
    /^[[:space:]]*sec:/ { sec = $2 }
    /^[[:space:]]*nanosec:/ { nsec = $2 }
    END {
      if (sec == "") exit 1
      printf "%.9f\n", sec + (nsec / 1000000000.0)
    }
  '
}

ros_topic_echo_once() {
  local topic="$1"
  local out_file="$2"
  local timeout_s="${3:-$UAV_CHECK_TOPIC_TIMEOUT_S}"
  timeout "${timeout_s}s" bash -lc "
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source '$WS_ROOT/install/setup.bash' >/dev/null 2>&1
export ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-3}'
export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}'
set -u
ros2 topic echo --no-daemon --once '$topic'
" > "$out_file" 2>&1
}

pose_z_is_valid() {
  local pose_file="$1"
  local min_z="$2"
  python3 - "$pose_file" "$min_z" <<'PY'
import math
import sys
from pathlib import Path

import yaml

path = Path(sys.argv[1])
min_z = float(sys.argv[2])
docs = [doc for doc in yaml.safe_load_all(path.read_text(encoding="utf-8")) if isinstance(doc, dict)]

for doc in reversed(docs):
    pose = doc.get("pose") or {}
    if isinstance(pose, dict) and "pose" in pose:
        pose = pose.get("pose") or {}
    position = pose.get("position") or {}
    try:
        z = float(position["z"])
    except Exception:
        continue
    if math.isfinite(z) and z >= min_z:
        raise SystemExit(0)

raise SystemExit(1)
PY
}

check_uav_follow_ready() {
  local session="$1"
  local label="$2"
  local check_log="$3"
  local deadline=$((SECONDS + UAV_CHECK_TIMEOUT_S))
  local pose_tmp cmd_tmp pane_tmp
  pose_tmp="$(mktemp)"
  cmd_tmp="$(mktemp)"
  pane_tmp="$(mktemp)"

  {
    echo "[$label] checking UAV follow health"
    echo "[$label] require pose=/$(printf '%s' "$UAV_NAME")/pose z>=${UAV_CHECK_MIN_Z}"
    if [ "$UAV_CHECK_REQUIRE_COMMAND" = true ]; then
      echo "[$label] require command=/$(printf '%s' "$UAV_NAME")/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
    else
      echo "[$label] command topic is optional; pose/spawn health is enough"
    fi
  } > "$check_log"

  while (( SECONDS < deadline )); do
    capture_follow_pane "$session" "$pane_tmp"
    if grep -Eq 'process has died.*(follow_uav|uav_simulator|simulator)|\[(follow_uav|follow_uav_odom|simulator)-[0-9]+\].*Traceback' "$pane_tmp" 2>/dev/null; then
      cat "$pane_tmp" >> "$check_log"
      echo "[$label] UAV follow health failed: follow/simulator process died" | tee -a "$check_log" >&2
      rm -f "$pose_tmp" "$cmd_tmp" "$pane_tmp"
      return 1
    fi

    if ! ros_topic_echo_once "/$UAV_NAME/pose" "$pose_tmp" "$UAV_CHECK_TOPIC_TIMEOUT_S"; then
      echo "[$label] waiting for /$UAV_NAME/pose" >> "$check_log"
      sleep 2
      continue
    fi

    if ! pose_z_is_valid "$pose_tmp" "$UAV_CHECK_MIN_Z"; then
      echo "[$label] /$UAV_NAME/pose exists but z is below ${UAV_CHECK_MIN_Z}" >> "$check_log"
      sleep 2
      continue
    fi

    if [ "$UAV_CHECK_REQUIRE_COMMAND" = true ]; then
      if ! ros_topic_echo_once "/$UAV_NAME/psdk_ros2/flight_control_setpoint_ENUposition_yaw" "$cmd_tmp" "$UAV_CHECK_TOPIC_TIMEOUT_S"; then
        echo "[$label] waiting for UAV command topic" >> "$check_log"
        sleep 2
        continue
      fi
    elif ros_topic_echo_once "/$UAV_NAME/psdk_ros2/flight_control_setpoint_ENUposition_yaw" "$cmd_tmp" "$UAV_CHECK_TOPIC_TIMEOUT_S"; then
      echo "[$label] optional UAV command topic already visible" >> "$check_log"
    else
      echo "[$label] optional UAV command topic not visible yet; continuing because pose is healthy" >> "$check_log"
    fi

    echo "[$label] UAV follow health OK" | tee -a "$check_log"
    rm -f "$pose_tmp" "$cmd_tmp" "$pane_tmp"
    return 0
  done

  capture_follow_pane "$session" "$pane_tmp"
  cat "$pane_tmp" >> "$check_log"
  echo "[$label] UAV follow health timed out after ${UAV_CHECK_TIMEOUT_S}s" | tee -a "$check_log" >&2
  rm -f "$pose_tmp" "$cmd_tmp" "$pane_tmp"
  return 1
}

stop_session() {
  local session="$1"
  local cmd=(./stop.sh tmux_1to1 "$WORLD" "session:=$session" "group_grace_s:=$STOP_GROUP_GRACE_S" "final_grace_s:=$STOP_FINAL_GRACE_S")
  if [ "$DRY_RUN" = true ]; then
    cmd+=("dry_run:=true")
    echo "[dry-run] stop:"
    print_cmd "${cmd[@]}"
    return 0
  fi
  "${cmd[@]}" || true
}

classify_route_log() {
  local log_file="$1"
  if grep -q "UGV Nav2 motion complete" "$log_file" 2>/dev/null; then
    printf 'completed'
  elif grep -Eq "Nav2 goal failed|Goal failed|Aborting handle|Resulting plan has 0 poses|Timed out" "$log_file" 2>/dev/null; then
    printf 'failed'
  else
    printf 'finished'
  fi
}

wait_for_route_driver_start() {
  local session="$1"
  local route="$2"
  local tmp_log="$3"
  local deadline=$((SECONDS + START_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if route_driver_running; then
      return 0
    fi
    capture_follow_pane "$session" "$tmp_log"
    if grep -Eq "UGV Nav2 motion|Generated .* motion waypoints|UGV Nav2 motion complete|Nav2 goal failed" "$tmp_log" 2>/dev/null; then
      return 0
    fi
    sleep 3
  done
  echo "Route '$route' did not start ugv_nav2_driver within ${START_TIMEOUT_S}s" >&2
  return 1
}

wait_for_route_done() {
  local session="$1"
  local route="$2"
  local route_log="$3"

  if route_timeout_disabled "$ROUTE_TIMEOUT_S" && route_timeout_disabled "$ROUTE_SIM_TIMEOUT_S"; then
    while route_driver_running; do
      sleep 5
    done
    capture_follow_pane "$session" "$route_log"
    return 0
  fi

  local deadline=0
  if ! route_timeout_disabled "$ROUTE_TIMEOUT_S"; then
    deadline=$((SECONDS + ROUTE_TIMEOUT_S))
  fi
  local sim_start=""
  local sim_now=""
  local sim_elapsed=""
  if ! route_timeout_disabled "$ROUTE_SIM_TIMEOUT_S"; then
    sim_start="$(current_sim_time_s || true)"
    if [ -n "$sim_start" ]; then
      echo "Route '$route' sim-time timer started at ${sim_start}s; timeout=${ROUTE_SIM_TIMEOUT_S}s"
    else
      echo "Route '$route' could not read /clock for sim-time timeout; falling back to wall timeout if enabled" >&2
    fi
  fi

  while route_driver_running; do
    if [ -n "$sim_start" ]; then
      sim_now="$(current_sim_time_s || true)"
      if [ -n "$sim_now" ]; then
        sim_elapsed="$(awk -v now="$sim_now" -v start="$sim_start" 'BEGIN { printf "%.6f", now - start }')"
        if awk -v elapsed="$sim_elapsed" -v limit="$ROUTE_SIM_TIMEOUT_S" 'BEGIN { exit !(elapsed >= limit) }'; then
          capture_follow_pane "$session" "$route_log"
          echo "Route '$route' timed out after ${ROUTE_SIM_TIMEOUT_S}s sim-time" >&2
          return 124
        fi
      fi
    fi
    if [ "$deadline" -gt 0 ] && (( SECONDS >= deadline )); then
      capture_follow_pane "$session" "$route_log"
      echo "Route '$route' timed out after ${ROUTE_TIMEOUT_S}s" >&2
      return 124
    fi
    sleep 5
  done

  capture_follow_pane "$session" "$route_log"
  return 0
}

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    routes:=*|nav2_goals:=*)
      ROUTES_RAW="${arg#*:=}"
      ;;
    exclude_routes:=*)
      EXCLUDE_ROUTES_RAW="${arg#exclude_routes:=}"
      ;;
    sweep:=*|sweep_mode:=*)
      SWEEP_MODE="${arg#*:=}"
      SWEEP_MODE_EXPLICIT="true"
      ;;
    mode:=*|stack:=*)
      MODE="${arg#*:=}"
      ;;
    lidar:=3d|scan_sensor:=3d)
      LIDAR="3d"
      ;;
    session_prefix:=*)
      SESSION_PREFIX="${arg#session_prefix:=}"
      ;;
    tmux_attach:=*|attach:=*)
      TMUX_ATTACH="${arg#*:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    start_timeout_s:=*)
      START_TIMEOUT_S="${arg#start_timeout_s:=}"
      ;;
    route_timeout_s:=*)
      ROUTE_TIMEOUT_S="${arg#route_timeout_s:=}"
      ;;
    route_sim_timeout_s:=*|sim_timeout_s:=*)
      ROUTE_SIM_TIMEOUT_S="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    metric_warmup_s:=*|metrics_warmup_s:=*)
      METRIC_WARMUP_S="${arg#*:=}"
      ;;
    repetitions:=*|reps:=*)
      REPETITIONS="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    repetition_indices:=*|rep_indices:=*|only_repetitions:=*)
      REPETITION_INDICES_RAW="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    replace_repetition:=*|replace_repetitions:=*|replace_rep:=*)
      REPETITION_INDICES_RAW="${arg#*:=}"
      OVERWRITE_EXISTING_RECORD="true"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    campaign_tag:=*)
      CAMPAIGN_TAG="${arg#campaign_tag:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    record:=*)
      RECORD="${arg#record:=}"
      ;;
    record_profile:=*)
      RECORD_PROFILE="${arg#record_profile:=}"
      ;;
    record_root:=*)
      RECORD_ROOT="${arg#record_root:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    skip_existing:=*|skip_existing_records:=*)
      SKIP_EXISTING_RECORD="${arg#*:=}"
      ;;
    overwrite_existing:=*|overwrite_records:=*|replace_existing:=*)
      OVERWRITE_EXISTING_RECORD="${arg#*:=}"
      ;;
    distance_sweep:=*|d_sweep:=*)
      DISTANCE_SWEEP="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_values:=*|d_sweep_values:=*|sweep_distances:=*|d_targets:=*)
      DISTANCE_SWEEP_VALUES="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_interval_s:=*|d_sweep_interval_s:=*|distance_interval_s:=*)
      DISTANCE_SWEEP_INTERVAL_S="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_start_delay_s:=*|d_sweep_start_delay_s:=*)
      DISTANCE_SWEEP_START_DELAY_S="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_z_offset_m:=*|d_sweep_z_offset_m:=*|sweep_follow_z_offset_m:=*)
      DISTANCE_SWEEP_Z_OFFSET_M="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_xy_anchor_margin_m:=*|d_sweep_xy_anchor_margin_m:=*)
      DISTANCE_SWEEP_XY_ANCHOR_MARGIN_M="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_node:=*|d_sweep_node:=*)
      DISTANCE_SWEEP_NODE="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_wait_timeout_s:=*|d_sweep_wait_timeout_s:=*)
      DISTANCE_SWEEP_WAIT_TIMEOUT_S="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_adapt_detection:=*|d_sweep_adapt_detection:=*)
      DISTANCE_SWEEP_ADAPT_DETECTION="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_status_topic:=*|d_sweep_status_topic:=*)
      DISTANCE_SWEEP_STATUS_TOPIC="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_no_det_window_s:=*|d_sweep_no_det_window_s:=*)
      DISTANCE_SWEEP_NO_DET_WINDOW_S="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_z_step_m:=*|d_sweep_z_step_m:=*)
      DISTANCE_SWEEP_Z_STEP_M="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_min_z_offset_m:=*|d_sweep_min_z_offset_m:=*)
      DISTANCE_SWEEP_MIN_Z_OFFSET_M="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    distance_sweep_restore_height:=*|d_sweep_restore_height:=*)
      DISTANCE_SWEEP_RESTORE_HEIGHT="${arg#*:=}"
      CAMPAIGN_ARGS_SEEN="true"
      ;;
    between_s:=*)
      BETWEEN_S="${arg#between_s:=}"
      ;;
    check_uav_follow:=*)
      CHECK_UAV_FOLLOW="${arg#check_uav_follow:=}"
      ;;
    uav_name:=*)
      UAV_NAME="${arg#uav_name:=}"
      EXTRA_TMUX_ARGS+=("$arg")
      ;;
    uav_check_timeout_s:=*)
      UAV_CHECK_TIMEOUT_S="${arg#uav_check_timeout_s:=}"
      ;;
    uav_check_topic_timeout_s:=*)
      UAV_CHECK_TOPIC_TIMEOUT_S="${arg#uav_check_topic_timeout_s:=}"
      ;;
    uav_check_min_z:=*)
      UAV_CHECK_MIN_Z="${arg#uav_check_min_z:=}"
      ;;
    uav_check_hold_ugv_s:=*)
      UAV_CHECK_HOLD_UGV_S="${arg#uav_check_hold_ugv_s:=}"
      ;;
    uav_check_require_command:=*)
      UAV_CHECK_REQUIRE_COMMAND="${arg#uav_check_require_command:=}"
      ;;
    skip_remaining_on_complete:=*)
      SKIP_REMAINING_ON_COMPLETE="${arg#skip_remaining_on_complete:=}"
      ;;
    chain_route_starts:=*|chain_routes:=*)
      CHAIN_ROUTE_STARTS="${arg#*:=}"
      ;;
    fallback_waypoints_on_route_failure:=*|fallback_waypoints:=*)
      FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE="${arg#*:=}"
      FALLBACK_WAYPOINTS_EXPLICIT="true"
      ;;
    stop_before_each:=*)
      STOP_BEFORE_EACH="${arg#stop_before_each:=}"
      ;;
    stop_after_each:=*|stop_after:=*)
      STOP_AFTER_EACH="${arg#*:=}"
      ;;
    stop_group_grace_s:=*)
      STOP_GROUP_GRACE_S="${arg#stop_group_grace_s:=}"
      ;;
    stop_final_grace_s:=*)
      STOP_FINAL_GRACE_S="${arg#stop_final_grace_s:=}"
      ;;
    waypoint_config_dir:=*)
      WAYPOINT_CONFIG_DIR="${arg#waypoint_config_dir:=}"
      ;;
    group_csv:=*)
      GROUP_CSV="${arg#group_csv:=}"
      ;;
    waypoint_csv:=*)
      echo "waypoint_csv:= is deprecated; use group_csv:= with waypoints_baylands_groups.csv as the single source." >&2
      exit 2
      ;;
    session:=*)
      SESSION_PREFIX="${arg#session:=}"
      ;;
    ugv_start_delay_s:=*)
      HAVE_UGV_START_DELAY_ARG="true"
      EXTRA_TMUX_ARGS+=("$arg")
      ;;
    detector_backend:=*)
      DETECTOR_BACKEND_ARG="${arg#detector_backend:=}"
      EXTRA_TMUX_ARGS+=("$arg")
      ;;
    detector_onnx_model:=*)
      DETECTOR_ONNX_MODEL_ARG="$arg"
      ;;
    onnx_model:=*)
      DETECTOR_ONNX_MODEL_ARG="detector_onnx_model:=${arg#onnx_model:=}"
      ;;
    lora_sf:=*|omnet_lora_sf:=*|sf:=*|lora_bw:=*|omnet_lora_bw:=*|bw:=*)
      EXTRA_TMUX_ARGS+=("$arg")
      ;;
    *)
      EXTRA_TMUX_ARGS+=("$arg")
      ;;
  esac
done

case "$DETECTOR_BACKEND_ARG" in
  onnx|onnxruntime|onnx_cpu|onnx_directml)
    if [ -n "$DETECTOR_ONNX_MODEL_ARG" ]; then
      EXTRA_TMUX_ARGS+=("$DETECTOR_ONNX_MODEL_ARG")
    fi
    ;;
  *)
    if [ -n "$DETECTOR_ONNX_MODEL_ARG" ]; then
      echo "[run_nav2_route_sweep] Ignoring $DETECTOR_ONNX_MODEL_ARG because detector_backend is '${DETECTOR_BACKEND_ARG:-default ultralytics}'." >&2
    fi
    ;;
esac

REPETITIONS_LIST=()
if repetition_indices_are_all "$REPETITION_INDICES_RAW"; then
  if ! [[ "$REPETITIONS" =~ ^[0-9]+$ ]] || [ "$REPETITIONS" -lt 1 ]; then
    echo "Invalid repetitions: $REPETITIONS" >&2
    exit 2
  fi
  mapfile -t REPETITIONS_LIST < <(seq 1 "$REPETITIONS")
else
  IFS=',' read -ra repetition_parts <<< "$REPETITION_INDICES_RAW"
  for repetition_part in "${repetition_parts[@]}"; do
    repetition_part_raw="$(trim "$repetition_part")"
    repetition_part="$(normalize_repetition_index "$repetition_part_raw" || true)"
    if [ -z "$repetition_part" ]; then
      echo "Invalid repetition index: $repetition_part_raw" >&2
      exit 2
    fi
    REPETITIONS_LIST+=("$repetition_part")
  done
  if [ "${#REPETITIONS_LIST[@]}" -eq 0 ]; then
    echo "No repetition indices selected." >&2
    exit 2
  fi
fi

if { [ "${#REPETITIONS_LIST[@]}" -gt 1 ] || [ "$CAMPAIGN_ARGS_SEEN" = "true" ]; } && [ "$SWEEP_MODE_EXPLICIT" != "true" ]; then
  SWEEP_MODE="routes"
fi

if { [ "${#REPETITIONS_LIST[@]}" -gt 1 ] || [ "$CAMPAIGN_ARGS_SEEN" = "true" ]; } && [ "$FALLBACK_WAYPOINTS_EXPLICIT" != "true" ]; then
  FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE="true"
fi

if [ "$RECORD" = true ] && [ -z "$RECORD_ROOT" ]; then
  RECORD_ROOT="bags/nav2_route_sweep"
fi

if [ -z "$CAMPAIGN_TAG" ]; then
  CAMPAIGN_TAG="nav2_route_sweep"
fi

if [[ "$WORLD" == baylands* ]]; then
  baylands_sync_waypoints "$DRY_RUN"
fi

mapfile -t ROUTES < <(split_routes "$ROUTES_RAW")
FILTERED_ROUTES=()
for route in "${ROUTES[@]}"; do
  if route_is_excluded "$route"; then
    continue
  fi
  FILTERED_ROUTES+=("$route")
done
ROUTES=("${FILTERED_ROUTES[@]}")
if [ "${#ROUTES[@]}" -eq 0 ]; then
  echo "No routes selected." >&2
  exit 2
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$STATE_DIR/$RUN_ID"
if [ "$DRY_RUN" != true ]; then
  mkdir -p "$RUN_DIR"
fi

RESULTS=()
CAMPAIGN_RESULTS=()
CURRENT_SESSION=""
CURRENT_JOB_LABEL=""
CURRENT_START_WAYPOINT=""
CURRENT_RUN_CONTEXT=""

write_summary_file() {
  local path="$1"
  local state="$2"
  {
    printf 'state: %s\n' "$state"
    printf 'run_id: %s\n' "$RUN_ID"
    printf 'world: %s\n' "$WORLD"
    printf 'mode: %s\n' "$MODE"
    printf 'lidar: %s\n' "$LIDAR"
    printf 'sweep: %s\n' "$SWEEP_MODE"
    printf 'repetitions: %s\n' "${REPETITIONS_LIST[*]}"
    printf 'route_sim_timeout_s: %s\n' "${ROUTE_SIM_TIMEOUT_S:-disabled}"
    printf 'record: %s\n' "$RECORD"
    printf 'record_root: %s\n' "${RECORD_ROOT:-}"
    printf 'campaign_tag: %s\n' "$CAMPAIGN_TAG"
    printf 'skip_existing: %s\n' "$SKIP_EXISTING_RECORD"
    printf 'overwrite_existing: %s\n' "$OVERWRITE_EXISTING_RECORD"
    printf 'distance_sweep: %s\n' "$DISTANCE_SWEEP"
    if distance_sweep_enabled; then
      printf 'distance_sweep_values: %s\n' "$DISTANCE_SWEEP_VALUES"
      printf 'distance_sweep_interval_s: %s\n' "$DISTANCE_SWEEP_INTERVAL_S"
      printf 'distance_sweep_z_offset_m: %s\n' "$DISTANCE_SWEEP_Z_OFFSET_M"
      printf 'distance_sweep_xy_anchor_margin_m: %s\n' "$DISTANCE_SWEEP_XY_ANCHOR_MARGIN_M"
      printf 'distance_sweep_node: %s\n' "$DISTANCE_SWEEP_NODE"
    fi
    printf 'chain_route_starts: %s\n' "$CHAIN_ROUTE_STARTS"
    printf 'fallback_waypoints_on_route_failure: %s\n' "$FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE"
    printf 'exclude_routes: %s\n' "$EXCLUDE_ROUTES_RAW"
    printf 'routes: %s\n' "${ROUTES[*]}"
    if [ -n "$CURRENT_SESSION" ]; then
      printf 'active_session: %s\n' "$CURRENT_SESSION"
      printf 'active_job: %s\n' "$CURRENT_JOB_LABEL"
      printf 'active_start_waypoint: %s\n' "$CURRENT_START_WAYPOINT"
      printf 'active_context: %s\n' "$CURRENT_RUN_CONTEXT"
    fi
    printf '\nresults:\n'
    if [ "${#RESULTS[@]}" -gt 0 ]; then
      printf '%s\n' "${RESULTS[@]}"
    fi
    printf '\ncampaign_results_csv:\n'
    printf 'repetition,route_index,route,job_label,start_waypoint,status,network_data_valid,record_out,omnet_result_dir,context\n'
    if [ "${#CAMPAIGN_RESULTS[@]}" -gt 0 ]; then
      printf '%s\n' "${CAMPAIGN_RESULTS[@]}"
    fi
    if [ -n "$MEASURED_METRICS_SUMMARY" ]; then
      printf '\nmeasured_metrics_summary:\n'
      printf '%s\n' "$MEASURED_METRICS_SUMMARY"
    fi
  } > "$path"
}

cleanup_on_interrupt() {
  echo
  echo "Interrupted."
  stop_distance_sweep
  if [ "$DRY_RUN" != true ]; then
    partial_summary="$RUN_DIR/summary.partial.txt"
    write_summary_file "$partial_summary" "interrupted"
    echo "Saved partial summary: $partial_summary"
    write_record_root_summary "interrupted" "summary.partial"
  fi
  if [ -n "$CURRENT_SESSION" ]; then
    stop_session "$CURRENT_SESSION"
  fi
  exit 130
}
trap cleanup_on_interrupt INT TERM

echo "Nav2 route sweep"
echo "  world: $WORLD"
echo "  mode: $MODE"
echo "  lidar: $LIDAR"
echo "  sweep: $SWEEP_MODE"
echo "  repetitions: ${REPETITIONS_LIST[*]}"
if route_timeout_disabled "$ROUTE_TIMEOUT_S"; then
  echo "  route timeout: disabled"
else
  echo "  route timeout: ${ROUTE_TIMEOUT_S}s"
fi
if route_timeout_disabled "$ROUTE_SIM_TIMEOUT_S"; then
  echo "  route sim timeout: disabled"
else
  echo "  route sim timeout: ${ROUTE_SIM_TIMEOUT_S}s"
fi
echo "  record: $RECORD"
if [ "$RECORD" = true ]; then
  echo "  record root: $RECORD_ROOT"
  echo "  record profile: $RECORD_PROFILE"
  echo "  campaign tag: $CAMPAIGN_TAG"
  echo "  skip existing: $SKIP_EXISTING_RECORD"
  echo "  overwrite existing: $OVERWRITE_EXISTING_RECORD"
fi
echo "  uav follow check: $CHECK_UAV_FOLLOW"
echo "  uav command required: $UAV_CHECK_REQUIRE_COMMAND"
echo "  distance sweep: $DISTANCE_SWEEP"
if distance_sweep_enabled; then
  echo "  distance sweep values: $DISTANCE_SWEEP_VALUES"
  echo "  distance sweep interval: ${DISTANCE_SWEEP_INTERVAL_S}s"
  echo "  distance sweep z offset: ${DISTANCE_SWEEP_Z_OFFSET_M}m"
  echo "  distance sweep xy margin: ${DISTANCE_SWEEP_XY_ANCHOR_MARGIN_M}m"
  echo "  distance sweep adaptive detection: $DISTANCE_SWEEP_ADAPT_DETECTION"
  if distance_sweep_adapt_enabled; then
    echo "  distance sweep status topic: $DISTANCE_SWEEP_STATUS_TOPIC"
    echo "  distance sweep no-det window: ${DISTANCE_SWEEP_NO_DET_WINDOW_S}s"
    echo "  distance sweep z step: ${DISTANCE_SWEEP_Z_STEP_M}m"
    echo "  distance sweep min z offset: ${DISTANCE_SWEEP_MIN_Z_OFFSET_M}m"
    echo "  distance sweep restore height: $DISTANCE_SWEEP_RESTORE_HEIGHT"
  fi
fi
echo "  skip remaining on complete: $SKIP_REMAINING_ON_COMPLETE"
echo "  chain route starts: $CHAIN_ROUTE_STARTS"
echo "  fallback waypoints on route failure: $FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE"
echo "  exclude routes: $EXCLUDE_ROUTES_RAW"
echo "  routes: ${ROUTES[*]}"
if [ "$DRY_RUN" != true ]; then
  echo "  logs: $RUN_DIR"
fi

for repetition in "${REPETITIONS_LIST[@]}"; do
  rep_label="rep$(pad2 "$repetition")"
  PREVIOUS_ROUTE_LAST_WAYPOINT=""
  selected_route_index=0

  for route in "${ROUTES[@]}"; do
    selected_route_index=$((selected_route_index + 1))
    route_index="$(route_number_for_route "$route" "$selected_route_index")"
    route_run_label="R$(pad2 "$route_index")_${route}"
    label_prefix="${rep_label}_R$(pad2 "$route_index")_"
    JOB_START_WAYPOINTS=()
    JOB_NAV2_GOALS=()
    JOB_LABELS=()
    route_last_waypoint=""
    route_completed="false"

    case "$SWEEP_MODE" in
      routes|route)
        if [ "$CHAIN_ROUTE_STARTS" = true ] && [ -n "$PREVIOUS_ROUTE_LAST_WAYPOINT" ]; then
          first_waypoint="$PREVIOUS_ROUTE_LAST_WAYPOINT"
        else
          first_waypoint="$(first_waypoint_for_route "$route")" || {
            echo "Skipping route '$route': could not resolve first waypoint" >&2
            RESULTS+=("${rep_label}_${route_run_label} skipped no_first_waypoint")
            continue
          }
        fi
        route_last_waypoint="$(last_waypoint_for_route "$route" 2>/dev/null || true)"
        JOB_START_WAYPOINTS+=("$first_waypoint")
        JOB_NAV2_GOALS+=("$route")
        JOB_LABELS+=("${label_prefix}${route}")
        ;;
      waypoints|waypoint)
        if ! append_waypoint_jobs "$route" "$label_prefix"; then
          RESULTS+=("${rep_label}_${route_run_label} skipped no_route_file")
          continue
        fi
        ;;
      *)
        echo "Invalid sweep mode: $SWEEP_MODE" >&2
        echo "Use sweep:=waypoints or sweep:=routes" >&2
        exit 2
        ;;
    esac

    job_index=0
    while [ "$job_index" -lt "${#JOB_START_WAYPOINTS[@]}" ]; do
      first_waypoint="${JOB_START_WAYPOINTS[$job_index]}"
      nav2_goals_arg="${JOB_NAV2_GOALS[$job_index]}"
      job_label="${JOB_LABELS[$job_index]}"
      record_leaf="${job_label#${rep_label}_}"
      session="${SESSION_PREFIX}-$(sanitize_name "$job_label")"
      CURRENT_SESSION="$session"
      route_log="$RUN_DIR/$(sanitize_name "$job_label").follow.log"
      tmp_log="${route_log}.startup"
      mapfile -t route_lidar_args < <(route_lidar_preset_args "$route" "$LIDAR" "${EXTRA_TMUX_ARGS[@]}")
      run_context="$(result_context "${route_lidar_args[@]}")"
      CURRENT_JOB_LABEL="$job_label"
      CURRENT_START_WAYPOINT="$first_waypoint"
      CURRENT_RUN_CONTEXT="$run_context"
      record_out=""
      omnet_result_dir=""
      uav_check_log=""
      distance_sweep_log=""
      distance_sweep_values_for_job=""
      if distance_sweep_enabled; then
        distance_sweep_values_for_job="$(distance_sweep_values_for_repetition "$repetition")"
      fi
      if [ "$RECORD" = true ]; then
        record_out="${RECORD_ROOT}/${rep_label}/${record_leaf}"
        omnet_result_dir="$(workspace_path "$record_out")/omnet"
      fi

      echo
      echo "=== ${rep_label} ${route_run_label} (start waypoint: $first_waypoint, nav2_goals: $nav2_goals_arg, session: $session) ==="

      if [ "$RECORD" = true ] && record_path_has_data "$record_out"; then
        if [ "$OVERWRITE_EXISTING_RECORD" = true ]; then
          if [ "$DRY_RUN" = true ]; then
            echo "[dry-run] would archive existing record output before rerun: $(workspace_path "$record_out")"
          else
            archive_existing_record_path "$record_out"
          fi
        elif [ "$SKIP_EXISTING_RECORD" = true ]; then
          echo "Skipping '$job_label': record output already exists at $(workspace_path "$record_out")"
          RESULTS+=("$job_label skipped_existing start=$first_waypoint")
          CAMPAIGN_RESULTS+=("$repetition,$route_index,$route,$job_label,$first_waypoint,skipped_existing,true,$record_out,$omnet_result_dir,$run_context")
          job_index=$((job_index + 1))
          continue
        else
          echo "Record output already exists for '$job_label': $(workspace_path "$record_out")" >&2
          echo "Use skip_existing:=true to leave it untouched, or overwrite_existing:=true to archive it and rerun." >&2
          RESULTS+=("$job_label existing_output start=$first_waypoint")
          CAMPAIGN_RESULTS+=("$repetition,$route_index,$route,$job_label,$first_waypoint,existing_output,false,$record_out,$omnet_result_dir,$run_context")
          job_index=$((job_index + 1))
          continue
        fi
      fi

      if [ "$STOP_BEFORE_EACH" = true ]; then
        stop_session "$session"
      fi

      run_cmd=(
        ./run.sh tmux_1to1 "$WORLD"
        "mode:=$MODE"
        "session:=$session"
        "waypoint:=$first_waypoint"
        "nav2_goals:=$nav2_goals_arg"
        "lidar:=$LIDAR"
        "tmux_attach:=$TMUX_ATTACH"
        "${EXTRA_TMUX_ARGS[@]}"
      )
      if [ "$RECORD" = true ]; then
        run_cmd+=(
          "record:=true"
          "record_profile:=$RECORD_PROFILE"
          "record_tag:=${CAMPAIGN_TAG}_${job_label}"
          "record_out:=$record_out"
          "omnet_result_dir:=$omnet_result_dir"
        )
      fi
      if [ "${#route_lidar_args[@]}" -gt 0 ]; then
        echo "Route '$route' lidar preset: ${route_lidar_args[*]}"
        run_cmd+=("${route_lidar_args[@]}")
      fi
      if [ "$CHECK_UAV_FOLLOW" = true ] && [ "$HAVE_UGV_START_DELAY_ARG" != "true" ] && [ "$UAV_CHECK_HOLD_UGV_S" != "0" ] && [ "$UAV_CHECK_HOLD_UGV_S" != "0.0" ]; then
        run_cmd+=("ugv_start_delay_s:=$UAV_CHECK_HOLD_UGV_S")
      fi
      if [ "$DRY_RUN" = true ]; then
        run_cmd+=("dry_run:=true")
        echo "[dry-run] start:"
        print_cmd "${run_cmd[@]}"
        if distance_sweep_enabled; then
          echo "[dry-run] distance sweep: node=$DISTANCE_SWEEP_NODE values=$distance_sweep_values_for_job interval_s=$DISTANCE_SWEEP_INTERVAL_S z_offset_m=$DISTANCE_SWEEP_Z_OFFSET_M xy_margin_m=$DISTANCE_SWEEP_XY_ANCHOR_MARGIN_M"
          if distance_sweep_adapt_enabled; then
            echo "[dry-run] adaptive sweep: status_topic=$DISTANCE_SWEEP_STATUS_TOPIC no_det_window_s=$DISTANCE_SWEEP_NO_DET_WINDOW_S z_step_m=$DISTANCE_SWEEP_Z_STEP_M min_z_offset_m=$DISTANCE_SWEEP_MIN_Z_OFFSET_M restore_height=$DISTANCE_SWEEP_RESTORE_HEIGHT"
          fi
        fi
        RESULTS+=("$job_label dry_run start=$first_waypoint")
        CAMPAIGN_RESULTS+=("$repetition,$route_index,$route,$job_label,$first_waypoint,dry_run,false,$record_out,$omnet_result_dir,$run_context")
        if [ "$SWEEP_MODE" = "routes" ] || [ "$SWEEP_MODE" = "route" ]; then
          route_completed="true"
        fi
        job_index=$((job_index + 1))
        continue
      fi

      if ! "${run_cmd[@]}"; then
        echo "Route '$job_label' failed to launch." >&2
        RESULTS+=("$job_label launch_failed start=$first_waypoint")
        CAMPAIGN_RESULTS+=("$repetition,$route_index,$route,$job_label,$first_waypoint,launch_failed,false,$record_out,$omnet_result_dir,$run_context")
        sync_job_logs "$record_out" "$route_log" "$tmp_log"
        stop_session "$session"
        CURRENT_SESSION=""
        CURRENT_JOB_LABEL=""
        CURRENT_START_WAYPOINT=""
        CURRENT_RUN_CONTEXT=""
        job_index=$((job_index + 1))
        continue
      fi

      if [ "$CHECK_UAV_FOLLOW" = true ]; then
        uav_check_log="$RUN_DIR/$(sanitize_name "$job_label").uav_check.log"
        if ! check_uav_follow_ready "$session" "$job_label" "$uav_check_log"; then
          capture_follow_pane "$session" "$route_log"
          status="uav_check_failed"
          echo "Route '$job_label' result: $status"
          RESULTS+=("$job_label $status start=$first_waypoint")
          CAMPAIGN_RESULTS+=("$repetition,$route_index,$route,$job_label,$first_waypoint,$status,false,$record_out,$omnet_result_dir,$run_context")
          sync_job_logs "$record_out" "$route_log" "$tmp_log" "$uav_check_log"
          if [ "$STOP_AFTER_EACH" = true ]; then
            stop_session "$session"
          fi
          CURRENT_SESSION=""
          CURRENT_JOB_LABEL=""
          CURRENT_START_WAYPOINT=""
          CURRENT_RUN_CONTEXT=""
          if [ "$BETWEEN_S" != "0" ] && [ "$BETWEEN_S" != "0.0" ]; then
            sleep "$BETWEEN_S"
          fi
          job_index=$((job_index + 1))
          continue
        fi
      fi

      if distance_sweep_enabled; then
        distance_sweep_log="$RUN_DIR/$(sanitize_name "$job_label").distance_sweep.log"
        start_distance_sweep "$job_label" "$distance_sweep_log" "$distance_sweep_values_for_job"
      fi

      if wait_for_route_driver_start "$session" "$job_label" "$tmp_log"; then
        if wait_for_route_done "$session" "$job_label" "$route_log"; then
          status="$(classify_route_log "$route_log")"
        else
          status="timeout"
        fi
      else
        capture_follow_pane "$session" "$route_log"
        status="start_timeout"
      fi
      stop_distance_sweep

      echo "Route '$job_label' result: $status"
      RESULTS+=("$job_label $status start=$first_waypoint")
      network_valid="false"
      case "$status" in
        completed|finished|timeout)
          network_valid="true"
          ;;
      esac
      CAMPAIGN_RESULTS+=("$repetition,$route_index,$route,$job_label,$first_waypoint,$status,$network_valid,$record_out,$omnet_result_dir,$run_context")
      sync_job_logs "$record_out" "$route_log" "$tmp_log" "$uav_check_log" "$distance_sweep_log"
      if [ "$status" = "completed" ]; then
        route_completed="true"
      fi

      if [ "$STOP_AFTER_EACH" = true ]; then
        stop_session "$session"
      fi
      CURRENT_SESSION=""
      CURRENT_JOB_LABEL=""
      CURRENT_START_WAYPOINT=""
      CURRENT_RUN_CONTEXT=""

      if [ "$SWEEP_MODE" = "routes" ] || [ "$SWEEP_MODE" = "route" ]; then
        if [ "$FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE" = true ] && [ "$CHAIN_ROUTE_STARTS" != true ] && [ "$job_index" -eq 0 ] && [ "$status" = "failed" ]; then
          echo "Route '$route' failed as a full route; falling back to sliced waypoint starts after the route start."
          append_waypoint_jobs "$route" "${label_prefix}fallback_" 1 || {
            echo "Route '$route' has no later fallback waypoints; moving to next route."
          }
        fi
      fi

      if [ "$BETWEEN_S" != "0" ] && [ "$BETWEEN_S" != "0.0" ]; then
        sleep "$BETWEEN_S"
      fi

      if [ "$SWEEP_MODE" != "routes" ] && [ "$SWEEP_MODE" != "route" ] && [ "$SKIP_REMAINING_ON_COMPLETE" = true ] && [ "$status" = "completed" ]; then
        echo "Route '$route' completed from '$first_waypoint'; skipping remaining start waypoints for this route."
        break
      fi
      if [ "$SWEEP_MODE" = "routes" ] || [ "$SWEEP_MODE" = "route" ]; then
        if [ "$FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE" = true ] && [[ "$job_label" == *"_fallback_"* ]] && [ "$SKIP_REMAINING_ON_COMPLETE" = true ] && [ "$status" = "completed" ]; then
          echo "Route '$route' completed from fallback '$first_waypoint'; skipping remaining fallback waypoints for this route."
          break
        fi
      fi
      job_index=$((job_index + 1))
    done

    if [ "$SWEEP_MODE" = "routes" ] || [ "$SWEEP_MODE" = "route" ]; then
      if [ "$route_completed" = "true" ] && [ -n "$route_last_waypoint" ]; then
        PREVIOUS_ROUTE_LAST_WAYPOINT="$route_last_waypoint"
      else
        PREVIOUS_ROUTE_LAST_WAYPOINT=""
      fi
    fi
  done
done

CURRENT_SESSION=""

MEASURED_METRICS_SUMMARY="$(generate_measured_metrics_summary || true)"
if [ -n "$MEASURED_METRICS_SUMMARY" ]; then
  echo
  printf '%s\n' "$MEASURED_METRICS_SUMMARY"
fi

echo
echo "Route sweep summary:"
for row in "${RESULTS[@]}"; do
  printf '  %s\n' "$row"
done

if [ "$DRY_RUN" != true ]; then
  write_summary_file "$RUN_DIR/summary.txt" "finished"
  echo "Saved summary: $RUN_DIR/summary.txt"
  write_record_root_summary "finished" "summary"
fi

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROOT="bags/ruben_c1_c4_selected_rosbags_2026-05-25"
CONDITIONS="C3,C4"
NETWORKS="lora"
LORA_SF="7"
LORA_BW="125kHz"
REPLAY_SCALE="2"
MAX_RUNS="0"
SAMPLE_STRIDE="1"
DRY_RUN=false
SKIP_EXISTING=true
MANIFEST=""

usage() {
  cat <<'EOF'
Usage:
  ./run.sh omnet_bag_replay_batch [root:=PATH] [conditions:=C3,C4] [networks:=lora,lora-duplex]

Options:
  root:=PATH              Root containing C3/ and C4/ run folders.
  conditions:=C3,C4       Conditions to process.
  networks:=lora          OMNeT networks to run. Use lora,lora-duplex for simplex+duplex.
  lora_sf:=7              LoRa spreading factor.
  lora_bw:=125kHz         LoRa bandwidth.
  replay_scale:=2         Offline replay scale passed to omnet_bag_replay.
  max_runs:=0             Max run folders per condition, 0 means all.
  sample_stride:=1        Process every Nth valid run folder. Use 2 to skip every other bag.
  skip_existing:=true     Skip runs whose network_metrics.csv already reaches completion.
  manifest:=PATH          CSV manifest path. Default: <root>/offline_omnet_manifest.csv
  dry_run:=true           Print planned commands only.
EOF
}

resolve_path() {
  local path="$1"
  case "$path" in
    /*) printf '%s\n' "$path" ;;
    *) printf '%s\n' "$WS_ROOT/$path" ;;
  esac
}

split_csv() {
  local value="$1"
  tr ',' '\n' <<<"$value" | sed '/^$/d'
}

has_topic() {
  local metadata="$1"
  local topic="$2"
  grep -Fq "name: $topic" "$metadata"
}

choose_ugv_topic() {
  local metadata="$1"
  local topic=""
  for topic in \
    "/a201_0000/ground_truth/odom" \
    "/a201_0000/platform/odom/filtered" \
    "/a201_0000/amcl_pose_odom" \
    "/a201_0000/platform/odom"; do
    if has_topic "$metadata" "$topic"; then
      printf '%s\n' "$topic"
      return 0
    fi
  done
  return 1
}

network_out_dir() {
  local run_dir="$1"
  local network="$2"
  local safe_bw="${LORA_BW//[^A-Za-z0-9]/}"
  printf '%s/offline_omnet/%s_sf%s_bw%s\n' "$run_dir" "$network" "$LORA_SF" "$safe_bw"
}

last_csv_sim_time() {
  local csv="$1"
  if [ ! -s "$csv" ]; then
    printf 'nan\n'
    return 0
  fi
  tail -n 1 "$csv" | awk -F, '{print $2}'
}

last_omnet_sim_time() {
  local log="$1"
  if [ ! -s "$log" ]; then
    printf 'nan\n'
    return 0
  fi
  grep '100% completed' "$log" | tail -n 1 | sed -n 's/.* t=\([^ ]*\) .*/\1/p'
}

sim_time_close_enough() {
  local csv_time="$1"
  local omnet_time="$2"
  python3 - "$csv_time" "$omnet_time" <<'PY'
import math
import sys

try:
    csv_t = float(sys.argv[1])
    omnet_t = float(sys.argv[2])
except ValueError:
    raise SystemExit(1)

if not (math.isfinite(csv_t) and math.isfinite(omnet_t)):
    raise SystemExit(1)
raise SystemExit(0 if csv_t >= omnet_t - 1.0 else 1)
PY
}

is_complete() {
  local out_dir="$1"
  local csv_time=""
  local omnet_time=""
  [ -s "$out_dir/network_metrics.csv" ] || return 1
  grep -q "100% completed" "$out_dir/omnet.log" 2>/dev/null || return 1
  csv_time="$(last_csv_sim_time "$out_dir/network_metrics.csv")"
  omnet_time="$(last_omnet_sim_time "$out_dir/omnet.log")"
  sim_time_close_enough "$csv_time" "$omnet_time"
}

shell_join() {
  local out="" part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    root:=*)
      ROOT="${arg#root:=}"
      ;;
    conditions:=*)
      CONDITIONS="${arg#conditions:=}"
      ;;
    networks:=*|network:=*)
      NETWORKS="${arg#*:=}"
      ;;
    lora_sf:=*|sf:=*)
      LORA_SF="${arg#*:=}"
      ;;
    lora_bw:=*|bw:=*)
      LORA_BW="${arg#*:=}"
      ;;
    replay_scale:=*|scale:=*)
      REPLAY_SCALE="${arg#*:=}"
      ;;
    max_runs:=*)
      MAX_RUNS="${arg#max_runs:=}"
      ;;
    sample_stride:=*|stride:=*|every_nth:=*)
      SAMPLE_STRIDE="${arg#*:=}"
      ;;
    skip_existing:=*)
      SKIP_EXISTING="${arg#skip_existing:=}"
      ;;
    manifest:=*)
      MANIFEST="${arg#manifest:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

ROOT="$(resolve_path "$ROOT")"
if [ -z "$MANIFEST" ]; then
  MANIFEST="$ROOT/offline_omnet_manifest.csv"
else
  MANIFEST="$(resolve_path "$MANIFEST")"
fi

case "$DRY_RUN" in true|false) ;; *) echo "Invalid dry_run: $DRY_RUN" >&2; exit 2 ;; esac
case "$SKIP_EXISTING" in true|false) ;; *) echo "Invalid skip_existing: $SKIP_EXISTING" >&2; exit 2 ;; esac
case "$SAMPLE_STRIDE" in
  ''|*[!0-9]*)
    echo "Invalid sample_stride: $SAMPLE_STRIDE" >&2
    exit 2
    ;;
esac
if [ "$SAMPLE_STRIDE" -lt 1 ]; then
  echo "Invalid sample_stride: $SAMPLE_STRIDE" >&2
  exit 2
fi

if [ "$DRY_RUN" != true ]; then
  mkdir -p "$(dirname "$MANIFEST")"
  if [ ! -f "$MANIFEST" ]; then
    printf 'condition,run_name,network,status,ugv_topic,uav_topic,metrics_csv,omnet_log,last_sim_time_s\n' >"$MANIFEST"
  fi
fi

total=0
for condition in $(split_csv "$CONDITIONS"); do
  condition_dir="$ROOT/$condition"
  if [ ! -d "$condition_dir" ]; then
    echo "[omnet_bag_replay_batch] missing condition dir: $condition_dir" >&2
    continue
  fi
  count=0
  seen=0
  while IFS= read -r bag_dir; do
    run_dir="$(dirname "$bag_dir")"
    metadata="$bag_dir/metadata.yaml"
    [ -f "$metadata" ] || continue
    if ! has_topic "$metadata" "/dji0/pose"; then
      echo "[omnet_bag_replay_batch] skip $(basename "$run_dir"): missing /dji0/pose" >&2
      continue
    fi
    if ! ugv_topic="$(choose_ugv_topic "$metadata")"; then
      echo "[omnet_bag_replay_batch] skip $(basename "$run_dir"): no supported UGV odom topic" >&2
      continue
    fi
    seen=$((seen + 1))
    if [ "$SAMPLE_STRIDE" -gt 1 ] && [ $(( (seen - 1) % SAMPLE_STRIDE )) -ne 0 ]; then
      echo "[omnet_bag_replay_batch] stride skip $condition $(basename "$run_dir") index=$seen stride=$SAMPLE_STRIDE"
      continue
    fi
    count=$((count + 1))
    if [ "$MAX_RUNS" != "0" ] && [ "$count" -gt "$MAX_RUNS" ]; then
      break
    fi
    for network in $(split_csv "$NETWORKS"); do
      out_dir="$(network_out_dir "$run_dir" "$network")"
      metrics_csv="$out_dir/network_metrics.csv"
      omnet_log="$out_dir/omnet.log"
      if [ "$SKIP_EXISTING" = true ] && is_complete "$out_dir"; then
        last_sim="$(last_csv_sim_time "$metrics_csv")"
        echo "[omnet_bag_replay_batch] skip complete $condition $(basename "$run_dir") $network sim=$last_sim"
        if [ "$DRY_RUN" != true ]; then
          printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
            "$condition" "$(basename "$run_dir")" "$network" "skipped_complete" \
            "$ugv_topic" "/dji0/pose" "$metrics_csv" "$omnet_log" "$last_sim" >>"$MANIFEST"
        fi
        continue
      fi
      cmd=(
        "$WS_ROOT/run.sh" omnet_bag_replay
        "run_dir:=$run_dir"
        "network:=$network"
        "lora_sf:=$LORA_SF"
        "lora_bw:=$LORA_BW"
        "ugv_topic:=$ugv_topic"
        "uav_topic:=/dji0/pose"
        "replay_scale:=$REPLAY_SCALE"
      )
      total=$((total + 1))
      if [ "$DRY_RUN" = true ]; then
        echo "$(shell_join "${cmd[@]}")"
        continue
      fi
      status="ok"
      if ! "${cmd[@]}"; then
        status="failed"
      fi
      last_sim="$(last_csv_sim_time "$metrics_csv")"
      if [ "$status" = "ok" ] && ! is_complete "$out_dir"; then
        status="incomplete"
      fi
      printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "$condition" "$(basename "$run_dir")" "$network" "$status" \
        "$ugv_topic" "/dji0/pose" "$metrics_csv" "$omnet_log" "$last_sim" >>"$MANIFEST"
    done
  done < <(find "$condition_dir" -mindepth 2 -maxdepth 2 -type d -name bag | sort)
done

if [ "$DRY_RUN" != true ]; then
  echo "manifest=$MANIFEST"
fi
echo "planned_or_processed=$total"

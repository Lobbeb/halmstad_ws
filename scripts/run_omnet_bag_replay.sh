#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

RUN_DIR=""
BAG_DIR=""
OUT_DIR=""
NETWORK="lora"
UI="cmdenv"
LORA_SF="7"
LORA_BW="125kHz"
SPEED="1.0"
REPLAY_SCALE=""
DURATION_S="auto"
START_S="0"
WALL_TIMEOUT_S="auto"
POSE_PORT="5555"
METRICS_PORT="5556"
UGV_TOPIC="/a201_0000/ground_truth/odom"
UAV_TOPIC="/dji0/pose"
DRY_RUN=false
EXTRA_OMNET_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  ./run.sh omnet_bag_replay run_dir:=bags/results/.../rep01/R01_rotundan [network:=lora|lora-duplex] [lora_sf:=7] [lora_bw:=125kHz]

Options:
  run_dir:=PATH       Run directory containing bag/ (recommended)
  bag:=PATH           Bag directory, if not using run_dir
  out:=PATH           Output directory (default: <run_dir>/offline_omnet/<network>_sf<SF>_bw<BW>)
  duration_s:=auto|N  OMNeT sim-time-limit. auto uses bag metadata duration
  replay_scale:=N     Faster/slower replay. Sets pose speed and OMNeT real-time scaling together.
  speed:=N            Pose replay speed only; use replay_scale for normal offline analysis
  wall_timeout_s:=auto|none|N
                      Kill OMNeT if wall time exceeds this. auto = duration + 120s
  dry_run:=true       Print commands only
EOF
}

shell_join() {
  local out="" part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

resolve_path() {
  local path="$1"
  case "$path" in
    /*) printf '%s\n' "$path" ;;
    *) printf '%s\n' "$WS_ROOT/$path" ;;
  esac
}

metadata_duration_s() {
  local metadata="$1"
  python3 - "$metadata" <<'PY'
import re
import sys
from pathlib import Path

text = Path(sys.argv[1]).read_text(encoding="utf-8", errors="ignore")
match = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
if not match:
    raise SystemExit("metadata duration not found")
duration = int(match.group(1)) / 1e9
print(f"{duration + 1.0:.3f}")
PY
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    run_dir:=*)
      RUN_DIR="$(resolve_path "${arg#run_dir:=}")"
      ;;
    bag:=*)
      BAG_DIR="$(resolve_path "${arg#bag:=}")"
      ;;
    out:=*|output:=*)
      OUT_DIR="$(resolve_path "${arg#*:=}")"
      ;;
    network:=*)
      NETWORK="${arg#network:=}"
      ;;
    ui:=*)
      UI="${arg#ui:=}"
      ;;
    lora_sf:=*|sf:=*)
      LORA_SF="${arg#*:=}"
      ;;
    lora_bw:=*|bw:=*)
      LORA_BW="${arg#*:=}"
      ;;
    speed:=*)
      SPEED="${arg#speed:=}"
      ;;
    replay_scale:=*|scale:=*)
      REPLAY_SCALE="${arg#*:=}"
      ;;
    duration_s:=*)
      DURATION_S="${arg#duration_s:=}"
      ;;
    start_s:=*)
      START_S="${arg#start_s:=}"
      ;;
    wall_timeout_s:=*|timeout_s:=*)
      WALL_TIMEOUT_S="${arg#*:=}"
      ;;
    pose_port:=*)
      POSE_PORT="${arg#pose_port:=}"
      ;;
    metrics_port:=*)
      METRICS_PORT="${arg#metrics_port:=}"
      ;;
    ugv_topic:=*)
      UGV_TOPIC="${arg#ugv_topic:=}"
      ;;
    uav_topic:=*)
      UAV_TOPIC="${arg#uav_topic:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    --*)
      EXTRA_OMNET_ARGS+=("$arg")
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -n "$REPLAY_SCALE" ]; then
  SPEED="$REPLAY_SCALE"
  EXTRA_OMNET_ARGS+=("--realtimescheduler-scaling=$REPLAY_SCALE")
fi

if [ -z "$BAG_DIR" ]; then
  if [ -z "$RUN_DIR" ]; then
    echo "Missing run_dir:=PATH or bag:=PATH" >&2
    usage >&2
    exit 2
  fi
  BAG_DIR="$RUN_DIR/bag"
fi

if [ -z "$RUN_DIR" ]; then
  RUN_DIR="$(cd "$BAG_DIR/.." && pwd)"
fi

if [ -z "$OUT_DIR" ]; then
  safe_bw="${LORA_BW//[^A-Za-z0-9]/}"
  OUT_DIR="$RUN_DIR/offline_omnet/${NETWORK}_sf${LORA_SF}_bw${safe_bw}"
fi

if [ ! -d "$BAG_DIR" ]; then
  echo "Missing bag directory: $BAG_DIR" >&2
  exit 1
fi

METADATA="$BAG_DIR/metadata.yaml"
if [ "$DURATION_S" = "auto" ]; then
  if [ ! -f "$METADATA" ]; then
    echo "duration_s:=auto needs bag metadata: $METADATA" >&2
    exit 1
  fi
  DURATION_S="$(metadata_duration_s "$METADATA")"
fi

if [ "$WALL_TIMEOUT_S" = "auto" ]; then
  WALL_TIMEOUT_S="$(python3 - "$DURATION_S" "${REPLAY_SCALE:-1.0}" <<'PY'
import math
import sys
duration = float(sys.argv[1])
scale = float(sys.argv[2])
scale = scale if scale > 0.0 else 1.0
print(int(math.ceil(duration / scale + 120.0)))
PY
)"
fi

POSE_LOG="$OUT_DIR/pose_replay.log"
METRICS_LOG="$OUT_DIR/metrics_capture.log"
OMNET_LOG="$OUT_DIR/omnet.log"
METRICS_CSV="$OUT_DIR/network_metrics.csv"
OMNET_RESULTS="$OUT_DIR/omnet"

POSE_CMD=(
  python3 "$SCRIPT_DIR/replay_bag_pose_tcp_bridge.py"
  --bag "$BAG_DIR"
  --port "$POSE_PORT"
  --ugv-topic "$UGV_TOPIC"
  --uav-topic "$UAV_TOPIC"
  --speed "$SPEED"
  --start-s "$START_S"
  --duration-s "$DURATION_S"
)
METRICS_CMD=(
  python3 "$SCRIPT_DIR/capture_omnet_metrics_csv.py"
  --port "$METRICS_PORT"
  --out "$METRICS_CSV"
)
OMNET_CMD=(
  "$WS_ROOT/run.sh" omnet
  "network:=$NETWORK"
  "ui:=$UI"
  "lora_sf:=$LORA_SF"
  "lora_bw:=$LORA_BW"
  "result_dir:=$OMNET_RESULTS"
  "--sim-time-limit=${DURATION_S}s"
  "--*.gazeboScheduler.port=$POSE_PORT"
  "--*.metricsServer.port=$METRICS_PORT"
  "${EXTRA_OMNET_ARGS[@]}"
)

if [ "$DRY_RUN" = true ]; then
  echo "bag=$BAG_DIR"
  echo "out=$OUT_DIR"
  echo "duration_s=$DURATION_S"
  echo "replay_scale=${REPLAY_SCALE:-1.0}"
  echo "wall_timeout_s=$WALL_TIMEOUT_S"
  echo "pose: $(shell_join "${POSE_CMD[@]}")"
  echo "metrics: $(shell_join "${METRICS_CMD[@]}")"
  echo "omnet: $(shell_join "${OMNET_CMD[@]}")"
  exit 0
fi

mkdir -p "$OUT_DIR" "$OMNET_RESULTS"

set +u
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$WS_ROOT/install/setup.bash"
fi
set -u

pose_pid=""
metrics_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [ -n "$metrics_pid" ] && kill -0 "$metrics_pid" 2>/dev/null; then
    kill -INT "$metrics_pid" 2>/dev/null || true
    sleep 0.2
    kill "$metrics_pid" 2>/dev/null || true
    wait "$metrics_pid" 2>/dev/null || true
  fi
  if [ -n "$pose_pid" ] && kill -0 "$pose_pid" 2>/dev/null; then
    kill -INT "$pose_pid" 2>/dev/null || true
    sleep 0.2
    kill "$pose_pid" 2>/dev/null || true
    wait "$pose_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"${POSE_CMD[@]}" >"$POSE_LOG" 2>&1 &
pose_pid="$!"
sleep 1
if ! kill -0 "$pose_pid" 2>/dev/null; then
  echo "Pose replay server failed. See $POSE_LOG" >&2
  exit 1
fi

"${METRICS_CMD[@]}" >"$METRICS_LOG" 2>&1 &
metrics_pid="$!"

echo "Offline OMNeT bag replay:"
echo "  bag: $BAG_DIR"
echo "  out: $OUT_DIR"
echo "  duration_s: $DURATION_S"
echo "  replay_scale: ${REPLAY_SCALE:-1.0}"
echo "  wall_timeout_s: $WALL_TIMEOUT_S"
echo "  metrics_csv: $METRICS_CSV"
echo "  omnet_results: $OMNET_RESULTS"

omnet_status=0
if [ "$WALL_TIMEOUT_S" = "none" ] || [ "$WALL_TIMEOUT_S" = "0" ]; then
  "${OMNET_CMD[@]}" >"$OMNET_LOG" 2>&1 || omnet_status=$?
else
  timeout --kill-after=10s "${WALL_TIMEOUT_S}s" "${OMNET_CMD[@]}" >"$OMNET_LOG" 2>&1 || omnet_status=$?
fi

if [ -n "$metrics_pid" ]; then
  for _ in {1..20}; do
    if ! kill -0 "$metrics_pid" 2>/dev/null; then
      wait "$metrics_pid" 2>/dev/null || true
      metrics_pid=""
      break
    fi
    sleep 0.1
  done
fi

cleanup

if [ "$omnet_status" -eq 124 ] || [ "$omnet_status" -eq 137 ]; then
  echo "OMNeT timed out after wall_timeout_s=$WALL_TIMEOUT_S. See $OMNET_LOG" >&2
  exit "$omnet_status"
fi
if [ "$omnet_status" -ne 0 ]; then
  echo "OMNeT exited with status $omnet_status. See $OMNET_LOG" >&2
  exit "$omnet_status"
fi

echo "Done."
echo "metrics_csv=$METRICS_CSV"
echo "omnet_log=$OMNET_LOG"

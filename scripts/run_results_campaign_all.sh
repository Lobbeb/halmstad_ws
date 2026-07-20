#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

RUNS=10
DURATION_S=360
WARMUP_S=45
WORLD="baylands"
GUI="false"
OUT_ROOT="bags/results"
WEIGHTS="models/obb/mymodels/baylands-leader-v0.pt"
SUPPORT_WEIGHTS=""
SUPPORT_BACKEND="ultralytics"
ROUTE_SCHEDULE="rotundan,road_to_west,road_to_spawn,spawn,parkinglot_west"
CONTINUE_ON_FAILURE=false
PLOTS=false

usage() {
  cat <<EOF
Usage: $0 --runs N --duration S --warmup S --world baylands --out bags/results [options]

Runs C1, C2, C3, and C4 sequentially using the Baylands Results harness.

Options:
  --gui true|false              Gazebo GUI, default false
  --weights PATH                Baylands .pt model for C2/C3, default models/obb/mymodels/baylands-leader-v0.pt
  --support-weights PATH        Baylands .pt model for C4, default same as --weights
  --support-backend NAME        C4 backend, default ultralytics
  --route-schedule LIST         Fixed Baylands route schedule, default rotundan,road_to_west,road_to_spawn,spawn,parkinglot_west
                                Repeats for runs > number of routes; x10 gives two passes.
  --continue-on-failure         Continue with later conditions after a condition failure
  --plots true|false            Try to generate simple plots after summary, default false
EOF
}

resolve_path() {
  local value="$1"
  if [[ "$value" = /* ]]; then
    printf '%s\n' "$value"
  else
    printf '%s/%s\n' "$WS_ROOT" "$value"
  fi
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --runs)
      RUNS="${2:-}"; shift 2 ;;
    --duration)
      DURATION_S="${2:-}"; shift 2 ;;
    --warmup)
      WARMUP_S="${2:-}"; shift 2 ;;
    --world)
      WORLD="${2:-}"; shift 2 ;;
    --gui)
      GUI="${2:-}"; shift 2 ;;
    --out)
      OUT_ROOT="${2:-}"; shift 2 ;;
    --weights)
      WEIGHTS="${2:-}"; shift 2 ;;
    --support-weights)
      SUPPORT_WEIGHTS="${2:-}"; shift 2 ;;
    --support-backend)
      SUPPORT_BACKEND="${2:-}"; shift 2 ;;
    --route-schedule)
      ROUTE_SCHEDULE="${2:-}"; shift 2 ;;
    --continue-on-failure)
      CONTINUE_ON_FAILURE=true; shift ;;
    --plots)
      PLOTS="${2:-}"; shift 2 ;;
    help|-h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2 ;;
  esac
done

if [ "$WORLD" != "baylands" ]; then
  echo "Results campaign is Baylands-only. Refusing world=$WORLD." >&2
  exit 2
fi

case "$GUI" in
  true|false) ;;
  *) echo "Invalid --gui value: $GUI" >&2; exit 2 ;;
esac
case "$SUPPORT_BACKEND" in
  ultralytics) ;;
  *)
    echo "results_campaign_all currently uses the Baylands .pt support model and requires --support-backend ultralytics." >&2
    exit 2 ;;
esac
case "$PLOTS" in
  true|false) ;;
  *) echo "Invalid --plots value: $PLOTS" >&2; exit 2 ;;
esac

WEIGHTS_ABS="$(resolve_path "$WEIGHTS")"
if [ ! -f "$WEIGHTS_ABS" ]; then
  echo "Baylands YOLO weights not found: $WEIGHTS_ABS" >&2
  exit 1
fi
if [ -z "$SUPPORT_WEIGHTS" ]; then
  SUPPORT_WEIGHTS="$WEIGHTS"
fi
SUPPORT_WEIGHTS_ABS="$(resolve_path "$SUPPORT_WEIGHTS")"
if [ ! -f "$SUPPORT_WEIGHTS_ABS" ]; then
  echo "Baylands support YOLO weights not found: $SUPPORT_WEIGHTS_ABS" >&2
  exit 1
fi

OUT_ROOT_ABS="$(resolve_path "$OUT_ROOT")"
mkdir -p "$OUT_ROOT_ABS"
LOG_FILE="$OUT_ROOT_ABS/campaign_all_$(date +%Y%m%dT%H%M%S).log"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "[results_campaign_all] log=$LOG_FILE"
echo "[results_campaign_all] world=$WORLD runs=$RUNS duration_s=$DURATION_S warmup_s=$WARMUP_S out=$OUT_ROOT_ABS"
echo "[results_campaign_all] weights=$WEIGHTS_ABS"
echo "[results_campaign_all] support_backend=$SUPPORT_BACKEND support_weights=$SUPPORT_WEIGHTS_ABS"
echo "[results_campaign_all] route_schedule=$ROUTE_SCHEDULE"

FAILED_CONDITIONS=()

summarize_all() {
  echo "[results_campaign_all] Summarising full campaign"
  (cd "$WS_ROOT" && python3 scripts/results_summarize_bag.py "$OUT_ROOT_ABS" --warmup "$WARMUP_S")
}

run_condition() {
  local condition="$1"
  shift
  local cmd=(
    ./run.sh results_campaign
    --condition "$condition"
    --runs "$RUNS"
    --duration "$DURATION_S"
    --warmup "$WARMUP_S"
    --world "$WORLD"
    --gui "$GUI"
    --out "$OUT_ROOT_ABS"
    --route-schedule "$ROUTE_SCHEDULE"
    "$@"
  )

  echo
  echo "[results_campaign_all] Starting $condition"
  if (cd "$WS_ROOT" && "${cmd[@]}"); then
    echo "[results_campaign_all] Completed $condition"
    return 0
  fi

  local status=$?
  echo "[results_campaign_all] FAILED $condition status=$status" >&2
  FAILED_CONDITIONS+=("$condition")
  if [ "$CONTINUE_ON_FAILURE" != true ]; then
    summarize_all || true
    exit "$status"
  fi
  return 0
}

run_condition C1
run_condition C2 --weights "$WEIGHTS_ABS"
run_condition C3 --weights "$WEIGHTS_ABS"
run_condition C4 --support-backend "$SUPPORT_BACKEND" --support-weights "$SUPPORT_WEIGHTS_ABS"

summarize_all

if [ "$PLOTS" = true ]; then
  echo "[results_campaign_all] Generating plots"
  if ! (cd "$WS_ROOT" && python3 scripts/results_make_plots.py "$OUT_ROOT_ABS/summary/runs.csv" --out "$OUT_ROOT_ABS/plots"); then
    echo "[results_campaign_all] Warning: plot generation failed; CSV summaries are still available." >&2
  fi
fi

if [ "${#FAILED_CONDITIONS[@]}" -gt 0 ]; then
  echo "[results_campaign_all] Completed with failures: ${FAILED_CONDITIONS[*]}" >&2
  exit 1
fi

echo "[results_campaign_all] Completed all conditions"

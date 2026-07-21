#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PHASE=""
TIMEOUT_S="12"
MAX_HAZARD_AGE_S="1.0"
STATE_DIR="/tmp/halmstad_ws/aerial_support_validation"
REQUIRE_PLAN_CHANGE=true

usage() {
  cat <<'EOF'
Usage: ./run.sh verify_aerial_support_chain phase:=baseline|hazard|expiry [timeout_s:=12] [max_hazard_age_s:=1.0] [state_dir:=/tmp/halmstad_ws/aerial_support_validation] [require_plan_change:=true|false]

This bounded observer does not start, stop, or modify a simulation. Run baseline before
the on-route synthetic hazard, hazard while it is active, and expiry after it publishes
an empty array. It stores timestamp-normalized topic snapshots in state_dir.
EOF
}

for arg in "$@"; do
  case "$arg" in
    help|--help|-h)
      usage
      exit 0
      ;;
    phase:=*)
      PHASE="${arg#phase:=}"
      ;;
    timeout_s:=*)
      TIMEOUT_S="${arg#timeout_s:=}"
      ;;
    max_hazard_age_s:=*)
      MAX_HAZARD_AGE_S="${arg#max_hazard_age_s:=}"
      ;;
    state_dir:=*)
      STATE_DIR="${arg#state_dir:=}"
      ;;
    require_plan_change:=*)
      REQUIRE_PLAN_CHANGE="${arg#require_plan_change:=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$PHASE" in
  baseline|hazard|expiry)
    ;;
  *)
    echo "phase:=baseline, phase:=hazard, or phase:=expiry is required." >&2
    exit 2
    ;;
esac

case "$REQUIRE_PLAN_CHANGE" in
  true|false)
    ;;
  *)
    echo "require_plan_change must be true or false." >&2
    exit 2
    ;;
esac

if ! [[ "$TIMEOUT_S" =~ ^[0-9]+([.][0-9]+)?$ ]] || ! [[ "$MAX_HAZARD_AGE_S" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "timeout_s and max_hazard_age_s must be non-negative numbers." >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

mkdir -p "$STATE_DIR"

sample_topic() {
  local topic="$1"
  timeout "${TIMEOUT_S}s" ros2 topic echo --once "$topic"
}

require_nonempty_hazard() {
  local topic="$1"
  local output
  if ! output="$(sample_topic "$topic")"; then
    echo "No bounded sample received from $topic" >&2
    return 1
  fi
  if ! grep -q '^hazards:' <<<"$output" || grep -q '^hazards: \[\]' <<<"$output"; then
    echo "Expected a non-empty AerialHazardArray on $topic" >&2
    printf '%s\n' "$output" >&2
    return 1
  fi
  printf '%s\n' "$output"
}

find_global_costmap_node() {
  local node
  while IFS= read -r node; do
    [ -n "$node" ] || continue
    if ros2 param get "$node" aerial_support_layer.plugin 2>/dev/null | grep -Fq 'lrs_halmstad_nav_plugins::AerialSupportLayer'; then
      printf '%s\n' "$node"
      return 0
    fi
  done < <(ros2 node list | grep '/global_costmap' || true)
  return 1
}

capture_hash() {
  local topic="$1"
  local output_file="$2"
  if ! sample_topic "$topic" > "$output_file"; then
    echo "No bounded sample received from $topic" >&2
    return 1
  fi
  sed -E '/^[[:space:]]*(stamp|sec|nanosec):/d' "$output_file" | sha256sum | awk '{print $1}'
}

hazard_stamp_age_s() {
  local hazard_output="$1"
  local clock_output="$2"
  local hazard_sec hazard_nsec clock_sec clock_nsec
  hazard_sec="$(awk '$1 == "last_seen:" {in_last_seen=1; next} in_last_seen && $1 == "sec:" {print $2; exit}' <<<"$hazard_output")"
  hazard_nsec="$(awk '$1 == "last_seen:" {in_last_seen=1; next} in_last_seen && $1 == "nanosec:" {print $2; exit}' <<<"$hazard_output")"
  clock_sec="$(awk '$1 == "clock:" {in_clock=1; next} in_clock && $1 == "sec:" {print $2; exit}' <<<"$clock_output")"
  clock_nsec="$(awk '$1 == "clock:" {in_clock=1; next} in_clock && $1 == "nanosec:" {print $2; exit}' <<<"$clock_output")"
  if [ -z "$hazard_sec" ] || [ -z "$hazard_nsec" ] || [ -z "$clock_sec" ] || [ -z "$clock_nsec" ]; then
    echo "Could not extract last_seen or /clock timestamp." >&2
    return 1
  fi
  python3 - "$hazard_sec" "$hazard_nsec" "$clock_sec" "$clock_nsec" <<'PY'
import sys
hazard_s, hazard_ns, clock_s, clock_ns = map(int, sys.argv[1:])
print((clock_s + clock_ns / 1e9) - (hazard_s + hazard_ns / 1e9))
PY
}

global_costmap_node="$(find_global_costmap_node || true)"
if [ -z "$global_costmap_node" ]; then
  echo "AerialSupportLayer was not found on a running global_costmap node." >&2
  exit 1
fi
if ! ros2 param get "$global_costmap_node" aerial_support_layer.enabled | grep -Eq 'true$'; then
  echo "AerialSupportLayer is loaded but aerial_support_layer.enabled is not true." >&2
  exit 1
fi

printf 'phase=%s\nglobal_costmap_node=%s\n' "$PHASE" "$global_costmap_node" | tee "$STATE_DIR/${PHASE}_report.txt"

if [ "$PHASE" = "baseline" ]; then
  baseline_costmap_hash="$(capture_hash /a201_0000/global_costmap/costmap_raw "$STATE_DIR/baseline_costmap.yaml")"
  baseline_plan_hash="$(capture_hash /a201_0000/plan "$STATE_DIR/baseline_plan.yaml")"
  printf 'baseline_costmap_hash=%s\nbaseline_plan_hash=%s\n' "$baseline_costmap_hash" "$baseline_plan_hash" | tee -a "$STATE_DIR/${PHASE}_report.txt"
  printf '%s\n' "$baseline_costmap_hash" > "$STATE_DIR/baseline_costmap.sha256"
  printf '%s\n' "$baseline_plan_hash" > "$STATE_DIR/baseline_plan.sha256"
  exit 0
fi

if [ "$PHASE" = "expiry" ]; then
  deadline=$((SECONDS + ${TIMEOUT_S%.*}))
  while true; do
    if ugv_output="$(sample_topic /coord/ugv/aerial_hazards 2>/dev/null)" && grep -q '^hazards: \[\]' <<<"$ugv_output"; then
      break
    fi
    if [ "$SECONDS" -ge "$deadline" ]; then
      echo "UGV hazard array did not become empty before timeout." >&2
      exit 1
    fi
  done
  expiry_costmap_hash="$(capture_hash /a201_0000/global_costmap/costmap_raw "$STATE_DIR/expiry_costmap.yaml")"
  if [ ! -f "$STATE_DIR/hazard_costmap.sha256" ]; then
    echo "Run phase:=hazard before phase:=expiry." >&2
    exit 1
  fi
  if [ "$expiry_costmap_hash" = "$(cat "$STATE_DIR/hazard_costmap.sha256")" ]; then
    echo "Global costmap did not change after the hazard array became empty." >&2
    exit 1
  fi
  printf 'ugv_hazards=empty\nexpiry_costmap_hash=%s\nexpiry_clearing=observed\n' "$expiry_costmap_hash" | tee -a "$STATE_DIR/${PHASE}_report.txt"
  printf '%s\n' "$expiry_costmap_hash" > "$STATE_DIR/expiry_costmap.sha256"
  exit 0
fi

source_output="$(require_nonempty_hazard /coord/support/dji1/aerial_hazards)"
require_nonempty_hazard /coord/dji0/aerial_hazards >/dev/null
ugv_output="$(require_nonempty_hazard /coord/ugv/aerial_hazards)"
clock_output="$(sample_topic /clock)"
age_s="$(hazard_stamp_age_s "$ugv_output" "$clock_output")"
if ! python3 - "$age_s" "$MAX_HAZARD_AGE_S" <<'PY'
import sys
age_s, max_age_s = map(float, sys.argv[1:])
raise SystemExit(0 if 0.0 <= age_s <= max_age_s else 1)
PY
then
  echo "UGV hazard age $age_s is outside [0, $MAX_HAZARD_AGE_S] seconds." >&2
  exit 1
fi

costmap_hash="$(capture_hash /a201_0000/global_costmap/costmap_raw "$STATE_DIR/${PHASE}_costmap.yaml")"
plan_hash="$(capture_hash /a201_0000/plan "$STATE_DIR/${PHASE}_plan.yaml")"
printf 'typed_flow=source,dji0,ugv\nhazard_age_s=%s\ncostmap_hash=%s\nplan_hash=%s\n' "$age_s" "$costmap_hash" "$plan_hash" | tee -a "$STATE_DIR/${PHASE}_report.txt"
printf '%s\n' "$costmap_hash" > "$STATE_DIR/${PHASE}_costmap.sha256"
printf '%s\n' "$plan_hash" > "$STATE_DIR/${PHASE}_plan.sha256"

if [ "$PHASE" = "hazard" ]; then
  if [ ! -f "$STATE_DIR/baseline_costmap.sha256" ] || [ ! -f "$STATE_DIR/baseline_plan.sha256" ]; then
    echo "Run phase:=baseline before phase:=hazard." >&2
    exit 1
  fi
  if [ "$costmap_hash" = "$(cat "$STATE_DIR/baseline_costmap.sha256")" ]; then
    echo "Global costmap did not change from the baseline snapshot." >&2
    exit 1
  fi
  if [ "$REQUIRE_PLAN_CHANGE" = true ] && [ "$plan_hash" = "$(cat "$STATE_DIR/baseline_plan.sha256")" ]; then
    echo "Plan did not change from the baseline snapshot." >&2
    exit 1
  fi
  printf 'costmap_change=observed\nplan_change=%s\n' "$([ "$plan_hash" != "$(cat "$STATE_DIR/baseline_plan.sha256")" ] && echo observed || echo not_observed)" | tee -a "$STATE_DIR/${PHASE}_report.txt"
fi

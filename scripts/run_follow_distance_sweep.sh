#!/usr/bin/env bash
set -euo pipefail

NODE="/follow_uav"
STATUS_TOPIC="/coord/leader_estimate_status"
VALUES="10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100"
Z_VALUES="7"
INTERVAL_S="10"
XY_MARGIN_M="2"
NO_DET_STOP_S="10"
WAIT_TIMEOUT_S="60"
LOG=""
ONCE=true

usage() {
  cat <<'EOF'
Usage:
  ./run.sh follow_distance_sweep [values:=10,15,...] [z:=7|z_values:=7,10,15,20] [interval_s:=10] [no_det_stop_s:=10]

Examples:
  ./run.sh follow_distance_sweep
  ./run.sh follow_distance_sweep z_values:=7,10,15,20 no_det_stop_s:=10
  ./run.sh follow_distance_sweep values:=10,20,30 z:=7 interval_s:=15
EOF
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

calc_xy_anchor_max() {
  python3 - "$1" "$2" "$3" <<'PY'
import math
import sys
d = max(0.0, float(sys.argv[1]))
z = max(0.0, min(float(sys.argv[2]), d))
margin = max(0.0, float(sys.argv[3]))
print(f"{math.sqrt(max(0.0, d*d - z*z)) + margin:.6f}")
PY
}

status_state() {
  local raw token
  raw="$(timeout 2s ros2 topic echo --no-daemon --once "$STATUS_TOPIC" std_msgs/msg/String --field data 2>/dev/null || true)"
  raw="${raw//$'\n'/ }"
  for token in $raw; do
    case "$token" in
      state=*)
        printf '%s\n' "${token#state=}" | tr '[:lower:]' '[:upper:]'
        return 0
        ;;
      OK|NO_DET|STALE)
        printf '%s\n' "$token"
        return 0
        ;;
    esac
  done
  printf 'UNKNOWN\n'
}

float_le() {
  awk -v a="$1" -v b="$2" 'BEGIN { exit !(a <= b) }'
}

elapsed_reached() {
  awk -v start="$1" -v window="$2" 'BEGIN { exit !((systime() - start) >= window) }'
}

apply_params() {
  local d="$1"
  local z="$2"
  local xy="$3"

  # Order matters: d_target must be raised before follow_z_offset_m can be raised.
  ros2 param set --no-daemon "$NODE" d_target "$d"
  ros2 param set --no-daemon "$NODE" follow_z_offset_m "$z"
  ros2 param set --no-daemon "$NODE" xy_anchor_max "$xy"
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    node:=*)
      NODE="${arg#node:=}"
      ;;
    values:=*|d_values:=*|d_targets:=*)
      VALUES="${arg#*:=}"
      ;;
    z:=*|height:=*)
      Z_VALUES="${arg#*:=}"
      ;;
    z_values:=*|heights:=*)
      Z_VALUES="${arg#*:=}"
      ;;
    interval_s:=*|interval:=*)
      INTERVAL_S="${arg#*:=}"
      ;;
    xy_margin_m:=*)
      XY_MARGIN_M="${arg#xy_margin_m:=}"
      ;;
    status_topic:=*)
      STATUS_TOPIC="${arg#status_topic:=}"
      ;;
    no_det_stop_s:=*|stop_no_det_s:=*)
      NO_DET_STOP_S="${arg#*:=}"
      ;;
    wait_timeout_s:=*)
      WAIT_TIMEOUT_S="${arg#wait_timeout_s:=}"
      ;;
    log:=*)
      LOG="${arg#log:=}"
      ;;
    once:=*)
      ONCE="${arg#once:=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -n "$LOG" ]; then
  mkdir -p "$(dirname "$LOG")"
  exec > >(tee -a "$LOG") 2>&1
fi

deadline="$(awk -v now="$(date +%s)" -v timeout="$WAIT_TIMEOUT_S" 'BEGIN { printf "%.0f\n", now + timeout }')"
while [ "$(date +%s)" -le "$deadline" ]; do
  if ros2 param get --no-daemon "$NODE" d_target >/dev/null 2>&1; then
    break
  fi
  echo "[follow_distance_sweep] waiting for $NODE ..."
  sleep 1
done

if ! ros2 param get --no-daemon "$NODE" d_target >/dev/null 2>&1; then
  echo "[follow_distance_sweep] $NODE is not ready after ${WAIT_TIMEOUT_S}s" >&2
  exit 1
fi

echo "[follow_distance_sweep] node=$NODE values=$VALUES z_values=$Z_VALUES interval_s=$INTERVAL_S no_det_stop_s=$NO_DET_STOP_S"

bad_since=""
while true; do
  IFS=',' read -ra d_parts <<< "$VALUES"
  IFS=',' read -ra z_parts <<< "$Z_VALUES"
  for raw_d in "${d_parts[@]}"; do
    d="$(trim "$raw_d")"
    [ -n "$d" ] || continue
    for raw_z in "${z_parts[@]}"; do
      z="$(trim "$raw_z")"
      [ -n "$z" ] || continue
      if ! float_le "$z" "$d"; then
        echo "[follow_distance_sweep] skip d_target=$d z=$z reason=z_gt_d"
        continue
      fi

      xy="$(calc_xy_anchor_max "$d" "$z" "$XY_MARGIN_M")"
      echo "[follow_distance_sweep] apply d_target=$d follow_z_offset_m=$z xy_anchor_max=$xy"
      apply_params "$d" "$z" "$xy"

      step_start_s="$(date +%s)"
      while ! elapsed_reached "$step_start_s" "$INTERVAL_S"; do
        state="$(status_state)"
        case "$state" in
          NO_DET)
            if [ -z "$bad_since" ]; then
              bad_since="$(date +%s)"
              echo "[follow_distance_sweep] no_det_start d_target=$d z=$z"
            elif elapsed_reached "$bad_since" "$NO_DET_STOP_S"; then
              echo "[follow_distance_sweep] stop reason=NO_DET_${NO_DET_STOP_S}s d_target=$d z=$z"
              exit 0
            fi
            ;;
          OK)
            if [ -n "$bad_since" ]; then
              echo "[follow_distance_sweep] detection_recovered d_target=$d z=$z"
            fi
            bad_since=""
            ;;
          *)
            :
            ;;
        esac
        sleep 1
      done
    done
  done

  case "$ONCE" in
    true|True|TRUE|1|yes|Yes|YES|on|On|ON)
      echo "[follow_distance_sweep] complete"
      exit 0
      ;;
  esac
done

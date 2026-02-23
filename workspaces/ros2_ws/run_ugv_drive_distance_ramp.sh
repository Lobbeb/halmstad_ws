#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/ruben/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

DRIVE_MODE="drive"
START_DISTANCE="10.0"
END_DISTANCE="40.0"
STEP_DISTANCE="5.0"
HOLD_SEC="20.0"
RAMP_START_DELAY="0.0"
TAKEOFF_DURATION=""
TAKEOFF_SPEED_MPS=""
UGV_MPS=""
UAV_MAX_Z_MPS=""
START_HEIGHT=""
END_HEIGHT=""
STEP_HEIGHT=""

INTERRUPTED=0
DRIVE_PID=""

usage() {
  cat <<'EOF'
Usage: ./run_ugv_drive_distance_ramp.sh [drive|drive_lidar] [options]

Options:
  start_distance=<m>   Initial follow distance (default: 10.0)
  end_distance=<m>     Final follow distance (default: 40.0)
  step_distance=<m>    Distance step per update (default: 5.0)
  start_height=<m>     Initial hover height above UGV (optional; enables height ramp)
  end_height=<m>       Final hover height above UGV (required if start_height used)
  step_height=<m>      Height step per update (default: 1.0 when height ramp enabled)
  hold_sec=<s>         Hold time between updates (default: 20.0)
  ramp_start_delay=<s> Extra delay after follower ready before ramping (default: 0.0)
  takeoff_duration=<s> Passed to bebop follower
  takeoff_speed_mps=<m/s> Passed to bebop follower (ignored if takeoff_duration is also set)
  ugv_mps=<m/s>        Passed to drive mode (and follower speed headroom via run_ugv_demo.sh)
  uav_max_z_mps=<m/s>  Passed to bebop follower max vertical speed

Examples:
  ./run_ugv_drive_distance_ramp.sh drive_lidar start_distance=10 end_distance=50 step_distance=5 start_height=10 end_height=25 step_height=2 hold_sec=15
  ./run_ugv_drive_distance_ramp.sh drive ugv_mps=1.2 takeoff_duration=8 start_distance=10 end_distance=30 start_height=10 end_height=18
EOF
}

on_interrupt() {
  INTERRUPTED=1
  if [[ -n "$DRIVE_PID" ]]; then
    kill -INT "$DRIVE_PID" >/dev/null 2>&1 || true
  fi
}

trap on_interrupt INT TERM

normalize_double() {
  local v="$1"
  if [[ "$v" =~ ^-?[0-9]+$ ]]; then
    echo "${v}.0"
  else
    echo "$v"
  fi
}

validate_double() {
  local label="$1"
  local v="$2"
  if [[ ! "$v" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    echo "Invalid ${label}: ${v}" >&2
    exit 1
  fi
}

fmt2() {
  awk -v x="$1" 'BEGIN { printf "%.2f", x }'
}

for arg in "$@"; do
  case "$arg" in
    -h|--help)
      usage
      exit 0
      ;;
    drive)
      DRIVE_MODE="drive"
      ;;
    drive_lidar)
      DRIVE_MODE="drive_lidar"
      ;;
    start_distance=*)
      START_DISTANCE="${arg#start_distance=}"
      ;;
    end_distance=*)
      END_DISTANCE="${arg#end_distance=}"
      ;;
    step_distance=*)
      STEP_DISTANCE="${arg#step_distance=}"
      ;;
    start_height=*)
      START_HEIGHT="${arg#start_height=}"
      ;;
    end_height=*)
      END_HEIGHT="${arg#end_height=}"
      ;;
    step_height=*)
      STEP_HEIGHT="${arg#step_height=}"
      ;;
    hold_sec=*)
      HOLD_SEC="${arg#hold_sec=}"
      ;;
    ramp_start_delay=*)
      RAMP_START_DELAY="${arg#ramp_start_delay=}"
      ;;
    takeoff_duration=*)
      TAKEOFF_DURATION="${arg#takeoff_duration=}"
      ;;
    takeoff_speed_mps=*)
      TAKEOFF_SPEED_MPS="${arg#takeoff_speed_mps=}"
      ;;
    ugv_mps=*)
      UGV_MPS="${arg#ugv_mps=}"
      ;;
    uav_max_z_mps=*)
      UAV_MAX_Z_MPS="${arg#uav_max_z_mps=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 1
      ;;
  esac
done

for name in START_DISTANCE END_DISTANCE STEP_DISTANCE HOLD_SEC RAMP_START_DELAY; do
  val="${!name}"
  val="$(normalize_double "$val")"
  validate_double "$name" "$val"
  printf -v "$name" '%s' "$val"
done

HEIGHT_RAMP_ENABLED=false
if [[ -n "$START_HEIGHT" || -n "$END_HEIGHT" || -n "$STEP_HEIGHT" ]]; then
  HEIGHT_RAMP_ENABLED=true
fi

if [[ "$HEIGHT_RAMP_ENABLED" == true ]]; then
  if [[ -z "$START_HEIGHT" || -z "$END_HEIGHT" ]]; then
    echo "Height ramp requires both start_height=<m> and end_height=<m>." >&2
    exit 1
  fi
  if [[ -z "$STEP_HEIGHT" ]]; then
    STEP_HEIGHT="1.0"
  fi
  for name in START_HEIGHT END_HEIGHT STEP_HEIGHT; do
    val="${!name}"
    val="$(normalize_double "$val")"
    validate_double "$name" "$val"
    printf -v "$name" '%s' "$val"
  done
fi

if [[ -n "$TAKEOFF_DURATION" ]]; then
  TAKEOFF_DURATION="$(normalize_double "$TAKEOFF_DURATION")"
  validate_double "takeoff_duration" "$TAKEOFF_DURATION"
fi

if [[ -n "$TAKEOFF_SPEED_MPS" ]]; then
  TAKEOFF_SPEED_MPS="$(normalize_double "$TAKEOFF_SPEED_MPS")"
  validate_double "takeoff_speed_mps" "$TAKEOFF_SPEED_MPS"
fi

if [[ -n "$UGV_MPS" ]]; then
  UGV_MPS="$(normalize_double "$UGV_MPS")"
  validate_double "ugv_mps" "$UGV_MPS"
fi

if [[ -n "$UAV_MAX_Z_MPS" ]]; then
  UAV_MAX_Z_MPS="$(normalize_double "$UAV_MAX_Z_MPS")"
  validate_double "uav_max_z_mps" "$UAV_MAX_Z_MPS"
fi

if [[ -f "$ROS_SETUP" ]]; then
  set +u
  source "$ROS_SETUP"
  set -u
else
  echo "ROS setup not found at $ROS_SETUP" >&2
  exit 1
fi

cd "$WORKSPACE"
if [[ -f "install/setup.bash" ]]; then
  set +u
  source "install/setup.bash"
  set -u
else
  echo "Workspace not built. Run colcon build first." >&2
  exit 1
fi

wait_for_node() {
  local node_name="$1"
  local timeout_s="${2:-60}"
  local start_ts=$SECONDS
  while (( SECONDS - start_ts < timeout_s )); do
    if ros2 node list 2>/dev/null | rg -q "^${node_name}\$"; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

wait_for_follow_ready() {
  local timeout_s="${1:-120}"
  local start_ts=$SECONDS
  while (( SECONDS - start_ts < timeout_s )); do
    if timeout 2s ros2 topic echo --once /bebop1/follow_ready 2>/dev/null | rg -q "data: true"; then
      return 0
    fi
    sleep 0.3
  done
  return 1
}

run_args=("$DRIVE_MODE" "follow" "follow_distance=$START_DISTANCE")
if [[ -n "$TAKEOFF_DURATION" ]]; then
  run_args+=("takeoff_duration=$TAKEOFF_DURATION")
fi
if [[ -n "$TAKEOFF_SPEED_MPS" ]]; then
  run_args+=("takeoff_speed_mps=$TAKEOFF_SPEED_MPS")
fi
if [[ -n "$UGV_MPS" ]]; then
  run_args+=("ugv_mps=$UGV_MPS")
fi
if [[ -n "$UAV_MAX_Z_MPS" ]]; then
  run_args+=("uav_max_z_mps=$UAV_MAX_Z_MPS")
fi

echo "Launching: ./run_ugv_demo.sh ${run_args[*]}"
"$WORKSPACE/run_ugv_demo.sh" "${run_args[@]}" &
DRIVE_PID=$!

if ! wait_for_node "/bebop_follow_ugv" 90; then
  echo "Timed out waiting for /bebop_follow_ugv node. Stopping drive." >&2
  kill -INT "$DRIVE_PID" >/dev/null 2>&1 || true
  wait "$DRIVE_PID" || true
  exit 1
fi

if wait_for_follow_ready 120; then
  echo "Follower ready. Starting distance ramp."
else
  echo "Timed out waiting for /bebop1/follow_ready=true. Starting ramp anyway." >&2
fi

if [[ "$HEIGHT_RAMP_ENABLED" == true ]]; then
  if ros2 param set /bebop_follow_ugv hover_height_above_ugv_m "$(fmt2 "$START_HEIGHT")" >/dev/null 2>&1; then
    echo "Set initial hover_height_above_ugv_m -> $(fmt2 "$START_HEIGHT") m"
  else
    echo "Warning: failed to set initial hover_height_above_ugv_m=$(fmt2 "$START_HEIGHT")" >&2
  fi
fi

if awk -v d="$RAMP_START_DELAY" 'BEGIN { exit !(d > 0.0) }'; then
  echo "Waiting extra ramp_start_delay=${RAMP_START_DELAY}s"
  sleep "$RAMP_START_DELAY"
fi

direction=1
if awk -v a="$END_DISTANCE" -v b="$START_DISTANCE" 'BEGIN { exit !(a < b) }'; then
  direction=-1
fi

step_mag="$(awk -v s="$STEP_DISTANCE" 'BEGIN { if (s < 0) s = -s; printf "%.6f", s }')"
if awk -v s="$step_mag" 'BEGIN { exit !(s < 0.0) }'; then
  echo "step_distance must be >= 0" >&2
  kill -INT "$DRIVE_PID" >/dev/null 2>&1 || true
  wait "$DRIVE_PID" || true
  exit 1
fi
if awk -v s="$step_mag" 'BEGIN { exit !(s == 0.0) }'; then
  if ! awk -v a="$START_DISTANCE" -v b="$END_DISTANCE" 'BEGIN { exit !(a == b) }'; then
    echo "step_distance=0 is only valid when start_distance == end_distance (constant distance)." >&2
    kill -INT "$DRIVE_PID" >/dev/null 2>&1 || true
    wait "$DRIVE_PID" || true
    exit 1
  fi
fi

current="$START_DISTANCE"
last_applied=""
height_current="${START_HEIGHT:-}"
height_last_applied=""
height_direction=1
height_step_mag=""

if [[ "$HEIGHT_RAMP_ENABLED" == true ]]; then
  if awk -v a="$END_HEIGHT" -v b="$START_HEIGHT" 'BEGIN { exit !(a < b) }'; then
    height_direction=-1
  fi
  height_step_mag="$(awk -v s="$STEP_HEIGHT" 'BEGIN { if (s < 0) s = -s; printf "%.6f", s }')"
  if awk -v s="$height_step_mag" 'BEGIN { exit !(s <= 0.0) }'; then
    echo "step_height must be > 0" >&2
    kill -INT "$DRIVE_PID" >/dev/null 2>&1 || true
    wait "$DRIVE_PID" || true
    exit 1
  fi
fi

while kill -0 "$DRIVE_PID" >/dev/null 2>&1; do
  current_fmt="$(fmt2 "$current")"
  if [[ "$current_fmt" != "$last_applied" ]]; then
    if ros2 param set /bebop_follow_ugv follow_distance_m "$current_fmt" >/dev/null 2>&1; then
      echo "Updated follow_distance_m -> ${current_fmt} m"
      last_applied="$current_fmt"
    else
      echo "Warning: failed to set follow_distance_m=${current_fmt} (node may still be initializing)" >&2
    fi
  fi

  if [[ "$HEIGHT_RAMP_ENABLED" == true ]]; then
    height_fmt="$(fmt2 "$height_current")"
    if [[ "$height_fmt" != "$height_last_applied" ]]; then
      if ros2 param set /bebop_follow_ugv hover_height_above_ugv_m "$height_fmt" >/dev/null 2>&1; then
        echo "Updated hover_height_above_ugv_m -> ${height_fmt} m"
        height_last_applied="$height_fmt"
      else
        echo "Warning: failed to set hover_height_above_ugv_m=${height_fmt}" >&2
      fi
    fi
  fi

  dist_done=false
  if awk -v c="$current" -v e="$END_DISTANCE" -v d="$direction" 'BEGIN { exit !((d > 0 && c >= e) || (d < 0 && c <= e)) }'; then
    dist_done=true
  fi

  height_done=true
  if [[ "$HEIGHT_RAMP_ENABLED" == true ]]; then
    height_done=false
    if awk -v c="$height_current" -v e="$END_HEIGHT" -v d="$height_direction" 'BEGIN { exit !((d > 0 && c >= e) || (d < 0 && c <= e)) }'; then
      height_done=true
    fi
  fi

  # Stop stepping once all enabled ramps reached their target.
  if [[ "$dist_done" == true && "$height_done" == true ]]; then
    if [[ "$HEIGHT_RAMP_ENABLED" == true ]]; then
      echo "Reached target follow distance ${END_DISTANCE} m and hover height ${END_HEIGHT} m. Holding until drive exits."
    else
      echo "Reached target follow distance ${END_DISTANCE} m. Holding until drive exits."
    fi
    break
  fi

  sleep "$HOLD_SEC"
  if [[ "$dist_done" != true ]]; then
    current="$(awk -v c="$current" -v step="$step_mag" -v d="$direction" 'BEGIN { printf "%.6f", c + d * step }')"
  fi
  if [[ "$HEIGHT_RAMP_ENABLED" == true && "$height_done" != true ]]; then
    height_current="$(awk -v c="$height_current" -v step="$height_step_mag" -v d="$height_direction" 'BEGIN { printf "%.6f", c + d * step }')"
  fi
done

# Keep wrapper attached to the drive process after ramp completes.
wait "$DRIVE_PID"

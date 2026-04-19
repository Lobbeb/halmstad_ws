#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SWEEP_STATE_DIR="$STATE_DIR/baylands_step_sweep"
PY_TWIST_PUBLISHER="$SCRIPT_DIR/publish_twist_stamped.py"

source "$SCRIPT_DIR/slam_state_common.sh"

WORLD="baylands"
GUI="false"
LIDAR_MODE="3d"
START_X="19.83"
START_Y="114.99"
START_Z="0.8"
START_YAW="0.0"
POINTS_FILE=""
AXIS="x"
DIRECTION="positive"
STEP_M="10.0"
STEPS="1"
STEPS_SET="false"
UNTIL_X=""
UNTIL_Y=""
PREFIX="baylands_step"
CIRCLE_LINEAR="0.0"
CIRCLE_ANGULAR="0.12"
CIRCLE_TURNS="1.0"
CMD_RATE_HZ="20.0"
SLAM_READY_TIMEOUT_S="90"
SLAM_WARMUP_S="4"
POSE_SETTLE_S="2"
DRY_RUN="false"
SHUTDOWN_GAZEBO="true"

GAZEBO_PID=""
SLAM_PID=""
STARTED_GAZEBO="false"
NAMESPACE=""
MANIFEST_PATH=""
declare -a POINT_NAMES=()
declare -a POINT_XS=()
declare -a POINT_YS=()

usage() {
  cat <<'EOF'
Usage: ./run.sh baylands_strip_sweep [args...]

Starts Baylands at a chosen pose, launches SLAM, drives the UGV in one small
spin, saves a submap, teleports to the next stop, and repeats.

Key args:
  x:=19.83 y:=114.99 z:=0.8 yaw:=0.0
  points_file:=/path/to/points.csv
  axis:=x|y
  direction:=positive|negative
  step_m:=10.0
  steps:=1
  until_x:=<meters>
  until_y:=<meters>
  prefix:=baylands_step
  gui:=false
  lidar:=3d
  circle_linear:=0.0
  circle_angular:=0.12
  circle_turns:=1.0
  slam_warmup_s:=4
  pose_settle_s:=2
  shutdown_gazebo:=true
  dry_run:=false

Examples:
  ./run.sh baylands_strip_sweep steps:=5
  ./run.sh baylands_strip_sweep x:=0 y:=0 steps:=8 step_m:=10
  ./run.sh baylands_strip_sweep points_file:=/home/ruben/halmstad_ws/maps/baylands_step_points.csv
  ./run.sh baylands_strip_sweep axis:=y direction:=negative until_y:=-120
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

float_abs() {
  awk -v v="$1" 'BEGIN { if (v < 0) v = -v; printf "%.9f\n", v }'
}

calc_circle_duration() {
  awk -v turns="$1" -v angular="$2" '
    BEGIN {
      if (angular == 0) {
        exit 1
      }
      if (angular < 0) {
        angular = -angular
      }
      printf "%.3f\n", (2.0 * 3.141592653589793 * turns) / angular
    }
  '
}

calc_target_pose() {
  local index="$1"
  local sign="1"
  local target_x="$START_X"
  local target_y="$START_Y"

  if [ "$DIRECTION" = "negative" ]; then
    sign="-1"
  fi

  if [ "$AXIS" = "x" ]; then
    target_x="$(awk -v start="$START_X" -v step="$STEP_M" -v i="$index" -v sign="$sign" 'BEGIN { printf "%.9f\n", start + (sign * step * i) }')"
  else
    target_y="$(awk -v start="$START_Y" -v step="$STEP_M" -v i="$index" -v sign="$sign" 'BEGIN { printf "%.9f\n", start + (sign * step * i) }')"
  fi

  printf '%s %s\n' "$target_x" "$target_y"
}

resolve_points_file() {
  local candidate="$1"

  if [ -f "$candidate" ]; then
    printf '%s\n' "$candidate"
    return 0
  fi

  if [ -f "$WS_ROOT/maps/$candidate" ]; then
    printf '%s\n' "$WS_ROOT/maps/$candidate"
    return 0
  fi

  return 1
}

load_points_file() {
  local path="$1"
  local line_no=0
  local raw_name=""
  local raw_x=""
  local raw_y=""
  local raw_rest=""

  POINT_NAMES=()
  POINT_XS=()
  POINT_YS=()

  while IFS=, read -r raw_name raw_x raw_y raw_rest; do
    line_no=$((line_no + 1))

    raw_name="${raw_name//$'\r'/}"
    raw_x="${raw_x//$'\r'/}"
    raw_y="${raw_y//$'\r'/}"

    if [ -z "$raw_name$raw_x$raw_y" ]; then
      continue
    fi
    if [[ "$raw_name" =~ ^[[:space:]]*# ]]; then
      continue
    fi

    if [ "$line_no" -eq 1 ]; then
      case "${raw_name,,}" in
        name|state_name|label|x)
          continue
          ;;
      esac
    fi

    if [ -n "$raw_y" ]; then
      POINT_NAMES+=("$raw_name")
      POINT_XS+=("$raw_x")
      POINT_YS+=("$raw_y")
    else
      POINT_NAMES+=("")
      POINT_XS+=("$raw_name")
      POINT_YS+=("$raw_x")
    fi
  done < "$path"

  if [ "${#POINT_XS[@]}" -eq 0 ]; then
    echo "[run_baylands_strip_sweep] No points were loaded from $path" >&2
    exit 1
  fi
}

pose_within_bounds() {
  local x="$1"
  local y="$2"
  local ok="true"

  if [ -n "$UNTIL_X" ]; then
    ok="$(awk -v current="$x" -v limit="$UNTIL_X" -v direction="$DIRECTION" '
      BEGIN {
        if (direction == "negative") {
          print (current + 1e-9 >= limit) ? "true" : "false"
        } else {
          print (current - 1e-9 <= limit) ? "true" : "false"
        }
      }
    ')"
    if [ "$ok" != "true" ]; then
      return 1
    fi
  fi

  if [ -n "$UNTIL_Y" ]; then
    ok="$(awk -v current="$y" -v limit="$UNTIL_Y" -v direction="$DIRECTION" '
      BEGIN {
        if (direction == "negative") {
          print (current + 1e-9 >= limit) ? "true" : "false"
        } else {
          print (current - 1e-9 <= limit) ? "true" : "false"
        }
      }
    ')"
    if [ "$ok" != "true" ]; then
      return 1
    fi
  fi

  return 0
}

ensure_no_existing_sim() {
  if pgrep -f "/home/ruben/halmstad_ws/scripts/run_gazebo_sim.sh" >/dev/null 2>&1 || \
     pgrep -f "^gz sim" >/dev/null 2>&1; then
    echo "[run_baylands_strip_sweep] A Gazebo sim already appears to be running. Stop it first." >&2
    exit 1
  fi
}

terminate_process_group() {
  local pid="${1:-}"
  local label="${2:-process}"
  local sig=""

  if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  for sig in INT TERM KILL; do
    kill "-$sig" -- "-$pid" 2>/dev/null || kill "-$sig" "$pid" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return 0
      fi
      sleep 1
    done
  done

  echo "[run_baylands_strip_sweep] Warning: $label (pid $pid) may still be running." >&2
}

cleanup() {
  terminate_process_group "$SLAM_PID" "SLAM"
  SLAM_PID=""

  if [ "$SHUTDOWN_GAZEBO" = "true" ] && [ "$STARTED_GAZEBO" = "true" ]; then
    terminate_process_group "$GAZEBO_PID" "Gazebo"
  fi
}

start_gazebo() {
  local x="$1"
  local y="$2"
  local log_path="$SWEEP_STATE_DIR/gazebo.log"

  mkdir -p "$SWEEP_STATE_DIR"
  : > "$log_path"

  echo "[run_baylands_strip_sweep] Starting Gazebo at x=$x y=$y z=$START_Z yaw=$START_YAW"
  setsid "$SCRIPT_DIR/run_gazebo_sim.sh" \
    "$WORLD" \
    "$GUI" \
    "x:=$x" \
    "y:=$y" \
    "z:=$START_Z" \
    "yaw:=$START_YAW" \
    >"$log_path" 2>&1 &
  GAZEBO_PID=$!
  STARTED_GAZEBO="true"
}

wait_for_gazebo() {
  local timeout_s="${1:-180}"
  local waited=0

  while [ "$waited" -lt "$timeout_s" ]; do
    if [ -n "$GAZEBO_PID" ] && ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
      echo "[run_baylands_strip_sweep] Gazebo exited early. See $SWEEP_STATE_DIR/gazebo.log" >&2
      return 1
    fi
    if slam_state_capture_gazebo_pose_env "$WS_ROOT" "$WORLD" 3 >/dev/null 2>&1; then
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done

  echo "[run_baylands_strip_sweep] Timed out waiting for Gazebo pose topics." >&2
  return 1
}

start_slam() {
  local index="$1"
  local log_path="$SWEEP_STATE_DIR/slam_${index}.log"
  local serialize_service=""

  mkdir -p "$SWEEP_STATE_DIR"
  : > "$log_path"

  echo "[run_baylands_strip_sweep] Starting SLAM for stop $index"
  setsid "$SCRIPT_DIR/run_slam.sh" \
    "lidar:=$LIDAR_MODE" \
    >"$log_path" 2>&1 &
  SLAM_PID=$!

  serialize_service="$(slam_service_base "$NAMESPACE")/serialize_map"
  if ! slam_wait_for_service_or_process "$serialize_service" "$SLAM_PID" "$SLAM_READY_TIMEOUT_S"; then
    echo "[run_baylands_strip_sweep] SLAM did not become ready. See $log_path" >&2
    return 1
  fi

  echo "[run_baylands_strip_sweep] SLAM ready, warming up for ${SLAM_WARMUP_S}s"
  sleep "$SLAM_WARMUP_S"
}

drive_circle() {
  local duration_s="$1"
  local topic="/${NAMESPACE}/cmd_vel"

  echo "[run_baylands_strip_sweep] Driving a circle for ${duration_s}s on ${topic}"
  python3 "$PY_TWIST_PUBLISHER" \
    --topic "$topic" \
    --linear-x "$CIRCLE_LINEAR" \
    --angular-z "$CIRCLE_ANGULAR" \
    --duration-s "$duration_s" \
    --rate-hz "$CMD_RATE_HZ" \
    --use-sim-time
}

save_submap() {
  local state_name="$1"
  local index="$2"
  local x="$3"
  local y="$4"

  echo "[run_baylands_strip_sweep] Saving submap ${state_name}"
  "$SCRIPT_DIR/run_slam_save_state.sh" "$state_name" "lidar:=$LIDAR_MODE"
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$index" \
    "$state_name" \
    "$x" \
    "$y" \
    "$START_Z" \
    "$START_YAW" \
    "$AXIS" \
    "$DIRECTION" \
    "$STEP_M" \
    "$LIDAR_MODE" \
    >> "$MANIFEST_PATH"
}

stop_slam() {
  if [ -n "$SLAM_PID" ]; then
    echo "[run_baylands_strip_sweep] Stopping SLAM"
    terminate_process_group "$SLAM_PID" "SLAM"
    SLAM_PID=""
    sleep 2
  fi
}

reposition_ugv() {
  local x="$1"
  local y="$2"

  echo "[run_baylands_strip_sweep] Moving UGV to next stop x=$x y=$y yaw=$START_YAW"
  "$SCRIPT_DIR/run_realign_yaw.sh" "$WORLD" "x:=$x" "y:=$y" "yaw:=$START_YAW"
  echo "[run_baylands_strip_sweep] Waiting ${POSE_SETTLE_S}s for the robot to settle"
  sleep "$POSE_SETTLE_S"
}

if [ "$#" -gt 0 ] && [[ "$1" != *=* ]] && [[ "$1" != *":="* ]]; then
  case "$1" in
    help|-h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[run_baylands_strip_sweep] Unknown positional argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    help|-h|--help)
      usage
      exit 0
      ;;
    x:=*)
      START_X="${1#x:=}"
      ;;
    y:=*)
      START_Y="${1#y:=}"
      ;;
    z:=*)
      START_Z="${1#z:=}"
      ;;
    yaw:=*)
      START_YAW="${1#yaw:=}"
      ;;
    points_file:=*)
      POINTS_FILE="${1#points_file:=}"
      ;;
    gui:=*)
      GUI="$(coerce_bool "${1#gui:=}")"
      ;;
    lidar:=*)
      LIDAR_MODE="${1#lidar:=}"
      ;;
    axis:=*)
      AXIS="${1#axis:=}"
      ;;
    direction:=*)
      DIRECTION="${1#direction:=}"
      ;;
    step_m:=*)
      STEP_M="${1#step_m:=}"
      ;;
    steps:=*)
      STEPS="${1#steps:=}"
      STEPS_SET="true"
      ;;
    until_x:=*)
      UNTIL_X="${1#until_x:=}"
      ;;
    until_y:=*)
      UNTIL_Y="${1#until_y:=}"
      ;;
    prefix:=*)
      PREFIX="${1#prefix:=}"
      ;;
    circle_linear:=*)
      CIRCLE_LINEAR="${1#circle_linear:=}"
      ;;
    circle_angular:=*)
      CIRCLE_ANGULAR="${1#circle_angular:=}"
      ;;
    circle_turns:=*)
      CIRCLE_TURNS="${1#circle_turns:=}"
      ;;
    cmd_rate_hz:=*)
      CMD_RATE_HZ="${1#cmd_rate_hz:=}"
      ;;
    slam_ready_timeout_s:=*)
      SLAM_READY_TIMEOUT_S="${1#slam_ready_timeout_s:=}"
      ;;
    slam_warmup_s:=*)
      SLAM_WARMUP_S="${1#slam_warmup_s:=}"
      ;;
    pose_settle_s:=*)
      POSE_SETTLE_S="${1#pose_settle_s:=}"
      ;;
    shutdown_gazebo:=*)
      SHUTDOWN_GAZEBO="$(coerce_bool "${1#shutdown_gazebo:=}")"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${1#dry_run:=}")"
      ;;
    *)
      echo "[run_baylands_strip_sweep] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

case "$AXIS" in
  x|y) ;;
  *)
    echo "[run_baylands_strip_sweep] axis must be x or y" >&2
    exit 2
    ;;
esac

case "$DIRECTION" in
  positive|negative) ;;
  *)
    echo "[run_baylands_strip_sweep] direction must be positive or negative" >&2
    exit 2
    ;;
esac

case "$LIDAR_MODE" in
  2d|3d) ;;
  *)
    echo "[run_baylands_strip_sweep] lidar must be 2d or 3d" >&2
    exit 2
    ;;
esac

if [ "$STEPS" = "0" ] && [ -z "$UNTIL_X" ] && [ -z "$UNTIL_Y" ]; then
  if [ -z "$POINTS_FILE" ]; then
    echo "[run_baylands_strip_sweep] Refusing an unbounded sweep. Set steps:=N or until_x:=/until_y:=." >&2
    exit 2
  fi
fi

if [ -n "$POINTS_FILE" ]; then
  if ! POINTS_FILE="$(resolve_points_file "$POINTS_FILE")"; then
    echo "[run_baylands_strip_sweep] Could not resolve points_file '$POINTS_FILE'" >&2
    exit 1
  fi
  load_points_file "$POINTS_FILE"
  if [ "$STEPS_SET" = "false" ]; then
    STEPS="${#POINT_XS[@]}"
  fi
fi

CIRCLE_DURATION_S="$(calc_circle_duration "$CIRCLE_TURNS" "$CIRCLE_ANGULAR")"
NAMESPACE="$(slam_state_namespace "$WS_ROOT")"
MANIFEST_PATH="$WS_ROOT/maps/${PREFIX}_manifest.csv"

mkdir -p "$SWEEP_STATE_DIR"

cat > "$MANIFEST_PATH" <<EOF
index,state_name,x,y,z,yaw,axis,direction,step_m,lidar
EOF

if [ "$DRY_RUN" = "true" ]; then
  echo "[run_baylands_strip_sweep] Dry run"
  echo "  world=$WORLD"
  echo "  gui=$GUI"
  echo "  lidar=$LIDAR_MODE"
  if [ -n "$POINTS_FILE" ]; then
    echo "  points_file=$POINTS_FILE"
    echo "  loaded_points=${#POINT_XS[@]}"
  else
    echo "  start=($START_X, $START_Y, $START_Z, yaw=$START_YAW)"
    echo "  axis=$AXIS direction=$DIRECTION step_m=$STEP_M"
  fi
  echo "  steps=$STEPS until_x=${UNTIL_X:-<none>} until_y=${UNTIL_Y:-<none>}"
  echo "  circle_linear=$CIRCLE_LINEAR circle_angular=$CIRCLE_ANGULAR circle_turns=$CIRCLE_TURNS"
  echo "  circle_duration_s=$CIRCLE_DURATION_S"
  echo "  prefix=$PREFIX"
  exit 0
fi

trap cleanup EXIT INT TERM

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

ensure_no_existing_sim

if [ -n "$POINTS_FILE" ]; then
  first_x="${POINT_XS[0]}"
  first_y="${POINT_YS[0]}"
else
  read -r first_x first_y <<< "$(calc_target_pose 0)"
fi
start_gazebo "$first_x" "$first_y"
wait_for_gazebo

index=0
completed=0

while :; do
  if [ "$STEPS" != "0" ] && [ "$index" -ge "$STEPS" ]; then
    break
  fi

  if [ -n "$POINTS_FILE" ]; then
    if [ "$index" -ge "${#POINT_XS[@]}" ]; then
      break
    fi
    target_x="${POINT_XS[$index]}"
    target_y="${POINT_YS[$index]}"
  else
    read -r target_x target_y <<< "$(calc_target_pose "$index")"
    if ! pose_within_bounds "$target_x" "$target_y"; then
      break
    fi
  fi

  if [ "$index" -gt 0 ]; then
    reposition_ugv "$target_x" "$target_y"
  fi

  start_slam "$index"
  drive_circle "$CIRCLE_DURATION_S"
  save_submap "${PREFIX}_${index}" "$index" "$target_x" "$target_y"
  stop_slam

  completed=$((completed + 1))
  index=$((index + 1))
done

echo "[run_baylands_strip_sweep] Completed $completed sweep stop(s)."
echo "[run_baylands_strip_sweep] Manifest: $MANIFEST_PATH"

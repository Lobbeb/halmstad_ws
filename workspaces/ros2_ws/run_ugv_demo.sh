#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/ruben/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORLD_NAME="default"
DO_BUILD=false
DRIVE_MODE=""
DO_BRIDGE=false
SPAWN_BEPOP=false
DO_FOLLOW=false
FOLLOW_TAKEOFF_DURATION=""
FOLLOW_TAKEOFF_SPEED=""
FOLLOW_DISTANCE=""
UGV_MPS=""
UAV_MAX_Z_MPS=""
INTERRUPTED=0

on_interrupt() {
  INTERRUPTED=1
}

run_allow_interrupt() {
  set +e
  "$@"
  local rc=$?
  set -e

  if [[ "$INTERRUPTED" -eq 1 ]]; then
    echo "Shutdown requested (Ctrl+C). Exiting cleanly."
    return 0
  fi

  case "$rc" in
    0|130|143)
      return 0
      ;;
    *)
      return "$rc"
      ;;
  esac
}

trap on_interrupt INT TERM

for arg in "$@"; do
  case "$arg" in
    build)
      DO_BUILD=true
      ;;
    drive)
      if [[ -n "$DRIVE_MODE" && "$DRIVE_MODE" != "drive_ugv" ]]; then
        echo "Use only one drive mode: 'drive' or 'drive_lidar'." >&2
        exit 1
      fi
      DRIVE_MODE="drive_ugv"
      ;;
    drive_lidar)
      if [[ -n "$DRIVE_MODE" && "$DRIVE_MODE" != "drive_lidar" ]]; then
        echo "Use only one drive mode: 'drive' or 'drive_lidar'." >&2
        exit 1
      fi
      DRIVE_MODE="drive_lidar"
      ;;
    bridge)
      DO_BRIDGE=true
      ;;
    bebop)
      SPAWN_BEPOP=true
      ;;
    camera)
      SPAWN_BEPOP=true
      ;;
    follow)
      DO_FOLLOW=true
      ;;
    takeoff_duration=*)
      FOLLOW_TAKEOFF_DURATION="${arg#takeoff_duration=}"
      ;;
    takeoff_speed_mps=*)
      FOLLOW_TAKEOFF_SPEED="${arg#takeoff_speed_mps=}"
      ;;
    follow_distance=*)
      FOLLOW_DISTANCE="${arg#follow_distance=}"
      ;;
    ugv_mps=*)
      UGV_MPS="${arg#ugv_mps=}"
      ;;
    uav_max_z_mps=*)
      UAV_MAX_Z_MPS="${arg#uav_max_z_mps=}"
      ;;
    world=*)
      WORLD_NAME="${arg#world=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      echo "Usage: ./run_ugv_demo.sh [build] [drive|drive_lidar] [follow] [follow_distance=<m>] [takeoff_duration=<sec>] [takeoff_speed_mps=<m/s>] [ugv_mps=<m/s>] [uav_max_z_mps=<m/s>] [bridge] [bebop] [camera] [world=<name|file>]" >&2
      exit 1
      ;;
  esac
done

if [[ -f "$ROS_SETUP" ]]; then
  set +u
  source "$ROS_SETUP"
  set -u
else
  echo "ROS setup not found at $ROS_SETUP" >&2
  exit 1
fi

cd "$WORKSPACE"


# Build the workspace if requested
if [[ "$DO_BUILD" == true ]]; then
  colcon build --packages-select ugv_gazebo_demo --build-base build --install-base install
fi

if [[ -f "install/setup.bash" ]]; then
  set +u
  source "install/setup.bash"
  set -u
else
  echo "Workspace not built. Run: colcon build --packages-select ugv_gazebo_demo --build-base build --install-base install" >&2
  exit 1
fi

if [[ -n "$DRIVE_MODE" && "$DO_BRIDGE" == true ]]; then
  echo "Use either 'drive/drive_lidar' or 'bridge' in one invocation, not both." >&2
  exit 1
fi

if [[ -z "$DRIVE_MODE" && "$DO_FOLLOW" == true ]]; then
  echo "The 'follow' option must be used with 'drive' or 'drive_lidar'." >&2
  exit 1
fi

if [[ -z "$DRIVE_MODE" && -n "$FOLLOW_DISTANCE" ]]; then
  echo "The 'follow_distance=<m>' option must be used with 'drive' or 'drive_lidar'." >&2
  exit 1
fi

normalize_double_arg() {
  local value="$1"
  if [[ "$value" =~ ^-?[0-9]+$ ]]; then
    echo "${value}.0"
  else
    echo "$value"
  fi
}

validate_double_arg() {
  local label="$1"
  local value="$2"
  if [[ ! "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    echo "Invalid ${label} value: ${value}. Expected numeric value (e.g. 10 or 10.5)." >&2
    exit 1
  fi
}

if [[ -n "$FOLLOW_TAKEOFF_DURATION" ]]; then
  FOLLOW_TAKEOFF_DURATION="$(normalize_double_arg "$FOLLOW_TAKEOFF_DURATION")"
  validate_double_arg "takeoff_duration" "$FOLLOW_TAKEOFF_DURATION"
fi

if [[ -n "$FOLLOW_TAKEOFF_SPEED" ]]; then
  FOLLOW_TAKEOFF_SPEED="$(normalize_double_arg "$FOLLOW_TAKEOFF_SPEED")"
  validate_double_arg "takeoff_speed_mps" "$FOLLOW_TAKEOFF_SPEED"
fi

if [[ -n "$FOLLOW_DISTANCE" ]]; then
  FOLLOW_DISTANCE="$(normalize_double_arg "$FOLLOW_DISTANCE")"
  validate_double_arg "follow_distance" "$FOLLOW_DISTANCE"
fi

if [[ -n "$UGV_MPS" ]]; then
  UGV_MPS="$(normalize_double_arg "$UGV_MPS")"
  validate_double_arg "ugv_mps" "$UGV_MPS"
fi

if [[ -n "$UAV_MAX_Z_MPS" ]]; then
  UAV_MAX_Z_MPS="$(normalize_double_arg "$UAV_MAX_Z_MPS")"
  validate_double_arg "uav_max_z_mps" "$UAV_MAX_Z_MPS"
fi

if [[ "$DO_BRIDGE" == true ]]; then
  run_allow_interrupt ros2 run ugv_gazebo_demo gazebo_pose_tcp_bridge --ros-args \
    -p odom_topics:="['/x2_ugv/odom', '/bebop1/odom']" \
    -p model_names:="['x2_ugv', 'bebop1']"
  exit 0
fi

if [[ -n "$DRIVE_MODE" ]]; then
  odom_topic_active() {
    local topic="$1"
    timeout 1.5s ros2 topic echo --once "$topic" >/dev/null 2>&1
  }

  wait_for_follow_ready() {
    local timeout_s="${1:-60}"
    local start_ts=$SECONDS
    while (( SECONDS - start_ts < timeout_s )); do
      if timeout 2s ros2 topic echo --once /bebop1/follow_ready 2>/dev/null | rg -q "data: true"; then
        return 0
      fi
      sleep 0.3
    done
    return 1
  }

  if [[ "$DO_FOLLOW" == false ]]; then
    if [[ -n "$FOLLOW_DISTANCE" ]]; then
      DO_FOLLOW=true
      echo "follow_distance was set; enabling bebop follower."
    elif odom_topic_active /x2_ugv/odom && odom_topic_active /bebop1/odom; then
      DO_FOLLOW=true
      echo "Detected active /x2_ugv/odom and /bebop1/odom; auto-starting bebop_follow_ugv."
    fi
  fi

  FOLLOW_PID=""

  stop_follow_if_running() {
    if [[ -n "$FOLLOW_PID" ]]; then
      kill -INT "$FOLLOW_PID" >/dev/null 2>&1 || true
      wait "$FOLLOW_PID" >/dev/null 2>&1 || true
      FOLLOW_PID=""
    fi
  }

  if [[ "$DO_FOLLOW" == true ]]; then
    follow_cmd=(ros2 run ugv_gazebo_demo bebop_follow_ugv)
    follow_ros_args=()
    if [[ -n "$FOLLOW_TAKEOFF_DURATION" ]]; then
      follow_ros_args+=(-p "takeoff_duration_s:=$FOLLOW_TAKEOFF_DURATION")
      echo "Using bebop takeoff_duration=${FOLLOW_TAKEOFF_DURATION}s"
    fi
    if [[ -n "$FOLLOW_TAKEOFF_SPEED" ]]; then
      follow_ros_args+=(-p "takeoff_speed_mps:=$FOLLOW_TAKEOFF_SPEED")
      echo "Using bebop takeoff_speed_mps=${FOLLOW_TAKEOFF_SPEED} m/s"
    fi
    if [[ -n "$FOLLOW_DISTANCE" ]]; then
      follow_ros_args+=(-p "follow_distance_m:=$FOLLOW_DISTANCE")
      echo "Using bebop follow_distance=${FOLLOW_DISTANCE}m"
    fi
    if [[ -n "$UGV_MPS" ]]; then
      # Give the follower headroom to keep pace and catch up after turns.
      local_follow_max_xy="$(awk -v s="$UGV_MPS" 'BEGIN { v = s * 1.8; if (v < 2.5) v = 2.5; printf "%.2f", v }')"
      follow_ros_args+=(-p "max_xy_speed_mps:=$local_follow_max_xy")
      echo "Using bebop max_xy_speed_mps=${local_follow_max_xy} (derived from ugv_mps=${UGV_MPS})"
    fi
    if [[ -n "$UAV_MAX_Z_MPS" ]]; then
      follow_ros_args+=(-p "max_z_speed_mps:=$UAV_MAX_Z_MPS")
      echo "Using bebop max_z_speed_mps=${UAV_MAX_Z_MPS} m/s"
    fi
    if (( ${#follow_ros_args[@]} > 0 )); then
      follow_cmd+=(--ros-args "${follow_ros_args[@]}")
    fi
    "${follow_cmd[@]}" &
    FOLLOW_PID=$!
    echo "Waiting for Bebop takeoff to complete before starting UGV drive..."
    if wait_for_follow_ready 75; then
      echo "Bebop follow is ready. Starting UGV drive."
    else
      echo "Timed out waiting for /bebop1/follow_ready=true. Starting UGV drive anyway." >&2
    fi
  fi

  set +e
  if [[ "$DRIVE_MODE" == "drive_lidar" ]]; then
    drive_cmd=(ros2 run ugv_gazebo_demo drive_lidar --ros-args -p scan_topic:=/x2_ugv/scan)
    if [[ -n "$UGV_MPS" ]]; then
      echo "Using ugv_mps=${UGV_MPS} m/s"
      drive_cmd+=(-p "ugv_mps:=$UGV_MPS")
    fi
    run_allow_interrupt "${drive_cmd[@]}"
    DRIVE_RC=$?
  else
    drive_cmd=(ros2 run ugv_gazebo_demo "$DRIVE_MODE")
    if [[ -n "$UGV_MPS" ]]; then
      echo "Using ugv_mps=${UGV_MPS} m/s"
      drive_cmd+=(--ros-args -p "ugv_mps:=$UGV_MPS")
    fi
    run_allow_interrupt "${drive_cmd[@]}"
    DRIVE_RC=$?
  fi
  set -e

  stop_follow_if_running
  exit "$DRIVE_RC"
else
  if pgrep -f "gz sim server" >/dev/null 2>&1; then
    echo "Gazebo server is already running. Close the existing Gazebo session first." >&2
    exit 1
  fi

  resolve_world_path() {
    local world_input="$1"
    local candidate

    if [[ -z "$world_input" ]]; then
      world_input="default"
    fi

    if [[ "$world_input" == */* || "$world_input" == .* ]]; then
      candidate="$world_input"
      [[ "$candidate" != /* ]] && candidate="$WORKSPACE/$candidate"
      [[ -f "$candidate" ]] && { echo "$candidate"; return 0; }
    else
      for candidate in \
        "$WORKSPACE/worlds/$world_input" \
        "$WORKSPACE/worlds/$world_input.sdf" \
        "$WORKSPACE/worlds/$world_input.world"; do
        if [[ -f "$candidate" ]]; then
          echo "$candidate"
          return 0
        fi
      done
    fi

    return 1
  }

  if ! WORLD_PATH="$(resolve_world_path "$WORLD_NAME")"; then
    echo "World not found for world=$WORLD_NAME" >&2
    echo "Available worlds in $WORKSPACE/worlds:" >&2
    find "$WORKSPACE/worlds" -maxdepth 1 -type f \( -name "*.sdf" -o -name "*.world" \) -printf "  %f\n" | sort >&2 || true
    exit 1
  fi

  run_allow_interrupt ros2 launch ugv_gazebo_demo ugv_flat_world.launch.py \
    world:="$WORLD_PATH" \
    spawn_bebop:="$SPAWN_BEPOP"
fi

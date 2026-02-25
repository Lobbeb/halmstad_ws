#!/usr/bin/env bash

# IMPORTANT:
# - This file is executed (bash scripts/run_round.sh ...), not sourced.
# - We source scripts/env.sh BEFORE enabling "nounset" (-u), because ROS setup.bash
#   is not nounset-safe on Ubuntu 24.04 (PEP 668 etc).

# Load ROS env (must happen before set -u)
source "$(dirname "$0")/env.sh"

# Now enable strict mode safely
set -euo pipefail

FOLLOW_PID=""
EST_PID=""
BAG_PID=""


QOS_FILE="$WS_ROOT/config/rosbag_qos.yaml"
if [ ! -f "$QOS_FILE" ]; then
  echo "ERROR: QoS override file missing: $QOS_FILE"
  exit 2
fi

# RUNBOOK expects: scripts/run_round.sh <run_id> <condition> <uav> <world> [--with-cameras]
if [ $# -lt 4 ]; then
  echo "Usage: $0 <run_id> <condition> <uav_name> <world> [--with-cameras]"
  exit 2
fi

RUN_ID="$1"
CONDITION="$2"
UAV="$3"
WORLD="$4"
WITH_CAMERAS="false"
if [ "${5:-}" = "--with-cameras" ]; then
  WITH_CAMERAS="true"
fi

LEADER_MODE="${LEADER_MODE:-odom}"
LEADER_PERCEPTION_ENABLE="${LEADER_PERCEPTION_ENABLE:-false}"
FOLLOW_PROFILE="${FOLLOW_PROFILE:-auto}" # auto|odom_core|estimate_robust
UGV_ODOM_TOPIC_EXPLICIT="${UGV_ODOM_TOPIC+x}"
UGV_ODOM_TOPIC="${UGV_ODOM_TOPIC:-/a201_0000/platform/odom}"
UGV_CMD_VEL_TOPIC="${UGV_CMD_VEL_TOPIC:-/a201_0000/cmd_vel}"
UGV_CMD_TOPICS="${UGV_CMD_TOPICS:-/a201_0000/cmd_vel,/a201_0000/platform/cmd_vel}"
REQUIRE_FLOW="${REQUIRE_FLOW:-1}"
NODE_SUFFIX="$(echo "$RUN_ID" | tr -c '[:alnum:]_' '_')"
YOLO_WEIGHTS="${YOLO_WEIGHTS:-}"
YOLO_DEVICE="${YOLO_DEVICE:-cpu}"
LEADER_IMAGE_TOPIC="${LEADER_IMAGE_TOPIC:-${LEADER_CAMERA_TOPIC:-/${UAV}/camera0/image_raw}}"
LEADER_CAMERA_INFO_TOPIC="${LEADER_CAMERA_INFO_TOPIC:-/${UAV}/camera0/camera_info}"
LEADER_DEPTH_TOPIC="${LEADER_DEPTH_TOPIC:-}"
LEADER_UAV_POSE_TOPIC="${LEADER_UAV_POSE_TOPIC:-/${UAV}/pose_cmd}"
LEADER_CONSTANT_RANGE_M="${LEADER_CONSTANT_RANGE_M:-8.0}"
LEADER_EST_SMOOTH_ALPHA="${LEADER_EST_SMOOTH_ALPHA:-0.35}"
LEADER_EST_MAX_HOLD_FRAMES="${LEADER_EST_MAX_HOLD_FRAMES:-3}"
LEADER_EST_MAX_HOLD_S="${LEADER_EST_MAX_HOLD_S:-0.5}"
LEADER_EST_XY_SMOOTH_ALPHA="${LEADER_EST_XY_SMOOTH_ALPHA:-0.5}"
LEADER_EST_RANGE_MODE="${LEADER_EST_RANGE_MODE:-auto}"
LEADER_EST_MAX_XY_JUMP_M="${LEADER_EST_MAX_XY_JUMP_M:-6.0}"
LEADER_EST_MAX_TARGET_SPEED_MPS="${LEADER_EST_MAX_TARGET_SPEED_MPS:-8.0}"
LEADER_EST_MAX_BEARING_JUMP_DEG="${LEADER_EST_MAX_BEARING_JUMP_DEG:-40.0}"
LEADER_EST_HZ="${LEADER_EST_HZ:-10.0}"
LEADER_CAM_PITCH_OFFSET_DEG="${LEADER_CAM_PITCH_OFFSET_DEG:--90.0}"
LEADER_CAM_YAW_OFFSET_DEG="${LEADER_CAM_YAW_OFFSET_DEG:-0.0}"
LEADER_TARGET_GROUND_Z_M="${LEADER_TARGET_GROUND_Z_M:-0.0}"
LEADER_GROUND_MIN_RANGE_M="${LEADER_GROUND_MIN_RANGE_M:-2.0}"
LEADER_GROUND_MAX_RANGE_M="${LEADER_GROUND_MAX_RANGE_M:-50.0}"
# Track explicit advanced follow overrides so auto profile can preserve user intent.
FOLLOW_MAX_STEP_M_PER_TICK_EXPLICIT="${FOLLOW_MAX_STEP_M_PER_TICK+x}"
FOLLOW_CMD_XY_DEADBAND_M_EXPLICIT="${FOLLOW_CMD_XY_DEADBAND_M+x}"
FOLLOW_MAX_YAW_STEP_RAD_PER_TICK_EXPLICIT="${FOLLOW_MAX_YAW_STEP_RAD_PER_TICK+x}"
FOLLOW_QUALITY_SCALE_ENABLE_EXPLICIT="${FOLLOW_QUALITY_SCALE_ENABLE+x}"
FOLLOW_STATE_MACHINE_ENABLE_EXPLICIT="${FOLLOW_STATE_MACHINE_ENABLE+x}"
FOLLOW_TRAJ_ENABLE_EXPLICIT="${FOLLOW_TRAJ_ENABLE+x}"
FOLLOW_TRAJ_REL_FRAME_ENABLE_EXPLICIT="${FOLLOW_TRAJ_REL_FRAME_ENABLE+x}"
FOLLOW_YAW_DEADBAND_RAD_EXPLICIT="${FOLLOW_YAW_DEADBAND_RAD+x}"
FOLLOW_YAW_UPDATE_XY_GATE_M_EXPLICIT="${FOLLOW_YAW_UPDATE_XY_GATE_M+x}"
FOLLOW_EST_HEADING_FROM_MOTION_ENABLE_EXPLICIT="${FOLLOW_EST_HEADING_FROM_MOTION_ENABLE+x}"
FOLLOW_TICK_HZ="${FOLLOW_TICK_HZ:-5.0}"
FOLLOW_MAX_STEP_M_PER_TICK="${FOLLOW_MAX_STEP_M_PER_TICK:-0.5}"
FOLLOW_CMD_XY_DEADBAND_M="${FOLLOW_CMD_XY_DEADBAND_M:-0.0}"
FOLLOW_MAX_YAW_STEP_RAD_PER_TICK="${FOLLOW_MAX_YAW_STEP_RAD_PER_TICK:-0.0}"
FOLLOW_QUALITY_SCALE_ENABLE="${FOLLOW_QUALITY_SCALE_ENABLE:-true}"
FOLLOW_QUALITY_STATUS_TIMEOUT_S="${FOLLOW_QUALITY_STATUS_TIMEOUT_S:-1.5}"
FOLLOW_QUALITY_CONF_MIN="${FOLLOW_QUALITY_CONF_MIN:-0.15}"
FOLLOW_QUALITY_CONF_GOOD="${FOLLOW_QUALITY_CONF_GOOD:-0.65}"
FOLLOW_QUALITY_LATENCY_REF_MS="${FOLLOW_QUALITY_LATENCY_REF_MS:-180.0}"
FOLLOW_QUALITY_MIN_STEP_SCALE="${FOLLOW_QUALITY_MIN_STEP_SCALE:-0.25}"
FOLLOW_QUALITY_HOLD_STEP_SCALE="${FOLLOW_QUALITY_HOLD_STEP_SCALE:-0.08}"
FOLLOW_QUALITY_DEADBAND_BOOST_M="${FOLLOW_QUALITY_DEADBAND_BOOST_M:-0.05}"
FOLLOW_STATE_MACHINE_ENABLE="${FOLLOW_STATE_MACHINE_ENABLE:-true}"
FOLLOW_STATE_OK_DEBOUNCE_TICKS="${FOLLOW_STATE_OK_DEBOUNCE_TICKS:-2}"
FOLLOW_STATE_BAD_DEBOUNCE_TICKS="${FOLLOW_STATE_BAD_DEBOUNCE_TICKS:-2}"
FOLLOW_TRAJ_ENABLE="${FOLLOW_TRAJ_ENABLE:-true}"
FOLLOW_TRAJ_REL_FRAME_ENABLE="${FOLLOW_TRAJ_REL_FRAME_ENABLE:-true}"
FOLLOW_TRAJ_REL_SMOOTH_ALPHA="${FOLLOW_TRAJ_REL_SMOOTH_ALPHA:-0.35}"
FOLLOW_TRAJ_POS_GAIN="${FOLLOW_TRAJ_POS_GAIN:-2.0}"
FOLLOW_TRAJ_MAX_SPEED_MPS="${FOLLOW_TRAJ_MAX_SPEED_MPS:-1.5}"
FOLLOW_TRAJ_MAX_ACCEL_MPS2="${FOLLOW_TRAJ_MAX_ACCEL_MPS2:-3.0}"
FOLLOW_TRAJ_QUALITY_MIN_SCALE="${FOLLOW_TRAJ_QUALITY_MIN_SCALE:-0.35}"
FOLLOW_TRAJ_RESET_ON_YAW_JUMP_RAD="${FOLLOW_TRAJ_RESET_ON_YAW_JUMP_RAD:-0.7}"
FOLLOW_YAW_DEADBAND_RAD="${FOLLOW_YAW_DEADBAND_RAD:-0.05}"
FOLLOW_YAW_UPDATE_XY_GATE_M="${FOLLOW_YAW_UPDATE_XY_GATE_M:-0.03}"
FOLLOW_EST_HEADING_FROM_MOTION_ENABLE="${FOLLOW_EST_HEADING_FROM_MOTION_ENABLE:-true}"
FOLLOW_EST_HEADING_ALPHA="${FOLLOW_EST_HEADING_ALPHA:-0.35}"
FOLLOW_EST_HEADING_MIN_SPEED_MPS="${FOLLOW_EST_HEADING_MIN_SPEED_MPS:-0.15}"
FOLLOW_EST_HEADING_MAX_DT_S="${FOLLOW_EST_HEADING_MAX_DT_S:-1.0}"
AUTO_SELECT_UGV_ODOM_TOPIC="${AUTO_SELECT_UGV_ODOM_TOPIC:-1}"
UGV_MOTION_PROFILE="${UGV_MOTION_PROFILE:-default}"
UGV_TURN_PATTERN="${UGV_TURN_PATTERN:-alternate}"
UGV_FORWARD_SPEED="${UGV_FORWARD_SPEED:-}"
UGV_FORWARD_TIME_S="${UGV_FORWARD_TIME_S:-}"
UGV_TURN_SPEED="${UGV_TURN_SPEED:-}"
UGV_TURN_TIME_S="${UGV_TURN_TIME_S:-}"
UGV_CYCLES="${UGV_CYCLES:-}"
UGV_CMD_PUB_RATE_HZ="${UGV_CMD_PUB_RATE_HZ:-12.0}"
UGV_CMD_STARTUP_PAD_S="${UGV_CMD_STARTUP_PAD_S:-0.7}"
UGV_VARIATION_ENABLE="${UGV_VARIATION_ENABLE:-true}"
UGV_VARIATION_AMPLITUDE="${UGV_VARIATION_AMPLITUDE:-0.20}"
UGV_PAUSE_EVERY_N="${UGV_PAUSE_EVERY_N:-0}"
UGV_PAUSE_TIME_S="${UGV_PAUSE_TIME_S:-0.0}"

FOLLOW_PROFILE_RESOLVED="$FOLLOW_PROFILE"
if [ "$FOLLOW_PROFILE" = "auto" ]; then
  if [ "$LEADER_MODE" = "odom" ]; then
    FOLLOW_PROFILE_RESOLVED="odom_core"
  else
    FOLLOW_PROFILE_RESOLVED="estimate_robust"
  fi
fi

# Align defaults with project architecture:
# - odom follow is the authoritative Stage 2.0 control path
# - estimate follow is experimental and benefits from shaping/robustness layers
if [ "$FOLLOW_PROFILE_RESOLVED" = "odom_core" ]; then
  [ -z "$FOLLOW_MAX_STEP_M_PER_TICK_EXPLICIT" ] && FOLLOW_MAX_STEP_M_PER_TICK="0.0"
  [ -z "$FOLLOW_CMD_XY_DEADBAND_M_EXPLICIT" ] && FOLLOW_CMD_XY_DEADBAND_M="0.0"
  [ -z "$FOLLOW_MAX_YAW_STEP_RAD_PER_TICK_EXPLICIT" ] && FOLLOW_MAX_YAW_STEP_RAD_PER_TICK="0.0"
  [ -z "$FOLLOW_YAW_DEADBAND_RAD_EXPLICIT" ] && FOLLOW_YAW_DEADBAND_RAD="0.0"
  [ -z "$FOLLOW_YAW_UPDATE_XY_GATE_M_EXPLICIT" ] && FOLLOW_YAW_UPDATE_XY_GATE_M="0.0"
  [ -z "$FOLLOW_QUALITY_SCALE_ENABLE_EXPLICIT" ] && FOLLOW_QUALITY_SCALE_ENABLE="false"
  [ -z "$FOLLOW_STATE_MACHINE_ENABLE_EXPLICIT" ] && FOLLOW_STATE_MACHINE_ENABLE="false"
  [ -z "$FOLLOW_TRAJ_ENABLE_EXPLICIT" ] && FOLLOW_TRAJ_ENABLE="false"
  [ -z "$FOLLOW_TRAJ_REL_FRAME_ENABLE_EXPLICIT" ] && FOLLOW_TRAJ_REL_FRAME_ENABLE="false"
  [ -z "$FOLLOW_EST_HEADING_FROM_MOTION_ENABLE_EXPLICIT" ] && FOLLOW_EST_HEADING_FROM_MOTION_ENABLE="false"
fi

YOLO_VARIANT="${YOLO_VARIANT:-auto}"
if [ "$YOLO_VARIANT" = "auto" ]; then
  case "$(basename "${YOLO_WEIGHTS:-}")" in
    *yolov5n*.pt) YOLO_VARIANT="v5n" ;;
    *yolov5s*.pt) YOLO_VARIANT="v5s" ;;
    "") YOLO_VARIANT="" ;;
    *) YOLO_VARIANT="custom" ;;
  esac
fi

# Ensure run root exists
RUN_ROOT="${RUN_ROOT:-$HOME/halmstad_ws/runs}"

# Put runs into runs/<group>/<run_id> (optional). Default keeps old behavior.
# You can set RUN_GROUP explicitly, or let it auto-pick based on CONDITION + LEADER_MODE.
RUN_GROUP="${RUN_GROUP:-auto}"

if [ "$RUN_GROUP" = "auto" ]; then
  if [ "$CONDITION" = "follow" ]; then
    if [ "$LEADER_MODE" = "pose" ] || [ "$LEADER_MODE" = "estimate" ]; then
      RUN_GROUP="estimator_pose"
    else
      RUN_GROUP="estimator"
    fi
  else
    RUN_GROUP="baseline"
  fi
fi

RUN_DIR="$RUN_ROOT/$RUN_GROUP/$RUN_ID"
mkdir -p "$RUN_DIR"
rm -rf "$RUN_DIR/bag"

START_TS="$(date -Iseconds)"

# Event marker topic (default keeps Stage-1 behavior unchanged)
EVENT_TOPIC="${EVENT_TOPIC:-/coord/events}"

pub_evt () {
  local s="$1"
  ros2 topic pub -1 "$EVENT_TOPIC" std_msgs/msg/String "{data: '$s'}" >/dev/null
  sleep 0.2
  ros2 topic pub -1 "$EVENT_TOPIC" std_msgs/msg/String "{data: '$s'}" >/dev/null
}

kill_cmd_vel_publishers () {
  pkill -9 -f "ros2 topic pub.*(/a201_0000/cmd_vel|/a201_0000/platform/cmd_vel)" 2>/dev/null || true
}

topic_has_flow () {
  local topic="$1"
  timeout 2s ros2 topic echo --once "$topic" >/dev/null 2>&1
}

auto_select_ugv_odom_topic () {
  # Preserve explicit user choice. Auto-selection is only for the default path.
  if [ -n "${UGV_ODOM_TOPIC_EXPLICIT}" ] || [ "$AUTO_SELECT_UGV_ODOM_TOPIC" != "1" ]; then
    return 0
  fi

  local candidates=(
    "/a201_0000/platform/odom"
    "/a201_0000/platform/odom/filtered"
  )
  local selected=""
  for t in "${candidates[@]}"; do
    if topic_has_flow "$t"; then
      selected="$t"
      break
    fi
  done

  if [ -n "$selected" ] && [ "$selected" != "$UGV_ODOM_TOPIC" ]; then
    echo "[run_round] Auto-selected live UGV_ODOM_TOPIC=$selected (default had no flow)"
    UGV_ODOM_TOPIC="$selected"
  fi
}

auto_select_ugv_odom_topic

export EVENT_TOPIC
export REQUIRE_FLOW
export UGV_CMD_TOPICS
export UGV_CMD_VEL_TOPIC
export REQUIRED_FLOW_TOPICS="$UGV_ODOM_TOPIC"
export UGV_MOTION_PROFILE
export UGV_TURN_PATTERN
export UGV_FORWARD_SPEED
export UGV_FORWARD_TIME_S
export UGV_TURN_SPEED
export UGV_TURN_TIME_S
export UGV_CYCLES
export UGV_CMD_PUB_RATE_HZ
export UGV_CMD_STARTUP_PAD_S
export UGV_VARIATION_ENABLE
export UGV_VARIATION_AMPLITUDE
export UGV_PAUSE_EVERY_N
export UGV_PAUSE_TIME_S

run_contract_check () {
  ros2 run lrs_halmstad contract_check "$WORLD" "$UAV" 10
}

# Sanity guard. On some setups the ros2 CLI-based auto-select probe can miss live flow
# intermittently, so retry with /odom/filtered if the default /odom flow check fails.
contract_rc=0
run_contract_check || contract_rc=$?
if [ "$contract_rc" -ne 0 ]; then
  if [ -z "${UGV_ODOM_TOPIC_EXPLICIT}" ] && [ "$REQUIRE_FLOW" = "1" ] && [ "$UGV_ODOM_TOPIC" = "/a201_0000/platform/odom" ]; then
    echo "[run_round] contract_check failed on $UGV_ODOM_TOPIC; retrying with /a201_0000/platform/odom/filtered"
    UGV_ODOM_TOPIC="/a201_0000/platform/odom/filtered"
    export REQUIRED_FLOW_TOPICS="$UGV_ODOM_TOPIC"
    run_contract_check
  else
    exit "$contract_rc"
  fi
fi

# Topics list aligns with your Stage-1 logging
TOPICS=(
  "/clock"
  "$UGV_ODOM_TOPIC"
  "/a201_0000/platform/odom/filtered"
  "/a201_0000/platform/cmd_vel"
  "$UGV_CMD_VEL_TOPIC"
  "/a201_0000/tf"
  "/a201_0000/tf_static"
  "$EVENT_TOPIC"
  "/${UAV}/pose_cmd"
  "/coord/leader_estimate"
  "/coord/leader_estimate_status"
  "/coord/follow_dist_cmd"
  "/coord/follow_tracking_error_cmd"
)

# If we publish markers to /coord/events_raw, also record the impaired output /coord/events
if [ "$EVENT_TOPIC" != "/coord/events" ]; then
  TOPICS+=("/coord/events")
fi

if [ "$WITH_CAMERAS" = "true" ]; then
  TOPICS+=("/${UAV}/camera0/image_raw")
  TOPICS+=("/${UAV}/camera0/camera_info")
  if [ -n "$LEADER_DEPTH_TOPIC" ]; then
    TOPICS+=("$LEADER_DEPTH_TOPIC")
  fi
  TOPICS+=("/a201_0000/sensors/camera_0/color/image")
  TOPICS+=("/a201_0000/sensors/camera_0/color/camera_info")
  TOPICS+=("/a201_0000/sensors/camera_0/depth/image")
  TOPICS+=("/a201_0000/sensors/camera_0/depth/compressedDepth")
fi


# Write meta.yaml
cat > "$RUN_DIR/meta.yaml" << EOF
run_id: ${RUN_ID}
run_group: ${RUN_GROUP}
timestamp: ${START_TS}
condition: ${CONDITION}
world: ${WORLD}
uav_name: ${UAV}
with_cameras: ${WITH_CAMERAS}
leader_mode: "${LEADER_MODE}"
leader_perception_enable: ${LEADER_PERCEPTION_ENABLE}
follow_profile_requested: "${FOLLOW_PROFILE}"
follow_profile_resolved: "${FOLLOW_PROFILE_RESOLVED}"
yolo_weights: "${YOLO_WEIGHTS}"
device: "${YOLO_DEVICE}"
yolo_variant: "${YOLO_VARIANT}"
leader_image_topic: "${LEADER_IMAGE_TOPIC}"
leader_est_smooth_alpha: ${LEADER_EST_SMOOTH_ALPHA}
leader_est_max_hold_frames: ${LEADER_EST_MAX_HOLD_FRAMES}
leader_est_max_hold_s: ${LEADER_EST_MAX_HOLD_S}
leader_est_xy_smooth_alpha: ${LEADER_EST_XY_SMOOTH_ALPHA}
leader_est_range_mode: "${LEADER_EST_RANGE_MODE}"
leader_est_max_xy_jump_m: ${LEADER_EST_MAX_XY_JUMP_M}
leader_est_max_target_speed_mps: ${LEADER_EST_MAX_TARGET_SPEED_MPS}
leader_est_max_bearing_jump_deg: ${LEADER_EST_MAX_BEARING_JUMP_DEG}
leader_est_hz: ${LEADER_EST_HZ}
leader_cam_pitch_offset_deg: ${LEADER_CAM_PITCH_OFFSET_DEG}
leader_cam_yaw_offset_deg: ${LEADER_CAM_YAW_OFFSET_DEG}
leader_target_ground_z_m: ${LEADER_TARGET_GROUND_Z_M}
leader_ground_min_range_m: ${LEADER_GROUND_MIN_RANGE_M}
leader_ground_max_range_m: ${LEADER_GROUND_MAX_RANGE_M}
follow_tick_hz: ${FOLLOW_TICK_HZ}
follow_max_step_m_per_tick: ${FOLLOW_MAX_STEP_M_PER_TICK}
follow_cmd_xy_deadband_m: ${FOLLOW_CMD_XY_DEADBAND_M}
follow_max_yaw_step_rad_per_tick: ${FOLLOW_MAX_YAW_STEP_RAD_PER_TICK}
follow_quality_scale_enable: ${FOLLOW_QUALITY_SCALE_ENABLE}
follow_quality_status_timeout_s: ${FOLLOW_QUALITY_STATUS_TIMEOUT_S}
follow_quality_conf_min: ${FOLLOW_QUALITY_CONF_MIN}
follow_quality_conf_good: ${FOLLOW_QUALITY_CONF_GOOD}
follow_quality_latency_ref_ms: ${FOLLOW_QUALITY_LATENCY_REF_MS}
follow_quality_min_step_scale: ${FOLLOW_QUALITY_MIN_STEP_SCALE}
follow_quality_hold_step_scale: ${FOLLOW_QUALITY_HOLD_STEP_SCALE}
follow_quality_deadband_boost_m: ${FOLLOW_QUALITY_DEADBAND_BOOST_M}
follow_state_machine_enable: ${FOLLOW_STATE_MACHINE_ENABLE}
follow_state_ok_debounce_ticks: ${FOLLOW_STATE_OK_DEBOUNCE_TICKS}
follow_state_bad_debounce_ticks: ${FOLLOW_STATE_BAD_DEBOUNCE_TICKS}
follow_traj_enable: ${FOLLOW_TRAJ_ENABLE}
follow_traj_rel_frame_enable: ${FOLLOW_TRAJ_REL_FRAME_ENABLE}
follow_traj_rel_smooth_alpha: ${FOLLOW_TRAJ_REL_SMOOTH_ALPHA}
follow_traj_pos_gain: ${FOLLOW_TRAJ_POS_GAIN}
follow_traj_max_speed_mps: ${FOLLOW_TRAJ_MAX_SPEED_MPS}
follow_traj_max_accel_mps2: ${FOLLOW_TRAJ_MAX_ACCEL_MPS2}
follow_traj_quality_min_scale: ${FOLLOW_TRAJ_QUALITY_MIN_SCALE}
follow_traj_reset_on_yaw_jump_rad: ${FOLLOW_TRAJ_RESET_ON_YAW_JUMP_RAD}
follow_yaw_deadband_rad: ${FOLLOW_YAW_DEADBAND_RAD}
follow_yaw_update_xy_gate_m: ${FOLLOW_YAW_UPDATE_XY_GATE_M}
follow_est_heading_from_motion_enable: ${FOLLOW_EST_HEADING_FROM_MOTION_ENABLE}
follow_est_heading_alpha: ${FOLLOW_EST_HEADING_ALPHA}
follow_est_heading_min_speed_mps: ${FOLLOW_EST_HEADING_MIN_SPEED_MPS}
follow_est_heading_max_dt_s: ${FOLLOW_EST_HEADING_MAX_DT_S}
ugv_motion_profile: "${UGV_MOTION_PROFILE}"
ugv_turn_pattern: "${UGV_TURN_PATTERN}"
ugv_cmd_pub_rate_hz: ${UGV_CMD_PUB_RATE_HZ}
ugv_cmd_startup_pad_s: ${UGV_CMD_STARTUP_PAD_S}
ugv_variation_enable: ${UGV_VARIATION_ENABLE}
ugv_variation_amplitude: ${UGV_VARIATION_AMPLITUDE}
ugv_pause_every_n: ${UGV_PAUSE_EVERY_N}
ugv_pause_time_s: ${UGV_PAUSE_TIME_S}
EOF
if [ -n "$LEADER_DEPTH_TOPIC" ]; then
  echo "leader_depth_topic: \"${LEADER_DEPTH_TOPIC}\"" >> "$RUN_DIR/meta.yaml"
fi
if [ -n "$UGV_FORWARD_SPEED" ]; then
  echo "ugv_forward_speed_override: ${UGV_FORWARD_SPEED}" >> "$RUN_DIR/meta.yaml"
fi
if [ -n "$UGV_FORWARD_TIME_S" ]; then
  echo "ugv_forward_time_s_override: ${UGV_FORWARD_TIME_S}" >> "$RUN_DIR/meta.yaml"
fi
if [ -n "$UGV_TURN_SPEED" ]; then
  echo "ugv_turn_speed_override: ${UGV_TURN_SPEED}" >> "$RUN_DIR/meta.yaml"
fi
if [ -n "$UGV_TURN_TIME_S" ]; then
  echo "ugv_turn_time_s_override: ${UGV_TURN_TIME_S}" >> "$RUN_DIR/meta.yaml"
fi
if [ -n "$UGV_CYCLES" ]; then
  echo "ugv_cycles_override: ${UGV_CYCLES}" >> "$RUN_DIR/meta.yaml"
fi

cleanup() {
  set +e
  kill_cmd_vel_publishers
  if [ -n "$FOLLOW_PID" ]; then
    kill "$FOLLOW_PID" 2>/dev/null || true
    wait "$FOLLOW_PID" 2>/dev/null || true
  fi
  if [ -n "$EST_PID" ]; then
    kill "$EST_PID" 2>/dev/null || true
    wait "$EST_PID" 2>/dev/null || true
  fi
  if [ -n "$BAG_PID" ]; then
    kill "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  kill_cmd_vel_publishers
}

trap cleanup EXIT

echo "[run_round] Recording rosbag to $RUN_DIR/bag ..."
ros2 bag record -o "$RUN_DIR/bag" \
  --qos-profile-overrides-path "$QOS_FILE" \
  --topics "${TOPICS[@]}" >/dev/null 2>&1 &
BAG_PID=$!
sleep 1.0 # Give rosbag a moment to start and create the bag directory
# Event markers
pub_evt "ROUND_START"


# UAV phase (Stage 1 default: grid sweep). Stage 2: follow+leash if CONDITION=follow.
pub_evt "UAV_PHASE_START"

if [ "$CONDITION" = "follow" ]; then
  echo "[run_round] CONDITION=follow -> starting leader_estimator + follow_uav (background) during UGV phase"

  EST_PID=""
  START_ESTIMATOR="false"
  if [ "$LEADER_MODE" = "pose" ] || [ "$LEADER_MODE" = "estimate" ]; then
    START_ESTIMATOR="true"
    echo "[run_round] Starting leader_estimator (background; control input source=$LEADER_MODE)"
  elif [ "$LEADER_PERCEPTION_ENABLE" = "true" ]; then
    START_ESTIMATOR="true"
    echo "[run_round] Starting leader_estimator in perception-only mode (follow uses odom)"
  fi

  if [ "$START_ESTIMATOR" = "true" ]; then
    EST_EXTRA_ARGS=()
    if [ -n "$LEADER_DEPTH_TOPIC" ]; then
      EST_EXTRA_ARGS+=(-p "depth_topic:=$LEADER_DEPTH_TOPIC")
    fi
    if [ -n "$YOLO_WEIGHTS" ]; then
      EST_EXTRA_ARGS+=(-p "yolo_weights:=$YOLO_WEIGHTS")
    fi
    ros2 run lrs_halmstad leader_estimator --ros-args \
      -r __node:=leader_estimator_${NODE_SUFFIX} \
      -p use_sim_time:=true \
      -p uav_name:="$UAV" \
      -p camera_topic:="$LEADER_IMAGE_TOPIC" \
      -p camera_info_topic:="$LEADER_CAMERA_INFO_TOPIC" \
      -p uav_pose_topic:="$LEADER_UAV_POSE_TOPIC" \
      -p out_topic:=/coord/leader_estimate \
      -p status_topic:=/coord/leader_estimate_status \
      -p publish_status:=true \
      -p est_hz:="$LEADER_EST_HZ" \
      -p image_timeout_s:=1.0 \
      -p uav_pose_timeout_s:=2.0 \
      -p constant_range_m:="$LEADER_CONSTANT_RANGE_M" \
      -p smooth_alpha:="$LEADER_EST_SMOOTH_ALPHA" \
      -p max_hold_frames:="$LEADER_EST_MAX_HOLD_FRAMES" \
      -p max_hold_s:="$LEADER_EST_MAX_HOLD_S" \
      -p xy_smooth_alpha:="$LEADER_EST_XY_SMOOTH_ALPHA" \
      -p range_mode:="$LEADER_EST_RANGE_MODE" \
      -p max_xy_jump_m:="$LEADER_EST_MAX_XY_JUMP_M" \
      -p max_target_speed_mps:="$LEADER_EST_MAX_TARGET_SPEED_MPS" \
      -p max_bearing_jump_deg:="$LEADER_EST_MAX_BEARING_JUMP_DEG" \
      -p cam_pitch_offset_deg:="$LEADER_CAM_PITCH_OFFSET_DEG" \
      -p cam_yaw_offset_deg:="$LEADER_CAM_YAW_OFFSET_DEG" \
      -p target_ground_z_m:="$LEADER_TARGET_GROUND_Z_M" \
      -p ground_min_range_m:="$LEADER_GROUND_MIN_RANGE_M" \
      -p ground_max_range_m:="$LEADER_GROUND_MAX_RANGE_M" \
      -p device:="$YOLO_DEVICE" \
      -p event_topic:="$EVENT_TOPIC" \
      "${EST_EXTRA_ARGS[@]}" \
      -p publish_events:=true \
      > "$RUN_DIR/leader_estimator.log" 2>&1 &
    EST_PID=$!
    sleep 0.3
  else
    echo "[run_round] LEADER_MODE=$LEADER_MODE and LEADER_PERCEPTION_ENABLE=$LEADER_PERCEPTION_ENABLE -> not starting leader_estimator"
  fi

  echo "[run_round] Starting follow_uav (leader_mode=$LEADER_MODE, follow_profile=$FOLLOW_PROFILE_RESOLVED)"
  ros2 run lrs_halmstad follow_uav --ros-args \
    -r __node:=follow_uav_${NODE_SUFFIX} \
    -p use_sim_time:=true \
    -p world:="$WORLD" \
    -p uav_name:="$UAV" \
    -p leader_input_type:="$LEADER_MODE" \
    -p leader_odom_topic:="$UGV_ODOM_TOPIC" \
    -p leader_pose_topic:=/coord/leader_estimate \
    -p tick_hz:="$FOLLOW_TICK_HZ" \
    -p d_target:=5.0 \
    -p d_max:=15.0 \
    -p z_alt:=10.0 \
    -p pose_timeout_s:=0.75 \
    -p min_cmd_period_s:=0.10 \
    -p smooth_alpha:=1.0 \
    -p max_step_m_per_tick:="$FOLLOW_MAX_STEP_M_PER_TICK" \
    -p cmd_xy_deadband_m:="$FOLLOW_CMD_XY_DEADBAND_M" \
    -p max_yaw_step_rad_per_tick:="$FOLLOW_MAX_YAW_STEP_RAD_PER_TICK" \
    -p leader_status_topic:=/coord/leader_estimate_status \
    -p quality_scale_enable:="$FOLLOW_QUALITY_SCALE_ENABLE" \
    -p quality_status_timeout_s:="$FOLLOW_QUALITY_STATUS_TIMEOUT_S" \
    -p quality_conf_min:="$FOLLOW_QUALITY_CONF_MIN" \
    -p quality_conf_good:="$FOLLOW_QUALITY_CONF_GOOD" \
    -p quality_latency_ref_ms:="$FOLLOW_QUALITY_LATENCY_REF_MS" \
    -p quality_min_step_scale:="$FOLLOW_QUALITY_MIN_STEP_SCALE" \
    -p quality_hold_step_scale:="$FOLLOW_QUALITY_HOLD_STEP_SCALE" \
    -p quality_deadband_boost_m:="$FOLLOW_QUALITY_DEADBAND_BOOST_M" \
    -p state_machine_enable:="$FOLLOW_STATE_MACHINE_ENABLE" \
    -p state_ok_debounce_ticks:="$FOLLOW_STATE_OK_DEBOUNCE_TICKS" \
    -p state_bad_debounce_ticks:="$FOLLOW_STATE_BAD_DEBOUNCE_TICKS" \
    -p traj_enable:="$FOLLOW_TRAJ_ENABLE" \
    -p traj_rel_frame_enable:="$FOLLOW_TRAJ_REL_FRAME_ENABLE" \
    -p traj_rel_smooth_alpha:="$FOLLOW_TRAJ_REL_SMOOTH_ALPHA" \
    -p traj_pos_gain:="$FOLLOW_TRAJ_POS_GAIN" \
    -p traj_max_speed_mps:="$FOLLOW_TRAJ_MAX_SPEED_MPS" \
    -p traj_max_accel_mps2:="$FOLLOW_TRAJ_MAX_ACCEL_MPS2" \
    -p traj_quality_min_scale:="$FOLLOW_TRAJ_QUALITY_MIN_SCALE" \
    -p traj_reset_on_yaw_jump_rad:="$FOLLOW_TRAJ_RESET_ON_YAW_JUMP_RAD" \
    -p yaw_deadband_rad:="$FOLLOW_YAW_DEADBAND_RAD" \
    -p yaw_update_xy_gate_m:="$FOLLOW_YAW_UPDATE_XY_GATE_M" \
    -p estimate_heading_from_motion_enable:="$FOLLOW_EST_HEADING_FROM_MOTION_ENABLE" \
    -p estimate_heading_alpha:="$FOLLOW_EST_HEADING_ALPHA" \
    -p estimate_heading_min_speed_mps:="$FOLLOW_EST_HEADING_MIN_SPEED_MPS" \
    -p estimate_heading_max_dt_s:="$FOLLOW_EST_HEADING_MAX_DT_S" \
    -p event_topic:="$EVENT_TOPIC" \
    > "$RUN_DIR/follow_uav.log" 2>&1 &
  FOLLOW_PID=$!

  pub_evt "UAV_PHASE_END"

else
  echo "[run_round] CONDITION=$CONDITION -> running Stage 1 UAV sweep"
  export UAV="$UAV"
  export WORLD="$WORLD"
  export RUN_DIR="$RUN_DIR"

  python3 - << 'PY'
import os, yaml, subprocess
from pathlib import Path

defaults_path = Path(os.path.expanduser("~/halmstad_ws/src/lrs_halmstad/resource/experiment_defaults.yaml"))
defaults = yaml.safe_load(defaults_path.read_text()) if defaults_path.exists() else {}

uav = os.environ["UAV"]
world = os.environ["WORLD"]
run_dir = os.environ["RUN_DIR"]

x_min = float(defaults.get("uav_grid_x_min", -15.0))
x_max = float(defaults.get("uav_grid_x_max",  15.0))
x_steps = int(defaults.get("uav_grid_x_steps", 6))
y_min = float(defaults.get("uav_grid_y_min", -25.0))
y_max = float(defaults.get("uav_grid_y_max",  25.0))
y_steps = int(defaults.get("uav_grid_y_steps", 10))
z = float(defaults.get("uav_altitude_m", 10.0))
wait_s = float(defaults.get("uav_wait_s", 1.0))

cmd = [
    "python3", os.path.expanduser("~/halmstad_ws/scripts/uav_setpose_sweep.py"),
    "--world", world,
    "--uav", uav,
    "--x_min", str(x_min), "--x_max", str(x_max), "--x_steps", str(x_steps),
    "--y_min", str(y_min), "--y_max", str(y_max), "--y_steps", str(y_steps),
    "--z", str(z),
    "--yaw", "0.0",
    "--wait", str(wait_s),
    "--log_csv", str(Path(run_dir) / "uav_setpoints.csv"),
]

print("[run_round] UAV sweep:", " ".join(cmd))
rc = subprocess.call(cmd)
raise SystemExit(rc)
PY

  pub_evt "UAV_PHASE_END"
fi



# UGV movement
pub_evt "UGV_PHASE_START"

python3 - << 'PY'
import os
import time
import yaml
from pathlib import Path
import math

import rclpy
from geometry_msgs.msg import TwistStamped

defaults_path = Path(os.path.expanduser("~/halmstad_ws/src/lrs_halmstad/resource/experiment_defaults.yaml"))
defaults = {}
if defaults_path.exists():
    defaults = yaml.safe_load(defaults_path.read_text()) or {}

speed_fwd = float(defaults.get("ugv_forward_speed", 0.5))
t_fwd = float(defaults.get("ugv_forward_time_s", 4.0))
speed_turn = float(defaults.get("ugv_turn_speed", 0.5))
t_turn = float(defaults.get("ugv_turn_time_s", 3.14))
cycles = int(defaults.get("ugv_cycles", 10))
pub_rate_hz = float(os.environ.get("UGV_CMD_PUB_RATE_HZ", "10.0"))
startup_pad_s = float(os.environ.get("UGV_CMD_STARTUP_PAD_S", "0.0"))
motion_profile = str(os.environ.get("UGV_MOTION_PROFILE", "default")).strip().lower()
turn_pattern = str(os.environ.get("UGV_TURN_PATTERN", "same")).strip().lower()
variation_enable = str(os.environ.get("UGV_VARIATION_ENABLE", "true")).strip().lower() not in ("0", "false", "no")
variation_amp = float(os.environ.get("UGV_VARIATION_AMPLITUDE", "0.20"))
pause_every_n = int(os.environ.get("UGV_PAUSE_EVERY_N", "0"))
pause_time_s = float(os.environ.get("UGV_PAUSE_TIME_S", "0.0"))

if motion_profile in ("fast_wide", "far_fast", "wide_fast"):
    speed_fwd *= 1.8
    t_fwd *= 1.6
    speed_turn *= 1.2
    t_turn *= 0.9
    cycles = max(cycles, 12)
elif motion_profile in ("fast", "speed"):
    speed_fwd *= 1.8
    speed_turn *= 1.2
elif motion_profile in ("wide", "far"):
    t_fwd *= 1.8
    cycles = max(cycles, 12)

if os.environ.get("UGV_FORWARD_SPEED"):
    speed_fwd = float(os.environ["UGV_FORWARD_SPEED"])
if os.environ.get("UGV_FORWARD_TIME_S"):
    t_fwd = float(os.environ["UGV_FORWARD_TIME_S"])
if os.environ.get("UGV_TURN_SPEED"):
    speed_turn = float(os.environ["UGV_TURN_SPEED"])
if os.environ.get("UGV_TURN_TIME_S"):
    t_turn = float(os.environ["UGV_TURN_TIME_S"])
if os.environ.get("UGV_CYCLES"):
    cycles = int(os.environ["UGV_CYCLES"])

if pub_rate_hz <= 0.0:
    pub_rate_hz = 10.0
if startup_pad_s < 0.0:
    startup_pad_s = 0.0
if variation_amp < 0.0:
    variation_amp = 0.0
if variation_amp > 0.45:
    variation_amp = 0.45
if pause_every_n < 0:
    pause_every_n = 0
if pause_time_s < 0.0:
    pause_time_s = 0.0

print(
    f"[run_round] UGV motion: profile={motion_profile} turn_pattern={turn_pattern} "
    f"v_fwd={speed_fwd} t_fwd={t_fwd} v_turn={speed_turn} t_turn={t_turn} "
    f"cycles={cycles} pub_rate_hz={pub_rate_hz} startup_pad_s={startup_pad_s} "
    f"variation_enable={variation_enable} variation_amp={variation_amp} "
    f"pause_every_n={pause_every_n} pause_time_s={pause_time_s}"
)

topic = os.environ.get("UGV_CMD_VEL_TOPIC", "/a201_0000/cmd_vel")

segments = []
for i in range(cycles):
    if variation_enable:
        # Long-period deterministic modulation (not a short repeating lookup table)
        # keeps the path reproducible but less obviously "the same loop" in the GUI.
        fwd_var = 0.65 * math.sin(0.73 * i) + 0.35 * math.sin(0.19 * i + 1.10)
        turn_var = 0.60 * math.cos(0.61 * i + 0.40) + 0.40 * math.sin(0.27 * i + 0.90)
        fs = max(0.40, 1.0 + variation_amp * fwd_var)
        ts = max(0.40, 1.0 + variation_amp * turn_var)
    else:
        fs = 1.0
        ts = 1.0

    segments.append(("fwd", speed_fwd * fs, 0.0, t_fwd))
    turn_sign = -1.0 if (turn_pattern in ("alternate", "zigzag", "zig-zag") and (i % 2)) else 1.0
    segments.append(("turn", 0.0, turn_sign * speed_turn * ts, t_turn))
    if pause_every_n > 0 and (i + 1) % pause_every_n == 0 and pause_time_s > 0.0:
        segments.append(("pause", 0.0, 0.0, pause_time_s))

est_motion_s = sum(float(secs) for _, _, _, secs in segments)
print(f"[run_round] UGV motion estimated duration: ~{est_motion_s:.1f}s (+startup_pad_s={startup_pad_s:.1f}s)")

def publish_cmd(pub, node, vx, wz):
    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.twist.linear.x = float(vx)
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = float(wz)
    pub.publish(msg)

rclpy.init()
node = rclpy.create_node("run_round_ugv_driver")
pub = node.create_publisher(TwistStamped, topic, 10)

dt = 1.0 / pub_rate_hz

# One-time discovery warmup (publish zeros) so short segments still produce motion.
if startup_pad_s > 0.0:
    t_end = time.monotonic() + startup_pad_s
    while time.monotonic() < t_end:
        publish_cmd(pub, node, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(dt)

for name, vx, wz, secs in segments:
    t_end = time.monotonic() + max(0.0, float(secs))
    while time.monotonic() < t_end:
        publish_cmd(pub, node, vx, wz)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(dt)

# Stop command at the end.
for _ in range(max(3, int(math.ceil(0.3 / dt)))):
    publish_cmd(pub, node, 0.0, 0.0)
    rclpy.spin_once(node, timeout_sec=0.0)
    time.sleep(dt)

node.destroy_node()
rclpy.shutdown()
PY

#
pub_evt "UGV_PHASE_END"

pub_evt "ROUND_END"

# Stop background nodes (also handled by EXIT trap, but we stop here to finish cleanly)
if [ -n "$FOLLOW_PID" ]; then
  echo "[run_round] Stopping follow_uav (pid=$FOLLOW_PID)"
  kill "$FOLLOW_PID" 2>/dev/null || true
  wait "$FOLLOW_PID" 2>/dev/null || true
  FOLLOW_PID=""
fi

if [ -n "$EST_PID" ]; then
  echo "[run_round] Stopping leader_estimator (pid=$EST_PID)"
  kill "$EST_PID" 2>/dev/null || true
  wait "$EST_PID" 2>/dev/null || true
  EST_PID=""
fi

# Stop bag
if [ -n "$BAG_PID" ]; then
  kill "$BAG_PID" 2>/dev/null || true
  wait "$BAG_PID" 2>/dev/null || true
  BAG_PID=""
fi

echo "[run_round] Done. Bag info:"
ros2 bag info "$RUN_DIR/bag" | head -n 60 || true

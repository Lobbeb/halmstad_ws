#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SCENARIO=""
TIMEOUT_S="10"
OUTPUT_ROOT="bags/validation/support_hazards"
RECORD=false
DRY_RUN=false
VALIDATION_DOMAIN_ID="${ROS_DOMAIN_ID:-47}"
RUN_STAMP="$(date +%Y%m%dT%H%M%S)-$$"
ORIGINAL_ARGS=("$@")
PIDS=()

usage() {
  cat <<'EOF'
Usage: ./run.sh validate_support_hazards \
  scenario:=task6_confirmation|task6_conflict|task7_quality|task7_stale|task7_expiry \
  [timeout_s:=10] [record:=true|false] \
  [out:=bags/validation/support_hazards] \
  [validation_domain_id:=47] [dry_run:=true|false]

Runs only bounded non-simulation ROS nodes. dji2 is started only by the explicit
Task 6/7 two-source scenarios. Evidence is written below a new timestamped folder;
record:=true adds the existing image-free support_hazard rosbag profile.
EOF
}

for arg in "$@"; do
  case "$arg" in
    help|--help|-h)
      usage
      exit 0
      ;;
    scenario:=*)
      SCENARIO="${arg#scenario:=}"
      ;;
    timeout_s:=*)
      TIMEOUT_S="${arg#timeout_s:=}"
      ;;
    record:=*)
      RECORD="${arg#record:=}"
      ;;
    out:=*)
      OUTPUT_ROOT="${arg#out:=}"
      ;;
    validation_domain_id:=*)
      VALIDATION_DOMAIN_ID="${arg#validation_domain_id:=}"
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

case "$SCENARIO" in
  task6_confirmation|task6_conflict|task7_quality|task7_stale|task7_expiry)
    ;;
  *)
    echo "A supported scenario:=... value is required." >&2
    usage >&2
    exit 2
    ;;
esac
case "$RECORD" in
  true|false) ;;
  *) echo "record must be true or false." >&2; exit 2 ;;
esac
case "$DRY_RUN" in
  true|false) ;;
  *) echo "dry_run must be true or false." >&2; exit 2 ;;
esac
if ! [[ "$TIMEOUT_S" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "timeout_s must be a positive number." >&2
  exit 2
fi
if ! [[ "$VALIDATION_DOMAIN_ID" =~ ^[0-9]+$ ]]; then
  echo "validation_domain_id must be a non-negative integer." >&2
  exit 2
fi

case "$OUTPUT_ROOT" in
  /*) OUTPUT_ROOT_ABS="$OUTPUT_ROOT" ;;
  *) OUTPUT_ROOT_ABS="$WS_ROOT/$OUTPUT_ROOT" ;;
esac
EVIDENCE_DIR="$OUTPUT_ROOT_ABS/${RUN_STAMP}_${SCENARIO}"

shell_join() {
  local output=""
  local part=""
  for part in "$@"; do
    printf -v output '%s%q ' "$output" "$part"
  done
  printf '%s' "${output% }"
}

json_escape() {
  local value="$1"
  value="${value//\/\\}"
  value="${value//\"/\\\"}"
  value="${value//$'\n'/\\n}"
  value="${value//$'\r'/\\r}"
  value="${value//$'\t'/\\t}"
  printf '%s' "$value"
}

write_manifest() {
  local status="$1"
  local result="$2"
  local git_head git_branch git_dirty invocation
  git_head="$(git -C "$WS_ROOT" rev-parse HEAD)"
  git_branch="$(git -C "$WS_ROOT" branch --show-current)"
  git_dirty=false
  if [ -n "$(git -C "$WS_ROOT" status --porcelain=v1)" ]; then
    git_dirty=true
  fi
  invocation="$(shell_join ./run.sh validate_support_hazards "${ORIGINAL_ARGS[@]}")"
  {
    printf '{\n'
    printf '  "schema_version": 1,\n'
    printf '  "prototype_scope": "post-thesis_future-work_support-chain",\n'
    printf '  "world": "baylands",\n'
    printf '  "scenario": "%s",\n' "$(json_escape "$SCENARIO")"
    printf '  "status": "%s",\n' "$(json_escape "$status")"
    printf '  "result": "%s",\n' "$(json_escape "$result")"
    printf '  "timeout_s": %s,\n' "$TIMEOUT_S"
    printf '  "record": %s,\n' "$RECORD"
    printf '  "validation_domain_id": %s,\n' "$VALIDATION_DOMAIN_ID"
    printf '  "dji2_opt_in": %s,\n' "$REQUIRE_DJI2"
    printf '  "aerial_costmap_enabled": false,\n'
    printf '  "git_branch": "%s",\n' "$(json_escape "$git_branch")"
    printf '  "git_commit": "%s",\n' "$(json_escape "$git_head")"
    printf '  "git_dirty": %s,\n' "$git_dirty"
    printf '  "invocation": "%s",\n' "$(json_escape "$invocation")"
    printf '  "fusion_command": "%s",\n' "$(json_escape "$(shell_join "${FUSION_CMD[@]}")")"
    printf '  "forwarder_command": "%s",\n' "$(json_escape "$(shell_join "${FORWARD_CMD[@]}")")"
    printf '  "dji1_command": "%s",\n' "$(json_escape "$(shell_join "${DJI1_CMD[@]}")")"
    printf '  "dji2_command": "%s",\n' "$(json_escape "$(shell_join "${DJI2_CMD[@]}")")"
    printf '  "verifier_command": "%s",\n' "$(json_escape "$(shell_join "${VERIFY_CMD[@]}")")"
    printf '  "limitations": [\n'
    printf '    "Synthetic contract evidence is not detector-accuracy evidence",\n'
    printf '    "No simulation or closed-loop navigation result is produced",\n'
    printf '    "Configured source quality is not a measured live network metric"\n'
    printf '  ],\n'
    printf '  "updated_at": "%s"\n' "$(date -Is)"
    printf '}\n'
  } > "$EVIDENCE_DIR/manifest.json"
}

cleanup() {
  local pid=""
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  PIDS=()
}

start_logged() {
  local label="$1"
  shift
  "$@" > "$EVIDENCE_DIR/${label}.log" 2>&1 &
  PIDS+=("$!")
}

export ROS_DOMAIN_ID="$VALIDATION_DOMAIN_ID"
export ROS2CLI_NO_DAEMON=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

FUSION_CMD=(
  ros2 run lrs_halmstad support_hazard_fusion --ros-args
  -p use_sim_time:=false
  -p stale_timeout_s:=0.75
  -p max_source_age_s:=0.75
  -p source_timeout_s:=0.75
)
FORWARD_CMD=(
  ros2 run lrs_halmstad dji0_to_ugv_forwarder --ros-args
  -p use_sim_time:=false
  -p hazard_forward_enable:=true
)
DJI1_CMD=(
  ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args
  -p use_sim_time:=false
  -p topic:=/coord/support/dji1/aerial_hazards
  -p source_uav:=dji1
  -p stable_track_id:=task8_dji1
  -p class_name:=hazard
  -p center_x:=4.0
  -p center_y:=5.0
  -p confidence:=0.9
  -p support_quality:=0.9
  -p publish_rate_hz:=5.0
  -p ttl_s:=2.0
)
DJI2_CMD=()
VERIFY_CMD=(
  ros2 run lrs_halmstad support_hazard_evidence live
  --timeout-s "$TIMEOUT_S"
  --max-age-s 1.0
  --output "$EVIDENCE_DIR"
)
REQUIRE_DJI2=false

case "$SCENARIO" in
  task6_confirmation)
    REQUIRE_DJI2=true
    FUSION_CMD+=( -p dji2_enable:=true )
    DJI2_CMD=(
      ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args
      -p use_sim_time:=false
      -p topic:=/coord/support/dji2/aerial_hazards
      -p source_uav:=dji2
      -p stable_track_id:=task8_dji2
      -p class_name:=hazard
      -p center_x:=4.2
      -p center_y:=5.1
      -p confidence:=0.9
      -p support_quality:=0.9
      -p start_delay_s:=1.0
      -p publish_rate_hz:=5.0
      -p ttl_s:=2.0
    )
    VERIFY_CMD+=(
      --require-dji2
      --expected-state CONFIRMED
      --expected-sources dji1,dji2
      --require-confirmation-promotion
    )
    ;;
  task6_conflict)
    REQUIRE_DJI2=true
    FUSION_CMD+=( -p dji2_enable:=true )
    DJI2_CMD=(
      ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args
      -p use_sim_time:=false
      -p topic:=/coord/support/dji2/aerial_hazards
      -p source_uav:=dji2
      -p stable_track_id:=task8_conflict_dji2
      -p class_name:=vehicle
      -p center_x:=4.1
      -p center_y:=5.0
      -p publish_rate_hz:=5.0
      -p ttl_s:=2.0
    )
    VERIFY_CMD+=(
      --require-dji2
      --expected-state CONFLICT
      --minimum-hazard-count 2
      --require-conflict
    )
    ;;
  task7_quality)
    REQUIRE_DJI2=true
    FUSION_CMD+=(
      -p dji2_enable:=true
      -p quality_weight_confidence:=0.1
      -p quality_weight_freshness:=0.1
      -p quality_weight_uncertainty:=0.1
      -p quality_weight_view:=0.1
      -p quality_weight_communication:=0.6
      -p dji1_communication_quality:=1.0
      -p dji2_communication_quality:=0.0
    )
    DJI2_CMD=(
      ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args
      -p use_sim_time:=false
      -p topic:=/coord/support/dji2/aerial_hazards
      -p source_uav:=dji2
      -p stable_track_id:=task8_quality_dji2
      -p class_name:=hazard
      -p center_x:=4.2
      -p center_y:=5.1
      -p confidence:=0.9
      -p support_quality:=0.9
      -p publish_rate_hz:=5.0
      -p ttl_s:=2.0
    )
    VERIFY_CMD+=(
      --require-dji2
      --expected-state CONFIRMED
      --expected-sources dji1,dji2
      --expected-selected-source dji1
    )
    ;;
  task7_stale)
    REQUIRE_DJI2=true
    FUSION_CMD+=( -p dji2_enable:=true )
    DJI2_CMD=(
      ros2 run lrs_halmstad synthetic_hazard_publisher --ros-args
      -p use_sim_time:=false
      -p topic:=/coord/support/dji2/aerial_hazards
      -p source_uav:=dji2
      -p stable_track_id:=task8_stale_dji2
      -p class_name:=hazard
      -p center_x:=4.2
      -p center_y:=5.1
      -p stamp_offset_s:=-2.0
      -p publish_rate_hz:=5.0
      -p ttl_s:=2.0
    )
    VERIFY_CMD+=(
      --require-dji2
      --expected-state CONFIRMED
      --expected-sources dji1
      --expected-selected-source dji1
    )
    ;;
  task7_expiry)
    DJI1_CMD+=(
      -p active_duration_s:=1.5
      -p publish_empty_after_active_duration:=true
    )
    VERIFY_CMD+=(--require-expiry)
    ;;
esac

RECORD_CMD=()
if [ "$RECORD" = true ]; then
  RECORD_CMD=(
    "$WS_ROOT/run.sh" record_experiment baylands
    mode:=follow
    profile:=support_hazard
    tag:="task8_${SCENARIO}"
    out:="$EVIDENCE_DIR/recording"
  )
fi

if [ "$DRY_RUN" = true ]; then
  echo "Scenario: $SCENARIO"
  echo "World: baylands"
  echo "Evidence dir: $EVIDENCE_DIR"
  echo "dji2 opt-in: $REQUIRE_DJI2"
  echo "Aerial costmap: false"
  echo "Fusion: $(shell_join "${FUSION_CMD[@]}")"
  echo "Forwarder: $(shell_join "${FORWARD_CMD[@]}")"
  echo "Verifier: $(shell_join "${VERIFY_CMD[@]}")"
  echo "dji1: $(shell_join "${DJI1_CMD[@]}")"
  if [ "${#DJI2_CMD[@]}" -gt 0 ]; then
    echo "dji2: $(shell_join "${DJI2_CMD[@]}")"
  fi
  if [ "${#RECORD_CMD[@]}" -gt 0 ]; then
    echo "Recorder: $(shell_join "${RECORD_CMD[@]}")"
  fi
  exit 0
fi

if [ -e "$EVIDENCE_DIR" ]; then
  echo "Refusing to overwrite existing evidence directory: $EVIDENCE_DIR" >&2
  exit 1
fi
mkdir -p "$EVIDENCE_DIR"
trap cleanup INT TERM EXIT
write_manifest running not_finished

start_logged fusion "${FUSION_CMD[@]}"
start_logged forwarder "${FORWARD_CMD[@]}"
if [ "${#RECORD_CMD[@]}" -gt 0 ]; then
  start_logged recorder "${RECORD_CMD[@]}"
fi
sleep 0.5

"${VERIFY_CMD[@]}" > "$EVIDENCE_DIR/verifier.log" 2>&1 &
VERIFIER_PID="$!"
PIDS+=("$VERIFIER_PID")
sleep 0.25
start_logged dji1 "${DJI1_CMD[@]}"
if [ "${#DJI2_CMD[@]}" -gt 0 ]; then
  start_logged dji2 "${DJI2_CMD[@]}"
fi

set +e
wait "$VERIFIER_PID"
verifier_status="$?"
set -e
cleanup
trap - INT TERM EXIT

if [ "$verifier_status" -eq 0 ]; then
  write_manifest completed verifier_passed
  echo "Validation passed: $SCENARIO"
  echo "Evidence: $EVIDENCE_DIR"
  exit 0
fi

write_manifest failed verifier_failed
echo "Validation failed: $SCENARIO" >&2
echo "Evidence: $EVIDENCE_DIR" >&2
exit "$verifier_status"

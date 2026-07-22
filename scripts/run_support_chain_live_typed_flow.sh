#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DJI2_ENABLE=false
AERIAL_SUPPORT_LAYER_ENABLE=false
FORWARD_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./run.sh support_chain_live_typed_flow \
  [dji2_enable:=true|false] \
  [aerial_support_layer_enable:=true|false] \
  [record:=true|false] [record_tag:=name] [dry_run:=true|false] [other operator args]

Runs the validated headless Baylands dji1 typed-flow profile with a fixed -60 degree
camera pitch. dji2 and the aerial costmap are disabled unless explicitly enabled.
Profile-owned world, mode, projector, camera-scan, and GUI arguments cannot be overridden.
EOF
}

validate_boolean() {
  local label="$1"
  local value="$2"
  case "$value" in
    true|false) ;;
    *)
      echo "$label must be true or false." >&2
      exit 2
      ;;
  esac
}

for arg in "$@"; do
  case "$arg" in
    help|--help|-h)
      usage
      exit 0
      ;;
    dji2_enable:=*)
      DJI2_ENABLE="${arg#dji2_enable:=}"
      ;;
    aerial_support_layer_enable:=*)
      AERIAL_SUPPORT_LAYER_ENABLE="${arg#aerial_support_layer_enable:=}"
      ;;
    baylands|world:=*|world=*|mode:=*|hazard_projector_enable:=*|support_camera_scan_enable:=*|support_camera_scan_uavs:=*|support_camera_scan_yaw_center_deg:=*|support_camera_scan_yaw_amplitude_deg:=*|support_camera_scan_pitch_deg:=*|support_camera_scan_pitch_amplitude_deg:=*|gui:=*)
      echo "The live typed-flow profile owns this argument: $arg" >&2
      echo "Use ./run.sh tmux_support_chain directly for a custom workflow." >&2
      exit 2
      ;;
    *:=*)
      FORWARD_ARGS+=("$arg")
      ;;
    *)
      echo "Unknown positional argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

validate_boolean "dji2_enable" "$DJI2_ENABLE"
validate_boolean "aerial_support_layer_enable" "$AERIAL_SUPPORT_LAYER_ENABLE"

exec bash "$WS_ROOT/scripts/run_tmux_support_chain.sh" baylands \
  mode:=follow \
  hazard_projector_enable:=true \
  support_camera_scan_enable:=true \
  support_camera_scan_uavs:=dji1 \
  support_camera_scan_yaw_center_deg:=0.0 \
  support_camera_scan_yaw_amplitude_deg:=0.0 \
  support_camera_scan_pitch_deg:=-60.0 \
  support_camera_scan_pitch_amplitude_deg:=0.0 \
  dji2_enable:="$DJI2_ENABLE" \
  aerial_support_layer_enable:="$AERIAL_SUPPORT_LAYER_ENABLE" \
  gui:=false \
  tmux_attach:=false \
  record:=false \
  "${FORWARD_ARGS[@]}"

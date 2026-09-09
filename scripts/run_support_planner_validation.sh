#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SCENARIO="valid"
OUTPUT=""
RVIZ="false"
TIMEOUT_S="75.0"
NAMESPACE="a201_0000"
MAP="$WS_ROOT/maps/baylands.yaml"
PARAMS_FILE="$WS_ROOT/src/lrs_halmstad/config/nav2_baylands_large_map.yaml"
DRY_RUN="false"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh support_planner_validation [scenario:=NAME] [option:=value ...]

Scenarios:
  map_check, baseline, valid, clearing, off_route, low_confidence, stale,
  layer_disabled

Options:
  output:=DIR       Evidence directory; defaults to evidence/support_planner_corrected/<scenario>.
  rviz:=true|false  Start optional RViz with the planner stack. Default: false.
  timeout_s:=S      Bounded runtime evidence timeout. Default: 75.0.
  namespace:=NAME   Nav2 namespace. Default: a201_0000.
  map:=PATH         Saved map YAML.
  params_file:=PATH Nav2 parameter YAML.
  dry_run:=true     Print the resolved command without starting ROS.

The runtime command stays in the foreground and exits after the bounded evidence
tool finishes. Press Ctrl-C to stop early; launch stops only processes it started.
EOF
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    scenario:=*) SCENARIO="${arg#scenario:=}" ;;
    output:=*) OUTPUT="${arg#output:=}" ;;
    rviz:=*) RVIZ="${arg#rviz:=}" ;;
    timeout_s:=*) TIMEOUT_S="${arg#timeout_s:=}" ;;
    namespace:=*) NAMESPACE="${arg#namespace:=}" ;;
    map:=*) MAP="${arg#map:=}" ;;
    params_file:=*) PARAMS_FILE="${arg#params_file:=}" ;;
    dry_run:=*) DRY_RUN="${arg#dry_run:=}" ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$SCENARIO" in
  map_check|baseline|valid|clearing|off_route|low_confidence|stale|layer_disabled) ;;
  *) echo "Invalid scenario: $SCENARIO" >&2; exit 2 ;;
esac
case "$RVIZ:$DRY_RUN" in
  true:true|true:false|false:true|false:false) ;;
  *) echo "rviz and dry_run must be true or false" >&2; exit 2 ;;
esac
if [ ! -f "$MAP" ]; then
  echo "Map YAML not found: $MAP" >&2
  exit 2
fi
if [ ! -f "$PARAMS_FILE" ]; then
  echo "Nav2 params not found: $PARAMS_FILE" >&2
  exit 2
fi
if [ -z "$OUTPUT" ]; then
  OUTPUT="$WS_ROOT/evidence/support_planner_corrected/$SCENARIO"
fi

if [ "$SCENARIO" = "map_check" ]; then
  COMMAND=(
    ros2 run lrs_halmstad support_hazard_evidence map-check
    --map "$MAP"
    --nav2-config "$PARAMS_FILE"
    --output "$OUTPUT"
    --start-x -71.39979517989994 --start-y 205.6352521524901
    --goal-x -72.53159610937462 --goal-y 185.3861158209806
    --hazard-x -72.0 --hazard-y 195.5
    --hazard-size-x 2.0 --hazard-size-y 2.0
    --variance-x 0.25 --variance-y 0.25 --covariance-sigma-scale 2.0
  )
else
  COMMAND=(
    ros2 launch lrs_halmstad support_planner_validation.launch.py
    "scenario:=$SCENARIO"
    "output:=$OUTPUT"
    "rviz:=$RVIZ"
    "timeout_s:=$TIMEOUT_S"
    "namespace:=$NAMESPACE"
    "map:=$MAP"
    "params_file:=$PARAMS_FILE"
  )
fi

printf '[support_planner_validation] scenario=%s output=%s time_model=wall\n' \
  "$SCENARIO" "$OUTPUT"
if [ "$DRY_RUN" = "true" ]; then
  printf '[support_planner_validation] '
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u
cd "$WS_ROOT"
exec "${COMMAND[@]}"

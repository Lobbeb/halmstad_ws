#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PY_SCRIPT="$SCRIPT_DIR/mark_map_points.py"

REFERENCE=""
IMAGE=""
MANIFEST="$WS_ROOT/maps/baylands_step_manifest.csv"
OUTPUT=""
STATE_PREFIX=""
RADIUS="10"
WORLD_BOUNDS=""
CONTROL_POINTS_FILE=""

usage() {
  cat <<'EOF'
Usage: ./run.sh mark_map_points [reference] [args...]

Examples:
  ./run.sh mark_map_points baylands_step_3
  ./run.sh mark_map_points reference:=baylands_step_3 image:=/path/to/screenshot.png
  ./run.sh mark_map_points image:=/path/to/screenshot.png world_bounds:=-50,50,200,-50
  ./run.sh mark_map_points image:=/path/to/screenshot.png control_points_file:=/path/to/control_points.csv
  ./run.sh mark_map_points baylands_merged_art.yaml manifest:=/home/ruben/halmstad_ws/maps/baylands_step_manifest.csv
EOF
}

resolve_reference() {
  local ref="$1"

  if [ -f "$ref" ]; then
    printf '%s\n' "$ref"
    return 0
  fi

  if [ -f "$WS_ROOT/maps/$ref" ]; then
    printf '%s\n' "$WS_ROOT/maps/$ref"
    return 0
  fi

  if [ -f "$WS_ROOT/maps/${ref}.yaml" ]; then
    printf '%s\n' "$WS_ROOT/maps/${ref}.yaml"
    return 0
  fi

  return 1
}

if [ "$#" -gt 0 ] && [[ "$1" != *=* ]] && [[ "$1" != *":="* ]]; then
  REFERENCE="$1"
  shift
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    help|-h|--help)
      usage
      exit 0
      ;;
    reference:=*)
      REFERENCE="${1#reference:=}"
      ;;
    image:=*)
      IMAGE="${1#image:=}"
      ;;
    world_bounds:=*)
      WORLD_BOUNDS="${1#world_bounds:=}"
      ;;
    control_points_file:=*)
      CONTROL_POINTS_FILE="${1#control_points_file:=}"
      ;;
    manifest:=*)
      MANIFEST="${1#manifest:=}"
      ;;
    output:=*)
      OUTPUT="${1#output:=}"
      ;;
    state_prefix:=*)
      STATE_PREFIX="${1#state_prefix:=}"
      ;;
    radius:=*)
      RADIUS="${1#radius:=}"
      ;;
    *)
      echo "[run_mark_map_points] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [ -z "$REFERENCE" ] && [ -z "$WORLD_BOUNDS" ] && [ -z "$CONTROL_POINTS_FILE" ]; then
  echo "[run_mark_map_points] Please provide a reference map YAML/map name, world_bounds:=x_min,y_min,x_max,y_max, or control_points_file:=/path/to/file.csv." >&2
  usage >&2
  exit 2
fi

REFERENCE_PATH=""
if [ -n "$REFERENCE" ]; then
  if ! REFERENCE_PATH="$(resolve_reference "$REFERENCE")"; then
    echo "[run_mark_map_points] Could not resolve reference '$REFERENCE'" >&2
    exit 1
  fi
fi

if [ -z "$REFERENCE_PATH" ] && [ -z "$IMAGE" ]; then
  echo "[run_mark_map_points] Screenshot mode requires image:=/path/to/image" >&2
  exit 2
fi

if [ -z "$OUTPUT" ]; then
  if [ -n "$REFERENCE_PATH" ]; then
    OUTPUT="${REFERENCE_PATH%.*}_marked.png"
  else
    OUTPUT="${IMAGE%.*}_marked.png"
  fi
fi

PY_ARGS=(
  --manifest "$MANIFEST"
  --output "$OUTPUT"
  --radius "$RADIUS"
)

if [ -n "$REFERENCE_PATH" ]; then
  PY_ARGS+=(--reference-yaml "$REFERENCE_PATH")
fi

if [ -n "$IMAGE" ]; then
  PY_ARGS+=(--image "$IMAGE")
fi

if [ -n "$WORLD_BOUNDS" ]; then
  PY_ARGS+=(--world-bounds "$WORLD_BOUNDS")
fi

if [ -n "$CONTROL_POINTS_FILE" ]; then
  PY_ARGS+=(--control-points-file "$CONTROL_POINTS_FILE")
fi

if [ -n "$STATE_PREFIX" ]; then
  PY_ARGS+=(--state-prefix "$STATE_PREFIX")
fi

python3 "$PY_SCRIPT" "${PY_ARGS[@]}"

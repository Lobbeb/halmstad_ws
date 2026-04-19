#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

SOURCE_YAML=""
OUTPUT_YAML=""
FACTOR="4"

while [ "$#" -gt 0 ]; do
  case "$1" in
    source:=*)
      SOURCE_YAML="${1#source:=}"
      ;;
    output:=*)
      OUTPUT_YAML="${1#output:=}"
      ;;
    factor:=*)
      FACTOR="${1#factor:=}"
      ;;
    *)
      echo "[run_make_nav_map] Unknown argument: $1" >&2
      echo "Usage: ./run.sh make_nav_map source:=/abs/path/map.yaml [output:=/abs/path/output.yaml] [factor:=4]" >&2
      exit 1
      ;;
  esac
  shift
done

if [ -z "$SOURCE_YAML" ]; then
  echo "[run_make_nav_map] source:=... is required" >&2
  echo "Usage: ./run.sh make_nav_map source:=/abs/path/map.yaml [output:=/abs/path/output.yaml] [factor:=4]" >&2
  exit 1
fi

if [[ "$SOURCE_YAML" != /* ]]; then
  SOURCE_YAML="$WS_ROOT/$SOURCE_YAML"
fi

if [ -n "$OUTPUT_YAML" ] && [[ "$OUTPUT_YAML" != /* ]]; then
  OUTPUT_YAML="$WS_ROOT/$OUTPUT_YAML"
fi

ARGS=(--source-yaml "$SOURCE_YAML" --factor "$FACTOR")
if [ -n "$OUTPUT_YAML" ]; then
  ARGS+=(--output-yaml "$OUTPUT_YAML")
fi

python3 "$SCRIPT_DIR/make_nav_map.py" "${ARGS[@]}"

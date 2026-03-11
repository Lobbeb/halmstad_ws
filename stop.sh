#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_WRAPPER_DIR="$SCRIPT_DIR/scripts"

list_commands() {
  local path base short

  shopt -s nullglob
  for path in "$SCRIPT_WRAPPER_DIR"/stop_*.sh; do
    base="$(basename "$path")"
    short="${base#stop_}"
    short="${short%.sh}"
    printf '%s\n' "$short"
  done
  shopt -u nullglob
}

print_usage() {
  cat <<EOF
Usage: ./stop.sh <command> [args...]

Available commands:
$(list_commands | sed 's/^/  - /')

You can also call the full script name, for example:
  ./stop.sh stop_tmux_1to1
EOF
}

resolve_script_path() {
  local command="$1"
  local candidate=""

  if [[ "$command" == *.sh ]]; then
    candidate="$SCRIPT_WRAPPER_DIR/$command"
  elif [[ "$command" == stop_* ]]; then
    candidate="$SCRIPT_WRAPPER_DIR/$command.sh"
    if [[ ! -x "$candidate" ]]; then
      candidate="$SCRIPT_WRAPPER_DIR/$command"
    fi
  else
    candidate="$SCRIPT_WRAPPER_DIR/stop_${command}.sh"
  fi

  if [[ -x "$candidate" ]]; then
    printf '%s\n' "$candidate"
    return 0
  fi

  return 1
}

if [[ $# -eq 0 ]]; then
  print_usage
  exit 1
fi

case "$1" in
  help|-h|--help|list)
    print_usage
    exit 0
    ;;
esac

if ! script_path="$(resolve_script_path "$1")"; then
  printf 'Unknown stop command: %s\n\n' "$1" >&2
  print_usage >&2
  exit 1
fi

shift
exec "$script_path" "$@"

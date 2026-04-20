#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_SCRIPTS_DIR="$SCRIPT_DIR/scripts"

collect_script_paths() {
  local prefix="$1"
  find "$RUN_SCRIPTS_DIR" -type f -name "${prefix}*.sh" -print | sort
}

list_commands() {
  local path base short

  while IFS= read -r path; do
    [ -n "$path" ] || continue
    base="$(basename "$path")"
    short="${base#run_}"
    short="${short%.sh}"
    printf '%s\n' "$short"
  done < <(collect_script_paths "run_")
}

print_usage() {
  cat <<EOF
Usage: ./run.sh <command> [args...]

Available commands:
$(list_commands | sed 's/^/  - /')

You can also call the full script name, for example:
  ./run.sh run_follow_control

Scripts may live in subfolders under scripts/, for example:
  ./run.sh slam_resume
EOF
}

find_unique_script_by_basename() {
  local basename="$1"
  local candidate=""
  local matches=()

  while IFS= read -r candidate; do
    [ -n "$candidate" ] || continue
    matches+=("$candidate")
  done < <(find "$RUN_SCRIPTS_DIR" -type f -name "$basename" -print | sort)

  case "${#matches[@]}" in
    1)
      printf '%s\n' "${matches[0]}"
      return 0
      ;;
    0)
      return 1
      ;;
    *)
      printf 'Ambiguous run command for %s:\n' "$basename" >&2
      printf '  - %s\n' "${matches[@]#$RUN_SCRIPTS_DIR/}" >&2
      return 2
      ;;
  esac
}

resolve_script_path() {
  local command="$1"
  local candidate=""
  local basename=""

  if [[ "$command" == */* ]]; then
    if [[ "$command" == *.sh ]]; then
      candidate="$RUN_SCRIPTS_DIR/$command"
    elif [[ "$command" == run_* ]]; then
      candidate="$RUN_SCRIPTS_DIR/$command.sh"
      if [[ ! -f "$candidate" ]]; then
        candidate="$RUN_SCRIPTS_DIR/$command"
      fi
    else
      candidate="$RUN_SCRIPTS_DIR/run_${command}.sh"
    fi

    if [[ -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  fi

  if [[ "$command" == *.sh ]]; then
    basename="$(basename "$command")"
  elif [[ "$command" == run_* ]]; then
    basename="${command}.sh"
  else
    basename="run_${command}.sh"
  fi

  find_unique_script_by_basename "$basename"
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
  printf 'Unknown run command: %s\n\n' "$1" >&2
  print_usage >&2
  exit 1
fi

shift
if [[ "$script_path" == *.sh ]] || [[ ! -x "$script_path" ]]; then
  exec bash "$script_path" "$@"
fi
exec "$script_path" "$@"

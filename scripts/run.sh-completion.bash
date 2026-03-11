_lrs_halmstad_run_dispatch_completion_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_lrs_halmstad_dispatch_complete() {
  local cur script_dir names path base short prefix

  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  script_dir="$_lrs_halmstad_run_dispatch_completion_dir"

  if [[ $COMP_CWORD -ne 1 ]]; then
    return 0
  fi

  names=("help" "list")

  case "${COMP_WORDS[0]}" in
    */stop.sh|stop.sh)
      prefix="stop_"
      ;;
    *)
      prefix="run_"
      ;;
  esac

  shopt -s nullglob
  for path in "$script_dir"/"${prefix}"*.sh; do
    base="$(basename "$path")"
    names+=("${base%.sh}")
    short="${base#${prefix}}"
    short="${short%.sh}"
    names+=("$short")
  done
  shopt -u nullglob

  COMPREPLY=($(compgen -W "${names[*]}" -- "$cur"))
}

complete -o bashdefault -o default -F _lrs_halmstad_dispatch_complete ./run.sh
complete -o bashdefault -o default -F _lrs_halmstad_dispatch_complete run.sh
complete -o bashdefault -o default -F _lrs_halmstad_dispatch_complete ./stop.sh
complete -o bashdefault -o default -F _lrs_halmstad_dispatch_complete stop.sh

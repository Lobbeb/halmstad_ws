_lrs_halmstad_shell_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$_lrs_halmstad_shell_dir/run.sh-completion.bash" ]; then
  . "$_lrs_halmstad_shell_dir/run.sh-completion.bash"
fi

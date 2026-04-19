#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${LRS_OMNET_PROJECT_ROOT:-$HOME/omnet_workspace/UAV_UGV}"
NETWORK="wifi"
UI="qtenv"
RESULT_DIR=""
DRY_RUN=false
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./run.sh omnet [network:=wifi|5g|lora] [ui:=cmdenv|qtenv] [project:=/path/UAV_UGV] [result_dir:=/path] [dry_run:=true|false] [extra OMNeT args...]

Examples:
  ./run.sh omnet
  ./run.sh omnet network:=5g
  ./run.sh omnet network:=lora ui:=qtenv
  ./run.sh omnet result_dir:=/tmp/omnet-results
EOF
}

shell_join() {
  local out=""
  local part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    network:=*|config:=*)
      NETWORK="${arg#*:=}"
      ;;
    ui:=*|env:=*)
      UI="${arg#*:=}"
      ;;
    project:=*|omnet_project:=*)
      PROJECT_ROOT="${arg#*:=}"
      ;;
    result_dir:=*|results:=*)
      RESULT_DIR="${arg#*:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    --*)
      EXTRA_ARGS+=("$arg")
      ;;
    -*)
      EXTRA_ARGS+=("$arg")
      ;;
    *:=*)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

case "$NETWORK" in
  wifi)
    CONFIG_NAME="Communication-GazeboBridge-WiFi"
    ;;
  5g)
    CONFIG_NAME="Communication-GazeboBridge-5G"
    ;;
  lora)
    CONFIG_NAME="Communication-GazeboBridge-LoRa"
    ;;
  *)
    echo "Invalid network: $NETWORK" >&2
    echo "Use network:=wifi, network:=5g, or network:=lora" >&2
    exit 2
    ;;
esac

case "$UI" in
  cmdenv)
    UI_NAME="Cmdenv"
    ;;
  qtenv)
    UI_NAME="Qtenv"
    ;;
  *)
    echo "Invalid ui: $UI" >&2
    echo "Use ui:=cmdenv or ui:=qtenv" >&2
    exit 2
    ;;
esac

case "$DRY_RUN" in
  true|false)
    ;;
  *)
    echo "Invalid dry_run option: $DRY_RUN" >&2
    echo "Use dry_run:=true or dry_run:=false" >&2
    exit 2
    ;;
esac

PROJECT_ROOT="$(cd -- "$PROJECT_ROOT" && pwd)"
WORKSPACE_DIR="$(cd -- "$PROJECT_ROOT/.." && pwd)"
SIM_BIN="$PROJECT_ROOT/UAV_UGV"
INI_FILE="$PROJECT_ROOT/omnetpp.ini"
OMNET_SETENV="${OMNETPP_SETENV:-$WORKSPACE_DIR/omnetpp-6.2.0/setenv}"

NED_PATH_PARTS=(
  "$PROJECT_ROOT/src"
  "$WORKSPACE_DIR/inet4.5/src"
)
if [ "$NETWORK" = "lora" ]; then
  NED_PATH_PARTS+=("$WORKSPACE_DIR/flora/src")
fi

NED_PATH="$(IFS=:; printf '%s' "${NED_PATH_PARTS[*]}")"

if [[ ! -f "$OMNET_SETENV" ]]; then
  echo "Missing OMNeT setenv: $OMNET_SETENV" >&2
  echo "Set OMNETPP_SETENV to your OMNeT setenv path if it lives elsewhere." >&2
  exit 1
fi

if [[ ! -x "$SIM_BIN" ]]; then
  echo "Missing executable: $SIM_BIN" >&2
  exit 1
fi

if [[ ! -f "$INI_FILE" ]]; then
  echo "Missing OMNeT ini file: $INI_FILE" >&2
  exit 1
fi

RESULT_DIR_ARGS=()
if [ -n "$RESULT_DIR" ]; then
  RESULT_DIR_ARGS+=("--result-dir=$RESULT_DIR")
else
  RESULT_DIR_ARGS+=("--result-dir=$PROJECT_ROOT/results")
  for arg in "${EXTRA_ARGS[@]}"; do
    if [[ "$arg" == --result-dir=* ]]; then
      RESULT_DIR_ARGS=()
      break
    fi
  done
fi

CMD=(
  "$SIM_BIN"
  -u "$UI_NAME"
  -f "$INI_FILE"
  -c "$CONFIG_NAME"
  -n "$NED_PATH"
  "${RESULT_DIR_ARGS[@]}"
  "${EXTRA_ARGS[@]}"
)

if [ "$DRY_RUN" = true ]; then
  echo "Project: $PROJECT_ROOT"
  echo "Config: $CONFIG_NAME"
  echo "UI: $UI_NAME"
  echo "setenv: $OMNET_SETENV"
  echo "Command: $(shell_join "${CMD[@]}")"
  exit 0
fi

if [ -n "$RESULT_DIR" ]; then
  mkdir -p "$RESULT_DIR"
else
  mkdir -p "$PROJECT_ROOT/results"
fi

# shellcheck source=/dev/null
set +u
source "$OMNET_SETENV"
set -u

cd "$PROJECT_ROOT"
exec "${CMD[@]}"

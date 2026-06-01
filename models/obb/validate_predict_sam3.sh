#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/ruben/halmstad_ws}"
MODEL_ROOT="${MODEL_ROOT:-${ROOT}/models/obb/mymodels}"
EXP="${EXP:-${ROOT}/models/obb/experiments/sam3_hybrid_full}"
MODEL="${MODEL:-${ROOT}/models/obb/mymodels/baylands-leader-v9-tuned-full.pt}"
RUN="${RUN:-$(basename "${MODEL%.pt}")}"
DEVICE="${DEVICE:-cpu}"
IMGSZ="${IMGSZ:-640}"
CONF="${CONF:-0.25}"
MAX_DET="${MAX_DET:-300}"
TOP_K="${TOP_K:-10}"
MAX_ISSUE="${MAX_ISSUE:-10}"
PYTHON="${PYTHON:-python3}"
VAL_AGNOSTIC_NMS="${VAL_AGNOSTIC_NMS:-false}"
VAL_COMPILE="${VAL_COMPILE:-false}"
DO_VAL="${DO_VAL:-1}"
DO_PREDICT="${DO_PREDICT:-1}"
DO_COMPARE="${DO_COMPARE:-1}"
DO_SCOREBOARD="${DO_SCOREBOARD:-}"
CLEAN_PREDICT="${CLEAN_PREDICT:-1}"
CLEAN_COMPARE="${CLEAN_COMPARE:-1}"
EVAL_DATASETS="${EVAL_DATASETS:-}"
BOARD_TAG="${BOARD_TAG:-}"
PREDICTIONS_ROOT="${PREDICTIONS_ROOT:-${EXP}/runs/predictions${BOARD_TAG:+_${BOARD_TAG}}}"
PREDICTION_ANALYSIS_ROOT="${PREDICTION_ANALYSIS_ROOT:-${EXP}/runs/prediction_analysis${BOARD_TAG:+_${BOARD_TAG}}}"
SCOREBOARD_OUT_DIR="${SCOREBOARD_OUT_DIR:-${EXP}/scoreboards${BOARD_TAG:+/${BOARD_TAG}}}"
SCOREBOARD_RUNS="${SCOREBOARD_RUNS:-}"

if [[ -z "$DO_SCOREBOARD" ]]; then
  if [[ "$DO_VAL" == "0" && "$DO_PREDICT" == "0" && "$DO_COMPARE" == "1" ]]; then
    DO_SCOREBOARD=0
  else
    DO_SCOREBOARD=1
  fi
fi

DEFAULT_DATASETS=(
  baylands_sam3_obb_super_75_15_10
  baylands_sam3_obb_generalization_art
  baylands_sam3_obb_generalization_playground
  baylands_sam3_obb_generalization_road_to_art
  baylands_sam3_obb_generalization_rotundan
)

if [[ -n "$EVAL_DATASETS" ]]; then
  # Space-separated list, e.g. EVAL_DATASETS="baylands_sam3_obb_super_75_15_10 baylands_sam3_obb_generalization_art"
  read -r -a DATASETS <<< "$EVAL_DATASETS"
else
  DATASETS=("${DEFAULT_DATASETS[@]}")
fi

dataset_yaml () {
  local dataset="$1"
  local root="${ROOT}/datasets/final/${dataset}"
  local yaml="/tmp/${dataset}_eval.yaml"

  cat > "$yaml" <<EOF
path: ${root}
train: images/train
val: images/val
test: images/test
nc: 1
names:
  0: ugv
EOF

  echo "$yaml"
}

legacy_dataset_name () {
  local dataset="$1"
  if [[ "$dataset" == baylands_sam3_obb_* ]]; then
    echo "baylands_${dataset#baylands_sam3_obb_}"
  else
    echo "$dataset"
  fi
}

sam3_dataset_name () {
  local dataset="$1"
  if [[ "$dataset" == baylands_* && "$dataset" != baylands_sam3_obb_* ]]; then
    echo "baylands_sam3_obb_${dataset#baylands_}"
  else
    echo "$dataset"
  fi
}

prediction_dir () {
  local dataset="$1"
  local candidates=(
    "$dataset"
    "$(legacy_dataset_name "$dataset")"
    "$(sam3_dataset_name "$dataset")"
  )
  local candidate
  local dir

  for candidate in "${candidates[@]}"; do
    dir="$PREDICTIONS_ROOT/$RUN/${candidate}_test_all"
    if [[ -d "$dir" ]]; then
      echo "$dir"
      return 0
    fi
  done

  echo "$PREDICTIONS_ROOT/$RUN/${dataset}_test_all"
}

run_val () {
  local dataset="$1"
  local split="$2"
  local name="$3"
  local yaml
  yaml="$(dataset_yaml "$dataset")"

  "$PYTHON" "${ROOT}/models/obb/run_obb_val.py" \
    --model "$MODEL" \
    --data "$yaml" \
    --dataset "$dataset" \
    --split "$split" \
    --imgsz "$IMGSZ" \
    --device "$DEVICE" \
    --project "$EXP/runs/val/$RUN" \
    --name "$name" \
    --agnostic-nms "$VAL_AGNOSTIC_NMS" \
    --compile "$VAL_COMPILE"
}

run_predict () {
  local dataset="$1"
  local out_dir="$PREDICTIONS_ROOT/$RUN/${dataset}_test_all"

  if [[ "$CLEAN_PREDICT" == "1" ]]; then
    rm -rf "$out_dir"
  fi

  "$PYTHON" "${ROOT}/models/obb/run_obb_predict.py" \
    --model "$MODEL" \
    --source "${ROOT}/datasets/final/${dataset}/images/test" \
    --imgsz "$IMGSZ" \
    --conf "$CONF" \
    --max-det "$MAX_DET" \
    --device "$DEVICE" \
    --save true \
    --save-txt true \
    --save-conf true \
    --project "$PREDICTIONS_ROOT/$RUN" \
    --name "${dataset}_test_all" \
    --exist-ok true
}

run_compare () {
  local dataset="$1"
  local out_dir="$PREDICTION_ANALYSIS_ROOT/$RUN/${dataset}"
  local pred_dir
  pred_dir="$(prediction_dir "$dataset")"

  if [[ "$CLEAN_COMPARE" == "1" ]]; then
    rm -rf "$out_dir"
  fi

  "$PYTHON" "${ROOT}/models/obb/compare_obb_predictions.py" \
    --dataset "${ROOT}/datasets/final/${dataset}" \
    --pred-dir "$pred_dir" \
    --split test \
    --out-dir "$out_dir" \
    --top-k "$TOP_K" \
    --max-issue "$MAX_ISSUE"
}

echo "Model:  $MODEL"
echo "Run:    $RUN"
echo "Device: $DEVICE"
echo "Exp:    $EXP"

if [[ "$DO_VAL" == "1" ]]; then
  echo
  echo "=== Validation ==="
  run_val baylands_sam3_obb_super_75_15_10 val baylands_sam3_obb_super_75_15_10_val

  for dataset in "${DATASETS[@]}"; do
    run_val "$dataset" test "${dataset}_test"
  done
fi

if [[ "$DO_PREDICT" == "1" ]]; then
  echo
  echo "=== Prediction ==="
  for dataset in "${DATASETS[@]}"; do
    run_predict "$dataset"
  done
fi

if [[ "$DO_COMPARE" == "1" ]]; then
  echo
  echo "=== Prediction analysis ==="
  for dataset in "${DATASETS[@]}"; do
    run_compare "$dataset"
  done
fi

if [[ "$DO_SCOREBOARD" == "1" ]]; then
  scoreboard_args=(
    --exp-root "$EXP"
    --prediction-analysis-root "$PREDICTION_ANALYSIS_ROOT"
    --out-dir "$SCOREBOARD_OUT_DIR"
  )
  if [[ -n "$SCOREBOARD_RUNS" ]]; then
    read -r -a scoreboard_runs <<< "$SCOREBOARD_RUNS"
    scoreboard_args+=(--runs "${scoreboard_runs[@]}")
  fi
  "$PYTHON" "${ROOT}/models/obb/build_sam3_full_scoreboard.py" \
    "${scoreboard_args[@]}"
fi

echo
echo "Done."
echo "Validation:          $EXP/runs/val/$RUN"
echo "Predictions:         $PREDICTIONS_ROOT/$RUN"
echo "Prediction analysis: $PREDICTION_ANALYSIS_ROOT/$RUN"
echo "Full scoreboard:     $SCOREBOARD_OUT_DIR/full_scoreboard.md"

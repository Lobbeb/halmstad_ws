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
TOP_K="${TOP_K:-20}"
MAX_ISSUE="${MAX_ISSUE:-50}"
PYTHON="${PYTHON:-python3}"
DO_VAL="${DO_VAL:-1}"
DO_PREDICT="${DO_PREDICT:-1}"
DO_COMPARE="${DO_COMPARE:-1}"
DO_SCOREBOARD="${DO_SCOREBOARD:-1}"
CLEAN_PREDICT="${CLEAN_PREDICT:-1}"
CLEAN_COMPARE="${CLEAN_COMPARE:-1}"

DATASETS=(
  baylands_super_75_15_10
  baylands_generalization_art
  baylands_generalization_playground
  baylands_generalization_road_to_art
  baylands_generalization_rotundan
)

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
    --name "$name"
}

run_predict () {
  local dataset="$1"
  local out_dir="$EXP/runs/predictions/$RUN/${dataset}_test_all"

  if [[ "$CLEAN_PREDICT" == "1" ]]; then
    rm -rf "$out_dir"
  fi

  yolo obb predict \
    model="$MODEL" \
    source="${ROOT}/datasets/final/${dataset}/images/test" \
    imgsz="$IMGSZ" \
    conf="$CONF" \
    device="$DEVICE" \
    save=true \
    save_txt=true \
    save_conf=true \
    project="$EXP/runs/predictions/$RUN" \
    name="${dataset}_test_all" \
    exist_ok=true
}

run_compare () {
  local dataset="$1"
  local out_dir="$EXP/runs/prediction_analysis/$RUN/${dataset}"

  if [[ "$CLEAN_COMPARE" == "1" ]]; then
    rm -rf "$out_dir"
  fi

  "$PYTHON" "${ROOT}/models/obb/compare_obb_predictions.py" \
    --dataset "${ROOT}/datasets/final/${dataset}" \
    --pred-dir "$EXP/runs/predictions/$RUN/${dataset}_test_all" \
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
  run_val baylands_super_75_15_10 val baylands_super_75_15_10_val

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
  "$PYTHON" "${ROOT}/models/obb/build_sam3_full_scoreboard.py" \
    --exp-root "$EXP" \
    --out-dir "$EXP/scoreboards"
fi

echo
echo "Done."
echo "Validation:          $EXP/runs/val/$RUN"
echo "Predictions:         $EXP/runs/predictions/$RUN"
echo "Prediction analysis: $EXP/runs/prediction_analysis/$RUN"
echo "Full scoreboard:     $EXP/scoreboards/full_scoreboard.md"

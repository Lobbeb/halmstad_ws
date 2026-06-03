# SSH GPU Training and Evaluation Workflow

Date: 2026-05-22

## Goal

Use the HH GPU lab and Ultralytics cloud/local tooling to improve the Baylands UGV OBB model trained on SAM3-refined labels, then evaluate each candidate on the main test split and the generalization datasets.

## Data

Final datasets live locally under:

```text
/home/ruben/halmstad_ws/datasets/final/
```

Main dataset:

```text
baylands_super_75_15_10
```

Generalization datasets:

```text
baylands_generalization_art
baylands_generalization_playground
baylands_generalization_road_to_art
baylands_generalization_rotundan
```

For HH GPU work, sync only needed dataset files:

```bash
rsync -av --info=progress2 \
  --include='/images/***' \
  --include='/labels/***' \
  --include='/dataset.yaml' \
  --exclude='*' \
  /home/ruben/halmstad_ws/datasets/final/baylands_super_75_15_10/ \
  rubcro20@bermingham.hh.se:/nfs/home/rubcro20/halmstad_ws/datasets/final/baylands_super_75_15_10/
```

## SSH GPU

Host:

```text
rubcro20@bermingham.hh.se
```

Remote workspace:

```text
/nfs/home/rubcro20/halmstad_ws
```

Remote Python environment:

```bash
source /tmp/rubcro20/sam3-obb/bin/activate
```

We redirect Ultralytics config/cache to `/tmp` to avoid home quota issues:

```bash
export YOLO_CONFIG_DIR=/tmp/rubcro20/ultralytics_config
export MPLCONFIGDIR=/tmp/rubcro20/matplotlib_config
export XDG_CACHE_HOME=/tmp/rubcro20/cache
```

## Model Tuning

Tuning script:

```text
/home/ruben/halmstad_ws/models/obb/tune_sam3_hybrid_full.py
```

Remote run example:

```bash
cd /nfs/home/rubcro20/halmstad_ws
source /tmp/rubcro20/sam3-obb/bin/activate

ROOT=/nfs/home/rubcro20/halmstad_ws \
BASE_MODEL=/nfs/home/rubcro20/halmstad_ws/models/obb/mymodels/baylands-leader-v7a-ft-nomosaic-lr0002.pt \
RUN=v8_tune_from_v7a \
DEVICE=0 \
EPOCHS=35 \
ITERATIONS=12 \
python models/obb/tune_sam3_hybrid_full.py
```

Important fix: `save=True` is required for this Ultralytics version, otherwise tuning can complete epochs but fail because no `best.pt` or `last.pt` checkpoint exists.

## Training Runs

Training configs and downloaded training metrics are stored here:

```text
models/obb/experiments/sam3_hybrid_full/configs/
models/obb/experiments/sam3_hybrid_full/metrics/
```

Weights are stored here:

```text
models/obb/mymodels/
```

Main run pattern:

1. Train on Ultralytics cloud or another GPU.
2. Download `<run>.pt` to `models/obb/mymodels/`.
3. Download training metrics to `experiments/sam3_hybrid_full/metrics/<run>.json`.
4. Keep config as `experiments/sam3_hybrid_full/configs/<run>.json`.

## Validation and Prediction

Evaluation script:

```text
models/obb/validate_predict_sam3.sh
```

Run locally:

```bash
cd /home/ruben/halmstad_ws
source /home/ruben/miniconda3/etc/profile.d/conda.sh
conda activate pytdml

MODEL=/home/ruben/halmstad_ws/models/obb/mymodels/<run>.pt \
RUN=<run> \
DEVICE=cpu \
./models/obb/validate_predict_sam3.sh
```

For `imgsz=768` models:

```bash
IMGSZ=768
```

The script writes:

```text
runs/val/<run>/
runs/predictions/<run>/
runs/prediction_analysis/<run>/
```

Prediction folders are cleaned before reruns to avoid duplicate YOLO label rows.

## Scoreboard

Main scoreboard:

```text
models/obb/experiments/sam3_hybrid_full/scoreboards/full_scoreboard.md
```

Regenerate:

```bash
python /home/ruben/halmstad_ws/models/obb/build_sam3_full_scoreboard.py \
  --exp-root /home/ruben/halmstad_ws/models/obb/experiments/sam3_hybrid_full \
  --out-dir /home/ruben/halmstad_ws/models/obb/experiments/sam3_hybrid_full/scoreboards
```

The scoreboard combines:

```text
training metrics
main validation/test metrics
generalization test metrics
prediction-analysis scores
```

## Current Finding

Best current model by full evaluation:

```text
baylands-leader-v7a-ft-nomosaic-lr0002
```

Reason:

```text
best eval_score
best main_test_mAP50-95
best gen_avg_mAP50-95
```

Small mosaic variants were competitive but did not beat v7a overall. Stronger color/geometric augmentation hurt generalization in the tested runs.

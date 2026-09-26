# SAM3 Hybrid OBB Model Tuning Pipeline

Date: 2026-05-22

## Goal

Improve the Baylands UGV oriented bounding box model after replacing the original labels with SAM3-refined OBB labels, while keeping the generalization datasets as unseen evaluation data.

The current best confirmed model is:

```text
baylands-leader-v9-tuned-full
```

It has the best combined evaluation score in:

```text
/home/ruben/halmstad_ws/models/obb/experiments/sam3_hybrid_full/scoreboards/full_scoreboard.md
```

## Data Setup

Final datasets live under:

```text
/home/ruben/halmstad_ws/datasets/final/
```

Main training dataset:

```text
baylands_super_75_15_10
```

Held-out/generalization datasets:

```text
baylands_generalization_art
baylands_generalization_playground
baylands_generalization_road_to_art
baylands_generalization_rotundan
```

The generalization datasets were used for evaluation, not for training, so they still measure out-of-distribution performance.

## Label Cleanup

Before final model selection, we cleaned the datasets based on model-vs-label disagreements:

- Found images where the dataset label was empty but models predicted a UGV.
- Manually reviewed those cases.
- Removed unusable images marked `remove`.
- Re-annotated missing UGV labels with the SAM3 OBB pipeline where possible.
- Manually fixed a small set of unresolved OBB labels.
- Synced corrected labels back into the final datasets.

Relevant manifests:

```text
notes/removed_unusable_20260522_023211.json
notes/reannotated_missing_gt_20260522_024118.json
notes/manual_fixed_unresolved_20260522_030154.json
notes/synced_manual_review_fixes_20260522_035116.json
```

## Training and Tuning Loop

For each model candidate:

1. Train on the final Baylands SAM3 OBB dataset.
2. Save the training config in:

```text
models/obb/experiments/sam3_hybrid_full/configs/<run>.json
```

3. Save trusted training metrics in:

```text
models/obb/experiments/sam3_hybrid_full/metrics/<run>.json
```

4. Store downloaded weights in:

```text
models/obb/mymodels/<run>.pt
```

5. Run local validation, prediction, prediction analysis, and scoreboard rebuild:

```bash
cd /home/ruben/halmstad_ws

MODEL=/home/ruben/halmstad_ws/models/obb/mymodels/<run>.pt \
RUN=<run> \
DEVICE=0 \
IMGSZ=640 \
./models/obb/validate_predict_sam3.sh
```

The eval script writes:

```text
runs/val/<run>/
runs/predictions/<run>/
runs/prediction_analysis/<run>/
scoreboards/full_scoreboard.md
scoreboards/full_scoreboard.csv
```

## Hyperparameter Tuning

We used the HH GPU lab for Ultralytics tuning through:

```text
/home/ruben/halmstad_ws/models/obb/tune_sam3_hybrid_full.py
```

The most useful tune run was:

```text
v8_tune_from_v7a
```

It started from:

```text
baylands-leader-v7a-ft-nomosaic-lr0002.pt
```

The best tuning result produced:

```text
models/obb/experiments/sam3_hybrid_full/runs/tune/v8_tune_from_v7a/weights/best.pt
```

Best tuned parameters were approximately:

```yaml
lr0: 0.00033
lrf: 0.00551
momentum: 0.91223
weight_decay: 0.0008
warmup_epochs: 2.44359
box: 8.26491
cls: 0.6279
dfl: 1.21425
angle: 0.79144
hsv_h: 0.02322
hsv_s: 0.44735
hsv_v: 0.3
degrees: 0.00188
translate: 0.05487
scale: 0.25
shear: 0.01004
perspective: 0.0002
flipud: 0.00254
fliplr: 0.3
mosaic: 0.25
close_mosaic: 11
```

Ultralytics Cloud could not set every tuned value exactly, especially very low `lrf`, so the later cloud runs used the closest allowed values.

## Main Model Branches

Important tested branches:

```text
v6: original strong baseline on SAM3 full dataset
v7a: fine-tuned no-mosaic branch, strongest early generalization
v7b/v7c: small/medium mosaic variants
v8a/v8b/v8c: extra training and mild augmentation experiments
v8 tune: HH GPU hyperparameter tuning result
v9: full train from tuned best model
v9a: same tuned setup with Ultralytics defaults kept
v9b: higher patience/save-period continuation style run
```

## Current Results

Top scoreboard rows at the time of writing:

| run | eval_score | train_mAP50-95 | main_test_mAP50-95 | gen_avg_mAP50-95 | pred_avg_score |
| --- | ---: | ---: | ---: | ---: | ---: |
| baylands-leader-v9-tuned-full | 0.7872 | 0.8309 | 0.8530 | 0.7078 | 0.7253 |
| baylands-leader-v9a-tuned-defaults | 0.7840 | 0.8354 | 0.8508 | 0.6972 | 0.7361 |
| baylands-leader-v7a-ft-nomosaic-lr0002 | 0.7823 | 0.8275 | 0.8315 | 0.7165 | 0.7458 |
| baylands-leader-v8b-tinymosaic | 0.7759 | 0.8271 | 0.8277 | 0.7032 | 0.7458 |
| baylands-leader-v9b-patient-saveper | 0.7755 | 0.8331 | 0.8353 | 0.6940 | 0.7335 |

Interpretation:

- `v9-tuned-full` is the best overall confirmed model.
- `v9a` has slightly better training/validation metrics, but weaker generalization than `v9`.
- `v7a` remains a strong reference because it generalizes well despite lower main-test score.
- More patience in `v9b` did not improve the full external score.
- Heavy or poorly targeted augmentation generally hurt generalization in this dataset.

## Evaluation Policy

We should keep the generalization datasets out of training until after real pipeline testing. They are currently our best check for whether a Baylands-trained model transfers to different-looking scenes.

Recommended next step:

1. Test `baylands-leader-v9-tuned-full.pt` in the real/sim visual-follow pipeline.
2. Inspect practical failure modes: missed detections, duplicate detections, unstable OBB angle, heading instability.
3. Only train a separate `generalized` branch if the practical test shows clear domain-transfer failures.


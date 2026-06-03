# Scoreboard Score Calculation

The full scoreboard is built by:

```bash
python3 models/obb/build_sam3_full_scoreboard.py --exp-root models/obb/experiments/sam3_hybrid_full
```

## Inputs

- `configs/<run>.json`: model/config metadata such as `model`, `data`, `lr0`, `mosaic`, `scale`, and `degrees`.
- `metrics/<run>.json`: training metrics from the training run.
- `runs/val/<run>/*/metrics.json`: local validation/test metrics for the main and generalization datasets.
- `runs/prediction_analysis/<run>/*/summary.json`: prediction analysis and issue counts.

`lr0` is read from `configs/<run>.json`. Logged metric keys such as `lr` or `lr/pg0` are not treated as `lr0`.

## Metric Score

Training and local validation metric scores use the same formula:

```text
metric_score =
  0.55 * mAP50-95
+ 0.20 * mAP50
+ 0.15 * recall
+ 0.10 * precision
- 0.025 * val/box_loss
- 0.025 * val/angle_loss
```

Required fields:

```text
mAP50-95
mAP50
recall
precision
```

Loss fields default to `0.0` if missing.

## Prediction Score

Prediction issue rate is:

```text
issue_rate =
  (no_prediction + multi_prediction + false_positive_empty + under_prediction) / images
```

Prediction score is:

```text
prediction_score = mean_best_iou_non_empty * (1 - issue_rate)
```

## Final Evaluation Score

The main ranking score is:

```text
eval_score =
  0.45 * main_test_mAP50-95
+ 0.35 * gen_avg_mAP50-95
+ 0.10 * train_mAP50-95
+ 0.10 * pred_avg_score
```

`main_test_mAP50-95` and `gen_avg_mAP50-95` are required. Training and prediction terms fall back to `0.0` if missing.

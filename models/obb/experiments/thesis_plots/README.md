# Thesis model plots

Generate thesis-style plots from the saved OBB model training, validation, prediction, and review outputs.

```bash
python3 models/obb/experiments/thesis_plots/make_thesis_model_plots.py
```

Optional thesis filter:

By default, all models present in the scoreboards are included. To reproduce the earlier pre-submission-only view:

Override if needed:

```bash
python3 models/obb/experiments/thesis_plots/make_thesis_model_plots.py \
  --max-rank 23 \
  --exclude-model-glob 'v10*,v9-2*'
```

Outputs are written to:

```text
models/obb/experiments/thesis_plots/plots/
```

Generated plots:

| File | Source | Purpose |
|---|---|---|
| `01_model_ranking_top_models.png` | `scoreboards/full_scoreboard.csv` | Compare top models across evaluation, main-test, generalization, and prediction scores. |
| `02_main_vs_generalization_scatter.png` | `scoreboards/full_scoreboard.csv` | Show robustness trade-off between main-test and generalization mAP. |
| `03_prediction_score_scatter.png` | `scoreboards/full_scoreboard.csv` | Compare main and generalization prediction scores. |
| `04_heldout_generalization_bars.png` | `scoreboards/heldout/heldout_scoreboard.csv` | Leave-one-scenario-out performance and issue rate. |
| `05_prediction_issue_breakdown.png` | `runs/prediction_analysis/<best-model>/*/summary.json` | Failure mode rates by dataset. |
| `06_iou_distribution_by_dataset.png` | `runs/prediction_analysis/<best-model>/*/comparison.csv` | OBB IoU distribution by dataset. |
| `07_route_validation_heatmap.png` | `experiments/results/analysis/all_validation_runs.csv` | Per-scenario validation mAP heatmap. |
| `08_tuning_fitness_curves.png` | `runs/tune/*/tune_results.ndjson` | Hyperparameter tuning fitness curves. |
| `09_chosen_amcl_vs_sam3_montage.jpg` | `datasets/review/chosen/` | Hand-picked AMCL-vs-SAM3 label comparison examples. |

A manifest is written to `plots_manifest.csv`.

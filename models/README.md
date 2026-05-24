# Models Scripts

Run commands from the workspace root unless noted:

```bash
cd /home/ruben/halmstad_ws
```

Use the ML environment that has `ultralytics`, `torch`, `transformers`, and OpenCV installed. Locally this is usually:

```bash
conda activate pytdml
```

On the HH GPU lab, use the prepared venv/conda setup from `scripts/hh_gpu_remote_setup_sam3_obb.sh`.

## Main Evaluation Flow

Run validation, full test prediction, prediction analysis, and scoreboards for one model:

```bash
MODEL=/home/ruben/halmstad_ws/models/obb/mymodels/baylands-leader-v9-tuned-full.pt \
RUN=baylands-leader-v9-tuned-full \
DEVICE=cpu \
./models/obb/validate_predict_sam3.sh
```

Environment arguments:

- `ROOT`: workspace root, default `/home/ruben/halmstad_ws`.
- `MODEL_ROOT`: default `$ROOT/models/obb/mymodels`.
- `EXP`: experiment root, default `$ROOT/models/obb/experiments/sam3_hybrid_full`.
- `MODEL`: `.pt` model to evaluate.
- `RUN`: output run name, default model filename without `.pt`.
- `DEVICE`: Ultralytics device, e.g. `cpu`, `0`, `cuda`.
- `IMGSZ`: inference image size, default `640`.
- `CONF`: prediction confidence threshold, default `0.25`.
- `TOP_K`: examples copied per ranked bucket, default `20`.
- `MAX_ISSUE`: examples copied per issue bucket, default `50`.
- `PYTHON`: Python executable, default `python3`.
- `DO_VAL`, `DO_PREDICT`, `DO_COMPARE`, `DO_SCOREBOARD`: set `0` to skip a stage.
- `CLEAN_PREDICT`, `CLEAN_COMPARE`: set `0` to keep previous output dirs.

Outputs land under:

```text
models/obb/experiments/sam3_hybrid_full/runs/val/<RUN>
models/obb/experiments/sam3_hybrid_full/runs/predictions/<RUN>
models/obb/experiments/sam3_hybrid_full/runs/prediction_analysis/<RUN>
models/obb/experiments/sam3_hybrid_full/scoreboards
```

## Python Scripts

### `models/annotate_dataset_sam3_obb.py`

Auto-label YOLO OBB datasets using detector OBB heading plus SAM3 segmentation.

```bash
python models/annotate_dataset_sam3_obb.py \
  --backend hf \
  --dataset /path/to/dataset \
  --output-root /path/to/output \
  --det-model /path/to/detector.pt \
  --sam-model /path/to/sam3 \
  --splits train val test \
  --device cuda \
  --det-device cuda \
  --imgsz 640 \
  --sam-size 644 \
  --obb-mode hybrid \
  --skip-existing
```

Arguments:

- `--backend {hf,ultralytics,ultralytics_semantic}`
- `--dataset PATH`
- `--output-root PATH`
- `--det-model PATH`
- `--sam-model PATH`
- `--splits SPLIT [SPLIT ...]`
- `--device DEVICE`
- `--det-device DEVICE`
- `--conf FLOAT`
- `--iou FLOAT`
- `--imgsz INT`
- `--max-det INT`
- `--classes [INT ...]`
- `--sam-size INT`
- `--sam-imgsz INT`
- `--obb-mode {minarea,hybrid,ultra_polygon}`
- `--write-empty`
- `--overwrite`
- `--debug`
- `--debug-every INT`
- `--num-shards INT`
- `--shard-index INT`
- `--skip-existing`

### `models/obb/run_obb_val.py`

Run one Ultralytics OBB validation and save `metrics.json`.

```bash
python models/obb/run_obb_val.py \
  --model models/obb/mymodels/baylands-leader-v9-tuned-full.pt \
  --data /tmp/baylands_eval.yaml \
  --dataset baylands_super_75_15_10 \
  --split test \
  --project models/obb/experiments/sam3_hybrid_full/runs/val/my-run \
  --name baylands_super_test \
  --device cpu \
  --imgsz 640
```

Arguments:

- `--model PATH`
- `--data PATH`
- `--dataset NAME`
- `--split {train,val,test}`
- `--project PATH`
- `--name NAME`
- `--device DEVICE`
- `--imgsz INT`
- `--iou FLOAT`
- `--single-cls`
- `--plots` / `--no-plots`

### `models/obb/compare_obb_predictions.py`

Compare predicted OBB labels against ground truth and copy useful review examples.

```bash
python models/obb/compare_obb_predictions.py \
  --dataset datasets/final/baylands_super_75_15_10 \
  --pred-dir models/obb/experiments/sam3_hybrid_full/runs/predictions/my-run/baylands_super_75_15_10_test_all \
  --split test \
  --out-dir models/obb/experiments/sam3_hybrid_full/runs/prediction_analysis/my-run/baylands_super_75_15_10
```

Arguments:

- `--dataset PATH`
- `--pred-dir PATH`
- `--split SPLIT`
- `--out-dir PATH`
- `--top-k INT`
- `--max-issue INT`

### `models/obb/build_sam3_full_scoreboard.py`

Build Markdown/CSV scoreboards from training metrics, validation metrics, and prediction analysis.

```bash
python models/obb/build_sam3_full_scoreboard.py \
  --exp-root models/obb/experiments/sam3_hybrid_full \
  --out-dir models/obb/experiments/sam3_hybrid_full/scoreboards
```

Arguments:

- `--exp-root PATH`
- `--out-dir PATH`

### `models/obb/tune_sam3_hybrid_full.py`

Run Ultralytics hyperparameter tuning for the full SAM3 OBB dataset.

```bash
python models/obb/tune_sam3_hybrid_full.py \
  --model models/obb/mymodels/baylands-leader-v9-tuned-full.pt \
  --name v10_tune_from_v9_tight \
  --device 0 \
  --epochs 35 \
  --iterations 40 \
  --space-preset v9_tight
```

Arguments:

- `--root PATH`
- `--dataset NAME`
- `--data PATH`
- `--data-root PATH`
- `--model PATH`
- `--name NAME`
- `--device DEVICE`
- `--epochs INT`
- `--iterations INT`
- `--imgsz INT`
- `--batch VALUE`
- `--workers INT`
- `--patience INT`
- `--cache VALUE`
- `--space-preset {broad,v8_refine,v9_tight}`
- `--space-file PATH`
- `--resume`

The same values can be passed as environment variables: `ROOT`, `DATASET`, `DATA`, `DATA_ROOT`, `BASE_MODEL`, `RUN`, `DEVICE`, `EPOCHS`, `ITERATIONS`, `IMGSZ`, `BATCH`, `WORKERS`, `PATIENCE`, `CACHE`, `SPACE_PRESET`, `SPACE_FILE`, `RESUME`.

### Other Scripts

- `models/analyze_validation_metrics.py`
  - Args: `--results`, `--metric {map50_95,map50,precision,recall,fitness}`, `--output`, `--show`.
- `models/obb/track_yolo_json_runs.py`
  - Args: `--metrics-dir`, `--configs-dir`, `--out-dir`.
- `models/model_predict.py`
  - Args: `--dataset`, `--dataset-path`, `--model`.
- `models/model_validate.py`
  - Args: `--dataset`, `--dataset-path`, `--model`.
- `models/run_all_validations.sh`
  - Batch wrapper with hardcoded model/dataset lists. Edit lists before using.
- `models/ultralytics_sam3_debug_one.py`
  - Hardcoded one-image SAM3 debug script. Edit constants at top before using.
- `models/baylands_leader_training.yaml`
  - Training parameter reference for Ultralytics UI/cloud runs.

## HH GPU Lab SSH

Use your HH account and the lab host. Current default host in scripts is `bermingham.hh.se`.

```bash
ssh -p 20022 rubcro20@bermingham.hh.se
```

Optional SSH config:

```sshconfig
Host hh-bermingham
  HostName bermingham.hh.se
  User rubcro20
  Port 20022
```

Then connect with:

```bash
ssh hh-bermingham
```

## Push To GPU Lab

Push the SAM3 annotation scripts, setup scripts, one detector model, and required data layout:

```bash
cd /home/ruben/halmstad_ws
HH_GPU_USER=rubcro20 HH_GPU_HOST=bermingham.hh.se ./scripts/hh_gpu_sync_sam3_obb.sh
```

Common environment arguments:

- `HH_GPU_USER`: required user, e.g. `rubcro20`.
- `HH_GPU_HOST`: default `bermingham.hh.se`.
- `HH_GPU_SSH_PORT`: default `20022`.
- `HH_GPU_REMOTE_ROOT`: default `/nfs/home/$HH_GPU_USER/halmstad_ws`.
- `DET_MODEL_NAME`: model copied by `hh_gpu_sync_sam3_obb.sh`, default `baylands-leader-v4-3.pt`.

Generic push example:

```bash
rsync -avz --info=progress2 -e "ssh -p 20022" \
  /home/ruben/halmstad_ws/models/obb/mymodels/baylands-leader-v9-tuned-full.pt \
  rubcro20@bermingham.hh.se:/nfs/home/rubcro20/halmstad_ws/models/obb/mymodels/
```

## Run On GPU Lab

Remote setup:

```bash
ssh -p 20022 rubcro20@bermingham.hh.se
/nfs/home/rubcro20/halmstad_ws/scripts/hh_gpu_remote_setup_sam3_obb.sh
```

Run SAM3 annotation:

```bash
SPLITS=train \
SAM_SIZE=644 \
DEVICE=cuda \
DET_DEVICE=cuda \
/nfs/home/rubcro20/halmstad_ws/scripts/hh_gpu_run_sam3_obb_remote.sh
```

Run generalization annotation:

```bash
SAM_SIZE=644 \
SPLITS="val test" \
/nfs/home/rubcro20/halmstad_ws/scripts/hh_gpu_run_generalization_sam3_obb_remote.sh
```

Run tuning:

```bash
RUN=v10_tune_from_v9_tight \
BASE_MODEL=/nfs/home/rubcro20/halmstad_ws/models/obb/mymodels/baylands-leader-v9-tuned-full.pt \
DEVICE=0 \
EPOCHS=35 \
ITERATIONS=40 \
PATIENCE=12 \
SPACE_PRESET=v9_tight \
CACHE=ram \
/nfs/home/rubcro20/halmstad_ws/scripts/hh_gpu_run_tune_sam3_full_remote.sh
```

Resume tuning:

```bash
RESUME=1 RUN=v10_tune_from_v9_tight-2 /nfs/home/rubcro20/halmstad_ws/scripts/hh_gpu_run_tune_sam3_full_remote.sh
```

## Pull From GPU Lab

Pull generated SAM3 labels:

```bash
cd /home/ruben/halmstad_ws
HH_GPU_USER=rubcro20 HH_GPU_HOST=bermingham.hh.se ./scripts/hh_gpu_fetch_sam3_obb_labels.sh
```

Pull tune results:

```bash
rsync -avz --info=progress2 -e "ssh -p 20022" \
  rubcro20@bermingham.hh.se:/nfs/home/rubcro20/halmstad_ws/models/obb/experiments/sam3_hybrid_full/runs/tune/v10_tune_from_v9_tight-2/ \
  /home/ruben/halmstad_ws/models/obb/experiments/sam3_hybrid_full/runs/tune/v10_tune_from_v9_tight-2/
```

Pull model weights:

```bash
rsync -avz --info=progress2 -e "ssh -p 20022" \
  rubcro20@bermingham.hh.se:/nfs/home/rubcro20/halmstad_ws/models/obb/mymodels/*.pt \
  /home/ruben/halmstad_ws/models/obb/mymodels/
```

Check remote jobs:

```bash
ssh -p 20022 rubcro20@bermingham.hh.se \
'ps -u "$USER" -o pid,etime,%cpu,%mem,cmd | egrep "python|tune|annotate|rsync|hf download" | grep -v egrep'
```

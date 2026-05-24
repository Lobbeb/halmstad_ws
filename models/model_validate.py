from ultralytics import YOLO
from pathlib import Path
import argparse
import json
from datetime import datetime

print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))

parser = argparse.ArgumentParser(description="Example script with arguments")
parser.add_argument("--dataset", required=True, help="Input name of dataset")
parser.add_argument("--dataset-path", required=False, help="If specified, overrides the default dataset path")
parser.add_argument("--model", required=True, default="baylands-leader-v4.2-best.pt", help="Input model file")

args = parser.parse_args()

dataset_path = Path("/home/ruben/halmstad_ws/datasets/tmp_datasets", args.dataset, "dataset.yaml")
if args.dataset_path:
    dataset_path = Path(args.dataset_path) / "dataset.yaml"
model_path = Path("obb/mymodels/", args.model)

dataset_name = Path(args.dataset).name
model_name = Path(args.model).stem

print("Dataset:", dataset_name)
if args.dataset_path:
    print("Dataset path:", args.dataset_path)
print("Model:", model_name, "\n")


# --- Validation ---

model = YOLO(str(model_path))

metrics = model.val(
    data=       str(dataset_path),
    task=       "obb",
    split=      "val",
    project=    Path("/home/ruben/halmstad_ws/models/results", model_name, dataset_name),
    single_cls= True,
    exist_ok=   True,
    visualize=  False,
    save_txt=   False,
    iou=        0.7,
    save_json=  True,
)

metrics_json_path = Path(metrics.save_dir) / "metrics.json"

with open(metrics_json_path, "w", encoding="utf-8") as f:
    json.dump(
        {
            "dataset": dataset_name,
            "model": args.model,
            "metrics": {
                key: float(value) if hasattr(value, "__float__") else str(value)
                for key, value in metrics.results_dict.items()
            },
        },
        f,
        indent=2,
    )

print("Saved metrics:", metrics_json_path)

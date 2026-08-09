#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

tmp_root = Path("/tmp") / os.environ.get("USER", "rubcro20")
os.environ.setdefault("YOLO_CONFIG_DIR", str(tmp_root / "ultralytics_config"))
os.environ.setdefault("MPLCONFIGDIR", str(tmp_root / "matplotlib_config"))
os.environ.setdefault("XDG_CACHE_HOME", str(tmp_root / "cache"))

from ultralytics import YOLO


def env(name: str, default: str) -> str:
    return os.environ.get(name, default)


def write_eval_yaml(dataset_root: Path, dataset: str) -> Path:
    yaml_path = Path("/tmp") / f"{dataset}_tune.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                f"path: {dataset_root}",
                "train: images/train",
                "val: images/val",
                "test: images/test",
                "nc: 1",
                "names:",
                "  0: ugv",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return yaml_path


def broad_space() -> dict[str, tuple[float, float] | tuple[float, float, float]]:
    return {
        "lr0": (1e-4, 8e-4),
        "lrf": (0.01, 0.03),
        "momentum": (0.88, 0.96),
        "weight_decay": (0.0002, 0.0008),
        "warmup_epochs": (1.0, 4.0),
        "box": (6.0, 10.0),
        "cls": (0.3, 0.8),
        "dfl": (1.0, 2.0),
        "hsv_h": (0.005, 0.025),
        "hsv_s": (0.15, 0.45),
        "hsv_v": (0.10, 0.30),
        "degrees": (0.0, 4.0),
        "translate": (0.01, 0.06),
        "scale": (0.05, 0.25),
        "shear": (0.0, 1.0),
        "perspective": (0.0, 0.0003),
        "flipud": (0.0, 0.10),
        "fliplr": (0.3, 0.7),
        "bgr": (0.0, 0.05),
        "mosaic": (0.0, 0.25),
        "close_mosaic": (0.0, 15.0),
    }


def v8_refine_space() -> dict[str, tuple[float, float] | tuple[float, float, float]]:
    """
    Narrower second-pass search around v8_tune_from_v7a iteration 11.

    A few values that hit the previous upper bound are allowed to move higher
    so the next tuner is not boxed in by the first run.
    """
    return {
        "lr0": (2e-4, 5.5e-4),
        "lrf": (0.01, 0.015),
        "momentum": (0.89, 0.94, 0.3),
        "weight_decay": (0.00055, 0.0012),
        "warmup_epochs": (1.7, 3.3),
        "box": (7.2, 9.5),
        "cls": (0.45, 0.78),
        "dfl": (1.0, 1.6),
        "hsv_h": (0.015, 0.03),
        "hsv_s": (0.30, 0.60),
        "hsv_v": (0.20, 0.40),
        "degrees": (0.0, 1.0),
        "translate": (0.035, 0.075),
        "scale": (0.15, 0.35),
        "shear": (0.0, 0.25),
        "perspective": (0.00005, 0.00035),
        "flipud": (0.0, 0.02),
        "fliplr": (0.25, 0.45),
        "bgr": (0.0, 0.01),
        "mosaic": (0.15, 0.40),
        "close_mosaic": (8.0, 16.0),
    }


def v9_tight_space() -> dict[str, tuple[float, float] | tuple[float, float, float]]:
    """
    Tight follow-up search around the v8 tuned winner and v9 full-run settings.

    This keeps the augmentation policy close to the current best model while
    still allowing small shifts in optimizer, loss balance, and mosaic timing.
    """
    return {
        "lr0": (2.2e-4, 4.5e-4),
        "lrf": (0.01, 0.018),
        "momentum": (0.895, 0.93, 0.3),
        "weight_decay": (0.00065, 0.0011),
        "warmup_epochs": (2.0, 3.0),
        "box": (7.7, 8.9),
        "cls": (0.52, 0.72),
        "dfl": (1.05, 1.40),
        "angle": (0.75, 1.05),
        "hsv_h": (0.018, 0.028),
        "hsv_s": (0.36, 0.55),
        "hsv_v": (0.24, 0.36),
        "degrees": (0.0, 0.5),
        "translate": (0.045, 0.065),
        "scale": (0.20, 0.32),
        "shear": (0.0, 0.08),
        "perspective": (0.00010, 0.00028),
        "flipud": (0.0, 0.01),
        "fliplr": (0.25, 0.40),
        "bgr": (0.0, 0.005),
        "mosaic": (0.18, 0.32),
        "close_mosaic": (9.0, 14.0),
    }


def load_space(path: Path) -> dict[str, tuple[float, ...]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    return {k: tuple(float(x) for x in v) for k, v in raw.items()}


def get_search_space(args: argparse.Namespace) -> dict[str, tuple[float, ...]]:
    if args.space_file:
        return load_space(args.space_file.expanduser().resolve())

    if args.space_preset == "broad":
        return broad_space()
    if args.space_preset == "v8_refine":
        return v8_refine_space()
    if args.space_preset == "v9_tight":
        return v9_tight_space()

    raise ValueError(f"Unknown space preset: {args.space_preset}")


def parse_args() -> argparse.Namespace:
    root_default = env("ROOT", "/home/ruben/halmstad_ws")
    resume_default = env("RESUME", "0").strip().lower() in {"1", "true", "yes", "on"}
    parser = argparse.ArgumentParser(
        description="Run a narrow Ultralytics OBB hyperparameter tune for the SAM3 hybrid full dataset."
    )
    parser.add_argument("--root", type=Path, default=Path(root_default))
    parser.add_argument("--dataset", default=env("DATASET", "baylands_super_75_15_10"))
    parser.add_argument("--data", type=Path, default=Path(env("DATA", "")) if env("DATA", "") else None)
    parser.add_argument("--data-root", type=Path, default=Path(env("DATA_ROOT", "")) if env("DATA_ROOT", "") else None)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(
            env(
                "BASE_MODEL",
                f"{root_default}/models/obb/mymodels/baylands-leader-v6-nomosaic.pt",
            )
        ),
    )
    parser.add_argument("--name", default=env("RUN", "v7_tune_from_nomosaic"))
    parser.add_argument("--device", default=env("DEVICE", "0"))
    parser.add_argument("--epochs", type=int, default=int(env("EPOCHS", "40")))
    parser.add_argument("--iterations", type=int, default=int(env("ITERATIONS", "25")))
    parser.add_argument("--imgsz", type=int, default=int(env("IMGSZ", "640")))
    parser.add_argument("--batch", default=env("BATCH", "-1"))
    parser.add_argument("--workers", type=int, default=int(env("WORKERS", "8")))
    parser.add_argument("--patience", type=int, default=int(env("PATIENCE", "12")))
    parser.add_argument("--cache", default=env("CACHE", "False"))
    parser.add_argument(
        "--space-preset",
        choices=("broad", "v8_refine", "v9_tight"),
        default=env("SPACE_PRESET", "broad"),
    )
    parser.add_argument(
        "--space-file",
        type=Path,
        default=Path(env("SPACE_FILE", "")) if env("SPACE_FILE", "") else None,
        help="Optional JSON file mapping hyperparameter names to [min, max] or [min, max, gain].",
    )
    parser.add_argument("--resume", action="store_true", default=resume_default)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.root.expanduser().resolve()
    model_path = args.model.expanduser().resolve()
    project = root / "models" / "obb" / "experiments" / "sam3_hybrid_full" / "runs" / "tune"
    dataset_root = (args.data_root or (root / "datasets" / "final" / args.dataset)).expanduser().resolve()
    data_yaml = args.data.expanduser().resolve() if args.data else write_eval_yaml(dataset_root, args.dataset)

    if not model_path.exists():
        raise FileNotFoundError(f"Base model not found: {model_path}")
    if not data_yaml.exists():
        raise FileNotFoundError(f"Data YAML not found: {data_yaml}")
    if not (dataset_root / "images" / "train").is_dir() or not (dataset_root / "images" / "val").is_dir():
        raise FileNotFoundError(
            "Dataset root must contain images/train and images/val. "
            f"Missing under: {dataset_root}. "
            "Sync the dataset or pass DATA_ROOT=/path/to/dataset."
        )

    search_space = get_search_space(args)

    print(f"Root:       {root}")
    print(f"Dataset:    {args.dataset}")
    print(f"Dataset root: {dataset_root}")
    print(f"Data YAML:  {data_yaml}")
    print(f"Base model: {model_path}")
    print(f"Project:    {project}")
    print(f"Name:       {args.name}")
    print(f"Device:     {args.device}")
    print(f"Space:      {args.space_file or args.space_preset}")

    model = YOLO(str(model_path))
    model.tune(
        data=str(data_yaml),
        epochs=args.epochs,
        iterations=args.iterations,
        imgsz=args.imgsz,
        batch=int(args.batch) if args.batch.lstrip("-").isdigit() else args.batch,
        device=args.device,
        workers=args.workers,
        optimizer="auto",
        patience=args.patience,
        cache=args.cache,
        cos_lr=True,
        amp=True,
        plots=True,
        save=True,
        val=True,
        space=search_space,
        project=str(project),
        name=args.name,
        resume=args.resume,
        single_cls=False,
        rect=False,
        mixup=0.0,
        cutmix=0.0,
        copy_paste=0.0,
        erasing=0.0,
        pose=1,
        kobj=0.5,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

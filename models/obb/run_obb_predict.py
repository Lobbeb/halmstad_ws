#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path
from typing import Any


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    value_l = value.strip().lower()
    if value_l in {"1", "true", "yes", "y", "on"}:
        return True
    if value_l in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected boolean value, got: {value}")


def parse_bool_or_string(value: str | bool) -> bool | str:
    if isinstance(value, bool):
        return value
    value_s = value.strip()
    value_l = value_s.lower()
    if value_l in {"1", "true", "yes", "y", "on"}:
        return True
    if value_l in {"0", "false", "no", "n", "off"}:
        return False
    return value_s


def parse_int_list(value: str) -> list[int]:
    text = str(value or "").strip()
    if not text:
        return []
    try:
        return [int(part.strip()) for part in text.split(",") if part.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Expected comma-separated integers, got: {value}") from exc


def maybe_set(kwargs: dict[str, Any], key: str, value: Any) -> None:
    if value is not None:
        kwargs[key] = value


def sample_source(source: Path, sample: int, random_sample: bool, seed: int, project: Path, name: str) -> str:
    if sample <= 0:
        return str(source)
    if source.is_file():
        return str(source)
    if not source.is_dir():
        raise FileNotFoundError(f"Prediction source does not exist: {source}")

    images = sorted(p for p in source.rglob("*") if p.is_file() and p.suffix.lower() in IMAGE_EXTS)
    if not images:
        raise FileNotFoundError(f"No images found under prediction source: {source}")
    if random_sample:
        rng = random.Random(seed)
        rng.shuffle(images)
    selected = images[:sample]

    list_dir = project / name
    list_dir.mkdir(parents=True, exist_ok=True)
    list_path = list_dir / "sample_sources.txt"
    list_path.write_text("\n".join(str(p) for p in selected) + "\n", encoding="utf-8")
    print(f"Sampled images: {len(selected)} -> {list_path}")
    return str(list_path)


def parse_csv_list(value: str) -> list[str]:
    return [part.strip() for part in str(value or "").split(",") if part.strip()]


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(text), encoding="utf-8")


def run_result_actions(results_iter, out_dir: Path, actions: list[str], plot_kwargs: dict[str, Any]) -> None:
    if not actions:
        for _ in results_iter:
            pass
        return

    out_dir.mkdir(parents=True, exist_ok=True)
    try:
        import cv2  # type: ignore
    except Exception:
        cv2 = None

    for idx, result in enumerate(results_iter):
        stem = Path(getattr(result, "path", "") or f"result_{idx:06d}").stem
        prefix = out_dir / f"{idx:06d}_{stem}"

        for action in actions:
            if action == "cpu":
                _ = result.cpu()
            elif action == "numpy":
                _ = result.numpy()
            elif action == "cuda":
                _ = result.cuda()
            elif action == "new":
                _ = result.new()
            elif action == "plot":
                img = result.plot(**plot_kwargs)
                if cv2 is not None:
                    cv2.imwrite(str(prefix.with_suffix(".plot.jpg")), img)
            elif action == "save":
                result.save(filename=str(prefix.with_suffix(".save.jpg")))
            elif action == "verbose":
                write_text(prefix.with_suffix(".verbose.txt"), result.verbose())
            elif action == "save_txt":
                result.save_txt(str(prefix.with_suffix(".txt")))
            elif action == "save_crop":
                result.save_crop(str(out_dir / "crops" / f"{idx:06d}_{stem}"))
            elif action == "summary":
                (prefix.with_suffix(".summary.json")).write_text(
                    json.dumps(result.summary(), indent=2),
                    encoding="utf-8",
                )
            elif action == "to_csv":
                write_text(prefix.with_suffix(".csv"), result.to_csv())
            elif action == "to_json":
                write_text(prefix.with_suffix(".json"), result.to_json())
            elif action == "to_df":
                df = result.to_df()
                write_text(prefix.with_suffix(".df.txt"), str(df))
            elif action == "show":
                result.show()
            else:
                raise ValueError(f"Unknown result action: {action}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Ultralytics OBB prediction.")
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--project", required=True, type=Path)
    parser.add_argument("--name", required=True)
    parser.add_argument("--task", default="obb")
    parser.add_argument("--device", default=None)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--half", type=parse_bool, default=False)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--max-det", type=int, default=300)
    parser.add_argument("--vid-stride", type=int, default=1)
    parser.add_argument("--stream-buffer", type=parse_bool, default=False)
    parser.add_argument("--visualize", type=parse_bool, default=False)
    parser.add_argument("--augment", type=parse_bool, default=False)
    parser.add_argument("--agnostic-nms", type=parse_bool, default=False)
    parser.add_argument("--classes", type=parse_int_list, default=None, help="Comma-separated class IDs, e.g. 0,2")
    parser.add_argument("--retina-masks", type=parse_bool, default=False)
    parser.add_argument("--embed", type=parse_int_list, default=None, help="Comma-separated layer indices for embeddings.")
    parser.add_argument(
        "--compile",
        type=parse_bool_or_string,
        default=False,
        help="False/true or a torch.compile mode string accepted by Ultralytics.",
    )
    parser.add_argument("--stream", type=parse_bool, default=False)
    parser.add_argument("--save", type=parse_bool, default=True)
    parser.add_argument("--save-frames", type=parse_bool, default=False)
    parser.add_argument("--save-txt", type=parse_bool, default=False)
    parser.add_argument("--save-conf", type=parse_bool, default=False)
    parser.add_argument("--save-crop", type=parse_bool, default=False)
    parser.add_argument("--show", type=parse_bool, default=False)
    parser.add_argument("--show-labels", type=parse_bool, default=True)
    parser.add_argument("--show-conf", type=parse_bool, default=True)
    parser.add_argument("--show-boxes", type=parse_bool, default=True)
    parser.add_argument("--line-width", type=int, default=None)
    parser.add_argument("--exist-ok", type=parse_bool, default=True)
    parser.add_argument("--verbose", type=parse_bool, default=True)
    parser.add_argument("--sample", type=int, default=0, help="If source is a directory, predict only this many images. 0 = all.")
    parser.add_argument("--random", type=parse_bool, default=False, help="Randomize images before --sample selection.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--result-actions",
        type=parse_csv_list,
        default=[],
        help="Comma-separated Results methods to run: cpu,numpy,cuda,new,plot,save,verbose,save_txt,save_crop,summary,to_df,to_csv,to_json,show.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    from ultralytics import YOLO

    model_path = args.model.expanduser().resolve()
    source = args.source.expanduser().resolve()
    project = args.project.expanduser().resolve()

    if not source.exists():
        raise FileNotFoundError(f"Prediction source does not exist: {source}")

    model = YOLO(str(model_path))
    source_arg = sample_source(source, args.sample, args.random, args.seed, project, args.name)
    predict_kwargs: dict[str, Any] = {
        "source": source_arg,
        "task": args.task,
        "imgsz": args.imgsz,
        "iou": args.iou,
        "half": args.half,
        "batch": args.batch,
        "max_det": args.max_det,
        "vid_stride": args.vid_stride,
        "stream_buffer": args.stream_buffer,
        "visualize": args.visualize,
        "augment": args.augment,
        "agnostic_nms": args.agnostic_nms,
        "retina_masks": args.retina_masks,
        "compile": args.compile,
        "stream": args.stream,
        "save": args.save,
        "save_frames": args.save_frames,
        "save_txt": args.save_txt,
        "save_conf": args.save_conf,
        "save_crop": args.save_crop,
        "show": args.show,
        "show_labels": args.show_labels,
        "show_conf": args.show_conf,
        "show_boxes": args.show_boxes,
        "project": str(project),
        "name": args.name,
        "exist_ok": args.exist_ok,
        "verbose": args.verbose,
    }
    maybe_set(predict_kwargs, "conf", args.conf)
    maybe_set(predict_kwargs, "device", args.device)
    maybe_set(predict_kwargs, "classes", args.classes)
    maybe_set(predict_kwargs, "embed", args.embed)
    maybe_set(predict_kwargs, "line_width", args.line_width)

    if args.result_actions and not args.stream:
        predict_kwargs["stream"] = True

    results = model.predict(**predict_kwargs)
    if args.stream or args.result_actions:
        plot_kwargs = {
            "labels": args.show_labels,
            "conf": args.show_conf,
            "boxes": args.show_boxes,
        }
        maybe_set(plot_kwargs, "line_width", args.line_width)
        run_result_actions(results, project / args.name / "result_actions", args.result_actions, plot_kwargs)

    print(f"Saved predictions: {project / args.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

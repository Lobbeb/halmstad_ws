#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


@dataclass
class ObbLabel:
    cls: int
    points: np.ndarray
    conf: float | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare YOLO OBB prediction labels against ground-truth labels."
    )
    parser.add_argument("--dataset", required=True, type=Path, help="Dataset root with images/ and labels/.")
    parser.add_argument("--pred-dir", required=True, type=Path, help="Ultralytics predict output folder.")
    parser.add_argument("--split", default="test", help="Dataset split to compare.")
    parser.add_argument("--out-dir", required=True, type=Path, help="Folder for CSV and selected examples.")
    parser.add_argument("--top-k", type=int, default=20, help="How many examples to copy per ranked bucket.")
    parser.add_argument("--max-issue", type=int, default=50, help="How many examples to copy per issue bucket.")
    return parser.parse_args()


def read_labels(path: Path, with_conf: bool) -> list[ObbLabel]:
    if not path.exists() or path.stat().st_size == 0:
        return []

    labels: list[ObbLabel] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) < 9:
            continue

        cls = int(float(parts[0]))
        coords = np.array([float(v) for v in parts[1:9]], dtype=np.float32).reshape(4, 2)
        conf = float(parts[9]) if with_conf and len(parts) > 9 else None
        labels.append(ObbLabel(cls=cls, points=order_points(coords), conf=conf))

    return labels


def order_points(points: np.ndarray) -> np.ndarray:
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    return points[np.argsort(angles)].astype(np.float32)


def polygon_area(points: np.ndarray) -> float:
    return abs(float(cv2.contourArea(points.astype(np.float32))))


def obb_iou(a: np.ndarray, b: np.ndarray) -> float:
    area_a = polygon_area(a)
    area_b = polygon_area(b)
    if area_a <= 0.0 or area_b <= 0.0:
        return 0.0

    try:
        inter_area, _ = cv2.intersectConvexConvex(
            a.astype(np.float32),
            b.astype(np.float32),
        )
    except cv2.error:
        return 0.0

    union = area_a + area_b - float(inter_area)
    if union <= 0.0:
        return 0.0
    return max(0.0, min(1.0, float(inter_area) / union))


def match_iou(gt: list[ObbLabel], pred: list[ObbLabel]) -> tuple[float, float, int]:
    if not gt or not pred:
        return 0.0, 0.0, 0

    candidates: list[tuple[float, int, int]] = []
    for gi, gt_label in enumerate(gt):
        for pi, pred_label in enumerate(pred):
            if gt_label.cls != pred_label.cls:
                continue
            candidates.append((obb_iou(gt_label.points, pred_label.points), gi, pi))

    candidates.sort(reverse=True)
    used_gt: set[int] = set()
    used_pred: set[int] = set()
    matched: list[float] = []

    for iou, gi, pi in candidates:
        if gi in used_gt or pi in used_pred:
            continue
        used_gt.add(gi)
        used_pred.add(pi)
        matched.append(iou)

    if not matched:
        return 0.0, 0.0, 0

    return max(matched), sum(matched) / len(matched), len(matched)


def find_images(image_dir: Path) -> list[Path]:
    images: list[Path] = []
    for ext in IMAGE_EXTS:
        images.extend(image_dir.glob(f"*{ext}"))
        images.extend(image_dir.glob(f"*{ext.upper()}"))
    return sorted(set(images))


def find_rendered_image(pred_dir: Path, stem: str) -> Path | None:
    for ext in IMAGE_EXTS:
        candidate = pred_dir / f"{stem}{ext}"
        if candidate.exists():
            return candidate
        candidate = pred_dir / f"{stem}{ext.upper()}"
        if candidate.exists():
            return candidate
    return None


def copy_example(row: dict[str, object], bucket: Path, dataset: Path, pred_dir: Path, split: str) -> None:
    stem = str(row["stem"])
    bucket.mkdir(parents=True, exist_ok=True)

    rendered = find_rendered_image(pred_dir, stem)
    if rendered is not None:
        shutil.copy2(rendered, bucket / rendered.name)

    gt_label = dataset / "labels" / split / f"{stem}.txt"
    pred_label = pred_dir / "labels" / f"{stem}.txt"

    labels_dir = bucket / "labels"
    labels_dir.mkdir(exist_ok=True)
    if gt_label.exists():
        shutil.copy2(gt_label, labels_dir / f"{stem}.gt.txt")
    if pred_label.exists():
        shutil.copy2(pred_label, labels_dir / f"{stem}.pred.txt")


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()

    dataset = args.dataset.expanduser().resolve()
    pred_dir = args.pred_dir.expanduser().resolve()
    out_dir = args.out_dir.expanduser().resolve()
    split = args.split

    images = find_images(dataset / "images" / split)
    if not images:
        raise FileNotFoundError(f"No images found in {dataset / 'images' / split}")

    rows: list[dict[str, object]] = []
    for image_path in images:
        stem = image_path.stem
        gt = read_labels(dataset / "labels" / split / f"{stem}.txt", with_conf=False)
        pred = read_labels(pred_dir / "labels" / f"{stem}.txt", with_conf=True)

        best_iou, mean_iou, matches = match_iou(gt, pred)
        pred_conf = max((p.conf for p in pred if p.conf is not None), default=None)

        if not gt and not pred:
            status = "correct_empty"
        elif not gt and pred:
            status = "false_positive_empty"
        elif gt and not pred:
            status = "no_prediction"
        elif len(pred) > 1:
            status = "multi_prediction"
        elif len(pred) < len(gt):
            status = "under_prediction"
        else:
            status = "matched"

        rows.append(
            {
                "stem": stem,
                "status": status,
                "gt_count": len(gt),
                "pred_count": len(pred),
                "matches": matches,
                "best_iou": round(best_iou, 6),
                "mean_iou": round(mean_iou, 6),
                "max_conf": "" if pred_conf is None else round(float(pred_conf), 6),
                "image": str(image_path),
                "pred_dir": str(pred_dir),
            }
        )

    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "comparison.csv", rows)

    non_empty_matches = [r for r in rows if int(r["gt_count"]) > 0 and int(r["pred_count"]) > 0]
    clean_matches = [r for r in non_empty_matches if r["status"] == "matched"]
    best = sorted(clean_matches, key=lambda r: float(r["best_iou"]), reverse=True)[: args.top_k]
    worst = sorted(clean_matches, key=lambda r: float(r["best_iou"]))[: args.top_k]

    for row in best:
        copy_example(row, out_dir / "best", dataset, pred_dir, split)
    for row in worst:
        copy_example(row, out_dir / "worst", dataset, pred_dir, split)

    for status in ["no_prediction", "multi_prediction", "false_positive_empty", "correct_empty", "under_prediction"]:
        status_rows = [r for r in rows if r["status"] == status][: args.max_issue]
        for row in status_rows:
            copy_example(row, out_dir / status, dataset, pred_dir, split)

    counts: dict[str, int] = {}
    for row in rows:
        counts[str(row["status"])] = counts.get(str(row["status"]), 0) + 1

    summary = {
        "dataset": str(dataset),
        "pred_dir": str(pred_dir),
        "split": split,
        "images": len(rows),
        "status_counts": counts,
        "mean_best_iou_non_empty": (
            sum(float(r["best_iou"]) for r in non_empty_matches) / len(non_empty_matches)
            if non_empty_matches
            else math.nan
        ),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(json.dumps(summary, indent=2))
    print(f"Wrote: {out_dir / 'comparison.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

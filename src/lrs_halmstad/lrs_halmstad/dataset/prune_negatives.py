#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from lrs_halmstad.dataset.sync_check import DATASETS_ROOT, IMAGE_EXTS, resolve_dataset_path

# All sibling directories to clean up alongside images and labels
SIBLING_DIRS = [
    "images",
    "labels",
    "labels_det",
    "metadata",
    "overlay",
    "overlay_detection",
    "overlay_obb",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            f"Delete frames where the UGV is not visible from datasets under {DATASETS_ROOT}. "
            "Catches both empty label files and frames where too few projected points fall within "
            "the image bounds (occluded/out-of-frame despite a non-empty label)."
        )
    )
    parser.add_argument(
        "datasets",
        nargs="+",
        help=(
            f"Dataset path(s), absolute or relative to {DATASETS_ROOT}. "
            "Examples: warehouse_v3 warehouse_v1/run1"
        ),
    )
    parser.add_argument(
        "--min-visible-points",
        type=int,
        default=8,
        help=(
            "Minimum number of projected_points that must fall within the image bounds. "
            "Frames with fewer are treated as not-visible and deleted. "
            "Default: 8. Use 0 to disable this check."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print what would be deleted without actually deleting anything.",
    )
    parser.add_argument(
        "--show",
        type=int,
        default=10,
        help="Maximum number of deleted stems to print per split. Use 0 to print all.",
    )
    return parser.parse_args()


def _find_sibling_files(dataset_dir: Path, split: str, stem: str) -> list[Path]:
    found: list[Path] = []
    for d in SIBLING_DIRS:
        sibling_dir = dataset_dir / d / split
        if not sibling_dir.is_dir():
            continue
        for ext in list(IMAGE_EXTS) + [".txt", ".json"]:
            candidate = sibling_dir / f"{stem}{ext}"
            if candidate.exists():
                found.append(candidate)
    return found


def _visible_point_count(metadata_path: Path) -> int:
    try:
        meta = json.loads(metadata_path.read_text(encoding="utf-8"))
    except Exception:
        return 0
    cam = meta.get("camera_model") or {}
    w = float(cam.get("width") or 0)
    h = float(cam.get("height") or 0)
    if w <= 0 or h <= 0:
        return 0
    pts = meta.get("projected_points") or []
    return sum(1 for u, v in pts if 0 <= u < w and 0 <= v < h)


def prune_dataset(
    dataset_dir: Path,
    min_visible_points: int,
    dry_run: bool,
    show_limit: int,
) -> int:
    labels_dir = dataset_dir / "labels"
    if not labels_dir.is_dir():
        raise FileNotFoundError(f"Dataset is missing labels/: {dataset_dir}")

    metadata_dir = dataset_dir / "metadata"
    has_metadata = metadata_dir.is_dir()
    if min_visible_points > 0 and not has_metadata:
        print("  Warning: no metadata/ dir — skipping visible-point check")

    total_deleted = 0
    splits = sorted(p.name for p in labels_dir.iterdir() if p.is_dir())

    for split in splits:
        split_labels_dir = labels_dir / split
        split_metadata_dir = metadata_dir / split if has_metadata else None

        bad_stems: list[tuple[str, str]] = []  # (stem, reason)

        for label_file in sorted(split_labels_dir.glob("*.txt")):
            stem = label_file.stem

            # Empty label = no bbox at all
            if label_file.stat().st_size == 0:
                bad_stems.append((stem, "empty_label"))
                continue

            # Too few projected points within image bounds
            if min_visible_points > 0 and split_metadata_dir is not None:
                meta_path = split_metadata_dir / f"{stem}.json"
                if meta_path.is_file():
                    visible = _visible_point_count(meta_path)
                    if visible < min_visible_points:
                        bad_stems.append((stem, f"visible_pts={visible}<{min_visible_points}"))

        if not bad_stems:
            print(f"  {split}: 0 bad frames")
            continue

        action = "Would delete" if dry_run else "Deleting"
        print(f"  {split}: {len(bad_stems)} bad frame(s)  ({action})")
        shown = bad_stems if show_limit == 0 else bad_stems[:show_limit]
        for stem, reason in shown:
            print(f"    {stem}  [{reason}]")
        if show_limit and len(shown) < len(bad_stems):
            print(f"    ... ({len(bad_stems) - len(shown)} more)")

        if not dry_run:
            for stem, _ in bad_stems:
                for f in _find_sibling_files(dataset_dir, split, stem):
                    f.unlink()
            total_deleted += len(bad_stems)

    return total_deleted


def main() -> None:
    args = parse_args()
    total = 0
    for value in args.datasets:
        dataset_dir = resolve_dataset_path(value)
        print(f"Dataset: {dataset_dir}")
        total += prune_dataset(
            dataset_dir,
            min_visible_points=args.min_visible_points,
            dry_run=args.dry_run,
            show_limit=args.show,
        )

    if args.dry_run:
        print("Dry run — nothing deleted.")
    else:
        print(f"Done: pruned {total} bad frame(s) across {len(args.datasets)} dataset(s).")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        raise SystemExit(130)

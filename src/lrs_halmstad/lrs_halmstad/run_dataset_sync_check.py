#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path


def _default_datasets_root() -> Path:
    configured_root = os.environ.get("LRS_HALMSTAD_DATASETS_ROOT", "").strip()
    if configured_root:
        return Path(configured_root).expanduser().resolve()
    return (Path(__file__).resolve().parents[3] / "datasets").resolve()


DATASETS_ROOT = _default_datasets_root()
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}
CATEGORY_EXTS = {
    "images": IMAGE_EXTS,
    "labels": {".txt"},
    "metadata": {".json"},
    "overlay": IMAGE_EXTS,
}
DEFAULT_OPTIONAL_CATEGORIES = ("labels", "metadata", "overlay")


@dataclass
class CategoryReport:
    missing_from_category: list[str] = field(default_factory=list)
    orphan_paths: list[Path] = field(default_factory=list)
    duplicate_groups: dict[str, list[Path]] = field(default_factory=dict)


@dataclass
class DatasetReport:
    dataset: Path
    splits: list[str]
    categories: list[str]
    reference_by_split: dict[str, str] = field(default_factory=dict)
    per_split: dict[str, dict[str, CategoryReport]] = field(default_factory=dict)

    def has_remaining_issues(self) -> bool:
        for split_reports in self.per_split.values():
            for report in split_reports.values():
                if report.missing_from_category or report.duplicate_groups or report.orphan_paths:
                    return True
        return False


def parse_args() -> argparse.Namespace:
    datasets_root = str(DATASETS_ROOT)
    parser = argparse.ArgumentParser(
        description=(
            f"Check a dataset under {datasets_root} for missing or orphaned files "
            "across images, labels, metadata, and overlay. When overlay/<split> exists, "
            "the kept set is the intersection of images/<split> and overlay/<split>."
        )
    )
    parser.add_argument(
        "dataset",
        nargs="?",
        default=None,
        help=(
            f"Dataset path, either absolute or relative to {datasets_root}. "
            f"Examples: warehouse_v1/run1 or {datasets_root}/warehouse_auto"
        ),
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help=f"Scan every dataset directory under {datasets_root} that contains an images/ folder.",
    )
    parser.add_argument(
        "--prune-orphans",
        action="store_true",
        help=(
            "Delete orphaned files from every category based on the kept set. "
            "If overlay/<split> exists, the kept set is images∩overlay; otherwise it is images."
        ),
    )
    parser.add_argument(
        "--show",
        type=int,
        default=20,
        help="Maximum number of example paths or stems to print per issue group. Use 0 to print all.",
    )
    return parser.parse_args()


def resolve_dataset_path(value: str) -> Path:
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return candidate.resolve()
    return (DATASETS_ROOT / candidate).resolve()


def discover_dataset_dirs(root: Path = DATASETS_ROOT) -> list[Path]:
    datasets: list[Path] = []
    seen: set[Path] = set()
    for images_dir in sorted(root.rglob("images")):
        if images_dir.is_dir():
            dataset_dir = images_dir.parent.resolve()
            if dataset_dir not in seen:
                seen.add(dataset_dir)
                datasets.append(dataset_dir)
    return datasets


def discover_splits(dataset_dir: Path, categories: list[str]) -> list[str]:
    splits: set[str] = set()
    for category in categories:
        category_dir = dataset_dir / category
        if not category_dir.is_dir():
            continue
        for child in category_dir.iterdir():
            if child.is_dir():
                splits.add(child.name)
    return sorted(splits)


def discover_categories(dataset_dir: Path) -> list[str]:
    categories = ["images"]
    for category in DEFAULT_OPTIONAL_CATEGORIES:
        if (dataset_dir / category).is_dir():
            categories.append(category)
    return categories


def collect_files(base_dir: Path, extensions: set[str]) -> tuple[dict[str, list[Path]], dict[str, list[Path]]]:
    files_by_stem: dict[str, list[Path]] = defaultdict(list)
    duplicates: dict[str, list[Path]] = {}
    if not base_dir.is_dir():
        return files_by_stem, duplicates

    for path in sorted(base_dir.rglob("*")):
        if not path.is_file() or path.suffix.lower() not in extensions:
            continue
        stem = path.relative_to(base_dir).with_suffix("").as_posix()
        files_by_stem[stem].append(path)

    for stem, paths in files_by_stem.items():
        if len(paths) > 1:
            duplicates[stem] = paths
    return files_by_stem, duplicates


def limit_items(items: list[str], maximum: int) -> list[str]:
    if maximum == 0 or len(items) <= maximum:
        return items
    return items[:maximum]


def print_report(report: DatasetReport, show_limit: int) -> None:
    print(f"Dataset: {report.dataset}")
    print(f"Splits: {', '.join(report.splits) if report.splits else 'none found'}")
    print(f"Categories checked: {', '.join(report.categories)}")
    print("")

    any_issue = False
    for split in report.splits:
        reference = report.reference_by_split.get(split, "images")
        print(f"[{split}] kept={reference}")
        split_has_issue = False
        for category, category_report in report.per_split.get(split, {}).items():
            missing = category_report.missing_from_category
            orphans = [str(path) for path in category_report.orphan_paths]
            duplicates = category_report.duplicate_groups
            if not (missing or orphans or duplicates):
                continue

            split_has_issue = True
            any_issue = True
            if missing:
                shown = limit_items(missing, show_limit)
                print(f"  missing {category}: {len(missing)}")
                for stem in shown:
                    print(f"    {stem}")
                if show_limit and len(shown) < len(missing):
                    print(f"    ... {len(missing) - len(shown)} more")
            if orphans:
                shown = limit_items(orphans, show_limit)
                print(f"  orphan {category}: {len(orphans)}")
                for path in shown:
                    print(f"    {path}")
                if show_limit and len(shown) < len(orphans):
                    print(f"    ... {len(orphans) - len(shown)} more")
            if duplicates:
                print(f"  duplicate {category}: {len(duplicates)} stem(s)")
                stems = limit_items(sorted(duplicates.keys()), show_limit)
                for stem in stems:
                    joined = ", ".join(str(path) for path in duplicates[stem])
                    print(f"    {stem}: {joined}")
                if show_limit and len(stems) < len(duplicates):
                    print(f"    ... {len(duplicates) - len(stems)} more")

        if not split_has_issue:
            print("  OK")
        print("")

    if not any_issue:
        print("No issues found.")


def prune_orphans(report: DatasetReport) -> int:
    removed = 0
    for split_reports in report.per_split.values():
        for category, category_report in split_reports.items():
            for path in category_report.orphan_paths:
                path.unlink()
                removed += 1
            category_report.orphan_paths = []
    return removed


def scan_dataset(dataset_dir: Path) -> DatasetReport:
    if not dataset_dir.is_dir():
        raise FileNotFoundError(f"Dataset directory not found: {dataset_dir}")
    if not (dataset_dir / "images").is_dir():
        raise FileNotFoundError(f"Dataset directory is missing images/: {dataset_dir}")

    categories = discover_categories(dataset_dir)
    splits = discover_splits(dataset_dir, categories)
    report = DatasetReport(dataset=dataset_dir, splits=splits, categories=categories)

    if not splits:
        raise FileNotFoundError(f"No split directories found under {dataset_dir / 'images'}")

    for split in splits:
        split_reports: dict[str, CategoryReport] = {}
        collected: dict[str, tuple[dict[str, list[Path]], dict[str, list[Path]]]] = {}
        for category in categories:
            category_dir = dataset_dir / category / split
            collected[category] = collect_files(category_dir, CATEGORY_EXTS[category])
            _, duplicates = collected[category]
            split_reports[category] = CategoryReport(duplicate_groups=duplicates)

        image_stems = set(collected["images"][0].keys())
        overlay_exists = "overlay" in categories and (dataset_dir / "overlay" / split).is_dir()
        if overlay_exists:
            overlay_stems = set(collected["overlay"][0].keys())
            kept_stems = image_stems & overlay_stems
            report.reference_by_split[split] = "images∩overlay"
        else:
            kept_stems = image_stems
            report.reference_by_split[split] = "images"

        for category in categories:
            files, duplicates = collected[category]
            category_stems = set(files.keys())
            split_reports[category].missing_from_category = sorted(kept_stems - category_stems)
            split_reports[category].orphan_paths = [
                path for stem in sorted(category_stems - kept_stems) for path in files[stem]
            ]

        report.per_split[split] = split_reports

    return report


def main() -> int:
    args = parse_args()

    if args.all and args.dataset:
        print("Use either a dataset path or --all, not both.", file=sys.stderr)
        return 2
    if not args.all and not args.dataset:
        print("Provide a dataset path or use --all.", file=sys.stderr)
        return 2

    if args.all:
        datasets = discover_dataset_dirs()
    else:
        dataset_path = resolve_dataset_path(args.dataset)
        if (dataset_path / "images").is_dir():
            datasets = [dataset_path]
        else:
            datasets = discover_dataset_dirs(dataset_path)
            if not datasets:
                print(
                    f"Dataset directory is missing images/ and contains no nested datasets: {dataset_path}",
                    file=sys.stderr,
                )
                return 1
    exit_code = 0

    for index, dataset_dir in enumerate(datasets):
        if index:
            print("=" * 80)
        try:
            report = scan_dataset(dataset_dir)
        except FileNotFoundError as exc:
            print(str(exc), file=sys.stderr)
            exit_code = 1
            continue

        if args.prune_orphans:
            removed = prune_orphans(report)
            if removed:
                print(f"Removed {removed} orphaned file(s) from {dataset_dir}.")
                print("")

        print_report(report, args.show)
        if report.has_remaining_issues():
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

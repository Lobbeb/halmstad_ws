#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import cv2
import numpy as np


DEFAULT_REVIEW_ROOT = Path(
    "/home/ruben/halmstad_ws/models/obb/experiments/sam3_hybrid_full/notes/manual_fix_visual_check"
)
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Rotate, resize, and move YOLO OBB label points."
    )
    parser.add_argument(
        "target",
        nargs="?",
        help=(
            "Label path, review label basename, or stem. If omitted, use --label. "
            "For manual review labels this can be just baylands_...__frame_...."
        ),
    )
    parser.add_argument(
        "--review-root",
        type=Path,
        default=DEFAULT_REVIEW_ROOT,
        help="Manual review root with images/, labels/, and index.tsv.",
    )
    parser.add_argument(
        "--image",
        default=None,
        help="Image path used for pixel-space rotation. Inferred for review labels.",
    )
    parser.add_argument(
        "--label",
        default=None,
        help="YOLO OBB label file to rotate. Can also be a review label basename/stem.",
    )
    parser.add_argument("--degrees", type=float, default=0.0, help="Rotation amount in degrees.")
    parser.add_argument(
        "--width-scale",
        type=float,
        default=1.0,
        help="Scale box width around center before rotation. Example: 1.10 grows width 10%%.",
    )
    parser.add_argument(
        "--height-scale",
        type=float,
        default=1.0,
        help="Scale box height around center before rotation. Example: 0.90 shrinks height 10%%.",
    )
    parser.add_argument(
        "--dx",
        type=float,
        default=0.0,
        help="Move box horizontally in normalized units after rotation. Positive moves right.",
    )
    parser.add_argument(
        "--dy",
        type=float,
        default=0.0,
        help="Move box vertically in normalized units after rotation. Positive moves down.",
    )
    parser.add_argument(
        "--direction",
        choices=("ccw", "cw"),
        default="ccw",
        help="Visual direction in image coordinates.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output label path. Defaults to overwriting --label when --in-place is set.",
    )
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="Overwrite --label. Required unless --output is provided.",
    )
    parser.add_argument(
        "--aabb-label",
        default=None,
        help="Optional YOLO AABB sidecar to update from the rotated OBB.",
    )
    parser.add_argument(
        "--sync-source",
        action="store_true",
        help="For manual review labels, also copy the edited label to the source_label in index.tsv and update source labels_aabb.",
    )
    parser.add_argument(
        "--no-clip",
        action="store_true",
        help="Do not shift boxes back inside 0..1 bounds.",
    )
    return parser.parse_args()


def find_image(image_dir: Path, stem: str) -> Path | None:
    for ext in IMAGE_EXTS:
        path = image_dir / f"{stem}{ext}"
        if path.exists():
            return path
    return None


def read_review_index(index_path: Path) -> dict[Path, dict[str, str]]:
    if not index_path.exists():
        return {}

    rows_by_label: dict[Path, dict[str, str]] = {}
    with index_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f, delimiter="\t"):
            label_file = row.get("label") or row.get("label_file")
            if not label_file:
                continue
            rows_by_label[Path(label_file).expanduser().resolve()] = row
    return rows_by_label


def resolve_label(label_value: str, review_root: Path) -> Path:
    raw = Path(label_value).expanduser()

    candidates: list[Path] = []
    if raw.is_absolute() or raw.parent != Path("."):
        candidates.append(raw)
    else:
        name = raw.name
        candidates.append(raw)
        candidates.append(review_root / "labels" / name)
        if not name.endswith(".txt"):
            candidates.append(review_root / "labels" / f"{name}.txt")

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()

    raise FileNotFoundError(
        "Could not resolve label. Tried:\n" + "\n".join(str(c) for c in candidates)
    )


def resolve_image(label_path: Path, image_value: str | None, review_root: Path) -> Path:
    if image_value:
        return Path(image_value).expanduser().resolve()

    if label_path.parent.name == "labels":
        inferred = find_image(label_path.parent.parent / "images", label_path.stem)
        if inferred:
            return inferred.resolve()

    inferred = find_image(review_root / "images", label_path.stem)
    if inferred:
        return inferred.resolve()

    raise FileNotFoundError(
        f"Could not infer image for {label_path}. Pass --image explicitly."
    )


def image_size(path: Path) -> tuple[int, int]:
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(f"Could not read image: {path}")
    h, w = image.shape[:2]
    return w, h


def rotate_points(
    coords: np.ndarray,
    image_w: int,
    image_h: int,
    degrees: float,
    direction: str,
    width_scale: float,
    height_scale: float,
    dx: float,
    dy: float,
    clip: bool,
) -> np.ndarray:
    pts = coords.copy().astype(np.float64)
    pts[:, 0] *= image_w
    pts[:, 1] *= image_h

    center = pts.mean(axis=0)

    if width_scale <= 0 or height_scale <= 0:
        raise ValueError("--width-scale and --height-scale must be greater than 0.")

    axis_w = pts[1] - pts[0]
    axis_h = pts[3] - pts[0]

    top_width = np.linalg.norm(pts[1] - pts[0])
    bottom_width = np.linalg.norm(pts[2] - pts[3])
    right_height = np.linalg.norm(pts[2] - pts[1])
    left_height = np.linalg.norm(pts[3] - pts[0])

    box_w = ((top_width + bottom_width) / 2.0) * width_scale
    box_h = ((right_height + left_height) / 2.0) * height_scale

    w_norm = np.linalg.norm(axis_w)
    if w_norm <= 1e-9:
        unit_w = np.array([1.0, 0.0], dtype=np.float64)
    else:
        unit_w = axis_w / w_norm

    # Force height to be perpendicular to width. This keeps the result a true
    # rectangle even if the input points were slightly skewed or previously clipped.
    unit_h = np.array([-unit_w[1], unit_w[0]], dtype=np.float64)
    if np.dot(unit_h, axis_h) < 0:
        unit_h *= -1.0

    # Image y-axis points down. A visual counter-clockwise rotation is negative
    # in standard x-right/y-down pixel coordinates.
    signed_degrees = -degrees if direction == "ccw" else degrees
    theta = math.radians(signed_degrees)
    c = math.cos(theta)
    s = math.sin(theta)
    rot = np.array([[c, -s], [s, c]], dtype=np.float64)

    unit_w = rot @ unit_w
    unit_h = rot @ unit_h

    half_w = box_w / 2.0
    half_h = box_h / 2.0
    rotated = np.array(
        [
            center - half_w * unit_w - half_h * unit_h,
            center + half_w * unit_w - half_h * unit_h,
            center + half_w * unit_w + half_h * unit_h,
            center - half_w * unit_w + half_h * unit_h,
        ],
        dtype=np.float64,
    )

    rotated[:, 0] /= image_w
    rotated[:, 1] /= image_h

    rotated[:, 0] += dx
    rotated[:, 1] += dy

    if clip:
        rotated = fit_rect_inside_bounds(rotated)

    return rotated


def fit_rect_inside_bounds(coords: np.ndarray) -> np.ndarray:
    fitted = coords.copy()

    min_x, max_x = float(fitted[:, 0].min()), float(fitted[:, 0].max())
    min_y, max_y = float(fitted[:, 1].min()), float(fitted[:, 1].max())

    span_x = max_x - min_x
    span_y = max_y - min_y

    if span_x > 1.0 or span_y > 1.0:
        print(
            "WARNING: box is larger than image bounds; leaving points unclipped to avoid a trapezoid.",
            file=sys.stderr,
        )
        return fitted

    shift_x = 0.0
    shift_y = 0.0

    if min_x < 0.0:
        shift_x = -min_x
    elif max_x > 1.0:
        shift_x = 1.0 - max_x

    if min_y < 0.0:
        shift_y = -min_y
    elif max_y > 1.0:
        shift_y = 1.0 - max_y

    fitted[:, 0] += shift_x
    fitted[:, 1] += shift_y
    return fitted


def format_obb_line(class_id: str, coords: np.ndarray) -> str:
    values = " ".join(f"{v:.6f}" for v in coords.reshape(-1))
    return f"{class_id} {values}"


def write_aabb_from_obb(aabb_path: Path, obb_lines: list[str]) -> None:
    out_lines: list[str] = []
    for line in obb_lines:
        parts = line.split()
        if len(parts) != 9:
            continue
        cls = parts[0]
        vals = [float(v) for v in parts[1:]]
        xs = vals[0::2]
        ys = vals[1::2]
        x1, x2 = min(xs), max(xs)
        y1, y2 = min(ys), max(ys)
        xc = (x1 + x2) / 2.0
        yc = (y1 + y2) / 2.0
        bw = x2 - x1
        bh = y2 - y1
        out_lines.append(f"{cls} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")

    aabb_path.parent.mkdir(parents=True, exist_ok=True)
    aabb_path.write_text(("\n".join(out_lines) + "\n") if out_lines else "", encoding="utf-8")


def sync_source_label(
    label_path: Path,
    out_lines: list[str],
    index_rows: dict[Path, dict[str, str]],
) -> None:
    row = index_rows.get(label_path.resolve())
    if not row:
        raise KeyError(f"No index.tsv row found for review label: {label_path}")

    source_label_raw = row.get("source_label")
    if not source_label_raw:
        raise KeyError(f"index.tsv row has no source_label for: {label_path}")

    source_label = Path(source_label_raw).expanduser().resolve()
    source_label.parent.mkdir(parents=True, exist_ok=True)
    source_label.write_text(("\n".join(out_lines) + "\n") if out_lines else "", encoding="utf-8")

    aabb_label = Path(str(source_label).replace("/labels/", "/labels_aabb/"))
    write_aabb_from_obb(aabb_label, out_lines)

    print(f"Synced source label: {source_label}")
    print(f"Synced source AABB:  {aabb_label}")


def main() -> int:
    args = parse_args()

    review_root = args.review_root.expanduser().resolve()
    label_arg = args.label or args.target

    if not label_arg:
        raise SystemExit("Provide a target label/stem or --label.")

    label_path = resolve_label(label_arg, review_root)
    image_path = resolve_image(label_path, args.image, review_root)
    index_rows = read_review_index(review_root / "index.tsv")

    if args.output is None and not args.in_place:
        raise SystemExit("Use --in-place or provide --output.")

    output_path = Path(args.output).expanduser().resolve() if args.output else label_path

    image_w, image_h = image_size(image_path)
    text = label_path.read_text(encoding="utf-8").strip()

    if not text:
        output_path.write_text("", encoding="utf-8")
        if args.aabb_label:
            write_aabb_from_obb(Path(args.aabb_label).expanduser().resolve(), [])
        if args.sync_source:
            sync_source_label(output_path, [], index_rows)
        print(f"Empty label kept: {output_path}")
        return 0

    out_lines: list[str] = []
    for raw in text.splitlines():
        parts = raw.split()
        if len(parts) != 9:
            raise ValueError(f"Expected 9 fields in {label_path}, got {len(parts)}: {raw}")
        cls = parts[0]
        coords = np.array([float(v) for v in parts[1:]], dtype=np.float64).reshape(4, 2)
        rotated = rotate_points(
            coords,
            image_w=image_w,
            image_h=image_h,
            degrees=args.degrees,
            direction=args.direction,
            width_scale=args.width_scale,
            height_scale=args.height_scale,
            dx=args.dx,
            dy=args.dy,
            clip=not args.no_clip,
        )
        out_lines.append(format_obb_line(cls, rotated))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(out_lines) + "\n", encoding="utf-8")

    if args.aabb_label:
        write_aabb_from_obb(Path(args.aabb_label).expanduser().resolve(), out_lines)
    if args.sync_source:
        sync_source_label(output_path, out_lines, index_rows)

    print(f"Wrote: {output_path}")
    if args.aabb_label:
        print(f"Updated AABB: {Path(args.aabb_label).expanduser().resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

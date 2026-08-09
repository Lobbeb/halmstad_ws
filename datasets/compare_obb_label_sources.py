#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
import re
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
FINAL_STEM_RE = re.compile(r"^(?P<route>.+)_(?P<version>v\d+)_(?P<frame>frame_.+)$")
OLD_COLOR = (255,0,90)
NEW_COLOR = (255,255,0)
LINE_WIDTH = 2
CENTER_RADIUS = 2
OUTLINE_WIDTH = 0
OUTLINE_COLOR = (0,0,0)
LINE_ALPHA = 0.9

class HelpFormatter(argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter):
    pass


def parse_rgb_color(value: str) -> tuple[int, int, int]:
    named = {
        "black": (0, 0, 0),
        "dark": (28, 28, 28),
        "gray": (128, 128, 128),
        "white": (255, 255, 255),
        "red": (0, 0, 255),
        "green": (0, 255, 0),
        "blue": (255, 0, 0),
    }
    if value in named:
        return named[value]
    parts = [p.strip() for p in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("Use black, dark, gray, white, red, green, blue, or R,G,B values.")
    try:
        rgb = tuple(int(p) for p in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("R,G,B values must be integers.") from exc
    if any(c < 0 or c > 255 for c in rgb):
        raise argparse.ArgumentTypeError("R,G,B values must be in 0..255.")
    r, g, b = rgb
    return b, g, r


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError("Use true or false.")


def find_image(dataset_root: Path, split: str, stem: str) -> Path | None:
    image_dir = dataset_root / "images" / split
    for ext in IMAGE_EXTS:
        path = image_dir / f"{stem}{ext}"
        if path.exists():
            return path
    return None


def selected_image_stems(image_dir: Path) -> set[str]:
    stems = set()
    suffixes = ("_amcl_vs_sam3",)
    for path in image_dir.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in IMAGE_EXTS:
            continue
        stem = path.stem
        for suffix in suffixes:
            if stem.endswith(suffix):
                stem = stem[: -len(suffix)]
        stems.add(stem)
    return stems


def read_obb(label_path: Path) -> list[np.ndarray]:
    boxes: list[np.ndarray] = []
    text = label_path.read_text(encoding="utf-8").strip()
    if not text:
        return boxes
    for line in text.splitlines():
        parts = line.split()
        if len(parts) != 9:
            continue
        coords = np.array([float(v) for v in parts[1:]], dtype=np.float32).reshape(4, 2)
        boxes.append(coords)
    return boxes


def draw_boxes(image: np.ndarray, boxes: list[np.ndarray], color: tuple[int, int, int]) -> np.ndarray:
    h, w = image.shape[:2]
    out = image.copy()
    layer = out.copy()
    for coords in boxes:
        pts = coords.copy()
        pts[:, 0] *= w
        pts[:, 1] *= h
        pts = pts.astype(np.int32)
        if OUTLINE_WIDTH > 0:
            cv2.polylines(
                layer,
                [pts],
                isClosed=True,
                color=OUTLINE_COLOR,
                thickness=max(1, LINE_WIDTH + 2 * OUTLINE_WIDTH),
            )
        cv2.polylines(layer, [pts], isClosed=True, color=color, thickness=max(1, LINE_WIDTH))
        if CENTER_RADIUS > 0:
            center = pts.mean(axis=0).astype(int)
            if OUTLINE_WIDTH > 0:
                cv2.circle(layer, tuple(center), CENTER_RADIUS + OUTLINE_WIDTH, OUTLINE_COLOR, -1, cv2.LINE_AA)
            cv2.circle(layer, tuple(center), CENTER_RADIUS, color, -1, cv2.LINE_AA)
    if LINE_ALPHA < 1.0:
        cv2.addWeighted(layer, max(0.0, min(1.0, LINE_ALPHA)), out, 1.0 - max(0.0, min(1.0, LINE_ALPHA)), 0, out)
    else:
        out = layer
    return out


def pixel_bounds(image: np.ndarray, boxes: list[np.ndarray], pad_fraction: float) -> tuple[int, int, int, int]:
    h, w = image.shape[:2]
    if not boxes:
        return 0, 0, w, h
    pts = np.vstack(boxes).copy()
    pts[:, 0] *= w
    pts[:, 1] *= h
    x0, y0 = pts.min(axis=0)
    x1, y1 = pts.max(axis=0)
    bw = max(1.0, x1 - x0)
    bh = max(1.0, y1 - y0)
    pad = max(bw, bh) * pad_fraction
    cx = (x0 + x1) * 0.5
    cy = (y0 + y1) * 0.5
    side = max(bw, bh) + 2 * pad
    x0 = int(round(cx - side * 0.5))
    x1 = int(round(cx + side * 0.5))
    y0 = int(round(cy - side * 0.5))
    y1 = int(round(cy + side * 0.5))
    if x0 < 0:
        x1 -= x0
        x0 = 0
    if y0 < 0:
        y1 -= y0
        y0 = 0
    if x1 > w:
        x0 -= x1 - w
        x1 = w
    if y1 > h:
        y0 -= y1 - h
        y1 = h
    return max(0, x0), max(0, y0), min(w, x1), min(h, y1)


def bounds_from_points(
    image_shape: tuple[int, int, int], points_px: np.ndarray, pad_fraction: float
) -> tuple[int, int, int, int]:
    h, w = image_shape[:2]
    if points_px.size == 0:
        return 0, 0, w, h
    x0, y0 = points_px.min(axis=0)
    x1, y1 = points_px.max(axis=0)
    bw = max(1.0, x1 - x0)
    bh = max(1.0, y1 - y0)
    pad = max(bw, bh) * pad_fraction
    cx = (x0 + x1) * 0.5
    cy = (y0 + y1) * 0.5
    side = max(bw, bh) + 2 * pad
    x0 = int(round(cx - side * 0.5))
    x1 = int(round(cx + side * 0.5))
    y0 = int(round(cy - side * 0.5))
    y1 = int(round(cy + side * 0.5))
    if x0 < 0:
        x1 -= x0
        x0 = 0
    if y0 < 0:
        y1 -= y0
        y0 = 0
    if x1 > w:
        x0 -= x1 - w
        x1 = w
    if y1 > h:
        y0 -= y1 - h
        y1 = h
    return max(0, x0), max(0, y0), min(w, x1), min(h, y1)


def common_pixel_bounds(selected, pad_fraction: float) -> tuple[tuple[int, int, int, int], tuple[int, int, int]]:
    shape = selected[0]["image"].shape
    base_h, base_w = shape[:2]
    all_points = []
    for item in selected:
        image = item["image"]
        h, w = image.shape[:2]
        for box in item["old_boxes"] + item["new_boxes"]:
            pts = box.copy()
            pts[:, 0] *= w
            pts[:, 1] *= h
            pts[:, 0] *= base_w / max(1, w)
            pts[:, 1] *= base_h / max(1, h)
            all_points.append(pts)
    if not all_points:
        return (0, 0, base_w, base_h), shape
    return bounds_from_points(shape, np.vstack(all_points), pad_fraction), shape


def scale_bounds(
    bounds: tuple[int, int, int, int], from_shape: tuple[int, int, int], to_shape: tuple[int, int, int]
) -> tuple[int, int, int, int]:
    from_h, from_w = from_shape[:2]
    to_h, to_w = to_shape[:2]
    sx = to_w / max(1, from_w)
    sy = to_h / max(1, from_h)
    x0, y0, x1, y1 = bounds
    return (
        max(0, min(to_w, int(round(x0 * sx)))),
        max(0, min(to_h, int(round(y0 * sy)))),
        max(0, min(to_w, int(round(x1 * sx)))),
        max(0, min(to_h, int(round(y1 * sy)))),
    )


def crop_resize(image: np.ndarray, bounds: tuple[int, int, int, int], tile_size: int) -> np.ndarray:
    x0, y0, x1, y1 = bounds
    crop = image[y0:y1, x0:x1]
    if crop.size == 0:
        crop = image
    return cv2.resize(crop, (tile_size, tile_size), interpolation=cv2.INTER_AREA)


def add_header(image: np.ndarray, title: str, color: tuple[int, int, int]) -> np.ndarray:
    pad = 44
    out = cv2.copyMakeBorder(image, pad, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
    cv2.putText(out, title, (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.82, color, 2, cv2.LINE_AA)
    return out


def add_panel_header(image: np.ndarray, title: str, color: tuple[int, int, int]) -> np.ndarray:
    header_h = 52
    out = cv2.copyMakeBorder(image, header_h, 0, 0, 0, cv2.BORDER_CONSTANT, value=(255, 255, 255))
    cv2.putText(out, title, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.84, color, 2, cv2.LINE_AA)
    return out


def make_grid(tiles: list[np.ndarray], cols: int, gutter: int, bg=(255, 255, 255)) -> np.ndarray:
    if not tiles:
        raise ValueError("No tiles for montage")
    tile_h, tile_w = tiles[0].shape[:2]
    rows = int(np.ceil(len(tiles) / cols))
    canvas_h = rows * tile_h + (rows - 1) * gutter
    canvas_w = cols * tile_w + (cols - 1) * gutter
    canvas = np.full((canvas_h, canvas_w, 3), bg, dtype=np.uint8)
    for idx, tile in enumerate(tiles):
        row = idx // cols
        col = idx % cols
        y = row * (tile_h + gutter)
        x = col * (tile_w + gutter)
        canvas[y : y + tile_h, x : x + tile_w] = tile
    return canvas


def make_paired_grid(
    old_tiles: list[np.ndarray], new_tiles: list[np.ndarray], cols: int, gutter: int, bg=(255, 255, 255)
) -> np.ndarray:
    if not old_tiles or not new_tiles:
        raise ValueError("No tiles for paired montage")
    tile_h, tile_w = old_tiles[0].shape[:2]
    rows = []
    for start in range(0, len(old_tiles), cols):
        for source in (old_tiles, new_tiles):
            canvas_w = cols * tile_w + (cols - 1) * gutter
            row = np.full((tile_h, canvas_w, 3), bg, dtype=np.uint8)
            for col, tile in enumerate(source[start : start + cols]):
                x = col * (tile_w + gutter)
                row[:, x : x + tile_w] = tile
            rows.append(row)
    canvas_h = len(rows) * tile_h + (len(rows) - 1) * gutter
    canvas = np.full((canvas_h, rows[0].shape[1], 3), bg, dtype=np.uint8)
    for idx, row in enumerate(rows):
        y = idx * (tile_h + gutter)
        canvas[y : y + tile_h, : row.shape[1]] = row
    return canvas


def select_visual_row_block(selected: list[dict], visual_row: int, paired_cols: int) -> list[dict]:
    if visual_row <= 0 or not selected:
        return selected
    cols = paired_cols if paired_cols > 0 else len(selected)
    block_idx = (visual_row - 1) // 2
    start = block_idx * cols
    return selected[start : start + cols]


def selection_suffix(selected: list[dict]) -> str:
    routes = sorted({item["route"] for item in selected})
    versions = sorted({item["version"] for item in selected})
    if len(routes) == 1 and len(versions) == 1:
        return f"_{routes[0]}_{versions[0]}"
    if len(routes) == 1:
        return f"_{routes[0]}"
    return "_mixed"


def selection_route_name(selected: list[dict]) -> str:
    routes = sorted({item["route"] for item in selected})
    versions = sorted({item["version"] for item in selected})
    if len(routes) == 1 and len(versions) == 1:
        return f"{routes[0]}_{versions[0]}"
    if len(routes) == 1:
        return routes[0]
    return "mixed"


def simple_montage_name(plottype: str, selected: list[dict], args, candidate_idx: int | None = None) -> str:
    outline = int(args.outline_width > 0)
    name = f"{plottype}_{selection_route_name(selected)}_{outline}"
    if args.visual_row_in_title and args.visual_row > 0:
        name = f"{plottype}_{selection_route_name(selected)}_vr{args.visual_row}_{outline}"
    if candidate_idx is not None:
        name = f"{name}_{candidate_idx:02d}"
    return f"{name}.jpg"


def find_old_label(old_root: Path, route: str, version: str, frame_stem: str) -> Path | None:
    label_root = old_root / route / version / "labels"
    if not label_root.exists():
        return None
    matches = sorted(label_root.rglob(f"{frame_stem}.txt"))
    return matches[0] if matches else None


def iter_final_labels(final_root: Path, splits: list[str]):
    for split in splits:
        label_dir = final_root / "labels" / split
        if not label_dir.exists():
            continue
        for path in sorted(label_dir.glob("*.txt")):
            match = FINAL_STEM_RE.match(path.stem)
            if match:
                yield split, path, match.groupdict()


def obb_difference(old_boxes: list[np.ndarray], new_boxes: list[np.ndarray]) -> float:
    if not old_boxes or not new_boxes:
        return -1.0
    old = old_boxes[0]
    new = new_boxes[0]
    old_center = old.mean(axis=0)
    new_center = new.mean(axis=0)
    center_delta = float(np.linalg.norm(old_center - new_center))
    corner_delta = float(np.mean(np.linalg.norm(old - new, axis=1)))
    return center_delta + corner_delta


def frame_sort_key(frame_stem: str) -> tuple[int, int]:
    match = re.match(r"frame_(\d+)_(\d+)$", frame_stem)
    if not match:
        return (0, 0)
    return (int(match.group(1)), int(match.group(2)))


def build_tiles(
    selected,
    tile_size: int,
    crop_pad: float,
    shared_bounds: tuple[int, int, int, int] | None = None,
    shared_shape: tuple[int, int, int] | None = None,
):
    old_tiles = []
    new_tiles = []
    for item in selected:
        image = item["image"]
        old_boxes = item["old_boxes"]
        new_boxes = item["new_boxes"]
        if shared_bounds is not None and shared_shape is not None:
            bounds = scale_bounds(shared_bounds, shared_shape, image.shape)
        else:
            bounds = pixel_bounds(image, old_boxes + new_boxes, crop_pad)
        old_tiles.append(crop_resize(draw_boxes(image, old_boxes, OLD_COLOR), bounds, tile_size))
        new_tiles.append(crop_resize(draw_boxes(image, new_boxes, NEW_COLOR), bounds, tile_size))
    return old_tiles, new_tiles


def write_montages(selected, output_dir: Path, args) -> int:
    if not selected:
        return 0
    style = args.montage_style
    if style in ("paired", "time_series", "all") and args.visual_row > 0:
        selected = select_visual_row_block(selected, args.visual_row, args.paired_cols)
        if not selected:
            return 0
    shared_bounds = None
    shared_shape = None
    if style == "time_series" and args.time_series_shared_crop:
        shared_bounds, shared_shape = common_pixel_bounds(selected, args.crop_pad)
    old_tiles, new_tiles = build_tiles(
        selected,
        args.tile_size,
        args.crop_pad,
        shared_bounds=shared_bounds,
        shared_shape=shared_shape,
    )
    bg = args.bg_color
    written = 0

    def write_image(name: str, image: np.ndarray) -> None:
        nonlocal written
        path = output_dir / name
        cv2.imwrite(str(path), image)
        written += 1
        print(f"Montage:       {path}")

    if style in ("side_by_side", "all"):
        old_panel = make_grid(old_tiles, args.montage_cols, args.tile_gutter, bg=bg)
        new_panel = make_grid(new_tiles, args.montage_cols, args.tile_gutter, bg=bg)
        if not args.no_headers:
            old_panel = add_panel_header(old_panel, "Projection from pose/camera model", OLD_COLOR)
            new_panel = add_panel_header(new_panel, "SAM3 mask-to-OBB auto-annotation", NEW_COLOR)
        spacer = np.full((old_panel.shape[0], 28, 3), bg, dtype=np.uint8)
        write_image(args.montage_name, np.hstack([old_panel, spacer, new_panel]))

    if style in ("separate", "all"):
        old_grid = make_grid(old_tiles, args.montage_cols, args.tile_gutter, bg=bg)
        new_grid = make_grid(new_tiles, args.montage_cols, args.tile_gutter, bg=bg)
        if not args.no_headers:
            old_grid = add_panel_header(old_grid, "Projection from pose/camera model", OLD_COLOR)
            new_grid = add_panel_header(new_grid, "SAM3 mask-to-OBB auto-annotation", NEW_COLOR)
        write_image(
            "obb_projection_amcl_3x3.jpg",
            old_grid,
        )
        write_image(
            "obb_sam3_mask_to_obb_3x3.jpg",
            new_grid,
        )

    if style in ("paired", "time_series", "all"):
        paired_cols = args.paired_cols if args.paired_cols > 0 else len(old_tiles)
        paired = make_paired_grid(old_tiles, new_tiles, paired_cols, args.tile_gutter, bg=bg)
        if not args.no_headers:
            paired = add_panel_header(paired, "Rows alternate: projection from pose/camera model, then SAM3 mask-to-OBB", OLD_COLOR)
        if style == "time_series":
            name = args.montage_name if args.montage_name != "obb_amcl_vs_sam3_montage.jpg" else simple_montage_name("time_series", selected, args)
        else:
            name = simple_montage_name("paired", selected, args)
        write_image(name, paired)

    return written


def select_time_series(comparable, length: int):
    windows = select_time_series_windows(comparable, length, 1)
    return windows[0] if windows else []


def select_time_series_windows(comparable, length: int, max_windows: int):
    return select_time_series_windows_for_groups(comparable, length, max_windows, group_by="route_version")


def select_time_series_windows_for_groups(comparable, length: int, max_windows: int, group_by: str):
    groups: dict[tuple[str, str], list[dict]] = {}
    for item in comparable:
        if group_by == "route":
            key = (item["route"], "")
        else:
            key = (item["route"], item["version"])
        groups.setdefault(key, []).append(item)

    candidates = []
    for key, items in groups.items():
        items = sorted(items, key=lambda item: frame_sort_key(item["frame_stem"]))
        if len(items) < length:
            continue
        for idx in range(0, len(items) - length + 1):
            window = items[idx : idx + length]
            score = sum(item["score"] for item in window) / float(length)
            candidates.append((score, key, idx, idx + length, window))

    candidates.sort(key=lambda item: item[0], reverse=True)
    selected = []
    used_ranges: dict[tuple[str, str], list[tuple[int, int]]] = {}
    for _score, key, start, end, window in candidates:
        overlaps = any(start < used_end and used_start < end for used_start, used_end in used_ranges.get(key, []))
        if overlaps:
            continue
        selected.append(window)
        used_ranges.setdefault(key, []).append((start, end))
        if len(selected) >= max_windows:
            break
    return selected


def select_best_time_series_per_group(comparable, length: int, group_by: str):
    groups: dict[tuple[str, str], list[dict]] = {}
    for item in comparable:
        if group_by == "route":
            key = (item["route"], "")
        else:
            key = (item["route"], item["version"])
        groups.setdefault(key, []).append(item)

    selected = []
    for _key, items in sorted(groups.items()):
        windows = select_time_series_windows_for_groups(items, length, 1, group_by=group_by)
        if windows:
            selected.append(windows[0])
    return selected


def main() -> int:
    global OLD_COLOR, NEW_COLOR, LINE_WIDTH, CENTER_RADIUS, OUTLINE_WIDTH, OUTLINE_COLOR, LINE_ALPHA
    parser = argparse.ArgumentParser(
        description="Create old AMCL-pose OBB vs SAM3 mask-to-OBB label comparison figures.",
        formatter_class=HelpFormatter,
        epilog="""Column/layout guide:
  --montage-cols controls grid columns for side_by_side, separate, and all.
    Example: --limit 9 --montage-cols 3 gives a 3x3 AMCL panel and a 3x3 SAM3 panel.

  --paired-cols controls columns for paired and time_series.
    These styles alternate AMCL/projection and SAM3 rows for each block of columns.
    Example: --series-length 5 --paired-cols 5 gives a 2x5 figure.
    Example: --series-length 10 --paired-cols 5 gives AMCL row 1, SAM3 row 1, AMCL row 2, SAM3 row 2.
    Example: add --visual-row 3 to replot only the frames from visual rows 3 and 4.
    Use --paired-cols 0 to put every selected frame in one long row.
""",
    )
    parser.add_argument("--final-root", default="datasets/final/baylands_sam3_obb_all")
    parser.add_argument("--old-root", default="datasets/baylands_leader_routes")
    parser.add_argument("--output", default="datasets/review/obb_amcl_vs_sam3")
    parser.add_argument("--image-dir", default="", help="Optional directory of chosen images; only matching image stems are plotted.")
    parser.add_argument("--splits", default="val,test,train")
    parser.add_argument("--limit", type=int, default=12)
    parser.add_argument("--random", action="store_true", help="Sample random comparable frames across all selected splits.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--route", default="", help="Optional route filter, for example parkinglot_east.")
    parser.add_argument("--exclude-routes", default="", help="Comma-separated routes to skip, for example art,playground.")
    parser.add_argument("--version", default="", help="Optional version filter, for example v3.")
    parser.add_argument("--rank-difference", action="store_true", help="Prefer examples where old and SAM3 OBBs differ.")
    parser.add_argument("--montage", action="store_true", help="Write one thesis-style old-vs-SAM3 montage.")
    parser.add_argument(
        "--montage-style",
        choices=["side_by_side", "separate", "paired", "time_series", "all"],
        default="side_by_side",
        help=(
            "Figure layout. side_by_side places AMCL and SAM3 grids next to each other; "
            "separate writes one AMCL grid and one SAM3 grid; paired alternates AMCL and "
            "SAM3 rows by --paired-cols blocks; time_series does the same as paired but "
            "selects consecutive frames."
        ),
    )
    parser.add_argument("--montage-name", default="obb_amcl_vs_sam3_montage.jpg")
    parser.add_argument("--montage-cols", type=int, default=3, help="Number of columns in side_by_side/separate grid panels.")
    parser.add_argument(
        "--paired-cols",
        type=int,
        default=0,
        help="Number of columns in paired/time_series row panels. 0 means all selected frames in one row.",
    )
    parser.add_argument(
        "--visual-row",
        type=int,
        default=0,
        help=(
            "For paired/time_series, replot only the frame block containing this visual row. "
            "Odd/even rows map to the same AMCL/SAM3 comparison block."
        ),
    )
    parser.add_argument(
        "--visual-row-in-title",
        type=parse_bool,
        default=False,
        help="Include visual row in generated filename/title when --visual-row is used.",
    )
    parser.add_argument("--series-length", type=int, default=10, help="Number of consecutive frames to use for each time_series candidate.")
    parser.add_argument("--series-candidates", type=int, default=1, help="For time_series montage, write this many non-overlapping candidate sequences.")
    parser.add_argument(
        "--time-series-shared-crop",
        type=parse_bool,
        default=True,
        help="For time_series montage, use one crop/zoom for the whole sequence.",
    )
    parser.add_argument(
        "--series-group-by",
        choices=["global", "route", "route_version"],
        default="global",
        help="For time_series montage, choose top global candidates or one best sequence per route/route-version.",
    )
    parser.add_argument("--no-headers", action="store_true", help="Omit AMCL/SAM3 text headers from generated comparison images.")
    parser.add_argument("--bg-color", type=parse_rgb_color, default=(28, 28, 28), help="Montage gutter/spacer color: dark, black, gray, white, or R,G,B.")
    parser.add_argument("--amcl-color", type=parse_rgb_color, default=OLD_COLOR, help="AMCL/projection OBB color: name or R,G,B.")
    parser.add_argument("--sam3-color", type=parse_rgb_color, default=NEW_COLOR, help="SAM3 OBB color: name or R,G,B.")
    parser.add_argument("--line-width", type=int, default=LINE_WIDTH, help="OBB outline thickness in pixels.")
    parser.add_argument("--center-radius", type=int, default=CENTER_RADIUS, help="OBB center dot radius in pixels. 0 disables the dot.")
    parser.add_argument("--outline-width", type=int, default=OUTLINE_WIDTH, help="Extra outline stroke width behind OBB line. 0 disables outline.")
    parser.add_argument("--outline-color", type=parse_rgb_color, default=OUTLINE_COLOR, help="Outline stroke color: name or R,G,B.")
    parser.add_argument("--line-alpha", type=float, default=LINE_ALPHA, help="OBB overlay opacity in 0..1. Lower values look visually thinner.")
    parser.add_argument("--tile-size", type=int, default=260)
    parser.add_argument("--tile-gutter", type=int, default=10)
    parser.add_argument("--crop-pad", type=float, default=2.0)
    args = parser.parse_args()

    OLD_COLOR = args.amcl_color
    NEW_COLOR = args.sam3_color
    LINE_WIDTH = args.line_width
    CENTER_RADIUS = args.center_radius
    OUTLINE_WIDTH = args.outline_width
    OUTLINE_COLOR = args.outline_color
    LINE_ALPHA = args.line_alpha

    final_root = Path(args.final_root).expanduser().resolve()
    old_root = Path(args.old_root).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    splits = [s.strip() for s in args.splits.split(",") if s.strip()]
    exclude_routes = {s.strip() for s in args.exclude_routes.split(",") if s.strip()}
    chosen_stems = selected_image_stems(Path(args.image_dir).expanduser().resolve()) if args.image_dir else set()
    written = 0
    checked = 0
    missing_old = 0
    missing_image = 0
    comparable = []

    candidates = list(iter_final_labels(final_root, splits))
    if args.random:
        rng = random.Random(args.seed)
        rng.shuffle(candidates)

    for split, final_label, parts in candidates:
        if chosen_stems and final_label.stem not in chosen_stems:
            continue
        route = parts["route"]
        version = parts["version"]
        frame_stem = parts["frame"]
        if route in exclude_routes:
            continue
        if args.route and route != args.route:
            continue
        if args.version and version != args.version:
            continue
        checked += 1

        old_label = find_old_label(old_root, route, version, frame_stem)
        if old_label is None:
            missing_old += 1
            continue

        image_path = find_image(final_root, split, final_label.stem)
        if image_path is None:
            missing_image += 1
            continue

        image = cv2.imread(str(image_path))
        if image is None:
            missing_image += 1
            continue

        old_boxes = read_obb(old_label)
        new_boxes = read_obb(final_label)
        comparable.append(
            {
                "score": obb_difference(old_boxes, new_boxes),
                "image": image,
                "old_boxes": old_boxes,
                "new_boxes": new_boxes,
                "stem": final_label.stem,
                "image_path": str(image_path),
                "old_label": str(old_label),
                "sam3_label": str(final_label),
                "route": route,
                "version": version,
                "frame_stem": frame_stem,
            }
        )
        if args.montage:
            if args.limit > 0 and len(comparable) >= args.limit and not args.rank_difference:
                break
            continue

        old_vis = draw_boxes(image, old_boxes, OLD_COLOR)
        new_vis = draw_boxes(image, new_boxes, NEW_COLOR)
        if not args.no_headers:
            old_vis = add_header(old_vis, "AMCL pose OBB", OLD_COLOR)
            new_vis = add_header(new_vis, "SAM3 mask-to-OBB", NEW_COLOR)
        combined = np.hstack([old_vis, new_vis])
        out_path = output_dir / f"{final_label.stem}_amcl_vs_sam3.jpg"
        cv2.imwrite(str(out_path), combined)
        written += 1
        if args.limit > 0 and written >= args.limit:
            break

    if args.montage:
        if args.montage_style == "time_series":
            series_length = args.series_length if args.series_length > 0 else (args.limit if args.limit > 0 else 10)
            if args.series_group_by == "global":
                windows = select_time_series_windows(comparable, series_length, max(1, args.series_candidates))
            else:
                windows = select_best_time_series_per_group(comparable, series_length, args.series_group_by)
            for idx, window in enumerate(windows, start=1):
                first = window[0]
                candidate_args = argparse.Namespace(**vars(args))
                if (args.series_candidates > 1 or args.series_group_by != "global") and args.montage_name == "obb_amcl_vs_sam3_montage.jpg":
                    candidate_args.montage_name = simple_montage_name("time_series", window, candidate_args, idx)
                write_montages(window, output_dir, candidate_args)
            selected = [item for window in windows for item in window]
        else:
            if args.rank_difference:
                comparable.sort(key=lambda item: item["score"], reverse=True)
            selected = comparable[: args.limit if args.limit > 0 else len(comparable)]
        if selected and args.montage_style != "time_series":
            write_montages(selected, output_dir, args)
        written = len(selected)

    print(f"Checked:       {checked}")
    print(f"Written:       {written}")
    print(f"Missing old:   {missing_old}")
    print(f"Missing image: {missing_image}")
    print(f"Output:        {output_dir}")
    return 0 if written else 1


if __name__ == "__main__":
    raise SystemExit(main())

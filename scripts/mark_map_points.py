#!/usr/bin/env python3
import argparse
import csv
import re
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def parse_yaml_map(yaml_path: Path):
    image_value = None
    resolution = None
    origin = None

    with yaml_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("image:"):
                image_value = line.split(":", 1)[1].strip()
            elif line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
            elif line.startswith("origin:"):
                values = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", line)
                if len(values) >= 2:
                    origin = (float(values[0]), float(values[1]))

    if resolution is None or origin is None:
        raise SystemExit(f"Failed to read resolution/origin from {yaml_path}")

    return image_value, resolution, origin


def resolve_image_path(yaml_path: Path, explicit_image: str | None):
    if explicit_image:
      return Path(explicit_image).expanduser().resolve()

    image_value, _, _ = parse_yaml_map(yaml_path)
    candidates = []
    if image_value:
        candidates.append((yaml_path.parent / image_value).resolve())
    candidates.append(yaml_path.with_suffix(".pgm"))
    candidates.append(yaml_path.with_suffix(".png"))
    candidates.append(yaml_path.with_suffix(".jpg"))
    candidates.append(yaml_path.with_suffix(".jpeg"))

    for candidate in candidates:
        if candidate.exists():
            return candidate

    raise SystemExit(
        f"Could not resolve an image for {yaml_path}. "
        "Pass --image explicitly if you want to mark a scaled screenshot."
    )


def world_to_pixel(x, y, origin_x, origin_y, resolution, image_height):
    px = (x - origin_x) / resolution
    py = image_height - ((y - origin_y) / resolution)
    return px, py


def world_to_pixel_bounds(x, y, x_min, y_min, x_max, y_max, image_width, image_height):
    if x_max == x_min or y_max == y_min:
        raise SystemExit("Invalid world bounds: zero width or height")

    px = ((x - x_min) / (x_max - x_min)) * image_width
    py = ((y_max - y) / (y_max - y_min)) * image_height
    return px, py


def load_control_points_csv(path: Path):
    world_points = []
    pixel_points = []

    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"world_x", "world_y", "pixel_x", "pixel_y"}
        if not reader.fieldnames or not required.issubset(set(reader.fieldnames)):
            raise SystemExit(
                f"{path} must be a CSV with columns: world_x, world_y, pixel_x, pixel_y"
            )
        for row in reader:
            try:
                world_points.append([float(row["world_x"]), float(row["world_y"])])
                pixel_points.append([float(row["pixel_x"]), float(row["pixel_y"])])
            except (TypeError, ValueError) as exc:
                raise SystemExit(f"Invalid control-point row in {path}: {row}") from exc

    if len(world_points) < 4:
        raise SystemExit("Need at least 4 control points for a perspective screenshot")

    return np.array(world_points, dtype=np.float32), np.array(pixel_points, dtype=np.float32)


def build_world_to_pixel_homography(control_points_file: Path):
    src_points, dst_points = load_control_points_csv(control_points_file)
    homography, mask = cv2.findHomography(src_points, dst_points, method=0)
    if homography is None:
        raise SystemExit(f"Failed to compute homography from {control_points_file}")
    return homography


def world_to_pixel_homography(x, y, homography):
    point = np.array([[[x, y]]], dtype=np.float32)
    projected = cv2.perspectiveTransform(point, homography)
    px = float(projected[0][0][0])
    py = float(projected[0][0][1])
    return px, py


def main():
    parser = argparse.ArgumentParser(
        description="Overlay world-coordinate points from a manifest onto a map image."
    )
    parser.add_argument("--reference-yaml", help="Map YAML with origin/resolution")
    parser.add_argument("--image", help="Explicit image to annotate")
    parser.add_argument(
        "--world-bounds",
        help="Axis-aligned screenshot bounds as x_min,y_min,x_max,y_max in Gazebo world meters",
    )
    parser.add_argument(
        "--control-points-file",
        help="CSV with world_x,world_y,pixel_x,pixel_y to project onto a rotated/perspective screenshot",
    )
    parser.add_argument("--manifest", required=True, help="CSV manifest with x/y columns")
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument("--state-prefix", default="", help="Only plot states starting with this prefix")
    parser.add_argument("--radius", type=int, default=10, help="Marker radius in pixels")
    args = parser.parse_args()

    manifest_path = Path(args.manifest).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    yaml_path = Path(args.reference_yaml).expanduser().resolve() if args.reference_yaml else None

    projection_mode_count = sum(
        1 for value in (args.reference_yaml, args.world_bounds, args.control_points_file) if value
    )
    if projection_mode_count == 0:
        raise SystemExit("Provide one of: --reference-yaml, --world-bounds, or --control-points-file")
    if projection_mode_count > 1:
        raise SystemExit("Use only one projection mode at a time")
    if (args.world_bounds or args.control_points_file) and not args.image:
        raise SystemExit("Screenshot modes require --image")

    if yaml_path is not None:
        image_value, resolution, origin = parse_yaml_map(yaml_path)
        image_path = resolve_image_path(yaml_path, args.image)
        origin_x, origin_y = origin
        projection_mode = "yaml"
    else:
        image_value = None
        resolution = None
        origin_x = None
        origin_y = None
        image_path = Path(args.image).expanduser().resolve()
        if args.world_bounds:
            projection_mode = "bounds"
            try:
                x_min_s, y_min_s, x_max_s, y_max_s = [v.strip() for v in args.world_bounds.split(",")]
                x_min = float(x_min_s)
                y_min = float(y_min_s)
                x_max = float(x_max_s)
                y_max = float(y_max_s)
            except Exception as exc:
                raise SystemExit(
                    "--world-bounds must look like x_min,y_min,x_max,y_max"
                ) from exc
        else:
            projection_mode = "homography"
            control_points_path = Path(args.control_points_file).expanduser().resolve()
            homography = build_world_to_pixel_homography(control_points_path)

    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    plotted = 0
    skipped = []

    with manifest_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            state_name = row.get("state_name", "").strip()
            if args.state_prefix and not state_name.startswith(args.state_prefix):
                continue

            try:
                x = float(row["x"])
                y = float(row["y"])
            except (KeyError, TypeError, ValueError):
                skipped.append((state_name or "<unknown>", "bad coordinates"))
                continue

            if projection_mode == "yaml":
                px, py = world_to_pixel(x, y, origin_x, origin_y, resolution, image.height)
            elif projection_mode == "bounds":
                px, py = world_to_pixel_bounds(
                    x, y, x_min, y_min, x_max, y_max, image.width, image.height
                )
            else:
                px, py = world_to_pixel_homography(x, y, homography)

            if px < 0 or py < 0 or px >= image.width or py >= image.height:
                skipped.append((state_name, f"outside image at ({px:.1f}, {py:.1f})"))
                continue

            left = px - args.radius
            top = py - args.radius
            right = px + args.radius
            bottom = py + args.radius
            draw.ellipse((left, top, right, bottom), outline=(255, 0, 0), width=3, fill=(255, 220, 0))
            draw.line((px - args.radius - 4, py, px + args.radius + 4, py), fill=(255, 0, 0), width=2)
            draw.line((px, py - args.radius - 4, px, py + args.radius + 4), fill=(255, 0, 0), width=2)
            label = state_name or row.get("index", "?")
            text_pos = (px + args.radius + 6, py - args.radius - 6)
            draw.text(text_pos, label, fill=(255, 0, 0), font=font)
            plotted += 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)

    if projection_mode == "yaml":
        print(f"reference_yaml={yaml_path}")
    elif projection_mode == "bounds":
        print(f"world_bounds={x_min},{y_min},{x_max},{y_max}")
    else:
        print(f"control_points_file={control_points_path}")
    print(f"reference_image={image_path}")
    if image_value:
        print(f"yaml_image_field={image_value}")
    print(f"output={output_path}")
    print(f"plotted={plotted}")
    print(f"skipped={len(skipped)}")
    for name, reason in skipped:
        print(f"skip: {name}: {reason}")


if __name__ == "__main__":
    main()

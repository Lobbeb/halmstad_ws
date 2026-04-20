#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image


TRINARY_VALUES = (0, 205, 254)


def parse_simple_yaml(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip()
    return data


def to_trinary(img: Image.Image) -> Image.Image:
    gray = img.convert("L")
    lut = []
    for i in range(256):
        nearest = min(TRINARY_VALUES, key=lambda candidate: abs(candidate - i))
        lut.append(nearest)
    return gray.point(lut, mode="L")


def build_default_output_yaml(source_yaml: Path, target_resolution: float) -> Path:
    resolution_cm = int(round(target_resolution * 100))
    return source_yaml.with_name(f"{source_yaml.stem}_nav_{resolution_cm}cm.yaml")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create a coarser navigation map variant from a full-resolution map YAML."
    )
    parser.add_argument("--source-yaml", required=True, help="Input map YAML path")
    parser.add_argument(
        "--output-yaml",
        help="Output YAML path. Defaults to '<source>_nav_<resolution_cm>cm.yaml'",
    )
    parser.add_argument(
        "--factor",
        type=int,
        default=4,
        help="Integer downsample factor. Default: 4",
    )
    args = parser.parse_args()

    if args.factor < 1:
        raise SystemExit("--factor must be >= 1")

    source_yaml = Path(args.source_yaml).expanduser().resolve()
    yaml_data = parse_simple_yaml(source_yaml)

    if "image" not in yaml_data or "resolution" not in yaml_data:
        raise SystemExit(f"{source_yaml} is missing required 'image' or 'resolution' fields")

    source_image = (source_yaml.parent / yaml_data["image"]).resolve()
    source_resolution = float(yaml_data["resolution"])
    target_resolution = source_resolution * args.factor

    output_yaml = (
        Path(args.output_yaml).expanduser().resolve()
        if args.output_yaml
        else build_default_output_yaml(source_yaml, target_resolution).resolve()
    )
    output_image = output_yaml.with_suffix(".pgm")

    Image.MAX_IMAGE_PIXELS = None
    img = Image.open(source_image)
    width = max(1, img.size[0] // args.factor)
    height = max(1, img.size[1] // args.factor)
    resized = img.resize((width, height), resample=Image.Resampling.NEAREST)
    cleaned = to_trinary(resized)

    output_image.parent.mkdir(parents=True, exist_ok=True)
    cleaned.save(output_image)

    output_yaml.write_text(
        "\n".join(
            [
                f"image: {output_image.name}",
                f"mode: {yaml_data.get('mode', 'trinary')}",
                f"resolution: {target_resolution:.3f}",
                f"origin: {yaml_data.get('origin', '[0.0, 0.0, 0.0]')}",
                f"negate: {yaml_data.get('negate', '0')}",
                f"occupied_thresh: {yaml_data.get('occupied_thresh', '0.65')}",
                f"free_thresh: {yaml_data.get('free_thresh', '0.196')}",
                "",
            ]
        ),
        encoding="utf-8",
    )

    print(f"[make_nav_map] Source image: {source_image}")
    print(f"[make_nav_map] Output image: {output_image}")
    print(f"[make_nav_map] Output yaml: {output_yaml}")
    print(f"[make_nav_map] Source resolution: {source_resolution:.3f} m/pixel")
    print(f"[make_nav_map] Output resolution: {target_resolution:.3f} m/pixel")
    print(f"[make_nav_map] Output size: {width}x{height}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

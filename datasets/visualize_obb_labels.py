#!/usr/bin/env python3
from pathlib import Path
import argparse
import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def find_image(image_dir: Path, stem: str) -> Path | None:
    for ext in IMAGE_EXTS:
        p = image_dir / f"{stem}{ext}"
        if p.exists():
            return p
    return None

def parse_color(value: str) -> tuple[int, int, int]:
    colors = {
        "green": (0, 255, 0),
        "red": (0, 0, 255),
        "blue": (255, 0, 0),
        "yellow": (0, 255, 255),
        "cyan": (255, 255, 0),
        "magenta": (255, 0, 255),
        "white": (255, 255, 255),
        "black": (0, 0, 0),
        "orange": (0, 165, 255),
        "purple": (128, 0, 128),
    }

    value = value.strip().lower()

    if value in colors:
        return colors[value]

    parts = [p.strip() for p in value.split(",")]
    if len(parts) == 3:
        try:
            b, g, r = [int(p) for p in parts]
        except ValueError:
            raise ValueError(f"Invalid color: {value}")

        if all(0 <= c <= 255 for c in (b, g, r)):
            return (b, g, r)

    raise ValueError(
        f"Invalid color '{value}'. Use a name like green/red/blue or B,G,R like 0,255,0."
    )

def draw_obb(image, label_path: Path, color: tuple[int, int, int]):
    h, w = image.shape[:2]

    text = label_path.read_text(encoding="utf-8").strip()

    if not text:
        cv2.putText(
            image,
            "EMPTY LABEL",
            (15, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return image

    for line in text.splitlines():
        parts = line.split()

        if len(parts) != 9:
            cv2.putText(
                image,
                f"BAD LABEL NF={len(parts)}",
                (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            continue

        cls = parts[0]
        coords = np.array([float(x) for x in parts[1:]], dtype=np.float32).reshape(4, 2)

        pts = coords.copy()
        pts[:, 0] *= w
        pts[:, 1] *= h
        pts = pts.astype(np.int32)

        cv2.polylines(image, [pts], isClosed=True, color=color, thickness=2)

        x, y = pts[0]
        cv2.putText(
            image,
            f"class {cls}",
            (int(x), max(20, int(y) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )

    return image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", 
                        default="/home/ruben/halmstad_ws/datasets/tmp_datasets/sam_work/inputs/baylands_super_75_15_10_mini10/images/train", 
                        required=False
                        )
    parser.add_argument("--labels",
                         required=True
                         )
    parser.add_argument("--output", 
                        default="/home/ruben/halmstad_ws/datasets/tmp_datasets/sam_work/review/debug_comparison", 
                        required=False,
                        )         
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument(
        "--color",
        default="green",
        help="OBB color: green, red, blue, yellow, cyan, magenta, white, black, or B,G,R like 0,255,0.",
    )
    args = parser.parse_args()

    obb_color = parse_color(args.color)

    image_dir = Path(args.images).expanduser().resolve()
    label_dir = Path(args.labels).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()

    output_dir.mkdir(parents=True, exist_ok=True)

    labels = sorted(label_dir.glob("*.txt"))

    if args.limit > 0:
        labels = labels[:args.limit]

    written = 0
    missing = 0

    for label in labels:
        img_path = find_image(image_dir, label.stem)

        if img_path is None:
            print(f"Missing image for label: {label.name}")
            missing += 1
            continue

        img = cv2.imread(str(img_path))

        if img is None:
            print(f"Could not read image: {img_path}")
            missing += 1
            continue

        vis = draw_obb(img, label, obb_color)
        out_path = output_dir / img_path.name
        cv2.imwrite(str(out_path), vis)
        written += 1

    print("Done")
    print("----")
    print(f"Images written: {written}")
    print(f"Missing:        {missing}")
    print(f"Output:         {output_dir}")


if __name__ == "__main__":
    main()

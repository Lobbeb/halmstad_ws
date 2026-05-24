#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import argparse
import time

import cv2
import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def obb_to_xyxy(obb_item) -> list[float]:
    pts = obb_item.xyxyxyxy[0]

    if isinstance(pts, torch.Tensor):
        pts = pts.detach().cpu().numpy()

    x1 = float(pts[:, 0].min())
    y1 = float(pts[:, 1].min())
    x2 = float(pts[:, 0].max())
    y2 = float(pts[:, 1].max())

    return [x1, y1, x2, y2]


def get_detector_class_id(obb_item) -> int:
    return int(obb_item.cls[0].item())


def order_points(points: np.ndarray) -> np.ndarray:
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]

    start = np.argmin(ordered[:, 0] + ordered[:, 1])
    ordered = np.roll(ordered, -start, axis=0)

    return ordered


def mask_to_minarea_obb(mask: np.ndarray) -> list[float] | None:
    mask = mask.astype(np.uint8)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)

    if len(contour) < 3:
        return None

    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect).astype(np.float32)

    h, w = mask.shape[:2]
    box[:, 0] /= w
    box[:, 1] /= h

    box = np.clip(box, 0.0, 1.0)
    box = order_points(box)

    return box.reshape(-1).tolist()

def polygon_to_minarea_obb(segment: np.ndarray) -> list[float] | None:
    pts = np.asarray(segment, dtype=np.float32)

    if pts.ndim != 2 or pts.shape[0] < 3 or pts.shape[1] != 2:
        return None

    rect = cv2.minAreaRect(pts)
    box = cv2.boxPoints(rect).astype(np.float32)

    box = np.clip(box, 0.0, 1.0)
    box = order_points(box)

    return box.reshape(-1).tolist()


def mask_to_hybrid_obb(mask: np.ndarray, detector_obb) -> list[float] | None:
    """
    Use SAM mask for center/extent, but use detector OBB for orientation.
    Useful when minAreaRect becomes too axis-aligned.
    """
    mask_bool = mask.astype(bool)
    ys, xs = np.where(mask_bool)

    if len(xs) < 3:
        return None

    h, w = mask.shape[:2]

    mask_pts = np.stack([xs.astype(np.float32), ys.astype(np.float32)], axis=1)

    det_pts = detector_obb.xyxyxyxy[0]
    if isinstance(det_pts, torch.Tensor):
        det_pts = det_pts.detach().cpu().numpy()
    det_pts = det_pts.astype(np.float32)

    edges = [
        det_pts[1] - det_pts[0],
        det_pts[2] - det_pts[1],
        det_pts[3] - det_pts[2],
        det_pts[0] - det_pts[3],
    ]

    lengths = [np.linalg.norm(e) for e in edges]
    axis_u = edges[int(np.argmax(lengths))]
    norm = np.linalg.norm(axis_u)

    if norm < 1e-6:
        return None

    axis_u = axis_u / norm
    axis_v = np.array([-axis_u[1], axis_u[0]], dtype=np.float32)

    center = mask_pts.mean(axis=0)

    rel = mask_pts - center
    proj_u = rel @ axis_u
    proj_v = rel @ axis_v

    min_u, max_u = proj_u.min(), proj_u.max()
    min_v, max_v = proj_v.min(), proj_v.max()

    corners = np.array(
        [
            center + min_u * axis_u + min_v * axis_v,
            center + max_u * axis_u + min_v * axis_v,
            center + max_u * axis_u + max_v * axis_v,
            center + min_u * axis_u + max_v * axis_v,
        ],
        dtype=np.float32,
    )

    corners[:, 0] /= w
    corners[:, 1] /= h
    corners = np.clip(corners, 0.0, 1.0)
    corners = order_points(corners)

    return corners.reshape(-1).tolist()


def mask_to_obb(mask_or_segment: np.ndarray, detector_obb, mode: str) -> list[float] | None:
    arr = np.asarray(mask_or_segment)

    # Ultralytics polygon directly, normalized Nx2.
    if arr.ndim == 2 and arr.shape[1] == 2:
        return polygon_to_minarea_obb(arr)

    if mode == "minarea":
        return mask_to_minarea_obb(arr)

    if mode == "hybrid":
        obb = mask_to_hybrid_obb(arr, detector_obb)
        if obb is not None:
            return obb
        return mask_to_minarea_obb(arr)

    if mode == "ultra_polygon":
        obb = mask_to_ultralytics_polygon_obb(arr)
        if obb is not None:
            return obb
        return mask_to_minarea_obb(arr)

    raise ValueError(f"Unknown OBB mode: {mode}")

def polygon_to_minarea_obb(segment: np.ndarray) -> list[float] | None:
    pts = np.asarray(segment, dtype=np.float32)

    if pts.ndim != 2 or pts.shape[0] < 3 or pts.shape[1] != 2:
        return None

    rect = cv2.minAreaRect(pts)
    box = cv2.boxPoints(rect).astype(np.float32)

    box = np.clip(box, 0.0, 1.0)
    box = order_points(box)

    return box.reshape(-1).tolist()


def mask_to_ultralytics_polygon_obb(mask: np.ndarray) -> list[float] | None:
    """
    Use HF SAM3 binary mask, but let Ultralytics convert the mask to normalized polygons.
    Then fit OBB from that polygon.
    """
    from ultralytics.engine.results import Masks

    mask = mask.astype(np.uint8)

    if mask.ndim != 2:
        return None

    h, w = mask.shape[:2]

    mask_tensor = torch.as_tensor(mask)

    if mask_tensor.ndim == 2:
        mask_tensor = mask_tensor[None, :, :]

    masks = Masks(mask_tensor, orig_shape=(h, w))

    if not masks.xyn or len(masks.xyn) == 0:
        return None

    segment = masks.xyn[0]

    if segment is None or len(segment) < 3:
        return None

    return polygon_to_minarea_obb(segment)


def make_timing(t0: float, det_time: float, sam_time: float) -> dict[str, float]:
    return {
        "det_time": det_time,
        "sam_time": sam_time,
        "total_time": time.perf_counter() - t0,
    }

def write_empty_label(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("", encoding="utf-8")

class HFSam3Backend:
    def __init__(self, model_path: str, device: str, sam_size: int):
        from transformers import Sam3Config, Sam3Model, Sam3Processor

        print("Loading HF SAM3...", flush=True)

        config = Sam3Config.from_pretrained(model_path, local_files_only=True)
        config.image_size = sam_size

        self.processor = Sam3Processor.from_pretrained(
            model_path,
            local_files_only=True,
            size={"height": sam_size, "width": sam_size},
        )

        self.model = Sam3Model.from_pretrained(
            model_path,
            config=config,
            local_files_only=True,
        ).to(device)

        self.model.eval()
        self.device = device
        self.sam_size = sam_size

        print(f"HF SAM3 size: {sam_size}", flush=True)

    def segment(self, img_path: Path, image: Image.Image, box_xyxy: list[float]):
        inputs = self.processor(
            images=image,
            input_boxes=[[box_xyxy]],
            input_boxes_labels=[[1]],
            return_tensors="pt",
        ).to(self.device)

        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_instance_segmentation(
            outputs,
            threshold=0.5,
            mask_threshold=0.5,
            target_sizes=inputs["original_sizes"].tolist(),
        )[0]

        masks = results["masks"]
        scores = results.get("scores", None)

        if len(masks) == 0:
            return None, None

        best_idx = 0
        if scores is not None and len(scores) > 0:
            best_idx = int(torch.argmax(scores).item())

        mask = masks[best_idx]
        if isinstance(mask, torch.Tensor):
            mask = mask.detach().cpu().numpy()

        score = None
        if scores is not None and len(scores) > 0:
            score = float(scores[best_idx])

        return mask, score


class UltralyticsSamBackend:
    def __init__(self, model_path: str, device: str, sam_imgsz: int):
        from ultralytics import SAM

        print("Loading Ultralytics SAM...", flush=True)

        self.model = SAM(model_path)
        self.device = device
        self.sam_imgsz = sam_imgsz

        print(f"Ultralytics SAM imgsz: {sam_imgsz}", flush=True)

    def segment(self, img_path: Path, image: Image.Image, box_xyxy: list[float]):
        results = self.model(
            str(img_path),
            bboxes=box_xyxy,
            device=self.device,
            imgsz=self.sam_imgsz,
            verbose=False,
        )

        if not results or results[0].masks is None:
            return None, None

        masks = results[0].masks

        if len(masks.data) == 0:
            return None, None

        if not masks.xyn or len(masks.xyn) == 0:
            return None, None

        segment = masks.xyn[0]
        return segment, None
class UltralyticsSam3SemanticBackend:
    """
    Uses ultralytics.models.sam.SAM3SemanticPredictor directly.

    This follows the SAM3 docs pattern:
      predictor.set_image(source)
      predictor2.inference_features(predictor.features, src_shape=..., bboxes=[...])
    """

    def __init__(self, model_path: str, device: str, sam_imgsz: int, conf: float = 0.5):
        from ultralytics.models.sam import SAM3SemanticPredictor

        print("Loading Ultralytics SAM3SemanticPredictor...", flush=True)

        overrides = dict(
            conf=conf,
            task="segment",
            mode="predict",
            model=model_path,
            imgsz=sam_imgsz,
            device=device,
            verbose=False,
        )

        self.feature_predictor = SAM3SemanticPredictor(overrides=overrides)
        self.prompt_predictor = SAM3SemanticPredictor(overrides=overrides)
        self.prompt_predictor.setup_model()

        self.device = device
        self.sam_imgsz = sam_imgsz
        self.last_image_path: str | None = None
        self.last_src_shape = None

        print(f"Ultralytics SAM3 semantic imgsz: {sam_imgsz}", flush=True)

    def _set_image_if_needed(self, img_path: Path):
        img_path_str = str(img_path)

        if self.last_image_path == img_path_str:
            return

        self.feature_predictor.set_image(img_path_str)

        image = cv2.imread(img_path_str)
        if image is None:
            raise RuntimeError(f"Could not read image: {img_path}")

        self.last_src_shape = image.shape[:2]
        self.last_image_path = img_path_str

    def segment(self, img_path: Path, image: Image.Image, box_xyxy: list[float]):
        self._set_image_if_needed(img_path)

        # SAM3SemanticPredictor expects a list of boxes.
        masks, boxes = self.prompt_predictor.inference_features(
            self.feature_predictor.features,
            src_shape=self.last_src_shape,
            bboxes=[box_xyxy],
        )

        print(
            f"semantic masks type={type(masks)} "
            f"boxes type={type(boxes)}",
            flush=True,
        )

        if masks is not None:
            print(f"semantic masks shape={getattr(masks, 'shape', None)}", flush=True)

        if boxes is not None:
            print(f"semantic boxes shape={getattr(boxes, 'shape', None)} boxes={boxes}", flush=True)

        if masks is None:
            return None, None

        if isinstance(masks, torch.Tensor):
            if masks.numel() == 0 or masks.shape[0] == 0:
                return None, None
            mask = masks[0].detach().cpu().numpy()
        else:
            masks = np.asarray(masks)
            if masks.size == 0 or masks.shape[0] == 0:
                return None, None
            mask = masks[0]

        # Some outputs can be float probabilities. Convert to binary if needed.
        if mask.dtype != np.bool_:
            mask = mask > 0.5

        return mask, None
    
def process_image(
    img_path: Path,
    out_file: Path,
    det_model,
    sam_backend,
    det_device: str,
    conf: float,
    iou: float,
    imgsz: int,
    max_det: int,
    classes: list[int] | None,
    write_empty_labels: bool,
    obb_mode: str,
    debug: bool,
):
    t0 = time.perf_counter()

    t_det0 = time.perf_counter()
    det_result = det_model(
        str(img_path),
        conf=conf,
        iou=iou,
        imgsz=imgsz,
        max_det=max_det,
        device=det_device,
        classes=classes,
        verbose=False,
    )[0]
    t_det1 = time.perf_counter()

    det_time = t_det1 - t_det0

    if det_result.obb is None or len(det_result.obb) == 0:
        if write_empty_labels:
            out_file.parent.mkdir(parents=True, exist_ok=True)
            write_empty_label(out_file)
        return "no_det", make_timing(t0, det_time, 0.0)

    image = Image.open(img_path).convert("RGB")

    lines: list[str] = []
    sam_time_total = 0.0

    for i in range(len(det_result.obb)):
        obb_single = det_result.obb[i : i + 1]

        class_id = get_detector_class_id(obb_single)
        box_xyxy = obb_to_xyxy(obb_single)

        t_sam0 = time.perf_counter()
        mask, score = sam_backend.segment(img_path, image, box_xyxy)
        t_sam1 = time.perf_counter()

        sam_time_total += t_sam1 - t_sam0

        if mask is None:
            continue

        obb = mask_to_obb(mask, obb_single, obb_mode)

        if obb is None:
            continue

        values = " ".join(f"{v:.6f}" for v in obb)
        lines.append(f"{class_id} {values}")

        if debug:
            score_txt = "" if score is None else f", sam_score={score:.4f}"
            print(f"    bbox={box_xyxy}{score_txt}", flush=True)

    out_file.parent.mkdir(parents=True, exist_ok=True)

    if lines:
        out_file.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return "written", make_timing(t0, det_time, sam_time_total)

    if write_empty_labels:
        write_empty_label(out_file)
        return "empty", make_timing(t0, det_time, sam_time_total)

    return "no_mask", make_timing(t0, det_time, sam_time_total)


def parse_classes(values: list[int] | None) -> list[int] | None:
    if values is None:
        return None

    if len(values) == 0:
        return None

    return values


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--backend",
        choices=["hf", "ultralytics", "ultralytics_semantic"],
        default="hf",
        help="Which SAM3 backend to use.",
    )

    parser.add_argument(
        "--dataset",
        default="/home/ruben/halmstad_ws/datasets/tmp_datasets/sam_work/inputs/baylands_super_75_15_10_mini10",
    )

    parser.add_argument(
        "--output-root",
        default="/home/ruben/halmstad_ws/datasets/tmp_datasets/sam_work/outputs/sam3_obb_from_masks_mini10",
    )

    parser.add_argument(
        "--det-model",
        default="/home/ruben/halmstad_ws/models/obb/mymodels/baylands-leader-v4-3.pt",
    )

    parser.add_argument(
        "--sam-model",
        default="/home/ruben/halmstad_ws/datasets/tmp_datasets/sam_work/models/sam3_hf",
        help="HF folder for --backend hf, or .pt/.pth for --backend ultralytics.",
    )

    parser.add_argument("--splits", nargs="+", default=["train", "val", "test"])

    parser.add_argument("--device", default="cpu", help="SAM device.")
    parser.add_argument("--det-device", default="cpu", help="Detector device.")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--max-det", type=int, default=1)
    parser.add_argument("--classes", nargs="*", type=int, default=[0])

    parser.add_argument("--sam-size", type=int, default=644, help="HF SAM3 square size.")
    parser.add_argument("--sam-imgsz", type=int, default=644, help="Ultralytics SAM image size.")

    parser.add_argument(
    "--obb-mode",
    choices=["minarea", "hybrid", "ultra_polygon"],
    default="minarea",
    help=(
        "minarea = OpenCV minAreaRect from mask. "
        "hybrid = SAM center/extent with detector angle. "
        "ultra_polygon = HF mask -> Ultralytics Masks.xyn -> minAreaRect."
    ),
)

    parser.add_argument("--write-empty", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--debug-every", type=int, default=25)
    parser.add_argument(
        "--num-shards",
        type=int,
        default=1,
        help="Total number of shards for parallel processing of a split.",
    )

    parser.add_argument(
        "--shard-index",
        type=int,
        default=0,
        help="Shard index to process, from 0 to num-shards-1.",
    )

    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip images whose output label file already exists. Useful for resuming interrupted runs.",
    )

    args = parser.parse_args()

    dataset = Path(args.dataset)
    output_root = Path(args.output_root)
    classes = parse_classes(args.classes)

    print("Configuration")
    print("-------------")
    print(f"Backend:      {args.backend}")
    print(f"Dataset:      {dataset}")
    print(f"Output root:  {output_root}")
    print(f"Detector:     {args.det_model}")
    print(f"SAM model:    {args.sam_model}")
    print(f"OBB mode:     {args.obb_mode}")
    print(f"Device:       {args.device}")
    print(f"Detector dev: {args.det_device}")
    print()

    print("Loading OBB detector...", flush=True)
    det_model = YOLO(args.det_model)

    if args.backend == "hf":
        sam_backend = HFSam3Backend(
            model_path=args.sam_model,
            device=args.device,
            sam_size=args.sam_size,
        )

    elif args.backend == "ultralytics":
        sam_backend = UltralyticsSamBackend(
            model_path=args.sam_model,
            device=args.device,
            sam_imgsz=args.sam_imgsz,
        )

    elif args.backend == "ultralytics_semantic":
        sam_backend = UltralyticsSam3SemanticBackend(
            model_path=args.sam_model,
            device=args.device,
            sam_imgsz=args.sam_imgsz,
            conf=0.5,
        )

    else:
        raise ValueError(f"Unknown backend: {args.backend}")

    for split in args.splits:
        image_dir = dataset / "images" / split
        label_dir = output_root / "labels" / split

        if not image_dir.is_dir():
            print(f"Skipping missing split: {image_dir}", flush=True)
            continue

        images = sorted(
            p
            for p in image_dir.rglob("*")
            if p.is_file() and p.suffix.lower() in IMAGE_EXTS
        )

        if args.num_shards < 1:
            raise ValueError("--num-shards must be >= 1")

        if args.shard_index < 0 or args.shard_index >= args.num_shards:
            raise ValueError("--shard-index must be in [0, num-shards)")

        total_images_before_shard = len(images)
        images = images[args.shard_index::args.num_shards]

        print(
            f"Shard:  {args.shard_index + 1}/{args.num_shards} "
            f"({len(images)} of {total_images_before_shard} images)",
            flush=True,
        )

        print()
        print(f"Split:  {split}", flush=True)
        print(f"Images: {len(images)}", flush=True)
        print(f"Output: {label_dir}", flush=True)

        counts = {
            "written": 0,
            "empty": 0,
            "no_det": 0,
            "no_mask": 0,
            "skipped_existing": 0,
        }

        timing_totals = {
            "images": 0,
            "det_time": 0.0,
            "sam_time": 0.0,
            "total_time": 0.0,
        }

        for idx, img_path in enumerate(images, 1):
            out_file = label_dir / f"{img_path.stem}.txt"

            if args.skip_existing and out_file.exists():
                counts["skipped_existing"] += 1

                if idx % args.debug_every == 0:
                    n = max(1, timing_totals["images"])
                    print(
                        f"  progress {idx}/{len(images)} | "
                        f"skipped_existing={counts['skipped_existing']} "
                        f"written={counts['written']} "
                        f"empty={counts['empty']} "
                        f"no_det={counts['no_det']} "
                        f"no_mask={counts['no_mask']} | "
                        f"processed={timing_totals['images']} "
                        f"avg_total={timing_totals['total_time'] / n:.2f}s/img",
                        flush=True,
                    )

                continue

            if out_file.exists() and not args.overwrite:
                raise FileExistsError(
                    f"Output label already exists: {out_file}\n"
                    "Use --skip-existing to resume, or --overwrite to replace labels."
                )

            should_debug = args.debug and (idx <= 5 or idx % args.debug_every == 0)

            print(f"[{idx}/{len(images)}] {img_path.name}", flush=True)

            status, timing = process_image(
                img_path=img_path,
                out_file=out_file,
                det_model=det_model,
                sam_backend=sam_backend,
                det_device=args.det_device,
                conf=args.conf,
                iou=args.iou,
                imgsz=args.imgsz,
                max_det=args.max_det,
                classes=classes,
                write_empty_labels=args.write_empty,
                obb_mode=args.obb_mode,
                debug=should_debug,
            )

            counts[status] = counts.get(status, 0) + 1

            timing_totals["images"] += 1
            timing_totals["det_time"] += timing["det_time"]
            timing_totals["sam_time"] += timing["sam_time"]
            timing_totals["total_time"] += timing["total_time"]

            if idx % args.debug_every == 0:
                n = max(1, timing_totals["images"])
                print(
                    f"  progress {idx}/{len(images)} | "
                    f"written={counts['written']} "
                    f"empty={counts['empty']} "
                    f"no_det={counts['no_det']} "
                    f"no_mask={counts['no_mask']} | "
                    f"avg_total={timing_totals['total_time'] / n:.2f}s/img "
                    f"avg_det={timing_totals['det_time'] / n:.2f}s/img "
                    f"avg_sam={timing_totals['sam_time'] / n:.2f}s/img",
                    flush=True,
                )

        print()
        print(f"Done split: {split}")
        for key, value in counts.items():
            print(f"  {key}: {value}")

        n = max(1, timing_totals["images"])
        total_time = timing_totals["total_time"]

        print()
        print("Timing")
        print("------")
        print(f"processed images: {timing_totals['images']}")
        print(f"total time:       {total_time:.2f}s")
        print(f"detector time:    {timing_totals['det_time']:.2f}s")
        print(f"SAM time:         {timing_totals['sam_time']:.2f}s")
        print(f"avg total:        {total_time / n:.2f}s/image")
        print(f"avg detector:     {timing_totals['det_time'] / n:.2f}s/image")
        print(f"avg SAM:          {timing_totals['sam_time'] / n:.2f}s/image")

        if total_time > 0:
            print(f"throughput:       {n / total_time:.3f} images/s")
        else:
            print("throughput:       n/a")

    print()
    print("All done")
    print(f"Output root: {output_root}")


if __name__ == "__main__":
    main()

from pathlib import Path
import cv2
import numpy as np
import torch
from PIL import Image

from ultralytics import YOLO
from transformers import Sam3Config, Sam3Model, Sam3Processor


LABEL_AABB = [0.391234, 0.689267, 0.076893, 0.120721]
LABEL_OBB = [0.359025, 0.627631, 0.452234, 0.646043, 0.435900, 0.793048, 0.342691, 0.774637]
SAM3_HF = "/home/ruben/halmstad_ws/datasets/tmp_datasets/sam_work/models/sam3_hf"
OUT_DIR = Path("/home/ruben/halmstad_ws/datasets/tmp_datasets/sam_work/review/hf_sam3_processor_debug")
IMG = "/home/ruben/halmstad_ws/datasets/tmp_datasets/sam_work/datasets/sam3_obb_test_mini10/images/test/spawn_v1_frame_000161_256800000000.jpg"
DET = "/home/ruben/halmstad_ws/models/obb/mymodels/baylands-leader-v4-3.pt"


def to_numpy(x):
    if isinstance(x, torch.Tensor):
        return x.detach().cpu().numpy()
    return np.asarray(x)


def point_to_box(point, radius=8.0):
    x, y = point
    return [
        float(x - radius),
        float(y - radius),
        float(x + radius),
        float(y + radius),
    ]

def run_detector():
    det = YOLO(DET)

    result = det(
        str(IMG),
        conf=0.25,
        imgsz=640,
        max_det=1,
        device="cpu",
        verbose=True,
    )[0]

    if result.obb is None or len(result.obb) == 0:
        raise SystemExit("No detector OBB")

    print("Result path:", result.path)
    print("Result names:", result.names)
    print("Result speed:", result.speed)
    print("OBB confidence:", result.obb.conf)
    print("OBB class:", result.obb.cls)
    print("OBB xywhr:", result.obb.xywhr)
    print("OBB corners:", result.obb.xyxyxyxy)

    result.save(filename="detector_obb_result.jpg")

    return result


def prompts_from_obb_result(result, idx=0):
    obb = result.obb[idx:idx + 1]

    pts = to_numpy(obb.xyxyxyxy[0]).astype(np.float32)
    xywhr = to_numpy(obb.xywhr[0]).astype(np.float32)

    conf = float(obb.conf[0])
    cls = int(obb.cls[0])

    box = [
        float(pts[:, 0].min()),
        float(pts[:, 1].min()),
        float(pts[:, 0].max()),
        float(pts[:, 1].max()),
    ]

    cx_box = (box[0] + box[2]) / 2.0
    cy_box = (box[1] + box[3]) / 2.0

    cx_obb = float(pts[:, 0].mean())
    cy_obb = float(pts[:, 1].mean())

    return {
        "obb": obb,
        "class_id": cls,
        "confidence": conf,
        "corners": pts,
        "xywhr": xywhr,
        "box": box,
        "center_point": [cx_box, cy_box],
        "obb_center_point": [cx_obb, cy_obb],
        "corner_points": pts.tolist(),
    }


def draw_prompt_and_mask(name, image_path, box, masks):
    img = cv2.imread(str(image_path))
    if img is None:
        raise RuntimeError(f"Could not read image: {image_path}")

    x1, y1, x2, y2 = map(int, box)
    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.putText(
        img,
        "prompt bbox",
        (x1, max(20, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 0, 0),
        2,
    )

    if masks is None:
        cv2.putText(
            img,
            "NO MASK",
            (15, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
        )
    else:
        masks = to_numpy(masks)

        if masks.ndim == 2:
            masks = masks[None, :, :]

        print(f"{name}: masks shape={masks.shape}")

        if masks.shape[0] > 0:
            mask = masks[0]

            if mask.shape[:2] != img.shape[:2]:
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (img.shape[1], img.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )

            mask_bool = mask.astype(bool)

            overlay = img.copy()
            overlay[mask_bool] = (0, 255, 0)
            img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)

            contours, _ = cv2.findContours(
                mask_bool.astype(np.uint8),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            if contours:
                contour = max(contours, key=cv2.contourArea)
                cv2.drawContours(img, [contour], -1, (0, 0, 255), 2)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"{name}.jpg"
    cv2.imwrite(str(out), img)
    print("Wrote:", out)


def load_hf_sam3(device="cpu", sam_size=644):
    config = Sam3Config.from_pretrained(SAM3_HF, local_files_only=True)
    config.image_size = sam_size

    processor = Sam3Processor.from_pretrained(
        SAM3_HF,
        local_files_only=True,
        size={"height": sam_size, "width": sam_size},
    )

    model = Sam3Model.from_pretrained(
        SAM3_HF,
        config=config,
        local_files_only=True,
    ).to(device)

    model.eval()

    return processor, model


def run_hf_query(
    name,
    processor,
    model,
    image,
    box,
    device="cpu",
):
    print()
    print(f"Query: {name}")
    print("box:", box)

    inputs = processor(
        images=image,
        input_boxes=[[box]],
        input_boxes_labels=[[1]],
        return_tensors="pt",
    ).to(device)

    with torch.no_grad():
        outputs = model(**inputs)

    results = processor.post_process_instance_segmentation(
        outputs,
        threshold=0.5,
        mask_threshold=0.5,
        target_sizes=inputs["original_sizes"].tolist(),
    )[0]

    masks = results["masks"]
    scores = results.get("scores", None)
    boxes = results.get("boxes", None)

    print("result keys:", results.keys())
    print("masks:", None if masks is None else masks.shape)
    print("boxes:", boxes)
    print("scores:", scores)

    if masks is None or len(masks) == 0:
        draw_prompt_and_mask(name, IMG, box, None)
        return

    best_idx = 0
    if scores is not None and len(scores) > 0:
        best_idx = int(torch.argmax(scores).item())

    mask = masks[best_idx]
    if isinstance(mask, torch.Tensor):
        mask = mask.detach().cpu().numpy()

    draw_prompt_and_mask(name, IMG, box, mask)


# 1. Run detector once
det_result = run_detector()
prompts = prompts_from_obb_result(det_result)

box = prompts["box"]
center = prompts["center_point"]
obb_center = prompts["obb_center_point"]
corners = prompts["corner_points"]

print()
print("Detector prompts")
print("----------------")
print("box:", box)
print("center:", center)
print("obb center:", obb_center)
print("corners:")
print(np.asarray(corners))

# 2. Load HF SAM3 once
device = "cpu"
processor, model = load_hf_sam3(device=device, sam_size=644)
image = Image.open(IMG).convert("RGB")

# 3. Test multiple HF box prompt variants

# Detector axis-aligned bbox
run_hf_query(
    name="01_box_only",
    processor=processor,
    model=model,
    image=image,
    box=box,
    device=device,
)

# Tiny box around detector bbox center
run_hf_query(
    name="02_center_tiny_box",
    processor=processor,
    model=model,
    image=image,
    box=point_to_box(center, radius=8.0),
    device=device,
)

# Tiny box around OBB polygon center
run_hf_query(
    name="03_obb_center_tiny_box",
    processor=processor,
    model=model,
    image=image,
    box=point_to_box(obb_center, radius=8.0),
    device=device,
)

# Slightly larger box around center
run_hf_query(
    name="04_center_medium_box",
    processor=processor,
    model=model,
    image=image,
    box=point_to_box(center, radius=16.0),
    device=device,
)

# Original detector bbox, still probably the best
run_hf_query(
    name="05_box_repeat_check",
    processor=processor,
    model=model,
    image=image,
    box=box,
    device=device,
)
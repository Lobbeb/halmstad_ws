from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional, Tuple


Corners2D = Tuple[Tuple[float, float], ...]


@dataclass
class Detection2D:
    u: float
    v: float
    conf: float
    bbox: Tuple[float, float, float, float]
    cls_id: Optional[int] = None
    cls_name: str = ""
    track_id: Optional[int] = None
    track_hits: int = 0
    track_age_s: float = 0.0
    track_state: str = "raw"
    track_switched: bool = False
    obb_corners: Optional[Corners2D] = None
    obb_heading_yaw: Optional[float] = None
    source: str = ""


@dataclass
class DetectionMessage:
    stamp_ns: int
    detection: Optional[Detection2D]
    metadata: dict[str, object]


def encode_detection_payload(
    stamp_ns: int,
    det: Optional[Detection2D],
    *,
    metadata: Optional[dict[str, object]] = None,
) -> str:
    payload = {
        "stamp_ns": int(stamp_ns),
        "valid": det is not None,
        "u": None if det is None else float(det.u),
        "v": None if det is None else float(det.v),
        "conf": -1.0 if det is None else float(det.conf),
        "bbox": [] if det is None else [float(value) for value in det.bbox],
        "cls_id": None if det is None else det.cls_id,
        "cls_name": "" if det is None else det.cls_name,
        "track_id": None if det is None else det.track_id,
        "track_hits": 0 if det is None else int(det.track_hits),
        "track_age_s": 0.0 if det is None else float(det.track_age_s),
        "track_state": "raw" if det is None else str(det.track_state),
        "track_switched": False if det is None else bool(det.track_switched),
        "obb_corners": [] if det is None or det.obb_corners is None else [[float(x), float(y)] for x, y in det.obb_corners],
        "obb_heading_yaw": None if det is None or det.obb_heading_yaw is None else float(det.obb_heading_yaw),
        "source": "" if det is None else str(det.source),
        "metadata": dict(metadata or {}),
    }
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=True)


def decode_detection_payload(data: str) -> DetectionMessage:
    try:
        payload = json.loads(data)
    except Exception as exc:
        raise ValueError("invalid_detection_payload_json") from exc

    stamp_ns = int(payload.get("stamp_ns", 0) or 0)
    if not bool(payload.get("valid", False)):
        return DetectionMessage(stamp_ns=stamp_ns, detection=None, metadata=dict(payload.get("metadata") or {}))

    bbox_raw = payload.get("bbox") or []
    if len(bbox_raw) < 4:
        raise ValueError("invalid_detection_payload_bbox")

    obb_corners = None
    obb_raw = payload.get("obb_corners") or []
    if len(obb_raw) == 4:
        obb_corners = tuple((float(point[0]), float(point[1])) for point in obb_raw)

    obb_heading_yaw = payload.get("obb_heading_yaw")
    det = Detection2D(
        u=float(payload.get("u")),
        v=float(payload.get("v")),
        conf=float(payload.get("conf", 0.0)),
        bbox=tuple(float(value) for value in bbox_raw[:4]),
        cls_id=None if payload.get("cls_id") is None else int(payload.get("cls_id")),
        cls_name=str(payload.get("cls_name", "")),
        track_id=None if payload.get("track_id") is None else int(payload.get("track_id")),
        track_hits=int(payload.get("track_hits", 0) or 0),
        track_age_s=float(payload.get("track_age_s", 0.0) or 0.0),
        track_state=str(payload.get("track_state", "raw")),
        track_switched=bool(payload.get("track_switched", False)),
        obb_corners=obb_corners,
        obb_heading_yaw=None if obb_heading_yaw is None else float(obb_heading_yaw),
        source=str(payload.get("source", "")),
    )
    return DetectionMessage(
        stamp_ns=stamp_ns,
        detection=det,
        metadata=dict(payload.get("metadata") or {}),
    )

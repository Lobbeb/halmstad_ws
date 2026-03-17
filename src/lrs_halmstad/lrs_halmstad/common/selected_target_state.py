from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


@dataclass
class SelectedTargetState:
    stamp: TimeMsg
    frame_id: str
    valid: bool
    bbox_center_x_px: float = 0.0
    bbox_center_y_px: float = 0.0
    bbox_size_x_px: float = 0.0
    bbox_size_y_px: float = 0.0
    bbox_theta_rad: float = 0.0
    confidence: float = 0.0
    class_id: str = ""
    track_id: str = ""
    projected_range_m: Optional[float] = None
    track_age_s: Optional[float] = None


def encode_selected_target_state_msg(state: SelectedTargetState) -> Detection2DArray:
    msg = Detection2DArray()
    msg.header = Header(stamp=state.stamp, frame_id=str(state.frame_id or ""))
    if not state.valid:
        return msg

    det = Detection2D()
    det.header = Header(stamp=state.stamp, frame_id=str(state.frame_id or ""))
    det.bbox.center.position.x = float(state.bbox_center_x_px)
    det.bbox.center.position.y = float(state.bbox_center_y_px)
    det.bbox.center.theta = float(state.bbox_theta_rad)
    det.bbox.size_x = float(state.bbox_size_x_px)
    det.bbox.size_y = float(state.bbox_size_y_px)
    det.id = str(state.track_id or "")

    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = str(state.class_id or "")
    result.hypothesis.score = float(state.confidence)
    result.pose.pose.position.x = (
        float(state.projected_range_m) if state.projected_range_m is not None else math.nan
    )
    result.pose.pose.position.y = float(state.track_age_s) if state.track_age_s is not None else math.nan
    result.pose.pose.position.z = math.nan
    det.results.append(result)
    msg.detections.append(det)
    return msg


def decode_selected_target_state_msg(msg: Detection2DArray) -> SelectedTargetState:
    if not msg.detections:
        return SelectedTargetState(
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
            valid=False,
        )

    det = msg.detections[0]
    result = det.results[0] if det.results else None
    projected_range_m = None
    track_age_s = None
    class_id = ""
    confidence = 0.0
    if result is not None:
        class_id = str(result.hypothesis.class_id or "")
        confidence = float(result.hypothesis.score)
        range_value = float(result.pose.pose.position.x)
        age_value = float(result.pose.pose.position.y)
        if math.isfinite(range_value):
            projected_range_m = range_value
        if math.isfinite(age_value):
            track_age_s = age_value

    return SelectedTargetState(
        stamp=det.header.stamp if det.header.stamp.sec != 0 or det.header.stamp.nanosec != 0 else msg.header.stamp,
        frame_id=det.header.frame_id or msg.header.frame_id,
        valid=True,
        bbox_center_x_px=float(det.bbox.center.position.x),
        bbox_center_y_px=float(det.bbox.center.position.y),
        bbox_size_x_px=float(det.bbox.size_x),
        bbox_size_y_px=float(det.bbox.size_y),
        bbox_theta_rad=float(det.bbox.center.theta),
        confidence=confidence,
        class_id=class_id,
        track_id=str(det.id or ""),
        projected_range_m=projected_range_m,
        track_age_s=track_age_s,
    )

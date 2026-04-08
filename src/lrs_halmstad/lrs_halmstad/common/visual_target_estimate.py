from __future__ import annotations

import math
from dataclasses import dataclass

from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


_META_SENTINEL = 4242.0
_MODE_TO_CODE = {
    "LOST": 0.0,
    "DEGRADED": 1.0,
    "PREDICTED": 2.0,
    "TRACKED": 3.0,
}
_CODE_TO_MODE = {int(code): mode for mode, code in _MODE_TO_CODE.items()}


@dataclass
class VisualTargetEstimate:
    stamp: TimeMsg
    frame_id: str
    valid: bool
    track_id: str = ""
    rel_x_m: float = math.nan
    rel_y_m: float = math.nan
    rel_z_m: float = math.nan
    rel_vx_mps: float = 0.0
    rel_vy_mps: float = 0.0
    rel_vz_mps: float = 0.0
    mode: str = "LOST"
    quality: float = 0.0
    target_age_s: float = math.nan
    predicted_age_s: float = 0.0
    continuity_score: float = 0.0
    consecutive_hits: int = 0
    consecutive_misses: int = 0

    @property
    def range_m(self) -> float:
        return float(self.rel_z_m)

    @property
    def bearing_rad(self) -> float:
        if not self.valid or not math.isfinite(self.rel_z_m) or self.rel_z_m <= 0.0:
            return math.nan
        return math.atan2(float(self.rel_x_m), float(self.rel_z_m))

    @property
    def elevation_rad(self) -> float:
        if not self.valid or not math.isfinite(self.rel_z_m) or self.rel_z_m <= 0.0:
            return math.nan
        return math.atan2(float(self.rel_y_m), float(self.rel_z_m))


def encode_visual_target_estimate_msg(state: VisualTargetEstimate) -> Odometry:
    msg = Odometry()
    msg.header = Header(stamp=state.stamp, frame_id=str(state.frame_id or ""))
    msg.child_frame_id = str(state.track_id or "")
    msg.pose.covariance[0] = float(state.quality)
    msg.pose.covariance[1] = float(state.target_age_s)
    msg.pose.covariance[2] = float(state.predicted_age_s)
    msg.pose.covariance[3] = float(state.continuity_score)
    msg.pose.covariance[4] = float(state.consecutive_hits)
    msg.pose.covariance[5] = float(state.consecutive_misses)
    msg.pose.covariance[6] = float(_MODE_TO_CODE.get(str(state.mode).upper(), _MODE_TO_CODE["LOST"]))
    msg.pose.covariance[35] = _META_SENTINEL

    if not state.valid:
        msg.pose.pose.position.x = math.nan
        msg.pose.pose.position.y = math.nan
        msg.pose.pose.position.z = math.nan
        msg.twist.twist.linear.x = math.nan
        msg.twist.twist.linear.y = math.nan
        msg.twist.twist.linear.z = math.nan
        return msg

    msg.pose.pose.position.x = float(state.rel_x_m)
    msg.pose.pose.position.y = float(state.rel_y_m)
    msg.pose.pose.position.z = float(state.rel_z_m)
    msg.twist.twist.linear.x = float(state.rel_vx_mps)
    msg.twist.twist.linear.y = float(state.rel_vy_mps)
    msg.twist.twist.linear.z = float(state.rel_vz_mps)
    return msg


def decode_visual_target_estimate_msg(msg: Odometry) -> VisualTargetEstimate:
    x = float(msg.pose.pose.position.x)
    y = float(msg.pose.pose.position.y)
    z = float(msg.pose.pose.position.z)
    vx = float(msg.twist.twist.linear.x)
    vy = float(msg.twist.twist.linear.y)
    vz = float(msg.twist.twist.linear.z)
    valid = math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and z > 0.0
    meta = list(msg.pose.covariance)
    has_meta = len(meta) >= 36 and math.isfinite(float(meta[35])) and abs(float(meta[35]) - _META_SENTINEL) < 1e-6
    if has_meta:
        quality = float(meta[0]) if math.isfinite(float(meta[0])) else (1.0 if valid else 0.0)
        target_age_s = float(meta[1]) if math.isfinite(float(meta[1])) else (0.0 if valid else math.nan)
        predicted_age_s = float(meta[2]) if math.isfinite(float(meta[2])) else 0.0
        continuity_score = float(meta[3]) if math.isfinite(float(meta[3])) else (1.0 if valid else 0.0)
        consecutive_hits = max(0, int(round(float(meta[4])))) if math.isfinite(float(meta[4])) else 0
        consecutive_misses = max(0, int(round(float(meta[5])))) if math.isfinite(float(meta[5])) else 0
        mode_code = int(round(float(meta[6]))) if math.isfinite(float(meta[6])) else 0
        mode = _CODE_TO_MODE.get(mode_code, "TRACKED" if valid else "LOST")
    else:
        quality = 1.0 if valid else 0.0
        target_age_s = 0.0 if valid else math.nan
        predicted_age_s = 0.0
        continuity_score = 1.0 if valid else 0.0
        consecutive_hits = 1 if valid else 0
        consecutive_misses = 0
        mode = "TRACKED" if valid else "LOST"

    return VisualTargetEstimate(
        stamp=msg.header.stamp,
        frame_id=msg.header.frame_id,
        valid=valid,
        track_id=str(msg.child_frame_id or ""),
        rel_x_m=x,
        rel_y_m=y,
        rel_z_m=z,
        rel_vx_mps=vx if math.isfinite(vx) else 0.0,
        rel_vy_mps=vy if math.isfinite(vy) else 0.0,
        rel_vz_mps=vz if math.isfinite(vz) else 0.0,
        mode=mode,
        quality=max(0.0, min(1.0, quality)),
        target_age_s=max(0.0, target_age_s) if math.isfinite(target_age_s) else math.nan,
        predicted_age_s=max(0.0, predicted_age_s),
        continuity_score=max(0.0, min(1.0, continuity_score)),
        consecutive_hits=consecutive_hits,
        consecutive_misses=consecutive_misses,
    )

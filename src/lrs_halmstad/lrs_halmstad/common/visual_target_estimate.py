from __future__ import annotations

import math
from dataclasses import dataclass

from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


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
    )

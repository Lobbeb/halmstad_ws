#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List


@dataclass
class MotionProfileConfig:
    motion_profile: str
    turn_pattern: str
    forward_speed: float
    forward_time_s: float
    turn_speed: float
    turn_time_s: float
    cycles: int
    variation_enable: bool
    variation_amplitude: float
    pause_every_n: int
    pause_time_s: float


@dataclass
class MotionSegment:
    name: str
    vx: float
    wz: float
    duration_s: float


@dataclass
class MotionWaypoint:
    segment_name: str
    x: float
    y: float
    yaw: float
    duration_s: float


def _orchard_corner_waypoints() -> List[MotionWaypoint]:
    # Map-frame corner goals for the current saved orchard map.
    # Keep a margin from the free-space bounds so Nav2 does not skim the edges.
    return [
        MotionWaypoint(segment_name="corner_bl", x=-8.5, y=-9.0, yaw=math.radians(90.0), duration_s=0.0),
        MotionWaypoint(segment_name="corner_tl", x=-8.5, y=26.0, yaw=math.radians(0.0), duration_s=0.0),
        MotionWaypoint(segment_name="corner_tr", x=36.0, y=26.0, yaw=math.radians(-90.0), duration_s=0.0),
        MotionWaypoint(segment_name="corner_br", x=36.0, y=-9.0, yaw=math.radians(180.0), duration_s=0.0),
    ]


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def build_route_waypoints(
    motion_profile: str,
    start_x: float,
    start_y: float,
    loop_count: int,
) -> List[MotionWaypoint]:
    profile = str(motion_profile).strip().lower()
    if profile not in ("orchard_corners", "orchard_route"):
        return []

    base_waypoints = _orchard_corner_waypoints()
    if not base_waypoints or loop_count <= 0:
        return []

    nearest_index = min(
        range(len(base_waypoints)),
        key=lambda idx: math.hypot(base_waypoints[idx].x - float(start_x), base_waypoints[idx].y - float(start_y)),
    )
    ordered = base_waypoints[nearest_index:] + base_waypoints[:nearest_index]

    waypoints: List[MotionWaypoint] = []
    for _ in range(max(1, int(loop_count))):
        for waypoint in ordered:
            waypoints.append(
                MotionWaypoint(
                    segment_name=waypoint.segment_name,
                    x=waypoint.x,
                    y=waypoint.y,
                    yaw=waypoint.yaw,
                    duration_s=waypoint.duration_s,
                )
            )
    return waypoints


def apply_motion_profile(config: MotionProfileConfig) -> MotionProfileConfig:
    adjusted = MotionProfileConfig(
        motion_profile=str(config.motion_profile).strip().lower(),
        turn_pattern=str(config.turn_pattern).strip().lower(),
        forward_speed=float(config.forward_speed),
        forward_time_s=float(config.forward_time_s),
        turn_speed=float(config.turn_speed),
        turn_time_s=float(config.turn_time_s),
        cycles=max(0, int(config.cycles)),
        variation_enable=bool(config.variation_enable),
        variation_amplitude=max(0.0, min(0.45, float(config.variation_amplitude))),
        pause_every_n=max(0, int(config.pause_every_n)),
        pause_time_s=max(0.0, float(config.pause_time_s)),
    )

    if adjusted.motion_profile in ("fast_wide", "far_fast", "wide_fast"):
        adjusted.forward_speed *= 1.8
        adjusted.forward_time_s *= 1.6
        adjusted.turn_speed *= 1.2
        adjusted.turn_time_s *= 0.9
        adjusted.cycles = max(adjusted.cycles, 12)
    elif adjusted.motion_profile in ("fast", "speed"):
        adjusted.forward_speed *= 1.8
        adjusted.turn_speed *= 1.2
    elif adjusted.motion_profile in ("wide", "far"):
        adjusted.forward_time_s *= 1.8
        adjusted.cycles = max(adjusted.cycles, 12)

    return adjusted


def build_segments(config: MotionProfileConfig) -> List[MotionSegment]:
    adjusted = apply_motion_profile(config)
    segments: List[MotionSegment] = []

    for index in range(adjusted.cycles):
        if adjusted.variation_enable:
            # Deterministic modulation keeps the motion repeatable while avoiding
            # the exact same turn radius every cycle.
            fwd_var = 0.65 * math.sin(0.73 * index) + 0.35 * math.sin(0.19 * index + 1.10)
            turn_var = 0.60 * math.cos(0.61 * index + 0.40) + 0.40 * math.sin(0.27 * index + 0.90)
            fwd_scale = max(0.40, 1.0 + adjusted.variation_amplitude * fwd_var)
            turn_scale = max(0.40, 1.0 + adjusted.variation_amplitude * turn_var)
        else:
            fwd_scale = 1.0
            turn_scale = 1.0

        segments.append(
            MotionSegment(
                name="fwd",
                vx=adjusted.forward_speed * fwd_scale,
                wz=0.0,
                duration_s=adjusted.forward_time_s,
            )
        )

        turn_sign = -1.0 if (adjusted.turn_pattern in ("alternate", "zigzag", "zig-zag") and (index % 2)) else 1.0
        segments.append(
            MotionSegment(
                name="turn",
                vx=0.0,
                wz=turn_sign * adjusted.turn_speed * turn_scale,
                duration_s=adjusted.turn_time_s,
            )
        )

        if adjusted.pause_every_n > 0 and ((index + 1) % adjusted.pause_every_n == 0) and adjusted.pause_time_s > 0.0:
            segments.append(
                MotionSegment(
                    name="pause",
                    vx=0.0,
                    wz=0.0,
                    duration_s=adjusted.pause_time_s,
                )
            )

    return segments


def integrate_segments(
    start_x: float,
    start_y: float,
    start_yaw: float,
    segments: List[MotionSegment],
) -> List[MotionWaypoint]:
    x = float(start_x)
    y = float(start_y)
    yaw = float(start_yaw)
    waypoints: List[MotionWaypoint] = []

    for segment in segments:
        if segment.name == "fwd":
            distance = segment.vx * segment.duration_s
            x += distance * math.cos(yaw)
            y += distance * math.sin(yaw)
        elif segment.name == "turn":
            yaw = normalize_angle(yaw + (segment.wz * segment.duration_s))

        waypoints.append(
            MotionWaypoint(
                segment_name=segment.name,
                x=x,
                y=y,
                yaw=yaw,
                duration_s=segment.duration_s,
            )
        )

    return waypoints

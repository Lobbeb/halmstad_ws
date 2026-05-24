#!/usr/bin/env bash
set -euo pipefail

RATE_HZ="1.0"
UGV_TOPIC="/a201_0000/ground_truth/odom"
UAV_TOPIC="/dji0/pose"
TARGET_M="10.0"
TOL_M="8.0"
STALE_S="3.0"
OMNET="true"
ONCE="false"
DRY_RUN="false"

usage() {
  cat <<'EOF'
Usage: ./run.sh motion_monitor [args...]

Headless terminal dashboard for checking that the UGV is moving and the UAV is following.
It subscribes to existing topics only; it does not publish anything.

Args:
  rate:=Hz                    Refresh rate, default 1.0
  ugv:=/topic                 UGV pose/odom topic, default /a201_0000/ground_truth/odom
  uav:=/topic                 UAV pose/odom topic, default /dji0/pose
  target:=m                   Expected UAV-UGV 3D distance, default 10.0
  tol:=m                      Distance tolerance for OK/WARN, default 8.0
  stale:=seconds              Topic stale threshold, default 3.0
  omnet:=true|false           Show PDR/RSSI/SNIR if available, default true
  once:=true|false            Render once and exit
  dry_run:=true|false

Examples:
  ./run.sh motion_monitor
  ./run.sh motion_monitor rate:=0.5
  ./run.sh motion_monitor target:=30 tol:=15
EOF
}

coerce_bool() {
  case "$1" in
    true|false) printf '%s\n' "$1" ;;
    *) echo "Invalid boolean value: $1" >&2; exit 2 ;;
  esac
}

validate_positive() {
  local name="$1"
  local value="$2"
  awk -v value="$value" 'BEGIN { exit !(value + 0 > 0) }' || {
    echo "Invalid $name: $value" >&2
    exit 2
  }
  printf '%s\n' "$value"
}

for arg in "$@"; do
  case "$arg" in
    -h|--help|help)
      usage
      exit 0
      ;;
    rate:=*)
      RATE_HZ="$(validate_positive rate "${arg#rate:=}")"
      ;;
    ugv:=*|ugv_topic:=*)
      UGV_TOPIC="${arg#*:=}"
      ;;
    uav:=*|uav_topic:=*)
      UAV_TOPIC="${arg#*:=}"
      ;;
    target:=*|target_m:=*)
      TARGET_M="$(validate_positive target "${arg#*:=}")"
      ;;
    tol:=*|tolerance:=*|tolerance_m:=*)
      TOL_M="$(validate_positive tolerance "${arg#*:=}")"
      ;;
    stale:=*|stale_s:=*)
      STALE_S="$(validate_positive stale "${arg#*:=}")"
      ;;
    omnet:=*)
      OMNET="$(coerce_bool "${arg#omnet:=}")"
      ;;
    once:=*)
      ONCE="$(coerce_bool "${arg#once:=}")"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${arg#dry_run:=}")"
      ;;
    *)
      echo "Unknown arg: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ "$DRY_RUN" = true ]; then
  cat <<EOF
motion_monitor:
  rate_hz: $RATE_HZ
  ugv_topic: $UGV_TOPIC
  uav_topic: $UAV_TOPIC
  target_m: $TARGET_M
  tolerance_m: $TOL_M
  stale_s: $STALE_S
  omnet: $OMNET
  once: $ONCE
EOF
  exit 0
fi

exec python3 - "$RATE_HZ" "$UGV_TOPIC" "$UAV_TOPIC" "$TARGET_M" "$TOL_M" "$STALE_S" "$OMNET" "$ONCE" <<'PY'
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:
    raise SystemExit(
        "ROS 2 Python modules are unavailable. Source the workspace first:\n"
        "  source /opt/ros/jazzy/setup.bash\n"
        "  source /home/ruben/halmstad_ws/install/setup.bash\n"
        f"Import error: {exc}"
    )


rate_hz = float(sys.argv[1])
ugv_topic = sys.argv[2]
uav_topic = sys.argv[3]
target_m = float(sys.argv[4])
tol_m = float(sys.argv[5])
stale_s = float(sys.argv[6])
show_omnet = sys.argv[7] == "true"
once = sys.argv[8] == "true"


def clear_screen() -> None:
    sys.stdout.write("\033[H\033[2J")


def fmt(value: Optional[float], digits: int = 2, suffix: str = "") -> str:
    if value is None or not math.isfinite(value):
        return "--"
    return f"{value:.{digits}f}{suffix}"


def bar(value: Optional[float], warn: float, bad: float) -> str:
    if value is None or not math.isfinite(value):
        return "WAIT"
    if value < warn:
        return "OK"
    if value < bad:
        return "WARN"
    return "BAD"


def pose_from_msg(msg: Any) -> Optional[tuple[float, float, float]]:
    try:
        if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
            p = msg.pose.pose.position
        elif hasattr(msg, "pose"):
            p = msg.pose.position
        else:
            return None
        return float(p.x), float(p.y), float(p.z)
    except Exception:
        return None


def twist_speed_from_msg(msg: Any) -> Optional[float]:
    try:
        t = msg.twist.twist
        return math.sqrt(float(t.linear.x) ** 2 + float(t.linear.y) ** 2 + float(t.linear.z) ** 2)
    except Exception:
        return None


@dataclass
class Track:
    label: str
    topic: str
    type_name: str = ""
    subscribed: bool = False
    last_time: Optional[float] = None
    prev_time: Optional[float] = None
    pos: Optional[tuple[float, float, float]] = None
    prev_pos: Optional[tuple[float, float, float]] = None
    speed_mps: Optional[float] = None
    samples: int = 0

    def update(self, msg: Any) -> None:
        now = time.monotonic()
        pos = pose_from_msg(msg)
        if pos is None:
            return
        self.prev_time = self.last_time
        self.prev_pos = self.pos
        self.last_time = now
        self.pos = pos
        self.samples += 1

        twist_speed = twist_speed_from_msg(msg)
        if twist_speed is not None and math.isfinite(twist_speed):
            self.speed_mps = twist_speed
            return
        if self.prev_pos is not None and self.prev_time is not None and now > self.prev_time:
            dx = pos[0] - self.prev_pos[0]
            dy = pos[1] - self.prev_pos[1]
            dz = pos[2] - self.prev_pos[2]
            self.speed_mps = math.sqrt(dx * dx + dy * dy + dz * dz) / (now - self.prev_time)

    def age(self) -> Optional[float]:
        if self.last_time is None:
            return None
        return time.monotonic() - self.last_time

    def status(self) -> str:
        age = self.age()
        if age is None:
            return "WAIT"
        if age > stale_s:
            return "STALE"
        speed = self.speed_mps
        if speed is None:
            return "LIVE"
        return "MOVING" if speed >= 0.05 else "HOLD"


@dataclass
class Scalar:
    label: str
    topic: str
    type_name: str = ""
    subscribed: bool = False
    last_time: Optional[float] = None
    value: Optional[float] = None

    def update(self, msg: Any) -> None:
        try:
            self.value = float(msg.data)
            self.last_time = time.monotonic()
        except Exception:
            return

    def age(self) -> Optional[float]:
        if self.last_time is None:
            return None
        return time.monotonic() - self.last_time


class Monitor(Node):
    def __init__(self) -> None:
        super().__init__("motion_monitor")
        self.tracks = [
            Track("UGV", ugv_topic),
            Track("UAV", uav_topic),
        ]
        self.scalars = []
        if show_omnet:
            self.scalars = [
                Scalar("PDR", "/omnet/packet_delivery_ratio"),
                Scalar("RSSI", "/omnet/rssi_dbm"),
                Scalar("SNIR", "/omnet/snir_db"),
            ]
        self.subscriptions_held = []
        self.create_timer(1.0, self.resolve_subscriptions)
        self.create_timer(1.0 / rate_hz, self.render)

    def resolve_subscriptions(self) -> None:
        names = {}
        for name, types in self.get_topic_names_and_types():
            if types:
                names[name] = types[0]

        for item in self.tracks + self.scalars:
            if item.subscribed:
                continue
            type_name = names.get(item.topic)
            if not type_name:
                continue
            try:
                msg_type = get_message(type_name)
            except Exception:
                continue
            item.type_name = type_name
            callback = item.update
            self.subscriptions_held.append(self.create_subscription(msg_type, item.topic, callback, 10))
            item.subscribed = True

    def render_track(self, track: Track) -> str:
        p = track.pos
        x = fmt(p[0], 2) if p else "--"
        y = fmt(p[1], 2) if p else "--"
        z = fmt(p[2], 2) if p else "--"
        age = track.age()
        age_s = fmt(age, 1, "s")
        speed = fmt(track.speed_mps, 2, "m/s")
        freshness = bar(age, stale_s * 0.5, stale_s)
        return (
            f"{track.label:<4} {track.status():<7} {freshness:<5} "
            f"age={age_s:<7} speed={speed:<9} "
            f"x={x:<9} y={y:<9} z={z:<7} samples={track.samples:<5} "
            f"{track.topic}"
        )

    def render_link(self) -> list[str]:
        ugv = self.tracks[0]
        uav = self.tracks[1]
        if ugv.pos is None or uav.pos is None:
            return ["LINK WAIT    waiting for both UGV and UAV poses"]
        dx = uav.pos[0] - ugv.pos[0]
        dy = uav.pos[1] - ugv.pos[1]
        dz = uav.pos[2] - ugv.pos[2]
        xy = math.hypot(dx, dy)
        d3 = math.sqrt(dx * dx + dy * dy + dz * dz)
        err = d3 - target_m
        fresh = (ugv.age() is not None and ugv.age() <= stale_s and uav.age() is not None and uav.age() <= stale_s)
        dist_ok = abs(err) <= tol_m
        state = "OK" if fresh and dist_ok else ("STALE" if not fresh else "WARN")
        return [
            (
                f"LINK {state:<5} 3d={d3:6.2f}m  xy={xy:6.2f}m  dz={dz:6.2f}m  "
                f"target={target_m:.1f}m  err={err:+.2f}m  tol=+/-{tol_m:.1f}m"
            )
        ]

    def render_omnet(self) -> list[str]:
        if not self.scalars:
            return []
        parts = []
        for scalar in self.scalars:
            age = scalar.age()
            suffix = ""
            value = scalar.value
            if scalar.label == "PDR" and value is not None and math.isfinite(value):
                value *= 100.0
                suffix = "%"
            elif scalar.label == "RSSI":
                suffix = "dBm"
            elif scalar.label == "SNIR":
                suffix = "dB"
            parts.append(f"{scalar.label}={fmt(value, 2, suffix)} age={fmt(age, 1, 's')}")
        return ["OMNET " + "  ".join(parts)]

    def render(self) -> None:
        clear_screen()
        print("Motion monitor  Ctrl-C to stop")
        print("-" * 118)
        print("Debug-only: reads UGV pose/odom for operator monitoring, not for UAV tracking.")
        print("-" * 118)
        for track in self.tracks:
            print(self.render_track(track))
        for line in self.render_link():
            print(line)
        for line in self.render_omnet():
            print(line)
        print("-" * 118)
        print(f"refresh={rate_hz:g}Hz stale={stale_s:g}s")
        sys.stdout.flush()
        if once:
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
PY

#!/usr/bin/env python3
import argparse
import csv
import math
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# Reuse your existing pose service caller from gz_pose.py if you want.
# To keep this self-contained, we use the standard Gazebo service interface via ros_gz_interfaces.
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Quaternion


def yaw_deg_to_quat(yaw_deg: float) -> Quaternion:
    yaw = math.radians(yaw_deg)
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


class UavFollower(Node):
    def __init__(self, world: str, uav: str, odom_topic: str):
        super().__init__("uav_follow_ugv")

        self.world = world
        self.uav = uav
        self.odom_topic = odom_topic

        self.ugv_xy: Optional[Tuple[float, float]] = None

        self.sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_cb,
            10
        )

        self.cli = self.create_client(SetEntityPose, f"/world/{world}/set_pose")

    def _odom_cb(self, msg: Odometry) -> None:
        self.ugv_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def wait_for_service(self, timeout_s: float = 15.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s:
            if self.cli.wait_for_service(timeout_sec=1.0):
                return True
            self.get_logger().info("service not available, waiting...")
        return False

    def wait_for_ugv_pose(self, timeout_s: float = 10.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ugv_xy is not None:
                return True
        return False

    def setpose(self, x: float, y: float, z: float, yaw_deg: float, timeout_s: float = 2.0) -> bool:
        req = SetEntityPose.Request()
        req.entity.name = self.uav

        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = yaw_deg_to_quat(yaw_deg)
        req.pose = pose

        fut = self.cli.call_async(req)

        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if fut.done():
                try:
                    resp = fut.result()
                    return bool(resp.success)
                except Exception:
                    return False
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--uav", required=True)
    ap.add_argument("--odom_topic", default="/a201_0000/platform/odom")
    ap.add_argument("--steps", type=int, default=20)
    ap.add_argument("--z", type=float, default=10.0)
    ap.add_argument("--offset_x", type=float, default=0.0)
    ap.add_argument("--offset_y", type=float, default=0.0)
    ap.add_argument("--yaw_mode", choices=["fixed", "face_ugv"], default="fixed")
    ap.add_argument("--yaw_deg", type=float, default=0.0)
    ap.add_argument("--dt", type=float, default=2.0, help="seconds between teleports")
    ap.add_argument("--log_csv", default="", help="CSV path for commanded setpoints")
    args = ap.parse_args()

    rclpy.init()
    node = UavFollower(args.world, args.uav, args.odom_topic)

    if not node.wait_for_service(15.0):
        node.get_logger().error(f"set_pose service not available: /world/{args.world}/set_pose")
        rclpy.shutdown()
        raise SystemExit(2)

    if not node.wait_for_ugv_pose(10.0):
        node.get_logger().error(f"UGV odom not available on {args.odom_topic}")
        rclpy.shutdown()
        raise SystemExit(3)

    csv_f = None
    csv_w = None
    if args.log_csv:
        p = Path(args.log_csv).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        csv_f = p.open("w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow(["#world", args.world])
        csv_w.writerow(["#uav", args.uav])
        csv_w.writerow(["#odom_topic", args.odom_topic])
        csv_w.writerow(["#steps", args.steps, "#dt", args.dt])
        csv_w.writerow(["#z", args.z, "#offset_x", args.offset_x, "#offset_y", args.offset_y])
        csv_w.writerow(["t_unix_s", "seq", "ugv_x", "ugv_y", "x_cmd", "y_cmd", "z_cmd", "yaw_deg_cmd", "ok"])

    for i in range(args.steps):
        # refresh odom
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)

        ugv_x, ugv_y = node.ugv_xy if node.ugv_xy is not None else (0.0, 0.0)
        x_cmd = ugv_x + args.offset_x
        y_cmd = ugv_y + args.offset_y

        if args.yaw_mode == "face_ugv":
            # face towards the UGV from the commanded UAV position
            dx = ugv_x - x_cmd
            dy = ugv_y - y_cmd
            yaw = math.degrees(math.atan2(dy, dx))
        else:
            yaw = args.yaw_deg

        ok = node.setpose(x_cmd, y_cmd, args.z, yaw, timeout_s=2.0)

        if csv_w:
            csv_w.writerow([time.time(), i, ugv_x, ugv_y, x_cmd, y_cmd, args.z, yaw, int(bool(ok))])

        time.sleep(args.dt)

    if csv_f:
        csv_f.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

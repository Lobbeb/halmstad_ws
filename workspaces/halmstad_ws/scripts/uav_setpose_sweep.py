#!/usr/bin/env python3
import math, time, argparse
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from tf_transformations import quaternion_from_euler
import csv
from pathlib import Path

class Sweeper(Node):
    def __init__(self, world):
        super().__init__('uav_setpose_sweeper')
        self.cli = self.create_client(SetEntityPose, f'/world/{world}/set_pose')
        self.world = world

    def wait_service(self, max_wait_s=10.0):
        waited = 0.0
        while not self.cli.wait_for_service(timeout_sec=1.0):
            waited += 1.0
            self.get_logger().info(f"service not available, waiting... ({waited:.0f}/{max_wait_s:.0f}s)")
            if waited >= max_wait_s:
                return False
        return True

    def call_setpose(self, name, x, y, z, yaw_deg):
        req = SetEntityPose.Request()
        q = quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg))
        req.entity.id = 0
        req.entity.name = name
        req.entity.type = req.entity.MODEL
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.x = q[0]
        req.pose.orientation.y = q[1]
        req.pose.orientation.z = q[2]
        req.pose.orientation.w = q[3]
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        return fut.done()

def frange(a, b, n):
    if n <= 1: return [a]
    step = (b - a) / (n - 1)
    return [a + i*step for i in range(n)]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--uav", required=True)
    ap.add_argument("--x_min", type=float, default=-15.0)
    ap.add_argument("--x_max", type=float, default=15.0)
    ap.add_argument("--x_steps", type=int, default=6)
    ap.add_argument("--y_min", type=float, default=-25.0)
    ap.add_argument("--y_max", type=float, default=25.0)
    ap.add_argument("--y_steps", type=int, default=10)
    ap.add_argument("--z", type=float, default=10.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--wait", type=float, default=0.5)
    ap.add_argument("--log_csv", default="", help="Optional CSV path to log commanded setpoints")

    args = ap.parse_args()

    rclpy.init()
    node = Sweeper(args.world)

    if not node.wait_service(15.0):
        raise SystemExit("ERROR: set_pose service not available")

    xs = frange(args.x_min, args.x_max, args.x_steps)
    ys = frange(args.y_min, args.y_max, args.y_steps)

    log_path = Path(args.log_csv).expanduser() if args.log_csv else None
    csv_f = None
    csv_w = None
    seq = 0

    if log_path:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        csv_f = log_path.open("w", newline="")
        csv_w = csv.writer(csv_f)
        csv_w.writerow(["#world", args.world])
        csv_w.writerow(["#uav", args.uav])
        csv_w.writerow(["#x_min", args.x_min, "#x_max", args.x_max, "#x_steps", args.x_steps])
        csv_w.writerow(["#y_min", args.y_min, "#y_max", args.y_max, "#y_steps", args.y_steps])
        csv_w.writerow(["#z", args.z, "#yaw_deg", args.yaw, "#wait_s", args.wait])
        csv_w.writerow(["t_unix_s", "seq", "x", "y", "z", "yaw_deg", "ok"])

    for y in ys:
        for x in xs:
            ok = node.call_setpose(args.uav, x, y, args.z, args.yaw)
            if csv_w:
                csv_w.writerow([time.time(), seq, float(x), float(y), float(args.z), float(args.yaw), int(bool(ok))])
                seq += 1

            if not ok:
                node.get_logger().warn(f"setpose timeout at x={x:.2f} y={y:.2f}")
            time.sleep(args.wait)

    if csv_f:
        csv_f.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

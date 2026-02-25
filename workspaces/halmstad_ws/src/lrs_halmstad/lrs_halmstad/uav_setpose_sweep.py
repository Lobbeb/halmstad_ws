#!/usr/bin/env python3

import csv
import math
import time
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from tf_transformations import quaternion_from_euler


class UavSetposeSweep(Node):
    def __init__(self) -> None:
        super().__init__("uav_setpose_sweep")

        self.declare_parameter("world", "orchard")
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("x_min", -15.0)
        self.declare_parameter("x_max", 15.0)
        self.declare_parameter("x_steps", 4)
        self.declare_parameter("y_min", -25.0)
        self.declare_parameter("y_max", 25.0)
        self.declare_parameter("y_steps", 5)
        self.declare_parameter("z_m", 10.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("wait_s", 1.0)
        self.declare_parameter("service_wait_s", 15.0)
        self.declare_parameter("request_timeout_s", 1.0)
        self.declare_parameter("start_delay_s", 0.0)
        self.declare_parameter("log_csv", "")

        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.x_steps = max(1, int(self.get_parameter("x_steps").value))
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.y_steps = max(1, int(self.get_parameter("y_steps").value))
        self.z_m = float(self.get_parameter("z_m").value)
        self.yaw_deg = float(self.get_parameter("yaw_deg").value)
        self.wait_s = max(0.0, float(self.get_parameter("wait_s").value))
        self.service_wait_s = max(0.1, float(self.get_parameter("service_wait_s").value))
        self.request_timeout_s = max(0.1, float(self.get_parameter("request_timeout_s").value))
        self.start_delay_s = max(0.0, float(self.get_parameter("start_delay_s").value))
        self.log_csv = str(self.get_parameter("log_csv").value)

        self._cli = self.create_client(SetEntityPose, f"/world/{self.world}/set_pose")

    def _wait_service(self) -> bool:
        deadline = time.monotonic() + self.service_wait_s
        while time.monotonic() < deadline:
            if self._cli.wait_for_service(timeout_sec=1.0):
                return True
            remaining = max(0.0, deadline - time.monotonic())
            self.get_logger().info(
                f"Waiting for /world/{self.world}/set_pose (remaining ~{remaining:.0f}s)"
            )
        return False

    @staticmethod
    def _frange(a: float, b: float, n: int) -> List[float]:
        if n <= 1:
            return [float(a)]
        step = (b - a) / float(n - 1)
        return [a + i * step for i in range(n)]

    def _call_setpose(self, x: float, y: float, z: float, yaw_deg: float) -> bool:
        req = SetEntityPose.Request()
        q = quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg))
        req.entity.id = 0
        req.entity.name = self.uav_name
        req.entity.type = req.entity.MODEL
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.x = float(q[0])
        req.pose.orientation.y = float(q[1])
        req.pose.orientation.z = float(q[2])
        req.pose.orientation.w = float(q[3])

        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.request_timeout_s)
        if not fut.done():
            return False
        try:
            _ = fut.result()
        except Exception as exc:  # pragma: no cover - runtime ROS transport exceptions
            self.get_logger().warn(f"set_pose call failed: {exc}")
            return False
        return True

    def run(self) -> int:
        if not self._wait_service():
            self.get_logger().error(f"Service /world/{self.world}/set_pose not available")
            return 2

        if self.start_delay_s > 0.0:
            self.get_logger().info(f"Start delay {self.start_delay_s:.1f}s before UAV sweep")
            time.sleep(self.start_delay_s)

        xs = self._frange(self.x_min, self.x_max, self.x_steps)
        ys = self._frange(self.y_min, self.y_max, self.y_steps)
        total = len(xs) * len(ys)
        self.get_logger().info(
            f"Starting UAV sweep: world={self.world} uav={self.uav_name} points={total} "
            f"grid=({self.x_steps}x{self.y_steps}) z={self.z_m:.2f} wait={self.wait_s:.2f}s"
        )

        csv_f = None
        csv_w = None
        if self.log_csv:
            log_path = Path(self.log_csv).expanduser()
            log_path.parent.mkdir(parents=True, exist_ok=True)
            csv_f = log_path.open("w", newline="")
            csv_w = csv.writer(csv_f)
            csv_w.writerow(["#world", self.world])
            csv_w.writerow(["#uav", self.uav_name])
            csv_w.writerow(["#x_min", self.x_min, "#x_max", self.x_max, "#x_steps", self.x_steps])
            csv_w.writerow(["#y_min", self.y_min, "#y_max", self.y_max, "#y_steps", self.y_steps])
            csv_w.writerow(["#z_m", self.z_m, "#yaw_deg", self.yaw_deg, "#wait_s", self.wait_s])
            csv_w.writerow(["t_unix_s", "seq", "x", "y", "z", "yaw_deg", "ok"])

        seq = 0
        ok_count = 0
        try:
            for y in ys:
                for x in xs:
                    ok = self._call_setpose(x, y, self.z_m, self.yaw_deg)
                    ok_count += int(ok)
                    if csv_w:
                        csv_w.writerow([
                            time.time(),
                            seq,
                            float(x),
                            float(y),
                            float(self.z_m),
                            float(self.yaw_deg),
                            int(bool(ok)),
                        ])
                    if not ok:
                        self.get_logger().warn(f"set_pose timeout/failure at x={x:.2f} y={y:.2f}")
                    seq += 1
                    if self.wait_s > 0.0:
                        time.sleep(self.wait_s)
        finally:
            if csv_f:
                csv_f.close()

        self.get_logger().info(f"UAV sweep complete ({ok_count}/{total} successful set_pose calls)")
        return 0 if ok_count > 0 else 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavSetposeSweep()
    rc = 0
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()

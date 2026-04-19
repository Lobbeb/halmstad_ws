#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter


def publish(pub, clock, linear_x, angular_z):
    msg = TwistStamped()
    msg.header.stamp = clock.now().to_msg()
    msg.twist.linear.x = linear_x
    msg.twist.angular.z = angular_z
    pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="Publish a stamped twist command for a fixed duration.")
    parser.add_argument("--topic", required=True, help="Target TwistStamped topic")
    parser.add_argument("--linear-x", type=float, default=0.0, help="Linear x velocity in m/s")
    parser.add_argument("--angular-z", type=float, default=0.0, help="Angular z velocity in rad/s")
    parser.add_argument("--duration-s", type=float, required=True, help="Publish duration in seconds")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="Publish rate in Hz")
    parser.add_argument("--use-sim-time", action="store_true", help="Use ROS sim time for stamps")
    parser.add_argument("--stop-burst", type=int, default=5, help="Zero-twist messages to send after motion")
    args = parser.parse_args()

    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    if args.duration_s < 0.0:
        raise SystemExit("--duration-s must be non-negative")

    rclpy.init(args=None)
    node = rclpy.create_node(
        "publish_twist_stamped",
        parameter_overrides=[Parameter("use_sim_time", value=args.use_sim_time)],
    )
    pub = node.create_publisher(TwistStamped, args.topic, 10)

    period_s = 1.0 / args.rate_hz
    end_time = time.monotonic() + args.duration_s

    try:
        # Prime the clock once so sim time can start flowing before the first command.
        rclpy.spin_once(node, timeout_sec=0.1)

        while time.monotonic() < end_time:
            publish(pub, node.get_clock(), args.linear_x, args.angular_z)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)

        for _ in range(max(1, args.stop_burst)):
            publish(pub, node.get_clock(), 0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

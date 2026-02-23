#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriveUgv(Node):
    def __init__(self):
        super().__init__("drive_ugv")
        self.declare_parameter("cmd_vel_topic", "/x2_ugv/cmd_vel")
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("ugv_mps", -1.0)

        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._linear_speed = self.get_parameter("linear_speed").get_parameter_value().double_value
        self._angular_speed = self.get_parameter("angular_speed").get_parameter_value().double_value
        ugv_mps = float(self.get_parameter("ugv_mps").value)
        if ugv_mps > 0.0:
            self._linear_speed = ugv_mps

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._start_time = self.get_clock().now()
        self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            f"Publishing ROS Twist to {self._cmd_vel_topic} (bridged to Gazebo), "
            f"linear={self._linear_speed:.2f} m/s angular={self._angular_speed:.2f} rad/s"
        )

    def _on_timer(self):
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        linear_x = 0.0
        angular_z = 0.0
        # Pattern: forward 5s, stop 1s, rotate 3s, stop 1s, repeat
        cycle = 10.0
        t = math.fmod(elapsed, cycle)

        if t < 5.0:
            linear_x = self._linear_speed
        elif t < 6.0:
            pass
        elif t < 9.0:
            angular_z = self._angular_speed
        else:
            pass

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = DriveUgv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

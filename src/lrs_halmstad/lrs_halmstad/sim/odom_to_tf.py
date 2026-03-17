#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _clean_frame_id(value: str) -> str:
    return str(value).lstrip('/')


class OdomToTf(Node):
    def __init__(self) -> None:
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic', 'platform/odom/filtered')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('child_frame_id', '')
        self.declare_parameter('copy_header_stamp', True)

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._frame_id_override = _clean_frame_id(self.get_parameter('frame_id').value)
        self._child_frame_id_override = _clean_frame_id(self.get_parameter('child_frame_id').value)
        self._copy_header_stamp = bool(self.get_parameter('copy_header_stamp').value)

        self._broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self._msg_count = 0

        self.get_logger().info(
            f'Publishing TF from odom topic {self._odom_topic} '
            f'(frame_id={self._frame_id_override or "<msg>"}, '
            f'child_frame_id={self._child_frame_id_override or "<msg>"})'
        )

    def _on_odom(self, msg: Odometry) -> None:
        frame_id = self._frame_id_override or _clean_frame_id(msg.header.frame_id) or 'odom'
        child_frame_id = self._child_frame_id_override or _clean_frame_id(msg.child_frame_id) or 'base_link'

        tf_msg = TransformStamped()
        if self._copy_header_stamp:
            tf_msg.header.stamp = msg.header.stamp
        else:
            tf_msg.header.stamp = self.get_clock().now().to_msg()

        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation

        self._broadcaster.sendTransform(tf_msg)

        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info(
                f'First odom received on {self._odom_topic}; TF {frame_id} -> {child_frame_id} is now available'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
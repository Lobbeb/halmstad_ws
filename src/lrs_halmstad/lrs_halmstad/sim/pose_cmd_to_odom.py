#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class _PoseToOdomBase(Node):
    """Shared logic for pose-to-odometry bridge nodes.

    Subclasses call ``_init_bridge(default_pose_topic, default_odom_topic)``
    in their ``__init__``, create their own subscription (message type and QoS
    vary between subclasses), and implement ``_on_pose`` to call
    ``_build_odom`` and publish ``self._pub``.
    """

    def _init_bridge(self, default_pose_topic: str, default_odom_topic: str) -> None:
        self.declare_parameter("pose_topic", default_pose_topic)
        self.declare_parameter("odom_topic", default_odom_topic)
        self.declare_parameter("frame_id", "")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("copy_header_stamp", True)

        self._pose_topic = str(self.get_parameter("pose_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._frame_id_override = str(self.get_parameter("frame_id").value)
        self._child_frame_id = str(self.get_parameter("child_frame_id").value)
        self._copy_header_stamp = bool(self.get_parameter("copy_header_stamp").value)
        self._msg_count = 0
        self._pub = self.create_publisher(Odometry, self._odom_topic, 10)

    def _build_odom(self, source_stamp, source_frame_id: str) -> Odometry:
        odom = Odometry()
        odom.header.stamp = (
            source_stamp if self._copy_header_stamp else self.get_clock().now().to_msg()
        )
        odom.header.frame_id = self._frame_id_override or source_frame_id or "map"
        odom.child_frame_id = self._child_frame_id
        return odom

    def _log_first_pose(self) -> None:
        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info(
                f"First pose received on {self._pose_topic}; "
                f"odom is now available on {self._odom_topic}"
            )


class PoseCmdToOdom(_PoseToOdomBase):
    def __init__(self) -> None:
        super().__init__("pose_cmd_to_odom")
        self._init_bridge("/dji0/pose_cmd", "/dji0/pose_cmd/odom")
        self._sub = self.create_subscription(
            PoseStamped, self._pose_topic, self._on_pose, 10
        )
        self.get_logger().info(
            f"Publishing synthetic odom {self._odom_topic} from pose commands {self._pose_topic} "
            f"(child_frame_id={self._child_frame_id or '<empty>'})"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        odom = self._build_odom(msg.header.stamp, msg.header.frame_id)
        odom.pose.pose = msg.pose
        self._pub.publish(odom)
        self._log_first_pose()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseCmdToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

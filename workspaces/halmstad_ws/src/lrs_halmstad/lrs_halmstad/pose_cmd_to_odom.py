#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PoseCmdToOdom(Node):
    def __init__(self) -> None:
        super().__init__("pose_cmd_to_odom")

        self.declare_parameter("pose_topic", "/dji0/pose_cmd")
        self.declare_parameter("odom_topic", "/dji0/pose_cmd/odom")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("copy_header_stamp", True)

        self._pose_topic = str(self.get_parameter("pose_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._frame_id_override = str(self.get_parameter("frame_id").value)
        self._child_frame_id = str(self.get_parameter("child_frame_id").value)
        self._copy_header_stamp = bool(self.get_parameter("copy_header_stamp").value)

        self._pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._sub = self.create_subscription(PoseStamped, self._pose_topic, self._on_pose, 10)
        self._msg_count = 0

        self.get_logger().info(
            f"Publishing synthetic odom {self._odom_topic} from pose commands {self._pose_topic} "
            f"(child_frame_id={self._child_frame_id or '<empty>'})"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        odom = Odometry()

        if self._copy_header_stamp:
            odom.header.stamp = msg.header.stamp
        else:
            odom.header.stamp = self.get_clock().now().to_msg()

        odom.header.frame_id = self._frame_id_override or msg.header.frame_id or "map"
        odom.child_frame_id = self._child_frame_id
        odom.pose.pose = msg.pose
        # Twist is left at zero because the source is a commanded/teleport pose stream.
        # This keeps the topic type-compatible for consumers that only need pose/orientation.

        self._pub.publish(odom)

        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info(
                f"First pose received on {self._pose_topic}; odom is now available on {self._odom_topic}"
            )


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

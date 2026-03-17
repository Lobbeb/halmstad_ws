#!/usr/bin/env python3

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PoseCovToOdom(Node):
    def __init__(self) -> None:
        super().__init__("pose_cov_to_odom")

        self.declare_parameter("pose_topic", "amcl_pose")
        self.declare_parameter("odom_topic", "amcl_pose_odom")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("copy_header_stamp", True)

        self._pose_topic = str(self.get_parameter("pose_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._frame_id_override = str(self.get_parameter("frame_id").value)
        self._child_frame_id = str(self.get_parameter("child_frame_id").value)
        self._copy_header_stamp = bool(self.get_parameter("copy_header_stamp").value)

        amcl_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        amcl_volatile_qos = QoSProfile(
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._subs = [
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._pose_topic,
                self._on_pose,
                amcl_qos,
            ),
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._pose_topic,
                self._on_pose,
                amcl_volatile_qos,
            ),
        ]
        self._msg_count = 0

        self.get_logger().info(
            f"Publishing synthetic odom {self._odom_topic} from pose source {self._pose_topic} "
            f"(child_frame_id={self._child_frame_id or '<empty>'})"
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        odom = Odometry()

        if self._copy_header_stamp:
            odom.header.stamp = msg.header.stamp
        else:
            odom.header.stamp = self.get_clock().now().to_msg()

        odom.header.frame_id = self._frame_id_override or msg.header.frame_id or "map"
        odom.child_frame_id = self._child_frame_id
        odom.pose = msg.pose
        # Twist is intentionally left at zero because AMCL provides pose only.

        self._pub.publish(odom)

        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info(
                f"First pose received on {self._pose_topic}; odom is now available on {self._odom_topic}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseCovToOdom()
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


if __name__ == "__main__":
    main()

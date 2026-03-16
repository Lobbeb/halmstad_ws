#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from lrs_halmstad.sim.pose_cmd_to_odom import _PoseToOdomBase


class PoseCovToOdom(_PoseToOdomBase):
    def __init__(self) -> None:
        super().__init__("pose_cov_to_odom")
        self._init_bridge("amcl_pose", "amcl_pose_odom")
        amcl_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            PoseWithCovarianceStamped, self._pose_topic, self._on_pose, amcl_qos
        )
        self.get_logger().info(
            f"Publishing synthetic odom {self._odom_topic} from pose source {self._pose_topic} "
            f"(child_frame_id={self._child_frame_id or '<empty>'})"
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        odom = self._build_odom(msg.header.stamp, msg.header.frame_id)
        odom.pose = msg.pose
        self._pub.publish(odom)
        self._log_first_pose()


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

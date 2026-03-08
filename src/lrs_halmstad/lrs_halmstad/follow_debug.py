from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String

from lrs_halmstad.follow_math import Pose2D, quat_from_yaw


class FollowDebugPublishers:
    def __init__(self, node, uav_name: str):
        self.follow_target_anchor_pub = node.create_publisher(
            PoseStamped,
            f"/{uav_name}/follow/target/anchor_pose",
            10,
        )
        self.follow_target_d_target_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/target/d_target_m",
            10,
        )
        self.follow_target_z_alt_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/target/z_alt_m",
            10,
        )
        self.follow_target_d_euclidean_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/target/d_euclidean_m",
            10,
        )
        self.follow_target_yaw_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/target/yaw_rad",
            10,
        )
        self.follow_actual_xy_distance_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/actual/xy_distance_m",
            10,
        )
        self.follow_actual_distance_3d_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/actual/distance_3d_m",
            10,
        )
        self.follow_actual_yaw_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/actual/yaw_rad",
            10,
        )
        self.follow_error_xy_distance_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/error/xy_distance_m",
            10,
        )
        self.follow_error_anchor_distance_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/error/anchor_distance_m",
            10,
        )
        self.follow_error_anchor_along_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/error/anchor_along_m",
            10,
        )
        self.follow_error_anchor_cross_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/error/anchor_cross_m",
            10,
        )
        self.follow_error_yaw_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/error/yaw_rad",
            10,
        )
        self.follow_debug_yaw_target_raw_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/debug/yaw_target_raw_rad",
            10,
        )
        self.follow_debug_yaw_target_unwrapped_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/debug/yaw_target_unwrapped_rad",
            10,
        )
        self.follow_debug_yaw_actual_unwrapped_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/debug/yaw_actual_unwrapped_rad",
            10,
        )
        self.follow_debug_yaw_error_raw_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/debug/yaw_error_raw_rad",
            10,
        )
        self.follow_debug_yaw_wrap_correction_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/debug/yaw_wrap_correction_rad",
            10,
        )
        self.follow_debug_yaw_wrap_active_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/debug/yaw_wrap_active",
            10,
        )
        self.follow_debug_yaw_step_limit_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/debug/yaw_step_limit_rad",
            10,
        )
        self.follow_debug_yaw_cmd_delta_pub = node.create_publisher(
            Float32,
            f"/{uav_name}/follow/debug/yaw_cmd_delta_rad",
            10,
        )
        self.follow_debug_yaw_mode_pub = node.create_publisher(
            String,
            f"/{uav_name}/follow/debug/yaw_mode",
            10,
        )

    def publish(
        self,
        *,
        stamp,
        frame_id: str,
        anchor_target: Pose2D,
        d_target: float,
        z_alt: float,
        d_euclidean: float,
        actual_xy_distance: float,
        actual_distance_3d: float,
        actual_yaw: float,
        target_yaw: float,
        xy_distance_error: float,
        anchor_distance_error: float,
        anchor_along_error: float,
        anchor_cross_error: float,
        yaw_error: float,
        yaw_target_raw: float | None = None,
        yaw_target_unwrapped: float | None = None,
        yaw_actual_unwrapped: float | None = None,
        yaw_error_raw: float | None = None,
        yaw_wrap_correction: float | None = None,
        yaw_wrap_active: float | None = None,
        yaw_step_limit: float | None = None,
        yaw_cmd_delta: float | None = None,
        yaw_mode: str | None = None,
    ) -> None:
        anchor_msg = PoseStamped()
        anchor_msg.header.stamp = stamp
        anchor_msg.header.frame_id = frame_id
        anchor_msg.pose.position.x = float(anchor_target.x)
        anchor_msg.pose.position.y = float(anchor_target.y)
        anchor_msg.pose.position.z = float(z_alt)
        quat = quat_from_yaw(float(anchor_target.yaw))
        anchor_msg.pose.orientation.x = float(quat[0])
        anchor_msg.pose.orientation.y = float(quat[1])
        anchor_msg.pose.orientation.z = float(quat[2])
        anchor_msg.pose.orientation.w = float(quat[3])
        self.follow_target_anchor_pub.publish(anchor_msg)

        self._publish_scalar(self.follow_target_d_target_pub, d_target)
        self._publish_scalar(self.follow_target_z_alt_pub, z_alt)
        self._publish_scalar(self.follow_target_d_euclidean_pub, d_euclidean)
        self._publish_scalar(self.follow_target_yaw_pub, target_yaw)

        self._publish_scalar(self.follow_actual_xy_distance_pub, actual_xy_distance)
        self._publish_scalar(self.follow_actual_distance_3d_pub, actual_distance_3d)
        self._publish_scalar(self.follow_actual_yaw_pub, actual_yaw)

        self._publish_scalar(self.follow_error_xy_distance_pub, xy_distance_error)
        self._publish_scalar(self.follow_error_anchor_distance_pub, anchor_distance_error)
        self._publish_scalar(self.follow_error_anchor_along_pub, anchor_along_error)
        self._publish_scalar(self.follow_error_anchor_cross_pub, anchor_cross_error)
        self._publish_scalar(self.follow_error_yaw_pub, yaw_error)
        self._publish_optional_scalar(self.follow_debug_yaw_target_raw_pub, yaw_target_raw)
        self._publish_optional_scalar(self.follow_debug_yaw_target_unwrapped_pub, yaw_target_unwrapped)
        self._publish_optional_scalar(self.follow_debug_yaw_actual_unwrapped_pub, yaw_actual_unwrapped)
        self._publish_optional_scalar(self.follow_debug_yaw_error_raw_pub, yaw_error_raw)
        self._publish_optional_scalar(self.follow_debug_yaw_wrap_correction_pub, yaw_wrap_correction)
        self._publish_optional_scalar(self.follow_debug_yaw_wrap_active_pub, yaw_wrap_active)
        self._publish_optional_scalar(self.follow_debug_yaw_step_limit_pub, yaw_step_limit)
        self._publish_optional_scalar(self.follow_debug_yaw_cmd_delta_pub, yaw_cmd_delta)
        self._publish_optional_string(self.follow_debug_yaw_mode_pub, yaw_mode)

    @staticmethod
    def _publish_scalar(pub, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    @classmethod
    def _publish_optional_scalar(cls, pub, value: float | None) -> None:
        if value is None:
            return
        cls._publish_scalar(pub, value)

    @staticmethod
    def _publish_optional_string(pub, value: str | None) -> None:
        if value is None:
            return
        msg = String()
        msg.data = value
        pub.publish(msg)

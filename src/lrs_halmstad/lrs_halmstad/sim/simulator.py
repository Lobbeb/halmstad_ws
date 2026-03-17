import re
import math
from typing import Set

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import PoseStamped, Point, Vector3Stamped

from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import Joy
from ros_gz_interfaces.srv import SetEntityPose

from lrs_halmstad.common.ros_params import yaml_param
from lrs_halmstad.common.world_names import gazebo_world_name
from lrs_halmstad.follow.follow_math import rotate_body_offset, wrap_pi, yaw_from_quat

class Simulator(Node):
    def __init__(self):
        super().__init__('uav_simulator')
        self.group = ReentrantCallbackGroup()

        self.declare_parameter("world", "warehouse")
        self.declare_parameter("uav_name", "")
        self.declare_parameter("name", "")
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("camera_mode", "integrated_joint")
        self.declare_parameter("camera_name", "camera0")
        yaml_param(self, "camera_x_offset_m")
        yaml_param(self, "camera_y_offset_m")
        yaml_param(self, "camera_z_offset_m")
        yaml_param(self, "camera_mount_pitch_deg")
        yaml_param(self, "camera_yaw_offset_deg")
        yaml_param(self, "camera_pan_sign")
        yaml_param(self, "gimbal_pitch_min_rad")
        yaml_param(self, "gimbal_pitch_max_rad")
        self.declare_parameter("publish_legacy_debug_topics", False)
        self.declare_parameter("set_pose_future_timeout_s", 0.5)
        yaml_param(self, "pan_rate_deg_s")
        yaml_param(self, "tilt_rate_deg_s")

        self.frame_id = "odom"
        self.world = self.get_parameter("world").value
        self.gz_world = gazebo_world_name(self.world)
        self.uav_name = self._resolve_uav_name()
        self.name = self.uav_name
        self.update_rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.period_time = 1.0 / self.update_rate_hz
        self.camera_mode = str(self.get_parameter("camera_mode").value).strip().lower()
        self.camera_name = str(self.get_parameter("camera_name").value).strip()
        self.camera_x_offset = float(self.get_parameter("camera_x_offset_m").value)
        self.camera_y_offset = float(self.get_parameter("camera_y_offset_m").value)
        self.camera_z_offset = float(self.get_parameter("camera_z_offset_m").value)
        self.camera_mount_pitch_deg = float(self.get_parameter("camera_mount_pitch_deg").value)
        self.camera_yaw_offset_deg = float(self.get_parameter("camera_yaw_offset_deg").value)
        self.camera_pan_sign = float(self.get_parameter("camera_pan_sign").value)
        self.gimbal_pitch_min = float(self.get_parameter("gimbal_pitch_min_rad").value)
        self.gimbal_pitch_max = float(self.get_parameter("gimbal_pitch_max_rad").value)
        self.publish_legacy_debug_topics = bool(self.get_parameter("publish_legacy_debug_topics").value)
        self.set_pose_future_timeout_s = max(0.0, float(self.get_parameter("set_pose_future_timeout_s").value))
        self.pan_rate_deg_s = float(self.get_parameter("pan_rate_deg_s").value)
        self.tilt_rate_deg_s = float(self.get_parameter("tilt_rate_deg_s").value)
        if self.camera_mode == "integrated":
            self.camera_mode = "integrated_joint"
        if self.camera_mode != "integrated_joint":
            self.get_logger().warn(
                f"camera_mode='{self.camera_mode}' is no longer supported; using 'integrated_joint'"
            )
            self.camera_mode = "integrated_joint"

        default_start_x, default_start_y, default_start_z = self._default_start_xyz(self.uav_name)
        self.declare_parameter("start_x", default_start_x)
        self.declare_parameter("start_y", default_start_y)
        self.declare_parameter("start_z", default_start_z)
        self.declare_parameter("start_yaw_deg", 0.0)

        self.world_position = Point()
        self.world_position.x = float(self.get_parameter("start_x").value)
        self.world_position.y = float(self.get_parameter("start_y").value)
        self.world_position.z = float(self.get_parameter("start_z").value)


        self.ns = self.get_namespace()
        print("NAMESPACE:", self.ns)

        self.yaw = math.radians(float(self.get_parameter("start_yaw_deg").value))
        self.update_msg = None
        self.tilt = 0.0
        self.pan = 0.0
        self.target_tilt = None  # None until first command received; no gimbal output until then
        self.target_pan = None
        self.future1 = None
        self.future1_sent_ns = None
        self._last_set_pose = None  # (x, y, z, yaw) — skip set_pose when unchanged
        self._last_tick_time = None

        self.cli = self.create_client(SetEntityPose, f'/world/{self.gz_world}/set_pose', callback_group=self.group)
        print(f"WAIT for service: /world/{self.gz_world}/set_pose'")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        pose_topic = self._uav_topic("pose")
        camera_pose_topic = self._uav_topic("debug_camera_pose")
        cmd_topic = self._uav_topic("psdk_ros2/flight_control_setpoint_ENUposition_yaw")
        tilt_topic = self._uav_topic("update_tilt")
        pan_topic = self._uav_topic("update_pan")
        gimbal_pitch_topic = self._gimbal_topic("pitch")
        gimbal_yaw_topic = self._gimbal_topic("yaw")
        yaw_debug_topic = self._uav_topic("debug_yaw")
        follow_actual_tilt_topic = self._uav_topic("follow/actual/tilt_deg")
        follow_error_tilt_topic = self._uav_topic("follow/error/tilt_deg")
        follow_target_tilt_topic = self._uav_topic("follow/target/tilt_deg")
        follow_target_pan_topic = self._uav_topic("follow/target/pan_deg")
        follow_actual_pan_topic = self._uav_topic("follow/actual/pan_deg")
        follow_error_pan_topic = self._uav_topic("follow/error/pan_deg")
        camera_target_pose_topic = self._camera_topic("target/center_pose")
        camera_actual_pose_topic = self._camera_topic("actual/center_pose")
        camera_target_world_yaw_topic = self._camera_topic("target/world_yaw_rad")
        camera_actual_world_yaw_topic = self._camera_topic("actual/world_yaw_rad")
        camera_error_world_yaw_topic = self._camera_topic("error/world_yaw_rad")

        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10, callback_group=self.group)
        self.camera_pose_pub = (
            self.create_publisher(PoseStamped, camera_pose_topic, 10, callback_group=self.group)
            if self.publish_legacy_debug_topics
            else None
        )
        self.yaw_debug_pub = (
            self.create_publisher(Vector3Stamped, yaw_debug_topic, 10, callback_group=self.group)
            if self.publish_legacy_debug_topics
            else None
        )
        self.follow_target_tilt_pub = self.create_publisher(
            Float32, follow_target_tilt_topic, 10, callback_group=self.group
        )
        self.follow_actual_tilt_pub = self.create_publisher(
            Float32, follow_actual_tilt_topic, 10, callback_group=self.group
        )
        self.follow_error_tilt_pub = self.create_publisher(
            Float32, follow_error_tilt_topic, 10, callback_group=self.group
        )
        self.follow_target_pan_pub = self.create_publisher(
            Float32, follow_target_pan_topic, 10, callback_group=self.group
        )
        self.follow_actual_pan_pub = self.create_publisher(
            Float32, follow_actual_pan_topic, 10, callback_group=self.group
        )
        self.follow_error_pan_pub = self.create_publisher(
            Float32, follow_error_pan_topic, 10, callback_group=self.group
        )
        self.camera_actual_pose_pub = self.create_publisher(
            PoseStamped, camera_actual_pose_topic, 10, callback_group=self.group
        )
        self.camera_actual_world_yaw_pub = self.create_publisher(
            Float32, camera_actual_world_yaw_topic, 10, callback_group=self.group
        )
        self.camera_error_world_yaw_pub = self.create_publisher(
            Float32, camera_error_world_yaw_topic, 10, callback_group=self.group
        )
        self.update_sub = self.create_subscription(
            Joy, cmd_topic, self.update_callback, 10, callback_group=self.group
        )
        self.update_tilt_sub = self.create_subscription(
            Float64, tilt_topic, self.update_tilt_callback, 10, callback_group=self.group
        )
        self.update_pan_sub = self.create_subscription(
            Float64, pan_topic, self.update_pan_callback, 10, callback_group=self.group
        )
        self.camera_target_pose_sub = self.create_subscription(
            PoseStamped, camera_target_pose_topic, self.update_target_camera_pose_callback, 10, callback_group=self.group
        )
        self.camera_target_world_yaw_sub = self.create_subscription(
            Float32, camera_target_world_yaw_topic, self.update_target_camera_world_yaw_callback, 10, callback_group=self.group
        )
        self.gimbal_pitch_pub = self.create_publisher(Float64, gimbal_pitch_topic, 10)
        self.gimbal_yaw_pub = self.create_publisher(Float64, gimbal_yaw_topic, 10)

        self.timer = self.create_timer(self.period_time, self.timer_callback, callback_group=self.group)
        self.get_logger().info(
            f"Using UAV '{self.uav_name}' topics: cmd={cmd_topic}, pose={pose_topic}, "
            f"gimbal_pitch={gimbal_pitch_topic}, debug_yaw={yaw_debug_topic}, "
            f"debug_camera_pose={camera_pose_topic}, "
            f"camera_mode={self.camera_mode}, "
            f"update_rate_hz={self.update_rate_hz:.1f}, "
            f"camera_offset=({self.camera_x_offset:.2f},{self.camera_y_offset:.2f},-{self.camera_z_offset:.2f}), "
            f"camera_mount_pitch_deg={self.camera_mount_pitch_deg:.1f}, "
            f"camera_yaw_offset_deg={self.camera_yaw_offset_deg:.1f}, "
            f"camera_pan_sign={self.camera_pan_sign:.1f}"
        )
        self.get_logger().info(
            f"Initial pose x={self.world_position.x:.2f} y={self.world_position.y:.2f} "
            f"z={self.world_position.z:.2f} yaw_deg={math.degrees(self.yaw):.1f}"
        )
        self.get_logger().info(
            "Legacy UAV command topic is interpreted as absolute ENU pose: x, y, z, yaw."
        )
        self.get_logger().info(
            "update_tilt is interpreted in degrees relative to the horizontal plane."
        )
        self.target_camera_world_yaw = None
        self.target_camera_pose = None

    def _uav_topic(self, suffix: str) -> str:
        return f"/{self.uav_name}/{suffix.lstrip('/')}"

    def _camera_topic(self, suffix: str) -> str:
        return f"/{self.uav_name}/{self.camera_name}/{suffix.lstrip('/')}"

    def _gimbal_topic(self, axis: str) -> str:
        return f"/model/{self.uav_name}/joint/{self.uav_name}_gimbal_joint/{axis}/cmd_pos"

    def _clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _default_start_xyz(self, uav_name: str):
        default_x = 0.0
        default_y = 0.0
        default_z = 7.0
        match = re.match(r"^dji(\d+)$", uav_name)
        if match:
            index = int(match.group(1))
            return default_x, default_y, default_z + float(index)
        return default_x, default_y, default_z

    def _graph_uav_candidates(self) -> Set[str]:
        patterns = [
            re.compile(r"^/([^/]+)/psdk_ros2/flight_control_setpoint_ENUposition_yaw$"),
            re.compile(r"^/([^/]+)/pose$"),
            re.compile(r"^/([^/]+)/pose_cmd(?:/odom)?$"),
            re.compile(r"^/([^/]+)/camera0/(?:image_raw|camera_info)$"),
        ]
        candidates: Set[str] = set()
        for topic_name, _ in self.get_topic_names_and_types():
            for pattern in patterns:
                match = pattern.match(topic_name)
                if match:
                    candidates.add(match.group(1))
                    break
        return candidates

    def _resolve_uav_name(self) -> str:
        explicit_uav = str(self.get_parameter("uav_name").value).strip()
        explicit_name = str(self.get_parameter("name").value).strip()
        if explicit_uav:
            return explicit_uav
        if explicit_name:
            return explicit_name

        namespace = str(self.get_namespace()).strip("/")
        if namespace:
            return namespace

        candidates = sorted(self._graph_uav_candidates())
        if len(candidates) == 1:
            self.get_logger().info(f"Auto-detected UAV from graph: {candidates[0]}")
            return candidates[0]
        if len(candidates) > 1:
            self.get_logger().warn(
                f"Multiple UAVs discovered {candidates}; using '{candidates[0]}'. "
                "Set parameter 'uav_name' to choose explicitly."
            )
            return candidates[0]

        self.get_logger().warn("Could not auto-detect UAV name; defaulting to 'dji0'.")
        return "dji0"

    def update_callback(self, msg):
        self.update_msg = msg

    def update_pan_callback(self, msg):
        new_pan = float(msg.data)
        if self.target_pan is None:
            self.pan = new_pan  # jump current to first command; no simulator-side ramp on arm
        self.target_pan = new_pan

    def update_tilt_callback(self, msg):
        new_tilt = float(msg.data)
        if self.target_tilt is None:
            self.tilt = new_tilt  # jump current to first command
        self.target_tilt = new_tilt

    @staticmethod
    def _step_toward(current: float, target: float, max_step: float) -> float:
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)

    def update_target_camera_pose_callback(self, msg):
        self.target_camera_pose = msg

    def update_target_camera_world_yaw_callback(self, msg):
        self.target_camera_world_yaw = float(msg.data)

    def update(self):
        if self.update_msg:
            if len(self.update_msg.axes) >= 4:
                self.world_position.x = float(self.update_msg.axes[0])
                self.world_position.y = float(self.update_msg.axes[1])
                self.world_position.z = float(self.update_msg.axes[2])
                self.yaw = float(self.update_msg.axes[3])
            self.update_msg = None
            
    def _future_pending(self, future, sent_ns):
        if future is None:
            return False
        if future.done():
            return False
        if sent_ns is None:
            return True
        age_s = (self.get_clock().now().nanoseconds - int(sent_ns)) * 1e-9
        if self.set_pose_future_timeout_s > 0.0 and age_s > self.set_pose_future_timeout_s:
            self.get_logger().warn(
                f"SetEntityPose future timed out after {age_s:.2f}s; clearing pending request"
            )
            return False
        return True

    def timer_callback(self):
        try:
            now = self.get_clock().now()
            if self._last_tick_time is not None:
                dt = max(0.0, (now - self._last_tick_time).nanoseconds * 1e-9)
            else:
                dt = self.period_time
            self._last_tick_time = now
            self.update()
            if self.target_pan is not None:
                if self.pan_rate_deg_s > 0.0:
                    self.pan = self._step_toward(self.pan, self.target_pan, self.pan_rate_deg_s * dt)
                else:
                    self.pan = self.target_pan
            if self.target_tilt is not None:
                if self.tilt_rate_deg_s > 0.0:
                    self.tilt = self._step_toward(self.tilt, self.target_tilt, self.tilt_rate_deg_s * dt)
                else:
                    self.tilt = self.target_tilt
            x = self.world_position.x
            y = self.world_position.y
            z = self.world_position.z
            current_pose = (x, y, z, self.yaw)
            if current_pose != self._last_set_pose:
                self.set_pose(self.name, x, y, z, self.yaw)
                self._last_set_pose = current_pose
            if self.target_tilt is not None or self.target_pan is not None:
                pitchmsg = Float64()
                pitchmsg.data = self._gimbal_pitch_cmd_rad(self.tilt)
                self.gimbal_pitch_pub.publish(pitchmsg)
                yawmsg = Float64()
                yawmsg.data = math.radians(self.pan)
                self.gimbal_yaw_pub.publish(yawmsg)
            now_msg = self.get_clock().now().to_msg()
            quat = quaternion_from_euler(0.0, 0.0, self.yaw)
            msg = PoseStamped()
            msg.header.stamp = now_msg
            msg.header.frame_id = self.frame_id
            msg.pose.position = self.world_position
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]
            self.pose_pub.publish(msg)
            pose_yaw = yaw_from_quat(quat[0], quat[1], quat[2], quat[3])
            yaw_debug = Vector3Stamped()
            yaw_debug.header.stamp = now_msg
            yaw_debug.header.frame_id = self.frame_id
            yaw_debug.vector.x = float(self.yaw)
            yaw_debug.vector.y = float(pose_yaw)
            yaw_debug.vector.z = float(wrap_pi(pose_yaw - self.yaw))
            if self.yaw_debug_pub is not None:
                self.yaw_debug_pub.publish(yaw_debug)
            target_tilt_deg = self._absolute_camera_tilt_deg(self.target_tilt if self.target_tilt is not None else self.tilt)
            actual_tilt_deg = self._absolute_camera_tilt_deg(self.tilt)
            actual_camera_yaw, _, _, _ = self._camera_pose_components(self.pan, actual_tilt_deg)
            camera_pose = self._camera_pose_msg(now_msg, x, y, z, self.pan, actual_tilt_deg)
            if self.camera_pose_pub is not None:
                self.camera_pose_pub.publish(camera_pose)
            self.camera_actual_pose_pub.publish(camera_pose)
            actual_camera_yaw_msg = Float32()
            actual_camera_yaw_msg.data = float(actual_camera_yaw)
            self.camera_actual_world_yaw_pub.publish(actual_camera_yaw_msg)
            if self.target_camera_world_yaw is not None:
                camera_yaw_error_msg = Float32()
                camera_yaw_error_msg.data = float(wrap_pi(actual_camera_yaw - self.target_camera_world_yaw))
                self.camera_error_world_yaw_pub.publish(camera_yaw_error_msg)
            target_tilt_msg = Float32()
            target_tilt_msg.data = float(target_tilt_deg)
            self.follow_target_tilt_pub.publish(target_tilt_msg)
            actual_tilt_msg = Float32()
            actual_tilt_msg.data = float(actual_tilt_deg)
            self.follow_actual_tilt_pub.publish(actual_tilt_msg)
            error_tilt_msg = Float32()
            error_tilt_msg.data = float(actual_tilt_deg - target_tilt_deg)
            self.follow_error_tilt_pub.publish(error_tilt_msg)
            target_pan_msg = Float32()
            target_pan_msg.data = float(self.target_pan)
            self.follow_target_pan_pub.publish(target_pan_msg)
            actual_pan_msg = Float32()
            actual_pan_msg.data = float(self.pan)
            self.follow_actual_pan_pub.publish(actual_pan_msg)
            error_pan_msg = Float32()
            error_pan_msg.data = float(self.pan - self.target_pan)
            self.follow_error_pan_pub.publish(error_pan_msg)
        except Exception as ex:
            print("Exception timer_callback:", ex, type(ex))


    def set_pose(self, name, x, y, z, yaw):
        try:
            if self._future_pending(self.future1, self.future1_sent_ns):
                return
            self.future1 = None
            self.future1_sent_ns = None
            robot_request = SetEntityPose.Request()
            quat1 = quaternion_from_euler(0.0, 0.0, yaw)
            robot_request.entity.id = 0
            robot_request.entity.name = name
            robot_request.entity.type = robot_request.entity.MODEL
            robot_request.pose.position.x = x
            robot_request.pose.position.y = y
            robot_request.pose.position.z = z
            robot_request.pose.orientation.x = quat1[0]
            robot_request.pose.orientation.y = quat1[1]
            robot_request.pose.orientation.z = quat1[2]
            robot_request.pose.orientation.w = quat1[3]
            ## print(robot_request)
            self.future1 = self.cli.call_async(robot_request)
            self.future1_sent_ns = self.get_clock().now().nanoseconds
            #rclpy.spin_until_future_complete(self, future1)
            #print(future1.result())
        except Exception as ex:
            print("Exception set_pose:", ex, type(ex))

    def _camera_pose_components(self, pan_deg, tilt_deg):
        yaw = self.yaw + math.radians((self.camera_pan_sign * pan_deg) + self.camera_yaw_offset_deg)
        quat = quaternion_from_euler(0.0, -math.radians(tilt_deg), yaw)
        offset_x, offset_y = rotate_body_offset(
            self.camera_x_offset,
            self.camera_y_offset,
            self.yaw,
        )
        return yaw, offset_x, offset_y, quat

    def _camera_pose_msg(self, stamp, x, y, z, pan_deg, tilt_deg):
        yaw, offset_x, offset_y, quat = self._camera_pose_components(pan_deg, tilt_deg)
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x + offset_x)
        msg.pose.position.y = float(y + offset_y)
        msg.pose.position.z = float(z - self.camera_z_offset)
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        return msg

    def _gimbal_pitch_cmd_rad(self, commanded_tilt_deg: float) -> float:
        joint_cmd_rad = -math.radians(commanded_tilt_deg + self.camera_mount_pitch_deg)
        return self._clamp(joint_cmd_rad, self.gimbal_pitch_min, self.gimbal_pitch_max)

    def _absolute_camera_tilt_deg(self, commanded_tilt_deg: float) -> float:
        joint_cmd_rad = self._gimbal_pitch_cmd_rad(commanded_tilt_deg)
        return float(-math.degrees(joint_cmd_rad) - self.camera_mount_pitch_deg)


def main(args=None):
    rclpy.init(args=args)
    node = Simulator()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    print("Spinning simple simulator")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        executor.shutdown()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    

if __name__ == '__main__':
    main()
            

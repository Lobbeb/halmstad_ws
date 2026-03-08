import os
import re
import rclpy
from rclpy.node import Node
import math
from typing import Set

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from std_msgs.msg import UInt8, Float64
from geometry_msgs.msg import PoseArray, PoseStamped, Vector3Stamped, QuaternionStamped, Quaternion, Point
from sensor_msgs.msg import Joy
from tf_transformations import euler_from_quaternion

class Controller(Node):
    def __init__(self):
        super().__init__('example_controller')
        self.declare_parameter("uav_name", "")
        self.declare_parameter("name", "")
        self.declare_parameter("update_rate_hz", 30.0)
        self.declare_parameter("gimbal_pitch_min_rad", -1.5708)
        self.declare_parameter("gimbal_pitch_max_rad", 0.7854)
        self.declare_parameter("waypoint_test_enable", True)
        self.declare_parameter("camera_test_motion_enable", True)
        self.declare_parameter("camera_test_pan_amp_deg", 35.0)
        self.declare_parameter("camera_test_pan_period_s", 8.0)
        self.declare_parameter("camera_test_tilt_center_deg", 10.0)
        self.declare_parameter("camera_test_tilt_amp_deg", 35.0)
        self.declare_parameter("camera_test_tilt_period_s", 5.0)
        self.declare_parameter("camera_test_tilt_phase_deg", 90.0)
        self.uav_name = self._resolve_uav_name()

        self.pan = 0.0
        self.tilt = -45.0

        self.old_error_x = 0.0
        self.old_error_y = 0.0
        self.world_position = None
        self.current_pose = None
        self.speed = 2.0
        self.update_rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.period_time = 1.0 / self.update_rate_hz
        cmd_topic = self._uav_topic("psdk_ros2/flight_control_setpoint_ENUposition_yaw")
        pose_topic = self._uav_topic("pose")
        tilt_topic = self._uav_topic("update_tilt")
        pan_topic = self._uav_topic("update_pan")
        gimbal_pitch_topic = self._gimbal_topic("pitch")
        gimbal_yaw_topic = self._gimbal_topic("yaw")
        self.gimbal_pitch_min = float(self.get_parameter("gimbal_pitch_min_rad").value)
        self.gimbal_pitch_max = float(self.get_parameter("gimbal_pitch_max_rad").value)
        self.waypoint_test_enable = bool(self.get_parameter("waypoint_test_enable").value)
        self.camera_test_motion_enable = bool(self.get_parameter("camera_test_motion_enable").value)
        self.camera_test_pan_amp_deg = float(self.get_parameter("camera_test_pan_amp_deg").value)
        self.camera_test_pan_period_s = float(self.get_parameter("camera_test_pan_period_s").value)
        self.camera_test_tilt_center_deg = float(self.get_parameter("camera_test_tilt_center_deg").value)
        self.camera_test_tilt_amp_deg = float(self.get_parameter("camera_test_tilt_amp_deg").value)
        self.camera_test_tilt_period_s = float(self.get_parameter("camera_test_tilt_period_s").value)
        self.camera_test_tilt_phase_rad = math.radians(
            float(self.get_parameter("camera_test_tilt_phase_deg").value)
        )
        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9
        self.last_camera_log_s = self.start_time_s
        self.ctrl_pos_yaw_pub = self.create_publisher(Joy, cmd_topic, 10)
        self.update_tilt_pub = self.create_publisher(Float64, tilt_topic, 10)
        self.update_pan_pub = self.create_publisher(Float64, pan_topic, 10)
        self.gimbal_pitch_pub = self.create_publisher(Float64, gimbal_pitch_topic, 10)
        self.gimbal_yaw_pub = self.create_publisher(Float64, gimbal_yaw_topic, 10)
        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        self.timer = self.create_timer(self.period_time, self.timer_callback)
        self.goal_positions = [[15.0, 15.0, 4.0],[25.0, 5.0, 4.0]]
        self.state = "idle"
        self.get_logger().info(
            f"Using UAV '{self.uav_name}' topics: cmd={cmd_topic}, pose={pose_topic}, "
            f"gimbal_pitch={gimbal_pitch_topic}"
        )
        self.get_logger().info(
            "Legacy UAV command topic is published as absolute ENU pose: x, y, z, yaw."
        )
        self.get_logger().info(
            f"update_rate_hz={self.update_rate_hz:.1f} "
            f"waypoint_test_enable={self.waypoint_test_enable} "
            f"camera_test_motion_enable={self.camera_test_motion_enable} "
            f"pan_amp={self.camera_test_pan_amp_deg:.1f}deg "
            f"tilt_center={self.camera_test_tilt_center_deg:.1f}deg "
            f"tilt_amp={self.camera_test_tilt_amp_deg:.1f}deg"
        )

    def _uav_topic(self, suffix: str) -> str:
        return f"/{self.uav_name}/{suffix.lstrip('/')}"

    def _gimbal_topic(self, axis: str) -> str:
        return f"/model/{self.uav_name}/joint/{self.uav_name}_gimbal_joint/{axis}/cmd_pos"

    def _clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _update_camera_test_motion(self):
        if not self.camera_test_motion_enable:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        t = now_s - self.start_time_s

        if self.camera_test_pan_period_s > 0.0:
            pan_phase = (2.0 * math.pi * t) / self.camera_test_pan_period_s
            self.pan = self.camera_test_pan_amp_deg * math.sin(pan_phase)
        if self.camera_test_tilt_period_s > 0.0:
            tilt_phase = (2.0 * math.pi * t) / self.camera_test_tilt_period_s
            self.tilt = (
                self.camera_test_tilt_center_deg
                + self.camera_test_tilt_amp_deg * math.sin(tilt_phase + self.camera_test_tilt_phase_rad)
            )

        if now_s - self.last_camera_log_s >= 2.0:
            self.get_logger().info(
                f"camera_test pan={self.pan:.1f}deg tilt={self.tilt:.1f}deg"
            )
            self.last_camera_log_s = now_s

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
                            

    def pose_callback(self, msg):
        self.current_pose = msg
        self.world_position = msg.pose.position
        # print("pose_callback:", self.world_position)

    def check_arrived(self, goal_position):
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z
        cx = self.world_position.x
        cy = self.world_position.y
        cz = self.world_position.z
        dx = cx-x
        dy = cy-y
        dist = math.sqrt(dx*dx+dy*dy)
        if dist < 0.2:
            return True
        else:
            return False

    def get_goal_yaw(self, goal_position):
        print("GET_GOAL_YAW")
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z
        cx = self.world_position.x
        cy = self.world_position.y
        cz = self.world_position.z
        dx = x-cx
        dy = y-cy
        print("DX:", dx)
        print("DY:", dy)
        yaw = math.atan2(dy, dx)
        return yaw

    def timer_callback(self):
        try:
            self._update_camera_test_motion()
            if self.current_pose and not self.waypoint_test_enable:
                self.publish_camera_commands()
                return
            # print("STATE:", self.state)
            if self.current_pose and self.state == "flying":
                if self.check_arrived(self.goal_position):
                    self.state = "idle"
                else:
                    self.control_yaw(self.goal_position, self.goal_yaw)
            if self.current_pose and self.goal_positions and self.state == "idle":
                newpos = self.goal_positions[0]
                self.goal_position = Point()
                self.goal_position.x = newpos[0]
                self.goal_position.y = newpos[1]
                self.goal_position.z = newpos[2]
                self.goal_yaw = self.get_goal_yaw(self.goal_position)
                print(f"FLY TO NEXT WP: {self.goal_position} - {math.degrees(self.goal_yaw)}")
                self.goal_positions = self.goal_positions[1:]
                self.state = "flying"
        except Exception as ex:
            print("Exception timer_callback:", ex, type(ex))
    
    def control_yaw(self, goal_position, goal_yaw):
        # print("control_yaw:", goal_position, goal_yaw)
        roll = 0.0
        pitch = 0.0
        # q = quaternion_from_euler(roll, pitch, self.goal_yaw)
        # quat = Quaternion(q[0], q[1], q[2], q[3])

        # from joypose, why does above not work?
        roll = 0.0
        pitch = 0.0
        yaw = goal_yaw
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        quat = Quaternion()
        quat.w = cr * cp * cy + sr * sp * sy
        quat.x = sr * cp * cy - cr * sp * sy
        quat.y = cr * sp * cy + sr * cp * sy
        quat.z = cr * cp * sy - sr * sp * cy
        self.control_pose(quat, goal_position)

    def control_pose(self, newquat, new_position):
        # print("control_pose:", newquat, new_position)
        (roll, pitch, yaw) = euler_from_quaternion ([newquat.x, newquat.y, newquat.z, newquat.w])
        fac = 0.6
        dfac = 0.0
        # dfac = 0.1
        dx = new_position.x - self.world_position.x
        dy = new_position.y - self.world_position.y
        len = math.sqrt(dx*dx+dy*dy)
        error_x = dx/len*self.speed*self.period_time
        error_y = dy/len*self.speed*self.period_time
        error_dx = (error_x - self.old_error_x)
        error_dy = (error_y - self.old_error_y)
        self.old_error_x = error_x
        self.old_error_y = error_y
            
        x_cmd = error_x*fac + dfac*error_dx
        y_cmd = error_y*fac + dfac*error_dy
        z_cmd = new_position.z
            
        msg = Joy()
        # The simulator adapter interprets this legacy topic as absolute ENU pose.
        msg.axes.append(self.world_position.x + x_cmd)
        msg.axes.append(self.world_position.y + y_cmd)
        msg.axes.append(z_cmd)
        msg.axes.append(yaw)
        self.ctrl_pos_yaw_pub.publish(msg)
        self.publish_camera_commands()

    def publish_camera_commands(self):
        pitchmsg = Float64()
        pitchmsg.data = self._clamp(
            -math.radians(self.tilt), self.gimbal_pitch_min, self.gimbal_pitch_max
        )
        self.gimbal_pitch_pub.publish(pitchmsg)
        yawmsg = Float64()
        yawmsg.data = math.radians(self.pan)
        self.gimbal_yaw_pub.publish(yawmsg)
        panmsg = Float64()
        panmsg.data = self.pan
        self.update_pan_pub.publish(panmsg)
        tiltmsg = Float64()
        tiltmsg.data = self.tilt
        self.update_tilt_pub.publish(tiltmsg)
        
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    print("Spinning simple controller")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    executor.shutdown()    
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
            

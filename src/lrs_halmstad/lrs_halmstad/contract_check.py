from __future__ import annotations

import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from lrs_halmstad.common.world_names import gazebo_world_name


REQUIRED_SERVICES = [
    '/world/{world}/set_pose',
]

REQUIRED_BASE_TOPICS = [
    '/clock',
    '/{ugv}/amcl_pose',
    '/{ugv}/amcl_pose_odom',
    '/{ugv}/cmd_vel',
    '/{ugv}/tf',
    '/{ugv}/tf_static',
]

REQUIRED_UAV_CAMERA_TOPICS = [
    '/{uav}/camera0/image_raw',
    '/{uav}/camera0/camera_info',
]

REQUIRED_UAV_SIM_TOPICS = [
    '/{uav}/pose',
    '/{uav}/psdk_ros2/flight_control_setpoint_ENUposition_yaw',
]

REQUIRED_FOLLOW_TOPICS = [
    '/{uav}/pose_cmd',
    '/{uav}/update_pan',
    '/{uav}/update_tilt',
]

REQUIRED_DETECTION_TOPICS = [
    '/coord/leader_detection',
]

REQUIRED_ESTIMATOR_TOPICS = [
    '/coord/leader_estimate',
    '/coord/leader_estimate_status',
    '/coord/leader_estimate_fault',
]


class ContractChecker(Node):
    def __init__(self):
        super().__init__('contract_checker')
        self._flow_counts: dict[str, int] = {}
        self._flow_subs = []

    def _topic_exists(self, name: str) -> bool:
        topics = self.get_topic_names_and_types()
        return any(t[0] == name for t in topics)

    def _service_exists(self, name: str) -> bool:
        services = self.get_service_names_and_types()
        return any(s[0] == name for s in services)

    def _truthy_env(self, name: str, default: str = "0") -> bool:
        val = os.environ.get(name, default).strip().lower()
        return val in ('1', 'true', 'yes', 'on')

    def _csv_env(self, name: str, default: str) -> list[str]:
        raw = os.environ.get(name, default)
        return [item.strip() for item in raw.split(',') if item.strip()]

    @staticmethod
    def _format_topics(templates: list[str], *, uav: str, ugv: str, world: str) -> list[str]:
        return [topic.format(uav=uav, ugv=ugv, world=world) for topic in templates]

    def _on_flow_msg(self, topic: str) -> None:
        self._flow_counts[topic] = self._flow_counts.get(topic, 0) + 1

    def _setup_flow_subscriptions(self, topics: list[str]) -> None:
        self._flow_counts = {t: 0 for t in topics}
        self._flow_subs = []
        for topic in topics:
            sub = self.create_subscription(
                Odometry,
                topic,
                lambda _msg, t=topic: self._on_flow_msg(t),
                10,
            )
            self._flow_subs.append(sub)

    def _cmd_vel_has_subscriber(self, cmd_topics: list[str]) -> bool:
        return any(len(self.get_subscriptions_info_by_topic(t)) > 0 for t in cmd_topics)

    def check(self, world: str, uav: str, timeout_s: float = 10.0) -> int:
        deadline = time.time() + timeout_s
        missing_topics = set()
        missing_services = set()

        ugv = os.environ.get('UGV_NAMESPACE', 'a201_0000').strip() or 'a201_0000'
        event_topic = os.environ.get('EVENT_TOPIC', '/coord/events').strip() or '/coord/events'
        require_flow = self._truthy_env('REQUIRE_FLOW', '0')
        require_uav_adapter = self._truthy_env('REQUIRE_UAV_ADAPTER', '0')
        require_follow_stack = self._truthy_env('REQUIRE_FOLLOW_STACK', '0')
        require_detection = self._truthy_env('REQUIRE_DETECTION', '0')
        require_estimator = self._truthy_env('REQUIRE_ESTIMATOR', '0')
        cmd_topics = self._csv_env(
            'UGV_CMD_TOPICS',
            f'/{ugv}/cmd_vel,/{ugv}/platform/cmd_vel',
        )
        flow_topics = self._csv_env(
            'REQUIRED_FLOW_TOPICS',
            f'/{ugv}/amcl_pose_odom',
        )

        required_topics = self._format_topics(REQUIRED_BASE_TOPICS, uav=uav, ugv=ugv, world=world)
        required_topics.extend(self._format_topics(REQUIRED_UAV_CAMERA_TOPICS, uav=uav, ugv=ugv, world=world))
        if require_uav_adapter:
            required_topics.extend(self._format_topics(REQUIRED_UAV_SIM_TOPICS, uav=uav, ugv=ugv, world=world))
        if require_follow_stack:
            required_topics.extend(self._format_topics(REQUIRED_FOLLOW_TOPICS, uav=uav, ugv=ugv, world=world))
        if require_detection:
            required_topics.extend(self._format_topics(REQUIRED_DETECTION_TOPICS, uav=uav, ugv=ugv, world=world))
        if require_estimator:
            required_topics.extend(self._format_topics(REQUIRED_ESTIMATOR_TOPICS, uav=uav, ugv=ugv, world=world))
        if event_topic != '/coord/events':
            required_topics.append(event_topic)

        required_services = [s.format(world=gazebo_world_name(world)) for s in REQUIRED_SERVICES]

        if require_flow and flow_topics:
            self._setup_flow_subscriptions(flow_topics)

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            missing_topics = {t for t in required_topics if not self._topic_exists(t)}
            missing_services = {s for s in required_services if not self._service_exists(s)}
            if not missing_topics and not missing_services:
                if require_flow:
                    flow_missing = [t for t in flow_topics if self._flow_counts.get(t, 0) <= 0]
                    if flow_missing or not self._cmd_vel_has_subscriber(cmd_topics):
                        time.sleep(0.20)
                        continue
                    self.get_logger().info(
                        f'Contract OK: world={world} ugv={ugv} uav={uav}; '
                        'topics/services and message flow verified'
                    )
                    return 0

                self.get_logger().info(
                    f'Contract OK: world={world} ugv={ugv} uav={uav}; required topics/services are available'
                )
                return 0

            time.sleep(0.20)

        if missing_topics:
            self.get_logger().error("Missing topics:\n  " + "\n  ".join(sorted(missing_topics)))
        if missing_services:
            self.get_logger().error("Missing services:\n  " + "\n  ".join(sorted(missing_services)))

        if require_flow:
            flow_missing = [t for t in flow_topics if self._flow_counts.get(t, 0) <= 0]
            if flow_missing:
                self.get_logger().error(
                    "No message flow on required topics:\n  " + "\n  ".join(sorted(flow_missing))
                )
            if not self._cmd_vel_has_subscriber(cmd_topics):
                self.get_logger().error(
                    "No subscribers on required cmd_vel topics:\n  " + "\n  ".join(sorted(cmd_topics))
                )

        return 2


def main(argv=None) -> None:
    argv = argv or sys.argv
    if len(argv) < 3:
        print(
            'Usage: ros2 run lrs_halmstad contract_check <world> <uav_name> [timeout_s]\n'
            'Environment overrides:\n'
            '  UGV_NAMESPACE=<name>           default: a201_0000\n'
            '  REQUIRE_UAV_ADAPTER=1          include /<uav>/pose and simulator command topics\n'
            '  REQUIRE_FOLLOW_STACK=1         include /<uav>/pose_cmd and camera control topics\n'
            '  REQUIRE_DETECTION=1            include /coord/leader_detection\n'
            '  REQUIRE_ESTIMATOR=1            include /coord/leader_estimate/*\n'
            '  REQUIRE_FLOW=1                 verify Odometry traffic on REQUIRED_FLOW_TOPICS\n'
            '  REQUIRED_FLOW_TOPICS=<csv>     default: /<ugv>/amcl_pose_odom\n'
            '  UGV_CMD_TOPICS=<csv>           default: /<ugv>/cmd_vel,/<ugv>/platform/cmd_vel'
        )
        raise SystemExit(2)

    world = argv[1]
    uav = argv[2]
    timeout_s = float(argv[3]) if len(argv) >= 4 else 10.0

    rclpy.init()
    node = ContractChecker()
    rc = node.check(world, uav, timeout_s)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)

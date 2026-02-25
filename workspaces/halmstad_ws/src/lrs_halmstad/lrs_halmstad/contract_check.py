from __future__ import annotations

import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


REQUIRED_SERVICES = [
    "/world/{world}/set_pose",
]

REQUIRED_TOPICS = [
    "/clock",
    "/a201_0000/platform/odom",
    "/a201_0000/cmd_vel",
    "/a201_0000/tf",
    "/a201_0000/tf_static",
]


class ContractChecker(Node):
    def __init__(self):
        super().__init__("contract_checker")
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
        return val in ("1", "true", "yes", "on")

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

        event_topic = os.environ.get("EVENT_TOPIC", "/coord/events").strip() or "/coord/events"
        require_flow = self._truthy_env("REQUIRE_FLOW", "0")
        cmd_topics_csv = os.environ.get("UGV_CMD_TOPICS", "/a201_0000/cmd_vel,/a201_0000/platform/cmd_vel")
        flow_topics_csv = os.environ.get("REQUIRED_FLOW_TOPICS", "/a201_0000/platform/odom")
        cmd_topics = [t.strip() for t in cmd_topics_csv.split(",") if t.strip()]
        flow_topics = [t.strip() for t in flow_topics_csv.split(",") if t.strip()]

        required_topics = [t.format(uav=uav) for t in REQUIRED_TOPICS]
        if event_topic != "/coord/events":
            required_topics.append("/coord/events")

        required_services = [s.format(world=world) for s in REQUIRED_SERVICES]

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
                    self.get_logger().info("Contract OK: topics/services and message flow verified")
                    return 0

                self.get_logger().info("Contract OK: required topics/services are available")
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
        print("Usage: ros2 run lrs_halmstad contract_check <world> <uav_name> [timeout_s]")
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

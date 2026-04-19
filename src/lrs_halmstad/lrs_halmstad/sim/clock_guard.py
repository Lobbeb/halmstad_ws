import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockGuard(Node):
    def __init__(self) -> None:
        super().__init__('clock_guard')
        self.declare_parameter('input_topic', '/clock_raw')
        self.declare_parameter('output_topic', '/clock')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        # /clock should always favor the freshest sample over queued history.
        # A deep reliable queue can surface stale ticks under heavy sim load.
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._last_time = None
        self._dropped = 0
        self._last_warn_wall = 0.0

        self._pub = self.create_publisher(Clock, output_topic, output_qos)
        self._sub = self.create_subscription(Clock, input_topic, self._on_clock, input_qos)
        self.get_logger().info(
            f'Publishing monotonic clock {output_topic} from raw source {input_topic}'
        )

    def _on_clock(self, msg: Clock) -> None:
        current = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if self._last_time is not None and current < self._last_time:
            self._dropped += 1
            now_wall = time.monotonic()
            if now_wall - self._last_warn_wall >= 2.0:
                delta = current - self._last_time
                self.get_logger().warning(
                    f'Dropped backward /clock tick delta={delta:.9f}s '
                    f'current={current:.9f}s last={self._last_time:.9f}s '
                    f'dropped={self._dropped}'
                )
                self._last_warn_wall = now_wall
            return

        self._last_time = current
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClockGuard()
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

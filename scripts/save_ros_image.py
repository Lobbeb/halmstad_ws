#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def _to_cv_image(msg: Image) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    encoding = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding in ("mono8", "8uc1"):
        return data.reshape((height, width))
    if encoding == "rgb8":
        image = data.reshape((height, width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return data.reshape((height, width, 3))
    if encoding == "rgba8":
        image = data.reshape((height, width, 4))
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)
    if encoding == "bgra8":
        return data.reshape((height, width, 4))

    raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")


class ImageSaver(Node):
    def __init__(self, topic: str, output_path: Path) -> None:
        super().__init__("baylands_topdown_image_saver")
        self._output_path = output_path
        self._saved = False
        self.create_subscription(Image, topic, self._callback, qos_profile_sensor_data)

    @property
    def saved(self) -> bool:
        return self._saved

    def _callback(self, msg: Image) -> None:
        if self._saved:
            return

        cv_image = _to_cv_image(msg)
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self._output_path), cv_image):
            raise RuntimeError(f"Failed to write image to {self._output_path}")

        self.get_logger().info(
            f"Saved {msg.width}x{msg.height} frame from '{msg.header.frame_id or 'unknown_frame'}' to {self._output_path}"
        )
        self._saved = True


def main() -> int:
    parser = argparse.ArgumentParser(description="Save the next ROS image message to disk.")
    parser.add_argument("--topic", required=True, help="ROS image topic to capture.")
    parser.add_argument("--output", required=True, help="Output image path.")
    parser.add_argument("--timeout", type=float, default=20.0, help="Timeout in seconds.")
    args = parser.parse_args()

    output_path = Path(args.output).expanduser().resolve()

    rclpy.init()
    node = ImageSaver(args.topic, output_path)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)

    try:
        while rclpy.ok() and not node.saved:
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.get_clock().now().nanoseconds >= deadline:
                raise TimeoutError(
                    f"Timed out after {args.timeout:.1f}s waiting for an image on {args.topic}"
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

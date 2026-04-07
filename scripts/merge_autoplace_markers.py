#!/usr/bin/env python3

import argparse
import math
import re
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers


CLIENT_ID = "halmstad_ws_merge_autoplace"


def normalize_namespace(namespace: str) -> str:
    namespace = namespace.strip()
    if not namespace:
        return "/merge_maps_tool"
    if not namespace.startswith("/"):
        namespace = f"/{namespace}"
    return namespace.rstrip("/")


def parse_known_markers(text: str) -> set[str]:
    known = set()
    if not text:
        return known
    for piece in text.replace(",", "\n").splitlines():
        piece = piece.strip()
        if piece:
            known.add(piece)
    return known


def load_metadata(path: str) -> dict[str, str]:
    data: dict[str, str] = {}
    for raw_line in Path(path).read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key] = value
    return data


def metadata_has_pose(metadata: dict[str, str]) -> bool:
    required = (
        "spawn_x",
        "spawn_y",
        "spawn_yaw",
        "map_pose_x",
        "map_pose_y",
        "map_pose_yaw",
    )
    return all(metadata.get(key) not in (None, "") for key in required)


def pose_to_matrix(x: float, y: float, yaw: float) -> list[list[float]]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [
        [c, -s, x],
        [s, c, y],
        [0.0, 0.0, 1.0],
    ]


def invert_matrix(matrix: list[list[float]]) -> list[list[float]]:
    c = matrix[0][0]
    s = matrix[1][0]
    x = matrix[0][2]
    y = matrix[1][2]
    return [
        [c, s, -(c * x + s * y)],
        [-s, c, (s * x - c * y)],
        [0.0, 0.0, 1.0],
    ]


def multiply_matrix(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [
            a[0][0] * b[0][0] + a[0][1] * b[1][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2],
        ],
        [
            a[1][0] * b[0][0] + a[1][1] * b[1][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2],
        ],
        [0.0, 0.0, 1.0],
    ]


def matrix_to_pose(matrix: list[list[float]]) -> tuple[float, float, float]:
    return matrix[0][2], matrix[1][2], math.atan2(matrix[1][0], matrix[0][0])


def quaternion_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def world_map_transform(metadata: dict[str, str]) -> list[list[float]]:
    if not metadata_has_pose(metadata):
        raise ValueError("metadata is missing spawn_* or map_pose_* fields")

    world_robot = pose_to_matrix(
        float(metadata["spawn_x"]),
        float(metadata["spawn_y"]),
        float(metadata["spawn_yaw"]),
    )
    map_robot = pose_to_matrix(
        float(metadata["map_pose_x"]),
        float(metadata["map_pose_y"]),
        float(metadata["map_pose_yaw"]),
    )
    return multiply_matrix(world_robot, invert_matrix(map_robot))


def relative_pose_from_metadata(
    reference_metadata_path: str,
    target_metadata_path: str,
) -> tuple[float, float, float]:
    reference = load_metadata(reference_metadata_path)
    target = load_metadata(target_metadata_path)

    reference_world_map = world_map_transform(reference)
    target_world_map = world_map_transform(target)
    reference_to_target = multiply_matrix(
        invert_matrix(reference_world_map),
        target_world_map,
    )
    return matrix_to_pose(reference_to_target)


class MergeMarkerClient(Node):
    def __init__(self, namespace: str) -> None:
        super().__init__("merge_autoplace_markers")
        self.namespace = normalize_namespace(namespace)
        self.marker_client = self.create_client(
            GetInteractiveMarkers,
            f"{self.namespace}/get_interactive_markers",
        )
        self.feedback_pub = self.create_publisher(
            InteractiveMarkerFeedback,
            f"{self.namespace}/feedback",
            10,
        )

    def wait_for_service(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.marker_client.wait_for_service(timeout_sec=0.25):
                return True
        return False

    def get_markers(self, timeout_s: float = 2.0):
        request = GetInteractiveMarkers.Request()
        future = self.marker_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise RuntimeError("failed to fetch interactive markers")
        response = future.result()
        return list(response.markers)

    def wait_for_new_marker(self, known_markers: set[str], timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            markers = self.get_markers()
            new_markers = [marker for marker in markers if marker.name not in known_markers]
            if len(new_markers) == 1:
                return new_markers[0]
            if len(new_markers) > 1:
                names = ", ".join(sorted(marker.name for marker in new_markers))
                raise RuntimeError(f"multiple new markers appeared: {names}")
            time.sleep(0.1)
        raise RuntimeError("timed out waiting for a new merge marker")

    def get_marker_by_name(self, marker_name: str, timeout_s: float = 2.0):
        markers = self.get_markers(timeout_s=timeout_s)
        for marker in markers:
            if marker.name == marker_name:
                return marker
        raise RuntimeError(f"marker '{marker_name}' was not found")

    def set_marker_pose(self, marker, x: float, y: float, yaw: float, attempts: int = 3) -> None:
        feedback = InteractiveMarkerFeedback()
        feedback.header = marker.header
        feedback.client_id = CLIENT_ID
        feedback.marker_name = marker.name
        feedback.control_name = ""
        feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        feedback.pose = Pose()
        feedback.pose.position.x = x
        feedback.pose.position.y = y
        feedback.pose.position.z = marker.pose.position.z
        feedback.pose.orientation.x = 0.0
        feedback.pose.orientation.y = 0.0
        feedback.pose.orientation.z = math.sin(yaw / 2.0)
        feedback.pose.orientation.w = math.cos(yaw / 2.0)

        for _ in range(attempts):
            self.feedback_pub.publish(feedback)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)


def marker_sort_key(marker) -> tuple[str, int, str]:
    match = re.search(r"^(.*?)(\d+)$", marker.name)
    if match:
        prefix, numeric = match.groups()
        return prefix, int(numeric), marker.name
    return marker.name, -1, marker.name


def command_list(args: argparse.Namespace) -> int:
    rclpy.init(args=None)
    node = MergeMarkerClient(args.namespace)
    try:
        if not node.wait_for_service(args.timeout):
            print("interactive marker service is not available", file=sys.stderr)
            return 1
        markers = node.get_markers(timeout_s=args.timeout)
        for marker in sorted(markers, key=marker_sort_key):
            print(marker.name)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_dump(args: argparse.Namespace) -> int:
    rclpy.init(args=None)
    node = MergeMarkerClient(args.namespace)
    try:
        if not node.wait_for_service(args.timeout):
            print("interactive marker service is not available", file=sys.stderr)
            return 1
        markers = node.get_markers(timeout_s=args.timeout)
        for marker in sorted(markers, key=marker_sort_key):
            yaw = quaternion_to_yaw(marker.pose.orientation.z, marker.pose.orientation.w)
            print(
                f"{marker.name}\t{marker.pose.position.x:.9f}\t"
                f"{marker.pose.position.y:.9f}\t{yaw:.9f}"
            )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_set_new_from_metadata(args: argparse.Namespace) -> int:
    known_markers = parse_known_markers(args.known_markers)
    try:
        target_x, target_y, target_yaw = relative_pose_from_metadata(
            args.reference_metadata,
            args.target_metadata,
        )
    except Exception as exc:
        print(f"failed to compute metadata placement: {exc}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = MergeMarkerClient(args.namespace)
    try:
        if not node.wait_for_service(args.timeout):
            print("interactive marker service is not available", file=sys.stderr)
            return 1

        marker = node.wait_for_new_marker(known_markers, args.timeout)
        node.set_marker_pose(marker, target_x, target_y, target_yaw)
        print(marker.name)
        print(f"x={target_x:.6f}")
        print(f"y={target_y:.6f}")
        print(f"yaw={target_yaw:.6f}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_set_new_pose(args: argparse.Namespace) -> int:
    known_markers = parse_known_markers(args.known_markers)

    rclpy.init(args=None)
    node = MergeMarkerClient(args.namespace)
    try:
        if not node.wait_for_service(args.timeout):
            print("interactive marker service is not available", file=sys.stderr)
            return 1

        marker = node.wait_for_new_marker(known_markers, args.timeout)
        node.set_marker_pose(marker, args.x, args.y, args.yaw)
        print(marker.name)
        print(f"x={args.x:.6f}")
        print(f"y={args.y:.6f}")
        print(f"yaw={args.yaw:.6f}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_wait_new(args: argparse.Namespace) -> int:
    known_markers = parse_known_markers(args.known_markers)

    rclpy.init(args=None)
    node = MergeMarkerClient(args.namespace)
    try:
        if not node.wait_for_service(args.timeout):
            print("interactive marker service is not available", file=sys.stderr)
            return 1

        marker = node.wait_for_new_marker(known_markers, args.timeout)
        print(marker.name)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_set_by_name(args: argparse.Namespace) -> int:
    rclpy.init(args=None)
    node = MergeMarkerClient(args.namespace)
    try:
        if not node.wait_for_service(args.timeout):
            print("interactive marker service is not available", file=sys.stderr)
            return 1

        marker = node.get_marker_by_name(args.marker_name, timeout_s=args.timeout)
        node.set_marker_pose(marker, args.x, args.y, args.yaw)
        print(marker.name)
        print(f"x={args.x:.6f}")
        print(f"y={args.y:.6f}")
        print(f"yaw={args.yaw:.6f}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Helpers for Slam Toolbox merge marker placement")
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="List current interactive marker names")
    list_parser.add_argument("--namespace", default="/merge_maps_tool")
    list_parser.add_argument("--timeout", type=float, default=5.0)
    list_parser.set_defaults(func=command_list)

    dump_parser = subparsers.add_parser(
        "dump",
        help="Dump interactive marker poses as: name<TAB>x<TAB>y<TAB>yaw",
    )
    dump_parser.add_argument("--namespace", default="/merge_maps_tool")
    dump_parser.add_argument("--timeout", type=float, default=5.0)
    dump_parser.set_defaults(func=command_dump)

    place_parser = subparsers.add_parser(
        "set-new-from-metadata",
        help="Wait for one newly-added marker and place it from saved metadata",
    )
    place_parser.add_argument("--namespace", default="/merge_maps_tool")
    place_parser.add_argument("--timeout", type=float, default=8.0)
    place_parser.add_argument("--known-markers", default="")
    place_parser.add_argument("--reference-metadata", required=True)
    place_parser.add_argument("--target-metadata", required=True)
    place_parser.set_defaults(func=command_set_new_from_metadata)

    pose_parser = subparsers.add_parser(
        "set-new-pose",
        help="Wait for one newly-added marker and place it at the given pose",
    )
    pose_parser.add_argument("--namespace", default="/merge_maps_tool")
    pose_parser.add_argument("--timeout", type=float, default=8.0)
    pose_parser.add_argument("--known-markers", default="")
    pose_parser.add_argument("--x", type=float, required=True)
    pose_parser.add_argument("--y", type=float, required=True)
    pose_parser.add_argument("--yaw", type=float, required=True)
    pose_parser.set_defaults(func=command_set_new_pose)

    wait_parser = subparsers.add_parser(
        "wait-new",
        help="Wait for one newly-added marker and print its name",
    )
    wait_parser.add_argument("--namespace", default="/merge_maps_tool")
    wait_parser.add_argument("--timeout", type=float, default=8.0)
    wait_parser.add_argument("--known-markers", default="")
    wait_parser.set_defaults(func=command_wait_new)

    set_name_parser = subparsers.add_parser(
        "set-by-name",
        help="Set the pose of an existing marker by name",
    )
    set_name_parser.add_argument("--namespace", default="/merge_maps_tool")
    set_name_parser.add_argument("--timeout", type=float, default=8.0)
    set_name_parser.add_argument("--marker-name", required=True)
    set_name_parser.add_argument("--x", type=float, required=True)
    set_name_parser.add_argument("--y", type=float, required=True)
    set_name_parser.add_argument("--yaw", type=float, required=True)
    set_name_parser.set_defaults(func=command_set_by_name)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())

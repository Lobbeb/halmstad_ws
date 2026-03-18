#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
import rclpy.clock
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - runtime dependency
    cv2 = None


@dataclass
class Pose3D:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    frame_id: str


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


class SimDatasetCapture(Node):
    def __init__(self) -> None:
        super().__init__("sim_dataset_capture")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("camera_pose_topic", "")
        self.declare_parameter("target_pose_topic", "/a201_0000/amcl_pose_odom")
        self.declare_parameter("output_dir", "datasets/warehouse_auto")
        self.declare_parameter("dataset_name", "warehouse_auto")
        self.declare_parameter("class_id", 0)
        self.declare_parameter("class_name", "ugv")
        self.declare_parameter("camera_pose_timeout_s", 10.0)
        self.declare_parameter("image_timeout_s", 10.0)
        self.declare_parameter("val_every_n", 10)
        self.declare_parameter("save_metadata", True)
        self.declare_parameter("save_overlay", True)
        self.declare_parameter("save_negative_examples", True)
        self.declare_parameter("min_bbox_pixels", 0.0, dyn_num)
        self.declare_parameter("min_bbox_area_px", 0.0, dyn_num)
        self.declare_parameter("target_length_m", 1.0, dyn_num)
        self.declare_parameter("target_width_m", 0.7, dyn_num)
        self.declare_parameter("target_height_m", 0.65, dyn_num)
        self.declare_parameter("target_base_z_m", 0.0, dyn_num)

        self.uav_name = str(self.get_parameter("uav_name").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value) or f"/{self.uav_name}/camera0/image_raw"
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value) or f"/{self.uav_name}/camera0/camera_info"
        self.camera_pose_topic = (
            str(self.get_parameter("camera_pose_topic").value)
            or f"/{self.uav_name}/camera0/actual/center_pose"
        )
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.output_dir = os.path.abspath(os.path.expanduser(str(self.get_parameter("output_dir").value)))
        self.dataset_name = str(self.get_parameter("dataset_name").value).strip() or "warehouse_auto"
        self.class_id = int(self.get_parameter("class_id").value)
        self.class_name = str(self.get_parameter("class_name").value).strip() or "ugv"
        self.camera_pose_timeout_s = float(self.get_parameter("camera_pose_timeout_s").value)
        self.image_timeout_s = float(self.get_parameter("image_timeout_s").value)
        self.val_every_n = int(self.get_parameter("val_every_n").value)
        self.save_metadata = bool(self.get_parameter("save_metadata").value)
        self.save_overlay = bool(self.get_parameter("save_overlay").value)
        self.save_negative_examples = bool(self.get_parameter("save_negative_examples").value)
        self.min_bbox_pixels = float(self.get_parameter("min_bbox_pixels").value)
        self.min_bbox_area_px = float(self.get_parameter("min_bbox_area_px").value)
        self.target_length_m = float(self.get_parameter("target_length_m").value)
        self.target_width_m = float(self.get_parameter("target_width_m").value)
        self.target_height_m = float(self.get_parameter("target_height_m").value)
        self.target_base_z_m = float(self.get_parameter("target_base_z_m").value)

        if cv2 is None:
            raise RuntimeError("OpenCV (cv2) is required for sim_dataset_capture")
        if self.class_id < 0:
            raise ValueError("class_id must be >= 0")
        if self.target_length_m <= 0.0 or self.target_width_m <= 0.0 or self.target_height_m <= 0.0:
            raise ValueError("target dimensions must be > 0")

        self.camera_model: Optional[CameraModel] = None
        self.last_image: Optional[Image] = None
        self.last_image_recv: Optional[float] = None  # wall time
        self.last_camera_pose: Optional[Pose3D] = None
        self.last_camera_pose_stamp: Optional[Time] = None
        self.last_camera_pose_recv: Optional[float] = None  # wall time
        self.last_target_pose: Optional[Pose3D] = None
        self.last_target_pose_stamp: Optional[Time] = None
        self.capture_index = 0
        self.last_wait_reason: str = "init"

        self._ensure_dataset_layout()
        self._write_dataset_yaml()

        self.create_subscription(Image, self.camera_topic, self.on_image, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.create_subscription(PoseStamped, self.camera_pose_topic, self.on_camera_pose, 10)
        self.create_subscription(Odometry, self.target_pose_topic, self.on_target_pose, 10)
        self.create_timer(2.0, self._status_tick, clock=rclpy.clock.Clock())

        self.get_logger().info(
            f"[sim_dataset_capture] Started: image={self.camera_topic}, info={self.camera_info_topic}, "
            f"camera_pose={self.camera_pose_topic}, target_pose={self.target_pose_topic}, "
            f"output_dir={self.output_dir}, trigger=amcl_pose_odom, class={self.class_name}:{self.class_id}"
        )

    def _ensure_dataset_layout(self) -> None:
        for subset in ("train", "val"):
            os.makedirs(os.path.join(self.output_dir, "images", subset), exist_ok=True)
            os.makedirs(os.path.join(self.output_dir, "labels", subset), exist_ok=True)
            if self.save_metadata:
                os.makedirs(os.path.join(self.output_dir, "metadata", subset), exist_ok=True)
            if self.save_overlay:
                os.makedirs(os.path.join(self.output_dir, "overlay", subset), exist_ok=True)

    def _write_dataset_yaml(self) -> None:
        dataset_yaml = os.path.join(self.output_dir, "dataset.yaml")
        content = (
            f"path: {self.output_dir}\n"
            "train: images/train\n"
            "val: images/val\n"
            f"names:\n  {self.class_id}: {self.class_name}\n"
        )
        with open(dataset_yaml, "w", encoding="ascii") as fh:
            fh.write(content)

    def on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) < 9:
            return
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        if fx <= 0.0 or fy <= 0.0 or msg.width <= 0 or msg.height <= 0:
            return
        self.camera_model = CameraModel(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            width=int(msg.width),
            height=int(msg.height),
        )

    def on_camera_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.last_camera_pose = Pose3D(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            qx=float(q.x),
            qy=float(q.y),
            qz=float(q.z),
            qw=float(q.w),
            frame_id=str(msg.header.frame_id),
        )
        try:
            self.last_camera_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_camera_pose_stamp = self.get_clock().now()
        self.last_camera_pose_recv = time.monotonic()

    def on_image(self, msg: Image) -> None:
        self.last_image = msg
        self.last_image_recv = time.monotonic()

    def on_target_pose(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.last_target_pose = Pose3D(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            qx=float(q.x),
            qy=float(q.y),
            qz=float(q.z),
            qw=float(q.w),
            frame_id=str(msg.header.frame_id),
        )
        try:
            self.last_target_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_target_pose_stamp = self.get_clock().now()
        self._try_capture()

    def _status_tick(self) -> None:
        if self.last_wait_reason != "capturing":
            self.get_logger().info(f"[sim_dataset_capture] Waiting: {self.last_wait_reason}")

    def _try_capture(self) -> None:
        now = self.get_clock().now()
        if not self._ready(now):
            return

        msg = self.last_image
        image = self._image_to_bgr(msg)
        if image is None:
            return

        bbox, projected_points = self._compute_bbox()
        if bbox is None:
            self.last_wait_reason = f"no_bbox pts={len(projected_points)}"
            if not self.save_negative_examples:
                return

        subset = "val" if self.val_every_n > 0 and ((self.capture_index + 1) % self.val_every_n == 0) else "train"
        stem = self._frame_stem(msg)

        image_path = os.path.join(self.output_dir, "images", subset, f"{stem}.jpg")
        label_path = os.path.join(self.output_dir, "labels", subset, f"{stem}.txt")
        overlay_path = os.path.join(self.output_dir, "overlay", subset, f"{stem}.jpg")
        metadata_path = os.path.join(self.output_dir, "metadata", subset, f"{stem}.json")

        if not cv2.imwrite(image_path, image):
            self.get_logger().warn(f"[sim_dataset_capture] Failed to save image: {image_path}")
            return

        with open(label_path, "w", encoding="ascii") as fh:
            if bbox is not None:
                xc, yc, bw, bh = bbox
                fh.write(f"{self.class_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")

        if self.save_overlay:
            overlay = self._draw_overlay(image, bbox, projected_points)
            cv2.imwrite(overlay_path, overlay)

        if self.save_metadata:
            metadata = self._metadata_dict(msg, subset, image_path, label_path, bbox, projected_points)
            with open(metadata_path, "w", encoding="utf-8") as fh:
                json.dump(metadata, fh, indent=2, sort_keys=True)

        self.capture_index += 1
        self.last_wait_reason = "capturing"
        self.get_logger().info(
            f"[sim_dataset_capture] Saved frame {self.capture_index}: subset={subset} "
            f"bbox={'yes' if bbox is not None else 'no'} file={os.path.basename(image_path)}"
        )

    def _ready(self, now: Time) -> bool:
        if self.camera_model is None:
            self.last_wait_reason = "waiting_for_camera_info"
            return False
        if self.last_camera_pose is None or self.last_camera_pose_recv is None:
            self.last_wait_reason = "waiting_for_camera_pose"
            return False
        if self.last_target_pose is None:
            self.last_wait_reason = "waiting_for_target_pose"
            return False
        if self.last_image is None or self.last_image_recv is None:
            self.last_wait_reason = "waiting_for_image"
            return False
        wall_now = time.monotonic()
        cam_age = wall_now - self.last_camera_pose_recv
        img_age = wall_now - self.last_image_recv
        if cam_age > self.camera_pose_timeout_s:
            self.last_wait_reason = f"stale_camera_pose age={cam_age:.2f}s"
            return False
        if img_age > self.image_timeout_s:
            self.last_wait_reason = f"stale_image age={img_age:.2f}s"
            return False
        self.last_wait_reason = "ready"
        return True


    def _frame_stem(self, msg: Image) -> str:
        try:
            stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            stamp = self.get_clock().now()
        return f"frame_{self.capture_index:06d}_{stamp.nanoseconds}"

    def _image_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            h = int(msg.height)
            w = int(msg.width)
            if h <= 0 or w <= 0:
                return None
            if msg.encoding == "bgr8":
                return np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3)).copy()
            if msg.encoding == "rgb8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            if msg.encoding == "bgra8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                return arr[:, :, :3].copy()
            if msg.encoding == "rgba8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                return cv2.cvtColor(arr[:, :, :3], cv2.COLOR_RGB2BGR)
            if msg.encoding == "mono8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
                return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        except Exception as exc:
            self.get_logger().warn(f"[sim_dataset_capture] Failed image decode: {exc}")
        return None

    def _cuboid_points_world(self) -> np.ndarray:
        tp = self.last_target_pose
        if tp is None:
            return np.zeros((0, 3), dtype=np.float64)
        half_l = 0.5 * self.target_length_m
        half_w = 0.5 * self.target_width_m
        h = self.target_height_m
        yaw = yaw_from_quat(tp.qx, tp.qy, tp.qz, tp.qw)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        def rotate_local(px: float, py: float, pz: float) -> Tuple[float, float, float]:
            wx = tp.x + cy * px - sy * py
            wy = tp.y + sy * px + cy * py
            wz = self.target_base_z_m + pz
            return (wx, wy, wz)

        xs = (-half_l, 0.0, half_l)
        ys = (-half_w, 0.0, half_w)
        zs = (0.0, 0.5 * h, h)
        points: List[Tuple[float, float, float]] = []
        for px in xs:
            for py in ys:
                points.append(rotate_local(px, py, 0.0))
                points.append(rotate_local(px, py, h))
        for px in (-half_l, half_l):
            for pz in zs:
                points.append(rotate_local(px, 0.0, pz))
        for py in (-half_w, half_w):
            for pz in zs:
                points.append(rotate_local(0.0, py, pz))
        for pz in zs:
            points.append(rotate_local(0.0, 0.0, pz))
        return np.array(points, dtype=np.float64)

    def _world_to_camera_projection(self, points_world: np.ndarray) -> np.ndarray:
        if self.last_camera_pose is None:
            return np.zeros((0, 3), dtype=np.float64)
        rot_world_body = quat_to_rot(
            self.last_camera_pose.qx,
            self.last_camera_pose.qy,
            self.last_camera_pose.qz,
            self.last_camera_pose.qw,
        )
        cam_pos = np.array(
            [self.last_camera_pose.x, self.last_camera_pose.y, self.last_camera_pose.z],
            dtype=np.float64,
        )
        relative = points_world - cam_pos
        points_body = relative @ rot_world_body
        points_camera = np.empty_like(points_body)
        points_camera[:, 0] = points_body[:, 0]
        points_camera[:, 1] = -points_body[:, 1]
        points_camera[:, 2] = -points_body[:, 2]
        return points_camera

    def _compute_bbox(self) -> Tuple[Optional[Tuple[float, float, float, float]], List[Tuple[float, float]]]:
        if self.camera_model is None:
            return None, []
        points_world = self._cuboid_points_world()
        points_camera = self._world_to_camera_projection(points_world)
        if points_camera.size == 0:
            return None, []

        projected: List[Tuple[float, float]] = []
        for x_forward, y_right, z_down in points_camera:
            if x_forward <= 0:
                continue
            u = self.camera_model.cx + self.camera_model.fx * (y_right / x_forward)
            v = self.camera_model.cy + self.camera_model.fy * (z_down / x_forward)
            if not math.isfinite(u) or not math.isfinite(v):
                continue
            projected.append((u, v))

        if not projected:
            return None, []

        xs = [p[0] for p in projected]
        ys = [p[1] for p in projected]
        x1 = max(0.0, min(xs))
        y1 = max(0.0, min(ys))
        x2 = min(float(self.camera_model.width - 1), max(xs))
        y2 = min(float(self.camera_model.height - 1), max(ys))

        bw = x2 - x1
        bh = y2 - y1

        xc = (x1 + x2) / (2.0 * self.camera_model.width)
        yc = (y1 + y2) / (2.0 * self.camera_model.height)
        bw_n = bw / self.camera_model.width
        bh_n = bh / self.camera_model.height
        return (xc, yc, bw_n, bh_n), projected

    def _draw_overlay(
        self,
        image: np.ndarray,
        bbox: Optional[Tuple[float, float, float, float]],
        projected_points: Sequence[Tuple[float, float]],
    ) -> np.ndarray:
        out = image.copy()
        for u, v in projected_points:
            cv2.circle(out, (int(round(u)), int(round(v))), 2, (0, 255, 255), -1)
        if bbox is not None and self.camera_model is not None:
            xc, yc, bw, bh = bbox
            x1 = int(round((xc - 0.5 * bw) * self.camera_model.width))
            y1 = int(round((yc - 0.5 * bh) * self.camera_model.height))
            x2 = int(round((xc + 0.5 * bw) * self.camera_model.width))
            y2 = int(round((yc + 0.5 * bh) * self.camera_model.height))
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 0), 2)
            cv2.putText(
                out,
                f"{self.class_name}:{self.class_id}",
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 220, 0),
                1,
                cv2.LINE_AA,
            )
        return out

    def _metadata_dict(
        self,
        msg: Image,
        subset: str,
        image_path: str,
        label_path: str,
        bbox: Optional[Tuple[float, float, float, float]],
        projected_points: Sequence[Tuple[float, float]],
    ) -> dict:
        stamp_ns = 0
        try:
            stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        except Exception:
            stamp_ns = self.get_clock().now().nanoseconds
        return {
            "bbox_yolo": None if bbox is None else {
                "class_id": self.class_id,
                "class_name": self.class_name,
                "x_center": bbox[0],
                "y_center": bbox[1],
                "width": bbox[2],
                "height": bbox[3],
            },
            "camera_model": None if self.camera_model is None else {
                "cx": self.camera_model.cx,
                "cy": self.camera_model.cy,
                "fx": self.camera_model.fx,
                "fy": self.camera_model.fy,
                "height": self.camera_model.height,
                "width": self.camera_model.width,
            },
            "camera_pose": None if self.last_camera_pose is None else {
                "frame_id": self.last_camera_pose.frame_id,
                "orientation": {
                    "w": self.last_camera_pose.qw,
                    "x": self.last_camera_pose.qx,
                    "y": self.last_camera_pose.qy,
                    "z": self.last_camera_pose.qz,
                },
                "position": {
                    "x": self.last_camera_pose.x,
                    "y": self.last_camera_pose.y,
                    "z": self.last_camera_pose.z,
                },
            },
            "dataset_name": self.dataset_name,
            "image_path": image_path,
            "label_path": label_path,
            "projected_points": [[float(u), float(v)] for u, v in projected_points],
            "stamp_ns": int(stamp_ns),
            "subset": subset,
            "target_dimensions_m": {
                "height": self.target_height_m,
                "length": self.target_length_m,
                "width": self.target_width_m,
            },
            "target_pose": None if self.last_target_pose is None else {
                "frame_id": self.last_target_pose.frame_id,
                "orientation": {
                    "w": self.last_target_pose.qw,
                    "x": self.last_target_pose.qx,
                    "y": self.last_target_pose.qy,
                    "z": self.last_target_pose.qz,
                },
                "position": {
                    "x": self.last_target_pose.x,
                    "y": self.last_target_pose.y,
                    "z": self.last_target_pose.z,
                },
            },
        }


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimDatasetCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

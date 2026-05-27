from __future__ import annotations

import math
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from fs_msgs.msg import Track
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from fsds_autonomy.constants import (
    DEFAULT_CAMERA_FOV_DEG,
    DEFAULT_CAMERA_OFFSETS,
    DEFAULT_CONE_HEIGHT_M,
    DEFAULT_CONE_WIDTH_M,
    FS_MSG_COLOR_TO_AUTONOMY,
    ConeColor,
)
from fsds_autonomy.geometry import Pose2, transform_global_to_local, yaw_from_quaternion
from fsds_autonomy.label_quality import refine_projected_label_with_color


class DatasetBuilder(Node):
    def __init__(self) -> None:
        super().__init__("fsds_dataset_builder")
        self.declare_parameter("enabled", False)
        self.declare_parameter("dataset_dir", "datasets/fsds_cones")
        self.declare_parameter("save_every_n_frames", 5)
        self.declare_parameter("camera_topic", "/fsds/cam1/image_color")
        self.declare_parameter("camera_name", "cam1")
        self.declare_parameter("min_box_px", 6)
        self.declare_parameter("camera_fov_deg", DEFAULT_CAMERA_FOV_DEG)
        self.declare_parameter("own_vehicle_mask_top_fraction", 0.62)
        self.declare_parameter("own_vehicle_mask_left_fraction", 0.12)
        self.declare_parameter("own_vehicle_mask_right_fraction", 0.88)
        self.declare_parameter("validate_projected_labels", True)
        self.declare_parameter("snap_labels_to_image_color", True)
        self.declare_parameter("label_search_scale", 2.6)
        self.declare_parameter("label_color_min_pixels", 5)
        self.declare_parameter("label_color_min_ratio", 0.012)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.dataset_dir = Path(str(self.get_parameter("dataset_dir").value)).expanduser()
        self.save_every = max(1, int(self.get_parameter("save_every_n_frames").value))
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.min_box_px = int(self.get_parameter("min_box_px").value)
        self.camera_fov_deg = float(self.get_parameter("camera_fov_deg").value)
        self.own_vehicle_mask_top = float(self.get_parameter("own_vehicle_mask_top_fraction").value)
        self.own_vehicle_mask_left = float(self.get_parameter("own_vehicle_mask_left_fraction").value)
        self.own_vehicle_mask_right = float(self.get_parameter("own_vehicle_mask_right_fraction").value)
        self.validate_projected_labels = bool(self.get_parameter("validate_projected_labels").value)
        self.snap_labels_to_image_color = bool(self.get_parameter("snap_labels_to_image_color").value)
        self.label_search_scale = float(self.get_parameter("label_search_scale").value)
        self.label_color_min_pixels = int(self.get_parameter("label_color_min_pixels").value)
        self.label_color_min_ratio = float(self.get_parameter("label_color_min_ratio").value)
        self.frame_count = 0
        self.track = None
        self.pose = None
        self.bridge = CvBridge()

        topic = str(self.get_parameter("camera_topic").value)
        self.create_subscription(Image, topic, self.on_image, 5)
        track_qos = QoSProfile(depth=1)
        track_qos.reliability = ReliabilityPolicy.RELIABLE
        track_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Track, "/fsds/testing_only/track", self.on_track, track_qos)
        self.create_subscription(Odometry, "/fsds/testing_only/odom", self.on_odom, 20)

        if self.enabled:
            (self.dataset_dir / "images").mkdir(parents=True, exist_ok=True)
            (self.dataset_dir / "labels").mkdir(parents=True, exist_ok=True)
            self.write_dataset_yaml()
            self.get_logger().info(f"Dataset recording enabled: {self.dataset_dir}")
        else:
            self.get_logger().info("Dataset recorder disabled; set enabled:=true to save synthetic labels")

    def on_track(self, msg: Track) -> None:
        self.track = msg

    def on_odom(self, msg: Odometry) -> None:
        self.pose = Pose2(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def write_dataset_yaml(self) -> None:
        names = ["yellow_cone", "blue_cone", "orange_cone", "large_orange_cone", "unknown_cone"]
        text = "\n".join(
            [
                f"path: {self.dataset_dir}",
                "train: images",
                "val: images",
                "names:",
                *[f"  {i}: {name}" for i, name in enumerate(names)],
                "",
            ]
        )
        (self.dataset_dir / "fsds_cones.yaml").write_text(text, encoding="utf-8")

    def project_cone(self, cone, image_width: int, image_height: int) -> tuple[int, float, float, float, float] | None:
        if self.pose is None:
            return None
        local_x, local_y = transform_global_to_local(cone.location.x, cone.location.y, self.pose)
        offset = DEFAULT_CAMERA_OFFSETS.get(self.camera_name, DEFAULT_CAMERA_OFFSETS["cam1"])
        cam_x = local_x - offset.x
        cam_y = local_y - offset.y
        if cam_x <= 0.25:
            return None

        f = image_width / (2.0 * math.tan(math.radians(self.camera_fov_deg) * 0.5))
        u = image_width * 0.5 - f * cam_y / cam_x
        cone_center_z = DEFAULT_CONE_HEIGHT_M * 0.5
        v = image_height * 0.5 - f * (cone_center_z - offset.z) / cam_x
        box_h = f * DEFAULT_CONE_HEIGHT_M / cam_x
        box_w = f * DEFAULT_CONE_WIDTH_M / cam_x
        if box_h < self.min_box_px or u < -box_w or u > image_width + box_w:
            return None
        if v < -box_h or v > image_height + box_h:
            return None
        if (
            v > image_height * self.own_vehicle_mask_top
            and image_width * self.own_vehicle_mask_left <= u <= image_width * self.own_vehicle_mask_right
        ):
            return None
        color = FS_MSG_COLOR_TO_AUTONOMY.get(int(cone.color), ConeColor.UNKNOWN)
        x_center = max(0.0, min(1.0, u / image_width))
        y_center = max(0.0, min(1.0, v / image_height))
        width = max(0.0, min(1.0, box_w / image_width))
        height = max(0.0, min(1.0, box_h / image_height))
        return color, x_center, y_center, width, height

    def on_image(self, msg: Image) -> None:
        if not self.enabled or self.track is None or self.pose is None:
            return
        self.frame_count += 1
        if self.frame_count % self.save_every != 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Could not convert image for dataset: {exc}")
            return

        height, width = frame.shape[:2]
        labels = []
        for cone in self.track.track:
            projected = self.project_cone(cone, width, height)
            if projected is None:
                continue
            if self.validate_projected_labels:
                refined = refine_projected_label_with_color(
                    frame,
                    projected,
                    search_scale=self.label_search_scale,
                    min_color_pixels=self.label_color_min_pixels,
                    min_color_ratio=self.label_color_min_ratio,
                    snap_to_color=self.snap_labels_to_image_color,
                )
                if refined is None:
                    continue
                projected = refined[0]
            labels.append(projected)

        if not labels:
            return

        stamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}_{self.frame_count:06d}"
        image_path = self.dataset_dir / "images" / f"{self.camera_name}_{stamp}.jpg"
        label_path = self.dataset_dir / "labels" / f"{self.camera_name}_{stamp}.txt"
        cv2.imwrite(str(image_path), frame)
        label_text = "\n".join(
            f"{int(label)} {x:.6f} {y:.6f} {w:.6f} {h:.6f}"
            for label, x, y, w, h in labels
        )
        label_path.write_text(label_text + "\n", encoding="utf-8")


def main() -> None:
    rclpy.init()
    rclpy.spin(DatasetBuilder())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

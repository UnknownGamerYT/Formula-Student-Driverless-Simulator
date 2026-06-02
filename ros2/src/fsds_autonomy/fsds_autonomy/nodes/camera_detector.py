from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import rclpy
from cv_bridge import CvBridge
from fsds_autonomy_msgs.msg import ConeArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from fsds_autonomy.constants import (
    COLOR_TO_CONE_CLASS,
    DEFAULT_CAMERA_FOV_DEG,
    DEFAULT_CAMERA_OFFSETS,
    ConeColor,
    STEREO_CAMERA_SOURCE,
    is_start_finish_color,
)
from fsds_autonomy.geometry import distance_xy, nearest_point_index, point, pose2_from_pose_stamped, transform_global_to_local
from fsds_autonomy.ml import OptionalYoloDetector, color_segment_cones, non_max_suppression
from fsds_autonomy.perception import CameraDetection
from fsds_autonomy.ros_utils import cone_msg, make_header
from fsds_autonomy.visualization import bgr_for_cone, project_car_point_to_camera


@dataclass
class CameraObservation:
    detections: list[CameraDetection]
    stamp_sec: float


class CameraDetector(Node):
    def __init__(self) -> None:
        super().__init__("fsds_camera_detector")
        self.declare_parameter("cam1_topic", "/fsds/cam1/image_color")
        self.declare_parameter("cam2_topic", "/fsds/cam2/image_color")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence", 0.45)
        self.declare_parameter("yolo_imgsz", 960)
        self.declare_parameter("yolo_device", "auto")
        self.declare_parameter("yolo_warmup", True)
        self.declare_parameter("use_color_fallback", True)
        self.declare_parameter("color_fallback_confidence", 0.28)
        self.declare_parameter("color_fallback_min_area_px", 120)
        self.declare_parameter("color_fallback_max_detections", 20)
        self.declare_parameter("color_fallback_roi_top_fraction", 0.34)
        self.declare_parameter("color_fallback_roi_bottom_fraction", 0.84)
        self.declare_parameter("color_fallback_own_vehicle_mask_top_fraction", 0.62)
        self.declare_parameter("color_fallback_own_vehicle_mask_left_fraction", 0.12)
        self.declare_parameter("color_fallback_own_vehicle_mask_right_fraction", 0.88)
        self.declare_parameter("duplicate_bbox_iou_threshold", 0.32)
        self.declare_parameter("duplicate_bbox_overlap_threshold", 0.62)
        self.declare_parameter("duplicate_local_distance_m", 0.55)
        self.declare_parameter("duplicate_cone_distance_m", 0.45)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("publish_run_image", True)
        self.declare_parameter("overlay_camera_name", "cam1")
        self.declare_parameter("camera_fov_deg", DEFAULT_CAMERA_FOV_DEG)
        self.declare_parameter("track_half_width_m", 1.75)
        self.declare_parameter("overlay_min_boundary_confidence", 0.30)
        self.declare_parameter("overlay_path_min_quality", 0.35)
        self.declare_parameter("overlay_max_range_m", 22.0)
        self.declare_parameter("overlay_path_max_error_m", 5.0)
        self.declare_parameter("overlay_path_max_step_m", 8.0)
        self.declare_parameter("stereo_enabled", True)
        self.declare_parameter("stereo_max_time_delta_sec", 0.30)
        self.declare_parameter("stereo_min_confidence", 0.50)
        self.declare_parameter("stereo_min_disparity_rad", 0.006)
        self.declare_parameter("stereo_good_disparity_rad", 0.025)
        self.declare_parameter("stereo_max_range_m", 25.0)
        self.declare_parameter("stereo_max_abs_y_m", 10.0)
        self.declare_parameter("stereo_max_match_cost", 1.25)

        self.bridge = CvBridge()
        self.detector = OptionalYoloDetector(
            str(self.get_parameter("model_path").value),
            float(self.get_parameter("confidence").value),
            int(self.get_parameter("yolo_imgsz").value),
            str(self.get_parameter("yolo_device").value),
            bool(self.get_parameter("yolo_warmup").value),
        )
        self.use_color_fallback = bool(self.get_parameter("use_color_fallback").value)
        self.color_fallback_confidence = float(self.get_parameter("color_fallback_confidence").value)
        self.color_fallback_min_area = int(self.get_parameter("color_fallback_min_area_px").value)
        self.color_fallback_max_detections = int(self.get_parameter("color_fallback_max_detections").value)
        self.color_fallback_roi_top = float(self.get_parameter("color_fallback_roi_top_fraction").value)
        self.color_fallback_roi_bottom = float(self.get_parameter("color_fallback_roi_bottom_fraction").value)
        self.color_fallback_own_vehicle_mask_top = float(
            self.get_parameter("color_fallback_own_vehicle_mask_top_fraction").value
        )
        self.color_fallback_own_vehicle_mask_left = float(
            self.get_parameter("color_fallback_own_vehicle_mask_left_fraction").value
        )
        self.color_fallback_own_vehicle_mask_right = float(
            self.get_parameter("color_fallback_own_vehicle_mask_right_fraction").value
        )
        self.duplicate_bbox_iou_threshold = float(self.get_parameter("duplicate_bbox_iou_threshold").value)
        self.duplicate_bbox_overlap_threshold = float(self.get_parameter("duplicate_bbox_overlap_threshold").value)
        self.duplicate_local_distance = float(self.get_parameter("duplicate_local_distance_m").value)
        self.duplicate_cone_distance = float(self.get_parameter("duplicate_cone_distance_m").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.publish_run_image = bool(self.get_parameter("publish_run_image").value)
        self.overlay_camera_name = str(self.get_parameter("overlay_camera_name").value)
        self.camera_fov_deg = float(self.get_parameter("camera_fov_deg").value)
        self.track_half_width = float(self.get_parameter("track_half_width_m").value)
        self.overlay_min_boundary_confidence = float(self.get_parameter("overlay_min_boundary_confidence").value)
        self.overlay_path_min_quality = float(self.get_parameter("overlay_path_min_quality").value)
        self.overlay_max_range = float(self.get_parameter("overlay_max_range_m").value)
        self.overlay_path_max_error = float(self.get_parameter("overlay_path_max_error_m").value)
        self.overlay_path_max_step = float(self.get_parameter("overlay_path_max_step_m").value)
        self.stereo_enabled = bool(self.get_parameter("stereo_enabled").value)
        self.stereo_max_time_delta = float(self.get_parameter("stereo_max_time_delta_sec").value)
        self.stereo_min_confidence = float(self.get_parameter("stereo_min_confidence").value)
        self.stereo_min_disparity = float(self.get_parameter("stereo_min_disparity_rad").value)
        self.stereo_good_disparity = float(self.get_parameter("stereo_good_disparity_rad").value)
        self.stereo_max_range = float(self.get_parameter("stereo_max_range_m").value)
        self.stereo_max_abs_y = float(self.get_parameter("stereo_max_abs_y_m").value)
        self.stereo_max_match_cost = float(self.get_parameter("stereo_max_match_cost").value)
        self.latest_pose = None
        self.latest_track = TrackMap()
        self.latest_reference_track = TrackMap()
        self.latest_live_track = TrackMap()
        self.latest_race_state = RaceState()
        self.latest_speed = 0.0
        self.latest_path_offset = 0.0
        self.latest_camera_detections: dict[str, CameraObservation] = {}
        self.latest_stereo_cones: list[tuple[float, float, int, float, str]] = []
        self.latest_stereo_used_indices: dict[str, set[int]] = {"cam1": set(), "cam2": set()}

        self.cones_pub = self.create_publisher(ConeArray, "/autonomy/camera_cones", 10)
        self.debug_pub = self.create_publisher(Image, "/autonomy/camera_debug", 1)
        self.run_pub = self.create_publisher(Image, "/autonomy/camera_run", 1)
        self.overlay_pubs = {
            "cam1": self.create_publisher(Image, "/autonomy/viz/camera/cam1_overlay", 1),
            "cam2": self.create_publisher(Image, "/autonomy/viz/camera/cam2_overlay", 1),
        }
        self.debug_overlay_pubs = {
            "cam1": self.create_publisher(Image, "/autonomy/viz/camera/cam1_debug", 1),
            "cam2": self.create_publisher(Image, "/autonomy/viz/camera/cam2_debug", 1),
        }
        self.run_overlay_pubs = {
            "cam1": self.create_publisher(Image, "/autonomy/viz/camera/cam1_run", 1),
            "cam2": self.create_publisher(Image, "/autonomy/viz/camera/cam2_run", 1),
        }

        for cam_name, topic_param in (("cam1", "cam1_topic"), ("cam2", "cam2_topic")):
            topic = str(self.get_parameter(topic_param).value)
            self.create_subscription(
                Image,
                topic,
                lambda msg, name=cam_name: self.on_image(msg, name),
                5,
            )
        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)
        self.create_subscription(TrackMap, "/autonomy/racing_line", self.on_track, 10)
        self.create_subscription(TrackMap, "/autonomy/reference_racing_line", self.on_reference_track, 10)
        self.create_subscription(TrackMap, "/autonomy/live_racing_line", self.on_live_track, 10)
        self.create_subscription(RaceState, "/autonomy/race_state", self.on_race_state, 10)
        self.create_subscription(Float32, "/autonomy/speed", self.on_speed, 10)
        self.create_subscription(Float32, "/autonomy/path_offset", self.on_path_offset, 10)

        if self.detector.available:
            self.get_logger().info(
                f"{self.detector.display_name} loaded for camera cones "
                f"(device={self.detector.device}, imgsz={self.detector.imgsz})"
            )
            if self.detector.warmup_error:
                self.get_logger().warn(f"YOLO warmup failed, first frame may be slower: {self.detector.warmup_error}")
        elif self.detector.loaded:
            names_preview = ", ".join(str(name) for _, name in list(self.detector.names.items())[:8])
            self.get_logger().warn(
                "YOLO weights loaded, but they do not expose cone classes. "
                f"Detected class names start with: {names_preview}. "
                "Camera cone detection is using low-trust HSV fallback until a fine-tuned cone model is provided."
            )
        else:
            detail = f": {self.detector.load_error}" if self.detector.load_error else ""
            self.get_logger().warn(f"YOLO detector unavailable{detail}; using HSV cone color fallback")

    def on_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def on_track(self, msg: TrackMap) -> None:
        self.latest_track = msg
        self.latest_live_track = msg

    def on_reference_track(self, msg: TrackMap) -> None:
        self.latest_reference_track = msg

    def on_live_track(self, msg: TrackMap) -> None:
        self.latest_live_track = msg
        self.latest_track = msg

    def on_race_state(self, msg: RaceState) -> None:
        self.latest_race_state = msg

    def on_speed(self, msg: Float32) -> None:
        self.latest_speed = float(msg.data)

    def on_path_offset(self, msg: Float32) -> None:
        self.latest_path_offset = float(msg.data)

    def on_image(self, msg: Image, camera_name: str) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Could not convert image from {camera_name}: {exc}")
            return

        backend = self.detector.display_name if self.detector.loaded else "HSV fallback"
        if not self.detector.available and not self.use_color_fallback:
            backend = "No detector"
        yolo_detections = self.detector.detect(frame, camera_name) if self.detector.available else []
        fallback_detections = []
        if self.use_color_fallback:
            fallback_detections = color_segment_cones(
                frame,
                camera_name,
                confidence=self.color_fallback_confidence,
                min_area=self.color_fallback_min_area,
                max_detections=self.color_fallback_max_detections,
                roi_top_fraction=self.color_fallback_roi_top,
                roi_bottom_fraction=self.color_fallback_roi_bottom,
                own_vehicle_mask_top_fraction=self.color_fallback_own_vehicle_mask_top,
                own_vehicle_mask_left_fraction=self.color_fallback_own_vehicle_mask_left,
                own_vehicle_mask_right_fraction=self.color_fallback_own_vehicle_mask_right,
                camera_fov_deg=self.camera_fov_deg,
            )
        detections = non_max_suppression(
            sorted(yolo_detections + fallback_detections, key=lambda item: item.confidence, reverse=True),
            max(30, len(yolo_detections) + self.color_fallback_max_detections),
            iou_threshold=self.duplicate_bbox_iou_threshold,
            overlap_threshold=self.duplicate_bbox_overlap_threshold,
            duplicate_local_distance_m=self.duplicate_local_distance,
        )
        if self.use_color_fallback:
            if self.detector.available:
                backend = f"{self.detector.display_name} + HSV backup"
            elif self.detector.loaded:
                backend = f"{self.detector.display_name} + HSV fallback low-trust"
            else:
                backend = "HSV fallback low-trust"

        self.latest_camera_detections[camera_name] = CameraObservation(detections, self.stamp_sec(msg))
        self.publish_camera_cones()

        if self.publish_debug_image:
            debug = self.draw_overlay(frame, detections, camera_name, backend)
            overlay_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            overlay_msg.header = msg.header
            if camera_name in self.debug_overlay_pubs:
                self.debug_overlay_pubs[camera_name].publish(overlay_msg)
            if camera_name in self.overlay_pubs:
                self.overlay_pubs[camera_name].publish(overlay_msg)
            if camera_name == self.overlay_camera_name:
                self.debug_pub.publish(overlay_msg)
        if self.publish_run_image:
            run = self.draw_run_overlay(frame, detections, camera_name, backend)
            run_msg = self.bridge.cv2_to_imgmsg(run, encoding="bgr8")
            run_msg.header = msg.header
            if camera_name in self.run_overlay_pubs:
                self.run_overlay_pubs[camera_name].publish(run_msg)
            if camera_name == self.overlay_camera_name:
                self.run_pub.publish(run_msg)

    def stamp_sec(self, msg: Image) -> float:
        _ = msg
        return self.get_clock().now().nanoseconds * 1e-9

    def colors_compatible(self, left: int, right: int) -> bool:
        if left == ConeColor.UNKNOWN or right == ConeColor.UNKNOWN:
            return True
        if is_start_finish_color(left) and is_start_finish_color(right):
            return True
        return int(left) == int(right)

    def merged_color(self, left: CameraDetection, right: CameraDetection) -> int:
        if left.color == ConeColor.UNKNOWN:
            return int(right.color)
        if right.color == ConeColor.UNKNOWN:
            return int(left.color)
        if left.color == right.color:
            return int(left.color)
        return int(left.color if left.confidence >= right.confidence else right.color)

    def triangulate_pair(
        self,
        cam_a: str,
        det_a: CameraDetection,
        cam_b: str,
        det_b: CameraDetection,
    ) -> tuple[float, float, float, float] | None:
        offset_a = DEFAULT_CAMERA_OFFSETS.get(cam_a, DEFAULT_CAMERA_OFFSETS["cam1"])
        offset_b = DEFAULT_CAMERA_OFFSETS.get(cam_b, DEFAULT_CAMERA_OFFSETS["cam2"])
        angle_a = offset_a.yaw + det_a.bearing
        angle_b = offset_b.yaw + det_b.bearing
        dir_ax, dir_ay = math.cos(angle_a), math.sin(angle_a)
        dir_bx, dir_by = math.cos(angle_b), math.sin(angle_b)

        denominator = dir_ax * dir_by - dir_ay * dir_bx
        disparity = abs(math.atan2(math.sin(angle_a - angle_b), math.cos(angle_a - angle_b)))
        if abs(denominator) < self.stereo_min_disparity or disparity < self.stereo_min_disparity:
            return None

        delta_x = offset_b.x - offset_a.x
        delta_y = offset_b.y - offset_a.y
        range_a = (delta_x * dir_by - delta_y * dir_bx) / denominator
        range_b = (delta_x * dir_ay - delta_y * dir_ax) / denominator
        if range_a <= 0.35 or range_b <= 0.35:
            return None

        point_ax = offset_a.x + range_a * dir_ax
        point_ay = offset_a.y + range_a * dir_ay
        point_bx = offset_b.x + range_b * dir_bx
        point_by = offset_b.y + range_b * dir_by
        x = 0.5 * (point_ax + point_bx)
        y = 0.5 * (point_ay + point_by)
        range_m = math.hypot(x, y)
        if x <= 0.2 or range_m > self.stereo_max_range or abs(y) > self.stereo_max_abs_y:
            return None

        monocular_range = 0.5 * (det_a.range + det_b.range)
        if abs(det_a.range - det_b.range) > max(2.5, 0.45 * monocular_range):
            return None
        range_cost = abs(monocular_range - range_m) / max(1.0, range_m)
        confidence_cost = 0.15 * abs(det_a.confidence - det_b.confidence)
        cost = range_cost + confidence_cost
        if cost > self.stereo_max_match_cost:
            return None
        return x, y, disparity, cost

    def stereo_triangulated_cones(self) -> tuple[list[tuple[float, float, int, float, str]], dict[str, set[int]]]:
        used = {"cam1": set(), "cam2": set()}
        if not self.stereo_enabled:
            return [], used
        cam1 = self.latest_camera_detections.get("cam1")
        cam2 = self.latest_camera_detections.get("cam2")
        if cam1 is None or cam2 is None:
            return [], used
        if abs(cam1.stamp_sec - cam2.stamp_sec) > self.stereo_max_time_delta:
            return [], used

        candidates = []
        for index_1, det_1 in enumerate(cam1.detections):
            if det_1.confidence < self.stereo_min_confidence:
                continue
            for index_2, det_2 in enumerate(cam2.detections):
                if det_2.confidence < self.stereo_min_confidence:
                    continue
                if not self.colors_compatible(det_1.color, det_2.color):
                    continue
                result = self.triangulate_pair("cam1", det_1, "cam2", det_2)
                if result is None:
                    continue
                x, y, disparity, cost = result
                stability = max(0.25, min(1.0, disparity / max(1e-3, self.stereo_good_disparity)))
                confidence = min(det_1.confidence, det_2.confidence) * stability * max(0.4, 1.0 - 0.35 * cost)
                color = self.merged_color(det_1, det_2)
                candidates.append((cost, -confidence, index_1, index_2, x, y, color, confidence))

        cones: list[tuple[float, float, int, float, str]] = []
        for _, _, index_1, index_2, x, y, color, confidence in sorted(candidates):
            if index_1 in used["cam1"] or index_2 in used["cam2"]:
                continue
            used["cam1"].add(index_1)
            used["cam2"].add(index_2)
            cones.append((x, y, color, confidence, f"{STEREO_CAMERA_SOURCE}(cam1+cam2)"))
        return cones, used

    def publish_camera_cones(self) -> None:
        header = make_header(self, "fsds/FSCar")
        array = ConeArray()
        array.header = header

        stereo_cones, used = self.stereo_triangulated_cones()
        self.latest_stereo_used_indices = {name: set(indices) for name, indices in used.items()}
        self.latest_stereo_cones = stereo_cones
        for x, y, color, confidence, source in stereo_cones:
            self.add_unique_cone(
                array.cones,
                cone_msg(
                    header,
                    x,
                    y,
                    0.0,
                    color,
                    confidence,
                    source,
                ),
            )

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        max_age = max(0.6, self.stereo_max_time_delta * 2.0)
        for camera_name, observation in self.latest_camera_detections.items():
            if now_sec - observation.stamp_sec > max_age:
                continue
            for index, detection in enumerate(observation.detections):
                if index in used.get(camera_name, set()):
                    continue
                x = detection.range * math.cos(detection.bearing)
                y = detection.range * math.sin(detection.bearing)
                self.add_unique_cone(
                    array.cones,
                    cone_msg(
                        header,
                        x,
                        y,
                        0.0,
                        detection.color,
                        detection.confidence,
                        detection.source,
                    ),
                )

        self.cones_pub.publish(array)

    def add_unique_cone(self, cones, candidate) -> None:
        best_index = -1
        best_distance = self.duplicate_cone_distance
        for index, existing in enumerate(cones):
            distance = math.hypot(
                existing.position.x - candidate.position.x,
                existing.position.y - candidate.position.y,
            )
            if distance <= best_distance:
                best_index = index
                best_distance = distance
        if best_index < 0:
            cones.append(candidate)
            return
        existing = cones[best_index]
        existing_score = float(existing.confidence) + (0.10 if STEREO_CAMERA_SOURCE in existing.source else 0.0)
        candidate_score = float(candidate.confidence) + (0.10 if STEREO_CAMERA_SOURCE in candidate.source else 0.0)
        if candidate_score > existing_score:
            cones[best_index] = candidate

    def draw_overlay(self, frame, detections: list[CameraDetection], camera_name: str, backend: str):
        debug = frame.copy()
        self.draw_path_overlay(debug, detections, camera_name)
        self.draw_stereo_markers(debug, camera_name)
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            color = bgr_for_cone(detection.color)
            label = COLOR_TO_CONE_CLASS.get(detection.color, "unknown_cone")
            bearing_deg = math.degrees(detection.bearing)
            text = f"{label} conf={detection.confidence:.2f} {detection.range:.1f}m {bearing_deg:+.0f}deg"
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                debug,
                text,
                (x1 + 4, max(16, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
        self.draw_status_panel(debug, camera_name, backend, len(detections))
        cv2.putText(
            debug,
            "blue/yellow=cone boundaries  cyan=live line  magenta=reference line",
            (10, debug.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return debug

    def draw_run_overlay(self, frame, detections: list[CameraDetection], camera_name: str, backend: str):
        run = frame.copy()
        self.draw_path_overlay(
            run,
            detections,
            camera_name,
            draw_detection_boundaries=False,
            draw_local_preview=False,
        )
        self.draw_stereo_markers(run, camera_name)
        used_count = self.draw_used_detection_boxes(run, detections, camera_name)
        self.draw_status_panel(run, camera_name, f"RUN {backend}", used_count, count_label="used")
        return run

    def draw_used_detection_boxes(self, frame, detections: list[CameraDetection], camera_name: str) -> int:
        used_indices = self.latest_stereo_used_indices.get(camera_name, set())
        drawn = 0
        for index, detection in enumerate(detections):
            if index not in used_indices:
                continue
            x1, y1, x2, y2 = detection.bbox
            color = bgr_for_cone(detection.color)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = COLOR_TO_CONE_CLASS.get(detection.color, "unknown_cone")
            cv2.putText(
                frame,
                f"{label} used {detection.confidence:.2f}",
                (x1 + 4, max(16, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
            drawn += 1
        return drawn

    def draw_status_panel(
        self,
        frame,
        camera_name: str,
        backend: str,
        detection_count: int,
        count_label: str = "detections",
    ) -> None:
        state = self.latest_race_state
        lines = [
            f"{camera_name} | {backend} | {count_label}={detection_count} stereo={len(self.latest_stereo_cones)}",
            (
                f"{state.behavior_state or 'Preview'} | target={state.target_speed:.1f}m/s "
                f"speed={self.latest_speed:.1f}m/s offset={self.latest_path_offset:+.2f}m"
            ),
            f"map={self.latest_track.quality:.2f} mode={state.mode or 'manual/preview'} status={state.status or 'waiting'}",
        ]
        width = min(frame.shape[1] - 20, max(420, int(frame.shape[1] * 0.58)))
        height = 22 + 21 * len(lines)
        overlay = frame.copy()
        cv2.rectangle(overlay, (8, 8), (8 + width, 8 + height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0.0, frame)
        for index, line in enumerate(lines):
            cv2.putText(
                frame,
                line[:96],
                (18, 32 + index * 21),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def draw_path_overlay(
        self,
        frame,
        detections: list[CameraDetection],
        camera_name: str,
        draw_detection_boundaries: bool = True,
        draw_local_preview: bool = True,
    ) -> None:
        height, width = frame.shape[:2]
        offset = DEFAULT_CAMERA_OFFSETS.get(camera_name, DEFAULT_CAMERA_OFFSETS["cam1"])

        pose = pose2_from_pose_stamped(self.latest_pose) if self.latest_pose is not None else None
        image_boundary_count = 0
        if draw_detection_boundaries:
            image_boundary_count = self.draw_image_detection_boundaries(frame, detections, width, height)
        if draw_detection_boundaries and image_boundary_count < 4:
            boundary_groups = self.live_boundary_points(detections)
            self.draw_projected_local_boundaries(frame, boundary_groups, width, height, offset)

        live_track = self.latest_live_track if self.latest_live_track.racing_line else self.latest_track
        reference_track = self.latest_reference_track if self.latest_reference_track.racing_line else TrackMap()

        if pose is None:
            if draw_local_preview:
                self.draw_projected_local_preview(frame, detections, camera_name, (255, 255, 0), 2)
            return

        if reference_track.quality >= self.overlay_path_min_quality:
            self.draw_projected_track_boundaries(frame, reference_track, pose, width, height, offset, thickness=1)
        if live_track.quality >= self.overlay_path_min_quality:
            self.draw_projected_track_boundaries(frame, live_track, pose, width, height, offset, thickness=2)

        current_path = []
        if live_track.quality >= self.overlay_path_min_quality:
            current_path = self.upcoming_path_points(
                live_track.racing_line or live_track.centerline,
                pose,
                closed_loop=bool(live_track.closed_loop),
            )
        if current_path:
            self.draw_projected_path(
                frame,
                current_path,
                pose,
                width,
                height,
                offset,
                local_y_offset=self.latest_path_offset,
                color=(255, 255, 0),
                thickness=3,
            )
        elif draw_local_preview:
            self.draw_projected_local_preview(frame, detections, camera_name, (255, 255, 0), 2)

        if reference_track.quality >= self.overlay_path_min_quality:
            racing_path = self.upcoming_path_points(
                reference_track.racing_line,
                pose,
                closed_loop=bool(reference_track.closed_loop),
            )
            self.draw_projected_path(
                frame,
                racing_path,
                pose,
                width,
                height,
                offset,
                local_y_offset=0.0,
                color=(255, 0, 255),
                thickness=2,
            )

    def draw_projected_track_boundaries(
        self,
        frame,
        track: TrackMap,
        pose,
        width: int,
        height: int,
        offset,
        thickness: int,
    ) -> None:
        for boundary_color, boundary_line in (
            (ConeColor.BLUE, track.blue_boundary_line),
            (ConeColor.YELLOW, track.yellow_boundary_line),
        ):
            boundary_path = self.upcoming_path_points(
                boundary_line,
                pose,
                closed_loop=bool(track.closed_loop),
            )
            if not boundary_path:
                continue
            self.draw_projected_path(
                frame,
                boundary_path,
                pose,
                width,
                height,
                offset,
                local_y_offset=0.0,
                color=bgr_for_cone(boundary_color),
                thickness=thickness,
            )

    def upcoming_path_points(self, global_points, pose, closed_loop: bool = False) -> list:
        points = list(global_points)
        if len(points) < 2:
            return []

        nearest = nearest_point_index(points, pose.x, pose.y)
        if nearest < 0:
            return []
        pose_point = point(pose.x, pose.y, 0.0)
        if distance_xy(points[nearest], pose_point) > self.overlay_path_max_error:
            return []

        upcoming = [points[nearest]]
        previous = points[nearest]
        travelled = 0.0
        index = nearest
        for _ in range(1, len(points)):
            index += 1
            if index >= len(points):
                if not closed_loop:
                    break
                index = 0
            step = distance_xy(previous, points[index])
            if step > self.overlay_path_max_step:
                break
            travelled += step
            if travelled > self.overlay_max_range:
                break
            upcoming.append(points[index])
            previous = points[index]

        return upcoming if len(upcoming) >= 2 else []

    def draw_projected_path(
        self,
        frame,
        global_points,
        pose,
        width,
        height,
        offset,
        local_y_offset: float,
        color,
        thickness: int,
    ) -> None:
        projected = []
        for path_point in global_points:
            local_x, local_y = transform_global_to_local(path_point.x, path_point.y, pose)
            if local_x <= 1.2 or local_x > self.overlay_max_range or abs(local_y + local_y_offset) > 12.0:
                projected.append(None)
                continue
            pixel = project_car_point_to_camera(
                local_x,
                local_y + local_y_offset,
                0.0,
                width,
                height,
                offset.x,
                offset.y,
                offset.z,
                self.camera_fov_deg,
            )
            projected.append((pixel, local_x, local_y + local_y_offset))

        for start, end in zip(projected, projected[1:]):
            if start is None or end is None:
                continue
            start_pixel, start_x, start_y = start
            end_pixel, end_x, end_y = end
            if start_pixel is None or end_pixel is None:
                continue
            if math.hypot(end_x - start_x, end_y - start_y) > 7.0:
                continue
            if abs(start_pixel[0] - end_pixel[0]) > width * 0.65 or abs(start_pixel[1] - end_pixel[1]) > height * 0.65:
                continue
            cv2.line(frame, start_pixel, end_pixel, color, thickness, cv2.LINE_AA)

    def map_boundary_points(self, pose) -> dict[int, list]:
        if pose is None or self.latest_track.quality < self.overlay_path_min_quality:
            return {}
        local_cones = []
        for cone in self.latest_track.cones:
            local_x, local_y = transform_global_to_local(cone.position.x, cone.position.y, pose)
            if 0.5 <= local_x <= self.overlay_max_range and abs(local_y) <= 12.0:
                local_cones.append((local_x, local_y, int(cone.color), float(cone.confidence)))
        return self.boundary_groups_from_local_cones(local_cones)

    def live_boundary_points(self, detections: list[CameraDetection]) -> dict[int, list]:
        local_cones = []
        if self.latest_stereo_cones:
            local_cones.extend(
                (x, y, color, confidence)
                for x, y, color, confidence, _ in self.latest_stereo_cones
                if confidence >= self.overlay_min_boundary_confidence
            )
        if sum(1 for _, _, color, _ in local_cones if color in (ConeColor.BLUE, ConeColor.YELLOW)) < 2:
            for detection in detections:
                if detection.confidence < self.overlay_min_boundary_confidence:
                    continue
                if detection.color not in (ConeColor.BLUE, ConeColor.YELLOW, ConeColor.ORANGE, ConeColor.LARGE_ORANGE):
                    continue
                x = detection.range * math.cos(detection.bearing)
                y = detection.range * math.sin(detection.bearing)
                if 0.8 <= x <= self.overlay_max_range and abs(y) <= 10.0:
                    local_cones.append((x, y, int(detection.color), float(detection.confidence)))
        return self.boundary_groups_from_local_cones(local_cones)

    def boundary_groups_from_local_cones(self, local_cones) -> dict[int, list]:
        groups = {ConeColor.BLUE: [], ConeColor.YELLOW: []}
        orange_cones = []
        for x, y, color, confidence in local_cones:
            if color == ConeColor.BLUE:
                groups[ConeColor.BLUE].append((x, y, confidence))
            elif color == ConeColor.YELLOW:
                groups[ConeColor.YELLOW].append((x, y, confidence))
            elif is_start_finish_color(color):
                orange_cones.append((x, y, confidence))

        for x, y, confidence in orange_cones:
            if y >= 0.0:
                groups[ConeColor.BLUE].append((x, y, confidence))
            else:
                groups[ConeColor.YELLOW].append((x, y, confidence))

        return {
            ConeColor.BLUE: self.ordered_local_boundary(groups[ConeColor.BLUE]),
            ConeColor.YELLOW: self.ordered_local_boundary(groups[ConeColor.YELLOW]),
        }

    def ordered_local_boundary(self, values) -> list:
        if not values:
            return []
        best_by_bin = {}
        for x, y, confidence in values:
            bin_index = int(x / 1.6)
            best = best_by_bin.get(bin_index)
            if best is None or confidence > best[2]:
                best_by_bin[bin_index] = (x, y, confidence)
        ordered = []
        for x, y, _ in sorted(best_by_bin.values(), key=lambda item: item[0]):
            if ordered:
                previous = ordered[-1]
                if math.hypot(x - previous.x, y - previous.y) > 7.0:
                    continue
                if abs(y - previous.y) > 4.5:
                    continue
            ordered.append(point(x, y, 0.0))
        return ordered

    def draw_projected_local_boundaries(self, frame, boundary_groups: dict[int, list], width: int, height: int, offset) -> None:
        for boundary_color, points in boundary_groups.items():
            if len(points) < 2:
                continue
            projected = []
            for local_point in points:
                pixel = project_car_point_to_camera(
                    local_point.x,
                    local_point.y,
                    local_point.z,
                    width,
                    height,
                    offset.x,
                    offset.y,
                    offset.z,
                    self.camera_fov_deg,
                )
                projected.append((pixel, local_point))
            draw_color = bgr_for_cone(boundary_color)
            for start, end in zip(projected, projected[1:]):
                start_pixel, start_point = start
                end_pixel, end_point = end
                if start_pixel is None or end_pixel is None:
                    continue
                if math.hypot(end_point.x - start_point.x, end_point.y - start_point.y) > 7.0:
                    continue
                cv2.line(frame, start_pixel, end_pixel, draw_color, 3, cv2.LINE_AA)

    def local_detection_preview_points(self, detections: list[CameraDetection]):
        local_cones = []
        if self.latest_stereo_cones:
            local_cones = [(x, y, color, confidence) for x, y, color, confidence, _ in self.latest_stereo_cones]
        else:
            for detection in detections:
                if detection.color in (ConeColor.ORANGE, ConeColor.LARGE_ORANGE, ConeColor.UNKNOWN):
                    continue
                x = detection.range * math.cos(detection.bearing)
                y = detection.range * math.sin(detection.bearing)
                if 0.8 <= x <= 18.0 and abs(y) <= 8.0:
                    local_cones.append((x, y, detection.color, detection.confidence))
        if not local_cones:
            return []

        points = [point(0.0, self.latest_path_offset, 0.0)]
        for start_x in (1.0, 4.0, 7.0, 10.0, 13.0):
            end_x = start_x + 3.5
            group = [cone for cone in local_cones if start_x <= cone[0] < end_x]
            if not group:
                continue
            blue = [y for _, y, color, _ in group if color == ConeColor.BLUE]
            yellow = [y for _, y, color, _ in group if color == ConeColor.YELLOW]
            if blue and yellow:
                center_y = 0.5 * (sum(blue) / len(blue) + sum(yellow) / len(yellow))
            elif blue:
                center_y = sum(blue) / len(blue) - self.track_half_width
            elif yellow:
                center_y = sum(yellow) / len(yellow) + self.track_half_width
            else:
                continue
            center_x = sum(x for x, _, _, _ in group) / len(group)
            points.append(point(center_x, center_y + self.latest_path_offset, 0.0))
        return points if len(points) >= 2 else []

    def draw_local_detection_preview(
        self,
        frame,
        detections: list[CameraDetection],
        camera_name: str,
    ) -> None:
        self.draw_projected_local_preview(frame, detections, camera_name, (255, 255, 0), 2)

    def draw_projected_local_preview(
        self,
        frame,
        detections: list[CameraDetection],
        camera_name: str,
        color,
        thickness: int,
    ) -> None:
        height, width = frame.shape[:2]
        offset = DEFAULT_CAMERA_OFFSETS.get(camera_name, DEFAULT_CAMERA_OFFSETS["cam1"])
        projected = []
        for local_point in self.local_detection_preview_points(detections):
            projected.append(
                project_car_point_to_camera(
                    local_point.x,
                    local_point.y,
                    local_point.z,
                    width,
                    height,
                    offset.x,
                    offset.y,
                    offset.z,
                    self.camera_fov_deg,
                )
            )
        for start, end in zip(projected, projected[1:]):
            if start is None or end is None:
                continue
            cv2.line(frame, start, end, color, thickness, cv2.LINE_AA)

    def draw_image_detection_boundaries(
        self,
        frame,
        detections: list[CameraDetection],
        width: int,
        height: int,
    ) -> int:
        groups = {ConeColor.BLUE: [], ConeColor.YELLOW: []}
        for detection in detections:
            if detection.confidence < self.overlay_min_boundary_confidence:
                continue
            color = int(detection.color)
            if color not in (ConeColor.BLUE, ConeColor.YELLOW, ConeColor.ORANGE, ConeColor.LARGE_ORANGE):
                continue
            x1, _, x2, y2 = detection.bbox
            pixel = (int(round(0.5 * (x1 + x2))), int(y2))
            if pixel[1] < int(height * 0.08) or pixel[1] > int(height * 0.95):
                continue
            if is_start_finish_color(color):
                color = ConeColor.BLUE if pixel[0] < width * 0.5 else ConeColor.YELLOW
            groups[color].append((float(detection.range), pixel, detection.confidence))

        drawn_points = 0
        for color, values in groups.items():
            if len(values) < 2:
                continue
            values = sorted(values, key=lambda item: item[0])
            draw_color = bgr_for_cone(color)
            previous = None
            for _, pixel, _ in values:
                cv2.circle(frame, pixel, 4, draw_color, -1, cv2.LINE_AA)
                if previous is not None:
                    dx = pixel[0] - previous[0]
                    dy = pixel[1] - previous[1]
                    if (dx * dx + dy * dy) ** 0.5 <= width * 0.35 and abs(dy) <= height * 0.42:
                        cv2.line(frame, previous, pixel, draw_color, 3, cv2.LINE_AA)
                previous = pixel
                drawn_points += 1
        return drawn_points

    def draw_stereo_markers(self, frame, camera_name: str) -> None:
        if not self.latest_stereo_cones:
            return
        height, width = frame.shape[:2]
        offset = DEFAULT_CAMERA_OFFSETS.get(camera_name, DEFAULT_CAMERA_OFFSETS["cam1"])
        for x, y, color, confidence, _ in self.latest_stereo_cones:
            pixel = project_car_point_to_camera(
                x,
                y,
                0.0,
                width,
                height,
                offset.x,
                offset.y,
                offset.z,
                self.camera_fov_deg,
            )
            if pixel is None:
                continue
            draw_color = bgr_for_cone(color)
            cv2.circle(frame, pixel, 7, draw_color, 2, cv2.LINE_AA)
            cv2.drawMarker(frame, pixel, draw_color, markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
            cv2.putText(
                frame,
                f"stereo {confidence:.2f}",
                (pixel[0] + 8, max(18, pixel[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                draw_color,
                1,
                cv2.LINE_AA,
            )


def main() -> None:
    rclpy.init()
    rclpy.spin(CameraDetector())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

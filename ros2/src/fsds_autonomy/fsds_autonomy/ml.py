from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np

from fsds_autonomy.constants import (
    CONE_CLASS_TO_COLOR,
    DEFAULT_CAMERA_FOV_DEG,
    DEFAULT_CONE_HEIGHT_M,
    ConeColor,
)
from fsds_autonomy.perception import CameraDetection


HSV_THRESHOLDS = {
    ConeColor.BLUE: ((95, 85, 45), (135, 255, 255)),
    ConeColor.YELLOW: ((18, 85, 80), (42, 255, 255)),
    ConeColor.ORANGE: ((3, 95, 80), (18, 255, 255)),
}


class OptionalYoloDetector:
    def __init__(
        self,
        model_path: str,
        confidence: float = 0.45,
        imgsz: int = 960,
        device: str = "auto",
        warmup: bool = True,
    ):
        self.model = None
        self.names = {}
        self.cone_class_ids: set[int] = set()
        self.confidence = confidence
        self.imgsz = int(imgsz)
        self.model_path = model_path
        self.device = str(device or "auto")
        self.predict_device = None if self.device.lower() in {"", "auto", "none"} else self.device
        self.backend = "unavailable"
        self.display_name = "YOLO unavailable"
        self.load_error = ""
        self.warmup_error = ""
        if not model_path:
            return
        path = Path(model_path).expanduser()
        if not path.exists() and "/" in model_path:
            return
        try:
            from ultralytics import YOLO

            model_source = str(path) if path.exists() else model_path
            self.model = YOLO(model_source)
            self.names = self._normalise_names(getattr(self.model, "names", {}) or {})
            self.cone_class_ids = {
                int(index)
                for index, name in self.names.items()
                if str(name) in CONE_CLASS_TO_COLOR or "cone" in str(name).lower()
            }
            self.backend = self._backend_name(path if path.exists() else Path(model_path))
            model_label = path.name if path.exists() else model_path
            if self.cone_class_ids:
                self.display_name = f"YOLO {self.backend} {model_label}"
            else:
                self.display_name = f"YOLO {self.backend} non-cone classes {model_label}"
            if warmup and self.cone_class_ids:
                self.warmup()
        except Exception as exc:
            self.model = None
            self.names = {}
            self.cone_class_ids = set()
            self.display_name = "YOLO unavailable"
            self.backend = "unavailable"
            self.load_error = str(exc)

    def _normalise_names(self, names) -> dict[int, str]:
        if isinstance(names, dict):
            return {int(index): str(name) for index, name in names.items()}
        if isinstance(names, (list, tuple)):
            return {index: str(name) for index, name in enumerate(names)}
        return {}

    def _backend_name(self, path: Path) -> str:
        suffix = path.suffix.lower()
        if suffix == ".engine":
            return "TensorRT"
        if suffix == ".onnx":
            return "ONNX"
        if suffix == ".pt":
            return "PyTorch"
        if suffix == ".torchscript":
            return "TorchScript"
        if suffix:
            return suffix.lstrip(".")
        return "Ultralytics"

    @property
    def loaded(self) -> bool:
        return self.model is not None

    @property
    def available(self) -> bool:
        return self.model is not None and bool(self.cone_class_ids)

    def _predict_kwargs(self, frame_bgr: np.ndarray) -> dict:
        kwargs = {
            "source": frame_bgr,
            "conf": self.confidence,
            "imgsz": self.imgsz,
            "verbose": False,
        }
        if self.predict_device is not None:
            kwargs["device"] = self.predict_device
        return kwargs

    def warmup(self) -> None:
        if not self.model:
            return
        try:
            warmup_size = max(32, self.imgsz)
            dummy = np.zeros((warmup_size, warmup_size, 3), dtype=np.uint8)
            self.model.predict(**self._predict_kwargs(dummy))
        except Exception as exc:
            self.warmup_error = str(exc)

    def detect(self, frame_bgr: np.ndarray, source: str) -> list[CameraDetection]:
        if not self.available:
            return []
        results = self.model.predict(**self._predict_kwargs(frame_bgr))
        detections: list[CameraDetection] = []
        height, width = frame_bgr.shape[:2]
        focal_px = width / (2.0 * math.tan(math.radians(DEFAULT_CAMERA_FOV_DEG) * 0.5))
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                xyxy = box.xyxy[0].detach().cpu().numpy().astype(int)
                cls = int(box.cls[0].detach().cpu().item())
                name = str(self.names.get(cls, cls))
                color = CONE_CLASS_TO_COLOR.get(name, ConeColor.UNKNOWN)
                if color == ConeColor.UNKNOWN and "cone" not in name:
                    continue
                conf = float(box.conf[0].detach().cpu().item())
                detection = detection_from_bbox(tuple(int(v) for v in xyxy), color, conf, width, focal_px, source)
                if detection is not None:
                    detections.append(detection)
        return detections


def detection_from_bbox(
    bbox: tuple[int, int, int, int],
    color: int,
    confidence: float,
    image_width: int,
    focal_px: float,
    source: str,
) -> CameraDetection | None:
    x1, y1, x2, y2 = bbox
    box_height = max(1, y2 - y1)
    box_width = max(1, x2 - x1)
    if box_height < 5 or box_width < 4:
        return None
    range_m = max(0.5, min(30.0, focal_px * DEFAULT_CONE_HEIGHT_M / box_height))
    center_x = 0.5 * (x1 + x2)
    # Camera projection uses +Y as left of the car. Image pixels increase to
    # the right, so a cone left of center must produce a positive bearing.
    bearing = math.atan2(image_width * 0.5 - center_x, focal_px)
    return CameraDetection(
        color=color,
        confidence=float(confidence),
        bbox=(int(x1), int(y1), int(x2), int(y2)),
        range=range_m,
        bearing=bearing,
        source=source,
    )


def color_segment_cones(
    frame_bgr: np.ndarray,
    source: str,
    confidence: float = 0.28,
    min_area: int = 120,
    max_detections: int = 20,
    roi_top_fraction: float = 0.34,
    roi_bottom_fraction: float = 0.84,
    own_vehicle_mask_top_fraction: float = 0.62,
    own_vehicle_mask_left_fraction: float = 0.12,
    own_vehicle_mask_right_fraction: float = 0.88,
    camera_fov_deg: float = DEFAULT_CAMERA_FOV_DEG,
) -> list[CameraDetection]:
    height, width = frame_bgr.shape[:2]
    focal_px = width / (2.0 * math.tan(math.radians(camera_fov_deg) * 0.5))
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    detections: list[CameraDetection] = []
    min_y = int(height * roi_top_fraction)
    max_y = int(height * roi_bottom_fraction)

    for color, (lower, upper) in HSV_THRESHOLDS.items():
        mask = cv2.inRange(hsv, np.asarray(lower, dtype=np.uint8), np.asarray(upper, dtype=np.uint8))
        mask[:min_y, :] = 0
        mask[max_y:, :] = 0
        vehicle_y = int(height * own_vehicle_mask_top_fraction)
        vehicle_x1 = int(width * own_vehicle_mask_left_fraction)
        vehicle_x2 = int(width * own_vehicle_mask_right_fraction)
        if vehicle_y < height and vehicle_x1 < vehicle_x2:
            mask[vehicle_y:, vehicle_x1:vehicle_x2] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if h < 6 or w < 4:
                continue
            if w > width * 0.18 or h > height * 0.62:
                continue
            aspect = w / float(max(1, h))
            if aspect < 0.12 or aspect > 1.75:
                continue
            extent = area / float(max(1, w * h))
            if extent < 0.10 or extent > 0.88:
                continue

            x1, y1, x2, y2 = expanded_cone_bbox(x, y, w, h, width, height)
            expanded_h = y2 - y1
            expanded_w = x2 - x1
            expanded_aspect = expanded_w / float(max(1, expanded_h))
            if expanded_aspect < 0.22 or expanded_aspect > 1.35:
                continue

            class_color = color
            if color == ConeColor.ORANGE and expanded_h > max(42, int(height * 0.18)):
                class_color = ConeColor.LARGE_ORANGE

            area_score = min(0.10, area / float(width * height) * 15.0)
            shape_score = 0.06 * (1.0 - min(1.0, abs(aspect - 0.55) / 0.75))
            fill_score = 0.04 * (1.0 - min(1.0, abs(extent - 0.42) / 0.42))
            box_confidence = min(0.42, confidence + area_score + shape_score + fill_score)
            detection = detection_from_bbox((x1, y1, x2, y2), class_color, box_confidence, width, focal_px, source)
            if detection is not None:
                if class_color == ConeColor.BLUE and detection.bearing < -0.60:
                    continue
                if class_color == ConeColor.YELLOW and detection.bearing > 0.60:
                    continue
                detections.append(detection)

    return non_max_suppression(sorted(detections, key=lambda item: item.confidence, reverse=True), max_detections)


def expanded_cone_bbox(x: int, y: int, w: int, h: int, width: int, height: int) -> tuple[int, int, int, int]:
    center_x = x + 0.5 * w
    y2 = y + h + int(0.10 * h)
    cone_h = max(h + 4, int(h * 1.75))
    cone_w = max(w + 4, int(cone_h * 0.48), int(w * 1.25))
    x1 = int(round(center_x - 0.5 * cone_w))
    x2 = int(round(center_x + 0.5 * cone_w))
    y1 = int(round(y2 - cone_h))
    return (
        max(0, min(width - 1, x1)),
        max(0, min(height - 1, y1)),
        max(0, min(width - 1, x2)),
        max(0, min(height - 1, y2)),
    )


def bbox_iou(left: tuple[int, int, int, int], right: tuple[int, int, int, int]) -> float:
    lx1, ly1, lx2, ly2 = left
    rx1, ry1, rx2, ry2 = right
    ix1 = max(lx1, rx1)
    iy1 = max(ly1, ry1)
    ix2 = min(lx2, rx2)
    iy2 = min(ly2, ry2)
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    intersection = iw * ih
    if intersection <= 0:
        return 0.0
    left_area = max(1, (lx2 - lx1) * (ly2 - ly1))
    right_area = max(1, (rx2 - rx1) * (ry2 - ry1))
    return intersection / float(left_area + right_area - intersection)


def bbox_overlap_fraction(left: tuple[int, int, int, int], right: tuple[int, int, int, int]) -> float:
    lx1, ly1, lx2, ly2 = left
    rx1, ry1, rx2, ry2 = right
    ix1 = max(lx1, rx1)
    iy1 = max(ly1, ry1)
    ix2 = min(lx2, rx2)
    iy2 = min(ly2, ry2)
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    intersection = iw * ih
    if intersection <= 0:
        return 0.0
    left_area = max(1, (lx2 - lx1) * (ly2 - ly1))
    right_area = max(1, (rx2 - rx1) * (ry2 - ry1))
    return intersection / float(min(left_area, right_area))


def detection_local_distance(left: CameraDetection, right: CameraDetection) -> float:
    left_x = left.range * math.cos(left.bearing)
    left_y = left.range * math.sin(left.bearing)
    right_x = right.range * math.cos(right.bearing)
    right_y = right.range * math.sin(right.bearing)
    return math.hypot(left_x - right_x, left_y - right_y)


def duplicate_detection(
    left: CameraDetection,
    right: CameraDetection,
    iou_threshold: float,
    overlap_threshold: float,
    local_distance_m: float,
) -> bool:
    overlaps = (
        bbox_iou(left.bbox, right.bbox) > iou_threshold
        or bbox_overlap_fraction(left.bbox, right.bbox) > overlap_threshold
    )
    return overlaps and detection_local_distance(left, right) <= local_distance_m


def non_max_suppression(
    detections: list[CameraDetection],
    max_detections: int,
    iou_threshold: float = 0.32,
    overlap_threshold: float = 0.62,
    duplicate_local_distance_m: float = 0.55,
) -> list[CameraDetection]:
    selected: list[CameraDetection] = []
    for detection in detections:
        if any(
            duplicate_detection(
                detection,
                kept,
                iou_threshold,
                overlap_threshold,
                duplicate_local_distance_m,
            )
            for kept in selected
        ):
            continue
        selected.append(detection)
        if len(selected) >= max_detections:
            break
    return selected

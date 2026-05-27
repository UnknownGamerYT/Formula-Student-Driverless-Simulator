from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from fsds_autonomy.constants import ConeColor


HSV_THRESHOLDS = {
    ConeColor.BLUE: ((95, 70, 35), (135, 255, 255)),
    ConeColor.YELLOW: ((17, 65, 65), (44, 255, 255)),
    ConeColor.ORANGE: ((2, 70, 65), (22, 255, 255)),
    ConeColor.LARGE_ORANGE: ((2, 70, 65), (22, 255, 255)),
}


@dataclass(frozen=True)
class LabelQualityStats:
    color_pixels: int
    required_pixels: int
    shift_px: float


def yolo_label_to_bbox(
    label: tuple[int, float, float, float, float], image_width: int, image_height: int
) -> tuple[int, int, int, int]:
    _, x_center, y_center, width, height = label
    box_w = max(1.0, width * image_width)
    box_h = max(1.0, height * image_height)
    cx = x_center * image_width
    cy = y_center * image_height
    x1 = int(round(cx - 0.5 * box_w))
    y1 = int(round(cy - 0.5 * box_h))
    x2 = int(round(cx + 0.5 * box_w))
    y2 = int(round(cy + 0.5 * box_h))
    return clamp_bbox((x1, y1, x2, y2), image_width, image_height)


def bbox_to_yolo_label(
    color: int, bbox: tuple[int, int, int, int], image_width: int, image_height: int
) -> tuple[int, float, float, float, float]:
    x1, y1, x2, y2 = clamp_bbox(bbox, image_width, image_height)
    box_w = max(1, x2 - x1)
    box_h = max(1, y2 - y1)
    return (
        int(color),
        (x1 + 0.5 * box_w) / image_width,
        (y1 + 0.5 * box_h) / image_height,
        box_w / image_width,
        box_h / image_height,
    )


def clamp_bbox(bbox: tuple[int, int, int, int], image_width: int, image_height: int) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    x1 = max(0, min(image_width - 1, x1))
    y1 = max(0, min(image_height - 1, y1))
    x2 = max(0, min(image_width - 1, x2))
    y2 = max(0, min(image_height - 1, y2))
    if x2 <= x1:
        x2 = min(image_width - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(image_height - 1, y1 + 1)
    return x1, y1, x2, y2


def expand_bbox(
    bbox: tuple[int, int, int, int], image_width: int, image_height: int, scale: float
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = bbox
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)
    box_w = max(1.0, (x2 - x1) * scale)
    box_h = max(1.0, (y2 - y1) * scale)
    margin = 6
    expanded = (
        int(round(cx - 0.5 * box_w)) - margin,
        int(round(cy - 0.5 * box_h)) - margin,
        int(round(cx + 0.5 * box_w)) + margin,
        int(round(cy + 0.5 * box_h)) + margin,
    )
    return clamp_bbox(expanded, image_width, image_height)


def refine_projected_label_with_color(
    frame_bgr: np.ndarray,
    label: tuple[int, float, float, float, float],
    *,
    search_scale: float = 2.6,
    min_color_pixels: int = 5,
    min_color_ratio: float = 0.012,
    snap_to_color: bool = True,
) -> tuple[tuple[int, float, float, float, float], LabelQualityStats] | None:
    height, width = frame_bgr.shape[:2]
    color = int(label[0])
    threshold = HSV_THRESHOLDS.get(color)
    if threshold is None:
        return None

    proposal = yolo_label_to_bbox(label, width, height)
    x1, y1, x2, y2 = proposal
    proposal_area = max(1, (x2 - x1) * (y2 - y1))
    search = expand_bbox(proposal, width, height, search_scale)
    sx1, sy1, sx2, sy2 = search

    crop = frame_bgr[sy1:sy2, sx1:sx2]
    if crop.size == 0:
        return None

    lower, upper = threshold
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.asarray(lower, dtype=np.uint8), np.asarray(upper, dtype=np.uint8))
    color_pixels = int(cv2.countNonZero(mask))
    required_pixels = max(int(min_color_pixels), int(proposal_area * float(min_color_ratio)))
    if color_pixels < required_pixels:
        return None

    if not snap_to_color:
        stats = LabelQualityStats(color_pixels=color_pixels, required_pixels=required_pixels, shift_px=0.0)
        return label, stats

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    proposal_cx = 0.5 * (x1 + x2)
    proposal_cy = 0.5 * (y1 + y2)
    best_rect: tuple[int, int, int, int] | None = None
    best_score = -1.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < max(2, min_color_pixels * 0.5):
            continue
        bx, by, bw, bh = cv2.boundingRect(contour)
        center_x = sx1 + bx + 0.5 * bw
        center_y = sy1 + by + 0.5 * bh
        distance = ((center_x - proposal_cx) ** 2 + (center_y - proposal_cy) ** 2) ** 0.5
        score = float(area) / (1.0 + 0.12 * distance)
        if score > best_score:
            best_score = score
            best_rect = (sx1 + bx, sy1 + by, sx1 + bx + bw, sy1 + by + bh)

    if best_rect is None:
        return None

    bx1, by1, bx2, by2 = best_rect
    blob_cx = 0.5 * (bx1 + bx2)
    blob_cy = 0.5 * (by1 + by2)
    box_w = max(1, x2 - x1)
    box_h = max(1, y2 - y1)

    snapped = clamp_bbox(
        (
            int(round(blob_cx - 0.5 * box_w)),
            int(round(blob_cy - 0.5 * box_h)),
            int(round(blob_cx + 0.5 * box_w)),
            int(round(blob_cy + 0.5 * box_h)),
        ),
        width,
        height,
    )
    refined = bbox_to_yolo_label(color, snapped, width, height)
    shift = ((blob_cx - proposal_cx) ** 2 + (blob_cy - proposal_cy) ** 2) ** 0.5
    stats = LabelQualityStats(color_pixels=color_pixels, required_pixels=required_pixels, shift_px=shift)
    return refined, stats

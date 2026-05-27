from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from fsds_autonomy.constants import ConeColor


@dataclass(frozen=True)
class ConeCluster:
    x: float
    y: float
    z: float
    width: float
    depth: float
    height: float
    points: int
    confidence: float

    @property
    def range(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def bearing(self) -> float:
        return math.atan2(self.y, self.x)


@dataclass(frozen=True)
class CameraDetection:
    color: int
    confidence: float
    bbox: tuple[int, int, int, int]
    range: float
    bearing: float
    source: str


def rotate_translate_xy(points: np.ndarray, offset_x: float, offset_y: float, yaw: float) -> np.ndarray:
    if points.size == 0:
        return points
    c = math.cos(yaw)
    s = math.sin(yaw)
    out = points.copy()
    x = points[:, 0]
    y = points[:, 1]
    out[:, 0] = offset_x + c * x - s * y
    out[:, 1] = offset_y + s * x + c * y
    return out


def ordered_clusters_from_points(
    points: np.ndarray,
    cluster_tolerance: float = 0.22,
    min_points: int = 3,
    max_cone_width: float = 0.70,
    max_cone_depth: float = 0.70,
    max_cone_height: float = 0.90,
) -> tuple[list[ConeCluster], list[ConeCluster]]:
    if points.size == 0:
        return [], []

    xy = points[:, :2]
    order = np.argsort(np.arctan2(xy[:, 1], xy[:, 0]))
    ordered = points[order]

    groups: list[list[np.ndarray]] = []
    current: list[np.ndarray] = [ordered[0]]
    for index in range(1, ordered.shape[0]):
        if np.linalg.norm(ordered[index, :2] - ordered[index - 1, :2]) <= cluster_tolerance:
            current.append(ordered[index])
        else:
            if current:
                groups.append(current)
            current = [ordered[index]]
    if current:
        groups.append(current)

    cones: list[ConeCluster] = []
    obstacles: list[ConeCluster] = []
    for group in groups:
        if len(group) < min_points:
            continue
        arr = np.asarray(group, dtype=np.float32)
        mins = arr.min(axis=0)
        maxs = arr.max(axis=0)
        extents = maxs - mins
        centroid = arr.mean(axis=0)
        cluster = ConeCluster(
            x=float(centroid[0]),
            y=float(centroid[1]),
            z=float(centroid[2]) if arr.shape[1] > 2 else 0.0,
            width=float(extents[1]),
            depth=float(extents[0]),
            height=float(extents[2]) if arr.shape[1] > 2 else 0.0,
            points=len(group),
            confidence=float(min(1.0, len(group) / 18.0)),
        )
        if (
            cluster.width <= max_cone_width
            and cluster.depth <= max_cone_depth
            and cluster.height <= max_cone_height
        ):
            cones.append(cluster)
        else:
            obstacles.append(cluster)
    return cones, obstacles


def filter_lidar_points(
    points: Iterable[tuple[float, float, float]],
    min_x: float = 0.05,
    max_x: float = 25.0,
    max_abs_y: float = 12.0,
    min_z: float = -1.5,
    max_z: float = 1.5,
) -> np.ndarray:
    rows = []
    for x, y, z in points:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if min_x <= x <= max_x and abs(y) <= max_abs_y and min_z <= z <= max_z:
            rows.append((float(x), float(y), float(z)))
    if not rows:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(rows, dtype=np.float32)


def fuse_color(existing_color: int, existing_confidence: float, new_color: int, new_confidence: float) -> int:
    if new_color == ConeColor.UNKNOWN:
        return existing_color
    if existing_color == ConeColor.UNKNOWN:
        return new_color
    if new_color == existing_color:
        return existing_color
    return new_color if new_confidence > existing_confidence else existing_color

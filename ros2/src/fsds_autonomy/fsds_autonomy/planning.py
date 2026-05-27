from __future__ import annotations

from collections import defaultdict
from typing import Iterable, Sequence

from geometry_msgs.msg import Point

from fsds_autonomy.constants import ConeColor, is_boundary_color
from fsds_autonomy.geometry import greedy_order_points, point, smooth_path, speed_profile_for_path
from fsds_autonomy.map_store import ConeLandmark


def landmarks_to_points(cones: Iterable[ConeLandmark], color: int | None = None) -> list[Point]:
    return [
        point(cone.x, cone.y, cone.z)
        for cone in cones
        if color is None or int(cone.color) == int(color)
    ]


def build_centerline_from_cones(cones: Sequence[ConeLandmark], max_pair_distance: float = 7.0) -> list[Point]:
    # Blue is the left boundary and yellow is the right boundary. Orange cones
    # are start/end gate markers, so they stay in the map but are not paired.
    blue = landmarks_to_points(cones, ConeColor.BLUE)
    yellow = landmarks_to_points(cones, ConeColor.YELLOW)
    if not blue or not yellow:
        return []

    midpoints: list[Point] = []
    used = set()
    for b in blue:
        best_index = None
        best_dist = max_pair_distance
        for index, y in enumerate(yellow):
            dist = ((b.x - y.x) ** 2 + (b.y - y.y) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best_index = index
        if best_index is None:
            continue
        used.add(best_index)
        y = yellow[best_index]
        midpoints.append(point(0.5 * (b.x + y.x), 0.5 * (b.y + y.y), 0.0))

    for index, y in enumerate(yellow):
        if index in used:
            continue
        best = min(blue, key=lambda b: (b.x - y.x) ** 2 + (b.y - y.y) ** 2)
        dist = ((best.x - y.x) ** 2 + (best.y - y.y) ** 2) ** 0.5
        if dist <= max_pair_distance:
            midpoints.append(point(0.5 * (best.x + y.x), 0.5 * (best.y + y.y), 0.0))

    if len(midpoints) < 2:
        return []
    return smooth_path(greedy_order_points(midpoints), passes=2)


def infer_map_quality(cones: Sequence[ConeLandmark], centerline: Sequence[Point]) -> float:
    if not cones:
        return 0.0
    colors = defaultdict(int)
    for cone in cones:
        colors[int(cone.color)] += 1
    color_balance = 0.0
    if colors[ConeColor.BLUE] and colors[ConeColor.YELLOW]:
        color_balance = min(colors[ConeColor.BLUE], colors[ConeColor.YELLOW]) / max(colors[ConeColor.BLUE], colors[ConeColor.YELLOW])
    boundary_count = sum(1 for cone in cones if is_boundary_color(cone.color))
    count_score = min(1.0, boundary_count / 80.0)
    line_score = min(1.0, len(centerline) / 40.0)
    if not centerline:
        return 0.0
    return float(0.45 * count_score + 0.35 * line_score + 0.20 * color_balance)


def build_racing_line(centerline: Sequence[Point]) -> tuple[list[Point], list[float]]:
    if not centerline:
        return [], []
    racing_line = smooth_path(centerline, passes=4)
    speed_profile = speed_profile_for_path(racing_line, min_speed=2.0, max_speed=7.0, curvature_gain=22.0)
    return racing_line, speed_profile

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

from geometry_msgs.msg import Point, PoseStamped, Quaternion


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def pose2_from_pose_stamped(msg: PoseStamped) -> Pose2:
    return Pose2(
        x=float(msg.pose.position.x),
        y=float(msg.pose.position.y),
        yaw=yaw_from_quaternion(msg.pose.orientation),
    )


def point(x: float, y: float, z: float = 0.0) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def distance_xy(a: Point, b: Point) -> float:
    return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))


def transform_local_to_global(local_x: float, local_y: float, pose: Pose2) -> tuple[float, float]:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return pose.x + c * local_x - s * local_y, pose.y + s * local_x + c * local_y


def transform_global_to_local(global_x: float, global_y: float, pose: Pose2) -> tuple[float, float]:
    dx = global_x - pose.x
    dy = global_y - pose.y
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def nearest_point_index(points: Sequence[Point], x: float, y: float) -> int:
    if not points:
        return -1
    best_index = 0
    best_dist = float("inf")
    for index, p in enumerate(points):
        dist = (float(p.x) - x) ** 2 + (float(p.y) - y) ** 2
        if dist < best_dist:
            best_dist = dist
            best_index = index
    return best_index


def greedy_order_points(points: Sequence[Point]) -> list[Point]:
    if len(points) <= 2:
        return list(points)

    remaining = list(points)
    start_index = min(range(len(remaining)), key=lambda i: remaining[i].x * remaining[i].x + remaining[i].y * remaining[i].y)
    ordered = [remaining.pop(start_index)]

    while remaining:
        last = ordered[-1]
        next_index = min(range(len(remaining)), key=lambda i: distance_xy(last, remaining[i]))
        ordered.append(remaining.pop(next_index))
    return ordered


def smooth_path(points: Sequence[Point], passes: int = 2) -> list[Point]:
    if len(points) < 3:
        return list(points)

    smoothed = [point(p.x, p.y, p.z) for p in points]
    for _ in range(max(0, passes)):
        next_points = [smoothed[0]]
        for i in range(1, len(smoothed) - 1):
            prev_p = smoothed[i - 1]
            cur_p = smoothed[i]
            next_p = smoothed[i + 1]
            next_points.append(
                point(
                    0.25 * prev_p.x + 0.50 * cur_p.x + 0.25 * next_p.x,
                    0.25 * prev_p.y + 0.50 * cur_p.y + 0.25 * next_p.y,
                    0.0,
                )
            )
        next_points.append(smoothed[-1])
        smoothed = next_points
    return smoothed


def path_length(points: Sequence[Point]) -> float:
    return sum(distance_xy(points[i - 1], points[i]) for i in range(1, len(points)))


def closed_path_has_duplicate_endpoint(points: Sequence[Point]) -> bool:
    if len(points) < 2:
        return False
    return distance_xy(points[0], points[-1]) <= 1e-6


def path_segments(points: Sequence[Point], closed_loop: bool = False):
    if len(points) < 2:
        return
    for index in range(1, len(points)):
        yield index - 1, points[index - 1], points[index]
    if closed_loop and len(points) > 2 and not closed_path_has_duplicate_endpoint(points):
        yield len(points) - 1, points[-1], points[0]


def path_segment_count(points: Sequence[Point], closed_loop: bool = False) -> int:
    if len(points) < 2:
        return 0
    if closed_loop and len(points) > 2 and not closed_path_has_duplicate_endpoint(points):
        return len(points)
    return len(points) - 1


def polyline_path_length(points: Sequence[Point], closed_loop: bool = False) -> float:
    return sum(distance_xy(a, b) for _, a, b in path_segments(points, closed_loop))


def closest_point_on_polyline_with_index(
    points: Sequence[Point],
    x: float,
    y: float,
    closed_loop: bool = False,
) -> tuple[Point, float, float, float] | None:
    if not points:
        return None
    if len(points) == 1:
        p = point(points[0].x, points[0].y, points[0].z)
        return p, 0.0, math.hypot(float(p.x) - x, float(p.y) - y), 0.0

    best_point = None
    best_station = 0.0
    best_distance = float("inf")
    best_index = 0.0
    station = 0.0
    for segment_index, start, end in path_segments(points, closed_loop):
        sx = float(start.x)
        sy = float(start.y)
        ex = float(end.x)
        ey = float(end.y)
        vx = ex - sx
        vy = ey - sy
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-12:
            segment_length = 0.0
            ratio = 0.0
        else:
            ratio = clamp(((x - sx) * vx + (y - sy) * vy) / length_sq, 0.0, 1.0)
            segment_length = math.sqrt(length_sq)
        px = sx + ratio * vx
        py = sy + ratio * vy
        distance = math.hypot(x - px, y - py)
        if distance < best_distance:
            best_point = point(px, py, float(start.z) + ratio * (float(end.z) - float(start.z)))
            best_station = station + ratio * segment_length
            best_distance = distance
            best_index = float(segment_index) + ratio
        station += segment_length

    if best_point is None:
        nearest = points[nearest_point_index(points, x, y)]
        best_point = point(nearest.x, nearest.y, nearest.z)
        best_distance = math.hypot(float(best_point.x) - x, float(best_point.y) - y)
        best_station = 0.0
        best_index = 0.0
    return best_point, best_station, best_distance, best_index


def closest_point_on_polyline(
    points: Sequence[Point],
    x: float,
    y: float,
    closed_loop: bool = False,
) -> tuple[Point, float, float] | None:
    result = closest_point_on_polyline_with_index(points, x, y, closed_loop=closed_loop)
    if result is None:
        return None
    closest, station, distance, _ = result
    return closest, station, distance


def point_at_path_index(
    points: Sequence[Point],
    index_position: float,
    closed_loop: bool = False,
) -> Point | None:
    if not points:
        return None
    if len(points) == 1:
        return point(points[0].x, points[0].y, points[0].z)

    segment_count = path_segment_count(points, closed_loop)
    if segment_count <= 0:
        return point(points[0].x, points[0].y, points[0].z)
    if closed_loop:
        index_position = index_position % float(segment_count)
    else:
        index_position = clamp(index_position, 0.0, float(segment_count))

    segment_index = int(math.floor(index_position))
    ratio = index_position - float(segment_index)
    if segment_index >= segment_count:
        segment_index = segment_count - 1
        ratio = 1.0
    start = points[segment_index]
    next_index = segment_index + 1
    if next_index >= len(points):
        next_index = 0 if closed_loop else len(points) - 1
    end = points[next_index]
    return point(
        float(start.x) + ratio * (float(end.x) - float(start.x)),
        float(start.y) + ratio * (float(end.y) - float(start.y)),
        float(start.z) + ratio * (float(end.z) - float(start.z)),
    )


def point_at_reference_path_index(
    points: Sequence[Point],
    reference_index: float,
    reference_segment_count: int,
    closed_loop: bool = False,
) -> Point | None:
    if reference_segment_count <= 0:
        return point_at_path_index(points, 0.0, closed_loop=closed_loop)
    segment_count = path_segment_count(points, closed_loop)
    if segment_count <= 0:
        return point_at_path_index(points, 0.0, closed_loop=closed_loop)
    target_index = reference_index * float(segment_count) / float(reference_segment_count)
    return point_at_path_index(points, target_index, closed_loop=closed_loop)


def point_at_path_fraction(
    points: Sequence[Point],
    fraction: float,
    closed_loop: bool = False,
) -> Point | None:
    if not points:
        return None
    if len(points) == 1:
        return point(points[0].x, points[0].y, points[0].z)

    total = polyline_path_length(points, closed_loop)
    if total <= 1e-9:
        return point(points[0].x, points[0].y, points[0].z)

    if closed_loop:
        target = (fraction % 1.0) * total
    else:
        target = clamp(fraction, 0.0, 1.0) * total

    station = 0.0
    last_end = points[-1]
    for _, start, end in path_segments(points, closed_loop):
        segment_length = distance_xy(start, end)
        last_end = end
        if segment_length <= 1e-9:
            continue
        if station + segment_length >= target:
            ratio = clamp((target - station) / segment_length, 0.0, 1.0)
            return point(
                float(start.x) + ratio * (float(end.x) - float(start.x)),
                float(start.y) + ratio * (float(end.y) - float(start.y)),
                float(start.z) + ratio * (float(end.z) - float(start.z)),
            )
        station += segment_length

    return point(last_end.x, last_end.y, last_end.z)


def curvature_at(points: Sequence[Point], index: int) -> float:
    if index <= 0 or index >= len(points) - 1:
        return 0.0

    p0 = points[index - 1]
    p1 = points[index]
    p2 = points[index + 1]
    a = distance_xy(p0, p1)
    b = distance_xy(p1, p2)
    c = distance_xy(p0, p2)
    denom = max(a * b * c, 1e-6)
    area2 = abs((p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x))
    return 2.0 * area2 / denom


def speed_profile_for_path(
    points: Sequence[Point],
    min_speed: float = 3.0,
    max_speed: float = 10.0,
    curvature_gain: float = 16.0,
) -> list[float]:
    if not points:
        return []
    speeds = []
    for i in range(len(points)):
        curvature = curvature_at(points, i)
        speed = max_speed / (1.0 + curvature_gain * curvature)
        speeds.append(clamp(speed, min_speed, max_speed))
    return speeds


def first_forward_lookahead(
    points: Sequence[Point],
    pose: Pose2,
    lookahead_m: float,
) -> Point | None:
    if not points:
        return None

    nearest = nearest_point_index(points, pose.x, pose.y)
    if nearest < 0:
        return None

    travelled = 0.0
    previous = points[nearest]
    for i in range(nearest + 1, len(points)):
        travelled += distance_xy(previous, points[i])
        previous = points[i]
        if travelled >= lookahead_m:
            return points[i]
    return points[-1]


def mean_point(points: Iterable[Point]) -> Point:
    values = list(points)
    if not values:
        return point(0.0, 0.0, 0.0)
    return point(
        sum(p.x for p in values) / len(values),
        sum(p.y for p in values) / len(values),
        sum(p.z for p in values) / len(values),
    )

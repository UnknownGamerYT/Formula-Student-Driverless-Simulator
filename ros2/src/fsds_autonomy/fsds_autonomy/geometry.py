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

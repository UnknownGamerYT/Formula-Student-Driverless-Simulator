from __future__ import annotations

import math
from collections import defaultdict
from typing import Iterable, Sequence

from geometry_msgs.msg import Point

from fsds_autonomy.constants import ConeColor, DEFAULT_CONE_WIDTH_M, is_boundary_color, is_start_finish_color
from fsds_autonomy.geometry import distance_xy, greedy_order_points, nearest_point_index, point, smooth_path, speed_profile_for_path
from fsds_autonomy.map_store import ConeLandmark


MAX_PATH_STEP_M = 8.0
MIN_RACING_POINT_SPACING_M = 0.35
MAX_RACING_TURN_RAD = math.radians(85.0)
MIN_BOUNDARY_POINT_SPACING_M = 0.45
DEFAULT_LANDMARK_MERGE_DISTANCE_M = 1.50
BOUNDARY_COLOR_CONFLICT_DISTANCE_M = 2.80
MAX_BOUNDARY_STEP_M = 7.0
MAX_CENTERLINE_BOUNDARY_DISTANCE_M = 8.0
RACING_CAR_WIDTH_M = 1.00
RACING_CONE_WIDTH_M = DEFAULT_CONE_WIDTH_M
RACING_CLEARANCE_MARGIN_M = 1.35
MIN_RACING_CONE_CLEARANCE_M = (
    0.5 * RACING_CAR_WIDTH_M + 0.5 * RACING_CONE_WIDTH_M + RACING_CLEARANCE_MARGIN_M
)
MAX_RACING_CORRIDOR_WIDTH_M = 7.5
RACING_TURN_THRESHOLD_RAD = math.radians(3.0)
RACING_TURN_CONTEXT_POINTS = 5


def landmarks_to_points(cones: Iterable[ConeLandmark], color: int | None = None) -> list[Point]:
    return [
        point(cone.x, cone.y, cone.z)
        for cone in cones
        if color is None or int(cone.color) == int(color)
    ]


def dedupe_cone_landmarks(
    cones: Sequence[ConeLandmark],
    merge_distance_m: float = DEFAULT_LANDMARK_MERGE_DISTANCE_M,
) -> list[ConeLandmark]:
    deduped: list[ConeLandmark] = []
    for cone in sorted(
        cones,
        key=lambda item: (float(item.confidence), int(item.observations)),
        reverse=True,
    ):
        best_index = -1
        best_distance = merge_distance_m
        for index, existing in enumerate(deduped):
            if not cone_colors_mergeable(int(existing.color), int(cone.color)):
                continue
            distance = math.hypot(float(existing.x) - float(cone.x), float(existing.y) - float(cone.y))
            if distance <= best_distance:
                best_distance = distance
                best_index = index
        if best_index < 0:
            deduped.append(
                ConeLandmark(
                    x=float(cone.x),
                    y=float(cone.y),
                    z=float(cone.z),
                    color=int(cone.color),
                    confidence=float(cone.confidence),
                    observations=int(cone.observations),
                )
            )
            continue

        existing = deduped[best_index]
        existing_observations = max(1, int(existing.observations))
        cone_observations = max(1, int(cone.observations))
        total_observations = existing_observations + cone_observations
        existing.x = (
            float(existing.x) * existing_observations + float(cone.x) * cone_observations
        ) / total_observations
        existing.y = (
            float(existing.y) * existing_observations + float(cone.y) * cone_observations
        ) / total_observations
        existing.z = (
            float(existing.z) * existing_observations + float(cone.z) * cone_observations
        ) / total_observations
        existing.observations = total_observations
        if (
            float(cone.confidence) > float(existing.confidence)
            or (
                abs(float(cone.confidence) - float(existing.confidence)) < 1e-6
                and cone_observations > existing_observations
            )
        ):
            existing.color = int(cone.color)
        existing.confidence = max(float(existing.confidence), float(cone.confidence))
    return remove_conflicting_boundary_cones(deduped)


def remove_conflicting_boundary_cones(cones: Sequence[ConeLandmark]) -> list[ConeLandmark]:
    keep = [True for _ in cones]
    for left_index, left in enumerate(cones):
        if not keep[left_index] or int(left.color) not in (ConeColor.BLUE, ConeColor.YELLOW):
            continue
        for right_index in range(left_index + 1, len(cones)):
            right = cones[right_index]
            if not keep[right_index] or int(right.color) not in (ConeColor.BLUE, ConeColor.YELLOW):
                continue
            if int(left.color) == int(right.color):
                continue
            distance = math.hypot(float(left.x) - float(right.x), float(left.y) - float(right.y))
            if distance > BOUNDARY_COLOR_CONFLICT_DISTANCE_M:
                continue
            left_score = landmark_score(left)
            right_score = landmark_score(right)
            if left_score >= right_score:
                keep[right_index] = False
            else:
                keep[left_index] = False
                break
    return [cone for cone, should_keep in zip(cones, keep) if should_keep]


def landmark_score(cone: ConeLandmark) -> tuple[int, float]:
    return int(cone.observations), float(cone.confidence)


def cone_colors_mergeable(left: int, right: int) -> bool:
    if int(left) == int(right):
        return True
    if int(left) == ConeColor.UNKNOWN or int(right) == ConeColor.UNKNOWN:
        return True
    return is_start_finish_color(left) and is_start_finish_color(right)


def longest_contiguous_path(points: Sequence[Point], max_step_m: float = MAX_PATH_STEP_M) -> list[Point]:
    if len(points) < 3:
        return list(points)

    segments: list[list[Point]] = []
    current = [points[0]]
    for next_point in points[1:]:
        if distance_xy(current[-1], next_point) <= max_step_m:
            current.append(next_point)
        else:
            segments.append(current)
            current = [next_point]
    segments.append(current)

    best = max(segments, key=len)
    return best if len(best) >= 3 else list(points)


def compact_path_points(points: Sequence[Point], min_step_m: float = MIN_RACING_POINT_SPACING_M) -> list[Point]:
    if len(points) < 3:
        return list(points)

    compacted = [point(points[0].x, points[0].y, points[0].z)]
    for next_point in points[1:-1]:
        if distance_xy(compacted[-1], next_point) >= min_step_m:
            compacted.append(point(next_point.x, next_point.y, next_point.z))

    last = points[-1]
    if len(compacted) < 2 or distance_xy(compacted[-1], last) >= min_step_m:
        compacted.append(point(last.x, last.y, last.z))
    else:
        compacted[-1] = point(last.x, last.y, last.z)
    return compacted


def turn_angle_at(points: Sequence[Point], index: int) -> float:
    if index <= 0 or index >= len(points) - 1:
        return 0.0

    p0 = points[index - 1]
    p1 = points[index]
    p2 = points[index + 1]
    ax = p1.x - p0.x
    ay = p1.y - p0.y
    bx = p2.x - p1.x
    by = p2.y - p1.y
    len_a = math.hypot(ax, ay)
    len_b = math.hypot(bx, by)
    if len_a < 1e-6 or len_b < 1e-6:
        return 0.0
    dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (len_a * len_b)))
    return math.acos(dot)


def limit_path_turns(points: Sequence[Point], max_turn_rad: float = MAX_RACING_TURN_RAD) -> list[Point]:
    limited = [point(p.x, p.y, p.z) for p in points]
    if len(limited) < 3:
        return limited

    for _ in range(len(limited)):
        worst_index = -1
        worst_turn = max_turn_rad
        for index in range(1, len(limited) - 1):
            turn = turn_angle_at(limited, index)
            if turn > worst_turn:
                worst_turn = turn
                worst_index = index
        if worst_index < 0:
            break
        limited.pop(worst_index)
        if len(limited) < 3:
            break
    return limited


def build_boundary_line_from_cones(cones: Sequence[ConeLandmark], color: int) -> list[Point]:
    points = landmarks_to_points(dedupe_cone_landmarks(cones), color)
    if len(points) <= 2:
        return points
    ordered = greedy_order_points(points)
    ordered = longest_contiguous_path(ordered, max_step_m=MAX_BOUNDARY_STEP_M)
    ordered = compact_path_points(ordered, min_step_m=MIN_BOUNDARY_POINT_SPACING_M)
    ordered = smooth_path(ordered, passes=1)
    return limit_path_turns(ordered)


def build_boundary_line_from_centerline_order(
    cones: Sequence[ConeLandmark],
    color: int,
    centerline: Sequence[Point],
) -> list[Point]:
    if len(centerline) < 3:
        return build_boundary_line_from_cones(cones, color)

    deduped = dedupe_cone_landmarks(cones)
    entries: list[tuple[int, float, tuple[int, float], ConeLandmark]] = []
    for cone in deduped:
        if int(cone.color) != int(color):
            continue
        cone_point = point(cone.x, cone.y, cone.z)
        center_index = nearest_point_index(centerline, cone_point.x, cone_point.y)
        if center_index < 0:
            continue
        center_distance = distance_xy(cone_point, centerline[center_index])
        if center_distance > MAX_CENTERLINE_BOUNDARY_DISTANCE_M:
            continue
        entries.append((center_index, center_distance, landmark_score(cone), cone))

    if len(entries) < 3:
        return build_boundary_line_from_cones(deduped, color)

    best_by_station: dict[int, tuple[float, tuple[int, float], ConeLandmark]] = {}
    for center_index, center_distance, score, cone in entries:
        existing = best_by_station.get(center_index)
        if existing is None or (
            center_distance,
            -score[0],
            -score[1],
        ) < (
            existing[0],
            -existing[1][0],
            -existing[1][1],
        ):
            best_by_station[center_index] = (center_distance, score, cone)

    ordered = [
        point(cone.x, cone.y, cone.z)
        for _, _, cone in (best_by_station[index] for index in sorted(best_by_station))
    ]
    if len(ordered) < 3:
        return build_boundary_line_from_cones(deduped, color)

    ordered = compact_path_points(ordered, min_step_m=MIN_BOUNDARY_POINT_SPACING_M)
    ordered = smooth_path(ordered, passes=1)
    return limit_path_turns(ordered)


def build_boundary_lines_from_cones(
    cones: Sequence[ConeLandmark],
    centerline: Sequence[Point] | None = None,
) -> tuple[list[Point], list[Point]]:
    deduped = dedupe_cone_landmarks(cones)
    if centerline is None:
        centerline = build_centerline_from_cones(deduped)
    if centerline and len(centerline) >= 3:
        return (
            build_boundary_line_from_centerline_order(deduped, ConeColor.BLUE, centerline),
            build_boundary_line_from_centerline_order(deduped, ConeColor.YELLOW, centerline),
        )
    return (
        build_boundary_line_from_cones(deduped, ConeColor.BLUE),
        build_boundary_line_from_cones(deduped, ConeColor.YELLOW),
    )


def build_centerline_from_cones(cones: Sequence[ConeLandmark], max_pair_distance: float = 7.0) -> list[Point]:
    # Blue is the left boundary and yellow is the right boundary. Orange cones
    # are start/end gate markers, so they stay in the map but are not paired.
    deduped = dedupe_cone_landmarks(cones)
    blue = landmarks_to_points(deduped, ConeColor.BLUE)
    yellow = landmarks_to_points(deduped, ConeColor.YELLOW)
    if not blue or not yellow:
        return []

    pair_candidates = []
    for blue_index, blue_point in enumerate(blue):
        for yellow_index, yellow_point in enumerate(yellow):
            dist = distance_xy(blue_point, yellow_point)
            if dist <= max_pair_distance:
                pair_candidates.append((dist, blue_index, yellow_index, blue_point, yellow_point))

    midpoints: list[Point] = []
    used_blue = set()
    used_yellow = set()
    for _, blue_index, yellow_index, blue_point, yellow_point in sorted(pair_candidates, key=lambda item: item[0]):
        if blue_index in used_blue or yellow_index in used_yellow:
            continue
        used_blue.add(blue_index)
        used_yellow.add(yellow_index)
        midpoints.append(point(0.5 * (blue_point.x + yellow_point.x), 0.5 * (blue_point.y + yellow_point.y), 0.0))

    if len(midpoints) < 2:
        return []
    ordered = greedy_order_points(midpoints)
    ordered = longest_contiguous_path(ordered, max_step_m=max(MAX_PATH_STEP_M, max_pair_distance * 1.15))
    ordered = compact_path_points(ordered, min_step_m=MIN_RACING_POINT_SPACING_M)
    return smooth_path(ordered, passes=2)


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


def build_racing_line(
    centerline: Sequence[Point],
    cones: Sequence[ConeLandmark] | None = None,
    reset_events: Sequence[dict] | None = None,
    reset_event_influence_radius_m: float = 7.0,
    reset_event_max_shift_m: float = 0.80,
) -> tuple[list[Point], list[float]]:
    if not centerline:
        return [], []
    target_line = build_track_width_racing_targets(centerline, cones) if cones else list(centerline)
    racing_line = sanitize_racing_line(target_line, cones)
    racing_line = apply_reset_event_avoidance(
        racing_line,
        cones,
        reset_events,
        influence_radius_m=reset_event_influence_radius_m,
        max_shift_m=reset_event_max_shift_m,
    )
    speed_profile = speed_profile_for_path(racing_line, min_speed=2.0, max_speed=7.0, curvature_gain=22.0)
    return racing_line, speed_profile


def build_track_width_racing_targets(
    centerline: Sequence[Point],
    cones: Sequence[ConeLandmark] | None,
) -> list[Point]:
    if len(centerline) < 3 or not cones:
        return list(centerline)

    deduped = dedupe_cone_landmarks(cones)
    blue = landmarks_to_points(deduped, ConeColor.BLUE)
    yellow = landmarks_to_points(deduped, ConeColor.YELLOW)
    if not blue or not yellow:
        return list(centerline)

    bias = racing_lateral_bias(centerline)
    targets: list[Point] = []
    for index, center_point in enumerate(centerline):
        nearest_blue = nearest_point_to_xy(blue, center_point.x, center_point.y)
        nearest_yellow = nearest_point_to_xy(yellow, center_point.x, center_point.y)
        if nearest_blue is None or nearest_yellow is None:
            targets.append(point(center_point.x, center_point.y, center_point.z))
            continue

        width = distance_xy(nearest_blue, nearest_yellow)
        if width < 1e-6 or width > MAX_RACING_CORRIDOR_WIDTH_M:
            targets.append(point(center_point.x, center_point.y, center_point.z))
            continue

        if width <= 2.0 * MIN_RACING_CONE_CLEARANCE_M:
            t = 0.5
        else:
            margin = min(0.49, MIN_RACING_CONE_CLEARANCE_M / width)
            usable_half = 0.5 - margin
            t = 0.5 + max(-1.0, min(1.0, bias[index])) * usable_half
        vx = float(nearest_blue.x) - float(nearest_yellow.x)
        vy = float(nearest_blue.y) - float(nearest_yellow.y)
        targets.append(point(float(nearest_yellow.x) + vx * t, float(nearest_yellow.y) + vy * t, 0.0))
    return targets


def racing_lateral_bias(points: Sequence[Point]) -> list[float]:
    if len(points) < 3:
        return [0.0 for _ in points]

    turn_signal = [signed_turn_at(points, index) for index in range(len(points))]
    smoothed_signal = smooth_scalar_values(turn_signal, passes=2)
    bias = [0.0 for _ in points]
    index = 1
    while index < len(points) - 1:
        if abs(smoothed_signal[index]) < RACING_TURN_THRESHOLD_RAD:
            index += 1
            continue
        sign = 1.0 if smoothed_signal[index] > 0.0 else -1.0
        start = index
        while (
            index < len(points) - 1
            and abs(smoothed_signal[index]) >= RACING_TURN_THRESHOLD_RAD
            and smoothed_signal[index] * sign > 0.0
        ):
            index += 1
        end = index - 1
        expanded_start = max(0, start - RACING_TURN_CONTEXT_POINTS)
        expanded_end = min(len(points) - 1, end + RACING_TURN_CONTEXT_POINTS)
        span = max(1, expanded_end - expanded_start)
        # Positive bias means use the blue/left side of the available track,
        # negative means use the yellow/right side. For a left turn, the
        # racing line enters/exits from the right and clips the left apex.
        outside_bias = -sign
        inside_bias = sign
        for point_index in range(expanded_start, expanded_end + 1):
            phase = (point_index - expanded_start) / span
            apex_weight = math.sin(math.pi * phase)
            target_bias = outside_bias + (inside_bias - outside_bias) * apex_weight
            if abs(target_bias) >= abs(bias[point_index]):
                bias[point_index] = target_bias
    return smooth_scalar_values(bias, passes=3)


def signed_turn_at(points: Sequence[Point], index: int) -> float:
    if index <= 0 or index >= len(points) - 1:
        return 0.0

    p0 = points[index - 1]
    p1 = points[index]
    p2 = points[index + 1]
    ax = float(p1.x) - float(p0.x)
    ay = float(p1.y) - float(p0.y)
    bx = float(p2.x) - float(p1.x)
    by = float(p2.y) - float(p1.y)
    len_a = math.hypot(ax, ay)
    len_b = math.hypot(bx, by)
    if len_a < 1e-6 or len_b < 1e-6:
        return 0.0
    cross = ax * by - ay * bx
    dot = ax * bx + ay * by
    return math.atan2(cross, dot)


def smooth_scalar_values(values: Sequence[float], passes: int = 1) -> list[float]:
    if len(values) < 3:
        return list(values)
    smoothed = [float(value) for value in values]
    for _ in range(max(0, passes)):
        next_values = [smoothed[0]]
        for index in range(1, len(smoothed) - 1):
            next_values.append(0.25 * smoothed[index - 1] + 0.50 * smoothed[index] + 0.25 * smoothed[index + 1])
        next_values.append(smoothed[-1])
        smoothed = next_values
    return smoothed


def sanitize_racing_line(
    points: Sequence[Point],
    cones: Sequence[ConeLandmark] | None = None,
) -> list[Point]:
    clean_path = longest_contiguous_path(points)
    clean_path = compact_path_points(clean_path)
    clean_path = limit_path_turns(clean_path)
    smoothing_passes = 3 if cones else 6
    racing_line = smooth_path(clean_path, passes=smoothing_passes)
    racing_line = compact_path_points(longest_contiguous_path(racing_line))
    racing_line = limit_path_turns(racing_line)
    racing_line = constrain_racing_line_to_cone_corridor(racing_line, cones)
    racing_line = smooth_path(racing_line, passes=1 if cones else 2)
    racing_line = constrain_racing_line_to_cone_corridor(racing_line, cones)
    racing_line = compact_path_points(longest_contiguous_path(racing_line))
    racing_line = limit_path_turns(racing_line)
    racing_line = constrain_racing_line_to_cone_corridor(racing_line, cones)
    return trim_unsafe_racing_endpoints(racing_line, cones)


def apply_reset_event_avoidance(
    points: Sequence[Point],
    cones: Sequence[ConeLandmark] | None,
    reset_events: Sequence[dict] | None,
    influence_radius_m: float = 7.0,
    max_shift_m: float = 0.80,
) -> list[Point]:
    if len(points) < 3 or not reset_events or influence_radius_m <= 0.0 or max_shift_m <= 0.0:
        return list(points)

    clusters = cluster_reset_events(reset_events)
    if not clusters:
        return list(points)

    boundary = []
    if cones:
        deduped = dedupe_cone_landmarks(cones)
        boundary = landmarks_to_points(deduped, ConeColor.BLUE) + landmarks_to_points(deduped, ConeColor.YELLOW)

    shifted: list[Point] = []
    max_total_shift = max_shift_m * 1.75
    for path_index, path_point in enumerate(points):
        shift_x = 0.0
        shift_y = 0.0
        for event in clusters:
            event_point = point(event["x"], event["y"], 0.0)
            distance = distance_xy(path_point, event_point)
            if distance > influence_radius_m:
                continue
            direction = reset_avoidance_direction(points, path_index, event_point, boundary)
            if direction is None:
                continue
            dir_x, dir_y = direction
            count_gain = min(1.65, 0.80 + 0.22 * math.sqrt(max(1.0, float(event["count"]))))
            weight = 0.5 * (1.0 + math.cos(math.pi * distance / influence_radius_m))
            amount = max_shift_m * count_gain * weight
            shift_x += dir_x * amount
            shift_y += dir_y * amount

        shift_norm = math.hypot(shift_x, shift_y)
        if shift_norm > max_total_shift:
            scale = max_total_shift / shift_norm
            shift_x *= scale
            shift_y *= scale
        shifted.append(point(path_point.x + shift_x, path_point.y + shift_y, path_point.z))

    shifted = constrain_racing_line_to_cone_corridor(shifted, cones)
    shifted = smooth_path(shifted, passes=1)
    shifted = constrain_racing_line_to_cone_corridor(shifted, cones)
    shifted = compact_path_points(longest_contiguous_path(shifted))
    shifted = limit_path_turns(shifted)
    shifted = constrain_racing_line_to_cone_corridor(shifted, cones)
    return trim_unsafe_racing_endpoints(shifted, cones)


def cluster_reset_events(
    reset_events: Sequence[dict],
    cluster_radius_m: float = 2.0,
) -> list[dict]:
    clusters: list[dict] = []
    for event in reset_events:
        reason = str(event.get("reason", "")).lower()
        if not reset_reason_affects_line(reason):
            continue
        try:
            event_x = float(event.get("x"))
            event_y = float(event.get("y"))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(event_x) or not math.isfinite(event_y):
            continue
        best = None
        best_distance = cluster_radius_m
        for cluster in clusters:
            distance = math.hypot(event_x - float(cluster["x"]), event_y - float(cluster["y"]))
            if distance <= best_distance:
                best = cluster
                best_distance = distance
        if best is None:
            clusters.append({"x": event_x, "y": event_y, "count": 1, "reasons": {reason.split()[0]}})
            continue
        count = int(best["count"]) + 1
        best["x"] = (float(best["x"]) * int(best["count"]) + event_x) / count
        best["y"] = (float(best["y"]) * int(best["count"]) + event_y) / count
        best["count"] = count
        best["reasons"].add(reason.split()[0])
    return clusters


def reset_reason_affects_line(reason: str) -> bool:
    if not reason:
        return False
    return (
        "cone_hit" in reason
        or "stuck" in reason
        or "forbidden_area=blue_side" in reason
        or "forbidden_area=yellow_side" in reason
    )


def reset_avoidance_direction(
    path: Sequence[Point],
    path_index: int,
    event_point: Point,
    boundary: Sequence[Point],
) -> tuple[float, float] | None:
    tangent = path_tangent(path, path_index)
    if tangent is None:
        return None
    tx, ty = tangent
    normal_x = -ty
    normal_y = tx
    path_point = path[path_index]
    side = (event_point.x - path_point.x) * normal_x + (event_point.y - path_point.y) * normal_y
    if abs(side) < 0.15 and boundary:
        nearest = nearest_point_to_xy(boundary, event_point.x, event_point.y)
        if nearest is not None:
            side = (nearest.x - path_point.x) * normal_x + (nearest.y - path_point.y) * normal_y
    if abs(side) < 0.15:
        dx = path_point.x - event_point.x
        dy = path_point.y - event_point.y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None
        return dx / norm, dy / norm
    sign = 1.0 if side > 0.0 else -1.0
    return -sign * normal_x, -sign * normal_y


def path_tangent(path: Sequence[Point], index: int) -> tuple[float, float] | None:
    if len(path) < 2:
        return None
    if index <= 0:
        p0 = path[0]
        p1 = path[1]
    elif index >= len(path) - 1:
        p0 = path[-2]
        p1 = path[-1]
    else:
        p0 = path[index - 1]
        p1 = path[index + 1]
    dx = float(p1.x) - float(p0.x)
    dy = float(p1.y) - float(p0.y)
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return None
    return dx / norm, dy / norm


def constrain_racing_line_to_cone_corridor(
    points: Sequence[Point],
    cones: Sequence[ConeLandmark] | None,
    min_clearance_m: float = MIN_RACING_CONE_CLEARANCE_M,
) -> list[Point]:
    if not points or not cones:
        return list(points)

    deduped = dedupe_cone_landmarks(cones)
    blue = landmarks_to_points(deduped, ConeColor.BLUE)
    yellow = landmarks_to_points(deduped, ConeColor.YELLOW)
    boundary = blue + yellow
    if not boundary:
        return list(points)

    constrained: list[Point] = []
    for path_point in points:
        adjusted = point(path_point.x, path_point.y, path_point.z)
        local_clearance = min_clearance_m
        nearest_blue = nearest_point_to_xy(blue, adjusted.x, adjusted.y)
        nearest_yellow = nearest_point_to_xy(yellow, adjusted.x, adjusted.y)
        if nearest_blue is not None and nearest_yellow is not None:
            boundary_width = distance_xy(nearest_blue, nearest_yellow)
            if boundary_width <= 2.0 * min_clearance_m:
                local_clearance = max(0.35, 0.5 * boundary_width)
            adjusted = project_point_inside_boundary_pair(
                adjusted,
                nearest_blue,
                nearest_yellow,
                min_clearance_m,
            )
        adjusted = push_point_away_from_cones(adjusted, boundary, local_clearance)
        constrained.append(adjusted)
    return constrained


def nearest_point_to_xy(points: Sequence[Point], x: float, y: float) -> Point | None:
    if not points:
        return None
    return min(points, key=lambda candidate: math.hypot(float(candidate.x) - x, float(candidate.y) - y))


def project_point_inside_boundary_pair(
    path_point: Point,
    blue_point: Point,
    yellow_point: Point,
    min_clearance_m: float,
) -> Point:
    width = distance_xy(blue_point, yellow_point)
    if width < 1e-6 or width > MAX_RACING_CORRIDOR_WIDTH_M:
        return point(path_point.x, path_point.y, path_point.z)

    vx = float(yellow_point.x) - float(blue_point.x)
    vy = float(yellow_point.y) - float(blue_point.y)
    t = ((float(path_point.x) - float(blue_point.x)) * vx + (float(path_point.y) - float(blue_point.y)) * vy) / (
        width * width
    )
    if width > 2.0 * min_clearance_m:
        margin = min(0.5, min_clearance_m / width)
        t = max(margin, min(1.0 - margin, t))
    else:
        t = 0.5
    return point(float(blue_point.x) + vx * t, float(blue_point.y) + vy * t, 0.0)


def push_point_away_from_cones(
    path_point: Point,
    cones: Sequence[Point],
    min_clearance_m: float,
) -> Point:
    adjusted = point(path_point.x, path_point.y, path_point.z)
    for _ in range(8):
        nearest = nearest_point_to_xy(cones, adjusted.x, adjusted.y)
        if nearest is None:
            return adjusted
        dx = float(adjusted.x) - float(nearest.x)
        dy = float(adjusted.y) - float(nearest.y)
        distance = math.hypot(dx, dy)
        if distance >= min_clearance_m:
            return adjusted
        if distance < 1e-6:
            adjusted.x += min_clearance_m
            continue
        scale = (min_clearance_m - distance) / distance
        adjusted.x += dx * scale
        adjusted.y += dy * scale
    return adjusted


def trim_unsafe_racing_endpoints(
    points: Sequence[Point],
    cones: Sequence[ConeLandmark] | None,
    min_clearance_m: float = MIN_RACING_CONE_CLEARANCE_M,
) -> list[Point]:
    if len(points) <= 8 or not cones:
        return list(points)

    deduped = dedupe_cone_landmarks(cones)
    boundary = landmarks_to_points(deduped, ConeColor.BLUE) + landmarks_to_points(deduped, ConeColor.YELLOW)
    if not boundary:
        return list(points)

    trimmed = [point(p.x, p.y, p.z) for p in points]
    min_remaining = max(8, int(math.ceil(len(trimmed) * 0.60)))
    while len(trimmed) > min_remaining and nearest_boundary_distance(trimmed[0], boundary) < min_clearance_m:
        trimmed.pop(0)
    while len(trimmed) > min_remaining and nearest_boundary_distance(trimmed[-1], boundary) < min_clearance_m:
        trimmed.pop()
    return trimmed


def nearest_boundary_distance(path_point: Point, boundary: Sequence[Point]) -> float:
    nearest = nearest_point_to_xy(boundary, path_point.x, path_point.y)
    if nearest is None:
        return float("inf")
    return distance_xy(path_point, nearest)

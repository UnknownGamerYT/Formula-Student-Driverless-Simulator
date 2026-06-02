from __future__ import annotations

import math

import rclpy
from fsds_autonomy_msgs.msg import ConeArray, ObstacleArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from fsds_autonomy.constants import ConeColor, is_start_finish_color
from fsds_autonomy.geometry import distance_xy, greedy_order_points, point, pose2_from_pose_stamped, transform_local_to_global
from fsds_autonomy.visualization import (
    arrow_marker,
    cone_color,
    line_marker,
    path_msg,
    rgba,
    sphere_marker,
    text_marker,
    triangle_list_marker,
)


class FoxgloveVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("fsds_foxglove_visualizer")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_local_cones", 80)
        self.declare_parameter("marker_lifetime_sec", 1.0)
        self.declare_parameter("forbidden_area_enabled", True)
        self.declare_parameter("forbidden_area_outer_band_m", 14.0)
        self.declare_parameter("forbidden_area_alpha", 0.26)
        self.declare_parameter("forbidden_area_max_boundary_gap_m", 24.0)
        self.declare_parameter("forbidden_area_max_boundary_step_m", 12.0)

        self.track_map = TrackMap()
        self.saved_track_map = TrackMap()
        self.live_track_map = TrackMap()
        self.racing_map = TrackMap()
        self.reference_racing_map = TrackMap()
        self.live_racing_map = TrackMap()
        self.local_cones = ConeArray()
        self.obstacles = ObstacleArray()
        self.pose = None
        self.race_state = RaceState()
        self.path_offset = 0.0
        self.marker_lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)
        self.forbidden_area_enabled = bool(self.get_parameter("forbidden_area_enabled").value)
        self.forbidden_area_outer_band = float(self.get_parameter("forbidden_area_outer_band_m").value)
        self.forbidden_area_alpha = float(self.get_parameter("forbidden_area_alpha").value)
        self.forbidden_area_max_boundary_gap = float(self.get_parameter("forbidden_area_max_boundary_gap_m").value)
        self.forbidden_area_max_boundary_step = float(self.get_parameter("forbidden_area_max_boundary_step_m").value)

        self.marker_pub = self.create_publisher(MarkerArray, "/autonomy/viz/map_markers", 10)
        self.saved_map_marker_pub = self.create_publisher(MarkerArray, "/autonomy/viz/saved_map_markers", 10)
        self.live_map_marker_pub = self.create_publisher(MarkerArray, "/autonomy/viz/live_map_markers", 10)
        self.centerline_pub = self.create_publisher(Path, "/autonomy/viz/map_centerline_path", 10)
        self.racing_line_pub = self.create_publisher(Path, "/autonomy/viz/optimal_racing_line_path", 10)
        self.reference_line_pub = self.create_publisher(Path, "/autonomy/viz/reference_racing_line_path", 10)
        self.live_line_pub = self.create_publisher(Path, "/autonomy/viz/live_racing_line_path", 10)
        self.current_line_pub = self.create_publisher(Path, "/autonomy/viz/current_drive_line_path", 10)
        self.reference_blue_boundary_pub = self.create_publisher(Path, "/autonomy/viz/reference_blue_boundary_path", 10)
        self.reference_yellow_boundary_pub = self.create_publisher(Path, "/autonomy/viz/reference_yellow_boundary_path", 10)
        self.live_blue_boundary_pub = self.create_publisher(Path, "/autonomy/viz/live_blue_boundary_path", 10)
        self.live_yellow_boundary_pub = self.create_publisher(Path, "/autonomy/viz/live_yellow_boundary_path", 10)

        self.create_subscription(TrackMap, "/autonomy/track_map", self.on_track_map, 10)
        self.create_subscription(TrackMap, "/autonomy/reference_track_map", self.on_saved_track_map, 10)
        self.create_subscription(TrackMap, "/autonomy/live_track_map", self.on_live_track_map, 10)
        self.create_subscription(TrackMap, "/autonomy/racing_line", self.on_racing_map, 10)
        self.create_subscription(TrackMap, "/autonomy/reference_racing_line", self.on_reference_racing_map, 10)
        self.create_subscription(TrackMap, "/autonomy/live_racing_line", self.on_live_racing_map, 10)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_local_cones, 10)
        self.create_subscription(ObstacleArray, "/autonomy/obstacles", self.on_obstacles, 10)
        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)
        self.create_subscription(RaceState, "/autonomy/race_state", self.on_race_state, 10)
        self.create_subscription(Float32, "/autonomy/path_offset", self.on_path_offset, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish_visuals)

    def on_track_map(self, msg: TrackMap) -> None:
        self.track_map = msg
        if not self.has_track_map_content(self.live_track_map):
            self.live_track_map = msg

    def on_saved_track_map(self, msg: TrackMap) -> None:
        self.saved_track_map = msg

    def on_live_track_map(self, msg: TrackMap) -> None:
        self.live_track_map = msg

    def on_racing_map(self, msg: TrackMap) -> None:
        self.racing_map = msg

    def on_reference_racing_map(self, msg: TrackMap) -> None:
        self.reference_racing_map = msg

    def on_live_racing_map(self, msg: TrackMap) -> None:
        self.live_racing_map = msg

    def on_local_cones(self, msg: ConeArray) -> None:
        self.local_cones = msg

    def on_obstacles(self, msg: ObstacleArray) -> None:
        self.obstacles = msg

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def on_race_state(self, msg: RaceState) -> None:
        self.race_state = msg

    def on_path_offset(self, msg: Float32) -> None:
        self.path_offset = float(msg.data)

    def current_points(self):
        if self.racing_map.racing_line:
            return self.racing_map.racing_line
        if self.track_map.centerline:
            return self.track_map.centerline
        return []

    def publish_visuals(self) -> None:
        header = self.get_header()
        markers = MarkerArray()
        marker_id = 1
        if self.forbidden_area_enabled:
            marker_id = self.add_forbidden_area_markers(markers, header, marker_id)

        for cone in self.track_map.cones:
            markers.markers.append(
                sphere_marker(
                    header,
                    marker_id,
                    "created_map_cones",
                    cone.position,
                    cone_color(cone.color, 0.95),
                    diameter=0.38,
                )
            )
            marker_id += 1
            if is_start_finish_color(cone.color):
                markers.markers.append(
                    text_marker(
                        header,
                        marker_id,
                        "start_finish_gate_text",
                        "START/END",
                        cone.position.x,
                        cone.position.y,
                        0.85,
                        rgba(1.0, 0.62, 0.05, 0.95),
                    )
                )
                marker_id += 1

        if self.pose is not None:
            pose2 = pose2_from_pose_stamped(self.pose)
            for index, cone in enumerate(self.local_cones.cones[: int(self.get_parameter("max_local_cones").value)]):
                gx, gy = transform_local_to_global(cone.position.x, cone.position.y, pose2)
                markers.markers.append(
                    sphere_marker(
                        header,
                        marker_id,
                        "live_fused_cones",
                        point(gx, gy, 0.0),
                        cone_color(cone.color, 0.45),
                        diameter=0.22,
                    )
                )
                marker_id += 1

            for obstacle in self.obstacles.obstacles:
                if obstacle.position.x < 0.0 or obstacle.position.x > 25.0:
                    continue
                gx, gy = transform_local_to_global(obstacle.position.x, obstacle.position.y, pose2)
                markers.markers.append(
                    sphere_marker(
                        header,
                        marker_id,
                        "live_obstacles",
                        point(gx, gy, 0.0),
                        rgba(1.0, 0.05, 0.02, 0.85),
                        diameter=max(0.4, obstacle.radius * 2.0),
                    )
                )
                marker_id += 1

            if abs(self.path_offset) > 0.05:
                avoid_points = []
                for local_x in (0.0, 3.0, 6.0, 9.0, 12.0):
                    gx, gy = transform_local_to_global(local_x, self.path_offset, pose2)
                    avoid_points.append(point(gx, gy, 0.0))
                markers.markers.append(
                    line_marker(
                        header,
                        marker_id,
                        "obstacle_avoid_line",
                        avoid_points,
                        rgba(0.0, 1.0, 0.2, 0.95),
                        width=0.18,
                        z_offset=0.22,
                    )
                )
                marker_id += 1

            markers.markers.append(
                arrow_marker(
                    header,
                    marker_id,
                    "vehicle_pose",
                    pose2.x,
                    pose2.y,
                    pose2.yaw,
                    rgba(1.0, 1.0, 1.0, 1.0),
                )
            )
            marker_id += 1
            markers.markers.append(
                text_marker(
                    header,
                    marker_id,
                    "race_state_text",
                    f"{self.race_state.behavior_state or 'Waiting'} | {self.race_state.target_speed:.1f} m/s | map {self.track_map.quality:.2f}",
                    pose2.x,
                    pose2.y,
                    1.4,
                    rgba(1.0, 1.0, 1.0, 0.95),
                )
            )
            marker_id += 1

        reference_boundaries = self.map_boundary_lines(self.reference_racing_map)
        reference_blue = self.display_boundary_line(
            reference_boundaries[ConeColor.BLUE],
            bool(self.reference_racing_map.closed_loop),
        )
        reference_yellow = self.display_boundary_line(
            reference_boundaries[ConeColor.YELLOW],
            bool(self.reference_racing_map.closed_loop),
        )
        if len(reference_blue) >= 2:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "reference_blue_boundary",
                    reference_blue,
                    cone_color(ConeColor.BLUE, 0.45),
                    width=0.09,
                    z_offset=0.12,
                )
            )
            marker_id += 1

        if len(reference_yellow) >= 2:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "reference_yellow_boundary",
                    reference_yellow,
                    cone_color(ConeColor.YELLOW, 0.45),
                    width=0.09,
                    z_offset=0.12,
                )
            )
            marker_id += 1

        live_boundary_map = self.live_racing_map if self.live_racing_map.cones else self.track_map
        live_boundaries = self.map_boundary_lines(live_boundary_map)
        live_blue = self.display_boundary_line(
            live_boundaries[ConeColor.BLUE],
            bool(live_boundary_map.closed_loop),
        )
        live_yellow = self.display_boundary_line(
            live_boundaries[ConeColor.YELLOW],
            bool(live_boundary_map.closed_loop),
        )
        if len(live_blue) >= 2:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "live_blue_boundary",
                    live_blue,
                    cone_color(ConeColor.BLUE, 0.95),
                    width=0.13,
                    z_offset=0.19,
                )
            )
            marker_id += 1

        if len(live_yellow) >= 2:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "live_yellow_boundary",
                    live_yellow,
                    cone_color(ConeColor.YELLOW, 0.95),
                    width=0.13,
                    z_offset=0.19,
                )
            )
            marker_id += 1

        if self.track_map.centerline:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "created_map_centerline",
                    self.track_map.centerline,
                    rgba(0.0, 0.9, 1.0, 0.9),
                    width=0.08,
                    z_offset=0.08,
                )
            )
            marker_id += 1

        reference_line = self.reference_racing_map.racing_line
        if reference_line:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "reference_racing_line",
                    reference_line,
                    rgba(1.0, 0.0, 0.9, 0.95),
                    width=0.11,
                    z_offset=0.11,
                )
            )
            marker_id += 1

        live_line = self.live_racing_map.racing_line or self.racing_map.racing_line
        if live_line:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "live_racing_line",
                    live_line,
                    rgba(0.0, 1.0, 1.0, 0.95),
                    width=0.13,
                    z_offset=0.17,
                )
            )
            marker_id += 1

        current = self.current_points()
        if current:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "current_drive_line",
                    current,
                    rgba(1.0, 1.0, 0.0, 0.95),
                    width=0.16,
                    z_offset=0.16,
                )
            )

        self.apply_marker_lifetime(markers)
        self.marker_pub.publish(markers)
        self.centerline_pub.publish(path_msg(header, self.track_map.centerline))
        self.racing_line_pub.publish(path_msg(header, reference_line or self.racing_map.racing_line))
        self.reference_line_pub.publish(path_msg(header, reference_line))
        self.live_line_pub.publish(path_msg(header, live_line))
        self.current_line_pub.publish(path_msg(header, current))
        self.reference_blue_boundary_pub.publish(path_msg(header, reference_blue))
        self.reference_yellow_boundary_pub.publish(path_msg(header, reference_yellow))
        self.live_blue_boundary_pub.publish(path_msg(header, live_blue))
        self.live_yellow_boundary_pub.publish(path_msg(header, live_yellow))
        saved_markers = self.build_saved_map_markers(header)
        live_markers = self.build_live_map_markers(header)
        self.apply_marker_lifetime(saved_markers)
        self.apply_marker_lifetime(live_markers)
        self.saved_map_marker_pub.publish(saved_markers)
        self.live_map_marker_pub.publish(live_markers)

    def apply_marker_lifetime(self, markers: MarkerArray) -> None:
        lifetime = max(0.0, float(self.marker_lifetime_sec))
        if lifetime <= 0.0:
            return
        sec = int(lifetime)
        nanosec = int(round((lifetime - sec) * 1_000_000_000))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        for marker in markers.markers:
            if marker.action != Marker.ADD:
                continue
            marker.lifetime.sec = sec
            marker.lifetime.nanosec = nanosec

    def add_forbidden_area_markers(self, markers: MarkerArray, header, marker_id: int) -> int:
        track_map = self.forbidden_area_map()
        if track_map is None:
            return marker_id
        boundaries = self.map_boundary_lines(track_map)
        if not self.boundary_loop_usable(boundaries[ConeColor.BLUE]):
            return marker_id
        if not self.boundary_loop_usable(boundaries[ConeColor.YELLOW]):
            return marker_id
        blue = self.closed_boundary_points(boundaries[ConeColor.BLUE])
        yellow = self.closed_boundary_points(boundaries[ConeColor.YELLOW])
        if len(blue) < 3 or len(yellow) < 3:
            return marker_id

        blue_area = abs(self.polygon_area(blue))
        yellow_area = abs(self.polygon_area(yellow))
        if blue_area < 10.0 or yellow_area < 10.0:
            return marker_id

        inner = blue if blue_area <= yellow_area else yellow
        outer = yellow if blue_area <= yellow_area else blue
        color = rgba(1.0, 0.0, 0.0, self.forbidden_area_alpha)

        inner_triangles = self.polygon_triangles(inner)
        if inner_triangles:
            markers.markers.append(
                triangle_list_marker(
                    header,
                    marker_id,
                    "forbidden_inner_area",
                    inner_triangles,
                    color,
                    z_offset=0.015,
                )
            )
            marker_id += 1

        outer_triangles = self.outer_band_triangles(outer, self.forbidden_area_outer_band)
        if outer_triangles:
            markers.markers.append(
                triangle_list_marker(
                    header,
                    marker_id,
                    "forbidden_outer_area",
                    outer_triangles,
                    color,
                    z_offset=0.012,
                )
            )
            marker_id += 1
        return marker_id

    def forbidden_area_map(self) -> TrackMap | None:
        candidates = [
            self.reference_racing_map,
            self.saved_track_map,
            self.track_map,
            self.racing_map,
        ]
        usable = []
        for track_map in candidates:
            if not self.full_loop_map_usable(track_map):
                continue
            usable.append(track_map)
        if not usable:
            return None
        return max(
            usable,
            key=lambda track_map: (
                float(track_map.quality),
                len(self.map_boundary_lines(track_map)[ConeColor.BLUE])
                + len(self.map_boundary_lines(track_map)[ConeColor.YELLOW]),
            ),
        )

    def full_loop_map_usable(self, track_map: TrackMap) -> bool:
        if not bool(track_map.closed_loop):
            return False
        if float(track_map.quality) < 0.55:
            return False
        boundaries = self.map_boundary_lines(track_map)
        checks = [
            (boundaries[ConeColor.BLUE], 12.0, 10.0),
            (boundaries[ConeColor.YELLOW], 12.0, 10.0),
            (track_map.centerline, 18.0, 18.0),
            (track_map.racing_line, 18.0, 18.0),
        ]
        for points, max_endpoint_gap, max_internal_step in checks:
            if len(points) < 8:
                return False
            if self.loop_endpoint_gap(points) > max_endpoint_gap:
                return False
            if self.max_loop_step(points) > max_internal_step:
                return False
        return True

    @staticmethod
    def closed_boundary_points(points) -> list:
        closed = [point(p.x, p.y, 0.0) for p in points]
        if len(closed) >= 3 and distance_xy(closed[0], closed[-1]) < 0.20:
            closed.pop()
        return closed

    def display_boundary_line(self, points, closed_loop: bool) -> list:
        display = [point(p.x, p.y, 0.0) for p in points]
        if (
            closed_loop
            and self.boundary_loop_usable(display)
            and len(display) >= 3
            and distance_xy(display[0], display[-1]) >= 0.20
        ):
            display.append(point(display[0].x, display[0].y, display[0].z))
        return display

    def boundary_loop_usable(self, points) -> bool:
        values = list(points)
        if len(values) < 3:
            return False
        steps = [distance_xy(values[index - 1], values[index]) for index in range(1, len(values))]
        if not steps:
            return False
        ordered_steps = sorted(steps)
        median_step = ordered_steps[len(ordered_steps) // 2]
        max_close_gap = max(self.forbidden_area_max_boundary_gap, 3.5 * median_step)
        max_internal_step = max(self.forbidden_area_max_boundary_step, 3.0 * median_step)
        close_gap = distance_xy(values[0], values[-1])
        return close_gap <= max_close_gap and max(steps) <= max_internal_step

    @staticmethod
    def polygon_area(points) -> float:
        if len(points) < 3:
            return 0.0
        area = 0.0
        for current, nxt in zip(points, points[1:] + points[:1]):
            area += current.x * nxt.y - nxt.x * current.y
        return 0.5 * area

    @staticmethod
    def polygon_center(points) -> tuple[float, float]:
        if not points:
            return 0.0, 0.0
        return (
            sum(float(p.x) for p in points) / len(points),
            sum(float(p.y) for p in points) / len(points),
        )

    def fan_triangles(self, polygon) -> list:
        if len(polygon) < 3:
            return []
        cx, cy = self.polygon_center(polygon)
        center = point(cx, cy, 0.0)
        triangles = []
        for current, nxt in zip(polygon, polygon[1:] + polygon[:1]):
            triangles.extend([center, current, nxt])
        return triangles

    def polygon_triangles(self, polygon) -> list:
        points = [point(p.x, p.y, 0.0) for p in polygon]
        if len(points) < 3:
            return []
        if distance_xy(points[0], points[-1]) < 0.20:
            points.pop()
        if len(points) < 3:
            return []
        if self.polygon_area(points) < 0.0:
            points.reverse()

        remaining = list(range(len(points)))
        triangles = []
        guard = 0
        while len(remaining) > 3 and guard < len(points) * len(points):
            guard += 1
            clipped = False
            for pos, index in enumerate(list(remaining)):
                prev_index = remaining[pos - 1]
                next_index = remaining[(pos + 1) % len(remaining)]
                prev_p = points[prev_index]
                cur_p = points[index]
                next_p = points[next_index]
                if not self.is_convex_corner(prev_p, cur_p, next_p):
                    continue
                if any(
                    self.point_in_triangle(points[other], prev_p, cur_p, next_p)
                    for other in remaining
                    if other not in (prev_index, index, next_index)
                ):
                    continue
                triangles.extend([prev_p, cur_p, next_p])
                remaining.remove(index)
                clipped = True
                break
            if not clipped:
                return []
        if len(remaining) == 3:
            triangles.extend([points[remaining[0]], points[remaining[1]], points[remaining[2]]])
        return triangles

    @staticmethod
    def is_convex_corner(prev_p, cur_p, next_p) -> bool:
        ax = float(cur_p.x) - float(prev_p.x)
        ay = float(cur_p.y) - float(prev_p.y)
        bx = float(next_p.x) - float(cur_p.x)
        by = float(next_p.y) - float(cur_p.y)
        return ax * by - ay * bx > 1e-6

    @staticmethod
    def point_in_triangle(p, a, b, c) -> bool:
        px, py = float(p.x), float(p.y)
        ax, ay = float(a.x), float(a.y)
        bx, by = float(b.x), float(b.y)
        cx, cy = float(c.x), float(c.y)
        v0x, v0y = cx - ax, cy - ay
        v1x, v1y = bx - ax, by - ay
        v2x, v2y = px - ax, py - ay
        dot00 = v0x * v0x + v0y * v0y
        dot01 = v0x * v1x + v0y * v1y
        dot02 = v0x * v2x + v0y * v2y
        dot11 = v1x * v1x + v1y * v1y
        dot12 = v1x * v2x + v1y * v2y
        denom = dot00 * dot11 - dot01 * dot01
        if abs(denom) < 1e-9:
            return False
        inv = 1.0 / denom
        u = (dot11 * dot02 - dot01 * dot12) * inv
        v = (dot00 * dot12 - dot01 * dot02) * inv
        eps = 1e-6
        return u >= -eps and v >= -eps and (u + v) <= 1.0 + eps

    def outer_band_triangles(self, polygon, band_width: float) -> list:
        if len(polygon) < 3 or band_width <= 0.0:
            return []
        points = [point(p.x, p.y, 0.0) for p in polygon]
        if distance_xy(points[0], points[-1]) < 0.20:
            points.pop()
        if len(points) < 3:
            return []
        ccw = self.polygon_area(points) > 0.0
        triangles = []
        count = len(points)
        for index in range(count):
            nxt = (index + 1) % count
            current = points[index]
            next_point = points[nxt]
            dx = float(next_point.x) - float(current.x)
            dy = float(next_point.y) - float(current.y)
            length = max(1e-6, math.hypot(dx, dy))
            if ccw:
                nx, ny = dy / length, -dx / length
            else:
                nx, ny = -dy / length, dx / length
            current_outer = point(float(current.x) + nx * band_width, float(current.y) + ny * band_width, 0.0)
            next_outer = point(float(next_point.x) + nx * band_width, float(next_point.y) + ny * band_width, 0.0)
            triangles.extend([current, next_point, next_outer])
            triangles.extend([current, next_outer, current_outer])
        return triangles

    @staticmethod
    def loop_endpoint_gap(points) -> float:
        values = list(points)
        if len(values) < 2:
            return float("inf")
        return distance_xy(values[0], values[-1])

    @staticmethod
    def max_loop_step(points) -> float:
        values = list(points)
        if len(values) < 2:
            return float("inf")
        return max(distance_xy(values[index - 1], values[index]) for index in range(1, len(values)))

    def build_saved_map_markers(self, header) -> MarkerArray:
        saved_map = self.saved_map_for_viz()
        return self.build_track_map_markers(
            header,
            saved_map,
            prefix="saved",
            cone_alpha=0.48,
            cone_diameter=0.30,
            line_alpha=0.58,
            line_width=0.08,
            racing_color=rgba(1.0, 0.0, 0.9, 0.72),
            z_offset=0.10,
            include_seen_cones=False,
        )

    def build_live_map_markers(self, header) -> MarkerArray:
        live_map = self.live_map_for_viz()
        return self.build_track_map_markers(
            header,
            live_map,
            prefix="live",
            cone_alpha=0.95,
            cone_diameter=0.36,
            line_alpha=0.95,
            line_width=0.12,
            racing_color=rgba(0.0, 1.0, 1.0, 0.95),
            z_offset=0.24,
            include_seen_cones=True,
        )

    def build_track_map_markers(
        self,
        header,
        track_map: TrackMap,
        prefix: str,
        cone_alpha: float,
        cone_diameter: float,
        line_alpha: float,
        line_width: float,
        racing_color,
        z_offset: float,
        include_seen_cones: bool,
    ) -> MarkerArray:
        markers = MarkerArray()
        marker_id = 1

        for cone in track_map.cones:
            markers.markers.append(
                sphere_marker(
                    header,
                    marker_id,
                    f"{prefix}_map_cones",
                    cone.position,
                    cone_color(cone.color, cone_alpha),
                    diameter=cone_diameter,
                    z_offset=z_offset,
                )
            )
            marker_id += 1
            if is_start_finish_color(cone.color):
                markers.markers.append(
                    text_marker(
                        header,
                        marker_id,
                        f"{prefix}_start_finish_gate_text",
                        "START/END",
                        cone.position.x,
                        cone.position.y,
                        z_offset + 0.55,
                        rgba(1.0, 0.62, 0.05, cone_alpha),
                    )
                )
                marker_id += 1

        boundaries = self.map_boundary_lines(track_map)
        blue_boundary = self.display_boundary_line(boundaries[ConeColor.BLUE], bool(track_map.closed_loop))
        yellow_boundary = self.display_boundary_line(boundaries[ConeColor.YELLOW], bool(track_map.closed_loop))
        if len(blue_boundary) >= 2:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    f"{prefix}_blue_boundary",
                    blue_boundary,
                    cone_color(ConeColor.BLUE, line_alpha),
                    width=line_width,
                    z_offset=z_offset + 0.05,
                )
            )
            marker_id += 1
        if len(yellow_boundary) >= 2:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    f"{prefix}_yellow_boundary",
                    yellow_boundary,
                    cone_color(ConeColor.YELLOW, line_alpha),
                    width=line_width,
                    z_offset=z_offset + 0.05,
                )
            )
            marker_id += 1
        if track_map.centerline:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    f"{prefix}_centerline",
                    track_map.centerline,
                    rgba(0.0, 0.9, 1.0, line_alpha),
                    width=max(0.05, line_width * 0.75),
                    z_offset=z_offset + 0.02,
                )
            )
            marker_id += 1
        if track_map.racing_line:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    f"{prefix}_racing_line",
                    track_map.racing_line,
                    racing_color,
                    width=max(0.07, line_width),
                    z_offset=z_offset + 0.10,
                )
            )
            marker_id += 1

        if include_seen_cones and self.pose is not None:
            pose2 = pose2_from_pose_stamped(self.pose)
            max_local = int(self.get_parameter("max_local_cones").value)
            for cone in self.local_cones.cones[:max_local]:
                gx, gy = transform_local_to_global(cone.position.x, cone.position.y, pose2)
                markers.markers.append(
                    sphere_marker(
                        header,
                        marker_id,
                        "live_seen_fused_cones",
                        point(gx, gy, 0.0),
                        cone_color(cone.color, 0.72),
                        diameter=0.22,
                        z_offset=z_offset + 0.28,
                    )
                )
                marker_id += 1

        return markers

    def saved_map_for_viz(self) -> TrackMap:
        if self.has_track_map_content(self.saved_track_map):
            return self.saved_track_map
        if self.has_track_map_content(self.reference_racing_map):
            return self.reference_racing_map
        return TrackMap()

    def live_map_for_viz(self) -> TrackMap:
        if self.has_track_map_content(self.live_track_map):
            return self.live_track_map
        if self.has_track_map_content(self.track_map):
            return self.track_map
        if self.has_track_map_content(self.live_racing_map):
            return self.live_racing_map
        return TrackMap()

    @staticmethod
    def has_track_map_content(track_map: TrackMap) -> bool:
        return bool(
            track_map.cones
            or track_map.blue_boundary_line
            or track_map.yellow_boundary_line
            or track_map.centerline
            or track_map.racing_line
        )

    def map_boundary_lines(self, track_map: TrackMap) -> dict[int, list]:
        if track_map.blue_boundary_line or track_map.yellow_boundary_line:
            stored = {
                ConeColor.BLUE: list(track_map.blue_boundary_line),
                ConeColor.YELLOW: list(track_map.yellow_boundary_line),
            }
            if (
                not bool(track_map.closed_loop)
                or not track_map.cones
                or (
                    self.boundary_loop_usable(stored[ConeColor.BLUE])
                    and self.boundary_loop_usable(stored[ConeColor.YELLOW])
                )
            ):
                return stored

        groups = {ConeColor.BLUE: [], ConeColor.YELLOW: []}
        start_finish = []
        for cone in track_map.cones:
            color = int(cone.color)
            cone_point = point(cone.position.x, cone.position.y, 0.0)
            if color == ConeColor.BLUE:
                groups[ConeColor.BLUE].append(cone_point)
            elif color == ConeColor.YELLOW:
                groups[ConeColor.YELLOW].append(cone_point)
            elif is_start_finish_color(color):
                start_finish.append(cone_point)

        for cone_point in start_finish:
            blue_dist = self.nearest_distance(cone_point, groups[ConeColor.BLUE])
            yellow_dist = self.nearest_distance(cone_point, groups[ConeColor.YELLOW])
            if blue_dist <= yellow_dist:
                groups[ConeColor.BLUE].append(cone_point)
            else:
                groups[ConeColor.YELLOW].append(cone_point)

        return {
            ConeColor.BLUE: self.order_boundary_points(groups[ConeColor.BLUE], track_map.centerline),
            ConeColor.YELLOW: self.order_boundary_points(groups[ConeColor.YELLOW], track_map.centerline),
        }

    def order_boundary_points(self, boundary_points, centerline) -> list:
        if len(boundary_points) <= 2:
            return list(boundary_points)
        if len(centerline) < 3:
            return greedy_order_points(boundary_points)

        centerline = list(centerline)
        best_by_station = {}
        for boundary_point in boundary_points:
            best_index = min(
                range(len(centerline)),
                key=lambda index: distance_xy(boundary_point, centerline[index]),
            )
            center_distance = distance_xy(boundary_point, centerline[best_index])
            if center_distance > 8.0:
                continue
            existing = best_by_station.get(best_index)
            if existing is None or center_distance < existing[0]:
                best_by_station[best_index] = (center_distance, boundary_point)
        if len(best_by_station) < 3:
            return greedy_order_points(boundary_points)
        return [boundary_point for _, boundary_point in (best_by_station[index] for index in sorted(best_by_station))]

    @staticmethod
    def nearest_distance(cone_point, boundary_points) -> float:
        if not boundary_points:
            return float("inf")
        return min(distance_xy(cone_point, boundary_point) for boundary_point in boundary_points)

    def get_header(self):
        header = self.track_map.header
        if not header.frame_id:
            header.frame_id = "fsds/map"
        header.stamp = self.get_clock().now().to_msg()
        return header


def main() -> None:
    rclpy.init()
    rclpy.spin(FoxgloveVisualizer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

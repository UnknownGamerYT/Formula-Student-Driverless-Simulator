from __future__ import annotations

import math
from pathlib import Path

import rclpy
from fsds_autonomy_msgs.msg import ConeArray, ObstacleArray, TrackMap
from rclpy.node import Node

from fsds_autonomy.constants import ConeColor, is_reliable_geometry_source
from fsds_autonomy.geometry import point, speed_profile_for_path
from fsds_autonomy.map_store import ConeLandmark, SavedTrackMap, load_reset_events, reset_events_path
from fsds_autonomy.planning import (
    MIN_RACING_CONE_CLEARANCE_M,
    apply_reset_event_avoidance,
    build_boundary_lines_from_cones,
    build_centerline_from_cones,
    build_racing_line,
    infer_map_quality,
    sanitize_racing_line,
)
from fsds_autonomy.ros_utils import track_map_to_msg


class RacelinePlanner(Node):
    def __init__(self) -> None:
        super().__init__("fsds_raceline_planner")
        self.declare_parameter("live_line_min_cone_confidence", 0.50)
        self.declare_parameter("live_line_confidence_lookahead_m", 10.0)
        self.declare_parameter("live_line_min_cones_per_side", 2)
        self.declare_parameter("live_line_reference_blend_enabled", True)
        self.declare_parameter("live_line_reference_blend_max_distance_m", 8.0)
        self.declare_parameter("live_line_min_quality_when_blended", 0.55)
        self.declare_parameter("live_line_min_quality_for_drive", 0.55)
        self.declare_parameter("live_line_min_points_for_drive", 12)
        self.declare_parameter("live_line_min_clearance_for_drive_m", MIN_RACING_CONE_CLEARANCE_M)
        self.declare_parameter("live_line_obstacle_danger_distance_m", 10.0)
        self.declare_parameter("live_line_obstacle_corridor_half_width_m", 1.7)
        self.declare_parameter("drive_centerline_until_closed_loop", True)
        self.declare_parameter("reference_max_cones_for_drive", 500)
        self.declare_parameter("map_dir", "maps")
        self.declare_parameter("reset_event_avoidance_enabled", True)
        self.declare_parameter("reset_event_avoidance_influence_radius_m", 7.0)
        self.declare_parameter("reset_event_avoidance_max_shift_m", 0.80)
        self.declare_parameter("reset_event_avoidance_max_events", 120)

        self.live_line_min_cone_confidence = float(self.get_parameter("live_line_min_cone_confidence").value)
        self.live_line_confidence_lookahead = float(self.get_parameter("live_line_confidence_lookahead_m").value)
        self.live_line_min_cones_per_side = int(self.get_parameter("live_line_min_cones_per_side").value)
        self.live_line_reference_blend_enabled = bool(
            self.get_parameter("live_line_reference_blend_enabled").value
        )
        self.live_line_reference_blend_max_distance = float(
            self.get_parameter("live_line_reference_blend_max_distance_m").value
        )
        self.live_line_min_quality_when_blended = float(
            self.get_parameter("live_line_min_quality_when_blended").value
        )
        self.live_line_min_quality_for_drive = float(
            self.get_parameter("live_line_min_quality_for_drive").value
        )
        self.live_line_min_points_for_drive = int(self.get_parameter("live_line_min_points_for_drive").value)
        self.live_line_min_clearance_for_drive = float(
            self.get_parameter("live_line_min_clearance_for_drive_m").value
        )
        self.live_line_obstacle_danger_distance = float(
            self.get_parameter("live_line_obstacle_danger_distance_m").value
        )
        self.live_line_obstacle_corridor_half_width = float(
            self.get_parameter("live_line_obstacle_corridor_half_width_m").value
        )
        self.drive_centerline_until_closed_loop = bool(
            self.get_parameter("drive_centerline_until_closed_loop").value
        )
        self.reference_max_cones_for_drive = int(self.get_parameter("reference_max_cones_for_drive").value)
        self.map_dir = Path(str(self.get_parameter("map_dir").value)).expanduser()
        self.reset_event_avoidance_enabled = bool(
            self.get_parameter("reset_event_avoidance_enabled").value
        )
        self.reset_event_avoidance_influence_radius = float(
            self.get_parameter("reset_event_avoidance_influence_radius_m").value
        )
        self.reset_event_avoidance_max_shift = float(
            self.get_parameter("reset_event_avoidance_max_shift_m").value
        )
        self.reset_event_avoidance_max_events = int(
            self.get_parameter("reset_event_avoidance_max_events").value
        )

        self.reference_plan: SavedTrackMap | None = None
        self.local_cones = ConeArray()
        self.obstacles = ObstacleArray()
        self.reset_event_cache_track_id = ""
        self.reset_event_cache_mtime = -1.0
        self.reset_event_cache: list[dict] = []

        self.pub = self.create_publisher(TrackMap, "/autonomy/racing_line", 10)
        self.live_pub = self.create_publisher(TrackMap, "/autonomy/live_racing_line", 10)
        self.reference_pub = self.create_publisher(TrackMap, "/autonomy/reference_racing_line", 10)
        self.create_subscription(TrackMap, "/autonomy/track_map", self.on_track_map, 10)
        self.create_subscription(TrackMap, "/autonomy/live_track_map", self.on_live_track_map, 10)
        self.create_subscription(TrackMap, "/autonomy/reference_track_map", self.on_reference_track_map, 10)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_local_cones, 10)
        self.create_subscription(ObstacleArray, "/autonomy/obstacles", self.on_obstacles, 10)

    def on_track_map(self, msg: TrackMap) -> None:
        planned = self.adjust_live_plan(self.plan_from_track_map(msg))
        drive_plan = self.drive_plan(planned)
        self.pub.publish(track_map_to_msg(self, drive_plan))
        self.live_pub.publish(track_map_to_msg(self, planned))

    def on_live_track_map(self, msg: TrackMap) -> None:
        planned = self.adjust_live_plan(self.plan_from_track_map(msg))
        self.live_pub.publish(track_map_to_msg(self, planned))

    def on_reference_track_map(self, msg: TrackMap) -> None:
        planned = self.plan_from_track_map(msg)
        self.reference_plan = planned
        if planned.racing_line and not self.reference_plan_usable(planned):
            self.get_logger().warn(
                f"Reference map kept for visualization but not drive fallback: "
                f"cones={len(planned.cones)} > max={self.reference_max_cones_for_drive}",
                throttle_duration_sec=10.0,
            )
        self.reference_pub.publish(track_map_to_msg(self, planned))

    def drive_plan(self, planned: SavedTrackMap) -> SavedTrackMap:
        reference = self.reference_plan_for_drive()
        drive = planned
        if reference is not None and not self.live_plan_safe_for_drive(planned):
            drive = reference
        elif not planned.racing_line or planned.quality < 0.20:
            drive = reference if reference is not None else planned

        if self.drive_centerline_until_closed_loop and (
            not drive.closed_loop or not self.live_plan_safe_for_drive(drive)
        ):
            center_drive = self.centerline_drive_plan(drive)
            if center_drive is not None:
                return center_drive
        if drive.racing_line and drive.quality >= 0.20:
            return drive
        if reference is not None:
            return reference
        return planned

    def centerline_drive_plan(self, source: SavedTrackMap) -> SavedTrackMap | None:
        if not source.centerline:
            return None
        center_points = [point(x, y, 0.0) for x, y in source.centerline]
        metadata = dict(source.metadata)
        if source.closed_loop:
            raw_center_points = list(center_points)
            candidate_points = sanitize_racing_line(raw_center_points, source.cones)
            candidate_points = apply_reset_event_avoidance(
                candidate_points,
                source.cones,
                self.reset_events_for_planning(source.track_id),
                influence_radius_m=self.reset_event_avoidance_influence_radius,
                max_shift_m=self.reset_event_avoidance_max_shift,
            )
            if self.path_usable_against_centerline(candidate_points, raw_center_points):
                center_points = candidate_points
                metadata["drive_line_source"] = "closed_loop_centerline_with_reset_avoidance"
            else:
                center_points = raw_center_points
                metadata["drive_line_source"] = "closed_loop_raw_centerline_due_discontinuous_sanitized_line"
            center_points = self.closed_loop_points(center_points, max_gap_m=18.0)
        else:
            metadata["drive_line_source"] = "open_loop_raw_centerline"
            metadata["reset_event_avoidance_skipped_until_closed_loop"] = True
        centerline = [(p.x, p.y) for p in center_points]
        return SavedTrackMap(
            track_id=source.track_id,
            closed_loop=bool(source.closed_loop),
            quality=source.quality,
            cones=source.cones,
            blue_boundary_line=source.blue_boundary_line,
            yellow_boundary_line=source.yellow_boundary_line,
            centerline=source.centerline,
            racing_line=centerline,
            speed_profile=speed_profile_for_path(center_points, min_speed=1.2, max_speed=3.0, curvature_gain=24.0),
            metadata=metadata,
        )

    def live_plan_safe_for_drive(self, planned: SavedTrackMap) -> bool:
        if not planned.racing_line or len(planned.racing_line) < self.live_line_min_points_for_drive:
            return False
        if not self.path_usable_against_centerline(planned.racing_line, planned.centerline):
            return False
        if planned.quality < self.live_line_min_quality_for_drive:
            return False
        boundary = [
            cone
            for cone in planned.cones
            if int(cone.color) in (ConeColor.BLUE, ConeColor.YELLOW)
        ]
        if not boundary:
            return False
        for x, y in self.racing_line_safety_samples(planned.racing_line):
            nearest = min(math.hypot(float(x) - cone.x, float(y) - cone.y) for cone in boundary)
            if nearest < self.live_line_min_clearance_for_drive:
                return False
        return True

    def racing_line_safety_samples(self, racing_line: list[tuple[float, float]]) -> list[tuple[float, float]]:
        samples: list[tuple[float, float]] = []
        if not racing_line:
            return samples
        samples.append((float(racing_line[0][0]), float(racing_line[0][1])))
        for start, end in zip(racing_line, racing_line[1:]):
            sx, sy = float(start[0]), float(start[1])
            ex, ey = float(end[0]), float(end[1])
            length = math.hypot(ex - sx, ey - sy)
            steps = max(1, int(math.ceil(length / 0.50)))
            for step in range(1, steps + 1):
                t = step / steps
                samples.append((sx + (ex - sx) * t, sy + (ey - sy) * t))
        return samples

    def reference_plan_for_drive(self) -> SavedTrackMap | None:
        if self.reference_plan is None or not self.reference_plan.racing_line:
            return None
        if not self.reference_plan_usable(self.reference_plan):
            return None
        return self.reference_plan

    def reference_plan_usable(self, reference: SavedTrackMap) -> bool:
        return len(reference.cones) <= max(1, self.reference_max_cones_for_drive)

    def on_local_cones(self, msg: ConeArray) -> None:
        self.local_cones = msg

    def on_obstacles(self, msg: ObstacleArray) -> None:
        self.obstacles = msg

    def plan_from_track_map(self, msg: TrackMap) -> SavedTrackMap:
        cones = [
            ConeLandmark(
                x=cone.position.x,
                y=cone.position.y,
                z=cone.position.z,
                color=cone.color,
                confidence=cone.confidence,
                observations=1,
            )
            for cone in msg.cones
        ]
        blue_boundary = list(msg.blue_boundary_line)
        yellow_boundary = list(msg.yellow_boundary_line)
        if not blue_boundary or not yellow_boundary:
            built_blue, built_yellow = build_boundary_lines_from_cones(cones)
            blue_boundary = blue_boundary or built_blue
            yellow_boundary = yellow_boundary or built_yellow
        centerline = list(msg.centerline) or build_centerline_from_cones(cones)
        reset_events = self.reset_events_for_planning(msg.track_id)
        generated_line, generated_speed_profile = build_racing_line(
            centerline,
            cones,
            reset_events=reset_events,
            reset_event_influence_radius_m=self.reset_event_avoidance_influence_radius,
            reset_event_max_shift_m=self.reset_event_avoidance_max_shift,
        )
        racing_line = generated_line
        speed_profile = generated_speed_profile
        racing_line_source = "generated_width_aware_line"
        incoming_racing_line = list(msg.racing_line)
        if not self.path_usable_against_centerline(racing_line, centerline):
            if self.path_usable_against_centerline(incoming_racing_line, centerline):
                racing_line = incoming_racing_line
                speed_profile = list(msg.speed_profile)
                if len(speed_profile) != len(racing_line):
                    speed_profile = speed_profile_for_path(
                        racing_line,
                        min_speed=2.0,
                        max_speed=5.0,
                        curvature_gain=24.0,
                    )
                racing_line_source = "incoming_track_map_line_due_discontinuous_generated_line"
            else:
                racing_line = list(centerline)
                speed_profile = speed_profile_for_path(
                    racing_line,
                    min_speed=2.0,
                    max_speed=5.0,
                    curvature_gain=24.0,
                )
                racing_line_source = "centerline_due_discontinuous_generated_and_incoming_lines"

        quality = max(float(msg.quality), infer_map_quality(cones, centerline))
        if msg.closed_loop:
            blue_boundary = self.closed_loop_points(blue_boundary, max_gap_m=10.0)
            yellow_boundary = self.closed_loop_points(yellow_boundary, max_gap_m=10.0)
            centerline = self.closed_loop_points(centerline, max_gap_m=18.0)
            racing_line_was = len(racing_line)
            racing_line = self.closed_loop_points(racing_line, max_gap_m=18.0)
            if speed_profile and len(racing_line) > racing_line_was:
                speed_profile = list(speed_profile) + [float(speed_profile[0])]
        planned = SavedTrackMap(
            track_id=msg.track_id,
            closed_loop=msg.closed_loop,
            quality=quality,
            cones=cones,
            blue_boundary_line=[(p.x, p.y) for p in blue_boundary],
            yellow_boundary_line=[(p.x, p.y) for p in yellow_boundary],
            centerline=[(p.x, p.y) for p in centerline],
            racing_line=[(p.x, p.y) for p in racing_line],
            speed_profile=speed_profile,
            metadata={"racing_line_source": racing_line_source},
        )
        return planned

    def reset_events_for_planning(self, track_id: str) -> list[dict]:
        if not self.reset_event_avoidance_enabled or not track_id:
            return []
        path = reset_events_path(self.map_dir, track_id)
        try:
            mtime = path.stat().st_mtime if path.exists() else 0.0
        except OSError:
            mtime = 0.0
        if self.reset_event_cache_track_id == track_id and abs(mtime - self.reset_event_cache_mtime) < 1e-9:
            return self.reset_event_cache
        self.reset_event_cache_track_id = track_id
        self.reset_event_cache_mtime = mtime
        try:
            self.reset_event_cache = load_reset_events(
                self.map_dir,
                track_id,
                max_events=max(1, self.reset_event_avoidance_max_events),
            )
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f"Could not load reset events for line avoidance: {exc}", throttle_duration_sec=5.0)
            self.reset_event_cache = []
        return self.reset_event_cache

    def live_corridor_confidence(self) -> float:
        blue = 0
        yellow = 0
        for cone in self.local_cones.cones:
            if cone.confidence < self.live_line_min_cone_confidence:
                continue
            if not is_reliable_geometry_source(cone.source):
                continue
            if cone.position.x < 0.8 or cone.position.x > self.live_line_confidence_lookahead:
                continue
            if abs(cone.position.y) > 7.0:
                continue
            color = int(cone.color)
            if color == ConeColor.BLUE:
                blue += 1
            elif color == ConeColor.YELLOW:
                yellow += 1

        required = max(1, self.live_line_min_cones_per_side)
        side_score = min(1.0, min(blue, yellow) / required)
        return float(side_score)

    def obstacle_danger_ahead(self) -> bool:
        for obstacle in self.obstacles.obstacles:
            if obstacle.confidence < 0.10:
                continue
            if obstacle.position.x < 0.0 or obstacle.position.x > self.live_line_obstacle_danger_distance:
                continue
            if abs(obstacle.position.y) <= self.live_line_obstacle_corridor_half_width + obstacle.radius:
                return True
        return False

    def adjust_live_plan(self, planned: SavedTrackMap) -> SavedTrackMap:
        reference = self.reference_plan_for_drive()
        if (
            not self.live_line_reference_blend_enabled
            or reference is None
            or not planned.racing_line
            or not reference.racing_line
            or self.obstacle_danger_ahead()
        ):
            return planned

        corridor_confidence = self.live_corridor_confidence()
        live_weight = max(0.0, min(1.0, corridor_confidence))
        if live_weight >= 0.999:
            return planned

        reference_weight = 1.0 - live_weight
        blended_line = []
        for x, y in planned.racing_line:
            ref = self.nearest_reference_point(x, y, reference.racing_line)
            if ref is None:
                blended_line.append((x, y))
                continue
            ref_x, ref_y = ref
            blended_line.append(
                (
                    live_weight * x + reference_weight * ref_x,
                    live_weight * y + reference_weight * ref_y,
                )
            )

        sanitized_line = sanitize_racing_line([point(x, y, 0.0) for x, y in blended_line], planned.cones)
        center_points = [point(x, y, 0.0) for x, y in planned.centerline]
        if not self.path_usable_against_centerline(sanitized_line, center_points):
            planned.metadata["live_reference_blend_rejected"] = "discontinuous_blended_line"
            return planned
        planned.racing_line = [(p.x, p.y) for p in sanitized_line]
        planned.speed_profile = speed_profile_for_path(
            sanitized_line,
            min_speed=2.0,
            max_speed=7.0,
            curvature_gain=22.0,
        )
        planned.quality = min(
            planned.quality,
            self.live_line_min_quality_when_blended
            + (1.0 - self.live_line_min_quality_when_blended) * live_weight,
        )
        return planned

    def nearest_reference_point(
        self,
        x: float,
        y: float,
        reference_line: list[tuple[float, float]],
    ) -> tuple[float, float] | None:
        best = None
        best_dist = self.live_line_reference_blend_max_distance
        for ref_x, ref_y in reference_line:
            dist = math.hypot(x - ref_x, y - ref_y)
            if dist < best_dist:
                best = (ref_x, ref_y)
                best_dist = dist
        return best

    @classmethod
    def path_usable_against_centerline(cls, path, centerline) -> bool:
        path_values = list(path)
        center_values = list(centerline)
        if len(center_values) < 8:
            return len(path_values) >= 8
        if len(path_values) < max(8, int(0.75 * len(center_values))):
            return False
        if cls.path_endpoint_gap(path_values) > max(18.0, cls.path_endpoint_gap(center_values) + 8.0):
            return False
        if cls.path_max_step(path_values) > 18.0:
            return False
        return True

    @classmethod
    def closed_loop_points(cls, points, max_gap_m: float):
        values = list(points)
        if len(values) < 2:
            return values
        gap = cls.path_endpoint_gap(values)
        if 0.05 < gap <= max_gap_m:
            values.append(values[0])
        return values

    @classmethod
    def path_endpoint_gap(cls, points) -> float:
        values = list(points)
        if len(values) < 2:
            return float("inf")
        sx, sy = cls.xy(values[0])
        ex, ey = cls.xy(values[-1])
        return math.hypot(sx - ex, sy - ey)

    @classmethod
    def path_max_step(cls, points) -> float:
        values = list(points)
        if len(values) < 2:
            return float("inf")
        return max(
            math.hypot(cls.xy(values[index])[0] - cls.xy(values[index - 1])[0], cls.xy(values[index])[1] - cls.xy(values[index - 1])[1])
            for index in range(1, len(values))
        )

    @staticmethod
    def xy(value) -> tuple[float, float]:
        if hasattr(value, "x") and hasattr(value, "y"):
            return float(value.x), float(value.y)
        return float(value[0]), float(value[1])


def main() -> None:
    rclpy.init()
    rclpy.spin(RacelinePlanner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

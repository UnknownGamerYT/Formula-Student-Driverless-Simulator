from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
from statistics import median

import rclpy
from fs_msgs.msg import ControlCommand, ExtraInfo
from fs_msgs.srv import Reset
from fsds_autonomy_msgs.msg import ConeArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String
from visualization_msgs.msg import MarkerArray

from fsds_autonomy.geometry import (
    closest_point_on_polyline_with_index,
    nearest_point_index,
    path_segment_count,
    point_at_reference_path_index,
)
from fsds_autonomy.visualization import delete_all_marker, rgba, text_marker, x_marker


class OfftrackResetMonitor(Node):
    def __init__(self) -> None:
        super().__init__("fsds_offtrack_reset_monitor")
        self.declare_parameter("enabled", True)
        self.declare_parameter("map_dir", "maps")
        self.declare_parameter("reset_on_cone_hit", True)
        self.declare_parameter("reset_on_offtrack", False)
        self.declare_parameter("reset_on_extreme_offtrack", True)
        self.declare_parameter("reset_on_stuck", True)
        self.declare_parameter("reset_on_forbidden_area", True)
        self.declare_parameter("reset_event_log_enabled", True)
        self.declare_parameter("reset_event_max_markers", 80)
        self.declare_parameter("reset_marker_size_m", 1.0)
        self.declare_parameter("cone_hit_reset_hold_sec", 0.0)
        self.declare_parameter("stuck_speed_threshold_mps", 0.08)
        self.declare_parameter("stuck_target_speed_threshold_mps", 0.5)
        self.declare_parameter("stuck_hold_sec", 8.0)
        self.declare_parameter("stuck_reset_min_go_sec", 6.0)
        self.declare_parameter("forbidden_area_hold_sec", 0.5)
        self.declare_parameter("forbidden_area_min_quality", 0.45)
        self.declare_parameter("forbidden_boundary_margin_m", 0.70)
        self.declare_parameter("forbidden_min_track_width_m", 2.2)
        self.declare_parameter("forbidden_max_track_width_m", 8.0)
        self.declare_parameter("forbidden_max_center_distance_m", 5.5)
        self.declare_parameter("forbidden_boundary_max_gap_m", 24.0)
        self.declare_parameter("forbidden_boundary_max_step_m", 12.0)
        self.declare_parameter("offtrack_distance_m", 4.5)
        self.declare_parameter("hard_offtrack_distance_m", 12.0)
        self.declare_parameter("extreme_offtrack_distance_m", 12.0)
        self.declare_parameter("extreme_offtrack_hold_sec", 0.5)
        self.declare_parameter("offtrack_hold_sec", 1.5)
        self.declare_parameter("reset_cooldown_sec", 8.0)
        self.declare_parameter("min_path_points", 8)
        self.declare_parameter("corridor_suppression_enabled", True)
        self.declare_parameter("corridor_lookahead_m", 16.0)
        self.declare_parameter("corridor_min_cones_per_side", 1)
        self.declare_parameter("corridor_min_confidence", 0.50)
        self.declare_parameter("corridor_min_width_m", 1.6)
        self.declare_parameter("corridor_max_width_m", 6.5)
        self.declare_parameter("corridor_center_tolerance_m", 2.8)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.map_dir = Path(str(self.get_parameter("map_dir").value)).expanduser()
        self.reset_on_cone_hit = bool(self.get_parameter("reset_on_cone_hit").value)
        self.reset_on_offtrack = bool(self.get_parameter("reset_on_offtrack").value)
        self.reset_on_extreme_offtrack = bool(self.get_parameter("reset_on_extreme_offtrack").value)
        self.reset_on_stuck = bool(self.get_parameter("reset_on_stuck").value)
        self.reset_on_forbidden_area = bool(self.get_parameter("reset_on_forbidden_area").value)
        self.reset_event_log_enabled = bool(self.get_parameter("reset_event_log_enabled").value)
        self.reset_event_max_markers = int(self.get_parameter("reset_event_max_markers").value)
        self.reset_marker_size = float(self.get_parameter("reset_marker_size_m").value)
        self.cone_hit_reset_hold_sec = float(self.get_parameter("cone_hit_reset_hold_sec").value)
        self.stuck_speed_threshold = float(self.get_parameter("stuck_speed_threshold_mps").value)
        self.stuck_target_speed_threshold = float(self.get_parameter("stuck_target_speed_threshold_mps").value)
        self.stuck_hold_sec = float(self.get_parameter("stuck_hold_sec").value)
        self.stuck_reset_min_go_sec = float(self.get_parameter("stuck_reset_min_go_sec").value)
        self.forbidden_area_hold_sec = float(self.get_parameter("forbidden_area_hold_sec").value)
        self.forbidden_area_min_quality = float(self.get_parameter("forbidden_area_min_quality").value)
        self.forbidden_boundary_margin = float(self.get_parameter("forbidden_boundary_margin_m").value)
        self.forbidden_min_track_width = float(self.get_parameter("forbidden_min_track_width_m").value)
        self.forbidden_max_track_width = float(self.get_parameter("forbidden_max_track_width_m").value)
        self.forbidden_max_center_distance = float(self.get_parameter("forbidden_max_center_distance_m").value)
        self.forbidden_boundary_max_gap = float(self.get_parameter("forbidden_boundary_max_gap_m").value)
        self.forbidden_boundary_max_step = float(self.get_parameter("forbidden_boundary_max_step_m").value)
        self.offtrack_distance = float(self.get_parameter("offtrack_distance_m").value)
        self.hard_offtrack_distance = float(self.get_parameter("hard_offtrack_distance_m").value)
        self.extreme_offtrack_distance = float(self.get_parameter("extreme_offtrack_distance_m").value)
        self.extreme_offtrack_hold_sec = float(self.get_parameter("extreme_offtrack_hold_sec").value)
        self.offtrack_hold_sec = float(self.get_parameter("offtrack_hold_sec").value)
        self.reset_cooldown_sec = float(self.get_parameter("reset_cooldown_sec").value)
        self.min_path_points = int(self.get_parameter("min_path_points").value)
        self.corridor_suppression_enabled = bool(self.get_parameter("corridor_suppression_enabled").value)
        self.corridor_lookahead = float(self.get_parameter("corridor_lookahead_m").value)
        self.corridor_min_cones_per_side = int(self.get_parameter("corridor_min_cones_per_side").value)
        self.corridor_min_confidence = float(self.get_parameter("corridor_min_confidence").value)
        self.corridor_min_width = float(self.get_parameter("corridor_min_width_m").value)
        self.corridor_max_width = float(self.get_parameter("corridor_max_width_m").value)
        self.corridor_center_tolerance = float(self.get_parameter("corridor_center_tolerance_m").value)

        self.pose = None
        self.track = TrackMap()
        self.live_track_map = TrackMap()
        self.reference_track_map = TrackMap()
        self.local_cones = ConeArray()
        self.race_state = RaceState()
        self.speed: float | None = None
        self.target_speed: float | None = None
        self.extra_info: ExtraInfo | None = None
        self.baseline_doo_counter: int | None = None
        self.pending_cone_hits = 0
        self.cone_hit_since = None
        self.stuck_since = None
        self.forbidden_area_since = None
        self.extreme_offtrack_since = None
        self.go_signal_since = None
        self.offtrack_since = None
        self.last_reset_time = None
        self.reset_in_flight = False
        self.reset_events: list[dict] = []
        self.reset_events_track_id = ""

        self.reset_client = self.create_client(Reset, "/fsds/reset")
        self.brake_pub = self.create_publisher(ControlCommand, "/fsds/control_command", 10)
        self.status_pub = self.create_publisher(String, "/autonomy/offtrack_reset_status", 10)
        self.reset_marker_pub = self.create_publisher(MarkerArray, "/autonomy/viz/reset_markers", 10)

        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)
        self.create_subscription(TrackMap, "/autonomy/racing_line", self.on_track, 10)
        self.create_subscription(TrackMap, "/autonomy/track_map", self.on_live_track_map, 10)
        self.create_subscription(TrackMap, "/autonomy/reference_track_map", self.on_reference_track_map, 10)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_cones, 10)
        self.create_subscription(RaceState, "/autonomy/race_state", self.on_race_state, 10)
        self.create_subscription(Float32, "/autonomy/speed", self.on_speed, 10)
        self.create_subscription(Float32, "/autonomy/target_speed", self.on_target_speed, 10)
        self.create_subscription(ExtraInfo, "/fsds/testing_only/extra_info", self.on_extra_info, 10)
        self.create_timer(0.1, self.tick)

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def on_track(self, msg: TrackMap) -> None:
        self.track = msg

    def on_live_track_map(self, msg: TrackMap) -> None:
        self.live_track_map = msg

    def on_reference_track_map(self, msg: TrackMap) -> None:
        self.reference_track_map = msg

    def on_cones(self, msg: ConeArray) -> None:
        self.local_cones = msg

    def on_race_state(self, msg: RaceState) -> None:
        self.race_state = msg
        if msg.track_id and msg.track_id != self.reset_events_track_id:
            self.load_reset_events(msg.track_id)

    def on_speed(self, msg: Float32) -> None:
        self.speed = float(msg.data)

    def on_target_speed(self, msg: Float32) -> None:
        self.target_speed = float(msg.data)

    def on_extra_info(self, msg: ExtraInfo) -> None:
        self.extra_info = msg
        current = int(msg.doo_counter)
        if self.baseline_doo_counter is None or current < self.baseline_doo_counter:
            self.baseline_doo_counter = current
            self.pending_cone_hits = 0
            self.cone_hit_since = None
            return

        new_hits = current - self.baseline_doo_counter
        if new_hits <= 0:
            self.pending_cone_hits = 0
            self.cone_hit_since = None
            return

        self.pending_cone_hits = new_hits
        if self.cone_hit_since is None:
            self.cone_hit_since = self.get_clock().now()

    def usable_path(self):
        if not bool(self.track.closed_loop) and len(self.track.centerline) >= self.min_path_points:
            return self.track.centerline
        if len(self.track.racing_line) >= self.min_path_points:
            return self.track.racing_line
        if len(self.track.centerline) >= self.min_path_points:
            return self.track.centerline
        return []

    def distance_to_path(self) -> float | None:
        if self.pose is None:
            return None
        path = self.usable_path()
        if not path:
            return None
        pos = self.pose.pose.position
        return min(math.hypot(pos.x - p.x, pos.y - p.y) for p in path)

    def visible_corridor_status(self) -> tuple[bool, str]:
        if not self.corridor_suppression_enabled:
            return False, "corridor=disabled"

        left = []
        right = []
        for cone in self.local_cones.cones:
            if cone.confidence < self.corridor_min_confidence:
                continue
            if cone.position.x < 0.5 or cone.position.x > self.corridor_lookahead:
                continue
            if cone.position.y > 0.30:
                left.append(float(cone.position.y))
            elif cone.position.y < -0.30:
                right.append(float(cone.position.y))

        if len(left) < self.corridor_min_cones_per_side or len(right) < self.corridor_min_cones_per_side:
            return False, f"corridor=weak left={len(left)} right={len(right)}"

        width = median(left) - median(right)
        center = 0.5 * (median(left) + median(right))
        ok = (
            self.corridor_min_width <= width <= self.corridor_max_width
            and abs(center) <= self.corridor_center_tolerance
        )
        return ok, f"corridor={'ok' if ok else 'bad'} left={len(left)} right={len(right)} width={width:.2f} center={center:.2f}"

    def cooldown_active(self) -> bool:
        if self.last_reset_time is None:
            return False
        return (self.get_clock().now() - self.last_reset_time).nanoseconds * 1e-9 < self.reset_cooldown_sec

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_brake(self) -> None:
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "fsds/FSCar"
        cmd.throttle = 0.0
        cmd.steering = 0.0
        cmd.brake = 1.0
        self.brake_pub.publish(cmd)

    def request_reset(self, reason: str) -> bool:
        if self.cooldown_active() or self.reset_in_flight:
            return False

        if not self.reset_client.service_is_ready():
            self.reset_client.wait_for_service(timeout_sec=0.05)
        if not self.reset_client.service_is_ready():
            self.get_logger().warn("/fsds/reset is not available yet", throttle_duration_sec=2.0)
            return False

        self.record_reset_event(reason)
        request = Reset.Request()
        request.wait_on_last_task = False
        future = self.reset_client.call_async(request)
        future.add_done_callback(self.on_reset_done)
        self.reset_in_flight = True
        self.last_reset_time = self.get_clock().now()
        self.get_logger().warn(f"Automatic reset requested: {reason}")
        return True

    def tick_cone_hit_reset(self) -> bool:
        if self.extra_info is None or self.baseline_doo_counter is None:
            self.publish_status("armed_cone_hit waiting_for_extra_info")
            return True

        current = int(self.extra_info.doo_counter)
        if self.pending_cone_hits <= 0:
            self.publish_status(f"armed_cone_hit doo={current}")
            return True

        now = self.get_clock().now()
        hit_for = 0.0
        if self.cone_hit_since is not None:
            hit_for = (now - self.cone_hit_since).nanoseconds * 1e-9

        self.publish_brake()
        self.publish_status(
            f"cone_hit doo={current} new_hits={self.pending_cone_hits} hold={hit_for:.1f}s"
        )
        if hit_for < self.cone_hit_reset_hold_sec or self.cooldown_active() or self.reset_in_flight:
            return True

        if self.request_reset(f"cone_hit doo={current} new_hits={self.pending_cone_hits}"):
            self.baseline_doo_counter = current
            self.pending_cone_hits = 0
            self.cone_hit_since = None
        return True

    def target_speed_for_stuck_check(self) -> float:
        if self.target_speed is not None:
            return float(self.target_speed)
        return float(self.race_state.target_speed)

    def tick_stuck_reset(self) -> bool:
        if not self.reset_on_stuck:
            self.stuck_since = None
            return False

        if self.speed is None:
            self.stuck_since = None
            return False

        now = self.get_clock().now()
        if self.go_signal_since is None:
            self.go_signal_since = now

        go_for = (now - self.go_signal_since).nanoseconds * 1e-9
        target_speed = self.target_speed_for_stuck_check()
        stuck_candidate = (
            abs(float(self.speed)) <= self.stuck_speed_threshold
            and target_speed >= self.stuck_target_speed_threshold
            and not bool(self.race_state.emergency_brake)
        )

        if not stuck_candidate or go_for < self.stuck_reset_min_go_sec:
            self.stuck_since = None
            return False

        if self.stuck_since is None:
            self.stuck_since = now

        stuck_for = (now - self.stuck_since).nanoseconds * 1e-9
        self.publish_status(
            f"stuck_watch speed={self.speed:.2f}mps target={target_speed:.2f}mps "
            f"hold={stuck_for:.1f}s"
        )
        if stuck_for < self.stuck_hold_sec or self.cooldown_active() or self.reset_in_flight:
            return True

        self.publish_brake()
        reason = (
            f"stuck speed={self.speed:.2f}mps target={target_speed:.2f}mps "
            f"hold={stuck_for:.1f}s"
        )
        if self.request_reset(reason):
            self.stuck_since = None
        return True

    def tick_extreme_offtrack_reset(self) -> bool:
        if not self.reset_on_extreme_offtrack:
            self.extreme_offtrack_since = None
            return False

        distance = self.distance_to_path()
        if distance is None or distance < self.extreme_offtrack_distance:
            self.extreme_offtrack_since = None
            return False

        loop_map = self.boundary_reset_map()
        if loop_map is not None:
            legal, status = self.forbidden_area_status(loop_map)
            if legal:
                self.extreme_offtrack_since = None
                self.publish_status(f"extreme_off_track_suppressed distance={distance:.2f}m {status}")
                return False

        corridor_ok, corridor_status = self.visible_corridor_status()
        if corridor_ok:
            self.extreme_offtrack_since = None
            self.publish_status(f"extreme_off_track_suppressed distance={distance:.2f}m {corridor_status}")
            return False

        now = self.get_clock().now()
        if self.extreme_offtrack_since is None:
            self.extreme_offtrack_since = now

        offtrack_for = (now - self.extreme_offtrack_since).nanoseconds * 1e-9
        self.publish_brake()
        self.publish_status(
            f"extreme_off_track distance={distance:.2f}m threshold={self.extreme_offtrack_distance:.2f}m "
            f"hold={offtrack_for:.1f}s {corridor_status}"
        )
        if offtrack_for < self.extreme_offtrack_hold_sec or self.cooldown_active() or self.reset_in_flight:
            return True

        if self.request_reset(f"extreme_off_track distance={distance:.2f}m hold={offtrack_for:.1f}s"):
            self.extreme_offtrack_since = None
        return True

    def boundary_reset_map(self) -> TrackMap | None:
        candidates = [self.live_track_map, self.reference_track_map, self.track]
        usable = [
            track_map
            for track_map in candidates
            if self.full_loop_map_usable(track_map)
        ]
        if not usable:
            return None
        return max(
            usable,
            key=lambda track_map: (
                float(track_map.quality),
                len(track_map.blue_boundary_line) + len(track_map.yellow_boundary_line),
            ),
        )

    def full_loop_map_usable(self, track_map: TrackMap) -> bool:
        if not bool(track_map.closed_loop):
            return False
        if float(track_map.quality) < self.forbidden_area_min_quality:
            return False
        checks = [
            (track_map.blue_boundary_line, 12.0, 10.0),
            (track_map.yellow_boundary_line, 12.0, 10.0),
            (track_map.centerline, 18.0, 18.0),
            (track_map.racing_line, 18.0, 18.0),
        ]
        for points, max_endpoint_gap, max_internal_step in checks:
            if len(points) < self.min_path_points:
                return False
            if self.loop_endpoint_gap(points) > max_endpoint_gap:
                return False
            if self.max_loop_step(points) > max_internal_step:
                return False
        return True

    @staticmethod
    def nearest_point(points, x: float, y: float):
        if not points:
            return None
        return min(points, key=lambda p: (float(p.x) - x) ** 2 + (float(p.y) - y) ** 2)

    def boundary_cross_section(self, track_map: TrackMap, x: float, y: float):
        closed_loop = bool(track_map.closed_loop)
        projection = closest_point_on_polyline_with_index(track_map.centerline, x, y, closed_loop=closed_loop)
        if projection is None:
            return None
        center, _, center_distance, center_index = projection
        center_segments = path_segment_count(track_map.centerline, closed_loop=closed_loop)
        if center_segments <= 0:
            return None
        fraction = center_index / float(center_segments)
        yellow = point_at_reference_path_index(
            track_map.yellow_boundary_line,
            center_index,
            center_segments,
            closed_loop=closed_loop,
        )
        blue = point_at_reference_path_index(
            track_map.blue_boundary_line,
            center_index,
            center_segments,
            closed_loop=closed_loop,
        )
        if yellow is None or blue is None:
            return None
        return center, yellow, blue, center_distance, fraction

    def boundary_loop_usable(self, points) -> bool:
        values = list(points)
        if len(values) < 3:
            return False
        steps = [
            math.hypot(
                float(values[index].x) - float(values[index - 1].x),
                float(values[index].y) - float(values[index - 1].y),
            )
            for index in range(1, len(values))
        ]
        if not steps:
            return False
        step_median = median(steps)
        close_gap = math.hypot(
            float(values[0].x) - float(values[-1].x),
            float(values[0].y) - float(values[-1].y),
        )
        max_close_gap = max(self.forbidden_boundary_max_gap, 3.5 * step_median)
        max_internal_step = max(self.forbidden_boundary_max_step, 3.0 * step_median)
        return close_gap <= max_close_gap and max(steps) <= max_internal_step

    @staticmethod
    def loop_endpoint_gap(points) -> float:
        values = list(points)
        if len(values) < 2:
            return float("inf")
        return math.hypot(float(values[0].x) - float(values[-1].x), float(values[0].y) - float(values[-1].y))

    @staticmethod
    def max_loop_step(points) -> float:
        values = list(points)
        if len(values) < 2:
            return float("inf")
        return max(
            math.hypot(
                float(values[index].x) - float(values[index - 1].x),
                float(values[index].y) - float(values[index - 1].y),
            )
            for index in range(1, len(values))
        )

    def forbidden_area_status(self, track_map: TrackMap) -> tuple[bool, str]:
        if self.pose is None:
            return True, "forbidden_area=waiting_for_pose"

        pos = self.pose.pose.position
        cross_section = self.boundary_cross_section(track_map, pos.x, pos.y)
        if cross_section is None:
            return True, "forbidden_area=waiting_for_centerline"
        center, yellow, blue, center_distance, fraction = cross_section

        vx = float(blue.x) - float(yellow.x)
        vy = float(blue.y) - float(yellow.y)
        width = math.hypot(vx, vy)
        if width < self.forbidden_min_track_width or width > self.forbidden_max_track_width:
            return True, f"forbidden_area=unreliable_width width={width:.2f}m"

        max_center_distance = max(self.forbidden_max_center_distance, 0.5 * width + self.forbidden_boundary_margin)
        if center_distance > max_center_distance:
            return False, (
                f"forbidden_area=outside_loop center_distance={center_distance:.2f}m "
                f"width={width:.2f}m"
            )

        wx = float(pos.x) - float(yellow.x)
        wy = float(pos.y) - float(yellow.y)
        lateral_fraction = (wx * vx + wy * vy) / max(1e-6, width * width)
        yellow_clearance = lateral_fraction * width
        blue_clearance = (1.0 - lateral_fraction) * width
        margin = min(self.forbidden_boundary_margin, max(0.0, 0.45 * width))
        if yellow_clearance < margin:
            return False, (
                f"forbidden_area=yellow_side clearance={yellow_clearance:.2f}m "
                f"margin={margin:.2f}m width={width:.2f}m progress={fraction:.3f}"
            )
        if blue_clearance < margin:
            return False, (
                f"forbidden_area=blue_side clearance={blue_clearance:.2f}m "
                f"margin={margin:.2f}m width={width:.2f}m progress={fraction:.3f}"
            )
        return True, (
            f"forbidden_area=clear yellow_clearance={yellow_clearance:.2f}m "
            f"blue_clearance={blue_clearance:.2f}m width={width:.2f}m progress={fraction:.3f}"
        )

    def tick_forbidden_area_reset(self) -> bool:
        if not self.reset_on_forbidden_area:
            self.forbidden_area_since = None
            return False

        track_map = self.boundary_reset_map()
        if track_map is None:
            self.forbidden_area_since = None
            return False

        legal, status = self.forbidden_area_status(track_map)
        if legal:
            self.forbidden_area_since = None
            return False

        now = self.get_clock().now()
        if self.forbidden_area_since is None:
            self.forbidden_area_since = now

        outside_for = (now - self.forbidden_area_since).nanoseconds * 1e-9
        self.publish_status(f"{status} hold={outside_for:.1f}s")
        if outside_for < self.forbidden_area_hold_sec or self.cooldown_active() or self.reset_in_flight:
            return True

        self.publish_brake()
        if self.request_reset(f"{status} hold={outside_for:.1f}s"):
            self.forbidden_area_since = None
        return True

    def tick(self) -> None:
        self.publish_reset_markers()
        if not self.enabled:
            self.publish_status("disabled")
            return
        if not self.race_state.go_signal_fresh:
            self.offtrack_since = None
            self.cone_hit_since = None
            self.stuck_since = None
            self.forbidden_area_since = None
            self.extreme_offtrack_since = None
            self.go_signal_since = None
            self.publish_status("waiting_for_go")
            return

        if self.go_signal_since is None:
            self.go_signal_since = self.get_clock().now()

        if self.reset_on_cone_hit and self.pending_cone_hits > 0:
            self.tick_cone_hit_reset()
            return

        if self.tick_stuck_reset():
            return

        if self.tick_extreme_offtrack_reset():
            return

        if self.tick_forbidden_area_reset():
            return

        if not self.reset_on_offtrack:
            self.offtrack_since = None
            if self.reset_on_cone_hit:
                self.tick_cone_hit_reset()
            else:
                self.publish_status("armed_no_reset_trigger")
            return

        distance = self.distance_to_path()
        if distance is None:
            self.offtrack_since = None
            self.publish_status("waiting_for_path")
            return

        corridor_ok, corridor_status = self.visible_corridor_status()
        if distance <= self.offtrack_distance:
            self.offtrack_since = None
            self.publish_status(f"on_track distance={distance:.2f}m {corridor_status}")
            return
        if corridor_ok and distance < self.hard_offtrack_distance:
            self.offtrack_since = None
            self.publish_status(f"on_track_corridor distance={distance:.2f}m {corridor_status}")
            return

        now = self.get_clock().now()
        if self.offtrack_since is None:
            self.offtrack_since = now
        self.publish_brake()

        offtrack_for = (now - self.offtrack_since).nanoseconds * 1e-9
        self.publish_status(f"off_track distance={distance:.2f}m hold={offtrack_for:.1f}s {corridor_status}")
        if offtrack_for < self.offtrack_hold_sec or self.cooldown_active() or self.reset_in_flight:
            return

        if self.request_reset(f"off_track distance={distance:.2f}m"):
            self.offtrack_since = None

    def on_reset_done(self, future) -> None:
        self.reset_in_flight = False
        try:
            future.result()
            self.publish_status("reset_complete")
            self.get_logger().info("Automatic reset complete")
        except Exception as exc:
            self.publish_status(f"reset_failed {exc}")
            self.get_logger().error(f"Off-track reset failed: {exc}")

    def reset_events_path(self, track_id: str) -> Path:
        safe_track = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in track_id.strip()) or "unknown"
        return self.map_dir / f"{safe_track}.reset_events.json"

    def load_reset_events(self, track_id: str) -> None:
        self.reset_events_track_id = track_id
        self.reset_events = []
        if not self.reset_event_log_enabled:
            return
        path = self.reset_events_path(track_id)
        if not path.exists():
            return
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Could not load reset event markers from {path}: {exc}")
            return
        if isinstance(data, list):
            self.reset_events = [item for item in data if isinstance(item, dict)]

    def save_reset_events(self) -> None:
        if not self.reset_event_log_enabled:
            return
        track_id = self.race_state.track_id or self.reset_events_track_id or "unknown"
        path = self.reset_events_path(track_id)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.reset_events, indent=2, sort_keys=True), encoding="utf-8")

    def record_reset_event(self, reason: str) -> None:
        if not self.reset_event_log_enabled or self.pose is None:
            return
        track_id = self.race_state.track_id or self.reset_events_track_id or "unknown"
        if track_id != self.reset_events_track_id:
            self.load_reset_events(track_id)
        pos = self.pose.pose.position
        event = {
            "time_utc": datetime.now(timezone.utc).isoformat(),
            "track_id": track_id,
            "x": float(pos.x),
            "y": float(pos.y),
            "reason": str(reason),
            "mode": str(self.race_state.mode),
            "behavior": str(self.race_state.behavior_state),
            "doo_counter": int(self.extra_info.doo_counter) if self.extra_info is not None else None,
        }
        event.update(self.reset_event_context(float(pos.x), float(pos.y)))
        self.reset_events.append(event)
        self.save_reset_events()
        self.get_logger().warn(
            f"Recorded reset marker x={event['x']:.2f} y={event['y']:.2f} reason={reason}"
        )

    def reset_event_context(self, x: float, y: float) -> dict:
        context: dict = {}
        path = self.usable_path()
        if path:
            nearest_index = nearest_point_index(path, x, y)
            if nearest_index >= 0:
                nearest = path[nearest_index]
                context["nearest_line_x"] = float(nearest.x)
                context["nearest_line_y"] = float(nearest.y)
                context["nearest_line_distance_m"] = math.hypot(x - float(nearest.x), y - float(nearest.y))
                tangent = self.path_tangent(path, nearest_index)
                if tangent is not None:
                    tx, ty = tangent
                    normal_x = -ty
                    normal_y = tx
                    context["line_lateral_error_m"] = (
                        (x - float(nearest.x)) * normal_x + (y - float(nearest.y)) * normal_y
                    )
        if self.track.cones:
            nearest_cone = min(
                self.track.cones,
                key=lambda cone: (float(cone.position.x) - x) ** 2 + (float(cone.position.y) - y) ** 2,
            )
            context["nearest_cone_x"] = float(nearest_cone.position.x)
            context["nearest_cone_y"] = float(nearest_cone.position.y)
            context["nearest_cone_color"] = int(nearest_cone.color)
            context["nearest_cone_distance_m"] = math.hypot(
                x - float(nearest_cone.position.x),
                y - float(nearest_cone.position.y),
            )
        return context

    @staticmethod
    def path_tangent(path, index: int) -> tuple[float, float] | None:
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

    def publish_reset_markers(self) -> None:
        header = self.track.header
        if not header.frame_id:
            header.frame_id = "fsds/map"
        header.stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(delete_all_marker(header))
        marker_id = 1
        max_markers = max(1, self.reset_event_max_markers)
        for event in self.reset_events[-max_markers:]:
            x = float(event.get("x", 0.0))
            y = float(event.get("y", 0.0))
            reason = str(event.get("reason", "reset"))
            color = rgba(1.0, 0.05, 0.05, 0.95) if "cone_hit" in reason else rgba(1.0, 0.55, 0.0, 0.95)
            markers.markers.append(
                x_marker(
                    header,
                    marker_id,
                    "reset_event_x",
                    x,
                    y,
                    color,
                    size=self.reset_marker_size,
                    width=0.14,
                    z_offset=0.42,
                )
            )
            marker_id += 1
            markers.markers.append(
                text_marker(
                    header,
                    marker_id,
                    "reset_event_text",
                    reason.split()[0],
                    x,
                    y,
                    0.95,
                    color,
                )
            )
            marker_id += 1
        self.reset_marker_pub.publish(markers)


def main() -> None:
    rclpy.init()
    rclpy.spin(OfftrackResetMonitor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

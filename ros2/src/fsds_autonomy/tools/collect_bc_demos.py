#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np


BASE_OBSERVATION_NAMES = [
    "speed_mps",
    "target_speed_mps",
    "progress_fraction",
    "episode_progress_fraction",
    "sector_fraction",
    "visited_sector_fraction",
    "centerline_error_m",
    "racing_line_error_m",
    "min_clearance_m",
    "yellow_clearance_m",
    "blue_clearance_m",
    "track_width_m",
    "previous_control_steering",
    "previous_control_throttle",
    "previous_control_brake",
    "track_quality",
    "track_closed_loop",
    "race_state_map_loaded",
    "race_state_go_signal_fresh",
    "stage_fraction",
    "direct_blend",
    "sin_yaw",
    "cos_yaw",
    "previous_action_path_offset",
    "previous_action_speed_delta",
    "previous_action_direct_steering",
    "previous_action_direct_throttle",
    "previous_action_direct_brake",
    "centerline_x_m",
    "centerline_y_m",
    "racing_line_x_m",
    "racing_line_y_m",
    "yellow_line_x_m",
    "yellow_line_y_m",
    "blue_line_x_m",
    "blue_line_y_m",
]

TRACK_SHAPE_OBSERVATION_NAMES = [
    "center_heading_error_4m",
    "center_heading_error_10m",
    "racing_heading_error_4m",
    "racing_heading_error_10m",
    "center_curvature_4m",
    "center_curvature_10m",
    "racing_curvature_4m",
    "racing_curvature_10m",
    "track_width_4m",
    "track_center_y_4m",
    "track_width_10m",
    "track_center_y_10m",
    "danger_zone_x_m",
    "danger_zone_y_m",
    "danger_zone_strength",
    "danger_zone_count_norm",
]

ACTION_NAMES = [
    "residual_path_offset",
    "residual_speed_delta",
    "direct_steering",
    "direct_throttle",
    "direct_brake",
]


@dataclass
class Pose2:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class Point3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class ControlSnapshot:
    steering: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0


@dataclass
class Bounds:
    max_path_offset_m: float = 0.60
    max_speed_delta_mps: float = 1.20
    max_direct_steering: float = 1.0
    max_direct_throttle: float = 1.0
    max_direct_brake: float = 1.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def pose2_from_pose_stamped(msg: Any | None) -> Pose2 | None:
    if msg is None:
        return None
    return Pose2(
        x=float(msg.pose.position.x),
        y=float(msg.pose.position.y),
        yaw=yaw_from_quaternion(msg.pose.orientation),
    )


def point3_from_msg(msg: Any) -> Point3:
    return Point3(float(msg.x), float(msg.y), float(getattr(msg, "z", 0.0)))


def distance_xy(a: Any, b: Any) -> float:
    return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))


def nearest_point_index(points: list[Any], x: float, y: float) -> int:
    if not points:
        return -1
    best_index = 0
    best_dist = float("inf")
    for index, point in enumerate(points):
        dist = (float(point.x) - x) ** 2 + (float(point.y) - y) ** 2
        if dist < best_dist:
            best_dist = dist
            best_index = index
    return best_index


def closed_path_has_duplicate_endpoint(points: list[Any]) -> bool:
    return len(points) >= 2 and distance_xy(points[0], points[-1]) <= 1e-6


def path_segment_count(points: list[Any], closed_loop: bool = False) -> int:
    if len(points) < 2:
        return 0
    if closed_loop and len(points) > 2 and not closed_path_has_duplicate_endpoint(points):
        return len(points)
    return len(points) - 1


def path_segments(points: list[Any], closed_loop: bool = False) -> Iterable[tuple[int, Any, Any]]:
    for index in range(1, len(points)):
        yield index - 1, points[index - 1], points[index]
    if closed_loop and len(points) > 2 and not closed_path_has_duplicate_endpoint(points):
        yield len(points) - 1, points[-1], points[0]


def closest_point_on_polyline_with_index(
    points: list[Any],
    x: float,
    y: float,
    closed_loop: bool = False,
) -> tuple[Point3, float, float, float] | None:
    if not points:
        return None
    if len(points) == 1:
        p = point3_from_msg(points[0])
        return p, 0.0, math.hypot(p.x - x, p.y - y), 0.0

    best_point: Point3 | None = None
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
            best_point = Point3(
                px,
                py,
                float(start.z) + ratio * (float(end.z) - float(start.z)),
            )
            best_station = station + ratio * segment_length
            best_distance = distance
            best_index = float(segment_index) + ratio
        station += segment_length

    if best_point is None:
        nearest = points[nearest_point_index(points, x, y)]
        best_point = point3_from_msg(nearest)
        best_distance = math.hypot(best_point.x - x, best_point.y - y)
    return best_point, best_station, best_distance, best_index


def point_at_path_index(points: list[Any], index_position: float, closed_loop: bool = False) -> Point3 | None:
    if not points:
        return None
    if len(points) == 1:
        return point3_from_msg(points[0])

    segment_count = path_segment_count(points, closed_loop)
    if segment_count <= 0:
        return point3_from_msg(points[0])
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
    return Point3(
        float(start.x) + ratio * (float(end.x) - float(start.x)),
        float(start.y) + ratio * (float(end.y) - float(start.y)),
        float(start.z) + ratio * (float(end.z) - float(start.z)),
    )


def point_at_reference_path_index(
    points: list[Any],
    reference_index: float,
    reference_segment_count: int,
    closed_loop: bool = False,
) -> Point3 | None:
    if reference_segment_count <= 0:
        return point_at_path_index(points, 0.0, closed_loop=closed_loop)
    segment_count = path_segment_count(points, closed_loop)
    if segment_count <= 0:
        return point_at_path_index(points, 0.0, closed_loop=closed_loop)
    target_index = reference_index * float(segment_count) / float(reference_segment_count)
    return point_at_path_index(points, target_index, closed_loop=closed_loop)


def transform_global_to_local(global_x: float, global_y: float, pose: Pose2) -> tuple[float, float]:
    dx = global_x - pose.x
    dy = global_y - pose.y
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def arc_lengths_for(path: list[Any]) -> list[float]:
    if not path:
        return []
    lengths = [0.0]
    for index in range(1, len(path)):
        lengths.append(lengths[-1] + distance_xy(path[index - 1], path[index]))
    return lengths


def point_at_s(path: list[Any], arc_lengths: list[float], station: float, closed_loop: bool) -> Point3:
    if not path:
        return Point3()
    if not arc_lengths:
        return point3_from_msg(path[0])
    if closed_loop and arc_lengths[-1] > 1e-6:
        station = station % arc_lengths[-1]
    else:
        station = clamp(station, 0.0, arc_lengths[-1])
    for index in range(1, len(arc_lengths)):
        if arc_lengths[index] < station:
            continue
        prev_s = arc_lengths[index - 1]
        next_s = arc_lengths[index]
        span = max(1e-6, next_s - prev_s)
        ratio = clamp((station - prev_s) / span, 0.0, 1.0)
        prev_p = path[index - 1]
        next_p = path[index]
        return Point3(
            float(prev_p.x) + (float(next_p.x) - float(prev_p.x)) * ratio,
            float(prev_p.y) + (float(next_p.y) - float(prev_p.y)) * ratio,
            float(prev_p.z) + (float(next_p.z) - float(prev_p.z)) * ratio,
        )
    return point3_from_msg(path[-1])


def parse_bool_action(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


class BCDemoCollector:
    def __init__(self, args: argparse.Namespace) -> None:
        from fs_msgs.msg import ControlCommand
        from fsds_autonomy_msgs.msg import ConeArray, RaceState, TrackMap
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from std_msgs.msg import Float32, String

        class _Node(Node):
            pass

        self.args = args
        self.ControlCommand = ControlCommand
        self.ConeArray = ConeArray
        self.RaceState = RaceState
        self.TrackMap = TrackMap
        self.PoseStamped = PoseStamped
        self.Float32 = Float32
        self.String = String
        self.node = _Node("fsds_bc_demo_collector")

        self.bounds = Bounds(
            max_path_offset_m=max(1e-6, float(args.max_path_offset_m)),
            max_speed_delta_mps=max(1e-6, float(args.max_speed_delta_mps)),
            max_direct_steering=max(1e-6, float(args.max_direct_steering)),
            max_direct_throttle=max(1e-6, float(args.max_direct_throttle)),
            max_direct_brake=max(1e-6, float(args.max_direct_brake)),
        )

        self.run_id = args.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.out_dir = args.out_dir.expanduser()
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.map_dir = args.map_dir.expanduser()
        self.jsonl_path = self.out_dir / f"bc_demo_{self.run_id}.jsonl"
        self.npz_path = self.out_dir / f"bc_demo_{self.run_id}.npz"
        self.jsonl_file = self.jsonl_path.open("a", encoding="utf-8") if args.jsonl else None

        self.pose: Any | None = None
        self.speed = 0.0
        self.have_speed = False
        self.target_speed = 0.0
        self.have_target_speed = False
        self.path_offset = 0.0
        self.have_path_offset = False
        self.race_state: Any | None = None
        self.track: Any | None = None
        self.local_cones: Any | None = None
        self.offtrack_status = ""
        self.reset_events_track_id = ""
        self.reset_events_mtime: float | None = None
        self.last_danger_refresh = 0.0
        self.danger_points: list[tuple[float, float, float]] = []

        self.centerline_points: list[Any] = []
        self.racing_points: list[Any] = []
        self.yellow_boundary_points: list[Any] = []
        self.blue_boundary_points: list[Any] = []
        self.path_points: list[Any] = []
        self.center_arc_lengths: list[float] = []
        self.racing_arc_lengths: list[float] = []
        self.yellow_arc_lengths: list[float] = []
        self.blue_arc_lengths: list[float] = []
        self.arc_lengths: list[float] = []
        self.path_length = 0.0
        self.path_signature: tuple[Any, ...] | None = None

        self.prev_progress_s: float | None = None
        self.episode_progress = 0.0
        self.prev_sector: int | None = None
        self.visited_sectors: set[int] = set()
        self.last_control = ControlSnapshot()
        self.last_action = np.zeros(5, dtype=np.float32)
        self.last_sample_time: float | None = None
        self.start_time = self.now_sec()
        self.sample_count = 0
        self.skipped_count = 0
        self.done = False

        self.buffer_observations: list[np.ndarray] = []
        self.buffer_actions: list[np.ndarray] = []
        self.buffer_times: list[float] = []
        self.buffer_commands: list[np.ndarray] = []
        self.npz_buffer_full = False

        self.write_metadata()

        self.node.create_subscription(ControlCommand, args.control_topic, self.on_control, args.control_qos)
        self.node.create_subscription(Float32, args.speed_topic, self.on_speed, 20)
        self.node.create_subscription(Float32, args.target_speed_topic, self.on_target_speed, 20)
        self.node.create_subscription(Float32, args.path_offset_topic, self.on_path_offset, 20)
        self.node.create_subscription(PoseStamped, args.pose_topic, self.on_pose, 20)
        self.node.create_subscription(RaceState, args.race_state_topic, self.on_race_state, 20)
        self.node.create_subscription(TrackMap, args.track_topic, self.on_track, 20)
        self.node.create_subscription(ConeArray, args.cones_topic, self.on_cones, 20)
        self.node.create_subscription(String, args.offtrack_status_topic, self.on_offtrack_status, 20)
        self.node.create_timer(0.5, self.check_limits)
        self.node.get_logger().warn(
            f"BC demo collector writing JSONL={self.jsonl_path if args.jsonl else 'disabled'} "
            f"NPZ={self.npz_path if args.npz else 'disabled'}"
        )

    def now_sec(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def write_metadata(self) -> None:
        payload = {
            "event": "metadata",
            "schema": "fsds_bc_demo_v1",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "run_id": self.run_id,
            "observation_layout": "live_full_control_v1",
            "base_observation_names": BASE_OBSERVATION_NAMES,
            "track_shape_observation_names": TRACK_SHAPE_OBSERVATION_NAMES,
            "action_layout": "live_full_control_sac_v1",
            "action_names": ACTION_NAMES,
            "action_bounds": {
                "low": [-1.0] * 5,
                "high": [1.0] * 5,
                "controller_inverse": {
                    "direct_steering": "steering / max_direct_steering",
                    "direct_throttle": "2 * throttle / max_direct_throttle - 1",
                    "direct_brake": "2 * brake / max_direct_brake - 1",
                },
            },
            "cone_count": int(self.args.cone_count),
            "preview_count": int(self.args.preview_count),
            "sector_count": int(self.args.sector_count),
            "danger_zone_enabled": bool(self.args.danger_zone_enabled),
            "danger_zone_radius_m": float(self.args.danger_zone_radius_m),
            "stage": int(self.args.stage),
            "residual_label_mode": str(self.args.residual_label_mode),
            "topics": {
                "control": self.args.control_topic,
                "pose": self.args.pose_topic,
                "speed": self.args.speed_topic,
                "target_speed": self.args.target_speed_topic,
                "path_offset": self.args.path_offset_topic,
                "race_state": self.args.race_state_topic,
                "track": self.args.track_topic,
                "cones": self.args.cones_topic,
            },
        }
        self.write_json(payload, flush=True)

    def on_speed(self, msg: Any) -> None:
        self.speed = float(msg.data)
        self.have_speed = True

    def on_target_speed(self, msg: Any) -> None:
        self.target_speed = float(msg.data)
        self.have_target_speed = True

    def on_path_offset(self, msg: Any) -> None:
        self.path_offset = float(msg.data)
        self.have_path_offset = True

    def on_pose(self, msg: Any) -> None:
        self.pose = msg

    def on_race_state(self, msg: Any) -> None:
        self.race_state = msg
        self.refresh_danger_points(str(msg.track_id or ""))

    def on_track(self, msg: Any) -> None:
        self.track = msg
        signature = self.track_signature(msg)
        if signature != self.path_signature:
            self.path_signature = signature
            self.rebuild_path_cache()

    @staticmethod
    def path_coordinate_signature(points: list[Any]) -> tuple[tuple[float, float], ...]:
        if not points:
            return ()
        step = max(1, len(points) // 8)
        samples = list(points[::step][:8])
        if samples[-1] is not points[-1]:
            samples.append(points[-1])
        return tuple((round(float(point.x), 2), round(float(point.y), 2)) for point in samples)

    def track_signature(self, msg: Any) -> tuple[Any, ...]:
        return (
            len(msg.centerline),
            len(msg.racing_line),
            len(msg.blue_boundary_line),
            len(msg.yellow_boundary_line),
            bool(msg.closed_loop),
            str(msg.track_id),
            round(float(msg.quality), 3),
            self.path_coordinate_signature(list(msg.centerline)),
            self.path_coordinate_signature(list(msg.racing_line)),
            self.path_coordinate_signature(list(msg.blue_boundary_line)),
            self.path_coordinate_signature(list(msg.yellow_boundary_line)),
        )

    def reset_events_path(self, track_id: str) -> Path:
        safe_track = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in track_id.strip()) or "unknown"
        return self.map_dir / f"{safe_track}.reset_events.json"

    def refresh_danger_points(self, track_id: str, *, force: bool = False) -> None:
        if not bool(self.args.danger_zone_enabled):
            return
        if not track_id and self.track is not None:
            track_id = str(self.track.track_id or "")
        track_id = track_id.strip()
        if not track_id:
            return
        now = self.now_sec()
        if not force and now - self.last_danger_refresh < 1.0:
            return
        self.last_danger_refresh = now
        path = self.reset_events_path(track_id)
        try:
            mtime = path.stat().st_mtime
        except OSError:
            if track_id != self.reset_events_track_id or self.reset_events_mtime is not None:
                self.reset_events_track_id = track_id
                self.reset_events_mtime = None
                self.danger_points = []
            return
        if track_id == self.reset_events_track_id and self.reset_events_mtime == mtime:
            return
        self.load_danger_points(track_id, path, mtime)

    def load_danger_points(self, track_id: str, path: Path | None = None, mtime: float | None = None) -> None:
        self.reset_events_track_id = track_id
        self.reset_events_mtime = mtime
        self.danger_points = []
        path = path or self.reset_events_path(track_id)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.node.get_logger().warn(f"Could not load BC danger zones from {path}: {exc}")
            return
        if not isinstance(data, list):
            return
        danger_reasons = (
            "cone_hit",
            "off_track",
            "extreme_off_track",
            "forbidden_area",
            "outside_closed_corridor",
            "stuck",
        )
        points: list[tuple[float, float, float]] = []
        for item in data:
            if not isinstance(item, dict):
                continue
            reason = str(item.get("reason") or "").lower()
            if not any(reason.startswith(prefix) or prefix in reason for prefix in danger_reasons):
                continue
            try:
                x = float(item.get("x"))
                y = float(item.get("y"))
            except (TypeError, ValueError):
                continue
            if not math.isfinite(x) or not math.isfinite(y) or math.hypot(x, y) < 1.0:
                continue
            weight = 1.0
            if "stuck" in reason:
                weight = 0.50
            elif "forbidden_area" in reason or "off_track" in reason:
                weight = 0.80
            points.append((x, y, weight))
        self.danger_points = points[-160:]

    def on_cones(self, msg: Any) -> None:
        self.local_cones = msg

    def on_offtrack_status(self, msg: Any) -> None:
        self.offtrack_status = str(msg.data)

    def on_control(self, msg: Any) -> None:
        now = self.now_sec()
        if self.done:
            return
        if not self.should_sample(now):
            self.last_control = ControlSnapshot(float(msg.steering), float(msg.throttle), float(msg.brake))
            return
        if not self.ready_to_sample():
            self.skipped_count += 1
            self.last_control = ControlSnapshot(float(msg.steering), float(msg.throttle), float(msg.brake))
            return
        if self.args.skip_emergency_brake and self.race_state is not None and bool(self.race_state.emergency_brake):
            self.skipped_count += 1
            self.last_control = ControlSnapshot(float(msg.steering), float(msg.throttle), float(msg.brake))
            return
        if abs(float(self.speed)) < float(self.args.min_speed_mps):
            self.skipped_count += 1
            self.last_control = ControlSnapshot(float(msg.steering), float(msg.throttle), float(msg.brake))
            return

        self.update_progress_state()
        observation = self.observation()
        action = self.command_to_action(msg)
        command = np.asarray(
            [float(msg.steering), float(msg.throttle), float(msg.brake)],
            dtype=np.float32,
        )

        self.sample_count += 1
        self.last_sample_time = now
        self.write_sample(now, observation, action, command)
        self.maybe_buffer_npz(now, observation, action, command)
        self.last_action = action
        self.last_control = ControlSnapshot(float(msg.steering), float(msg.throttle), float(msg.brake))
        self.check_limits()

    def should_sample(self, now: float) -> bool:
        if now - self.start_time < max(0.0, float(self.args.warmup_sec)):
            return False
        sample_hz = float(self.args.sample_hz)
        if sample_hz <= 0.0:
            return True
        min_dt = 1.0 / sample_hz
        return self.last_sample_time is None or now - self.last_sample_time >= min_dt

    def ready_to_sample(self) -> bool:
        if not self.args.require_ready:
            return self.pose is not None
        if self.pose is None or self.race_state is None or self.track is None:
            return False
        if self.args.require_speed and not self.have_speed:
            return False
        if self.args.require_target_speed and not self.have_target_speed:
            return False
        if self.args.require_go_signal and not bool(self.race_state.go_signal_fresh):
            return False
        if self.args.require_closed_loop and not bool(self.track.closed_loop):
            return False
        if float(self.track.quality) < float(self.args.min_track_quality):
            return False
        if len(self.path_points) < int(self.args.min_path_points):
            return False
        return True

    def command_to_action(self, msg: Any) -> np.ndarray:
        residual_path = 0.0
        residual_speed = 0.0
        if self.args.residual_label_mode == "path-speed":
            residual_path = self.path_offset / self.bounds.max_path_offset_m if self.have_path_offset else 0.0
            residual_speed = (self.target_speed - self.speed) / self.bounds.max_speed_delta_mps

        steering = float(msg.steering) / self.bounds.max_direct_steering
        throttle = 2.0 * clamp(float(msg.throttle), 0.0, self.bounds.max_direct_throttle) / self.bounds.max_direct_throttle - 1.0
        brake = 2.0 * clamp(float(msg.brake), 0.0, self.bounds.max_direct_brake) / self.bounds.max_direct_brake - 1.0
        return np.clip(
            np.asarray([residual_path, residual_speed, steering, throttle, brake], dtype=np.float32),
            -1.0,
            1.0,
        )

    def rebuild_path_cache(self) -> None:
        if self.track is None:
            return
        self.centerline_points = list(self.track.centerline)
        self.racing_points = list(self.track.racing_line)
        self.yellow_boundary_points = list(self.track.yellow_boundary_line)
        self.blue_boundary_points = list(self.track.blue_boundary_line)
        self.path_points = self.centerline_points or self.racing_points
        self.center_arc_lengths = arc_lengths_for(self.centerline_points)
        self.racing_arc_lengths = arc_lengths_for(self.racing_points)
        self.yellow_arc_lengths = arc_lengths_for(self.yellow_boundary_points)
        self.blue_arc_lengths = arc_lengths_for(self.blue_boundary_points)
        self.arc_lengths = arc_lengths_for(self.path_points)
        self.path_length = self.arc_lengths[-1] if self.arc_lengths else 0.0
        self.prev_progress_s = None
        self.episode_progress = 0.0
        self.prev_sector = None
        self.visited_sectors.clear()

    def progress_s(self) -> float | None:
        pose = pose2_from_pose_stamped(self.pose)
        if pose is None or not self.path_points or not self.arc_lengths:
            return None
        index = nearest_point_index(self.path_points, pose.x, pose.y)
        if index < 0:
            return None
        return self.arc_lengths[min(index, len(self.arc_lengths) - 1)]

    def current_sector(self) -> int | None:
        progress = self.progress_s()
        if progress is None or self.path_length <= 1.0:
            return None
        return int((progress / self.path_length) * int(self.args.sector_count)) % int(self.args.sector_count)

    def update_progress_state(self) -> None:
        current = self.progress_s()
        current_sector = self.current_sector()
        if current_sector is not None:
            self.visited_sectors.add(current_sector)
        if current is None or self.path_length <= 1.0:
            self.prev_progress_s = current
            return
        if self.prev_progress_s is None:
            self.prev_progress_s = current
            self.prev_sector = current_sector
            return
        delta = current - self.prev_progress_s
        if self.track is not None and bool(self.track.closed_loop):
            if delta < -0.5 * self.path_length:
                delta += self.path_length
                if self.args.reset_episode_on_lap_wrap:
                    self.episode_progress = 0.0
                    self.visited_sectors.clear()
            elif delta > 0.5 * self.path_length:
                delta -= self.path_length
        delta = float(np.clip(delta, -5.0, 8.0))
        self.episode_progress += max(0.0, delta)
        self.prev_progress_s = current
        self.prev_sector = current_sector

    def path_error(self, path: list[Any]) -> float:
        pose = pose2_from_pose_stamped(self.pose)
        if pose is None or not path:
            return 0.0
        index = nearest_point_index(path, pose.x, pose.y)
        if index < 0:
            return 0.0
        return math.hypot(pose.x - float(path[index].x), pose.y - float(path[index].y))

    def boundary_clearance(self) -> dict[str, float] | None:
        pose = pose2_from_pose_stamped(self.pose)
        if (
            pose is None
            or self.track is None
            or not self.centerline_points
            or not self.blue_boundary_points
            or not self.yellow_boundary_points
        ):
            return None
        closed_loop = bool(self.track.closed_loop)
        projection = closest_point_on_polyline_with_index(self.centerline_points, pose.x, pose.y, closed_loop=closed_loop)
        center_segments = path_segment_count(self.centerline_points, closed_loop=closed_loop)
        if projection is None or center_segments <= 0:
            return None
        _, _, center_distance, center_index = projection
        blue = point_at_reference_path_index(
            self.blue_boundary_points,
            center_index,
            center_segments,
            closed_loop=closed_loop,
        )
        yellow = point_at_reference_path_index(
            self.yellow_boundary_points,
            center_index,
            center_segments,
            closed_loop=closed_loop,
        )
        if blue is None or yellow is None:
            return None
        vx = blue.x - yellow.x
        vy = blue.y - yellow.y
        width = math.hypot(vx, vy)
        if width < 1e-6:
            return None
        wx = pose.x - yellow.x
        wy = pose.y - yellow.y
        fraction = (wx * vx + wy * vy) / (width * width)
        yellow_clearance = fraction * width
        blue_clearance = (1.0 - fraction) * width
        return {
            "yellow_clearance_m": yellow_clearance,
            "blue_clearance_m": blue_clearance,
            "min_clearance_m": min(yellow_clearance, blue_clearance),
            "track_width_m": width,
            "center_distance_m": center_distance,
            "progress_fraction": center_index / float(center_segments),
        }

    def local_line_offsets(self) -> dict[str, float]:
        center_x, center_y = self.local_offset_to_nearest(self.centerline_points)
        racing_x, racing_y = self.local_offset_to_nearest(self.racing_points)
        yellow_x, yellow_y = self.local_offset_to_nearest(self.yellow_boundary_points)
        blue_x, blue_y = self.local_offset_to_nearest(self.blue_boundary_points)
        return {
            "centerline_x_m": center_x,
            "centerline_y_m": center_y,
            "racing_line_x_m": racing_x,
            "racing_line_y_m": racing_y,
            "yellow_line_x_m": yellow_x,
            "yellow_line_y_m": yellow_y,
            "blue_line_x_m": blue_x,
            "blue_line_y_m": blue_y,
        }

    def local_offset_to_nearest(self, path: list[Any]) -> tuple[float, float]:
        pose = pose2_from_pose_stamped(self.pose)
        if pose is None or not path:
            return 0.0, 0.0
        index = nearest_point_index(path, pose.x, pose.y)
        if index < 0:
            return 0.0, 0.0
        return transform_global_to_local(float(path[index].x), float(path[index].y), pose)

    def cone_features(self) -> np.ndarray:
        cone_count = int(self.args.cone_count)
        cone_feature_len = 8
        features = np.zeros(cone_count * cone_feature_len, dtype=np.float32)
        if self.local_cones is None:
            return features
        cones = [
            cone
            for cone in self.local_cones.cones
            if float(cone.position.x) > 0.0 and abs(float(cone.position.y)) < 12.0
        ]
        cones.sort(
            key=lambda cone: (
                float(cone.range)
                if float(cone.range) > 0.0
                else math.hypot(float(cone.position.x), float(cone.position.y))
            )
        )
        for index, cone in enumerate(cones[:cone_count]):
            color = int(cone.color)
            start = index * cone_feature_len
            features[start : start + cone_feature_len] = np.asarray(
                [
                    float(cone.position.x),
                    float(cone.position.y),
                    float(cone.range),
                    float(cone.confidence),
                    1.0 if color == 0 else 0.0,
                    1.0 if color == 1 else 0.0,
                    1.0 if color in (2, 3) else 0.0,
                    1.0 if color == 4 else 0.0,
                ],
                dtype=np.float32,
            )
        return features

    def preview_features(self, path: list[Any], arc_lengths: list[float]) -> np.ndarray:
        preview_count = int(self.args.preview_count)
        features = np.zeros(preview_count * 2, dtype=np.float32)
        pose = pose2_from_pose_stamped(self.pose)
        if pose is None or not path or not arc_lengths or self.path_length <= 1.0:
            return features
        current = self.progress_s() or 0.0
        closed_loop = bool(self.track.closed_loop) if self.track is not None else False
        for index in range(preview_count):
            lookahead = 2.0 + 2.0 * index
            point = point_at_s(path, arc_lengths, current + lookahead, closed_loop=closed_loop)
            local_x, local_y = transform_global_to_local(point.x, point.y, pose)
            start = index * 2
            features[start] = float(local_x)
            features[start + 1] = float(local_y)
        return features

    def track_shape_features(self) -> np.ndarray:
        features = np.zeros(len(TRACK_SHAPE_OBSERVATION_NAMES), dtype=np.float32)
        pose = pose2_from_pose_stamped(self.pose)
        if pose is None or self.path_length <= 1.0:
            features[12:16] = self.danger_zone_features()
            return features
        station = self.progress_s() or 0.0
        width_4, center_y_4 = self.boundary_width_center_y_ahead(station, 4.0, pose)
        width_10, center_y_10 = self.boundary_width_center_y_ahead(station, 10.0, pose)
        features[:] = np.asarray(
            [
                self.path_heading_error_ahead(self.centerline_points, self.center_arc_lengths, station, 4.0, pose),
                self.path_heading_error_ahead(self.centerline_points, self.center_arc_lengths, station, 10.0, pose),
                self.path_heading_error_ahead(self.racing_points, self.racing_arc_lengths, station, 4.0, pose),
                self.path_heading_error_ahead(self.racing_points, self.racing_arc_lengths, station, 10.0, pose),
                self.path_curvature_ahead(self.centerline_points, self.center_arc_lengths, station, 4.0),
                self.path_curvature_ahead(self.centerline_points, self.center_arc_lengths, station, 10.0),
                self.path_curvature_ahead(self.racing_points, self.racing_arc_lengths, station, 4.0),
                self.path_curvature_ahead(self.racing_points, self.racing_arc_lengths, station, 10.0),
                width_4,
                center_y_4,
                width_10,
                center_y_10,
                *self.danger_zone_features().tolist(),
            ],
            dtype=np.float32,
        )
        return features

    def path_heading_error_ahead(
        self,
        path: list[Any],
        arc_lengths: list[float],
        station: float,
        lookahead: float,
        pose: Pose2,
    ) -> float:
        if len(path) < 2 or len(arc_lengths) < 2:
            return 0.0
        closed_loop = bool(self.track.closed_loop) if self.track is not None else False
        p0 = point_at_s(path, arc_lengths, station + lookahead, closed_loop=closed_loop)
        p1 = point_at_s(path, arc_lengths, station + lookahead + 2.0, closed_loop=closed_loop)
        heading = math.atan2(p1.y - p0.y, p1.x - p0.x)
        return float(normalize_angle(heading - pose.yaw))

    def path_curvature_ahead(self, path: list[Any], arc_lengths: list[float], station: float, lookahead: float) -> float:
        if len(path) < 3 or len(arc_lengths) < 3:
            return 0.0
        closed_loop = bool(self.track.closed_loop) if self.track is not None else False
        before = max(0.0, lookahead - 2.0)
        p0 = point_at_s(path, arc_lengths, station + before, closed_loop=closed_loop)
        p1 = point_at_s(path, arc_lengths, station + lookahead, closed_loop=closed_loop)
        p2 = point_at_s(path, arc_lengths, station + lookahead + 2.0, closed_loop=closed_loop)
        d01 = distance_xy(p0, p1)
        d12 = distance_xy(p1, p2)
        if d01 < 1e-3 or d12 < 1e-3:
            return 0.0
        h1 = math.atan2(p1.y - p0.y, p1.x - p0.x)
        h2 = math.atan2(p2.y - p1.y, p2.x - p1.x)
        curvature = normalize_angle(h2 - h1) / max(1.0, d01 + d12)
        return float(clamp(curvature, -1.0, 1.0))

    def boundary_width_center_y_ahead(self, station: float, lookahead: float, pose: Pose2) -> tuple[float, float]:
        if (
            len(self.blue_boundary_points) < 2
            or len(self.yellow_boundary_points) < 2
            or not self.blue_arc_lengths
            or not self.yellow_arc_lengths
            or self.path_length <= 1.0
        ):
            return 0.0, 0.0
        ahead_station = station + lookahead
        if self.track is not None and bool(self.track.closed_loop):
            progress_fraction = (ahead_station % self.path_length) / self.path_length
        else:
            progress_fraction = clamp(ahead_station / self.path_length, 0.0, 1.0)
        closed_loop = bool(self.track.closed_loop) if self.track is not None else False
        blue = point_at_s(
            self.blue_boundary_points,
            self.blue_arc_lengths,
            progress_fraction * self.blue_arc_lengths[-1],
            closed_loop=closed_loop,
        )
        yellow = point_at_s(
            self.yellow_boundary_points,
            self.yellow_arc_lengths,
            progress_fraction * self.yellow_arc_lengths[-1],
            closed_loop=closed_loop,
        )
        width = distance_xy(blue, yellow)
        center_x = 0.5 * (blue.x + yellow.x)
        center_y = 0.5 * (blue.y + yellow.y)
        _, local_center_y = transform_global_to_local(center_x, center_y, pose)
        return float(width), float(local_center_y)

    def danger_zone_features(self) -> np.ndarray:
        features = np.zeros(4, dtype=np.float32)
        pose = pose2_from_pose_stamped(self.pose)
        if not bool(self.args.danger_zone_enabled) or pose is None or not self.danger_points:
            return features
        radius = max(0.1, float(self.args.danger_zone_radius_m))
        nearest: tuple[float, float, float, float] | None = None
        for x, y, weight in self.danger_points:
            distance = math.hypot(pose.x - x, pose.y - y)
            if nearest is None or distance < nearest[0]:
                nearest = (distance, x, y, weight)
        if nearest is None:
            return features
        distance, x, y, weight = nearest
        local_x, local_y = transform_global_to_local(x, y, pose)
        strength = clamp(1.0 - distance / radius, 0.0, 1.0) * weight
        features[:] = np.asarray(
            [
                clamp(float(local_x), -30.0, 30.0),
                clamp(float(local_y), -30.0, 30.0),
                clamp(strength, 0.0, 1.0),
                min(1.0, len(self.danger_points) / 80.0),
            ],
            dtype=np.float32,
        )
        return features

    def stage_direct_blend(self) -> float:
        stage = int(self.args.stage)
        if stage < 3:
            return 0.0
        if stage == 3:
            return clamp(float(self.args.stage3_direct_blend), 0.0, 1.0)
        return clamp(float(self.args.stage4_direct_blend), 0.0, 1.0)

    def observation(self) -> np.ndarray:
        if self.race_state is not None or self.track is not None:
            track_id = str(getattr(self.race_state, "track_id", "") or getattr(self.track, "track_id", "") or "")
            self.refresh_danger_points(track_id)
        pose = pose2_from_pose_stamped(self.pose)
        yaw = pose.yaw if pose is not None else 0.0
        progress = self.progress_s() or 0.0
        progress_norm = progress / max(1.0, self.path_length)
        sector = self.current_sector()
        sector_count = max(1, int(self.args.sector_count))
        sector_norm = float(sector if sector is not None else 0) / max(1, sector_count - 1)
        center_error = self.path_error(self.centerline_points)
        racing_error = self.path_error(self.racing_points)
        clear = self.boundary_clearance() or {}
        line_offsets = self.local_line_offsets()
        track_quality = float(self.track.quality) if self.track is not None else 0.0
        track_closed = bool(self.track.closed_loop) if self.track is not None else False
        map_loaded = bool(self.race_state.map_loaded) if self.race_state is not None else False
        go_signal = bool(self.race_state.go_signal_fresh) if self.race_state is not None else False
        base_values = np.asarray(
            [
                self.speed,
                self.target_speed,
                progress_norm,
                self.episode_progress / max(1.0, self.path_length),
                sector_norm,
                len(self.visited_sectors) / max(1.0, float(sector_count)),
                center_error,
                racing_error,
                float(clear.get("min_clearance_m", 0.0)),
                float(clear.get("yellow_clearance_m", 0.0)),
                float(clear.get("blue_clearance_m", 0.0)),
                float(clear.get("track_width_m", 0.0)),
                self.last_control.steering,
                self.last_control.throttle,
                self.last_control.brake,
                track_quality,
                1.0 if track_closed else 0.0,
                1.0 if map_loaded else 0.0,
                1.0 if go_signal else 0.0,
                int(self.args.stage) / 4.0,
                self.stage_direct_blend(),
                math.sin(yaw),
                math.cos(yaw),
                float(self.last_action[0]),
                float(self.last_action[1]),
                float(self.last_action[2]),
                float(self.last_action[3]),
                float(self.last_action[4]),
                line_offsets["centerline_x_m"],
                line_offsets["centerline_y_m"],
                line_offsets["racing_line_x_m"],
                line_offsets["racing_line_y_m"],
                line_offsets["yellow_line_x_m"],
                line_offsets["yellow_line_y_m"],
                line_offsets["blue_line_x_m"],
                line_offsets["blue_line_y_m"],
            ],
            dtype=np.float32,
        )
        base = np.concatenate([base_values, self.track_shape_features()]).astype(np.float32)
        return np.concatenate(
            [
                base,
                self.cone_features(),
                self.preview_features(self.centerline_points, self.center_arc_lengths),
                self.preview_features(self.racing_points, self.racing_arc_lengths),
                self.preview_features(self.yellow_boundary_points, self.yellow_arc_lengths),
                self.preview_features(self.blue_boundary_points, self.blue_arc_lengths),
            ]
        ).astype(np.float32)

    def write_sample(self, now: float, observation: np.ndarray, action: np.ndarray, command: np.ndarray) -> None:
        pose = pose2_from_pose_stamped(self.pose)
        clear = self.boundary_clearance() or {}
        payload = {
            "event": "sample",
            "schema": "fsds_bc_demo_v1",
            "run_id": self.run_id,
            "seq": self.sample_count,
            "time_sec": now,
            "observation": [float(value) for value in observation.tolist()],
            "action": [float(value) for value in action.tolist()],
            "raw_command": {
                "steering": float(command[0]),
                "throttle": float(command[1]),
                "brake": float(command[2]),
            },
            "context": {
                "speed_mps": float(self.speed),
                "target_speed_mps": float(self.target_speed),
                "path_offset_m": float(self.path_offset) if self.have_path_offset else None,
                "pose": {
                    "x": pose.x,
                    "y": pose.y,
                    "yaw_rad": pose.yaw,
                }
                if pose is not None
                else None,
                "track": {
                    "track_id": str(self.track.track_id) if self.track is not None else "",
                    "closed_loop": bool(self.track.closed_loop) if self.track is not None else False,
                    "quality": float(self.track.quality) if self.track is not None else 0.0,
                    "centerline_points": len(self.centerline_points),
                    "racing_line_points": len(self.racing_points),
                    "blue_boundary_points": len(self.blue_boundary_points),
                    "yellow_boundary_points": len(self.yellow_boundary_points),
                },
                "race_state": {
                    "mode": str(self.race_state.mode) if self.race_state is not None else "",
                    "behavior_state": str(self.race_state.behavior_state) if self.race_state is not None else "",
                    "map_loaded": bool(self.race_state.map_loaded) if self.race_state is not None else False,
                    "go_signal_fresh": bool(self.race_state.go_signal_fresh) if self.race_state is not None else False,
                    "emergency_brake": bool(self.race_state.emergency_brake) if self.race_state is not None else False,
                },
                "clearance": {key: float(value) for key, value in clear.items()},
                "offtrack_status": self.offtrack_status,
            },
        }
        self.write_json(payload, flush=self.sample_count % int(self.args.flush_every_n) == 0)

    def maybe_buffer_npz(
        self,
        now: float,
        observation: np.ndarray,
        action: np.ndarray,
        command: np.ndarray,
    ) -> None:
        if not self.args.npz or self.npz_buffer_full:
            return
        limit = int(self.args.npz_max_samples)
        if limit > 0 and len(self.buffer_observations) >= limit:
            self.npz_buffer_full = True
            self.node.get_logger().warn(
                f"NPZ buffer limit reached at {limit} samples; continuing JSONL only"
            )
            return
        self.buffer_observations.append(observation.copy())
        self.buffer_actions.append(action.copy())
        self.buffer_times.append(float(now))
        self.buffer_commands.append(command.copy())

    def write_json(self, payload: dict[str, Any], flush: bool = False) -> None:
        if self.jsonl_file is None:
            return
        self.jsonl_file.write(json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n")
        if flush:
            self.jsonl_file.flush()

    def check_limits(self) -> None:
        if self.done:
            return
        if int(self.args.max_samples) > 0 and self.sample_count >= int(self.args.max_samples):
            self.done = True
        max_duration = float(self.args.max_duration_sec)
        if max_duration > 0.0 and self.now_sec() - self.start_time >= max_duration:
            self.done = True
        if self.done:
            self.node.get_logger().warn(
                f"BC demo collection stopping samples={self.sample_count} skipped={self.skipped_count}"
            )

    def finalize(self) -> None:
        if self.args.npz and self.buffer_observations:
            metadata = {
                "schema": "fsds_bc_demo_npz_v1",
                "run_id": self.run_id,
                "source_jsonl": str(self.jsonl_path) if self.args.jsonl else "",
                "observation_layout": "live_full_control_v1",
                "action_layout": "live_full_control_sac_v1",
                "base_observation_names": BASE_OBSERVATION_NAMES,
                "track_shape_observation_names": TRACK_SHAPE_OBSERVATION_NAMES,
                "action_names": ACTION_NAMES,
                "cone_count": int(self.args.cone_count),
                "preview_count": int(self.args.preview_count),
                "sector_count": int(self.args.sector_count),
                "danger_zone_enabled": bool(self.args.danger_zone_enabled),
                "danger_zone_radius_m": float(self.args.danger_zone_radius_m),
                "sample_count": len(self.buffer_observations),
                "npz_buffer_full": bool(self.npz_buffer_full),
            }
            np.savez_compressed(
                self.npz_path,
                observations=np.asarray(self.buffer_observations, dtype=np.float32),
                actions=np.asarray(self.buffer_actions, dtype=np.float32),
                times=np.asarray(self.buffer_times, dtype=np.float64),
                raw_commands=np.asarray(self.buffer_commands, dtype=np.float32),
                metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
            )
        summary = {
            "event": "summary",
            "schema": "fsds_bc_demo_v1",
            "run_id": self.run_id,
            "sample_count": int(self.sample_count),
            "skipped_count": int(self.skipped_count),
            "jsonl_path": str(self.jsonl_path) if self.args.jsonl else "",
            "npz_path": str(self.npz_path) if self.args.npz and self.buffer_observations else "",
            "npz_buffer_full": bool(self.npz_buffer_full),
        }
        self.write_json(summary, flush=True)
        if self.jsonl_file is not None:
            self.jsonl_file.flush()
            self.jsonl_file.close()
            self.jsonl_file = None
        self.node.get_logger().warn(
            f"BC demo collection complete samples={self.sample_count} skipped={self.skipped_count} "
            f"jsonl={self.jsonl_path if self.args.jsonl else 'disabled'} "
            f"npz={self.npz_path if self.args.npz and self.buffer_observations else 'disabled'}"
        )

    def destroy_node(self) -> None:
        self.node.destroy_node()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Collect bounded behavior-cloning demonstrations from live FSDS ROS2 topics. "
            "Samples match fsds_autonomy.rl.live_env.LiveFullControlEnv observations and 5D SAC actions."
        )
    )
    parser.add_argument("--out-dir", type=Path, default=Path("~/.fsds_autonomy/bc_demos"))
    parser.add_argument("--map-dir", type=Path, default=Path("~/.fsds_autonomy/maps"))
    parser.add_argument("--run-id", default="")
    parser.add_argument("--jsonl", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--npz", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--npz-max-samples", type=int, default=200_000)
    parser.add_argument("--flush-every-n", type=int, default=25)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--warmup-sec", type=float, default=1.0)
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--max-duration-sec", type=float, default=0.0)
    parser.add_argument("--control-topic", default="/fsds/control_command")
    parser.add_argument("--pose-topic", default="/autonomy/pose")
    parser.add_argument("--speed-topic", default="/autonomy/speed")
    parser.add_argument("--target-speed-topic", default="/autonomy/target_speed")
    parser.add_argument("--path-offset-topic", default="/autonomy/path_offset")
    parser.add_argument("--race-state-topic", default="/autonomy/race_state")
    parser.add_argument("--track-topic", default="/autonomy/racing_line")
    parser.add_argument("--cones-topic", default="/autonomy/fused_cones")
    parser.add_argument("--offtrack-status-topic", default="/autonomy/offtrack_reset_status")
    parser.add_argument("--control-qos", type=int, default=100)
    parser.add_argument("--require-ready", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-speed", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-target-speed", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-go-signal", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-closed-loop", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--min-track-quality", type=float, default=0.60)
    parser.add_argument("--min-path-points", type=int, default=8)
    parser.add_argument("--min-speed-mps", type=float, default=0.0)
    parser.add_argument("--skip-emergency-brake", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-episode-on-lap-wrap", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--cone-count", type=int, default=12)
    parser.add_argument("--preview-count", type=int, default=8)
    parser.add_argument("--sector-count", type=int, default=24)
    parser.add_argument("--danger-zone-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--danger-zone-radius-m", type=float, default=7.0)
    parser.add_argument("--stage", type=int, default=4)
    parser.add_argument("--stage3-direct-blend", type=float, default=0.35)
    parser.add_argument("--stage4-direct-blend", type=float, default=1.0)
    parser.add_argument("--max-path-offset-m", type=float, default=0.60)
    parser.add_argument("--max-speed-delta-mps", type=float, default=1.20)
    parser.add_argument("--max-direct-steering", type=float, default=1.0)
    parser.add_argument("--max-direct-throttle", type=float, default=1.0)
    parser.add_argument("--max-direct-brake", type=float, default=1.0)
    parser.add_argument(
        "--residual-label-mode",
        choices=["none", "path-speed"],
        default="path-speed",
        help=(
            "How to fill the two residual action channels. 'none' writes zeros. "
            "'path-speed' uses /autonomy/path_offset and target_speed-speed, clipped to SAC bounds."
        ),
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if not args.jsonl and not args.npz:
        raise SystemExit("Enable at least one output format with --jsonl or --npz")

    import rclpy

    rclpy.init()
    collector: BCDemoCollector | None = None
    try:
        collector = BCDemoCollector(args)
        while rclpy.ok() and not collector.done:
            rclpy.spin_once(collector.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if collector is not None:
            collector.finalize()
            collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
from typing import Any

import rclpy
from fs_msgs.msg import ControlCommand
from fsds_autonomy_msgs.msg import ConeArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String

from fsds_autonomy.constants import ConeColor, is_reliable_geometry_source
from fsds_autonomy.geometry import distance_xy, nearest_point_index, point, pose2_from_pose_stamped


class TopicRate:
    def __init__(self, window: int = 120) -> None:
        self.count = 0
        self.last_sec: float | None = None
        self.intervals: deque[float] = deque(maxlen=max(2, window))

    def tick(self, now_sec: float) -> None:
        self.count += 1
        if self.last_sec is not None:
            dt = max(0.0, now_sec - self.last_sec)
            if dt > 0.0:
                self.intervals.append(dt)
        self.last_sec = now_sec

    def summary(self) -> dict[str, Any]:
        if not self.intervals:
            return {"count": self.count, "hz_avg": None, "dt_avg_sec": None}
        avg = sum(self.intervals) / len(self.intervals)
        return {
            "count": self.count,
            "hz_avg": 1.0 / avg if avg > 0.0 else None,
            "dt_avg_sec": avg,
            "dt_min_sec": min(self.intervals),
            "dt_max_sec": max(self.intervals),
            "window": len(self.intervals),
        }


class DriveRecorder(Node):
    def __init__(self) -> None:
        super().__init__("fsds_drive_recorder")
        self.declare_parameter("enabled", True)
        self.declare_parameter("log_dir", str(Path.home() / ".fsds_autonomy" / "drive_logs"))
        self.declare_parameter("run_id", "")
        self.declare_parameter("flush_every_n", 25)
        self.declare_parameter("summary_rate_hz", 1.0)
        self.declare_parameter("record_controls", True)
        self.declare_parameter("record_summaries", True)
        self.declare_parameter("latest_symlink", True)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.flush_every_n = max(1, int(self.get_parameter("flush_every_n").value))
        self.record_controls = bool(self.get_parameter("record_controls").value)
        self.record_summaries = bool(self.get_parameter("record_summaries").value)
        self.latest_symlink = bool(self.get_parameter("latest_symlink").value)
        self.run_id = str(self.get_parameter("run_id").value).strip()
        if not self.run_id:
            self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.log_dir = Path(str(self.get_parameter("log_dir").value)).expanduser()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.log_dir / f"drive_{self.run_id}.jsonl"
        self.file = self.path.open("a", encoding="utf-8") if self.enabled else None

        self.control_seq = 0
        self.write_count = 0
        self.topic_rates: dict[str, TopicRate] = {}
        self.last_control_time: float | None = None
        self.control_intervals: deque[float] = deque(maxlen=250)

        self.speed: float | None = None
        self.target_speed: float | None = None
        self.path_offset: float | None = None
        self.pose: PoseStamped | None = None
        self.race_state: RaceState | None = None
        self.track: TrackMap | None = None
        self.fused_cones: ConeArray | None = None
        self.controller_diag = ""
        self.mapper_diag = ""
        self.sensor_fusion_diag = ""
        self.offtrack_status = ""

        if self.enabled:
            self.write_json(
                {
                    "event": "metadata",
                    "run_id": self.run_id,
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                    "log_path": str(self.path),
                    "schema": "fsds_drive_recorder_v1",
                }
            )
            if self.latest_symlink:
                self.update_latest_symlink()
            self.get_logger().warn(f"Drive recorder writing {self.path}")
        else:
            self.get_logger().info("Drive recorder disabled")

        self.create_subscription(ControlCommand, "/fsds/control_command", self.on_control, 100)
        self.create_subscription(Float32, "/autonomy/speed", self.on_speed, 20)
        self.create_subscription(Float32, "/autonomy/target_speed", self.on_target_speed, 20)
        self.create_subscription(Float32, "/autonomy/path_offset", self.on_path_offset, 20)
        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 20)
        self.create_subscription(RaceState, "/autonomy/race_state", self.on_race_state, 20)
        self.create_subscription(TrackMap, "/autonomy/racing_line", self.on_track, 20)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_fused_cones, 20)
        self.create_subscription(String, "/autonomy/controller_diagnostics", self.on_controller_diag, 20)
        self.create_subscription(String, "/autonomy/mapper_diagnostics", self.on_mapper_diag, 20)
        self.create_subscription(String, "/autonomy/sensor_fusion_diagnostics", self.on_sensor_fusion_diag, 20)
        self.create_subscription(String, "/autonomy/offtrack_reset_status", self.on_offtrack_status, 20)

        period = 1.0 / max(0.1, float(self.get_parameter("summary_rate_hz").value))
        self.create_timer(period, self.publish_summary)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self, topic: str) -> float:
        now_sec = self.now_sec()
        self.topic_rates.setdefault(topic, TopicRate()).tick(now_sec)
        return now_sec

    def on_speed(self, msg: Float32) -> None:
        self.tick("/autonomy/speed")
        self.speed = float(msg.data)

    def on_target_speed(self, msg: Float32) -> None:
        self.tick("/autonomy/target_speed")
        self.target_speed = float(msg.data)

    def on_path_offset(self, msg: Float32) -> None:
        self.tick("/autonomy/path_offset")
        self.path_offset = float(msg.data)

    def on_pose(self, msg: PoseStamped) -> None:
        self.tick("/autonomy/pose")
        self.pose = msg

    def on_race_state(self, msg: RaceState) -> None:
        self.tick("/autonomy/race_state")
        self.race_state = msg

    def on_track(self, msg: TrackMap) -> None:
        self.tick("/autonomy/racing_line")
        self.track = msg

    def on_fused_cones(self, msg: ConeArray) -> None:
        self.tick("/autonomy/fused_cones")
        self.fused_cones = msg

    def on_controller_diag(self, msg: String) -> None:
        self.tick("/autonomy/controller_diagnostics")
        self.controller_diag = str(msg.data)

    def on_mapper_diag(self, msg: String) -> None:
        self.tick("/autonomy/mapper_diagnostics")
        self.mapper_diag = str(msg.data)

    def on_sensor_fusion_diag(self, msg: String) -> None:
        self.tick("/autonomy/sensor_fusion_diagnostics")
        self.sensor_fusion_diag = str(msg.data)

    def on_offtrack_status(self, msg: String) -> None:
        self.tick("/autonomy/offtrack_reset_status")
        self.offtrack_status = str(msg.data)

    def on_control(self, msg: ControlCommand) -> None:
        now_sec = self.tick("/fsds/control_command")
        if not self.enabled or not self.record_controls:
            return
        self.control_seq += 1
        dt_sec = None
        hz = None
        if self.last_control_time is not None:
            dt_sec = max(0.0, now_sec - self.last_control_time)
            if dt_sec > 0.0:
                hz = 1.0 / dt_sec
                self.control_intervals.append(dt_sec)
        self.last_control_time = now_sec

        self.write_json(
            {
                "event": "control",
                "run_id": self.run_id,
                "seq": self.control_seq,
                "time_sec": now_sec,
                "stamp": self.stamp_dict(msg.header.stamp),
                "dt_sec": dt_sec,
                "hz": hz,
                "command": {
                    "throttle": float(msg.throttle),
                    "brake": float(msg.brake),
                    "steering": float(msg.steering),
                },
                "context": self.context_snapshot(),
            }
        )

    def publish_summary(self) -> None:
        if not self.enabled or not self.record_summaries:
            return
        self.write_json(
            {
                "event": "summary",
                "run_id": self.run_id,
                "time_sec": self.now_sec(),
                "control": self.control_rate_summary(),
                "topics": {topic: rate.summary() for topic, rate in sorted(self.topic_rates.items())},
                "context": self.context_snapshot(),
            }
        )

    def context_snapshot(self) -> dict[str, Any]:
        return {
            "speed_mps": self.speed,
            "target_speed_mps": self.target_speed,
            "path_offset_m": self.path_offset,
            "pose": self.pose_dict(),
            "race_state": self.race_state_dict(),
            "track": self.track_summary(),
            "cones": self.cone_summary(),
            "edge": self.edge_summary(),
            "controller_diagnostics": self.controller_diag,
            "mapper_diagnostics": self.mapper_diag,
            "sensor_fusion_diagnostics": self.sensor_fusion_diag,
            "offtrack_reset_status": self.offtrack_status,
        }

    def control_rate_summary(self) -> dict[str, Any]:
        if not self.control_intervals:
            return {"count": self.control_seq, "hz_avg": None}
        avg = sum(self.control_intervals) / len(self.control_intervals)
        return {
            "count": self.control_seq,
            "hz_avg": 1.0 / avg if avg > 0.0 else None,
            "dt_avg_sec": avg,
            "dt_min_sec": min(self.control_intervals),
            "dt_max_sec": max(self.control_intervals),
            "window": len(self.control_intervals),
        }

    def pose_dict(self) -> dict[str, Any] | None:
        if self.pose is None:
            return None
        pose = pose2_from_pose_stamped(self.pose)
        return {
            "x": pose.x,
            "y": pose.y,
            "yaw_rad": pose.yaw,
            "yaw_deg": math.degrees(pose.yaw),
            "stamp": self.stamp_dict(self.pose.header.stamp),
        }

    def race_state_dict(self) -> dict[str, Any] | None:
        if self.race_state is None:
            return None
        msg = self.race_state
        return {
            "mission": str(msg.mission),
            "track_id": str(msg.track_id),
            "mode": str(msg.mode),
            "behavior_state": str(msg.behavior_state),
            "target_speed": float(msg.target_speed),
            "target_steering": float(msg.target_steering),
            "map_quality": float(msg.map_quality),
            "map_loaded": bool(msg.map_loaded),
            "emergency_brake": bool(msg.emergency_brake),
            "go_signal_fresh": bool(msg.go_signal_fresh),
            "status": str(msg.status),
        }

    def track_summary(self) -> dict[str, Any] | None:
        if self.track is None:
            return None
        return {
            "track_id": str(self.track.track_id),
            "closed_loop": bool(self.track.closed_loop),
            "quality": float(self.track.quality),
            "cones": len(self.track.cones),
            "centerline_points": len(self.track.centerline),
            "racing_line_points": len(self.track.racing_line),
            "blue_boundary_points": len(self.track.blue_boundary_line),
            "yellow_boundary_points": len(self.track.yellow_boundary_line),
            "speed_profile_points": len(self.track.speed_profile),
            "centerline_endpoint_gap_m": self.endpoint_gap(self.track.centerline),
            "racing_line_endpoint_gap_m": self.endpoint_gap(self.track.racing_line),
        }

    def cone_summary(self) -> dict[str, Any] | None:
        if self.fused_cones is None:
            return None
        cones = list(self.fused_cones.cones)
        reliable = [cone for cone in cones if is_reliable_geometry_source(cone.source)]
        forward = [cone for cone in reliable if 0.0 < float(cone.position.x) < 12.0]
        close_path = [
            cone
            for cone in forward
            if abs(float(cone.position.y)) < 1.25 and float(cone.confidence) >= 0.50
        ]
        left = [cone for cone in forward if int(cone.color) == ConeColor.BLUE]
        right = [cone for cone in forward if int(cone.color) == ConeColor.YELLOW]
        unknown = [cone for cone in forward if int(cone.color) == ConeColor.UNKNOWN]
        nearest = min(
            forward,
            key=lambda cone: math.hypot(float(cone.position.x), float(cone.position.y)),
            default=None,
        )
        return {
            "count": len(cones),
            "reliable_count": len(reliable),
            "forward_count": len(forward),
            "close_path_count": len(close_path),
            "blue_forward_count": len(left),
            "yellow_forward_count": len(right),
            "unknown_forward_count": len(unknown),
            "nearest_forward": self.cone_dict(nearest),
        }

    def edge_summary(self) -> dict[str, Any] | None:
        if self.pose is None or self.track is None:
            return None
        pose = pose2_from_pose_stamped(self.pose)
        pos = point(pose.x, pose.y, 0.0)
        center_distance = self.nearest_distance(self.track.centerline, pos)
        racing_distance = self.nearest_distance(self.track.racing_line, pos)
        clearances = self.boundary_clearances(pos)
        return {
            "centerline_distance_m": center_distance,
            "racing_line_distance_m": racing_distance,
            "boundary_clearance": clearances,
        }

    def boundary_clearances(self, pos) -> dict[str, Any] | None:
        if (
            self.track is None
            or not self.track.centerline
            or not self.track.blue_boundary_line
            or not self.track.yellow_boundary_line
        ):
            return None
        center_index = nearest_point_index(self.track.centerline, pos.x, pos.y)
        if center_index < 0:
            return None
        center = self.track.centerline[center_index]
        blue = self.nearest_point(self.track.blue_boundary_line, center.x, center.y)
        yellow = self.nearest_point(self.track.yellow_boundary_line, center.x, center.y)
        if blue is None or yellow is None:
            return None
        vx = float(blue.x) - float(yellow.x)
        vy = float(blue.y) - float(yellow.y)
        width = math.hypot(vx, vy)
        if width < 1e-6:
            return None
        wx = float(pos.x) - float(yellow.x)
        wy = float(pos.y) - float(yellow.y)
        fraction = (wx * vx + wy * vy) / (width * width)
        yellow_clearance = fraction * width
        blue_clearance = (1.0 - fraction) * width
        nearest_side = "yellow" if yellow_clearance < blue_clearance else "blue"
        return {
            "center_index": int(center_index),
            "track_width_m": width,
            "yellow_clearance_m": yellow_clearance,
            "blue_clearance_m": blue_clearance,
            "min_clearance_m": min(yellow_clearance, blue_clearance),
            "nearest_side": nearest_side,
            "lateral_fraction_yellow_to_blue": fraction,
        }

    def write_json(self, payload: dict[str, Any]) -> None:
        if self.file is None:
            return
        self.file.write(json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n")
        self.write_count += 1
        if self.write_count % self.flush_every_n == 0:
            self.file.flush()

    def update_latest_symlink(self) -> None:
        latest = self.log_dir / "latest.jsonl"
        try:
            if latest.exists() or latest.is_symlink():
                latest.unlink()
            os.symlink(self.path.name, latest)
        except OSError as exc:
            self.get_logger().warn(f"Could not update latest drive log symlink: {exc}")

    @staticmethod
    def stamp_dict(stamp) -> dict[str, int]:
        return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}

    @staticmethod
    def cone_dict(cone) -> dict[str, Any] | None:
        if cone is None:
            return None
        return {
            "x": float(cone.position.x),
            "y": float(cone.position.y),
            "range": float(cone.range),
            "bearing": float(cone.bearing),
            "color": int(cone.color),
            "confidence": float(cone.confidence),
            "source": str(cone.source),
        }

    @staticmethod
    def endpoint_gap(points) -> float | None:
        if len(points) < 2:
            return None
        return distance_xy(points[0], points[-1])

    @staticmethod
    def nearest_point(points, x: float, y: float):
        if not points:
            return None
        return min(points, key=lambda p: (float(p.x) - x) ** 2 + (float(p.y) - y) ** 2)

    @staticmethod
    def nearest_distance(points, pos) -> float | None:
        nearest = DriveRecorder.nearest_point(points, pos.x, pos.y)
        if nearest is None:
            return None
        return distance_xy(nearest, pos)

    def destroy_node(self) -> bool:
        if self.file is not None:
            self.file.flush()
            self.file.close()
            self.file = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DriveRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

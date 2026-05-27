from __future__ import annotations

from pathlib import Path

import rclpy
from fsds_autonomy_msgs.msg import ConeArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from fsds_autonomy.constants import ConeColor, is_reliable_geometry_source, side_color_from_local_y
from fsds_autonomy.geometry import pose2_from_pose_stamped, transform_local_to_global
from fsds_autonomy.map_store import (
    ConeLandmark,
    SavedTrackMap,
    is_usable_track_map,
    load_track_map,
    save_track_map,
    track_map_sanity_reasons,
)
from fsds_autonomy.planning import build_centerline_from_cones, build_racing_line, infer_map_quality
from fsds_autonomy.ros_utils import track_map_to_msg


class Mapper(Node):
    def __init__(self) -> None:
        super().__init__("fsds_mapper")
        self.declare_parameter("map_dir", "maps")
        self.declare_parameter("association_radius_m", 0.75)
        self.declare_parameter("save_period_sec", 5.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("update_loaded_map_in_race_from_map", False)

        self.map_dir = Path(str(self.get_parameter("map_dir").value)).expanduser()
        self.association_radius = float(self.get_parameter("association_radius_m").value)
        self.update_loaded_map_in_race_from_map = bool(
            self.get_parameter("update_loaded_map_in_race_from_map").value
        )
        self.track_id = "unknown"
        self.mode = "waiting_for_go"
        self.pose = None
        self.landmarks: list[ConeLandmark] = []
        self.loaded_track_map: SavedTrackMap | None = None
        self.loaded_once = False

        self.map_pub = self.create_publisher(TrackMap, "/autonomy/track_map", 10)
        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_cones, 10)
        self.create_subscription(RaceState, "/autonomy/mission_state", self.on_mission, 10)

        publish_period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(publish_period, self.publish_map)
        self.create_timer(float(self.get_parameter("save_period_sec").value), self.save_if_ready)

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = pose2_from_pose_stamped(msg)

    def on_mission(self, msg: RaceState) -> None:
        if msg.track_id and msg.track_id != self.track_id:
            self.track_id = msg.track_id
            self.landmarks = []
            self.loaded_track_map = None
            self.loaded_once = False
        self.mode = msg.mode
        if not self.loaded_once and msg.map_loaded and msg.mode == "race_from_map":
            saved = load_track_map(self.map_dir, self.track_id)
            if saved and is_usable_track_map(saved):
                self.loaded_track_map = saved
                self.landmarks = list(saved.cones)
                self.get_logger().info(f"Loaded saved map for {self.track_id}: {len(self.landmarks)} cones")
            self.loaded_once = True

    def on_cones(self, msg: ConeArray) -> None:
        if self.pose is None or self.mode == "waiting_for_go":
            return
        if (
            self.mode == "race_from_map"
            and self.loaded_track_map is not None
            and not self.update_loaded_map_in_race_from_map
        ):
            return
        for cone in msg.cones:
            if cone.range > 25.0 or cone.confidence < 0.10:
                continue
            if not is_reliable_geometry_source(cone.source):
                continue
            color = int(cone.color)
            if color == ConeColor.UNKNOWN:
                # LiDAR geometry has no cone color; use the FSDS side convention
                # when camera fusion has not assigned a class yet.
                color = side_color_from_local_y(cone.position.y)
                if color == ConeColor.UNKNOWN:
                    continue
            x, y = transform_local_to_global(cone.position.x, cone.position.y, self.pose)
            self.upsert_landmark(x, y, cone.position.z, color, cone.confidence)

    def upsert_landmark(self, x: float, y: float, z: float, color: int, confidence: float) -> None:
        best = None
        best_dist = self.association_radius
        for landmark in self.landmarks:
            dist = ((landmark.x - x) ** 2 + (landmark.y - y) ** 2) ** 0.5
            if dist < best_dist:
                best = landmark
                best_dist = dist
        if best is None:
            self.landmarks.append(
                ConeLandmark(x=x, y=y, z=z, color=int(color), confidence=float(confidence), observations=1)
            )
            return
        n = max(1, best.observations)
        best.x = (best.x * n + x) / (n + 1)
        best.y = (best.y * n + y) / (n + 1)
        best.z = (best.z * n + z) / (n + 1)
        if color != 4 and confidence >= best.confidence:
            best.color = int(color)
        best.confidence = max(best.confidence, float(confidence))
        best.observations += 1

    def current_track_map(self) -> SavedTrackMap:
        if (
            self.mode == "race_from_map"
            and self.loaded_track_map is not None
            and not self.update_loaded_map_in_race_from_map
        ):
            return self.loaded_track_map
        centerline = build_centerline_from_cones(self.landmarks)
        racing_line, speed_profile = build_racing_line(centerline)
        quality = infer_map_quality(self.landmarks, centerline)
        closed_loop = bool(len(centerline) > 12 and quality > 0.45)
        return SavedTrackMap(
            track_id=self.track_id,
            closed_loop=closed_loop,
            quality=quality,
            cones=self.landmarks,
            centerline=[(p.x, p.y) for p in centerline],
            racing_line=[(p.x, p.y) for p in racing_line],
            speed_profile=speed_profile,
            metadata={"mode": self.mode},
        )

    def publish_map(self) -> None:
        self.map_pub.publish(track_map_to_msg(self, self.current_track_map()))

    def save_if_ready(self) -> None:
        if self.mode == "waiting_for_go" or len(self.landmarks) < 8:
            return
        if (
            self.mode == "race_from_map"
            and self.loaded_track_map is not None
            and not self.update_loaded_map_in_race_from_map
        ):
            return
        track_map = self.current_track_map()
        if not is_usable_track_map(track_map):
            reasons = "; ".join(track_map_sanity_reasons(track_map))
            self.get_logger().warn(
                f"Not saving unusable map cones={len(track_map.cones)} centerline={len(track_map.centerline)} "
                f"racing_line={len(track_map.racing_line)} quality={track_map.quality:.2f}; {reasons}",
                throttle_duration_sec=15.0,
            )
            return
        path = save_track_map(self.map_dir, track_map)
        self.get_logger().info(
            f"Saved map {path} cones={len(track_map.cones)} quality={track_map.quality:.2f}",
            throttle_duration_sec=15.0,
        )


def main() -> None:
    rclpy.init()
    rclpy.spin(Mapper())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

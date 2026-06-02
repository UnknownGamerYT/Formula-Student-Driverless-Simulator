from __future__ import annotations

from pathlib import Path

import rclpy
from fsds_autonomy_msgs.msg import RaceState, TrackMap
from rclpy.node import Node
from std_msgs.msg import String

from fsds_autonomy.map_store import (
    SavedTrackMap,
    is_usable_track_map,
    load_track_map,
    track_map_sanity_reasons,
)
from fsds_autonomy.ros_utils import track_map_to_msg


class SavedMapPublisher(Node):
    """Publish a verified saved map directly for lightweight RL training."""

    def __init__(self) -> None:
        super().__init__("fsds_saved_map_publisher")
        self.declare_parameter("map_dir", "maps")
        self.declare_parameter("track_id", "A")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("require_closed_loop", True)
        self.declare_parameter("publish_live_track_map", True)

        self.map_dir = Path(str(self.get_parameter("map_dir").value)).expanduser()
        self.default_track_id = str(self.get_parameter("track_id").value)
        self.require_closed_loop = bool(self.get_parameter("require_closed_loop").value)
        self.publish_live_track_map = bool(self.get_parameter("publish_live_track_map").value)

        self.mission_state = RaceState()
        self.loaded_track_id = ""
        self.track_map: SavedTrackMap | None = None
        self.last_error = ""

        self.track_pub = self.create_publisher(TrackMap, "/autonomy/track_map", 10)
        self.live_track_pub = self.create_publisher(TrackMap, "/autonomy/live_track_map", 10)
        self.reference_pub = self.create_publisher(TrackMap, "/autonomy/reference_track_map", 10)
        self.racing_pub = self.create_publisher(TrackMap, "/autonomy/racing_line", 10)
        self.diag_pub = self.create_publisher(String, "/autonomy/saved_map_publisher_diagnostics", 10)
        self.create_subscription(RaceState, "/autonomy/mission_state", self.on_mission_state, 10)

        period = 1.0 / max(0.5, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish)

    def on_mission_state(self, msg: RaceState) -> None:
        self.mission_state = msg

    def active_track_id(self) -> str:
        return str(self.mission_state.track_id or self.default_track_id or "A")

    def ensure_loaded(self) -> bool:
        track_id = self.active_track_id()
        if self.track_map is not None and self.loaded_track_id == track_id:
            return True
        loaded = load_track_map(self.map_dir, track_id)
        if loaded is None:
            self.track_map = None
            self.loaded_track_id = track_id
            self.last_error = f"missing saved map track_id={track_id} map_dir={self.map_dir}"
            return False
        reasons = track_map_sanity_reasons(loaded, require_closed_loop=self.require_closed_loop)
        if reasons:
            self.track_map = None
            self.loaded_track_id = track_id
            self.last_error = f"unusable saved map track_id={track_id}: {'; '.join(reasons)}"
            return False
        if self.require_closed_loop and not is_usable_track_map(loaded, require_closed_loop=True):
            self.track_map = None
            self.loaded_track_id = track_id
            self.last_error = f"unusable saved map track_id={track_id}"
            return False
        self.track_map = loaded
        self.loaded_track_id = track_id
        self.last_error = ""
        self.get_logger().info(
            f"Loaded saved map for RL fast mode: track={track_id} cones={len(loaded.cones)} "
            f"centerline={len(loaded.centerline)} racing_line={len(loaded.racing_line)} "
            f"quality={loaded.quality:.2f} closed_loop={loaded.closed_loop}"
        )
        return True

    def publish(self) -> None:
        diag = String()
        if not self.ensure_loaded() or self.track_map is None:
            diag.data = self.last_error or "waiting_for_saved_map"
            self.diag_pub.publish(diag)
            return
        msg = track_map_to_msg(self, self.track_map)
        self.track_pub.publish(msg)
        self.reference_pub.publish(msg)
        self.racing_pub.publish(msg)
        if self.publish_live_track_map:
            self.live_track_pub.publish(msg)
        diag.data = (
            f"published track={self.track_map.track_id} quality={self.track_map.quality:.2f} "
            f"closed_loop={self.track_map.closed_loop} cones={len(self.track_map.cones)} "
            f"racing_line={len(self.track_map.racing_line)}"
        )
        self.diag_pub.publish(diag)


def main() -> None:
    rclpy.init()
    rclpy.spin(SavedMapPublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

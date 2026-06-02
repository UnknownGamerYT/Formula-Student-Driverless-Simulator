from __future__ import annotations

from pathlib import Path

import rclpy
from fs_msgs.msg import GoSignal
from fsds_autonomy_msgs.msg import RaceState
from rclpy.node import Node

from fsds_autonomy.map_store import is_usable_track_map, load_track_map, map_path, track_map_sanity_reasons
from fsds_autonomy.ros_utils import race_state_msg


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("fsds_mission_manager")
        self.declare_parameter("map_dir", "maps")
        self.declare_parameter("go_timeout_sec", 4.0)
        self.declare_parameter("publish_rate_hz", 5.0)

        self.map_dir = Path(str(self.get_parameter("map_dir").value)).expanduser()
        self.go_timeout_sec = float(self.get_parameter("go_timeout_sec").value)

        self.mission = "idle"
        self.track_id = "unknown"
        self.mode = "waiting_for_go"
        self.map_loaded = False
        self.last_go_time = None

        self.state_pub = self.create_publisher(RaceState, "/autonomy/mission_state", 10)
        self.create_subscription(GoSignal, "/fsds/signal/go", self.on_go, 10)
        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish_state)

    def on_go(self, msg: GoSignal) -> None:
        self.last_go_time = self.get_clock().now()
        new_mission = msg.mission or "trackdrive"
        new_track = msg.track or "unknown"
        changed = new_mission != self.mission or new_track != self.track_id

        self.mission = new_mission
        self.track_id = new_track
        saved_map = load_track_map(self.map_dir, self.track_id)
        self.map_loaded = saved_map is not None and is_usable_track_map(saved_map, require_closed_loop=True)

        if self.mission == "autocross":
            if changed:
                old_map = map_path(self.map_dir, self.track_id)
                if old_map.exists():
                    old_map.unlink()
                    self.get_logger().info(f"Deleted old autocross map for {self.track_id}: {old_map}")
            self.mode = "learn_race"
            self.map_loaded = False
        elif self.map_loaded:
            self.mode = "race_from_map"
        else:
            self.mode = "learn_race"
            if saved_map is not None and not is_usable_track_map(saved_map, require_closed_loop=True):
                reasons = "; ".join(track_map_sanity_reasons(saved_map, require_closed_loop=True))
                self.get_logger().warn(
                    f"Ignoring unusable saved map for {self.track_id}: "
                    f"centerline={len(saved_map.centerline)} racing_line={len(saved_map.racing_line)} "
                    f"speed_profile={len(saved_map.speed_profile)} quality={saved_map.quality:.2f}; {reasons}",
                    throttle_duration_sec=10.0,
                )

        if changed:
            self.get_logger().info(
                f"GO received: mission={self.mission} track={self.track_id} mode={self.mode} map_loaded={self.map_loaded}"
            )

    def go_signal_fresh(self) -> bool:
        if self.last_go_time is None:
            return False
        elapsed_ns = (self.get_clock().now() - self.last_go_time).nanoseconds
        return elapsed_ns * 1e-9 <= self.go_timeout_sec

    def publish_state(self) -> None:
        fresh = self.go_signal_fresh()
        mode = self.mode if fresh else "waiting_for_go"
        status = "go" if fresh else "waiting for /fsds/signal/go"
        self.state_pub.publish(
            race_state_msg(
                self,
                self.mission,
                self.track_id,
                mode,
                "mission",
                0.0,
                0.0,
                0.0,
                self.map_loaded,
                False,
                fresh,
                status,
            )
        )


def main() -> None:
    rclpy.init()
    rclpy.spin(MissionManager())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

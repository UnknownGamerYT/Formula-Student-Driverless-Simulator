from __future__ import annotations

import math

import rclpy
from fs_msgs.msg import ControlCommand
from fs_msgs.srv import Reset
from fsds_autonomy_msgs.msg import RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from fsds_autonomy.geometry import distance_xy


class OfftrackResetMonitor(Node):
    def __init__(self) -> None:
        super().__init__("fsds_offtrack_reset_monitor")
        self.declare_parameter("enabled", True)
        self.declare_parameter("offtrack_distance_m", 4.5)
        self.declare_parameter("offtrack_hold_sec", 1.5)
        self.declare_parameter("reset_cooldown_sec", 8.0)
        self.declare_parameter("min_path_points", 8)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.offtrack_distance = float(self.get_parameter("offtrack_distance_m").value)
        self.offtrack_hold_sec = float(self.get_parameter("offtrack_hold_sec").value)
        self.reset_cooldown_sec = float(self.get_parameter("reset_cooldown_sec").value)
        self.min_path_points = int(self.get_parameter("min_path_points").value)

        self.pose = None
        self.track = TrackMap()
        self.race_state = RaceState()
        self.offtrack_since = None
        self.last_reset_time = None
        self.reset_in_flight = False

        self.reset_client = self.create_client(Reset, "/fsds/reset")
        self.brake_pub = self.create_publisher(ControlCommand, "/fsds/control_command", 10)
        self.status_pub = self.create_publisher(String, "/autonomy/offtrack_reset_status", 10)

        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)
        self.create_subscription(TrackMap, "/autonomy/racing_line", self.on_track, 10)
        self.create_subscription(RaceState, "/autonomy/race_state", self.on_race_state, 10)
        self.create_timer(0.1, self.tick)

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def on_track(self, msg: TrackMap) -> None:
        self.track = msg

    def on_race_state(self, msg: RaceState) -> None:
        self.race_state = msg

    def usable_path(self):
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

    def tick(self) -> None:
        if not self.enabled:
            self.publish_status("disabled")
            return
        if not self.race_state.go_signal_fresh:
            self.offtrack_since = None
            self.publish_status("waiting_for_go")
            return

        distance = self.distance_to_path()
        if distance is None:
            self.offtrack_since = None
            self.publish_status("waiting_for_path")
            return

        if distance <= self.offtrack_distance:
            self.offtrack_since = None
            self.publish_status(f"on_track distance={distance:.2f}m")
            return

        now = self.get_clock().now()
        if self.offtrack_since is None:
            self.offtrack_since = now
        self.publish_brake()

        offtrack_for = (now - self.offtrack_since).nanoseconds * 1e-9
        self.publish_status(f"off_track distance={distance:.2f}m hold={offtrack_for:.1f}s")
        if offtrack_for < self.offtrack_hold_sec or self.cooldown_active() or self.reset_in_flight:
            return

        if not self.reset_client.service_is_ready():
            self.reset_client.wait_for_service(timeout_sec=0.05)
        if not self.reset_client.service_is_ready():
            self.get_logger().warn("/fsds/reset is not available yet", throttle_duration_sec=2.0)
            return

        request = Reset.Request()
        request.wait_on_last_task = False
        future = self.reset_client.call_async(request)
        future.add_done_callback(self.on_reset_done)
        self.reset_in_flight = True
        self.last_reset_time = now
        self.offtrack_since = None
        self.get_logger().warn(f"Off-track reset requested at distance={distance:.2f}m")

    def on_reset_done(self, future) -> None:
        self.reset_in_flight = False
        try:
            future.result()
            self.publish_status("reset_complete")
            self.get_logger().info("Off-track reset complete")
        except Exception as exc:
            self.publish_status(f"reset_failed {exc}")
            self.get_logger().error(f"Off-track reset failed: {exc}")


def main() -> None:
    rclpy.init()
    rclpy.spin(OfftrackResetMonitor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

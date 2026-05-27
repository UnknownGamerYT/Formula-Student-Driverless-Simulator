from __future__ import annotations

import math
from statistics import median

import rclpy
from fs_msgs.msg import ControlCommand
from fsds_autonomy_msgs.msg import ConeArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32

from fsds_autonomy.constants import is_reliable_geometry_source
from fsds_autonomy.geometry import (
    clamp,
    first_forward_lookahead,
    pose2_from_pose_stamped,
    transform_global_to_local,
)


class Controller(Node):
    def __init__(self) -> None:
        super().__init__("fsds_controller")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("lookahead_m", 5.0)
        self.declare_parameter("max_steering_angle_rad", 0.48)
        self.declare_parameter("track_half_width_m", 1.75)
        self.declare_parameter("throttle_kp", 0.12)
        self.declare_parameter("brake_kp", 0.35)
        self.declare_parameter("max_throttle", 0.55)
        self.declare_parameter("max_brake", 0.80)
        self.declare_parameter("max_path_offset_m", 1.0)
        self.declare_parameter("launch_throttle", 0.45)
        self.declare_parameter("launch_speed_threshold_mps", 0.60)
        self.declare_parameter("local_cone_min_confidence", 0.35)

        self.lookahead = float(self.get_parameter("lookahead_m").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle_rad").value)
        self.track_half_width = float(self.get_parameter("track_half_width_m").value)
        self.throttle_kp = float(self.get_parameter("throttle_kp").value)
        self.brake_kp = float(self.get_parameter("brake_kp").value)
        self.max_throttle = float(self.get_parameter("max_throttle").value)
        self.max_brake = float(self.get_parameter("max_brake").value)
        self.max_path_offset = float(self.get_parameter("max_path_offset_m").value)
        self.launch_throttle = float(self.get_parameter("launch_throttle").value)
        self.launch_speed_threshold = float(self.get_parameter("launch_speed_threshold_mps").value)
        self.local_cone_min_confidence = float(self.get_parameter("local_cone_min_confidence").value)

        self.pose = None
        self.speed = 0.0
        self.target_speed = 0.0
        self.race_state = RaceState()
        self.track = TrackMap()
        self.local_cones = ConeArray()
        self.path_offset = 0.0

        self.control_pub = self.create_publisher(ControlCommand, "/fsds/control_command", 10)
        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)
        self.create_subscription(Float32, "/autonomy/speed", self.on_speed, 10)
        self.create_subscription(Float32, "/autonomy/target_speed", self.on_target_speed, 10)
        self.create_subscription(RaceState, "/autonomy/race_state", self.on_race_state, 10)
        self.create_subscription(TrackMap, "/autonomy/racing_line", self.on_track, 10)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_local_cones, 10)
        self.create_subscription(Float32, "/autonomy/path_offset", self.on_path_offset, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish_control)

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def on_speed(self, msg: Float32) -> None:
        self.speed = float(msg.data)

    def on_target_speed(self, msg: Float32) -> None:
        self.target_speed = float(msg.data)

    def on_race_state(self, msg: RaceState) -> None:
        self.race_state = msg

    def on_track(self, msg: TrackMap) -> None:
        self.track = msg

    def on_local_cones(self, msg: ConeArray) -> None:
        self.local_cones = msg

    def on_path_offset(self, msg: Float32) -> None:
        self.path_offset = clamp(float(msg.data), -self.max_path_offset, self.max_path_offset)

    def global_path_target_local(self) -> tuple[float, float] | None:
        if self.pose is None or not self.track.racing_line:
            return None
        pose = pose2_from_pose_stamped(self.pose)
        target = first_forward_lookahead(self.track.racing_line, pose, self.lookahead)
        if target is None:
            return None
        local_x, local_y = transform_global_to_local(target.x, target.y, pose)
        if local_x < 0.5:
            return None
        return local_x, local_y + self.path_offset

    def local_cone_target(self) -> tuple[float, float] | None:
        cones = [
            cone
            for cone in self.local_cones.cones
            if 0.8 < cone.position.x < 14.0
            and abs(cone.position.y) < 7.0
            and cone.confidence >= self.local_cone_min_confidence
            and is_reliable_geometry_source(cone.source)
        ]
        if not cones:
            return None

        samples = []
        for start_x in (0.8, 3.5, 6.5, 9.5):
            end_x = start_x + 3.5
            group = [cone for cone in cones if start_x <= cone.position.x < end_x]
            if not group:
                continue
            left = [cone.position.y for cone in group if cone.position.y > 0.20]
            right = [cone.position.y for cone in group if cone.position.y < -0.20]
            if left and right:
                center_y = 0.5 * (median(left) + median(right))
            elif left:
                center_y = median(left) - self.track_half_width
            elif right:
                center_y = median(right) + self.track_half_width
            else:
                continue
            center_x = median([cone.position.x for cone in group])
            samples.append((float(center_x), float(center_y)))

        if not samples:
            return None

        desired_x = min(self.lookahead, max(2.5, samples[-1][0]))
        target_x, target_y = min(samples, key=lambda sample: abs(sample[0] - desired_x))
        target_y += self.path_offset
        return target_x, target_y

    def steering_command(self) -> float:
        target = self.global_path_target_local() or self.local_cone_target()
        if target is None:
            return 0.0
        local_x, local_y = target
        heading_error = math.atan2(local_y, max(0.1, local_x))
        # FSDS steering convention: negative is left, positive is right.
        return clamp(-heading_error / self.max_steering_angle, -1.0, 1.0)

    def publish_control(self) -> None:
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "fsds/FSCar"

        steering = self.steering_command()
        target_speed = max(0.0, self.target_speed)
        speed_error = target_speed - self.speed

        if self.race_state.emergency_brake or not self.race_state.go_signal_fresh:
            throttle = 0.0
            brake = 1.0 if self.speed > 0.4 else 0.2
        elif target_speed <= 0.1:
            throttle = 0.0
            brake = 0.4
        elif speed_error >= 0.0:
            throttle = clamp(self.throttle_kp * speed_error, 0.0, self.max_throttle)
            if target_speed > 0.4 and self.speed < self.launch_speed_threshold:
                throttle = max(throttle, min(self.launch_throttle, self.max_throttle))
            brake = 0.0
        else:
            throttle = 0.0
            brake = clamp(-self.brake_kp * speed_error, 0.0, self.max_brake)

        cmd.steering = float(steering)
        cmd.throttle = float(throttle)
        cmd.brake = float(brake)
        self.control_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

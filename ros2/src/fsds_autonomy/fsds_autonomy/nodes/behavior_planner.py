from __future__ import annotations

import math
from statistics import median

import rclpy
from fsds_autonomy_msgs.msg import ConeArray, ObstacleArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32

from fsds_autonomy.constants import is_reliable_geometry_source
from fsds_autonomy.geometry import clamp, nearest_point_index
from fsds_autonomy.ros_utils import race_state_msg


class BehaviorPlanner(Node):
    def __init__(self) -> None:
        super().__init__("fsds_behavior_planner")
        self.declare_parameter("first_lap_speed_mps", 1.6)
        self.declare_parameter("mapping_speed_mps", 2.2)
        self.declare_parameter("race_speed_mps", 5.0)
        self.declare_parameter("map_quality_for_full_mapping_speed", 0.55)
        self.declare_parameter("map_quality_for_speed_profile", 0.65)
        self.declare_parameter("startup_ramp_sec", 10.0)
        self.declare_parameter("target_accel_limit_mps2", 0.40)
        self.declare_parameter("target_decel_limit_mps2", 2.50)
        self.declare_parameter("visible_curve_enabled", True)
        self.declare_parameter("visible_curve_lookahead_m", 18.0)
        self.declare_parameter("visible_curve_min_speed_mps", 1.1)
        self.declare_parameter("visible_straight_speed_mps", 3.0)
        self.declare_parameter("visible_curve_straight_angle_rad", 0.08)
        self.declare_parameter("visible_curve_hard_angle_rad", 0.55)
        self.declare_parameter("visible_curve_min_samples", 3)
        self.declare_parameter("visible_curve_min_cones", 4)
        self.declare_parameter("visible_curve_min_cone_confidence", 0.35)
        self.declare_parameter("slow_distance_m", 7.0)
        self.declare_parameter("brake_distance_m", 2.3)
        self.declare_parameter("corridor_half_width_m", 1.4)
        self.declare_parameter("track_half_width_m", 1.75)
        self.declare_parameter("obstacle_avoid_enabled", True)
        self.declare_parameter("obstacle_avoid_distance_m", 10.0)
        self.declare_parameter("obstacle_avoid_offset_m", 0.85)
        self.declare_parameter("obstacle_avoid_clearance_m", 0.55)
        self.declare_parameter("obstacle_avoid_speed_mps", 1.4)
        self.declare_parameter("obstacle_avoid_hold_sec", 1.0)
        self.declare_parameter("obstacle_stop_if_blocked_distance_m", 5.0)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.first_lap_speed = float(self.get_parameter("first_lap_speed_mps").value)
        self.mapping_speed = float(self.get_parameter("mapping_speed_mps").value)
        self.race_speed = float(self.get_parameter("race_speed_mps").value)
        self.map_quality_for_full_mapping_speed = float(
            self.get_parameter("map_quality_for_full_mapping_speed").value
        )
        self.map_quality_for_speed_profile = float(self.get_parameter("map_quality_for_speed_profile").value)
        self.startup_ramp_sec = float(self.get_parameter("startup_ramp_sec").value)
        self.target_accel_limit = float(self.get_parameter("target_accel_limit_mps2").value)
        self.target_decel_limit = float(self.get_parameter("target_decel_limit_mps2").value)
        self.visible_curve_enabled = bool(self.get_parameter("visible_curve_enabled").value)
        self.visible_curve_lookahead = float(self.get_parameter("visible_curve_lookahead_m").value)
        self.visible_curve_min_speed = float(self.get_parameter("visible_curve_min_speed_mps").value)
        self.visible_straight_speed = float(self.get_parameter("visible_straight_speed_mps").value)
        self.visible_curve_straight_angle = float(self.get_parameter("visible_curve_straight_angle_rad").value)
        self.visible_curve_hard_angle = float(self.get_parameter("visible_curve_hard_angle_rad").value)
        self.visible_curve_min_samples = int(self.get_parameter("visible_curve_min_samples").value)
        self.visible_curve_min_cones = int(self.get_parameter("visible_curve_min_cones").value)
        self.visible_curve_min_cone_confidence = float(
            self.get_parameter("visible_curve_min_cone_confidence").value
        )
        self.slow_distance = float(self.get_parameter("slow_distance_m").value)
        self.brake_distance = float(self.get_parameter("brake_distance_m").value)
        self.corridor_half_width = float(self.get_parameter("corridor_half_width_m").value)
        self.track_half_width = float(self.get_parameter("track_half_width_m").value)
        self.obstacle_avoid_enabled = bool(self.get_parameter("obstacle_avoid_enabled").value)
        self.obstacle_avoid_distance = float(self.get_parameter("obstacle_avoid_distance_m").value)
        self.obstacle_avoid_offset = float(self.get_parameter("obstacle_avoid_offset_m").value)
        self.obstacle_avoid_clearance = float(self.get_parameter("obstacle_avoid_clearance_m").value)
        self.obstacle_avoid_speed = float(self.get_parameter("obstacle_avoid_speed_mps").value)
        self.obstacle_avoid_hold_sec = float(self.get_parameter("obstacle_avoid_hold_sec").value)
        self.obstacle_stop_if_blocked_distance = float(
            self.get_parameter("obstacle_stop_if_blocked_distance_m").value
        )

        self.mission_state = RaceState()
        self.track = TrackMap()
        self.obstacles = ObstacleArray()
        self.local_cones = ConeArray()
        self.pose = None
        self.filtered_target_speed = 0.0
        self.last_publish_time = self.get_clock().now()
        self.go_start_time = None
        self.last_curve_status = "visible_curve=unknown"
        self.path_offset = 0.0
        self.held_obstacle_offset = 0.0
        self.last_obstacle_avoid_time = None

        self.speed_pub = self.create_publisher(Float32, "/autonomy/target_speed", 10)
        self.path_offset_pub = self.create_publisher(Float32, "/autonomy/path_offset", 10)
        self.state_pub = self.create_publisher(RaceState, "/autonomy/race_state", 10)
        self.create_subscription(RaceState, "/autonomy/mission_state", self.on_mission, 10)
        self.create_subscription(TrackMap, "/autonomy/racing_line", self.on_track, 10)
        self.create_subscription(ObstacleArray, "/autonomy/obstacles", self.on_obstacles, 10)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_local_cones, 10)
        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish)

    def on_mission(self, msg: RaceState) -> None:
        self.mission_state = msg

    def on_track(self, msg: TrackMap) -> None:
        self.track = msg

    def on_obstacles(self, msg: ObstacleArray) -> None:
        self.obstacles = msg

    def on_local_cones(self, msg: ConeArray) -> None:
        self.local_cones = msg

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def corridor_obstacles(self, lookahead: float | None = None):
        max_x = lookahead if lookahead is not None else max(self.slow_distance, self.obstacle_avoid_distance)
        return [
            obstacle
            for obstacle in self.obstacles.obstacles
            if 0.0 <= obstacle.position.x <= max_x
            and abs(obstacle.position.y) <= self.corridor_half_width + obstacle.radius
            and obstacle.confidence >= 0.10
        ]

    def clearance_for_offset(self, offset: float, obstacles) -> float:
        boundary_clearance = max(0.0, self.corridor_half_width - abs(offset))
        clearance = boundary_clearance
        for obstacle in obstacles:
            lateral_clearance = abs(obstacle.position.y - offset) - obstacle.radius
            clearance = min(clearance, lateral_clearance)
        return clearance

    def choose_avoid_offset(self, closest, obstacles) -> tuple[float, float]:
        max_offset = max(0.0, min(self.obstacle_avoid_offset, self.corridor_half_width * 0.75))
        candidates = [max_offset, -max_offset]
        if closest.position.y > 0.15:
            candidates = [-max_offset, max_offset]
        elif closest.position.y < -0.15:
            candidates = [max_offset, -max_offset]

        scored = [(self.clearance_for_offset(offset, obstacles), offset) for offset in candidates]
        scored.sort(reverse=True)
        return scored[0]

    def obstacle_plan(self) -> tuple[bool, float | None, float, str]:
        relevant = self.corridor_obstacles()
        closest = None
        closest_dist = float("inf")
        for obstacle in relevant:
            dist = math.hypot(obstacle.position.x, obstacle.position.y)
            if dist < closest_dist:
                closest = obstacle
                closest_dist = dist
        if closest is None:
            if self.last_obstacle_avoid_time is not None:
                held_for = (self.get_clock().now() - self.last_obstacle_avoid_time).nanoseconds * 1e-9
                if held_for <= self.obstacle_avoid_hold_sec and abs(self.held_obstacle_offset) > 0.05:
                    return (
                        False,
                        self.obstacle_avoid_speed,
                        self.held_obstacle_offset,
                        f"avoid_obstacle_hold offset={self.held_obstacle_offset:.2f}m hold={held_for:.1f}s",
                    )
            self.held_obstacle_offset = 0.0
            return False, None, 0.0, "clear"

        if closest_dist <= self.brake_distance:
            self.held_obstacle_offset = 0.0
            return True, 0.0, 0.0, f"emergency_brake obstacle={closest_dist:.1f}m"

        if self.obstacle_avoid_enabled and closest_dist <= self.obstacle_avoid_distance:
            avoid_obstacles = self.corridor_obstacles(self.obstacle_avoid_distance)
            clearance, offset = self.choose_avoid_offset(closest, avoid_obstacles)
            if clearance >= self.obstacle_avoid_clearance:
                speed_cap = max(0.4, self.obstacle_avoid_speed)
                side = "left" if offset > 0.0 else "right"
                self.held_obstacle_offset = offset
                self.last_obstacle_avoid_time = self.get_clock().now()
                return (
                    False,
                    speed_cap,
                    offset,
                    f"avoid_obstacle side={side} offset={offset:.2f}m clearance={clearance:.2f}m obstacle={closest_dist:.1f}m",
                )
            if closest_dist <= self.obstacle_stop_if_blocked_distance:
                return (
                    True,
                    0.0,
                    0.0,
                    f"blocked_obstacle_stop clearance={clearance:.2f}m obstacle={closest_dist:.1f}m",
                )
            return (
                False,
                min(1.0, self.obstacle_avoid_speed),
                0.0,
                f"blocked_obstacle_slow clearance={clearance:.2f}m obstacle={closest_dist:.1f}m",
            )

        if closest_dist <= self.slow_distance:
            factor = max(0.2, (closest_dist - self.brake_distance) / (self.slow_distance - self.brake_distance))
            speed_cap = max(0.7, self.visible_straight_speed * factor)
            return False, speed_cap, 0.0, f"slowing obstacle={closest_dist:.1f}m"

        return False, None, 0.0, "clear"

    def visible_corridor_samples(self) -> list[tuple[float, float]]:
        cones = [
            cone
            for cone in self.local_cones.cones
            if 1.0 <= cone.position.x <= self.visible_curve_lookahead
            and abs(cone.position.y) <= self.corridor_half_width + self.track_half_width + 1.0
            and cone.confidence >= self.visible_curve_min_cone_confidence
            and is_reliable_geometry_source(cone.source)
        ]
        if len(cones) < self.visible_curve_min_cones:
            return []

        bin_width = 4.0
        samples: list[tuple[float, float]] = [(0.0, 0.0)]
        start_x = 1.0
        while start_x < self.visible_curve_lookahead:
            end_x = min(self.visible_curve_lookahead, start_x + bin_width)
            group = [cone for cone in cones if start_x <= cone.position.x < end_x]
            if group:
                left = [cone.position.y for cone in group if cone.position.y > 0.15]
                right = [cone.position.y for cone in group if cone.position.y < -0.15]
                if left and right:
                    center_y = 0.5 * (median(left) + median(right))
                elif left:
                    center_y = median(left) - self.track_half_width
                elif right:
                    center_y = median(right) + self.track_half_width
                else:
                    center_y = 0.0
                center_x = median([cone.position.x for cone in group])
                samples.append((float(center_x), float(center_y)))
            start_x = end_x

        ordered = sorted(samples, key=lambda p: p[0])
        deduped: list[tuple[float, float]] = []
        for x, y in ordered:
            if deduped and abs(x - deduped[-1][0]) < 0.5:
                continue
            deduped.append((x, y))
        return deduped

    def visible_curve_speed(self) -> float | None:
        if not self.visible_curve_enabled:
            self.last_curve_status = "visible_curve=disabled"
            return None

        samples = self.visible_corridor_samples()
        if len(samples) < self.visible_curve_min_samples:
            self.last_curve_status = "visible_curve=insufficient"
            return None

        headings = []
        for i in range(1, len(samples)):
            dx = samples[i][0] - samples[i - 1][0]
            dy = samples[i][1] - samples[i - 1][1]
            if dx <= 0.2:
                continue
            headings.append(math.atan2(dy, dx))

        if not headings:
            self.last_curve_status = "visible_curve=insufficient"
            return None

        heading_severity = max(abs(angle) for angle in headings)
        bend_severity = 0.0
        for i in range(1, len(headings)):
            bend = abs(math.atan2(math.sin(headings[i] - headings[i - 1]), math.cos(headings[i] - headings[i - 1])))
            bend_severity = max(bend_severity, bend)

        lateral_sweep = max(abs(y) for _, y in samples[1:])
        lateral_severity = math.atan2(lateral_sweep, max(1.0, samples[-1][0]))
        severity = max(heading_severity, bend_severity, lateral_severity)

        span = max(0.01, self.visible_curve_hard_angle - self.visible_curve_straight_angle)
        curve_factor = clamp((severity - self.visible_curve_straight_angle) / span, 0.0, 1.0)
        speed = self.visible_straight_speed - curve_factor * (self.visible_straight_speed - self.visible_curve_min_speed)
        speed = clamp(speed, self.visible_curve_min_speed, self.visible_straight_speed)
        self.last_curve_status = (
            f"visible_curve speed={speed:.2f}mps angle={math.degrees(severity):.1f}deg samples={len(samples)}"
        )
        return speed

    def nominal_speed(self) -> float:
        if not self.mission_state.go_signal_fresh:
            return 0.0

        visible_speed = self.visible_curve_speed()

        if (
            self.mission_state.mode == "race_from_map"
            and self.track.quality >= self.map_quality_for_speed_profile
            and self.track.speed_profile
            and self.pose is not None
            and self.track.racing_line
        ):
            index = nearest_point_index(self.track.racing_line, self.pose.pose.position.x, self.pose.pose.position.y)
            if 0 <= index < len(self.track.speed_profile):
                planned_speed = min(float(self.track.speed_profile[index]), self.race_speed)
                return min(planned_speed, visible_speed) if visible_speed is not None else planned_speed

        if self.mission_state.mode == "race_from_map" and self.track.quality >= 0.35:
            return min(self.race_speed, visible_speed) if visible_speed is not None else self.race_speed

        quality_gain = clamp(
            self.track.quality / max(0.01, self.map_quality_for_full_mapping_speed),
            0.0,
            1.0,
        )
        cautious_speed = min(self.first_lap_speed, self.mapping_speed)
        mapping_speed = cautious_speed + (self.mapping_speed - cautious_speed) * quality_gain
        if visible_speed is None:
            return mapping_speed
        return clamp(visible_speed, self.visible_curve_min_speed, max(mapping_speed, self.visible_straight_speed))

    def startup_factor(self) -> float:
        if not self.mission_state.go_signal_fresh:
            self.go_start_time = None
            return 0.0
        now = self.get_clock().now()
        if self.go_start_time is None:
            self.go_start_time = now
        if self.startup_ramp_sec <= 0.0:
            return 1.0
        elapsed = (now - self.go_start_time).nanoseconds * 1e-9
        return clamp(elapsed / self.startup_ramp_sec, 0.20, 1.0)

    def rate_limit_speed(self, desired_speed: float) -> float:
        now = self.get_clock().now()
        dt = clamp((now - self.last_publish_time).nanoseconds * 1e-9, 0.0, 0.25)
        self.last_publish_time = now

        if desired_speed <= self.filtered_target_speed:
            limit = max(0.0, self.target_decel_limit) * dt
        else:
            limit = max(0.0, self.target_accel_limit) * dt

        delta = clamp(desired_speed - self.filtered_target_speed, -limit, limit)
        self.filtered_target_speed += delta
        if desired_speed <= 0.01 and self.filtered_target_speed < 0.05:
            self.filtered_target_speed = 0.0
        return self.filtered_target_speed

    def publish(self) -> None:
        emergency, obstacle_speed_cap, path_offset, status = self.obstacle_plan()
        nominal_speed = self.nominal_speed()
        if obstacle_speed_cap is not None:
            nominal_speed = min(nominal_speed, obstacle_speed_cap)
        desired_speed = 0.0 if emergency else nominal_speed * self.startup_factor()
        target_speed = self.rate_limit_speed(desired_speed)
        self.path_offset = 0.0 if emergency else path_offset
        if not self.mission_state.go_signal_fresh:
            behavior = "WaitingForGo"
            status = "waiting for GO"
            self.path_offset = 0.0
        elif emergency:
            behavior = "EmergencyBrake"
        elif abs(self.path_offset) > 0.05:
            behavior = "AvoidObstacle"
        elif obstacle_speed_cap is not None:
            behavior = "SlowForObstacle"
        elif self.mission_state.mode == "race_from_map":
            behavior = "FollowRaceline"
        elif self.last_curve_status.startswith("visible_curve speed="):
            behavior = "MapVisibleCurve"
        else:
            behavior = "MapCautious"

        if status == "clear":
            status = self.last_curve_status

        speed_msg = Float32()
        speed_msg.data = float(target_speed)
        self.speed_pub.publish(speed_msg)

        offset_msg = Float32()
        offset_msg.data = float(self.path_offset)
        self.path_offset_pub.publish(offset_msg)

        self.state_pub.publish(
            race_state_msg(
                self,
                self.mission_state.mission,
                self.mission_state.track_id,
                self.mission_state.mode,
                behavior,
                target_speed,
                self.path_offset,
                float(self.track.quality),
                self.mission_state.map_loaded,
                emergency,
                self.mission_state.go_signal_fresh,
                status,
            )
        )


def main() -> None:
    rclpy.init()
    rclpy.spin(BehaviorPlanner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

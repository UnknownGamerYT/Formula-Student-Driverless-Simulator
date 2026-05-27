from __future__ import annotations

import rclpy
from fsds_autonomy_msgs.msg import ConeArray, ObstacleArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray

from fsds_autonomy.constants import ConeColor, is_start_finish_color
from fsds_autonomy.geometry import distance_xy, greedy_order_points, point, pose2_from_pose_stamped, transform_local_to_global
from fsds_autonomy.visualization import (
    arrow_marker,
    cone_color,
    delete_all_marker,
    line_marker,
    path_msg,
    rgba,
    sphere_marker,
    text_marker,
)


class FoxgloveVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("fsds_foxglove_visualizer")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_local_cones", 80)

        self.track_map = TrackMap()
        self.racing_map = TrackMap()
        self.local_cones = ConeArray()
        self.obstacles = ObstacleArray()
        self.pose = None
        self.race_state = RaceState()
        self.path_offset = 0.0

        self.marker_pub = self.create_publisher(MarkerArray, "/autonomy/viz/map_markers", 10)
        self.centerline_pub = self.create_publisher(Path, "/autonomy/viz/map_centerline_path", 10)
        self.racing_line_pub = self.create_publisher(Path, "/autonomy/viz/optimal_racing_line_path", 10)
        self.current_line_pub = self.create_publisher(Path, "/autonomy/viz/current_drive_line_path", 10)

        self.create_subscription(TrackMap, "/autonomy/track_map", self.on_track_map, 10)
        self.create_subscription(TrackMap, "/autonomy/racing_line", self.on_racing_map, 10)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_local_cones, 10)
        self.create_subscription(ObstacleArray, "/autonomy/obstacles", self.on_obstacles, 10)
        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)
        self.create_subscription(RaceState, "/autonomy/race_state", self.on_race_state, 10)
        self.create_subscription(Float32, "/autonomy/path_offset", self.on_path_offset, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish_visuals)

    def on_track_map(self, msg: TrackMap) -> None:
        self.track_map = msg

    def on_racing_map(self, msg: TrackMap) -> None:
        self.racing_map = msg

    def on_local_cones(self, msg: ConeArray) -> None:
        self.local_cones = msg

    def on_obstacles(self, msg: ObstacleArray) -> None:
        self.obstacles = msg

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def on_race_state(self, msg: RaceState) -> None:
        self.race_state = msg

    def on_path_offset(self, msg: Float32) -> None:
        self.path_offset = float(msg.data)

    def current_points(self):
        if self.racing_map.racing_line:
            return self.racing_map.racing_line
        if self.track_map.centerline:
            return self.track_map.centerline
        return []

    def publish_visuals(self) -> None:
        header = self.get_header()
        markers = MarkerArray()
        markers.markers.append(delete_all_marker(header))
        marker_id = 1

        for cone in self.track_map.cones:
            markers.markers.append(
                sphere_marker(
                    header,
                    marker_id,
                    "created_map_cones",
                    cone.position,
                    cone_color(cone.color, 0.95),
                    diameter=0.38,
                )
            )
            marker_id += 1
            if is_start_finish_color(cone.color):
                markers.markers.append(
                    text_marker(
                        header,
                        marker_id,
                        "start_finish_gate_text",
                        "START/END",
                        cone.position.x,
                        cone.position.y,
                        0.85,
                        rgba(1.0, 0.62, 0.05, 0.95),
                    )
                )
                marker_id += 1

        if self.pose is not None:
            pose2 = pose2_from_pose_stamped(self.pose)
            for index, cone in enumerate(self.local_cones.cones[: int(self.get_parameter("max_local_cones").value)]):
                gx, gy = transform_local_to_global(cone.position.x, cone.position.y, pose2)
                markers.markers.append(
                    sphere_marker(
                        header,
                        marker_id,
                        "live_fused_cones",
                        point(gx, gy, 0.0),
                        cone_color(cone.color, 0.45),
                        diameter=0.22,
                    )
                )
                marker_id += 1

            for obstacle in self.obstacles.obstacles:
                if obstacle.position.x < 0.0 or obstacle.position.x > 25.0:
                    continue
                gx, gy = transform_local_to_global(obstacle.position.x, obstacle.position.y, pose2)
                markers.markers.append(
                    sphere_marker(
                        header,
                        marker_id,
                        "live_obstacles",
                        point(gx, gy, 0.0),
                        rgba(1.0, 0.05, 0.02, 0.85),
                        diameter=max(0.4, obstacle.radius * 2.0),
                    )
                )
                marker_id += 1

            if abs(self.path_offset) > 0.05:
                avoid_points = []
                for local_x in (0.0, 3.0, 6.0, 9.0, 12.0):
                    gx, gy = transform_local_to_global(local_x, self.path_offset, pose2)
                    avoid_points.append(point(gx, gy, 0.0))
                markers.markers.append(
                    line_marker(
                        header,
                        marker_id,
                        "obstacle_avoid_line",
                        avoid_points,
                        rgba(0.0, 1.0, 0.2, 0.95),
                        width=0.18,
                        z_offset=0.22,
                    )
                )
                marker_id += 1

            markers.markers.append(
                arrow_marker(
                    header,
                    marker_id,
                    "vehicle_pose",
                    pose2.x,
                    pose2.y,
                    pose2.yaw,
                    rgba(1.0, 1.0, 1.0, 1.0),
                )
            )
            marker_id += 1
            markers.markers.append(
                text_marker(
                    header,
                    marker_id,
                    "race_state_text",
                    f"{self.race_state.behavior_state or 'Waiting'} | {self.race_state.target_speed:.1f} m/s | map {self.track_map.quality:.2f}",
                    pose2.x,
                    pose2.y,
                    1.4,
                    rgba(1.0, 1.0, 1.0, 0.95),
                )
            )
            marker_id += 1

        boundary_lines = self.map_boundary_lines()
        if len(boundary_lines[ConeColor.BLUE]) >= 2:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "created_map_left_blue_boundary",
                    boundary_lines[ConeColor.BLUE],
                    cone_color(ConeColor.BLUE, 0.95),
                    width=0.13,
                    z_offset=0.18,
                )
            )
            marker_id += 1

        if len(boundary_lines[ConeColor.YELLOW]) >= 2:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "created_map_right_yellow_boundary",
                    boundary_lines[ConeColor.YELLOW],
                    cone_color(ConeColor.YELLOW, 0.95),
                    width=0.13,
                    z_offset=0.18,
                )
            )
            marker_id += 1

        if self.track_map.centerline:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "created_map_centerline",
                    self.track_map.centerline,
                    rgba(0.0, 0.9, 1.0, 0.9),
                    width=0.08,
                    z_offset=0.08,
                )
            )
            marker_id += 1

        if self.racing_map.racing_line:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "optimal_racing_line",
                    self.racing_map.racing_line,
                    rgba(1.0, 0.0, 0.9, 0.95),
                    width=0.11,
                    z_offset=0.11,
                )
            )
            marker_id += 1

        current = self.current_points()
        if current:
            markers.markers.append(
                line_marker(
                    header,
                    marker_id,
                    "current_drive_line",
                    current,
                    rgba(1.0, 1.0, 0.0, 0.95),
                    width=0.16,
                    z_offset=0.16,
                )
            )

        self.marker_pub.publish(markers)
        self.centerline_pub.publish(path_msg(header, self.track_map.centerline))
        self.racing_line_pub.publish(path_msg(header, self.racing_map.racing_line))
        self.current_line_pub.publish(path_msg(header, current))

    def map_boundary_lines(self) -> dict[int, list]:
        groups = {ConeColor.BLUE: [], ConeColor.YELLOW: []}
        start_finish = []
        for cone in self.track_map.cones:
            color = int(cone.color)
            cone_point = point(cone.position.x, cone.position.y, 0.0)
            if color == ConeColor.BLUE:
                groups[ConeColor.BLUE].append(cone_point)
            elif color == ConeColor.YELLOW:
                groups[ConeColor.YELLOW].append(cone_point)
            elif is_start_finish_color(color):
                start_finish.append(cone_point)

        for cone_point in start_finish:
            blue_dist = self.nearest_distance(cone_point, groups[ConeColor.BLUE])
            yellow_dist = self.nearest_distance(cone_point, groups[ConeColor.YELLOW])
            if blue_dist <= yellow_dist:
                groups[ConeColor.BLUE].append(cone_point)
            else:
                groups[ConeColor.YELLOW].append(cone_point)

        return {
            ConeColor.BLUE: greedy_order_points(groups[ConeColor.BLUE]),
            ConeColor.YELLOW: greedy_order_points(groups[ConeColor.YELLOW]),
        }

    @staticmethod
    def nearest_distance(cone_point, boundary_points) -> float:
        if not boundary_points:
            return float("inf")
        return min(distance_xy(cone_point, boundary_point) for boundary_point in boundary_points)

    def get_header(self):
        header = self.track_map.header
        if not header.frame_id:
            header.frame_id = "fsds/map"
        header.stamp = self.get_clock().now().to_msg()
        return header


def main() -> None:
    rclpy.init()
    rclpy.spin(FoxgloveVisualizer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

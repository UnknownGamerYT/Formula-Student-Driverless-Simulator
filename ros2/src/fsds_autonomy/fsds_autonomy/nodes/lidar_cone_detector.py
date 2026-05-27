from __future__ import annotations

import math

import rclpy
from fsds_autonomy_msgs.msg import ConeArray, ObstacleArray
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from fsds_autonomy.constants import DEFAULT_LIDAR_OFFSETS, ConeColor
from fsds_autonomy.perception import (
    filter_lidar_points,
    ordered_clusters_from_points,
    rotate_translate_xy,
)
from fsds_autonomy.ros_utils import cone_msg, make_header, obstacle_msg


class LidarConeDetector(Node):
    def __init__(self) -> None:
        super().__init__("fsds_lidar_cone_detector")
        self.declare_parameter("lidar1_topic", "/fsds/lidar/Lidar1")
        self.declare_parameter("lidar2_topic", "/fsds/lidar/Lidar2")
        self.declare_parameter("cluster_tolerance", 0.22)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("max_range_m", 25.0)
        self.declare_parameter("max_abs_y_m", 12.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.cluster_tolerance = float(self.get_parameter("cluster_tolerance").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.max_abs_y_m = float(self.get_parameter("max_abs_y_m").value)

        self.latest_cones_by_source = {}
        self.latest_obstacles_by_source = {}
        self.cones_pub = self.create_publisher(ConeArray, "/autonomy/lidar_cones", 10)
        self.obstacles_pub = self.create_publisher(ObstacleArray, "/autonomy/obstacles", 10)

        for lidar_name, topic_param in (("Lidar1", "lidar1_topic"), ("Lidar2", "lidar2_topic")):
            topic = str(self.get_parameter(topic_param).value)
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, name=lidar_name: self.on_cloud(msg, name),
                10,
            )

        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish)

    def points_from_cloud(self, msg: PointCloud2):
        for row in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            yield float(row[0]), float(row[1]), float(row[2])

    def on_cloud(self, msg: PointCloud2, lidar_name: str) -> None:
        points = filter_lidar_points(
            self.points_from_cloud(msg),
            max_x=self.max_range_m,
            max_abs_y=self.max_abs_y_m,
        )
        offset = DEFAULT_LIDAR_OFFSETS[lidar_name]
        points = rotate_translate_xy(points, offset.x, offset.y, math.radians(offset.yaw))
        cones, obstacles = ordered_clusters_from_points(
            points,
            cluster_tolerance=self.cluster_tolerance,
            min_points=self.min_cluster_points,
        )

        self.latest_cones_by_source[lidar_name] = cones
        self.latest_obstacles_by_source[lidar_name] = obstacles

    def publish(self) -> None:
        header = make_header(self, "fsds/FSCar")
        cone_array = ConeArray()
        cone_array.header = header
        for source, cones in self.latest_cones_by_source.items():
            for cone in cones:
                if cone.range < 0.3:
                    continue
                cone_array.cones.append(
                    cone_msg(
                        header,
                        cone.x,
                        cone.y,
                        cone.z,
                        ConeColor.UNKNOWN,
                        cone.confidence,
                        source,
                    )
                )
        self.cones_pub.publish(cone_array)

        obstacle_array = ObstacleArray()
        obstacle_array.header = header
        for source, obstacles in self.latest_obstacles_by_source.items():
            for obstacle in obstacles:
                if obstacle.x < 0.5:
                    continue
                radius = max(0.25, 0.5 * max(obstacle.width, obstacle.depth))
                obstacle_array.obstacles.append(
                    obstacle_msg(
                        header,
                        obstacle.x,
                        obstacle.y,
                        radius,
                        obstacle.confidence,
                        "unknown_cluster",
                        source,
                    )
                )
        self.obstacles_pub.publish(obstacle_array)


def main() -> None:
    rclpy.init()
    rclpy.spin(LidarConeDetector())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32

from fsds_autonomy.geometry import quaternion_from_yaw, yaw_from_quaternion


class StateEstimator(Node):
    def __init__(self) -> None:
        super().__init__("fsds_state_estimator")
        self.declare_parameter("use_testing_odom", False)
        self.declare_parameter("publish_rate_hz", 50.0)

        self.use_testing_odom = bool(self.get_parameter("use_testing_odom").value)
        self.anchor_lat = None
        self.anchor_lon = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.last_integrate_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseStamped, "/autonomy/pose", 10)
        self.speed_pub = self.create_publisher(Float32, "/autonomy/speed", 10)

        self.create_subscription(Imu, "/fsds/imu", self.on_imu, 50)
        self.create_subscription(NavSatFix, "/fsds/gps", self.on_gps, 10)
        self.create_subscription(TwistWithCovarianceStamped, "/fsds/gss", self.on_gss, 20)
        if self.use_testing_odom:
            self.create_subscription(Odometry, "/fsds/testing_only/odom", self.on_testing_odom, 20)

        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish)

    def on_imu(self, msg: Imu) -> None:
        self.yaw = yaw_from_quaternion(msg.orientation)

    def on_gps(self, msg: NavSatFix) -> None:
        if self.use_testing_odom:
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if self.anchor_lat is None:
            self.anchor_lat = msg.latitude
            self.anchor_lon = msg.longitude
            return
        earth_radius = 6378137.0
        lat0 = math.radians(self.anchor_lat)
        self.x = math.radians(msg.latitude - self.anchor_lat) * earth_radius
        self.y = math.radians(msg.longitude - self.anchor_lon) * earth_radius * math.cos(lat0)

    def on_gss(self, msg: TwistWithCovarianceStamped) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.hypot(vx, vy)
        if self.use_testing_odom:
            return
        now = self.get_clock().now()
        dt = max(0.0, min(0.1, (now - self.last_integrate_time).nanoseconds * 1e-9))
        self.last_integrate_time = now
        self.x += math.cos(self.yaw) * self.speed * dt
        self.y += math.sin(self.yaw) * self.speed * dt

    def on_testing_odom(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def publish(self) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "fsds/map"
        pose.pose.position.x = float(self.x)
        pose.pose.position.y = float(self.y)
        pose.pose.orientation = quaternion_from_yaw(self.yaw)
        self.pose_pub.publish(pose)

        speed = Float32()
        speed.data = float(self.speed)
        self.speed_pub.publish(speed)


def main() -> None:
    rclpy.init()
    rclpy.spin(StateEstimator())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

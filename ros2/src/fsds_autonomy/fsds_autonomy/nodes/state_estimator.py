from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from fsds_autonomy.geometry import quaternion_from_yaw, yaw_from_quaternion


class StateEstimator(Node):
    def __init__(self) -> None:
        super().__init__("fsds_state_estimator")
        self.declare_parameter("use_testing_odom", False)
        self.declare_parameter("use_gps_position", True)
        self.declare_parameter("gps_correction_alpha", 0.65)
        self.declare_parameter("gps_max_std_m", 5.0)
        self.declare_parameter("gps_max_jump_m", 25.0)
        self.declare_parameter("gps_use_configured_origin", False)
        self.declare_parameter("gps_origin_latitude", 0.0)
        self.declare_parameter("gps_origin_longitude", 0.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("map_frame_id", "fsds/map")
        self.declare_parameter("vehicle_frame_id", "fsds/FSCar")
        self.declare_parameter("publish_rate_hz", 50.0)

        self.use_testing_odom = bool(self.get_parameter("use_testing_odom").value)
        self.use_gps_position = bool(self.get_parameter("use_gps_position").value)
        self.gps_correction_alpha = float(self.get_parameter("gps_correction_alpha").value)
        self.gps_max_std = float(self.get_parameter("gps_max_std_m").value)
        self.gps_max_jump = float(self.get_parameter("gps_max_jump_m").value)
        self.gps_use_configured_origin = bool(self.get_parameter("gps_use_configured_origin").value)
        self.anchor_lat = None
        self.anchor_lon = None
        if self.gps_use_configured_origin:
            self.anchor_lat = float(self.get_parameter("gps_origin_latitude").value)
            self.anchor_lon = float(self.get_parameter("gps_origin_longitude").value)
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.map_frame_id = str(self.get_parameter("map_frame_id").value)
        self.vehicle_frame_id = str(self.get_parameter("vehicle_frame_id").value)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.latest_gps_x = None
        self.latest_gps_y = None
        self.gps_initialized_pose = False
        self.last_integrate_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseStamped, "/autonomy/pose", 10)
        self.gps_pose_pub = self.create_publisher(PoseStamped, "/autonomy/gps_pose", 10)
        self.speed_pub = self.create_publisher(Float32, "/autonomy/speed", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf_enabled else None

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
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if not self.gps_fix_usable(msg):
            return
        if self.anchor_lat is None:
            self.anchor_lat = msg.latitude
            self.anchor_lon = msg.longitude

        gps_x, gps_y = self.gps_to_local_xy(msg.latitude, msg.longitude)
        self.latest_gps_x = gps_x
        self.latest_gps_y = gps_y
        self.publish_gps_pose(msg, gps_x, gps_y)

        if self.use_testing_odom or not self.use_gps_position:
            return

        if not self.gps_initialized_pose:
            self.x = gps_x
            self.y = gps_y
            self.gps_initialized_pose = True
            return

        jump = math.hypot(gps_x - self.x, gps_y - self.y)
        if jump > self.gps_max_jump:
            self.get_logger().warn(
                f"Ignoring GPS correction jump={jump:.1f}m > {self.gps_max_jump:.1f}m",
                throttle_duration_sec=2.0,
            )
            return

        alpha = max(0.0, min(1.0, self.gps_correction_alpha))
        self.x = (1.0 - alpha) * self.x + alpha * gps_x
        self.y = (1.0 - alpha) * self.y + alpha * gps_y

    def gps_fix_usable(self, msg: NavSatFix) -> bool:
        cov = list(msg.position_covariance)
        if len(cov) >= 5 and cov[0] > 0.0 and cov[4] > 0.0:
            horizontal_std = max(math.sqrt(max(0.0, cov[0])), math.sqrt(max(0.0, cov[4])))
            if horizontal_std > self.gps_max_std:
                self.get_logger().warn(
                    f"Ignoring GPS fix std={horizontal_std:.2f}m > {self.gps_max_std:.2f}m",
                    throttle_duration_sec=2.0,
                )
                return False
        return True

    def gps_to_local_xy(self, latitude: float, longitude: float) -> tuple[float, float]:
        earth_radius = 6378137.0
        lat0 = math.radians(float(self.anchor_lat))
        east = math.radians(longitude - float(self.anchor_lon)) * earth_radius * math.cos(lat0)
        north = math.radians(latitude - float(self.anchor_lat)) * earth_radius
        return east, north

    def publish_gps_pose(self, msg: NavSatFix, x: float, y: float) -> None:
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = quaternion_from_yaw(self.yaw)
        self.gps_pose_pub.publish(pose)

    def on_gss(self, msg: TwistWithCovarianceStamped) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.hypot(vx, vy)
        if self.use_testing_odom:
            return
        now = self.get_clock().now()
        dt = max(0.0, min(0.1, (now - self.last_integrate_time).nanoseconds * 1e-9))
        self.last_integrate_time = now
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (cos_yaw * vx - sin_yaw * vy) * dt
        self.y += (sin_yaw * vx + cos_yaw * vy) * dt

    def on_testing_odom(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.gps_initialized_pose = False

    def publish(self) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame_id
        pose.pose.position.x = float(self.x)
        pose.pose.position.y = float(self.y)
        pose.pose.orientation = quaternion_from_yaw(self.yaw)
        self.pose_pub.publish(pose)
        self.publish_vehicle_tf(pose)

        speed = Float32()
        speed.data = float(self.speed)
        self.speed_pub.publish(speed)

    def publish_vehicle_tf(self, pose: PoseStamped) -> None:
        if self.tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header.stamp = pose.header.stamp
        transform.header.frame_id = self.map_frame_id
        transform.child_frame_id = self.vehicle_frame_id
        transform.transform.translation.x = float(pose.pose.position.x)
        transform.transform.translation.y = float(pose.pose.position.y)
        transform.transform.translation.z = float(pose.pose.position.z)
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    rclpy.spin(StateEstimator())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

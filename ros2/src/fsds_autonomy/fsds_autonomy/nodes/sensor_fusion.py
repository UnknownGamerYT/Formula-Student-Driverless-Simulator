from __future__ import annotations

import math

import rclpy
from fsds_autonomy_msgs.msg import ConeArray
from rclpy.node import Node
from std_msgs.msg import String

from fsds_autonomy.constants import COLOR_TO_CONE_CLASS, ConeColor, STEREO_CAMERA_SOURCE
from fsds_autonomy.perception import fuse_color
from fsds_autonomy.ros_utils import cone_msg, make_header


class SensorFusion(Node):
    def __init__(self) -> None:
        super().__init__("fsds_sensor_fusion")
        self.declare_parameter("bearing_gate_rad", 0.18)
        self.declare_parameter("range_gate_m", 5.0)
        self.declare_parameter("include_camera_only_geometry", False)
        self.declare_parameter("include_stereo_camera_geometry", True)
        self.declare_parameter("camera_only_min_confidence", 0.65)
        self.declare_parameter("stereo_camera_min_confidence", 0.45)
        self.declare_parameter("stereo_position_gate_m", 2.0)
        self.declare_parameter("stereo_lidar_position_blend", 0.35)
        self.declare_parameter("diagnostics_enabled", True)
        self.declare_parameter("diagnostics_publish_rate_hz", 5.0)
        self.declare_parameter("diagnostics_warn_distance_m", 0.75)
        self.declare_parameter("diagnostics_error_distance_m", 1.50)
        self.declare_parameter("diagnostics_max_rows", 12)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.bearing_gate = float(self.get_parameter("bearing_gate_rad").value)
        self.range_gate = float(self.get_parameter("range_gate_m").value)
        self.include_camera_only_geometry = bool(self.get_parameter("include_camera_only_geometry").value)
        self.include_stereo_camera_geometry = bool(self.get_parameter("include_stereo_camera_geometry").value)
        self.camera_only_min_confidence = float(self.get_parameter("camera_only_min_confidence").value)
        self.stereo_camera_min_confidence = float(self.get_parameter("stereo_camera_min_confidence").value)
        self.stereo_position_gate = float(self.get_parameter("stereo_position_gate_m").value)
        self.stereo_lidar_position_blend = float(self.get_parameter("stereo_lidar_position_blend").value)
        self.diagnostics_enabled = bool(self.get_parameter("diagnostics_enabled").value)
        self.diagnostics_publish_rate = float(self.get_parameter("diagnostics_publish_rate_hz").value)
        self.diagnostics_warn_distance = float(self.get_parameter("diagnostics_warn_distance_m").value)
        self.diagnostics_error_distance = float(self.get_parameter("diagnostics_error_distance_m").value)
        self.diagnostics_max_rows = int(self.get_parameter("diagnostics_max_rows").value)
        self.latest_lidar = ConeArray()
        self.latest_camera = ConeArray()
        self.last_diagnostics_time = self.get_clock().now()

        self.pub = self.create_publisher(ConeArray, "/autonomy/fused_cones", 10)
        self.diagnostics_pub = self.create_publisher(String, "/autonomy/sensor_fusion_diagnostics", 10)
        self.create_subscription(ConeArray, "/autonomy/lidar_cones", self.on_lidar, 10)
        self.create_subscription(ConeArray, "/autonomy/camera_cones", self.on_camera, 10)
        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish)

    def is_stereo_cone(self, cone) -> bool:
        return STEREO_CAMERA_SOURCE in cone.source

    def position_distance(self, first, second) -> float:
        return math.hypot(
            float(first.position.x) - float(second.position.x),
            float(first.position.y) - float(second.position.y),
        )

    def bearing_delta(self, first, second) -> float:
        return math.atan2(
            math.sin(float(first.bearing) - float(second.bearing)),
            math.cos(float(first.bearing) - float(second.bearing)),
        )

    def match_score(self, lidar_cone, camera_cone) -> float | None:
        bearing_diff = abs(
            math.atan2(
                math.sin(lidar_cone.bearing - camera_cone.bearing),
                math.cos(lidar_cone.bearing - camera_cone.bearing),
            )
        )
        range_diff = abs(lidar_cone.range - camera_cone.range)
        if bearing_diff > self.bearing_gate or range_diff > self.range_gate:
            return None

        if self.is_stereo_cone(camera_cone):
            position_diff = self.position_distance(lidar_cone, camera_cone)
            if position_diff > self.stereo_position_gate:
                return None
            return 0.45 * bearing_diff + 0.03 * range_diff + 0.20 * position_diff

        return bearing_diff + 0.05 * range_diff

    def fused_lidar_stereo_position(self, lidar_cone, camera_cone) -> tuple[float, float, float]:
        if not self.is_stereo_cone(camera_cone):
            return lidar_cone.position.x, lidar_cone.position.y, lidar_cone.position.z

        max_blend = max(0.0, min(0.65, self.stereo_lidar_position_blend))
        camera_strength = camera_cone.confidence / max(1e-6, camera_cone.confidence + lidar_cone.confidence)
        stereo_weight = max_blend * min(1.0, 2.0 * camera_strength)
        lidar_weight = 1.0 - stereo_weight
        return (
            lidar_weight * lidar_cone.position.x + stereo_weight * camera_cone.position.x,
            lidar_weight * lidar_cone.position.y + stereo_weight * camera_cone.position.y,
            lidar_weight * lidar_cone.position.z + stereo_weight * camera_cone.position.z,
        )

    def nearest_lidar(self, stereo_cone):
        best = None
        best_index = -1
        best_distance = float("inf")
        for index, lidar_cone in enumerate(self.latest_lidar.cones):
            distance = self.position_distance(lidar_cone, stereo_cone)
            if distance < best_distance:
                best = lidar_cone
                best_index = index
                best_distance = distance
        return best_index, best, best_distance

    def diagnostic_status(self, distance: float) -> str:
        if distance >= self.diagnostics_error_distance:
            return "ERROR"
        if distance >= self.diagnostics_warn_distance:
            return "WARN"
        return "OK"

    def color_name(self, color: int) -> str:
        return COLOR_TO_CONE_CLASS.get(int(color), "unknown_cone")

    def publish_diagnostics(self, matched_camera_indices: set[int], fused_camera_indices: set[int]) -> None:
        if not self.diagnostics_enabled:
            return

        now = self.get_clock().now()
        min_period = 1.0 / max(0.1, self.diagnostics_publish_rate)
        if (now - self.last_diagnostics_time).nanoseconds * 1e-9 < min_period:
            return
        self.last_diagnostics_time = now

        stereo = [
            (index, cone)
            for index, cone in enumerate(self.latest_camera.cones)
            if self.is_stereo_cone(cone)
        ]
        rows = []
        status_counts = {"OK": 0, "WARN": 0, "ERROR": 0, "NO_LIDAR": 0}
        max_error = 0.0

        for stereo_index, stereo_cone in stereo:
            lidar_index, lidar_cone, position_error = self.nearest_lidar(stereo_cone)
            if lidar_cone is None:
                status_counts["NO_LIDAR"] += 1
                rows.append(
                    (
                        float("inf"),
                        (
                            f"NO_LIDAR stereo[{stereo_index}] fused={stereo_index in fused_camera_indices} "
                            f"stereo=({stereo_cone.position.x:.2f},{stereo_cone.position.y:.2f}) "
                            f"r={stereo_cone.range:.2f}m b={math.degrees(stereo_cone.bearing):+.1f}deg "
                            f"conf={stereo_cone.confidence:.2f} color={self.color_name(stereo_cone.color)}"
                        ),
                    )
                )
                continue

            dx = stereo_cone.position.x - lidar_cone.position.x
            dy = stereo_cone.position.y - lidar_cone.position.y
            dr = stereo_cone.range - lidar_cone.range
            db = math.degrees(self.bearing_delta(stereo_cone, lidar_cone))
            status = self.diagnostic_status(position_error)
            status_counts[status] += 1
            max_error = max(max_error, position_error)
            rows.append(
                (
                    position_error,
                    (
                        f"{status} stereo[{stereo_index}] nearest_lidar[{lidar_index}] "
                        f"matched={stereo_index in matched_camera_indices} fused={stereo_index in fused_camera_indices} "
                        f"dist_error={position_error:.2f}m dx={dx:+.2f}m dy={dy:+.2f}m "
                        f"range_error={dr:+.2f}m bearing_error={db:+.1f}deg | "
                        f"lidar={lidar_cone.source} ({lidar_cone.position.x:.2f},{lidar_cone.position.y:.2f}) "
                        f"r={lidar_cone.range:.2f}m b={math.degrees(lidar_cone.bearing):+.1f}deg conf={lidar_cone.confidence:.2f} | "
                        f"stereo=({stereo_cone.position.x:.2f},{stereo_cone.position.y:.2f}) "
                        f"r={stereo_cone.range:.2f}m b={math.degrees(stereo_cone.bearing):+.1f}deg "
                        f"conf={stereo_cone.confidence:.2f} color={self.color_name(stereo_cone.color)}"
                    ),
                )
            )

        overall = "OK"
        if status_counts["NO_LIDAR"] and not self.latest_lidar.cones:
            overall = "NO_LIDAR"
        if status_counts["WARN"]:
            overall = "WARN"
        if status_counts["ERROR"]:
            overall = "ERROR"
        if not stereo:
            overall = "NO_STEREO"

        lines = [
            (
                f"status={overall} lidar={len(self.latest_lidar.cones)} stereo={len(stereo)} "
                f"matched={len(matched_camera_indices)} fused_stereo={len(fused_camera_indices)} "
                f"ok={status_counts['OK']} warn={status_counts['WARN']} error={status_counts['ERROR']} "
                f"no_lidar={status_counts['NO_LIDAR']} max_error={max_error:.2f}m "
                f"warn_at={self.diagnostics_warn_distance:.2f}m error_at={self.diagnostics_error_distance:.2f}m"
            )
        ]
        rows.sort(key=lambda item: item[0], reverse=True)
        lines.extend(row for _, row in rows[: max(0, self.diagnostics_max_rows)])

        msg = String()
        msg.data = "\n".join(lines)
        self.diagnostics_pub.publish(msg)
        if overall == "ERROR":
            self.get_logger().warn(lines[0], throttle_duration_sec=2.0)

    def on_lidar(self, msg: ConeArray) -> None:
        self.latest_lidar = msg

    def on_camera(self, msg: ConeArray) -> None:
        self.latest_camera = msg

    def publish(self) -> None:
        header = make_header(self, "fsds/FSCar")
        fused = ConeArray()
        fused.header = header

        used_camera = set()
        fused_camera_indices = set()
        for lidar_cone in self.latest_lidar.cones:
            best_index = None
            best_score = float("inf")
            for index, camera_cone in enumerate(self.latest_camera.cones):
                score = self.match_score(lidar_cone, camera_cone)
                if score is not None and score < best_score:
                    best_index = index
                    best_score = score

            color = lidar_cone.color
            confidence = lidar_cone.confidence
            source = lidar_cone.source
            x = lidar_cone.position.x
            y = lidar_cone.position.y
            z = lidar_cone.position.z
            if best_index is not None:
                camera_cone = self.latest_camera.cones[best_index]
                used_camera.add(best_index)
                if self.is_stereo_cone(camera_cone):
                    fused_camera_indices.add(best_index)
                color = fuse_color(color, confidence, camera_cone.color, camera_cone.confidence)
                x, y, z = self.fused_lidar_stereo_position(lidar_cone, camera_cone)
                confidence_bonus = 0.20 if self.is_stereo_cone(camera_cone) else 0.15
                confidence = min(1.0, max(confidence, camera_cone.confidence) + confidence_bonus)
                source = f"{lidar_cone.source}+{camera_cone.source}"

            fused.cones.append(
                cone_msg(
                    header,
                    x,
                    y,
                    z,
                    color,
                    confidence,
                    source,
                )
            )

        for index, camera_cone in enumerate(self.latest_camera.cones):
            is_stereo = self.is_stereo_cone(camera_cone)
            if not self.include_camera_only_geometry and not (
                is_stereo and self.include_stereo_camera_geometry
            ):
                continue
            if index in used_camera:
                continue
            if camera_cone.color == ConeColor.UNKNOWN:
                continue
            min_confidence = self.stereo_camera_min_confidence if is_stereo else self.camera_only_min_confidence
            if camera_cone.confidence < min_confidence:
                continue
            confidence_scale = 0.85 if is_stereo else 0.6
            if is_stereo:
                fused_camera_indices.add(index)
            fused.cones.append(
                cone_msg(
                    header,
                    camera_cone.position.x,
                    camera_cone.position.y,
                    camera_cone.position.z,
                    camera_cone.color,
                    max(0.15, camera_cone.confidence * confidence_scale),
                    camera_cone.source,
                )
            )

        self.pub.publish(fused)
        self.publish_diagnostics(used_camera, fused_camera_indices)


def main() -> None:
    rclpy.init()
    rclpy.spin(SensorFusion())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

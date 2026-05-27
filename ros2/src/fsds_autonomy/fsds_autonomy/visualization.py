from __future__ import annotations

import math
from typing import Iterable, Sequence

from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from fsds_autonomy.constants import ConeColor, is_start_finish_color
from fsds_autonomy.geometry import point, quaternion_from_yaw


def rgba(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    color = ColorRGBA()
    color.r = float(r)
    color.g = float(g)
    color.b = float(b)
    color.a = float(a)
    return color


def scale(x: float, y: float, z: float) -> Vector3:
    msg = Vector3()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


def cone_color(color: int, alpha: float = 1.0) -> ColorRGBA:
    if color == ConeColor.YELLOW:
        return rgba(1.0, 0.85, 0.05, alpha)
    if color == ConeColor.BLUE:
        return rgba(0.05, 0.25, 1.0, alpha)
    if is_start_finish_color(color):
        return rgba(1.0, 0.38, 0.02, alpha)
    return rgba(0.72, 0.72, 0.72, alpha)


def line_marker(
    header: Header,
    marker_id: int,
    ns: str,
    points: Sequence[Point],
    color: ColorRGBA,
    width: float,
    z_offset: float = 0.06,
) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = float(width)
    marker.color = color
    marker.pose.orientation.w = 1.0
    marker.points = [point(p.x, p.y, z_offset) for p in points]
    return marker


def sphere_marker(
    header: Header,
    marker_id: int,
    ns: str,
    position: Point,
    color: ColorRGBA,
    diameter: float = 0.32,
    z_offset: float = 0.16,
) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = float(position.x)
    marker.pose.position.y = float(position.y)
    marker.pose.position.z = float(position.z) + z_offset
    marker.pose.orientation.w = 1.0
    marker.scale = scale(diameter, diameter, diameter)
    marker.color = color
    return marker


def arrow_marker(
    header: Header,
    marker_id: int,
    ns: str,
    x: float,
    y: float,
    yaw: float,
    color: ColorRGBA,
) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = 0.25
    marker.pose.orientation = quaternion_from_yaw(yaw)
    marker.scale = scale(1.2, 0.25, 0.25)
    marker.color = color
    return marker


def text_marker(
    header: Header,
    marker_id: int,
    ns: str,
    text: str,
    x: float,
    y: float,
    z: float,
    color: ColorRGBA,
) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position = point(x, y, z)
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.65
    marker.color = color
    marker.text = text
    return marker


def delete_all_marker(header: Header) -> Marker:
    marker = Marker()
    marker.header = header
    marker.action = Marker.DELETEALL
    return marker


def path_msg(header: Header, points: Iterable[Point]) -> Path:
    msg = Path()
    msg.header = header
    for p in points:
        pose = PoseStamped()
        pose.header = header
        pose.pose.position = point(p.x, p.y, p.z)
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def bgr_for_cone(color: int) -> tuple[int, int, int]:
    if color == ConeColor.YELLOW:
        return (0, 220, 255)
    if color == ConeColor.BLUE:
        return (255, 80, 20)
    if is_start_finish_color(color):
        return (0, 130, 255)
    return (180, 180, 180)


def project_car_point_to_camera(
    local_x: float,
    local_y: float,
    local_z: float,
    image_width: int,
    image_height: int,
    camera_x: float,
    camera_y: float,
    camera_z: float,
    fov_deg: float,
) -> tuple[int, int] | None:
    cam_x = local_x - camera_x
    cam_y = local_y - camera_y
    cam_z = local_z - camera_z
    if cam_x <= 0.35:
        return None
    focal = image_width / (2.0 * math.tan(math.radians(fov_deg) * 0.5))
    u = image_width * 0.5 - focal * cam_y / cam_x
    v = image_height * 0.5 - focal * cam_z / cam_x
    if u < -image_width or u > 2.0 * image_width or v < -image_height or v > 2.0 * image_height:
        return None
    return int(round(u)), int(round(v))

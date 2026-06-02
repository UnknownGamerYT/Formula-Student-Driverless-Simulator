from __future__ import annotations

import math
from pathlib import Path

from builtin_interfaces.msg import Time
from fsds_autonomy_msgs.msg import (
    DetectedCone,
    DetectedObstacle,
    RaceState,
    TrackMap,
)
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from fsds_autonomy.constants import ConeColor
from fsds_autonomy.geometry import point
from fsds_autonomy.map_store import ConeLandmark, SavedTrackMap


def make_header(node, frame_id: str = "fsds/map") -> Header:
    header = Header()
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = frame_id
    return header


def cone_msg(
    header: Header,
    x: float,
    y: float,
    z: float = 0.0,
    color: int = ConeColor.UNKNOWN,
    confidence: float = 0.0,
    source: str = "",
) -> DetectedCone:
    msg = DetectedCone()
    msg.header = header
    msg.position = point(x, y, z)
    msg.color = int(color)
    msg.range = float(math.hypot(x, y))
    msg.bearing = float(math.atan2(y, x))
    msg.confidence = float(confidence)
    msg.source = source
    return msg


def obstacle_msg(
    header: Header,
    x: float,
    y: float,
    radius: float,
    confidence: float,
    label: str,
    source: str,
) -> DetectedObstacle:
    msg = DetectedObstacle()
    msg.header = header
    msg.position = point(x, y, 0.0)
    msg.radius = float(radius)
    msg.confidence = float(confidence)
    msg.label = label
    msg.source = source
    return msg


def track_map_to_msg(node, track_map: SavedTrackMap, frame_id: str = "fsds/map") -> TrackMap:
    msg = TrackMap()
    msg.header = make_header(node, frame_id)
    msg.track_id = track_map.track_id
    msg.closed_loop = bool(track_map.closed_loop)
    msg.quality = float(track_map.quality)
    msg.cones = [
        cone_msg(
            msg.header,
            cone.x,
            cone.y,
            cone.z,
            cone.color,
            cone.confidence,
            f"map:{cone.observations}",
        )
        for cone in track_map.cones
    ]
    msg.blue_boundary_line = [point(x, y, 0.0) for x, y in track_map.blue_boundary_line]
    msg.yellow_boundary_line = [point(x, y, 0.0) for x, y in track_map.yellow_boundary_line]
    msg.centerline = [point(x, y, 0.0) for x, y in track_map.centerline]
    msg.racing_line = [point(x, y, 0.0) for x, y in track_map.racing_line]
    msg.speed_profile = [float(speed) for speed in track_map.speed_profile]
    return msg


def track_map_from_msg(msg: TrackMap) -> SavedTrackMap:
    return SavedTrackMap(
        track_id=msg.track_id,
        closed_loop=msg.closed_loop,
        quality=msg.quality,
        cones=[
            ConeLandmark(
                x=cone.position.x,
                y=cone.position.y,
                z=cone.position.z,
                color=cone.color,
                confidence=cone.confidence,
                observations=1,
            )
            for cone in msg.cones
        ],
        blue_boundary_line=[(p.x, p.y) for p in msg.blue_boundary_line],
        yellow_boundary_line=[(p.x, p.y) for p in msg.yellow_boundary_line],
        centerline=[(p.x, p.y) for p in msg.centerline],
        racing_line=[(p.x, p.y) for p in msg.racing_line],
        speed_profile=list(msg.speed_profile),
    )


def race_state_msg(
    node,
    mission: str,
    track_id: str,
    mode: str,
    behavior_state: str,
    target_speed: float,
    target_steering: float,
    map_quality: float,
    map_loaded: bool,
    emergency_brake: bool,
    go_signal_fresh: bool,
    status: str,
) -> RaceState:
    msg = RaceState()
    msg.header = make_header(node, "fsds/map")
    msg.mission = mission
    msg.track_id = track_id
    msg.mode = mode
    msg.behavior_state = behavior_state
    msg.target_speed = float(target_speed)
    msg.target_steering = float(target_steering)
    msg.map_quality = float(map_quality)
    msg.map_loaded = bool(map_loaded)
    msg.emergency_brake = bool(emergency_brake)
    msg.go_signal_fresh = bool(go_signal_fresh)
    msg.status = status
    return msg


def resolve_repo_path(path_text: str, package_dir: Path) -> Path:
    path = Path(path_text).expanduser()
    if path.is_absolute():
        return path
    return package_dir / path

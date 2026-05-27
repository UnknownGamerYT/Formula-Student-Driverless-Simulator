from __future__ import annotations

from dataclasses import dataclass


class ConeColor:
    YELLOW = 0
    BLUE = 1
    ORANGE = 2
    LARGE_ORANGE = 3
    UNKNOWN = 4


# FSDS track convention in the car/map frame:
# +Y is left of the car, so blue cones are the left boundary and yellow cones
# are the right boundary. Orange cones mark the start/end gate and should be
# kept in the map/visualization but ignored as normal racing-line boundaries.
LEFT_BOUNDARY_COLOR = ConeColor.BLUE
RIGHT_BOUNDARY_COLOR = ConeColor.YELLOW
START_FINISH_COLORS = (ConeColor.ORANGE, ConeColor.LARGE_ORANGE)
STEREO_CAMERA_SOURCE = "CameraStereo"


CONE_CLASS_NAMES = [
    "yellow_cone",
    "blue_cone",
    "orange_cone",
    "large_orange_cone",
    "unknown_cone",
]

CONE_CLASS_TO_COLOR = {name: index for index, name in enumerate(CONE_CLASS_NAMES)}
COLOR_TO_CONE_CLASS = {index: name for index, name in enumerate(CONE_CLASS_NAMES)}

# fs_msgs/Cone uses BLUE=0, YELLOW=1, ORANGE_BIG=2, ORANGE_SMALL=3, UNKNOWN=4.
FS_MSG_COLOR_TO_AUTONOMY = {
    0: ConeColor.BLUE,
    1: ConeColor.YELLOW,
    2: ConeColor.LARGE_ORANGE,
    3: ConeColor.ORANGE,
    4: ConeColor.UNKNOWN,
}


def is_left_boundary_color(color: int) -> bool:
    return int(color) == LEFT_BOUNDARY_COLOR


def is_right_boundary_color(color: int) -> bool:
    return int(color) == RIGHT_BOUNDARY_COLOR


def is_boundary_color(color: int) -> bool:
    return is_left_boundary_color(color) or is_right_boundary_color(color)


def is_start_finish_color(color: int) -> bool:
    return int(color) in START_FINISH_COLORS


def side_color_from_local_y(local_y: float, deadband_m: float = 0.25) -> int:
    if local_y > deadband_m:
        return LEFT_BOUNDARY_COLOR
    if local_y < -deadband_m:
        return RIGHT_BOUNDARY_COLOR
    return ConeColor.UNKNOWN


def is_reliable_geometry_source(source: str) -> bool:
    return "Lidar" in source or STEREO_CAMERA_SOURCE in source


@dataclass(frozen=True)
class SensorOffset:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


DEFAULT_LIDAR_OFFSETS = {
    "Lidar1": SensorOffset(x=0.45, y=0.0, z=0.55, yaw=0.0),
    "Lidar2": SensorOffset(x=1.20, y=0.0, z=0.20, yaw=0.0),
}

DEFAULT_CAMERA_OFFSETS = {
    "cam1": SensorOffset(x=-0.30, y=-0.16, z=0.80, yaw=0.0),
    "cam2": SensorOffset(x=-0.30, y=0.16, z=0.80, yaw=0.0),
}

DEFAULT_CAMERA_FOV_DEG = 90.0
DEFAULT_CONE_HEIGHT_M = 0.325
DEFAULT_CONE_WIDTH_M = 0.23

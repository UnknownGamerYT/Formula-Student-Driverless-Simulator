from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path

from fsds_autonomy.constants import COLOR_TO_CONE_CLASS, ConeColor


@dataclass
class ConeLandmark:
    x: float
    y: float
    z: float = 0.0
    color: int = ConeColor.UNKNOWN
    confidence: float = 0.0
    observations: int = 0
    first_seen_sec: float = 0.0
    last_seen_sec: float = 0.0
    last_seen_frame: int = -1
    hit_streak: int = 0
    missed_frames: int = 0


@dataclass
class SavedTrackMap:
    track_id: str
    closed_loop: bool = False
    quality: float = 0.0
    cones: list[ConeLandmark] = field(default_factory=list)
    blue_boundary_line: list[tuple[float, float]] = field(default_factory=list)
    yellow_boundary_line: list[tuple[float, float]] = field(default_factory=list)
    centerline: list[tuple[float, float]] = field(default_factory=list)
    racing_line: list[tuple[float, float]] = field(default_factory=list)
    speed_profile: list[float] = field(default_factory=list)
    metadata: dict = field(default_factory=dict)


def track_map_sanity_reasons(
    track_map: SavedTrackMap,
    min_line_points: int = 8,
    min_quality: float = 0.55,
    max_cones: int = 800,
    max_line_points: int = 600,
    max_cones_per_100m2: float = 18.0,
    max_racing_turn_deg: float = 90.0,
    require_closed_loop: bool = False,
) -> list[str]:
    reasons: list[str] = []
    if require_closed_loop and not bool(track_map.closed_loop):
        reasons.append("closed_loop false")
    if require_closed_loop and bool(track_map.closed_loop):
        for name, line in (
            ("blue_boundary_line", track_map.blue_boundary_line),
            ("yellow_boundary_line", track_map.yellow_boundary_line),
            ("centerline", track_map.centerline),
            ("racing_line", track_map.racing_line),
        ):
            if len(line) >= min_line_points:
                close_gap = polyline_endpoint_gap(line)
                max_step = max_polyline_step_m(line)
                max_allowed_gap = 12.0 if "boundary" in name else 18.0
                max_allowed_step = 10.0 if "boundary" in name else 18.0
                if close_gap > max_allowed_gap:
                    reasons.append(f"{name} open gap too large ({close_gap:.1f}m > {max_allowed_gap:.1f}m)")
                if max_step > max_allowed_step:
                    reasons.append(f"{name} internal gap too large ({max_step:.1f}m > {max_allowed_step:.1f}m)")
            elif "boundary" in name:
                reasons.append(f"{name} too short ({len(line)} < {min_line_points})")
        start_gate_x = track_map.metadata.get("start_gate_x")
        start_gate_y = track_map.metadata.get("start_gate_y")
        if start_gate_x is not None and start_gate_y is not None:
            for name, line in (
                ("blue_boundary_line", track_map.blue_boundary_line),
                ("yellow_boundary_line", track_map.yellow_boundary_line),
                ("centerline", track_map.centerline),
                ("racing_line", track_map.racing_line),
            ):
                if len(line) >= min_line_points:
                    nearest_start = nearest_polyline_distance_m(line, float(start_gate_x), float(start_gate_y))
                    max_start_distance = 9.0 if "boundary" in name else 8.0
                    if nearest_start > max_start_distance:
                        reasons.append(f"{name} misses start gate ({nearest_start:.1f}m > {max_start_distance:.1f}m)")
    if len(track_map.centerline) < min_line_points:
        reasons.append(f"centerline too short ({len(track_map.centerline)} < {min_line_points})")
    if len(track_map.racing_line) < min_line_points:
        reasons.append(f"racing_line too short ({len(track_map.racing_line)} < {min_line_points})")
    if len(track_map.speed_profile) < min_line_points:
        reasons.append(f"speed_profile too short ({len(track_map.speed_profile)} < {min_line_points})")
    if float(track_map.quality) < min_quality:
        reasons.append(f"quality too low ({track_map.quality:.2f} < {min_quality:.2f})")
    if len(track_map.cones) > max_cones:
        reasons.append(f"too many cones ({len(track_map.cones)} > {max_cones})")
    if len(track_map.centerline) > max_line_points or len(track_map.racing_line) > max_line_points:
        reasons.append(
            f"too many line points (centerline={len(track_map.centerline)} racing_line={len(track_map.racing_line)})"
        )
    max_turn = max_polyline_turn_deg(track_map.racing_line)
    if max_turn > max_racing_turn_deg:
        reasons.append(f"racing_line turn too sharp ({max_turn:.1f}deg > {max_racing_turn_deg:.1f}deg)")
    if len(track_map.cones) >= 20:
        xs = [cone.x for cone in track_map.cones]
        ys = [cone.y for cone in track_map.cones]
        area_m2 = max(1.0, (max(xs) - min(xs)) * (max(ys) - min(ys)))
        density = len(track_map.cones) / area_m2 * 100.0
        if density > max_cones_per_100m2:
            reasons.append(f"cone density too high ({density:.1f} > {max_cones_per_100m2:.1f} per 100m2)")
    return reasons


def max_polyline_turn_deg(points: list[tuple[float, float]]) -> float:
    max_turn = 0.0
    for index in range(1, len(points) - 1):
        p0 = points[index - 1]
        p1 = points[index]
        p2 = points[index + 1]
        ax = p1[0] - p0[0]
        ay = p1[1] - p0[1]
        bx = p2[0] - p1[0]
        by = p2[1] - p1[1]
        len_a = math.hypot(ax, ay)
        len_b = math.hypot(bx, by)
        if len_a < 1e-6 or len_b < 1e-6:
            continue
        dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (len_a * len_b)))
        max_turn = max(max_turn, math.degrees(math.acos(dot)))
    return max_turn


def polyline_endpoint_gap(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return float("inf")
    return math.hypot(float(points[0][0]) - float(points[-1][0]), float(points[0][1]) - float(points[-1][1]))


def max_polyline_step_m(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return float("inf")
    return max(
        math.hypot(float(points[index][0]) - float(points[index - 1][0]), float(points[index][1]) - float(points[index - 1][1]))
        for index in range(1, len(points))
    )


def nearest_polyline_distance_m(points: list[tuple[float, float]], x: float, y: float) -> float:
    if not points:
        return float("inf")
    return min(math.hypot(float(px) - x, float(py) - y) for px, py in points)


def is_usable_track_map(
    track_map: SavedTrackMap,
    min_line_points: int = 8,
    min_quality: float = 0.55,
    require_closed_loop: bool = False,
) -> bool:
    return not track_map_sanity_reasons(
        track_map,
        min_line_points=min_line_points,
        min_quality=min_quality,
        require_closed_loop=require_closed_loop,
    )


def color_name(color: int) -> str:
    return COLOR_TO_CONE_CLASS.get(int(color), COLOR_TO_CONE_CLASS[ConeColor.UNKNOWN])


def map_path(map_dir: Path, track_id: str) -> Path:
    safe_track = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in track_id.strip()) or "unknown"
    return map_dir / f"{safe_track}.json"


def reset_events_path(map_dir: Path, track_id: str) -> Path:
    safe_track = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in track_id.strip()) or "unknown"
    return map_dir / f"{safe_track}.reset_events.json"


def load_reset_events(map_dir: Path, track_id: str, max_events: int | None = None) -> list[dict]:
    path = reset_events_path(map_dir, track_id)
    if not path.exists():
        return []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    if not isinstance(data, list):
        return []
    events = [item for item in data if isinstance(item, dict)]
    if max_events is not None and max_events > 0:
        return events[-max_events:]
    return events


def save_track_map(map_dir: Path, track_map: SavedTrackMap) -> Path:
    map_dir.mkdir(parents=True, exist_ok=True)
    path = map_path(map_dir, track_map.track_id)
    data = asdict(track_map)
    data["cones"] = [
        {
            "x": float(cone.x),
            "y": float(cone.y),
            "z": float(cone.z),
            "color": int(cone.color),
            "confidence": float(cone.confidence),
            "observations": int(cone.observations),
            "color_name": color_name(cone.color),
        }
        for cone in track_map.cones
    ]
    tmp_path = path.with_name(f".{path.name}.tmp")
    tmp_path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
    os.replace(tmp_path, path)
    return path


def load_track_map(map_dir: Path, track_id: str) -> SavedTrackMap | None:
    path = map_path(map_dir, track_id)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    cones = [
        ConeLandmark(
            x=float(item["x"]),
            y=float(item["y"]),
            z=float(item.get("z", 0.0)),
            color=int(item.get("color", ConeColor.UNKNOWN)),
            confidence=float(item.get("confidence", 0.0)),
            observations=int(item.get("observations", 0)),
            first_seen_sec=float(item.get("first_seen_sec", 0.0)),
            last_seen_sec=float(item.get("last_seen_sec", 0.0)),
            last_seen_frame=int(item.get("last_seen_frame", -1)),
            hit_streak=int(item.get("hit_streak", 0)),
            missed_frames=int(item.get("missed_frames", 0)),
        )
        for item in data.get("cones", [])
    ]
    return SavedTrackMap(
        track_id=str(data.get("track_id", track_id)),
        closed_loop=bool(data.get("closed_loop", False)),
        quality=float(data.get("quality", 0.0)),
        cones=cones,
        blue_boundary_line=[tuple(pair) for pair in data.get("blue_boundary_line", [])],
        yellow_boundary_line=[tuple(pair) for pair in data.get("yellow_boundary_line", [])],
        centerline=[tuple(pair) for pair in data.get("centerline", [])],
        racing_line=[tuple(pair) for pair in data.get("racing_line", [])],
        speed_profile=[float(value) for value in data.get("speed_profile", [])],
        metadata=dict(data.get("metadata", {})),
    )

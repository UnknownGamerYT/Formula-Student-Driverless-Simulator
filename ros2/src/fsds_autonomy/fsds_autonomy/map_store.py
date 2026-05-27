from __future__ import annotations

import json
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


@dataclass
class SavedTrackMap:
    track_id: str
    closed_loop: bool = False
    quality: float = 0.0
    cones: list[ConeLandmark] = field(default_factory=list)
    centerline: list[tuple[float, float]] = field(default_factory=list)
    racing_line: list[tuple[float, float]] = field(default_factory=list)
    speed_profile: list[float] = field(default_factory=list)
    metadata: dict = field(default_factory=dict)


def track_map_sanity_reasons(
    track_map: SavedTrackMap,
    min_line_points: int = 8,
    min_quality: float = 0.25,
    max_cones: int = 800,
    max_line_points: int = 600,
    max_cones_per_100m2: float = 18.0,
) -> list[str]:
    reasons: list[str] = []
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
    if len(track_map.cones) >= 20:
        xs = [cone.x for cone in track_map.cones]
        ys = [cone.y for cone in track_map.cones]
        area_m2 = max(1.0, (max(xs) - min(xs)) * (max(ys) - min(ys)))
        density = len(track_map.cones) / area_m2 * 100.0
        if density > max_cones_per_100m2:
            reasons.append(f"cone density too high ({density:.1f} > {max_cones_per_100m2:.1f} per 100m2)")
    return reasons


def is_usable_track_map(track_map: SavedTrackMap, min_line_points: int = 8, min_quality: float = 0.25) -> bool:
    return not track_map_sanity_reasons(track_map, min_line_points=min_line_points, min_quality=min_quality)


def color_name(color: int) -> str:
    return COLOR_TO_CONE_CLASS.get(int(color), COLOR_TO_CONE_CLASS[ConeColor.UNKNOWN])


def map_path(map_dir: Path, track_id: str) -> Path:
    safe_track = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in track_id.strip()) or "unknown"
    return map_dir / f"{safe_track}.json"


def save_track_map(map_dir: Path, track_map: SavedTrackMap) -> Path:
    map_dir.mkdir(parents=True, exist_ok=True)
    path = map_path(map_dir, track_map.track_id)
    data = asdict(track_map)
    data["cones"] = [
        {
            **asdict(cone),
            "color_name": color_name(cone.color),
        }
        for cone in track_map.cones
    ]
    path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
    return path


def load_track_map(map_dir: Path, track_id: str) -> SavedTrackMap | None:
    path = map_path(map_dir, track_id)
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    cones = [
        ConeLandmark(
            x=float(item["x"]),
            y=float(item["y"]),
            z=float(item.get("z", 0.0)),
            color=int(item.get("color", ConeColor.UNKNOWN)),
            confidence=float(item.get("confidence", 0.0)),
            observations=int(item.get("observations", 0)),
        )
        for item in data.get("cones", [])
    ]
    return SavedTrackMap(
        track_id=str(data.get("track_id", track_id)),
        closed_loop=bool(data.get("closed_loop", False)),
        quality=float(data.get("quality", 0.0)),
        cones=cones,
        centerline=[tuple(pair) for pair in data.get("centerline", [])],
        racing_line=[tuple(pair) for pair in data.get("racing_line", [])],
        speed_profile=[float(value) for value in data.get("speed_profile", [])],
        metadata=dict(data.get("metadata", {})),
    )

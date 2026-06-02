#!/usr/bin/env python3
from __future__ import annotations

from collections import Counter
from collections import deque
import argparse
import json
import math
from pathlib import Path
import re
import statistics


SOURCE_RE = re.compile(r"\bsource=([^\s]+)")
REASON_RE = re.compile(r"\breason=([^\s]+)")


def values():
    return {
        "speed": [],
        "target_speed": [],
        "steering": [],
        "throttle": [],
        "brake": [],
        "dt": [],
        "boundary_min": [],
        "yellow_clearance": [],
        "blue_clearance": [],
        "centerline_distance": [],
    }


def add_value(bucket: list[float], value) -> None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return
    if math.isfinite(number):
        bucket.append(number)


def describe(name: str, bucket: list[float]) -> str:
    if not bucket:
        return f"{name}: n=0"
    return (
        f"{name}: n={len(bucket)} avg={statistics.fmean(bucket):.3f} "
        f"min={min(bucket):.3f} max={max(bucket):.3f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize fsds_drive_recorder JSONL logs.")
    parser.add_argument("log", type=Path)
    parser.add_argument("--last", type=int, default=0, help="Only summarize the last N control events.")
    args = parser.parse_args()

    path = args.log.expanduser()
    stats = values()
    source_counts = Counter()
    reason_counts = Counter()
    nearest_side_counts = Counter()
    event_counts = Counter()
    low_clearance_examples = []
    controls = deque(maxlen=max(0, int(args.last)) or None)

    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            event_counts[str(event.get("event", "unknown"))] += 1
            if event.get("event") != "control":
                continue
            controls.append((line_no, event))

    for line_no, event in controls:
        command = event.get("command", {})
        context = event.get("context", {})
        edge = context.get("edge") or {}
        boundary = edge.get("boundary_clearance") or {}
        diag = str(context.get("controller_diagnostics", ""))

        add_value(stats["steering"], command.get("steering"))
        add_value(stats["throttle"], command.get("throttle"))
        add_value(stats["brake"], command.get("brake"))
        add_value(stats["dt"], event.get("dt_sec"))
        add_value(stats["speed"], context.get("speed_mps"))
        add_value(stats["target_speed"], context.get("target_speed_mps"))
        add_value(stats["boundary_min"], boundary.get("min_clearance_m"))
        add_value(stats["yellow_clearance"], boundary.get("yellow_clearance_m"))
        add_value(stats["blue_clearance"], boundary.get("blue_clearance_m"))
        add_value(stats["centerline_distance"], edge.get("centerline_distance_m"))
        nearest_side_counts[str(boundary.get("nearest_side", "unknown"))] += 1

        source_match = SOURCE_RE.search(diag)
        if source_match:
            source_counts[source_match.group(1)] += 1
        reason_match = REASON_RE.search(diag)
        if reason_match:
            reason_counts[reason_match.group(1)] += 1

        clearance = boundary.get("min_clearance_m")
        if isinstance(clearance, (float, int)) and clearance < 0.75 and len(low_clearance_examples) < 12:
            pose = context.get("pose") or {}
            low_clearance_examples.append(
                (
                    line_no,
                    float(clearance),
                    boundary.get("nearest_side"),
                    pose.get("x"),
                    pose.get("y"),
                    command.get("steering"),
                    command.get("throttle"),
                    command.get("brake"),
                    diag[:160],
                )
            )

    print(f"log: {path}")
    if args.last > 0:
        print(f"scope: last {len(controls)} control events")
    print(f"events: {dict(event_counts)}")
    for key in ("dt", "speed", "target_speed", "steering", "throttle", "brake", "boundary_min", "yellow_clearance", "blue_clearance", "centerline_distance"):
        print(describe(key, stats[key]))
    if stats["dt"]:
        avg_dt = statistics.fmean(stats["dt"])
        print(f"control_hz_avg: {1.0 / avg_dt:.2f}")
    print(f"target_sources: {dict(source_counts)}")
    print(f"control_reasons: {dict(reason_counts)}")
    print(f"nearest_boundary_side: {dict(nearest_side_counts)}")
    print("low_clearance_examples:")
    for item in low_clearance_examples:
        line_no, clearance, side, x, y, steering, throttle, brake, diag = item
        pose_text = "unknown"
        if isinstance(x, (float, int)) and isinstance(y, (float, int)):
            pose_text = f"{x:.2f},{y:.2f}"
        print(
            f"  line={line_no} clearance={clearance:.2f} side={side} "
            f"pose=({pose_text}) cmd=(steer={steering:.2f},thr={throttle:.2f},brake={brake:.2f}) {diag}"
        )


if __name__ == "__main__":
    main()

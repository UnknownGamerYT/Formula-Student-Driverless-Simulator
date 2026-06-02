#!/usr/bin/env python3
from __future__ import annotations

from collections import Counter
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


DEFAULT_RUNS_DIR = Path("/home/hard/.fsds_autonomy/runs")


def finite_float(value, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def recent_run_paths(limit: int) -> list[Path]:
    candidates: list[Path] = []
    if not DEFAULT_RUNS_DIR.exists():
        return candidates
    for path in DEFAULT_RUNS_DIR.rglob("*"):
        if path.name not in ("episode_summaries.jsonl", "monitor.csv"):
            continue
        candidates.append(path.parent)
    unique = sorted(set(candidates), key=lambda path: path.stat().st_mtime, reverse=True)
    return unique[: max(1, limit)]


def load_jsonl(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(item, dict):
                rows.append(item)
    return rows


def load_monitor(path: Path, step_dt: float) -> list[dict]:
    rows: list[dict] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(line for line in handle if not line.startswith("#"))
        for index, row in enumerate(reader, start=1):
            progress = finite_float(row.get("episode_progress_m"))
            length = finite_float(row.get("path_length_m"))
            progress_fraction = progress / length if length > 1.0 else 0.0
            steps = finite_float(row.get("l"))
            terminal = str(row.get("terminal_reason") or "")
            rows.append(
                {
                    "schema": "monitor_csv_fallback",
                    "episode_index": index,
                    "episode_step": int(steps),
                    "control_time_sec": steps * step_dt,
                    "terminal_reason": terminal,
                    "lap_complete": terminal == "lap_complete",
                    "progress_m": progress,
                    "path_length_m": length,
                    "progress_fraction": progress_fraction,
                    "progress_percent": 100.0 * progress_fraction,
                    "sector_count": int(finite_float(row.get("sector_count"))),
                    "stage_after_update": int(finite_float(row.get("stage_after_episode") or row.get("stage"))),
                    "direct_blend_end": finite_float(row.get("direct_blend")),
                    "reward_total": finite_float(row.get("r") or row.get("reward_total")),
                    "speed_avg_mps": finite_float(row.get("episode_avg_speed_mps") or row.get("speed_mps")),
                    "speed_max_mps": finite_float(row.get("episode_max_speed_mps") or row.get("speed_mps")),
                    "clearance_min_m": finite_float(row.get("episode_min_clearance_m") or row.get("min_clearance_m")),
                    "clearance_avg_m": finite_float(row.get("episode_avg_clearance_m") or row.get("min_clearance_m")),
                    "truncated": False,
                }
            )
    return rows


def load_run(path: Path, step_dt: float) -> tuple[str, list[dict]]:
    path = path.expanduser()
    if path.is_file():
        if path.name == "monitor.csv":
            return str(path.parent), load_monitor(path, step_dt)
        return str(path.parent), load_jsonl(path)
    summary = path / "episode_summaries.jsonl"
    if summary.exists():
        return str(path), load_jsonl(summary)
    monitor = path / "monitor.csv"
    if monitor.exists():
        return str(path), load_monitor(monitor, step_dt)
    return str(path), []


def seconds_text(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "-"
    return f"{value:.1f}s"


def summarize_rows(name: str, rows: list[dict]) -> dict:
    if not rows:
        return {"run": name, "episodes": 0}
    laps = [row for row in rows if bool(row.get("lap_complete")) or row.get("terminal_reason") == "lap_complete"]
    lap_times = [finite_float(row.get("control_time_sec")) for row in laps if finite_float(row.get("control_time_sec")) > 0.0]
    progress = [finite_float(row.get("progress_fraction")) for row in rows]
    rewards = [finite_float(row.get("reward_total")) for row in rows]
    speeds = [finite_float(row.get("speed_avg_mps")) for row in rows if finite_float(row.get("speed_avg_mps")) > 0.0]
    clearances = [finite_float(row.get("clearance_min_m")) for row in rows if finite_float(row.get("clearance_min_m")) != 0.0]
    terminal_counts = Counter(str(row.get("terminal_reason") or ("truncated" if row.get("truncated") else "unknown")) for row in rows)
    stage_counts = Counter(str(row.get("stage_after_update") or row.get("stage") or "?") for row in rows)
    last = rows[-1]
    best_progress_index = max(range(len(rows)), key=lambda index: finite_float(rows[index].get("progress_fraction")))
    best_progress_row = rows[best_progress_index]
    return {
        "run": name,
        "episodes": len(rows),
        "laps": len(laps),
        "best_progress_percent": 100.0 * max(progress),
        "best_progress_episode": int(best_progress_row.get("episode_index") or best_progress_index + 1),
        "last_progress_percent": 100.0 * finite_float(last.get("progress_fraction")),
        "best_lap_sec": min(lap_times) if lap_times else None,
        "median_lap_sec": statistics.median(lap_times) if lap_times else None,
        "last_lap_sec": lap_times[-1] if lap_times else None,
        "recent_avg_reward": statistics.fmean(rewards[-min(10, len(rewards)) :]) if rewards else 0.0,
        "best_reward": max(rewards) if rewards else 0.0,
        "avg_speed_mps": statistics.fmean(speeds) if speeds else 0.0,
        "best_avg_speed_mps": max(speeds) if speeds else 0.0,
        "min_clearance_m": min(clearances) if clearances else 0.0,
        "last_terminal": str(last.get("terminal_reason") or ("truncated" if last.get("truncated") else "")),
        "last_stage": str(last.get("stage_after_update") or last.get("stage") or "?"),
        "terminal_counts": dict(terminal_counts.most_common()),
        "stage_counts": dict(stage_counts.most_common()),
    }


def print_table(summaries: list[dict]) -> None:
    print(
        "run | eps | laps | best_prog | last_prog | best_lap | med_lap | "
        "recent_reward | avg_speed | min_clear | last_stage | top_terminals"
    )
    for summary in summaries:
        terminals = ", ".join(f"{key}:{value}" for key, value in list(summary.get("terminal_counts", {}).items())[:3])
        print(
            f"{Path(summary['run']).name} | "
            f"{summary.get('episodes', 0)} | "
            f"{summary.get('laps', 0)} | "
            f"{summary.get('best_progress_percent', 0.0):.1f}% | "
            f"{summary.get('last_progress_percent', 0.0):.1f}% | "
            f"{seconds_text(summary.get('best_lap_sec'))} | "
            f"{seconds_text(summary.get('median_lap_sec'))} | "
            f"{summary.get('recent_avg_reward', 0.0):.1f} | "
            f"{summary.get('avg_speed_mps', 0.0):.2f} | "
            f"{summary.get('min_clearance_m', 0.0):.2f} | "
            f"{summary.get('last_stage', '?')} | "
            f"{terminals}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize live RL episode-level progress and terminal stats.")
    parser.add_argument("paths", nargs="*", type=Path, help="Run directories, episode_summaries.jsonl files, or monitor.csv files.")
    parser.add_argument("--recent", type=int, default=12, help="When no paths are given, summarize this many recent runs.")
    parser.add_argument("--step-dt", type=float, default=0.10, help="Fallback control step time for monitor.csv rows.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON instead of a table.")
    args = parser.parse_args()

    paths = args.paths or recent_run_paths(args.recent)
    summaries = []
    for path in paths:
        name, rows = load_run(path, step_dt=args.step_dt)
        summaries.append(summarize_rows(name, rows))

    if args.json:
        print(json.dumps(summaries, indent=2, sort_keys=True))
    else:
        print_table(summaries)


if __name__ == "__main__":
    main()

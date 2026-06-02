#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import shlex
import subprocess
import time


@dataclass(frozen=True)
class AgentSpec:
    index: int
    domain: int
    seed: int
    api_port: int


def parse_agent(value: str) -> AgentSpec:
    parts = value.split(":")
    if len(parts) not in (3, 4):
        raise argparse.ArgumentTypeError("agent must be index:domain:seed[:api_port]")
    index = int(parts[0])
    api_port = int(parts[3]) if len(parts) == 4 else 41460 + index
    return AgentSpec(index=index, domain=int(parts[1]), seed=int(parts[2]), api_port=api_port)


def read_monitor(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        lines = [line for line in handle if not line.startswith("#")]
    if not lines:
        return []
    return list(csv.DictReader(lines))


def latest_checkpoint(checkpoint_dir: Path) -> Path | None:
    checkpoints = sorted(
        checkpoint_dir.glob("sac_live_full_control_*_steps.zip"),
        key=lambda path: path.stat().st_mtime,
    )
    return checkpoints[-1] if checkpoints else None


def checkpoint_steps(path: Path) -> int:
    stem = path.stem
    try:
        return int(stem.rsplit("_", 2)[-2])
    except (IndexError, ValueError):
        return 0


def monitor_ready(rows: list[dict[str, str]], min_laps: int, min_best_progress_fraction: float) -> tuple[bool, str]:
    if not rows:
        return False, "no monitor rows"
    laps = sum(1 for row in rows if str(row.get("terminal_reason") or "") == "lap_complete")
    best_fraction = 0.0
    for row in rows:
        try:
            progress = float(row.get("episode_progress_m") or 0.0)
            path_length = float(row.get("path_length_m") or 0.0)
        except ValueError:
            continue
        if path_length > 1.0:
            best_fraction = max(best_fraction, progress / path_length)
    if laps < min_laps:
        return False, f"laps={laps}/{min_laps} best={100.0 * best_fraction:.1f}%"
    if best_fraction < min_best_progress_fraction:
        return False, f"laps={laps} best={100.0 * best_fraction:.1f}%"
    return True, f"laps={laps} best={100.0 * best_fraction:.1f}%"


def load_state(path: Path) -> dict[str, bool]:
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(data, dict):
        return {}
    return {str(key): bool(value) for key, value in data.items()}


def save_state(path: Path, state: dict[str, bool]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(state, indent=2, sort_keys=True), encoding="utf-8")


def start_stage2_agent(agent: AgentSpec, checkpoint: Path, args: argparse.Namespace) -> None:
    session = f"msim{agent.index}_rl"
    run_name = f"msim{agent.index}_sac_seed{agent.seed}_domain{agent.domain}_stage2_safe_resets"
    out_dir = args.next_base.expanduser() / run_name
    log_path = args.log_dir.expanduser() / f"msim{agent.index}_stage2_random_rl.log"
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    subprocess.run(["tmux", "kill-session", "-t", session], check=False)
    random_start_args = "--no-random-start-enabled "
    if args.enable_random_start:
        random_start_args = (
            "--random-start-enabled "
            "--random-start-min-stage 2 "
            f"--random-start-port {agent.api_port} "
            "--random-start-skip-reset "
            "--no-random-start-reset-before-teleport "
            "--random-start-pause-during-teleport "
            f"--random-start-pre-teleport-brake-sec {args.random_start_pre_teleport_brake_sec} "
            f"--random-start-stop-speed-mps {args.random_start_stop_speed_mps} "
            f"--random-start-stop-timeout-sec {args.random_start_stop_timeout_sec} "
            f"--random-start-disable-after-failures {args.random_start_disable_after_failures} "
        )
    command = (
        f"export ROS_DOMAIN_ID={agent.domain}; "
        "cd /home/hard/Desktop/Formula-Student-Driverless-Simulator/ros2; "
        "source /opt/ros/jazzy/setup.bash; "
        "source install/setup.bash; "
        "python3 -u src/fsds_autonomy/tools/train_rl_live.py "
        "--algo SAC "
        "--steps 200000 "
        "--auto-steps "
        "--min-steps 50000 "
        "--step-chunk 10000 "
        "--auto-learning-rate "
        "--learning-rate 0.0003 "
        "--final-learning-rate 0.00005 "
        "--device auto "
        f"--seed {agent.seed} "
        f"--load-model {shlex.quote(str(checkpoint))} "
        "--auto-load-replay-buffer "
        "--variable-episode-steps "
        "--min-episode-steps 3500 "
        "--max-episode-steps 8000 "
        "--map-dir /home/hard/.fsds_autonomy/maps "
        "--danger-zone-enabled "
        "--danger-zone-radius-m 7.0 "
        "--danger-zone-penalty 0.35 "
        f"{random_start_args}"
        "--start-stage 2 "
        "--stage1-residual-scale 0.25 "
        "--stage2-residual-scale 0.50 "
        "--stage2-min-sectors 12 "
        "--stage3-min-sectors 22 "
        "--stage2-clean-episodes 3 "
        "--stage3-clean-episodes 4 "
        "--stage4-clean-laps 2 "
        "--reset-status-hold-terminal-sec 1.5 "
        "--time-penalty 0.0 "
        "--sector-time-reference-sec 25 "
        "--sector-time-reference-reward 0.5 "
        "--sector-time-reward-max 10 "
        "--speed-reward 0.04 "
        "--speed-reward-power 2.0 "
        "--speed-reward-max-per-step 0.40 "
        "--speed-reward-min-clearance-m 1.00 "
        "--speed-reward-full-clearance-m 1.60 "
        "--lap-bonus 250 "
        "--lap-time-reference-sec 600 "
        "--lap-time-reference-reward 50 "
        "--lap-time-reward-max 1000 "
        "--reward-log "
        f"--out {shlex.quote(str(out_dir))} "
        f"> {shlex.quote(str(log_path))} 2>&1"
    )
    subprocess.run(["tmux", "new-session", "-d", "-s", session, "bash", "-lc", command], check=True)
    print(
        f"started {session} from {checkpoint.name} with Stage 2 safe resets on RPC {agent.api_port} -> {out_dir}",
        flush=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Promote multisim RL agents to Stage 2 after clean lap evidence.")
    parser.add_argument("--current-base", type=Path, default=Path("/home/hard/.fsds_autonomy/runs/multisim_varsteps"))
    parser.add_argument("--next-base", type=Path, default=Path("/home/hard/.fsds_autonomy/runs/multisim_stage2_random"))
    parser.add_argument("--log-dir", type=Path, default=Path("/home/hard/.fsds_autonomy/logs"))
    parser.add_argument("--state-path", type=Path, default=Path("/home/hard/.fsds_autonomy/runs/multisim_stage2_random/watchdog_state.json"))
    parser.add_argument("--agent", action="append", type=parse_agent, default=[])
    parser.add_argument("--min-laps", type=int, default=2)
    parser.add_argument("--min-best-progress-fraction", type=float, default=0.95)
    parser.add_argument("--min-checkpoint-steps", type=int, default=10000)
    parser.add_argument("--enable-random-start", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--random-start-pre-teleport-brake-sec", type=float, default=0.5)
    parser.add_argument("--random-start-stop-speed-mps", type=float, default=0.20)
    parser.add_argument("--random-start-stop-timeout-sec", type=float, default=2.0)
    parser.add_argument("--random-start-disable-after-failures", type=int, default=1)
    parser.add_argument("--poll-sec", type=float, default=60.0)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    agents = args.agent or [
        AgentSpec(1, 71, 81, 41461),
        AgentSpec(2, 72, 82, 41462),
        AgentSpec(3, 73, 83, 41463),
        AgentSpec(4, 74, 84, 41464),
    ]
    state = load_state(args.state_path.expanduser())
    while True:
        for agent in agents:
            key = f"msim{agent.index}"
            if state.get(key):
                continue
            current_dir = args.current_base.expanduser() / f"msim{agent.index}_sac_seed{agent.seed}_domain{agent.domain}"
            monitor_rows = read_monitor(current_dir / "monitor.csv")
            ready, reason = monitor_ready(monitor_rows, args.min_laps, args.min_best_progress_fraction)
            checkpoint = latest_checkpoint(current_dir / "checkpoints")
            if checkpoint is None:
                print(f"{key}: waiting for checkpoint; {reason}", flush=True)
                continue
            steps = checkpoint_steps(checkpoint)
            if steps < args.min_checkpoint_steps:
                print(f"{key}: waiting checkpoint steps {steps}/{args.min_checkpoint_steps}; {reason}", flush=True)
                continue
            if not ready:
                print(f"{key}: waiting for clean lap evidence; {reason}; checkpoint={checkpoint.name}", flush=True)
                continue
            start_stage2_agent(agent, checkpoint, args)
            state[key] = True
            save_state(args.state_path.expanduser(), state)
        if all(state.get(f"msim{agent.index}") for agent in agents) or args.once:
            return
        time.sleep(max(5.0, float(args.poll_sec)))


if __name__ == "__main__":
    main()

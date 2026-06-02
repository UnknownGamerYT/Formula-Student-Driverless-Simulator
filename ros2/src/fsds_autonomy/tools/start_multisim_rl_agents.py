#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import shlex
import subprocess
import time
from typing import Any


SIM_ROOT = Path("/home/hard/Desktop/Formula-Student-Driverless-Simulator")
ROS_ROOT = SIM_ROOT / "ros2"
UE_ROOT = SIM_ROOT / "UE5Builds/LinuxArm64"


def shell_quote(value: str | Path) -> str:
    return shlex.quote(str(value))


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().read_text(encoding="utf-8"))


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=False), encoding="utf-8")


def find_model(model_path: Path | None) -> Path:
    if model_path is not None:
        path = model_path.expanduser()
        if not path.exists():
            raise FileNotFoundError(path)
        return path
    candidates = sorted(
        Path("/home/hard/.fsds_autonomy/runs/fsds_cones").glob("*/weights/best.pt"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if candidates:
        return candidates[0]
    fallback = Path("/home/hard/Desktop/Driverless_FSD_HARD/ros2_ws/src/cone_detection_model/yolo26n.pt")
    if not fallback.exists():
        raise FileNotFoundError("No cone model found and fallback yolo26n.pt is missing")
    return fallback


def map_file(map_dir: Path, track_id: str) -> Path:
    return map_dir.expanduser() / f"{track_id}.json"


def load_centerline(map_dir: Path, track_id: str) -> list[tuple[float, float]]:
    data = load_json(map_file(map_dir, track_id))
    points = data.get("centerline") or []
    centerline: list[tuple[float, float]] = []
    for point in points:
        if not isinstance(point, list) or len(point) < 2:
            continue
        centerline.append((float(point[0]), float(point[1])))
    if len(centerline) < 3:
        raise RuntimeError(f"Map {map_file(map_dir, track_id)} does not have a usable centerline")
    return centerline


def heading_deg(points: list[tuple[float, float]], index: int) -> float:
    prev_point = points[(index - 1) % len(points)]
    next_point = points[(index + 1) % len(points)]
    dx = next_point[0] - prev_point[0]
    dy = next_point[1] - prev_point[1]
    return math.degrees(math.atan2(dy, dx))


def choose_spawn_indices(count: int, points: list[tuple[float, float]], args: argparse.Namespace) -> list[int]:
    if count <= 0:
        return []
    if args.spawn_mode == "start":
        return [0 for _ in range(count)]
    rng = random.Random(args.spawn_seed)
    n = len(points)
    indices: list[int] = []
    for offset in range(count):
        if args.spawn_mode == "random":
            indices.append(rng.randrange(n))
        else:
            jitter = rng.uniform(-0.15, 0.15) if args.spawn_jitter else 0.0
            fraction = (offset + max(-0.40, min(0.40, jitter))) / max(1, count)
            indices.append(int(round((fraction % 1.0) * n)) % n)
    return indices


def make_agent_settings(
    template: dict[str, Any],
    settings_path: Path,
    api_port: int,
    spawn: tuple[float, float, float] | None,
    vehicle_name: str,
) -> None:
    data = json.loads(json.dumps(template))
    data["ApiServerPort"] = api_port
    vehicle = data.setdefault("Vehicles", {}).setdefault(vehicle_name, {})
    if spawn is not None:
        x, y, yaw = spawn
        vehicle["X"] = float(x)
        vehicle["Y"] = float(y)
        vehicle["Z"] = 0.0
        vehicle["Pitch"] = 0.0
        vehicle["Roll"] = 0.0
        vehicle["Yaw"] = float(yaw)
    write_json(settings_path, data)


def latest_checkpoint(search_bases: list[Path], agent_name: str) -> Path | None:
    matches: list[Path] = []
    for base in search_bases:
        base = base.expanduser()
        if not base.exists():
            continue
        matches.extend(base.glob(f"{agent_name}*/checkpoints/sac_live_full_control_*_steps.zip"))
    if not matches:
        return None
    return sorted(matches, key=lambda path: path.stat().st_mtime)[-1]


def tmux_kill(session: str) -> None:
    subprocess.run(["tmux", "kill-session", "-t", session], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def tmux_start(session: str, command: str) -> None:
    subprocess.run(["tmux", "new-session", "-d", "-s", session, "bash", "-lc", command], check=True)


def start_agent(
    index: int,
    domain: int,
    seed: int,
    api_port: int,
    settings_path: Path,
    model_path: Path,
    run_dir: Path,
    checkpoint: Path | None,
    args: argparse.Namespace,
) -> None:
    prefix = f"msim{index}"
    if args.kill_existing:
        for suffix in ("rl", "autonomy", "bridge", "sim"):
            tmux_kill(f"{prefix}_{suffix}")

    log_dir = args.log_dir.expanduser()
    log_dir.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(parents=True, exist_ok=True)

    sim_cmd = (
        f"cd {shell_quote(UE_ROOT)}; "
        f"DISPLAY={shell_quote(args.display)} ./Blocks.sh "
        "-vulkan -windowed "
        f"-ResX={args.res_x} -ResY={args.res_y} "
        "-nosound -NoVSync -stdout -FullStdOutLogOutput -log "
        f"-settings{shell_quote(settings_path)} "
        f"-ExecCmds={shell_quote('r.VSync 0,t.MaxFPS 0')} "
        f"> {shell_quote(log_dir / f'{prefix}_sim.log')} 2>&1"
    )
    tmux_start(f"{prefix}_sim", sim_cmd)
    time.sleep(max(0.0, args.launch_gap_sec))

    bridge_cmd = (
        f"export ROS_DOMAIN_ID={domain}; "
        "source /opt/ros/jazzy/setup.bash; "
        f"source {shell_quote(ROS_ROOT / 'install/setup.bash')}; "
        "ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py "
        "host:=127.0.0.1 "
        f"api_port:={api_port} "
        f"settings_path:={shell_quote(settings_path)} "
        "manual_mode:=false timeout:=120.0 "
        f"> {shell_quote(log_dir / f'{prefix}_bridge.log')} 2>&1"
    )
    tmux_start(f"{prefix}_bridge", bridge_cmd)
    time.sleep(max(0.0, args.launch_gap_sec))

    autonomy_cmd = (
        f"export ROS_DOMAIN_ID={domain}; "
        f"cd {shell_quote(ROS_ROOT)}; "
        "source /opt/ros/jazzy/setup.bash; "
        "source install/setup.bash; "
        "ros2 launch fsds_autonomy fsds_autonomy.launch.py "
        f"map_dir:={shell_quote(args.map_dir.expanduser())} "
        f"model_path:={shell_quote(model_path)} "
        "use_testing_odom:=true "
        "auto_reset_enabled:=false "
        "dataset_enabled:=false "
        "camera_enabled:=true "
        "mapper_enabled:=false "
        "raceline_planner_enabled:=false "
        "saved_map_publisher_enabled:=true "
        "foxglove_visualizer_enabled:=false "
        "drive_log_enabled:=false "
        f"> {shell_quote(log_dir / f'{prefix}_autonomy.log')} 2>&1"
    )
    tmux_start(f"{prefix}_autonomy", autonomy_cmd)
    time.sleep(max(0.0, args.launch_gap_sec))

    load_model_args = ""
    if checkpoint is not None:
        load_model_args = f"--load-model {shell_quote(checkpoint)} --auto-load-replay-buffer "
    random_start_args = "--no-random-start-enabled "
    if args.rl_random_start_enabled:
        random_start_args = (
            "--random-start-enabled "
            f"--random-start-min-stage {args.rl_random_start_min_stage} "
            f"--random-start-port {api_port} "
            "--random-start-skip-reset "
            "--no-random-start-reset-before-teleport "
            "--random-start-pause-during-teleport "
            f"--random-start-pre-teleport-brake-sec {args.rl_random_start_pre_teleport_brake_sec} "
            f"--random-start-stop-speed-mps {args.rl_random_start_stop_speed_mps} "
            f"--random-start-stop-timeout-sec {args.rl_random_start_stop_timeout_sec} "
            f"--random-start-disable-after-failures {args.rl_random_start_disable_after_failures} "
        )
    rl_cmd = (
        f"export ROS_DOMAIN_ID={domain}; "
        f"cd {shell_quote(ROS_ROOT)}; "
        "source /opt/ros/jazzy/setup.bash; "
        "source install/setup.bash; "
        "python3 -u src/fsds_autonomy/tools/train_rl_live.py "
        "--algo SAC "
        f"--steps {args.steps} "
        "--auto-steps "
        f"--min-steps {args.min_steps} "
        f"--step-chunk {args.step_chunk} "
        "--auto-learning-rate "
        f"--learning-rate {args.learning_rate} "
        f"--final-learning-rate {args.final_learning_rate} "
        f"--device {shell_quote(args.device)} "
        f"--seed {seed} "
        f"{load_model_args}"
        "--variable-episode-steps "
        f"--min-episode-steps {args.min_episode_steps} "
        f"--max-episode-steps {args.max_episode_steps} "
        f"--map-dir {shell_quote(args.map_dir.expanduser())} "
        "--danger-zone-enabled "
        f"--danger-zone-radius-m {args.danger_zone_radius_m} "
        f"--danger-zone-penalty {args.danger_zone_penalty} "
        "--reset-on-episode "
        f"{random_start_args}"
        f"--start-stage {args.start_stage} "
        "--stage1-residual-scale 0.25 "
        "--stage2-residual-scale 0.50 "
        "--stage2-min-sectors 12 "
        "--stage3-min-sectors 22 "
        "--stage2-clean-episodes 3 "
        "--stage3-clean-episodes 4 "
        "--stage4-clean-laps 2 "
        "--reset-status-hold-terminal-sec 1.5 "
        "--mistake-budget-enabled "
        "--mistake-budget-limit 35 "
        "--mistake-recovery-per-step 0.35 "
        "--offtrack-grace-steps 20 "
        "--reset-bad-grace-steps 15 "
        "--low-clearance-terminal-m 0.35 "
        "--low-clearance-grace-steps 30 "
        "--wrong-direction-grace-steps 25 "
        "--no-progress-grace-steps 250 "
        "--no-progress-min-delta-m 0.01 "
        "--no-progress-step-penalty 0.20 "
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
        f"--out {shell_quote(run_dir)} "
        f"> {shell_quote(log_dir / f'{prefix}_rl.log')} 2>&1"
    )
    tmux_start(f"{prefix}_rl", rl_cmd)


def main() -> None:
    parser = argparse.ArgumentParser(description="Start multiple FSDS live-RL agents with isolated simulators and ROS domains.")
    parser.add_argument("--count", type=int, default=2)
    parser.add_argument("--first-index", type=int, default=1)
    parser.add_argument("--base-domain", type=int, default=71)
    parser.add_argument("--base-port", type=int, default=41461)
    parser.add_argument("--base-seed", type=int, default=81)
    parser.add_argument("--track-id", default="A")
    parser.add_argument("--map-dir", type=Path, default=Path("/home/hard/.fsds_autonomy/maps"))
    parser.add_argument("--settings-template", type=Path, default=Path("/home/hard/.fsds_autonomy/multisim/settings_1_41461.json"))
    parser.add_argument("--settings-dir", type=Path, default=Path("/home/hard/.fsds_autonomy/multisim"))
    parser.add_argument("--vehicle-name", default="FSCar")
    parser.add_argument("--model-path", type=Path, default=None)
    parser.add_argument(
        "--load-rl-model",
        type=Path,
        default=None,
        help="Optional SAC checkpoint to warm-start every agent instead of searching per-agent checkpoints.",
    )
    parser.add_argument("--run-base", type=Path, default=Path("/home/hard/.fsds_autonomy/runs/multisim_spawn_random"))
    parser.add_argument("--resume-search-base", action="append", type=Path, default=[])
    parser.add_argument("--log-dir", type=Path, default=Path("/home/hard/.fsds_autonomy/logs"))
    parser.add_argument("--display", default=":1")
    parser.add_argument("--res-x", type=int, default=640)
    parser.add_argument("--res-y", type=int, default=360)
    parser.add_argument("--launch-gap-sec", type=float, default=2.0)
    parser.add_argument("--spawn-mode", choices=["even", "random", "start"], default="start")
    parser.add_argument(
        "--spawn-settings-supported",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable non-start spawn settings only after the packaged UE binary honors vehicle X/Y/Z settings.",
    )
    parser.add_argument("--spawn-seed", type=int, default=20260530)
    parser.add_argument("--spawn-jitter", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--kill-existing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--start-stage", type=int, default=2)
    parser.add_argument("--steps", type=int, default=200_000)
    parser.add_argument("--min-steps", type=int, default=50_000)
    parser.add_argument("--step-chunk", type=int, default=10_000)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--final-learning-rate", type=float, default=5e-5)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--min-episode-steps", type=int, default=3500)
    parser.add_argument("--max-episode-steps", type=int, default=8000)
    parser.add_argument("--danger-zone-radius-m", type=float, default=7.0)
    parser.add_argument("--danger-zone-penalty", type=float, default=0.35)
    parser.add_argument("--rl-random-start-enabled", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--rl-random-start-min-stage", type=int, default=2)
    parser.add_argument("--rl-random-start-pre-teleport-brake-sec", type=float, default=0.5)
    parser.add_argument("--rl-random-start-stop-speed-mps", type=float, default=0.20)
    parser.add_argument("--rl-random-start-stop-timeout-sec", type=float, default=2.0)
    parser.add_argument("--rl-random-start-disable-after-failures", type=int, default=1)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.count < 1:
        raise SystemExit("--count must be at least 1")
    spawn_warning = ""
    if args.spawn_mode != "start" and not args.spawn_settings_supported:
        spawn_warning = (
            "WARNING: this packaged simulator ignores vehicle X/Y/Z spawn settings. "
            "Falling back to the normal start; pass --spawn-settings-supported only after rebuilding the UE binary with spawn-position support."
        )
        args.spawn_mode = "start"

    template = load_json(args.settings_template)
    model_path = find_model(args.model_path)
    centerline = load_centerline(args.map_dir, args.track_id)
    spawn_indices = choose_spawn_indices(args.count, centerline, args)
    resume_bases = args.resume_search_base or [
        args.run_base,
        Path("/home/hard/.fsds_autonomy/runs/multisim_stage2_random"),
        Path("/home/hard/.fsds_autonomy/runs/multisim_varsteps"),
    ]

    print(f"Using cone model: {model_path}", flush=True)
    print(f"Using map: {map_file(args.map_dir, args.track_id)}", flush=True)
    if spawn_warning:
        print(spawn_warning, flush=True)
    if args.spawn_mode != "start":
        print("Randomization mode: simulator-spawn sectors, no runtime teleporting", flush=True)
    else:
        print("Randomization mode: disabled, all agents start from the normal start", flush=True)

    run_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    settings_paths: list[Path] = []
    for offset in range(args.count):
        index = args.first_index + offset
        domain = args.base_domain + offset
        api_port = args.base_port + offset
        seed = args.base_seed + offset
        spawn_index = spawn_indices[offset]
        x, y = centerline[spawn_index]
        yaw = heading_deg(centerline, spawn_index)
        spawn = None if args.spawn_mode == "start" else (x, y, yaw)
        settings_path = args.settings_dir.expanduser() / f"settings_{index}_{api_port}_spawn_{run_id}.json"
        make_agent_settings(template, settings_path, api_port, spawn, args.vehicle_name)
        settings_paths.append(settings_path)
        agent_name = f"msim{index}_sac_seed{seed}_domain{domain}"
        run_dir = args.run_base.expanduser() / f"{agent_name}_spawn_{run_id}"
        checkpoint = args.load_rl_model.expanduser() if args.load_rl_model is not None else latest_checkpoint(resume_bases, agent_name)
        if checkpoint is not None and not checkpoint.exists():
            raise FileNotFoundError(checkpoint)
        checkpoint_text = checkpoint.name if checkpoint is not None else "fresh"
        print(
            f"msim{index}: domain={domain} port={api_port} seed={seed} "
            f"spawn_index={spawn_index} x={x:.2f} y={y:.2f} yaw={yaw:.1f} checkpoint={checkpoint_text}",
            flush=True,
        )
        if not args.dry_run:
            start_agent(index, domain, seed, api_port, settings_path, model_path, run_dir, checkpoint, args)

    if args.dry_run:
        print("Dry run complete; settings files were written but no tmux sessions were started.", flush=True)
    else:
        print("Started multi-agent RL sessions. Check with: tmux ls | grep msim", flush=True)
        print("Logs are under:", args.log_dir.expanduser(), flush=True)
        print("Run outputs are under:", args.run_base.expanduser(), flush=True)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import rclpy

from fsds_autonomy.rl.live_env import LiveFullControlEnv


class LiveRlMetricsCallback:
    def __init__(self) -> None:
        from stable_baselines3.common.callbacks import BaseCallback

        class _Callback(BaseCallback):
            def _on_step(inner_self) -> bool:
                infos: list[dict[str, Any]] = inner_self.locals.get("infos", [])
                for info in infos:
                    if not info:
                        continue
                    for key in (
                        "episode_progress_m",
                        "episode_progress_fraction",
                        "episode_step_limit",
                        "episode_step_fraction",
                        "path_length_m",
                        "speed_mps",
                        "episode_avg_speed_mps",
                        "episode_max_speed_mps",
                        "centerline_error_m",
                        "racing_line_error_m",
                        "min_clearance_m",
                        "episode_min_clearance_m",
                        "episode_avg_clearance_m",
                        "sector_count",
                        "stage",
                        "stage_after_episode",
                        "stage_changed",
                        "direct_blend",
                        "danger_zone_strength",
                        "mistake_budget",
                        "mistake_budget_limit",
                        "mistake_increment",
                        "offtrack_steps",
                        "reset_bad_steps",
                        "low_clearance_steps",
                        "wrong_direction_steps",
                        "no_progress_steps",
                        "reward_total",
                        "episode_reward_total",
                        "reward_lap_completion",
                        "reward_sector",
                        "reward_speed",
                        "reward_danger_zone",
                        "reward_low_clearance",
                        "reward_critical_clearance",
                        "reward_no_progress",
                        "centerline_x_m",
                        "centerline_y_m",
                        "racing_line_x_m",
                        "racing_line_y_m",
                        "yellow_line_x_m",
                        "yellow_line_y_m",
                        "blue_line_x_m",
                        "blue_line_y_m",
                    ):
                        if key in info:
                            inner_self.logger.record(f"live_rl/{key}", info[key])
                    if info.get("cone_hit"):
                        inner_self.logger.record("live_rl/cone_hit", 1.0)
                    if info.get("reset_bad"):
                        inner_self.logger.record("live_rl/reset_bad", 1.0)
                    if info.get("offtrack"):
                        inner_self.logger.record("live_rl/offtrack", 1.0)
                    reason = str(info.get("terminal_reason") or "")
                    if reason:
                        inner_self.logger.record("live_rl/terminal", 1.0)
                return True

        self.callback = _Callback()


def build_model(algo: str, env, args):
    learning_rate = linear_schedule(args.learning_rate, args.final_learning_rate) if args.auto_learning_rate else args.learning_rate
    if args.load_model:
        load_path = str(args.load_model.expanduser())
        if algo == "SAC":
            from stable_baselines3 import SAC as Algo
        elif algo == "TD3":
            from stable_baselines3 import TD3 as Algo
        else:
            from stable_baselines3 import PPO as Algo
        print(f"Loading existing {algo} policy from {load_path}", flush=True)
        return Algo.load(
            load_path,
            env=env,
            device=args.device,
            tensorboard_log=str(args.out),
            custom_objects={"learning_rate": learning_rate},
        )

    model_kwargs = {
        "verbose": 1,
        "tensorboard_log": str(args.out),
        "device": args.device,
        "seed": args.seed,
        "learning_rate": learning_rate,
    }
    if algo == "SAC":
        from stable_baselines3 import SAC as Algo

        model_kwargs.update(
            {
                "learning_starts": args.learning_starts,
                "buffer_size": args.buffer_size,
                "train_freq": 1,
                "gradient_steps": 1,
            }
        )
    elif algo == "TD3":
        from stable_baselines3 import TD3 as Algo

        model_kwargs.update(
            {
                "learning_starts": args.learning_starts,
                "buffer_size": args.buffer_size,
                "train_freq": (1, "step"),
                "gradient_steps": 1,
            }
        )
    else:
        from stable_baselines3 import PPO as Algo

        model_kwargs.update({"n_steps": args.ppo_n_steps, "batch_size": args.ppo_batch_size})
    return Algo("MlpPolicy", env, **model_kwargs)


def load_replay_buffer_if_available(model, algo: str, args) -> None:
    if algo not in {"SAC", "TD3"} or not hasattr(model, "load_replay_buffer"):
        return
    replay_path = args.load_replay_buffer
    if replay_path is None and args.auto_load_replay_buffer and args.load_model:
        model_path = args.load_model.expanduser()
        stem = model_path.stem
        suffix = "_steps"
        if stem.endswith(suffix):
            prefix = stem[: -len(suffix)]
            step_part = prefix.rsplit("_", 1)[-1]
            base_prefix = prefix[: -(len(step_part) + 1)]
            candidate = model_path.with_name(f"{base_prefix}_replay_buffer_{step_part}_steps.pkl")
            if candidate.exists():
                replay_path = candidate
    if replay_path is None:
        return
    replay_path = replay_path.expanduser()
    if not replay_path.exists():
        print(f"Replay buffer not found, continuing without it: {replay_path}", flush=True)
        return
    print(f"Loading replay buffer from {replay_path}", flush=True)
    model.load_replay_buffer(str(replay_path))


def linear_schedule(initial: float, final: float):
    initial = float(initial)
    final = float(final)

    def schedule(progress_remaining: float) -> float:
        progress_remaining = max(0.0, min(1.0, float(progress_remaining)))
        return final + (initial - final) * progress_remaining

    return schedule


def read_monitor_summary(path: Path) -> dict[str, float]:
    if not path.exists():
        return {"episodes": 0.0, "best_progress": 0.0, "best_reward": -math.inf, "lap_completions": 0.0}
    rows = []
    with path.open(newline="", encoding="utf-8") as handle:
        filtered = (line for line in handle if not line.startswith("#"))
        rows = list(csv.DictReader(filtered))
    if not rows:
        return {"episodes": 0.0, "best_progress": 0.0, "best_reward": -math.inf, "lap_completions": 0.0}
    progress = [float(row.get("episode_progress_m") or 0.0) for row in rows]
    rewards = [float(row.get("r") or 0.0) for row in rows]
    laps = sum(1 for row in rows if str(row.get("terminal_reason") or "") == "lap_complete")
    recent = rows[-min(10, len(rows)) :]
    recent_progress = [float(row.get("episode_progress_m") or 0.0) for row in recent]
    recent_rewards = [float(row.get("r") or 0.0) for row in recent]
    return {
        "episodes": float(len(rows)),
        "best_progress": max(progress),
        "best_reward": max(rewards),
        "lap_completions": float(laps),
        "recent_avg_progress": sum(recent_progress) / max(1, len(recent_progress)),
        "recent_avg_reward": sum(recent_rewards) / max(1, len(recent_rewards)),
    }


def learn_with_optional_auto_steps(model, args, callbacks) -> None:
    if not args.auto_steps:
        model.learn(total_timesteps=args.steps, callback=callbacks, progress_bar=False)
        return

    monitor_path = args.out / "monitor.csv"
    trained_steps = 0
    stale_chunks = 0
    best_score = -math.inf
    max_steps = max(1, int(args.steps))
    min_steps = min(max_steps, max(1, int(args.min_steps)))
    step_chunk = max(1, int(args.step_chunk))
    while trained_steps < max_steps:
        chunk = min(step_chunk, max_steps - trained_steps)
        model.learn(
            total_timesteps=chunk,
            callback=callbacks,
            progress_bar=False,
            reset_num_timesteps=(trained_steps == 0),
        )
        trained_steps += chunk
        summary = read_monitor_summary(monitor_path)
        score = (
            10_000.0 * summary["lap_completions"]
            + summary["best_progress"]
            + 0.01 * max(-100_000.0, summary["best_reward"])
        )
        improved = score > best_score + args.auto_step_min_score_improvement
        if improved:
            best_score = score
            stale_chunks = 0
        else:
            stale_chunks += 1
        print(
            "auto_steps "
            f"trained={trained_steps}/{max_steps} episodes={int(summary['episodes'])} "
            f"best_progress={summary['best_progress']:.1f}m laps={int(summary['lap_completions'])} "
            f"recent_reward={summary.get('recent_avg_reward', 0.0):.1f} stale_chunks={stale_chunks}",
            flush=True,
        )
        if trained_steps >= min_steps and stale_chunks >= args.auto_step_patience_chunks:
            print("auto_steps stopping after progress plateau within the configured step budget", flush=True)
            break


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a live full-control RL policy in the running FSDS simulator.")
    parser.add_argument("--algo", default="SAC", choices=["SAC", "TD3", "PPO"])
    parser.add_argument("--steps", type=int, default=200_000)
    parser.add_argument("--auto-steps", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--min-steps", type=int, default=50_000)
    parser.add_argument("--step-chunk", type=int, default=10_000)
    parser.add_argument("--auto-step-patience-chunks", type=int, default=4)
    parser.add_argument("--auto-step-min-score-improvement", type=float, default=1.0)
    parser.add_argument("--learning-starts", type=int, default=1000)
    parser.add_argument("--buffer-size", type=int, default=100_000)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--final-learning-rate", type=float, default=5e-5)
    parser.add_argument("--auto-learning-rate", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--out", type=Path, default=Path("/home/hard/.fsds_autonomy/runs/rl_live_full_control"))
    parser.add_argument("--load-model", type=Path, default=None, help="Optional SAC/TD3/PPO .zip to resume or BC-warm-start from.")
    parser.add_argument("--load-replay-buffer", type=Path, default=None)
    parser.add_argument("--auto-load-replay-buffer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--step-dt", type=float, default=0.10)
    parser.add_argument("--max-episode-steps", type=int, default=8000)
    parser.add_argument("--min-episode-steps", type=int, default=3500)
    parser.add_argument("--variable-episode-steps", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--min-track-quality", type=float, default=0.60)
    parser.add_argument("--require-closed-loop", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-on-episode", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--map-wait-timeout-sec", type=float, default=3600.0)
    parser.add_argument("--map-dir", type=Path, default=Path("/home/hard/.fsds_autonomy/maps"))
    parser.add_argument("--danger-zone-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--danger-zone-radius-m", type=float, default=7.0)
    parser.add_argument("--danger-zone-penalty", type=float, default=0.35)
    parser.add_argument("--random-start-enabled", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--random-start-min-stage", type=int, default=2)
    parser.add_argument("--random-start-settle-sec", type=float, default=0.6)
    parser.add_argument("--random-start-host", default="127.0.0.1")
    parser.add_argument("--random-start-port", type=int, default=41451)
    parser.add_argument("--random-start-vehicle", default="FSCar")
    parser.add_argument("--random-start-z-offset-m", type=float, default=0.0)
    parser.add_argument("--random-start-reset-before-teleport", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--random-start-skip-reset", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--random-start-pause-during-teleport", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--random-start-pre-teleport-brake-sec", type=float, default=0.5)
    parser.add_argument("--random-start-stop-speed-mps", type=float, default=0.20)
    parser.add_argument("--random-start-stop-timeout-sec", type=float, default=2.0)
    parser.add_argument("--random-start-disable-after-failures", type=int, default=1)
    parser.add_argument("--max-path-offset-m", type=float, default=0.60)
    parser.add_argument("--max-speed-delta-mps", type=float, default=1.20)
    parser.add_argument("--max-direct-steering", type=float, default=1.0)
    parser.add_argument("--max-direct-throttle", type=float, default=1.0)
    parser.add_argument("--max-direct-brake", type=float, default=1.0)
    parser.add_argument("--cone-count", type=int, default=12)
    parser.add_argument("--preview-count", type=int, default=8)
    parser.add_argument("--sector-count", type=int, default=24)
    parser.add_argument("--start-stage", type=int, default=1)
    parser.add_argument("--stage2-min-sectors", type=int, default=12)
    parser.add_argument("--stage3-min-sectors", type=int, default=22)
    parser.add_argument("--stage2-clean-episodes", type=int, default=3)
    parser.add_argument("--stage3-clean-episodes", type=int, default=4)
    parser.add_argument("--stage-bad-episode-demotion-count", type=int, default=4)
    parser.add_argument("--stage1-residual-scale", type=float, default=0.25)
    parser.add_argument("--stage2-residual-scale", type=float, default=0.50)
    parser.add_argument("--stage3-residual-scale", type=float, default=0.80)
    parser.add_argument("--stage4-clean-laps", type=int, default=2)
    parser.add_argument("--stage3-direct-blend", type=float, default=0.35)
    parser.add_argument("--stage4-direct-blend", type=float, default=1.0)
    parser.add_argument("--reset-status-hold-terminal-sec", type=float, default=1.5)
    parser.add_argument("--progress-reward", type=float, default=4.0)
    parser.add_argument("--sector-bonus", type=float, default=8.0)
    parser.add_argument("--sector-time-reference-sec", type=float, default=25.0)
    parser.add_argument("--sector-time-reference-reward", type=float, default=0.5)
    parser.add_argument("--sector-time-reward-max", type=float, default=10.0)
    parser.add_argument("--speed-reward", type=float, default=0.04)
    parser.add_argument("--speed-reward-power", type=float, default=2.0)
    parser.add_argument("--speed-reward-max-per-step", type=float, default=0.40)
    parser.add_argument("--speed-reward-min-clearance-m", type=float, default=1.00)
    parser.add_argument("--speed-reward-full-clearance-m", type=float, default=1.60)
    parser.add_argument("--time-penalty", type=float, default=0.0)
    parser.add_argument("--centerline-penalty", type=float, default=0.04)
    parser.add_argument("--racing-line-penalty", type=float, default=0.03)
    parser.add_argument("--low-clearance-penalty", type=float, default=3.0)
    parser.add_argument("--backward-penalty", type=float, default=8.0)
    parser.add_argument("--action-penalty", type=float, default=0.02)
    parser.add_argument("--action-smoothness-penalty", type=float, default=0.08)
    parser.add_argument("--throttle-brake-conflict-penalty", type=float, default=1.5)
    parser.add_argument("--cone-hit-penalty", type=float, default=180.0)
    parser.add_argument("--reset-penalty", type=float, default=220.0)
    parser.add_argument("--offtrack-penalty", type=float, default=180.0)
    parser.add_argument("--stuck-penalty", type=float, default=60.0)
    parser.add_argument("--lap-bonus", type=float, default=250.0)
    parser.add_argument("--lap-time-reference-sec", type=float, default=600.0)
    parser.add_argument("--lap-time-reference-reward", type=float, default=50.0)
    parser.add_argument("--lap-time-reward-max", type=float, default=1000.0)
    parser.add_argument("--safe-clearance-m", type=float, default=1.20)
    parser.add_argument("--promotion-min-clearance-m", type=float, default=0.90)
    parser.add_argument("--mistake-budget-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--mistake-budget-limit", type=float, default=35.0)
    parser.add_argument("--mistake-recovery-per-step", type=float, default=0.35)
    parser.add_argument("--offtrack-grace-steps", type=int, default=20)
    parser.add_argument("--reset-bad-grace-steps", type=int, default=15)
    parser.add_argument("--low-clearance-terminal-m", type=float, default=0.35)
    parser.add_argument("--low-clearance-grace-steps", type=int, default=30)
    parser.add_argument("--wrong-direction-grace-steps", type=int, default=25)
    parser.add_argument("--no-progress-grace-steps", type=int, default=250)
    parser.add_argument("--no-progress-min-delta-m", type=float, default=0.01)
    parser.add_argument("--offtrack-step-penalty", type=float, default=4.0)
    parser.add_argument("--reset-bad-step-penalty", type=float, default=5.0)
    parser.add_argument("--low-clearance-step-penalty", type=float, default=1.5)
    parser.add_argument("--wrong-direction-step-penalty", type=float, default=1.0)
    parser.add_argument("--no-progress-step-penalty", type=float, default=0.20)
    parser.add_argument("--reward-log", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reward-log-path", type=Path, default=None)
    parser.add_argument("--reward-log-flush-every", type=int, default=25)
    parser.add_argument("--episode-summary-log", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--episode-summary-log-path", type=Path, default=None)
    parser.add_argument("--episode-summary-log-flush-every", type=int, default=1)
    parser.add_argument("--checkpoint-freq", type=int, default=10_000)
    parser.add_argument("--ppo-n-steps", type=int, default=1024)
    parser.add_argument("--ppo-batch-size", type=int, default=128)
    args = parser.parse_args()

    args.out = args.out.expanduser()
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "checkpoints").mkdir(parents=True, exist_ok=True)
    reward_log_path = ""
    if args.reward_log:
        reward_log_path = str((args.reward_log_path or (args.out / "reward_steps.jsonl")).expanduser())
    episode_summary_log_path = ""
    if args.episode_summary_log:
        episode_summary_log_path = str(
            (args.episode_summary_log_path or (args.out / "episode_summaries.jsonl")).expanduser()
        )

    rclpy.init()
    env = None
    try:
        from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
        from stable_baselines3.common.monitor import Monitor

        env = LiveFullControlEnv(
            step_dt=args.step_dt,
            max_episode_steps=args.max_episode_steps,
            min_episode_steps=args.min_episode_steps,
            variable_episode_steps_enabled=args.variable_episode_steps,
            max_path_offset_m=args.max_path_offset_m,
            max_speed_delta_mps=args.max_speed_delta_mps,
            max_direct_steering=args.max_direct_steering,
            max_direct_throttle=args.max_direct_throttle,
            max_direct_brake=args.max_direct_brake,
            min_track_quality=args.min_track_quality,
            require_closed_loop=args.require_closed_loop,
            reset_on_episode=args.reset_on_episode,
            map_wait_timeout_sec=args.map_wait_timeout_sec,
            map_dir=str(args.map_dir.expanduser()),
            danger_zone_enabled=args.danger_zone_enabled,
            danger_zone_radius_m=args.danger_zone_radius_m,
            danger_zone_penalty=args.danger_zone_penalty,
            random_start_enabled=args.random_start_enabled,
            random_start_min_stage=args.random_start_min_stage,
            random_start_settle_sec=args.random_start_settle_sec,
            random_start_host=args.random_start_host,
            random_start_port=args.random_start_port,
            random_start_vehicle=args.random_start_vehicle,
            random_start_z_offset_m=args.random_start_z_offset_m,
            random_start_reset_before_teleport=args.random_start_reset_before_teleport,
            random_start_skip_reset=args.random_start_skip_reset,
            random_start_pause_during_teleport=args.random_start_pause_during_teleport,
            random_start_pre_teleport_brake_sec=args.random_start_pre_teleport_brake_sec,
            random_start_stop_speed_mps=args.random_start_stop_speed_mps,
            random_start_stop_timeout_sec=args.random_start_stop_timeout_sec,
            random_start_disable_after_failures=args.random_start_disable_after_failures,
            cone_count=args.cone_count,
            preview_count=args.preview_count,
            sector_count=args.sector_count,
            start_stage=args.start_stage,
            stage2_min_sectors=args.stage2_min_sectors,
            stage3_min_sectors=args.stage3_min_sectors,
            stage2_clean_episodes=args.stage2_clean_episodes,
            stage3_clean_episodes=args.stage3_clean_episodes,
            stage_bad_episode_demotion_count=args.stage_bad_episode_demotion_count,
            stage1_residual_scale=args.stage1_residual_scale,
            stage2_residual_scale=args.stage2_residual_scale,
            stage3_residual_scale=args.stage3_residual_scale,
            stage4_clean_laps=args.stage4_clean_laps,
            stage3_direct_blend=args.stage3_direct_blend,
            stage4_direct_blend=args.stage4_direct_blend,
            progress_reward=args.progress_reward,
            sector_bonus=args.sector_bonus,
            sector_time_reference_sec=args.sector_time_reference_sec,
            sector_time_reference_reward=args.sector_time_reference_reward,
            sector_time_reward_max=args.sector_time_reward_max,
            speed_reward=args.speed_reward,
            speed_reward_power=args.speed_reward_power,
            speed_reward_max_per_step=args.speed_reward_max_per_step,
            speed_reward_min_clearance_m=args.speed_reward_min_clearance_m,
            speed_reward_full_clearance_m=args.speed_reward_full_clearance_m,
            time_penalty=args.time_penalty,
            centerline_penalty=args.centerline_penalty,
            racing_line_penalty=args.racing_line_penalty,
            low_clearance_penalty=args.low_clearance_penalty,
            backward_penalty=args.backward_penalty,
            action_penalty=args.action_penalty,
            action_smoothness_penalty=args.action_smoothness_penalty,
            throttle_brake_conflict_penalty=args.throttle_brake_conflict_penalty,
            cone_hit_penalty=args.cone_hit_penalty,
            reset_penalty=args.reset_penalty,
            offtrack_penalty=args.offtrack_penalty,
            stuck_penalty=args.stuck_penalty,
            lap_bonus=args.lap_bonus,
            lap_time_reference_sec=args.lap_time_reference_sec,
            lap_time_reference_reward=args.lap_time_reference_reward,
            lap_time_reward_max=args.lap_time_reward_max,
            safe_clearance_m=args.safe_clearance_m,
            promotion_min_clearance_m=args.promotion_min_clearance_m,
            reset_status_hold_terminal_sec=args.reset_status_hold_terminal_sec,
            mistake_budget_enabled=args.mistake_budget_enabled,
            mistake_budget_limit=args.mistake_budget_limit,
            mistake_recovery_per_step=args.mistake_recovery_per_step,
            offtrack_grace_steps=args.offtrack_grace_steps,
            reset_bad_grace_steps=args.reset_bad_grace_steps,
            low_clearance_terminal_m=args.low_clearance_terminal_m,
            low_clearance_grace_steps=args.low_clearance_grace_steps,
            wrong_direction_grace_steps=args.wrong_direction_grace_steps,
            no_progress_grace_steps=args.no_progress_grace_steps,
            no_progress_min_delta_m=args.no_progress_min_delta_m,
            offtrack_step_penalty=args.offtrack_step_penalty,
            reset_bad_step_penalty=args.reset_bad_step_penalty,
            low_clearance_step_penalty=args.low_clearance_step_penalty,
            wrong_direction_step_penalty=args.wrong_direction_step_penalty,
            no_progress_step_penalty=args.no_progress_step_penalty,
            reward_log_path=reward_log_path,
            reward_log_flush_every=args.reward_log_flush_every,
            episode_summary_log_path=episode_summary_log_path,
            episode_summary_log_flush_every=args.episode_summary_log_flush_every,
            seed=args.seed,
        )
        env = Monitor(
            env,
            filename=str(args.out / "monitor.csv"),
            info_keywords=(
                "episode_progress_m",
                "episode_progress_fraction",
                "episode_step_limit",
                "episode_step_fraction",
                "path_length_m",
                "speed_mps",
                "episode_avg_speed_mps",
                "episode_max_speed_mps",
                "min_clearance_m",
                "episode_min_clearance_m",
                "episode_avg_clearance_m",
                "sector_count",
                "stage",
                "stage_after_episode",
                "stage_changed",
                "direct_blend",
                "danger_zone_strength",
                "mistake_budget",
                "mistake_budget_limit",
                "mistake_increment",
                "offtrack_steps",
                "reset_bad_steps",
                "low_clearance_steps",
                "wrong_direction_steps",
                "no_progress_steps",
                "terminal_reason",
                "reward_total",
                "episode_reward_total",
                "reward_lap_completion",
                "reward_sector",
                "reward_speed",
                "reward_danger_zone",
                "reward_low_clearance",
                "reward_critical_clearance",
                "reward_no_progress",
                "centerline_x_m",
                "centerline_y_m",
                "racing_line_x_m",
                "racing_line_y_m",
                "yellow_line_x_m",
                "yellow_line_y_m",
                "blue_line_x_m",
                "blue_line_y_m",
            ),
        )
        model = build_model(args.algo, env, args)
        load_replay_buffer_if_available(model, args.algo, args)
        callbacks = [
            LiveRlMetricsCallback().callback,
            CheckpointCallback(
                save_freq=max(1, args.checkpoint_freq),
                save_path=str(args.out / "checkpoints"),
                name_prefix=f"{args.algo.lower()}_live_full_control",
                save_replay_buffer=args.algo in {"SAC", "TD3"},
                save_vecnormalize=False,
            ),
        ]
        learn_with_optional_auto_steps(model, args, CallbackList(callbacks))
        model_path = args.out / f"{args.algo.lower()}_live_full_control"
        model.save(str(model_path))
        if args.algo in {"SAC", "TD3"} and hasattr(model, "save_replay_buffer"):
            model.save_replay_buffer(str(args.out / f"{args.algo.lower()}_live_full_control_replay_buffer"))
        print(f"Saved live full-control RL policy to {model_path}.zip")
    finally:
        if env is not None:
            env.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

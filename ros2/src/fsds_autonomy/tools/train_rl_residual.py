#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from fsds_autonomy.rl.env import SavedMapResidualEnv


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a residual speed/path policy from a saved FSDS map.")
    parser.add_argument("--map-dir", required=True)
    parser.add_argument("--track-id", required=True)
    parser.add_argument("--algo", default="SAC", choices=["SAC", "TD3", "PPO"])
    parser.add_argument("--steps", type=int, default=200_000)
    parser.add_argument("--learning-starts", type=int, default=1000)
    parser.add_argument("--max-env-steps", type=int, default=1500)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--out", default="runs/rl_residual")
    args = parser.parse_args()
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.algo == "SAC":
        from stable_baselines3 import SAC as Algo
    elif args.algo == "TD3":
        from stable_baselines3 import TD3 as Algo
    else:
        from stable_baselines3 import PPO as Algo

    env = SavedMapResidualEnv(args.map_dir, args.track_id, max_steps=args.max_env_steps)
    model_kwargs = {
        "verbose": 1,
        "tensorboard_log": str(out_dir),
        "device": args.device,
        "seed": args.seed,
    }
    if args.algo in {"SAC", "TD3"}:
        model_kwargs["learning_starts"] = args.learning_starts
    model = Algo("MlpPolicy", env, **model_kwargs)
    model.learn(total_timesteps=args.steps)
    model_path = out_dir / f"{args.algo.lower()}_{args.track_id}"
    model.save(str(model_path))
    print(f"Saved residual policy to {model_path}.zip")


if __name__ == "__main__":
    main()

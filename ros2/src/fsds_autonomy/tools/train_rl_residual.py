#!/usr/bin/env python3
from __future__ import annotations

import argparse

from fsds_autonomy.rl.env import SavedMapResidualEnv


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a residual speed/path policy from a saved FSDS map.")
    parser.add_argument("--map-dir", required=True)
    parser.add_argument("--track-id", required=True)
    parser.add_argument("--algo", default="SAC", choices=["SAC", "TD3", "PPO"])
    parser.add_argument("--steps", type=int, default=200_000)
    parser.add_argument("--out", default="runs/rl_residual")
    args = parser.parse_args()

    if args.algo == "SAC":
        from stable_baselines3 import SAC as Algo
    elif args.algo == "TD3":
        from stable_baselines3 import TD3 as Algo
    else:
        from stable_baselines3 import PPO as Algo

    env = SavedMapResidualEnv(args.map_dir, args.track_id)
    model = Algo("MlpPolicy", env, verbose=1, tensorboard_log=args.out)
    model.learn(total_timesteps=args.steps)
    model.save(f"{args.out}/{args.algo.lower()}_{args.track_id}")


if __name__ == "__main__":
    main()

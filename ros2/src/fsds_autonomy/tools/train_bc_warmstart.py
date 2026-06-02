#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class LoadedDataset:
    observations: np.ndarray
    actions: np.ndarray
    sources: list[str]


def parse_int_list(value: str) -> list[int]:
    items = [item.strip() for item in value.split(",") if item.strip()]
    if not items:
        raise argparse.ArgumentTypeError("expected comma-separated positive integers")
    parsed = [int(item) for item in items]
    if any(item <= 0 for item in parsed):
        raise argparse.ArgumentTypeError("hidden sizes must be positive")
    return parsed


def parse_float_list(value: str, expected: int) -> list[float]:
    items = [item.strip() for item in value.split(",") if item.strip()]
    if len(items) != expected:
        raise argparse.ArgumentTypeError(f"expected {expected} comma-separated floats")
    return [float(item) for item in items]


def expand_inputs(paths: list[Path]) -> list[Path]:
    expanded: list[Path] = []
    for path in paths:
        path = path.expanduser()
        if path.is_dir():
            expanded.extend(sorted(path.glob("*.npz")))
            expanded.extend(sorted(path.glob("*.jsonl")))
        else:
            expanded.append(path)
    seen: set[Path] = set()
    unique: list[Path] = []
    for path in expanded:
        resolved = path.resolve()
        if resolved not in seen:
            seen.add(resolved)
            unique.append(path)
    return unique


def load_jsonl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    observations: list[list[float]] = []
    actions: list[list[float]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("event") != "sample":
                continue
            obs = event.get("observation")
            action = event.get("action")
            if not isinstance(obs, list) or not isinstance(action, list):
                continue
            observations.append([float(value) for value in obs])
            actions.append([float(value) for value in action])
    if not observations:
        return np.zeros((0, 0), dtype=np.float32), np.zeros((0, 5), dtype=np.float32)
    return np.asarray(observations, dtype=np.float32), np.asarray(actions, dtype=np.float32)


def load_npz(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with np.load(path, allow_pickle=False) as data:
        observations = np.asarray(data["observations"], dtype=np.float32)
        actions = np.asarray(data["actions"], dtype=np.float32)
    return observations, actions


def load_dataset(paths: list[Path], max_samples: int, seed: int, clip_actions: bool) -> LoadedDataset:
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    sources: list[str] = []
    expected_obs_dim: int | None = None
    expected_action_dim = 5

    for path in expand_inputs(paths):
        if not path.exists():
            raise FileNotFoundError(path)
        if path.suffix == ".npz":
            obs, act = load_npz(path)
        elif path.suffix == ".jsonl":
            obs, act = load_jsonl(path)
        else:
            continue
        if obs.ndim != 2 or act.ndim != 2 or len(obs) == 0:
            continue
        if act.shape[1] != expected_action_dim:
            print(f"Skipping {path}: expected action_dim=5, got {act.shape[1]}", flush=True)
            continue
        if expected_obs_dim is None:
            expected_obs_dim = int(obs.shape[1])
        if obs.shape[1] != expected_obs_dim:
            print(f"Skipping {path}: expected obs_dim={expected_obs_dim}, got {obs.shape[1]}", flush=True)
            continue
        observations.append(obs)
        actions.append(act)
        sources.append(str(path))

    if not observations:
        raise RuntimeError("No valid BC samples found")

    all_observations = np.concatenate(observations, axis=0).astype(np.float32, copy=False)
    all_actions = np.concatenate(actions, axis=0).astype(np.float32, copy=False)
    finite = np.isfinite(all_observations).all(axis=1) & np.isfinite(all_actions).all(axis=1)
    all_observations = all_observations[finite]
    all_actions = all_actions[finite]
    if clip_actions:
        all_actions = np.clip(all_actions, -1.0, 1.0)

    if max_samples > 0 and len(all_observations) > max_samples:
        rng = np.random.default_rng(seed)
        indices = rng.choice(len(all_observations), size=max_samples, replace=False)
        all_observations = all_observations[indices]
        all_actions = all_actions[indices]

    if len(all_observations) == 0:
        raise RuntimeError("All samples were filtered out")
    return LoadedDataset(all_observations, all_actions, sources)


def split_indices(n: int, val_fraction: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    indices = rng.permutation(n)
    val_count = int(round(n * max(0.0, min(0.8, val_fraction))))
    if n > 1:
        val_count = min(max(1, val_count), n - 1)
    else:
        val_count = 0
    val_indices = indices[:val_count]
    train_indices = indices[val_count:]
    return train_indices, val_indices


def make_weighted_mse(torch_module: Any, weights: list[float]):
    import torch

    weight_tensor = torch.as_tensor(weights, dtype=torch.float32)

    def loss_fn(prediction: Any, target: Any) -> Any:
        weights_on_device = weight_tensor.to(device=prediction.device)
        return torch.mean(((prediction - target) ** 2) * weights_on_device)

    return loss_fn


def train_mlp(
    dataset: LoadedDataset,
    train_indices: np.ndarray,
    val_indices: np.ndarray,
    args: argparse.Namespace,
    run_dir: Path,
) -> dict[str, Any]:
    import torch
    from torch import nn
    from torch.utils.data import DataLoader, TensorDataset

    class BoundedMlpPolicy(nn.Module):
        def __init__(
            self,
            obs_dim: int,
            action_dim: int,
            hidden_sizes: list[int],
            obs_mean: np.ndarray,
            obs_std: np.ndarray,
            dropout: float,
        ) -> None:
            super().__init__()
            self.register_buffer("obs_mean", torch.as_tensor(obs_mean, dtype=torch.float32))
            self.register_buffer("obs_std", torch.as_tensor(obs_std, dtype=torch.float32))
            layers: list[nn.Module] = []
            last_dim = obs_dim
            for hidden_size in hidden_sizes:
                layers.append(nn.Linear(last_dim, hidden_size))
                layers.append(nn.ReLU())
                if dropout > 0.0:
                    layers.append(nn.Dropout(p=dropout))
                last_dim = hidden_size
            layers.append(nn.Linear(last_dim, action_dim))
            layers.append(nn.Tanh())
            self.net = nn.Sequential(*layers)

        def forward(self, obs: Any) -> Any:
            obs = (obs - self.obs_mean) / self.obs_std
            return self.net(obs)

    torch.manual_seed(int(args.seed))
    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)

    obs = dataset.observations
    actions = dataset.actions
    obs_mean = obs[train_indices].mean(axis=0)
    obs_std = obs[train_indices].std(axis=0)
    obs_std = np.maximum(obs_std, float(args.min_obs_std))

    train_ds = TensorDataset(
        torch.as_tensor(obs[train_indices], dtype=torch.float32),
        torch.as_tensor(actions[train_indices], dtype=torch.float32),
    )
    val_ds = TensorDataset(
        torch.as_tensor(obs[val_indices], dtype=torch.float32),
        torch.as_tensor(actions[val_indices], dtype=torch.float32),
    )
    train_loader = DataLoader(train_ds, batch_size=int(args.batch_size), shuffle=True, drop_last=False)
    val_loader = DataLoader(val_ds, batch_size=int(args.batch_size), shuffle=False, drop_last=False)

    model = BoundedMlpPolicy(
        obs_dim=obs.shape[1],
        action_dim=actions.shape[1],
        hidden_sizes=args.hidden_sizes,
        obs_mean=obs_mean,
        obs_std=obs_std,
        dropout=max(0.0, float(args.dropout)),
    ).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=float(args.lr), weight_decay=float(args.weight_decay))
    loss_fn = make_weighted_mse(torch, args.action_weights)

    best_val = float("inf")
    best_state: dict[str, Any] | None = None
    history: list[dict[str, float]] = []
    patience_left = int(args.patience)

    for epoch in range(1, int(args.epochs) + 1):
        model.train()
        train_loss_sum = 0.0
        train_count = 0
        for batch_obs, batch_actions in train_loader:
            batch_obs = batch_obs.to(device)
            batch_actions = batch_actions.to(device)
            optimizer.zero_grad(set_to_none=True)
            prediction = model(batch_obs)
            loss = loss_fn(prediction, batch_actions)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=float(args.grad_clip_norm))
            optimizer.step()
            train_loss_sum += float(loss.detach().cpu()) * len(batch_obs)
            train_count += len(batch_obs)

        model.eval()
        val_loss_sum = 0.0
        val_count = 0
        with torch.no_grad():
            for batch_obs, batch_actions in val_loader:
                batch_obs = batch_obs.to(device)
                batch_actions = batch_actions.to(device)
                prediction = model(batch_obs)
                loss = loss_fn(prediction, batch_actions)
                val_loss_sum += float(loss.detach().cpu()) * len(batch_obs)
                val_count += len(batch_obs)

        train_loss = train_loss_sum / max(1, train_count)
        val_loss = val_loss_sum / max(1, val_count) if val_count else train_loss
        history.append({"epoch": float(epoch), "train_loss": train_loss, "val_loss": val_loss})
        print(f"mlp epoch={epoch:03d} train_loss={train_loss:.6f} val_loss={val_loss:.6f}", flush=True)

        if val_loss + float(args.min_delta) < best_val:
            best_val = val_loss
            best_state = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}
            patience_left = int(args.patience)
        else:
            patience_left -= 1
            if int(args.patience) > 0 and patience_left <= 0:
                print("mlp early stopping after validation plateau", flush=True)
                break

    if best_state is not None:
        model.load_state_dict(best_state)
    model.eval()

    model_path = run_dir / "bc_mlp_policy.pt"
    checkpoint = {
        "schema": "fsds_bc_mlp_policy_v1",
        "model_state_dict": model.state_dict(),
        "obs_dim": int(obs.shape[1]),
        "action_dim": int(actions.shape[1]),
        "hidden_sizes": list(args.hidden_sizes),
        "dropout": float(args.dropout),
        "obs_mean": obs_mean.astype(np.float32),
        "obs_std": obs_std.astype(np.float32),
        "action_names": [
            "residual_path_offset",
            "residual_speed_delta",
            "direct_steering",
            "direct_throttle",
            "direct_brake",
        ],
        "best_val_loss": float(best_val),
        "history": history,
    }
    torch.save(checkpoint, model_path)
    return {
        "artifact": "mlp",
        "path": str(model_path),
        "best_val_loss": float(best_val),
        "epochs": len(history),
    }


def train_sac_actor(
    dataset: LoadedDataset,
    train_indices: np.ndarray,
    val_indices: np.ndarray,
    args: argparse.Namespace,
    run_dir: Path,
) -> dict[str, Any]:
    import torch
    from torch.utils.data import DataLoader, TensorDataset

    try:
        import gymnasium as gym
        from gymnasium import spaces
        from stable_baselines3 import SAC
    except Exception as exc:
        raise RuntimeError(f"SAC warm-start requires gymnasium and stable_baselines3: {exc}") from exc

    class ShapeOnlyEnv(gym.Env):
        metadata = {"render_modes": []}

        def __init__(self, obs_dim: int, action_dim: int) -> None:
            super().__init__()
            self.observation_space = spaces.Box(
                low=np.full(obs_dim, -1000.0, dtype=np.float32),
                high=np.full(obs_dim, 1000.0, dtype=np.float32),
                dtype=np.float32,
            )
            self.action_space = spaces.Box(
                low=np.full(action_dim, -1.0, dtype=np.float32),
                high=np.full(action_dim, 1.0, dtype=np.float32),
                dtype=np.float32,
            )

        def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
            super().reset(seed=seed)
            return np.zeros(self.observation_space.shape, dtype=np.float32), {}

        def step(self, action: np.ndarray):
            obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            return obs, 0.0, False, False, {}

    def resolve_device(device_name: str) -> torch.device:
        if device_name == "auto":
            return torch.device("cuda" if torch.cuda.is_available() else "cpu")
        return torch.device(device_name)

    torch.manual_seed(int(args.seed))
    obs = dataset.observations
    actions = dataset.actions
    env = ShapeOnlyEnv(obs.shape[1], actions.shape[1])
    policy_kwargs = {"net_arch": list(args.sac_net_arch)}
    if args.sac_load:
        model = SAC.load(str(args.sac_load.expanduser()), env=env, device=args.device)
    else:
        model = SAC(
            "MlpPolicy",
            env,
            verbose=0,
            seed=int(args.seed),
            device=args.device,
            learning_rate=float(args.sac_lr),
            buffer_size=max(1, int(args.sac_buffer_size)),
            learning_starts=1,
            batch_size=int(args.batch_size),
            policy_kwargs=policy_kwargs,
        )

    device = torch.device(model.device)
    train_ds = TensorDataset(
        torch.as_tensor(obs[train_indices], dtype=torch.float32),
        torch.as_tensor(actions[train_indices], dtype=torch.float32),
    )
    val_ds = TensorDataset(
        torch.as_tensor(obs[val_indices], dtype=torch.float32),
        torch.as_tensor(actions[val_indices], dtype=torch.float32),
    )
    train_loader = DataLoader(train_ds, batch_size=int(args.batch_size), shuffle=True, drop_last=False)
    val_loader = DataLoader(val_ds, batch_size=int(args.batch_size), shuffle=False, drop_last=False)
    actor = model.actor
    actor.train()
    optimizer = torch.optim.AdamW(actor.parameters(), lr=float(args.sac_lr), weight_decay=float(args.weight_decay))
    loss_fn = make_weighted_mse(torch, args.action_weights)

    best_val = float("inf")
    best_state: dict[str, Any] | None = None
    patience_left = int(args.patience)
    history: list[dict[str, float]] = []

    for epoch in range(1, int(args.epochs) + 1):
        actor.train()
        train_loss_sum = 0.0
        train_count = 0
        for batch_obs, batch_actions in train_loader:
            batch_obs = batch_obs.to(device)
            batch_actions = batch_actions.to(device)
            optimizer.zero_grad(set_to_none=True)
            prediction = actor(batch_obs, deterministic=True)
            loss = loss_fn(prediction, batch_actions)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(actor.parameters(), max_norm=float(args.grad_clip_norm))
            optimizer.step()
            train_loss_sum += float(loss.detach().cpu()) * len(batch_obs)
            train_count += len(batch_obs)

        actor.eval()
        val_loss_sum = 0.0
        val_count = 0
        with torch.no_grad():
            for batch_obs, batch_actions in val_loader:
                batch_obs = batch_obs.to(device)
                batch_actions = batch_actions.to(device)
                prediction = actor(batch_obs, deterministic=True)
                loss = loss_fn(prediction, batch_actions)
                val_loss_sum += float(loss.detach().cpu()) * len(batch_obs)
                val_count += len(batch_obs)

        train_loss = train_loss_sum / max(1, train_count)
        val_loss = val_loss_sum / max(1, val_count) if val_count else train_loss
        history.append({"epoch": float(epoch), "train_loss": train_loss, "val_loss": val_loss})
        print(f"sac_actor epoch={epoch:03d} train_loss={train_loss:.6f} val_loss={val_loss:.6f}", flush=True)

        if val_loss + float(args.min_delta) < best_val:
            best_val = val_loss
            best_state = {key: value.detach().cpu().clone() for key, value in actor.state_dict().items()}
            patience_left = int(args.patience)
        else:
            patience_left -= 1
            if int(args.patience) > 0 and patience_left <= 0:
                print("sac_actor early stopping after validation plateau", flush=True)
                break

    if best_state is not None:
        actor.load_state_dict(best_state)
    model.policy.actor.load_state_dict(actor.state_dict())
    model_path = run_dir / "sac_bc_warmstart"
    model.save(str(model_path))
    history_path = run_dir / "sac_bc_warmstart_history.json"
    history_path.write_text(json.dumps(history, indent=2, sort_keys=True), encoding="utf-8")
    return {
        "artifact": "sac",
        "path": str(model_path) + ".zip",
        "history_path": str(history_path),
        "best_val_loss": float(best_val),
        "epochs": len(history),
    }


def write_manifest(
    run_dir: Path,
    dataset: LoadedDataset,
    train_indices: np.ndarray,
    val_indices: np.ndarray,
    args: argparse.Namespace,
    artifacts: list[dict[str, Any]],
) -> Path:
    action_stats = {
        "min": dataset.actions.min(axis=0).astype(float).tolist(),
        "max": dataset.actions.max(axis=0).astype(float).tolist(),
        "mean": dataset.actions.mean(axis=0).astype(float).tolist(),
        "std": dataset.actions.std(axis=0).astype(float).tolist(),
    }
    manifest = {
        "schema": "fsds_bc_warmstart_manifest_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "sources": dataset.sources,
        "sample_count": int(len(dataset.observations)),
        "train_count": int(len(train_indices)),
        "val_count": int(len(val_indices)),
        "obs_dim": int(dataset.observations.shape[1]),
        "action_dim": int(dataset.actions.shape[1]),
        "action_stats": action_stats,
        "args": {
            "artifact": args.artifact,
            "epochs": int(args.epochs),
            "batch_size": int(args.batch_size),
            "lr": float(args.lr),
            "sac_lr": float(args.sac_lr),
            "hidden_sizes": list(args.hidden_sizes),
            "sac_net_arch": list(args.sac_net_arch),
            "seed": int(args.seed),
        },
        "artifacts": artifacts,
    }
    path = run_dir / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Train bounded behavior-cloning warm-start artifacts from collect_bc_demos JSONL/NPZ logs. "
            "The action target is the LiveFullControlEnv 5D SAC action vector in [-1, 1]."
        )
    )
    parser.add_argument("logs", nargs="+", type=Path, help="BC demo .jsonl/.npz files, or directories containing them.")
    parser.add_argument("--out-dir", type=Path, default=Path("~/.fsds_autonomy/bc_warmstarts"))
    parser.add_argument("--run-id", default="")
    parser.add_argument("--artifact", choices=["mlp", "sac", "both"], default="sac")
    parser.add_argument("--epochs", type=int, default=35)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--sac-lr", type=float, default=3e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-5)
    parser.add_argument("--dropout", type=float, default=0.0)
    parser.add_argument("--grad-clip-norm", type=float, default=5.0)
    parser.add_argument("--min-delta", type=float, default=1e-6)
    parser.add_argument("--patience", type=int, default=8)
    parser.add_argument("--val-fraction", type=float, default=0.10)
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--hidden-sizes", type=parse_int_list, default=parse_int_list("256,256"))
    parser.add_argument("--sac-net-arch", type=parse_int_list, default=parse_int_list("256,256"))
    parser.add_argument("--sac-buffer-size", type=int, default=100_000)
    parser.add_argument("--sac-load", type=Path, default=None, help="Optional existing SAC .zip to actor-warm-start.")
    parser.add_argument("--clip-actions", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--min-obs-std", type=float, default=1e-3)
    parser.add_argument(
        "--action-weights",
        type=lambda value: parse_float_list(value, 5),
        default=parse_float_list("1,1,1,1,1", 5),
        help="Comma-separated MSE weights for the 5 SAC action channels.",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.epochs <= 0:
        raise SystemExit("--epochs must be positive")
    if args.batch_size <= 0:
        raise SystemExit("--batch-size must be positive")

    run_id = args.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = args.out_dir.expanduser() / run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    dataset = load_dataset(args.logs, max_samples=int(args.max_samples), seed=int(args.seed), clip_actions=bool(args.clip_actions))
    train_indices, val_indices = split_indices(len(dataset.observations), float(args.val_fraction), int(args.seed))
    print(
        f"loaded samples={len(dataset.observations)} train={len(train_indices)} val={len(val_indices)} "
        f"obs_dim={dataset.observations.shape[1]} action_dim={dataset.actions.shape[1]}",
        flush=True,
    )

    artifacts: list[dict[str, Any]] = []
    if args.artifact in {"mlp", "both"}:
        artifacts.append(train_mlp(dataset, train_indices, val_indices, args, run_dir))
    if args.artifact in {"sac", "both"}:
        artifacts.append(train_sac_actor(dataset, train_indices, val_indices, args, run_dir))

    manifest_path = write_manifest(run_dir, dataset, train_indices, val_indices, args, artifacts)
    print(f"wrote manifest: {manifest_path}", flush=True)
    for artifact in artifacts:
        print(f"wrote {artifact['artifact']}: {artifact['path']}", flush=True)


if __name__ == "__main__":
    main()

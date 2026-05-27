from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from fsds_autonomy.geometry import path_length
from fsds_autonomy.map_store import load_track_map

try:
    import gymnasium as gym
    from gymnasium import spaces
except Exception:  # pragma: no cover - optional dependency path
    gym = None
    spaces = None


class SavedMapResidualEnv(gym.Env if gym is not None else object):
    """Small Gymnasium-style residual policy environment built from a saved map.

    The environment is intentionally lightweight. It is for pretraining speed and
    path-offset choices before trying anything inside Unreal/FSDS.
    """

    metadata = {"render_modes": []}

    def __init__(self, map_dir: str, track_id: str, max_steps: int = 1500):
        if gym is None or spaces is None:  # pragma: no cover - optional dependency path
            raise RuntimeError("Install gymnasium before using SavedMapResidualEnv")

        self.map_data = load_track_map(Path(map_dir).expanduser(), track_id)
        if self.map_data is None or not self.map_data.racing_line:
            raise ValueError(f"No saved racing line for track '{track_id}' in {map_dir}")

        self.path = np.asarray(self.map_data.racing_line, dtype=np.float32)
        self.speeds = np.asarray(self.map_data.speed_profile or [5.0] * len(self.path), dtype=np.float32)
        self.max_steps = max_steps
        self.observation_space = spaces.Box(
            low=np.full(8, -100.0, dtype=np.float32),
            high=np.full(8, 100.0, dtype=np.float32),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )
        self.reset()

    def reset(self, *, seed=None, options=None):
        if gym is not None:
            super().reset(seed=seed)
        self.index = 0
        self.step_count = 0
        self.lateral_error = 0.0
        self.speed = float(self.speeds[0])
        return self._obs(), {}

    def step(self, action):
        speed_delta, path_offset, brake = np.asarray(action, dtype=np.float32)
        target_speed = max(0.0, float(self.speeds[self.index]) + 3.0 * float(speed_delta) - 4.0 * float(brake))
        self.speed += 0.2 * (target_speed - self.speed)
        self.lateral_error = 0.85 * self.lateral_error + 0.35 * float(path_offset)
        self.index = min(len(self.path) - 1, self.index + max(1, int(self.speed * 0.18)))
        self.step_count += 1

        offtrack = abs(self.lateral_error) > 1.8
        done = self.index >= len(self.path) - 1 or self.step_count >= self.max_steps or offtrack
        reward = self.speed * 0.1 - abs(self.lateral_error) * 0.3 - float(offtrack) * 20.0
        return self._obs(), reward, done, False, {"offtrack": offtrack}

    def _obs(self):
        idx = self.index
        next_idx = min(len(self.path) - 1, idx + 3)
        dx = float(self.path[next_idx, 0] - self.path[idx, 0])
        dy = float(self.path[next_idx, 1] - self.path[idx, 1])
        heading = math.atan2(dy, dx)
        curvature_proxy = 0.0
        if idx + 2 < len(self.path):
            a = self.path[idx]
            b = self.path[idx + 1]
            c = self.path[idx + 2]
            curvature_proxy = float(np.cross(b - a, c - b))
        return np.asarray(
            [
                self.speed,
                float(self.speeds[idx]),
                self.lateral_error,
                heading,
                curvature_proxy,
                idx / max(1, len(self.path) - 1),
                len(self.map_data.cones) / 100.0,
                self.map_data.quality,
            ],
            dtype=np.float32,
        )

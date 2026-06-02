from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except Exception:  # pragma: no cover - optional dependency path
    gym = None
    spaces = None

import rclpy
from fs_msgs.msg import ControlCommand, ExtraInfo
from fs_msgs.srv import Reset
from fsds_autonomy_msgs.msg import ConeArray, RaceState, TrackMap
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String

from fsds_autonomy.constants import ConeColor
from fsds_autonomy.geometry import (
    clamp,
    closest_point_on_polyline_with_index,
    distance_xy,
    nearest_point_index,
    normalize_angle,
    path_segment_count,
    point_at_reference_path_index,
    pose2_from_pose_stamped,
    transform_global_to_local,
)


class LiveFullControlEnv(gym.Env if gym is not None else object):
    """Live Gymnasium environment for staged FSDS full-control RL.

    Action layout is fixed at five values in [-1, 1]:
    0. residual path offset
    1. residual speed delta
    2. direct steering
    3. direct throttle
    4. direct brake

    Curriculum stage controls how much of that action reaches the car. Stage 1
    and 2 publish only residual line/speed nudges. Stage 3 blends partial direct
    steering/throttle/brake. Stage 4 allows full direct control, while the
    controller and reset monitor still apply safety overrides.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        step_dt: float = 0.10,
        max_episode_steps: int = 1800,
        min_episode_steps: int = 1800,
        variable_episode_steps_enabled: bool = False,
        max_path_offset_m: float = 0.60,
        max_speed_delta_mps: float = 1.20,
        max_direct_steering: float = 1.0,
        max_direct_throttle: float = 1.0,
        max_direct_brake: float = 1.0,
        min_track_quality: float = 0.60,
        require_closed_loop: bool = True,
        reset_on_episode: bool = True,
        reset_wait_sec: float = 2.0,
        map_wait_timeout_sec: float = 3600.0,
        cone_count: int = 12,
        preview_count: int = 8,
        sector_count: int = 24,
        start_stage: int = 1,
        stage2_min_sectors: int = 6,
        stage3_min_sectors: int = 14,
        stage2_clean_episodes: int = 2,
        stage3_clean_episodes: int = 3,
        stage_bad_episode_demotion_count: int = 4,
        stage1_residual_scale: float = 0.25,
        stage2_residual_scale: float = 0.50,
        stage3_residual_scale: float = 0.80,
        stage4_clean_laps: int = 1,
        stage3_direct_blend: float = 0.35,
        stage4_direct_blend: float = 1.0,
        progress_reward: float = 4.0,
        sector_bonus: float = 8.0,
        sector_time_reference_sec: float = 25.0,
        sector_time_reference_reward: float = 0.5,
        sector_time_reward_max: float = 10.0,
        speed_reward: float = 0.04,
        speed_reward_power: float = 2.0,
        speed_reward_max_per_step: float = 0.40,
        speed_reward_min_clearance_m: float = 1.00,
        speed_reward_full_clearance_m: float = 1.60,
        time_penalty: float = 0.0,
        centerline_penalty: float = 0.04,
        racing_line_penalty: float = 0.03,
        low_clearance_penalty: float = 3.0,
        backward_penalty: float = 8.0,
        action_penalty: float = 0.02,
        action_smoothness_penalty: float = 0.08,
        throttle_brake_conflict_penalty: float = 1.5,
        cone_hit_penalty: float = 180.0,
        reset_penalty: float = 220.0,
        offtrack_penalty: float = 180.0,
        stuck_penalty: float = 60.0,
        lap_bonus: float = 0.0,
        lap_time_reference_sec: float = 600.0,
        lap_time_reference_reward: float = 10.0,
        lap_time_reward_max: float = 500.0,
        safe_clearance_m: float = 1.20,
        promotion_min_clearance_m: float = 0.90,
        stuck_speed_mps: float = 0.08,
        stuck_hold_steps: int = 50,
        reset_status_fresh_sec: float = 2.0,
        reset_status_hold_terminal_sec: float = 1.5,
        mistake_budget_enabled: bool = True,
        mistake_budget_limit: float = 35.0,
        mistake_recovery_per_step: float = 0.35,
        offtrack_grace_steps: int = 20,
        reset_bad_grace_steps: int = 15,
        low_clearance_terminal_m: float = 0.35,
        low_clearance_grace_steps: int = 30,
        wrong_direction_grace_steps: int = 25,
        no_progress_grace_steps: int = 250,
        no_progress_min_delta_m: float = 0.01,
        offtrack_step_penalty: float = 4.0,
        reset_bad_step_penalty: float = 5.0,
        low_clearance_step_penalty: float = 1.5,
        wrong_direction_step_penalty: float = 1.0,
        no_progress_step_penalty: float = 0.20,
        map_dir: str = "/home/hard/.fsds_autonomy/maps",
        danger_zone_enabled: bool = True,
        danger_zone_radius_m: float = 7.0,
        danger_zone_penalty: float = 0.35,
        random_start_enabled: bool = False,
        random_start_min_stage: int = 2,
        random_start_settle_sec: float = 0.6,
        random_start_host: str = "127.0.0.1",
        random_start_port: int = 41451,
        random_start_vehicle: str = "FSCar",
        random_start_z_offset_m: float = 0.0,
        random_start_reset_before_teleport: bool = False,
        random_start_skip_reset: bool = True,
        random_start_pause_during_teleport: bool = True,
        random_start_pre_teleport_brake_sec: float = 0.5,
        random_start_stop_speed_mps: float = 0.20,
        random_start_stop_timeout_sec: float = 2.0,
        random_start_disable_after_failures: int = 1,
        reward_log_path: str = "",
        reward_log_flush_every: int = 25,
        episode_summary_log_path: str = "",
        episode_summary_log_flush_every: int = 1,
        seed: int | None = None,
    ) -> None:
        if gym is None or spaces is None:  # pragma: no cover - optional dependency path
            raise RuntimeError("Install gymnasium before using LiveFullControlEnv")

        super().__init__()
        self.step_dt = max(0.02, float(step_dt))
        self.max_episode_steps = max(1, int(max_episode_steps))
        self.min_episode_steps = min(self.max_episode_steps, max(1, int(min_episode_steps)))
        self.variable_episode_steps_enabled = bool(variable_episode_steps_enabled)
        self.current_episode_step_limit = self.max_episode_steps
        self.max_path_offset = max(0.0, float(max_path_offset_m))
        self.max_speed_delta = max(0.0, float(max_speed_delta_mps))
        self.max_direct_steering = clamp(float(max_direct_steering), 0.0, 1.0)
        self.max_direct_throttle = clamp(float(max_direct_throttle), 0.0, 1.0)
        self.max_direct_brake = clamp(float(max_direct_brake), 0.0, 1.0)
        self.min_track_quality = float(min_track_quality)
        self.require_closed_loop = bool(require_closed_loop)
        self.reset_on_episode = bool(reset_on_episode)
        self.reset_wait_sec = max(0.0, float(reset_wait_sec))
        self.map_wait_timeout_sec = max(1.0, float(map_wait_timeout_sec))
        self.cone_count = max(0, int(cone_count))
        self.preview_count = max(0, int(preview_count))
        self.sector_count = max(1, int(sector_count))
        self.stage = clamp(float(start_stage), 0.0, 4.0)
        self.stage = int(self.stage)
        self.stage2_min_sectors = max(1, int(stage2_min_sectors))
        self.stage3_min_sectors = max(self.stage2_min_sectors, int(stage3_min_sectors))
        self.stage2_clean_episodes = max(1, int(stage2_clean_episodes))
        self.stage3_clean_episodes = max(1, int(stage3_clean_episodes))
        self.stage_bad_episode_demotion_count = max(1, int(stage_bad_episode_demotion_count))
        self.stage1_residual_scale = clamp(float(stage1_residual_scale), 0.0, 1.0)
        self.stage2_residual_scale = clamp(float(stage2_residual_scale), 0.0, 1.0)
        self.stage3_residual_scale = clamp(float(stage3_residual_scale), 0.0, 1.0)
        self.stage4_clean_laps = max(1, int(stage4_clean_laps))
        self.stage3_direct_blend = clamp(float(stage3_direct_blend), 0.0, 1.0)
        self.stage4_direct_blend = clamp(float(stage4_direct_blend), 0.0, 1.0)

        self.progress_reward = float(progress_reward)
        self.sector_bonus = float(sector_bonus)
        self.sector_time_reference_sec = max(0.1, float(sector_time_reference_sec))
        self.sector_time_reference_reward = max(0.0, float(sector_time_reference_reward))
        self.sector_time_reward_max = max(self.sector_time_reference_reward, float(sector_time_reward_max))
        self.speed_reward = float(speed_reward)
        self.speed_reward_power = max(0.5, float(speed_reward_power))
        self.speed_reward_max_per_step = max(0.0, float(speed_reward_max_per_step))
        self.speed_reward_min_clearance = max(0.0, float(speed_reward_min_clearance_m))
        self.speed_reward_full_clearance = max(self.speed_reward_min_clearance + 1e-3, float(speed_reward_full_clearance_m))
        self.time_penalty = float(time_penalty)
        self.centerline_penalty = float(centerline_penalty)
        self.racing_line_penalty = float(racing_line_penalty)
        self.low_clearance_penalty = float(low_clearance_penalty)
        self.backward_penalty = float(backward_penalty)
        self.action_penalty = float(action_penalty)
        self.action_smoothness_penalty = float(action_smoothness_penalty)
        self.throttle_brake_conflict_penalty = float(throttle_brake_conflict_penalty)
        self.cone_hit_penalty = float(cone_hit_penalty)
        self.reset_penalty = float(reset_penalty)
        self.offtrack_penalty = float(offtrack_penalty)
        self.stuck_penalty = float(stuck_penalty)
        self.lap_bonus = float(lap_bonus)
        self.lap_time_reference_sec = max(1.0, float(lap_time_reference_sec))
        self.lap_time_reference_reward = max(0.0, float(lap_time_reference_reward))
        self.lap_time_reward_max = max(self.lap_time_reference_reward, float(lap_time_reward_max))
        self.safe_clearance = float(safe_clearance_m)
        self.promotion_min_clearance = float(promotion_min_clearance_m)
        self.stuck_speed = float(stuck_speed_mps)
        self.stuck_hold_steps = max(1, int(stuck_hold_steps))
        self.reset_status_fresh_sec = max(0.1, float(reset_status_fresh_sec))
        self.reset_status_hold_terminal_sec = max(0.0, float(reset_status_hold_terminal_sec))
        self.mistake_budget_enabled = bool(mistake_budget_enabled)
        self.mistake_budget_limit = max(0.0, float(mistake_budget_limit))
        self.mistake_recovery_per_step = max(0.0, float(mistake_recovery_per_step))
        self.offtrack_grace_steps = max(1, int(offtrack_grace_steps))
        self.reset_bad_grace_steps = max(1, int(reset_bad_grace_steps))
        self.low_clearance_terminal = max(0.0, float(low_clearance_terminal_m))
        self.low_clearance_grace_steps = max(1, int(low_clearance_grace_steps))
        self.wrong_direction_grace_steps = max(1, int(wrong_direction_grace_steps))
        self.no_progress_grace_steps = max(1, int(no_progress_grace_steps))
        self.no_progress_min_delta = max(0.0, float(no_progress_min_delta_m))
        self.offtrack_step_penalty = max(0.0, float(offtrack_step_penalty))
        self.reset_bad_step_penalty = max(0.0, float(reset_bad_step_penalty))
        self.low_clearance_step_penalty = max(0.0, float(low_clearance_step_penalty))
        self.wrong_direction_step_penalty = max(0.0, float(wrong_direction_step_penalty))
        self.no_progress_step_penalty = max(0.0, float(no_progress_step_penalty))
        self.map_dir = Path(str(map_dir)).expanduser()
        self.danger_zone_enabled = bool(danger_zone_enabled)
        self.danger_zone_radius = max(0.1, float(danger_zone_radius_m))
        self.danger_zone_penalty = max(0.0, float(danger_zone_penalty))
        self.random_start_enabled = bool(random_start_enabled)
        self.random_start_min_stage = max(1, int(random_start_min_stage))
        self.random_start_settle_sec = max(0.0, float(random_start_settle_sec))
        self.random_start_host = str(random_start_host)
        self.random_start_port = int(random_start_port)
        self.random_start_vehicle = str(random_start_vehicle)
        self.random_start_z_offset = float(random_start_z_offset_m)
        self.random_start_reset_before_teleport = bool(random_start_reset_before_teleport)
        self.random_start_skip_reset = bool(random_start_skip_reset)
        self.random_start_pause_during_teleport = bool(random_start_pause_during_teleport)
        self.random_start_pre_teleport_brake_sec = max(0.0, float(random_start_pre_teleport_brake_sec))
        self.random_start_stop_speed = max(0.0, float(random_start_stop_speed_mps))
        self.random_start_stop_timeout_sec = max(0.0, float(random_start_stop_timeout_sec))
        self.random_start_disable_after_failures = max(1, int(random_start_disable_after_failures))

        self.track_shape_feature_len = 16
        self.base_obs_len = 36 + self.track_shape_feature_len
        self.cone_feature_len = 8
        self.preview_feature_len = 2
        obs_len = (
            self.base_obs_len
            + self.cone_count * self.cone_feature_len
            + 4 * self.preview_count * self.preview_feature_len
        )
        self.observation_space = spaces.Box(
            low=np.full(obs_len, -1000.0, dtype=np.float32),
            high=np.full(obs_len, 1000.0, dtype=np.float32),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=np.full(5, -1.0, dtype=np.float32),
            high=np.full(5, 1.0, dtype=np.float32),
            dtype=np.float32,
        )

        self.node = Node("fsds_live_full_control_rl_env")
        self.path_offset_pub = self.node.create_publisher(Float32, "/autonomy/rl_path_offset", 10)
        self.speed_delta_pub = self.node.create_publisher(Float32, "/autonomy/rl_speed_delta", 10)
        self.direct_control_pub = self.node.create_publisher(ControlCommand, "/autonomy/rl_control_command", 10)
        self.direct_blend_pub = self.node.create_publisher(Float32, "/autonomy/rl_direct_blend", 10)
        self.reset_client = self.node.create_client(Reset, "/fsds/reset")

        self.node.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 20)
        self.node.create_subscription(Float32, "/autonomy/speed", self.on_speed, 20)
        self.node.create_subscription(Float32, "/autonomy/target_speed", self.on_target_speed, 20)
        self.node.create_subscription(RaceState, "/autonomy/race_state", self.on_race_state, 20)
        self.node.create_subscription(TrackMap, "/autonomy/racing_line", self.on_track, 20)
        self.node.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_cones, 20)
        self.node.create_subscription(ControlCommand, "/fsds/control_command", self.on_control, 20)
        self.node.create_subscription(ExtraInfo, "/fsds/testing_only/extra_info", self.on_extra_info, 20)
        self.node.create_subscription(String, "/autonomy/offtrack_reset_status", self.on_reset_status, 20)

        self.pose: PoseStamped | None = None
        self.speed = 0.0
        self.target_speed = 0.0
        self.race_state = RaceState()
        self.track = TrackMap()
        self.local_cones = ConeArray()
        self.last_control = ControlCommand()
        self.extra_info: ExtraInfo | None = None
        self.reset_status = ""
        self.reset_status_time = 0.0
        self.reset_events_track_id = ""
        self.reset_events_mtime: float | None = None
        self.last_danger_refresh = 0.0
        self.danger_points: list[tuple[float, float, float]] = []
        self.airsim_client: Any | None = None
        self.airsim_pose_types: tuple[Any, Any, Any] | None = None
        self.random_start_disabled = False
        self.random_start_failures = 0

        self.path_points: list[Point] = []
        self.centerline_points: list[Point] = []
        self.racing_points: list[Point] = []
        self.yellow_boundary_points: list[Point] = []
        self.blue_boundary_points: list[Point] = []
        self.arc_lengths: list[float] = []
        self.center_arc_lengths: list[float] = []
        self.racing_arc_lengths: list[float] = []
        self.yellow_arc_lengths: list[float] = []
        self.blue_arc_lengths: list[float] = []
        self.path_length = 0.0
        self.path_signature: tuple[Any, ...] | None = None

        self.episode_step = 0
        self.episode_progress = 0.0
        self.episode_min_clearance = float("inf")
        self.prev_progress_s: float | None = None
        self.prev_sector: int | None = None
        self.next_sector: int | None = None
        self.last_sector_step = 0
        self.visited_sectors: set[int] = set()
        self.prev_doo_counter: int | None = None
        self.stuck_window: deque[bool] = deque(maxlen=self.stuck_hold_steps)
        self.mistake_budget = 0.0
        self.offtrack_steps = 0
        self.reset_bad_steps = 0
        self.low_clearance_steps = 0
        self.wrong_direction_steps = 0
        self.no_progress_steps = 0
        self.last_action = np.zeros(5, dtype=np.float32)
        self.prev_action = np.zeros(5, dtype=np.float32)
        self.clean_laps = 0
        self.stage2_candidate_episodes = 0
        self.stage3_candidate_episodes = 0
        self.bad_episode_count = 0
        self.total_step_count = 0
        self.episode_index = 0
        self.seed = seed
        self.reward_log_flush_every = max(1, int(reward_log_flush_every))
        self.reward_log_count = 0
        self.reward_log_file = None
        if reward_log_path:
            path = Path(reward_log_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.reward_log_file = path.open("a", encoding="utf-8")
        self.episode_summary_log_flush_every = max(1, int(episode_summary_log_flush_every))
        self.episode_summary_log_count = 0
        self.episode_summary_log_file = None
        if episode_summary_log_path:
            path = Path(episode_summary_log_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.episode_summary_log_file = path.open("a", encoding="utf-8")
        self.reset_episode_metrics()
        self.np_random = np.random.default_rng(seed)

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def on_speed(self, msg: Float32) -> None:
        self.speed = float(msg.data)

    def on_target_speed(self, msg: Float32) -> None:
        self.target_speed = float(msg.data)

    def on_race_state(self, msg: RaceState) -> None:
        self.race_state = msg
        self.refresh_danger_points(str(msg.track_id or ""))

    def on_track(self, msg: TrackMap) -> None:
        self.track = msg
        signature = self.track_signature(msg)
        if signature != self.path_signature:
            self.path_signature = signature
            self.rebuild_path_cache()

    @staticmethod
    def path_coordinate_signature(points: list[Point]) -> tuple[tuple[float, float], ...]:
        if not points:
            return ()
        step = max(1, len(points) // 8)
        samples = list(points[::step][:8])
        if samples[-1] is not points[-1]:
            samples.append(points[-1])
        return tuple((round(float(point.x), 2), round(float(point.y), 2)) for point in samples)

    def track_signature(self, msg: TrackMap) -> tuple[Any, ...]:
        return (
            len(msg.centerline),
            len(msg.racing_line),
            len(msg.blue_boundary_line),
            len(msg.yellow_boundary_line),
            bool(msg.closed_loop),
            str(msg.track_id),
            round(float(msg.quality), 3),
            self.path_coordinate_signature(list(msg.centerline)),
            self.path_coordinate_signature(list(msg.racing_line)),
            self.path_coordinate_signature(list(msg.blue_boundary_line)),
            self.path_coordinate_signature(list(msg.yellow_boundary_line)),
        )

    def reset_events_path(self, track_id: str) -> Path:
        safe_track = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in track_id.strip()) or "unknown"
        return self.map_dir / f"{safe_track}.reset_events.json"

    def refresh_danger_points(self, track_id: str, *, force: bool = False) -> None:
        if not self.danger_zone_enabled:
            return
        track_id = track_id.strip() or str(self.track.track_id or "").strip()
        if not track_id:
            return
        now = time.monotonic()
        if not force and now - self.last_danger_refresh < 1.0:
            return
        self.last_danger_refresh = now
        path = self.reset_events_path(track_id)
        try:
            mtime = path.stat().st_mtime
        except OSError:
            if track_id != self.reset_events_track_id or self.reset_events_mtime is not None:
                self.reset_events_track_id = track_id
                self.reset_events_mtime = None
                self.danger_points = []
            return
        if track_id == self.reset_events_track_id and self.reset_events_mtime == mtime:
            return
        self.load_danger_points(track_id, path, mtime)

    def load_danger_points(self, track_id: str, path: Path | None = None, mtime: float | None = None) -> None:
        self.reset_events_track_id = track_id
        self.reset_events_mtime = mtime
        self.danger_points = []
        path = path or self.reset_events_path(track_id)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.node.get_logger().warn(f"Could not load RL danger zones from {path}: {exc}")
            return
        if not isinstance(data, list):
            return

        points: list[tuple[float, float, float]] = []
        danger_reasons = (
            "cone_hit",
            "off_track",
            "extreme_off_track",
            "forbidden_area",
            "outside_closed_corridor",
            "stuck",
        )
        for item in data:
            if not isinstance(item, dict):
                continue
            reason = str(item.get("reason") or "").lower()
            if not any(reason.startswith(prefix) or prefix in reason for prefix in danger_reasons):
                continue
            try:
                x = float(item.get("x"))
                y = float(item.get("y"))
            except (TypeError, ValueError):
                continue
            if not math.isfinite(x) or not math.isfinite(y):
                continue
            # Repeated reset-at-spawn/stuck markers are useful for debugging,
            # but they make poor avoidance targets for driving around the track.
            if math.hypot(x, y) < 1.0:
                continue
            weight = 1.0
            if "stuck" in reason:
                weight = 0.50
            elif "forbidden_area" in reason or "off_track" in reason:
                weight = 0.80
            points.append((x, y, weight))

        self.danger_points = points[-160:]
        if self.danger_points:
            self.node.get_logger().info(f"Loaded {len(self.danger_points)} RL danger zones from {path}")

    def on_cones(self, msg: ConeArray) -> None:
        self.local_cones = msg

    def on_control(self, msg: ControlCommand) -> None:
        self.last_control = msg

    def on_extra_info(self, msg: ExtraInfo) -> None:
        self.extra_info = msg

    def on_reset_status(self, msg: String) -> None:
        self.reset_status = str(msg.data)
        self.reset_status_time = time.monotonic()

    @staticmethod
    def arc_lengths_for(path: list[Point]) -> list[float]:
        if not path:
            return []
        lengths = [0.0]
        for index in range(1, len(path)):
            lengths.append(lengths[-1] + distance_xy(path[index - 1], path[index]))
        return lengths

    def rebuild_path_cache(self) -> None:
        self.centerline_points = list(self.track.centerline)
        self.racing_points = list(self.track.racing_line)
        self.yellow_boundary_points = list(self.track.yellow_boundary_line)
        self.blue_boundary_points = list(self.track.blue_boundary_line)
        self.path_points = self.centerline_points or self.racing_points
        self.center_arc_lengths = self.arc_lengths_for(self.centerline_points)
        self.racing_arc_lengths = self.arc_lengths_for(self.racing_points)
        self.yellow_arc_lengths = self.arc_lengths_for(self.yellow_boundary_points)
        self.blue_arc_lengths = self.arc_lengths_for(self.blue_boundary_points)
        self.arc_lengths = self.arc_lengths_for(self.path_points)
        self.path_length = self.arc_lengths[-1] if self.arc_lengths else 0.0

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.np_random = np.random.default_rng(seed)
        self.publish_neutral()
        did_reset = False
        random_start_active = self.random_start_active()
        skip_episode_reset = random_start_active and self.random_start_skip_reset
        if self.reset_on_episode and self.ready_for_learning() and not skip_episode_reset:
            self.call_reset()
            self.spin_for(self.reset_wait_sec)
            did_reset = True
        self.wait_until_ready()
        random_start_active = self.random_start_active()
        if random_start_active:
            if (
                self.random_start_reset_before_teleport
                and not self.random_start_skip_reset
                and not did_reset
                and self.ready_for_learning()
            ):
                self.call_reset()
                self.spin_for(self.reset_wait_sec)
                self.wait_until_ready()
                did_reset = True
            self.brake_before_random_start()
            random_started = self.teleport_to_random_start()
            if random_started:
                self.reset_status = ""
                self.reset_status_time = 0.0
                self.spin_for(self.random_start_settle_sec)
            elif self.random_start_skip_reset and self.reset_on_episode and not did_reset and self.ready_for_learning():
                self.call_reset()
                self.spin_for(self.reset_wait_sec)
                self.wait_until_ready()
        if self.stage <= 0:
            self.stage = 1
        self.current_episode_step_limit = self.sample_episode_step_limit()
        self.episode_step = 0
        self.episode_progress = 0.0
        self.episode_min_clearance = float("inf")
        self.prev_progress_s = self.progress_s()
        self.prev_sector = self.current_sector()
        self.next_sector = ((self.prev_sector + 1) % self.sector_count) if self.prev_sector is not None else None
        self.last_sector_step = 0
        self.visited_sectors = {self.prev_sector} if self.prev_sector is not None else set()
        self.prev_doo_counter = int(self.extra_info.doo_counter) if self.extra_info is not None else None
        self.stuck_window.clear()
        self.mistake_budget = 0.0
        self.offtrack_steps = 0
        self.reset_bad_steps = 0
        self.low_clearance_steps = 0
        self.wrong_direction_steps = 0
        self.no_progress_steps = 0
        self.last_action[:] = 0.0
        self.prev_action[:] = 0.0
        self.episode_index += 1
        self.reset_episode_metrics()
        return self.obs(), {}

    def sample_episode_step_limit(self) -> int:
        if not self.variable_episode_steps_enabled or self.min_episode_steps >= self.max_episode_steps:
            return self.max_episode_steps
        return int(self.np_random.integers(self.min_episode_steps, self.max_episode_steps + 1))

    def reset_episode_metrics(self) -> None:
        self.episode_metrics: dict[str, Any] = {
            "start_wall_time": time.time(),
            "start_total_step": int(self.total_step_count),
            "start_stage": int(self.stage),
            "start_direct_blend": float(self.stage_direct_blend()),
            "start_sector": self.prev_sector,
            "start_progress_s": float(self.prev_progress_s) if self.prev_progress_s is not None else None,
            "start_pose": self.pose_snapshot(),
            "start_doo_counter": int(self.extra_info.doo_counter) if self.extra_info is not None else None,
            "reward_total": 0.0,
            "reward_parts": {},
            "positive_progress_m": 0.0,
            "backward_progress_m": 0.0,
            "speed_sum": 0.0,
            "speed_count": 0,
            "speed_max": 0.0,
            "target_speed_sum": 0.0,
            "target_speed_count": 0,
            "clearance_sum": 0.0,
            "clearance_count": 0,
            "clearance_min": float("inf"),
            "center_error_sum": 0.0,
            "center_error_count": 0,
            "center_error_max": 0.0,
            "racing_error_sum": 0.0,
            "racing_error_count": 0,
            "racing_error_max": 0.0,
            "control_count": 0,
            "abs_steering_sum": 0.0,
            "throttle_sum": 0.0,
            "brake_sum": 0.0,
            "full_steer_steps": 0,
            "throttle_steps": 0,
            "brake_steps": 0,
            "raw_action_abs_sum": [0.0] * 5,
            "scaled_path_offset_abs_sum": 0.0,
            "scaled_speed_delta_abs_sum": 0.0,
            "scaled_steering_abs_sum": 0.0,
            "scaled_throttle_sum": 0.0,
            "scaled_brake_sum": 0.0,
            "direct_blend_sum": 0.0,
            "danger_zone_sum": 0.0,
            "danger_zone_max": 0.0,
            "safe_clearance_violation_steps": 0,
        }

    def update_episode_metrics(
        self,
        *,
        action: np.ndarray,
        scaled_action: dict[str, float],
        reward: float,
        reward_parts: dict[str, float],
        positive_progress: float,
        backward_progress: float,
        center_error: float,
        racing_error: float,
        clearance: dict[str, float] | None,
        min_clearance: float,
        danger_zone_strength: float,
    ) -> None:
        metrics = self.episode_metrics
        metrics["reward_total"] += float(reward)
        parts = metrics["reward_parts"]
        for key, value in reward_parts.items():
            parts[key] = float(parts.get(key, 0.0)) + float(value)
        metrics["positive_progress_m"] += float(max(0.0, positive_progress))
        metrics["backward_progress_m"] += float(max(0.0, backward_progress))

        speed = float(self.speed)
        target_speed = float(self.target_speed)
        metrics["speed_sum"] += speed
        metrics["speed_count"] += 1
        metrics["speed_max"] = max(float(metrics["speed_max"]), speed)
        metrics["target_speed_sum"] += target_speed
        metrics["target_speed_count"] += 1

        if clearance is not None and math.isfinite(float(min_clearance)):
            metrics["clearance_sum"] += float(min_clearance)
            metrics["clearance_count"] += 1
            metrics["clearance_min"] = min(float(metrics["clearance_min"]), float(min_clearance))
            if float(min_clearance) < self.safe_clearance:
                metrics["safe_clearance_violation_steps"] += 1

        center_error = abs(float(center_error))
        racing_error = abs(float(racing_error))
        metrics["center_error_sum"] += center_error
        metrics["center_error_count"] += 1
        metrics["center_error_max"] = max(float(metrics["center_error_max"]), center_error)
        metrics["racing_error_sum"] += racing_error
        metrics["racing_error_count"] += 1
        metrics["racing_error_max"] = max(float(metrics["racing_error_max"]), racing_error)

        control = self.last_control
        steering = abs(float(control.steering))
        throttle = float(control.throttle)
        brake = float(control.brake)
        metrics["control_count"] += 1
        metrics["abs_steering_sum"] += steering
        metrics["throttle_sum"] += throttle
        metrics["brake_sum"] += brake
        if steering >= 0.98:
            metrics["full_steer_steps"] += 1
        if throttle > 0.05:
            metrics["throttle_steps"] += 1
        if brake > 0.05:
            metrics["brake_steps"] += 1

        abs_action = np.abs(np.asarray(action, dtype=np.float32))
        for index in range(min(5, len(abs_action))):
            metrics["raw_action_abs_sum"][index] += float(abs_action[index])
        metrics["scaled_path_offset_abs_sum"] += abs(float(scaled_action.get("path_offset", 0.0)))
        metrics["scaled_speed_delta_abs_sum"] += abs(float(scaled_action.get("speed_delta", 0.0)))
        metrics["scaled_steering_abs_sum"] += abs(float(scaled_action.get("steering", 0.0)))
        metrics["scaled_throttle_sum"] += float(scaled_action.get("throttle", 0.0))
        metrics["scaled_brake_sum"] += float(scaled_action.get("brake", 0.0))
        metrics["direct_blend_sum"] += float(scaled_action.get("blend", 0.0))
        metrics["danger_zone_sum"] += float(danger_zone_strength)
        metrics["danger_zone_max"] = max(float(metrics["danger_zone_max"]), float(danger_zone_strength))

    def episode_average(self, name: str) -> float:
        return self.metric_average(f"{name}_sum", f"{name}_count")

    def metric_average(self, sum_key: str, count_key: str) -> float:
        count = int(self.episode_metrics.get(count_key, 0))
        if count <= 0:
            return 0.0
        return float(self.episode_metrics.get(sum_key, 0.0)) / float(count)

    def episode_metric_max(self, name: str) -> float:
        value = self.episode_metrics.get(f"{name}_max", 0.0)
        try:
            value = float(value)
        except (TypeError, ValueError):
            return 0.0
        return value if math.isfinite(value) else 0.0

    def pose_snapshot(self) -> dict[str, float | None]:
        pose_msg = self.pose.pose if self.pose is not None else None
        if pose_msg is None:
            return {"x": None, "y": None, "z": None, "yaw_rad": None}
        q = pose_msg.orientation
        yaw = math.atan2(
            2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
            1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z)),
        )
        return {
            "x": float(pose_msg.position.x),
            "y": float(pose_msg.position.y),
            "z": float(pose_msg.position.z),
            "yaw_rad": float(yaw),
        }

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        scaled_action = self.scaled_action(action)
        self.publish_scaled_action(**scaled_action)
        self.spin_for(self.step_dt)
        self.refresh_danger_points(str(self.race_state.track_id or self.track.track_id or ""))
        self.episode_step += 1
        self.total_step_count += 1
        self.prev_action = self.last_action.copy()
        self.last_action = action

        progress = self.progress_delta()
        positive_progress = max(0.0, progress)
        backward_progress = max(0.0, -progress)
        self.episode_progress += positive_progress
        new_sector_count, wrong_direction, sector_reward, crossed_sectors = self.update_sector_progress(progress)
        center_error = self.path_error(self.centerline_points)
        racing_error = self.path_error(self.racing_points)
        clearance = self.boundary_clearance()
        min_clearance = clearance["min_clearance_m"] if clearance else 0.0
        line_offsets = self.local_line_offsets()
        self.episode_min_clearance = min(self.episode_min_clearance, min_clearance)
        low_clearance = max(0.0, self.safe_clearance - min_clearance)
        inside_track = clearance is not None and min_clearance > 0.0

        reward = 0.0
        reward_parts: dict[str, float] = {}

        def add_reward(name: str, value: float) -> None:
            nonlocal reward
            value = float(value)
            reward_parts[name] = reward_parts.get(name, 0.0) + value
            reward += value

        add_reward("progress", self.progress_reward * positive_progress)
        add_reward("sector", sector_reward)
        add_reward("speed_shaping", self.safe_speed_reward(inside_track, positive_progress, min_clearance))
        add_reward("time", -self.time_penalty * self.step_dt)
        add_reward("centerline_error", -self.centerline_penalty * center_error)
        add_reward("racing_line_error", -self.racing_line_penalty * racing_error)
        add_reward("low_clearance", -self.low_clearance_penalty * low_clearance * low_clearance)
        add_reward("danger_zone", -self.danger_zone_reward_penalty())
        add_reward("backward", -self.backward_penalty * backward_progress)
        effective_action = self.effective_action_for_penalty(action)
        prev_effective_action = self.effective_action_for_penalty(self.prev_action)
        add_reward("action_size", -self.action_penalty * float(np.abs(effective_action).sum()))
        add_reward(
            "action_smoothness",
            -self.action_smoothness_penalty * float(np.abs(effective_action - prev_effective_action).sum()),
        )
        add_reward(
            "throttle_brake_conflict",
            -self.throttle_brake_conflict_penalty
            * float(effective_action[3])
            * float(effective_action[4]),
        )
        if wrong_direction:
            add_reward("wrong_direction", -self.backward_penalty)
        else:
            reward_parts["wrong_direction"] = 0.0

        terminal_reason = ""
        cone_hit = self.cone_hit()
        reset_bad = self.reset_status_bad()
        stuck = self.stuck()
        offtrack = clearance is not None and min_clearance < -0.10
        low_clearance_critical = clearance is not None and 0.0 <= min_clearance < self.low_clearance_terminal
        no_progress_stall = self.update_no_progress_stall(positive_progress)
        hard_reset_status = reset_bad and self.reset_status.lower().startswith(("cone_hit ", "stuck "))
        mistake_increment = self.update_mistake_budget(
            offtrack=offtrack,
            reset_bad=reset_bad,
            low_clearance_critical=low_clearance_critical,
            wrong_direction=wrong_direction,
        )
        budget_exhausted = (
            self.mistake_budget_enabled
            and self.mistake_budget_limit > 0.0
            and mistake_increment > 0.0
            and self.mistake_budget >= self.mistake_budget_limit
        )
        lap_done = (
            self.path_length > 1.0
            and self.episode_progress >= 0.95 * self.path_length
            and len(self.visited_sectors) >= max(1, int(0.80 * self.sector_count))
        )

        for terminal_key in (
            "cone_hit",
            "reset",
            "offtrack",
            "stuck",
            "critical_clearance",
            "wrong_direction_terminal",
            "no_progress_stall",
            "mistake_budget_terminal",
            "lap_completion",
        ):
            reward_parts[terminal_key] = 0.0
        if reset_bad and not hard_reset_status:
            add_reward("reset_warning", -self.reset_bad_step_penalty)
        else:
            reward_parts["reset_warning"] = 0.0
        if offtrack:
            add_reward("offtrack_warning", -self.offtrack_step_penalty)
        else:
            reward_parts["offtrack_warning"] = 0.0
        if low_clearance_critical:
            clearance_scale = max(0.0, self.low_clearance_terminal - min_clearance) / max(0.10, self.low_clearance_terminal)
            add_reward("critical_clearance", -self.low_clearance_step_penalty * (1.0 + clearance_scale))
        if wrong_direction:
            add_reward("wrong_direction_warning", -self.wrong_direction_step_penalty)
        else:
            reward_parts["wrong_direction_warning"] = 0.0
        if self.no_progress_steps > 0:
            add_reward("no_progress_warning", -self.no_progress_step_penalty)
        else:
            reward_parts["no_progress_warning"] = 0.0

        if cone_hit:
            add_reward("cone_hit", -self.cone_hit_penalty)
            terminal_reason = "cone_hit"
        elif hard_reset_status:
            add_reward("reset", -self.reset_penalty)
            terminal_reason = self.reset_status or "reset"
        elif not self.mistake_budget_enabled and reset_bad:
            add_reward("reset", -self.reset_penalty)
            terminal_reason = self.reset_status or "reset"
        elif not self.mistake_budget_enabled and offtrack:
            add_reward("offtrack", -self.offtrack_penalty)
            terminal_reason = "outside_closed_corridor"
        elif stuck:
            add_reward("stuck", -self.stuck_penalty)
            terminal_reason = "stuck"
        elif self.mistake_budget_enabled and reset_bad and self.reset_bad_steps >= self.reset_bad_grace_steps:
            add_reward("reset", -self.reset_penalty)
            terminal_reason = self.reset_status or "reset_sustained"
        elif self.mistake_budget_enabled and offtrack and self.offtrack_steps >= self.offtrack_grace_steps:
            add_reward("offtrack", -self.offtrack_penalty)
            terminal_reason = "outside_closed_corridor_sustained"
        elif self.mistake_budget_enabled and low_clearance_critical and self.low_clearance_steps >= self.low_clearance_grace_steps:
            add_reward("critical_clearance", -0.5 * self.offtrack_penalty)
            terminal_reason = "low_clearance_sustained"
        elif self.mistake_budget_enabled and wrong_direction and self.wrong_direction_steps >= self.wrong_direction_grace_steps:
            add_reward("wrong_direction_terminal", -self.reset_penalty)
            terminal_reason = "wrong_direction_sustained"
        elif no_progress_stall:
            add_reward("no_progress_stall", -self.stuck_penalty)
            terminal_reason = "no_progress_stall"
        elif budget_exhausted:
            add_reward("mistake_budget_terminal", -self.reset_penalty)
            terminal_reason = "mistake_budget_exhausted"
        elif lap_done:
            add_reward("lap_completion", self.lap_completion_reward())
            terminal_reason = "lap_complete"

        terminated = bool(terminal_reason)
        truncated = self.episode_step >= self.current_episode_step_limit
        danger_zone_strength = float(self.danger_zone_features()[2])
        self.update_episode_metrics(
            action=action,
            scaled_action=scaled_action,
            reward=float(reward),
            reward_parts=reward_parts,
            positive_progress=positive_progress,
            backward_progress=backward_progress,
            center_error=center_error,
            racing_error=racing_error,
            clearance=clearance,
            min_clearance=min_clearance,
            danger_zone_strength=danger_zone_strength,
        )
        info: dict[str, Any] = {
            "danger_zone_strength": danger_zone_strength,
            "episode_step_limit": self.current_episode_step_limit,
            "episode_step_fraction": self.episode_step / max(1.0, float(self.current_episode_step_limit)),
            "progress_m": progress,
            "episode_progress_m": self.episode_progress,
            "episode_progress_fraction": self.episode_progress / self.path_length if self.path_length > 1.0 else 0.0,
            "path_length_m": self.path_length,
            "speed_mps": self.speed,
            "episode_avg_speed_mps": self.episode_average("speed"),
            "episode_max_speed_mps": self.episode_metric_max("speed"),
            "centerline_error_m": center_error,
            "racing_line_error_m": racing_error,
            "min_clearance_m": min_clearance,
            "episode_min_clearance_m": self.episode_min_clearance,
            "episode_avg_clearance_m": self.episode_average("clearance"),
            "sector_count": len(self.visited_sectors),
            "stage": self.stage,
            "direct_blend": self.stage_direct_blend(),
            "cone_hit": cone_hit,
            "reset_bad": reset_bad,
            "offtrack": offtrack,
            "low_clearance_critical": low_clearance_critical,
            "mistake_budget": self.mistake_budget,
            "mistake_budget_limit": self.mistake_budget_limit,
            "mistake_increment": mistake_increment,
            "offtrack_steps": self.offtrack_steps,
            "reset_bad_steps": self.reset_bad_steps,
            "low_clearance_steps": self.low_clearance_steps,
            "wrong_direction_steps": self.wrong_direction_steps,
            "no_progress_steps": self.no_progress_steps,
            "terminal_reason": terminal_reason,
            "reset_status": self.reset_status,
            "reward_total": float(reward),
            "episode_reward_total": self.episode_metrics["reward_total"],
            "reward_lap_completion": reward_parts["lap_completion"],
            "reward_sector": reward_parts["sector"],
            "reward_speed": reward_parts["speed_shaping"],
            "reward_danger_zone": reward_parts["danger_zone"],
            "reward_low_clearance": reward_parts["low_clearance"],
            "reward_critical_clearance": reward_parts["critical_clearance"],
            "reward_no_progress": reward_parts["no_progress_warning"] + reward_parts["no_progress_stall"],
            "centerline_x_m": line_offsets["centerline_x_m"],
            "centerline_y_m": line_offsets["centerline_y_m"],
            "racing_line_x_m": line_offsets["racing_line_x_m"],
            "racing_line_y_m": line_offsets["racing_line_y_m"],
            "yellow_line_x_m": line_offsets["yellow_line_x_m"],
            "yellow_line_y_m": line_offsets["yellow_line_y_m"],
            "blue_line_x_m": line_offsets["blue_line_x_m"],
            "blue_line_y_m": line_offsets["blue_line_y_m"],
        }
        self.write_reward_log(
            action=action,
            scaled_action=scaled_action,
            reward=float(reward),
            reward_parts=reward_parts,
            progress=progress,
            center_error=center_error,
            racing_error=racing_error,
            clearance=clearance,
            terminal_reason=terminal_reason,
            truncated=truncated,
            crossed_sectors=crossed_sectors,
        )
        if terminated or truncated:
            stage_before_update = int(self.stage)
            self.finish_episode(terminal_reason, truncated)
            stage_after_update = int(self.stage)
            info["stage_after_episode"] = stage_after_update
            info["stage_changed"] = float(stage_after_update != stage_before_update)
            info["terminal_reason"] = terminal_reason or ("truncated" if truncated else "")
            self.write_episode_summary(
                terminal_reason=terminal_reason,
                truncated=truncated,
                stage_before_update=stage_before_update,
                stage_after_update=stage_after_update,
            )
            self.publish_neutral()
        return self.obs(), float(reward), terminated, truncated, info

    def finish_episode(self, terminal_reason: str, truncated: bool) -> None:
        bad = terminal_reason not in ("", "lap_complete") and not truncated
        if bad:
            self.bad_episode_count += 1
            self.reset_promotion_counters()
            if self.stage > 1 and self.bad_episode_count >= self.stage_bad_episode_demotion_count:
                self.stage -= 1
                self.bad_episode_count = 0
                self.reset_promotion_counters()
                self.node.get_logger().warn(f"RL curriculum demoted to stage {self.stage} after repeated bad episodes")
            return

        clean_enough = not bad and self.episode_min_clearance >= self.promotion_min_clearance
        if not clean_enough:
            self.reset_promotion_counters()
            return
        self.bad_episode_count = 0
        sectors = len(self.visited_sectors)

        if self.stage <= 1:
            if sectors < self.stage2_min_sectors:
                return
            self.stage2_candidate_episodes += 1
            if self.stage2_candidate_episodes >= self.stage2_clean_episodes:
                self.promote_to_stage(2)
            return

        if self.stage == 2:
            if sectors < self.stage3_min_sectors:
                return
            self.stage3_candidate_episodes += 1
            if self.stage3_candidate_episodes >= self.stage3_clean_episodes:
                self.promote_to_stage(3)
            return

        if self.stage == 3 and terminal_reason == "lap_complete":
            self.clean_laps += 1
            if self.clean_laps >= self.stage4_clean_laps:
                self.promote_to_stage(4)
            return

        if self.stage >= 4 and terminal_reason == "lap_complete":
            self.clean_laps += 1

    def reset_promotion_counters(self) -> None:
        self.stage2_candidate_episodes = 0
        self.stage3_candidate_episodes = 0
        self.clean_laps = 0

    def promote_to_stage(self, new_stage: int) -> None:
        new_stage = max(1, min(4, int(new_stage)))
        if new_stage <= self.stage:
            return
        self.stage = new_stage
        self.reset_promotion_counters()
        self.node.get_logger().info(f"RL curriculum promoted to stage {self.stage}")

    def publish_neutral(self) -> None:
        self.publish_scaled_action(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def publish_hard_brake(self) -> None:
        self.publish_scaled_action(0.0, 0.0, 0.0, 0.0, 1.0, 0.0)

    def publish_action(self, action: np.ndarray) -> None:
        self.publish_scaled_action(**self.scaled_action(action))

    def scaled_action(self, action: np.ndarray) -> dict[str, float]:
        residual_scale = self.stage_residual_scale()
        path_offset = float(action[0]) * self.max_path_offset * residual_scale
        speed_delta = float(action[1]) * self.max_speed_delta * residual_scale
        blend = self.stage_direct_blend()
        steering = float(action[2]) * self.max_direct_steering if blend > 0.0 else 0.0
        throttle = self.direct_throttle_from_action(action) if blend > 0.0 else 0.0
        brake = self.direct_brake_from_action(action) if blend > 0.0 else 0.0
        return {
            "path_offset": path_offset,
            "speed_delta": speed_delta,
            "steering": steering,
            "throttle": throttle,
            "brake": brake,
            "blend": blend,
        }

    def lap_completion_reward(self) -> float:
        lap_time = max(self.step_dt, self.episode_step * self.step_dt)
        timed_reward = self.lap_time_reference_reward * self.lap_time_reference_sec / lap_time
        return self.lap_bonus + min(self.lap_time_reward_max, timed_reward)

    def sector_gate_reward(self) -> float:
        elapsed_steps = max(1, self.episode_step - self.last_sector_step)
        elapsed_sec = max(self.step_dt, elapsed_steps * self.step_dt)
        timed_reward = self.sector_time_reference_reward * self.sector_time_reference_sec / elapsed_sec
        return self.sector_bonus + min(self.sector_time_reward_max, timed_reward)

    def update_no_progress_stall(self, positive_progress: float) -> bool:
        if positive_progress >= self.no_progress_min_delta:
            self.no_progress_steps = 0
            return False
        if self.episode_step < max(10, int(2.0 / self.step_dt)):
            self.no_progress_steps = 0
            return False
        expected_to_move = self.target_speed > 0.5 or abs(self.speed) > 0.5 or self.stage_direct_blend() > 0.0
        if expected_to_move:
            self.no_progress_steps += 1
        else:
            self.no_progress_steps = 0
        return self.no_progress_steps >= self.no_progress_grace_steps

    def update_mistake_budget(
        self,
        *,
        offtrack: bool,
        reset_bad: bool,
        low_clearance_critical: bool,
        wrong_direction: bool,
    ) -> float:
        self.offtrack_steps = self.offtrack_steps + 1 if offtrack else 0
        self.reset_bad_steps = self.reset_bad_steps + 1 if reset_bad else 0
        self.low_clearance_steps = self.low_clearance_steps + 1 if low_clearance_critical else 0
        self.wrong_direction_steps = self.wrong_direction_steps + 1 if wrong_direction else 0

        if not self.mistake_budget_enabled:
            self.mistake_budget = 0.0
            return 0.0

        increment = 0.0
        if offtrack:
            increment += 3.0
        if reset_bad:
            increment += 3.0
        if low_clearance_critical:
            increment += 1.0
        if wrong_direction:
            increment += 1.5

        if increment > 0.0:
            self.mistake_budget += increment
        else:
            self.mistake_budget = max(0.0, self.mistake_budget - self.mistake_recovery_per_step)
        return increment

    def safe_speed_reward(self, inside_track: bool, positive_progress: float, min_clearance: float) -> float:
        if not inside_track or positive_progress <= 0.0 or self.speed <= 0.0 or self.speed_reward <= 0.0:
            return 0.0
        if min_clearance <= self.speed_reward_min_clearance:
            return 0.0
        clearance_span = max(1e-6, self.speed_reward_full_clearance - self.speed_reward_min_clearance)
        clearance_alpha = clamp((min_clearance - self.speed_reward_min_clearance) / clearance_span, 0.0, 1.0)
        # Smoothstep keeps speed reward soft near the boundary and full only
        # when the car has meaningful room on both sides.
        clearance_gate = clearance_alpha * clearance_alpha * (3.0 - 2.0 * clearance_alpha)
        value = self.speed_reward * (max(0.0, self.speed) ** self.speed_reward_power) * self.step_dt * clearance_gate
        return min(self.speed_reward_max_per_step, value)

    def danger_zone_reward_penalty(self) -> float:
        if self.danger_zone_penalty <= 0.0:
            return 0.0
        strength = float(self.danger_zone_features()[2])
        if strength <= 0.0:
            return 0.0
        speed_gate = clamp(max(0.0, self.speed) / 2.0, 0.0, 1.0)
        return self.danger_zone_penalty * strength * strength * (0.5 + 0.5 * speed_gate)

    def publish_scaled_action(
        self,
        path_offset: float,
        speed_delta: float,
        steering: float,
        throttle: float,
        brake: float,
        blend: float,
    ) -> None:
        offset_msg = Float32()
        offset_msg.data = float(np.clip(path_offset, -self.max_path_offset, self.max_path_offset))
        speed_msg = Float32()
        speed_msg.data = float(np.clip(speed_delta, -self.max_speed_delta, self.max_speed_delta))
        direct_msg = ControlCommand()
        direct_msg.header.stamp = self.node.get_clock().now().to_msg()
        direct_msg.header.frame_id = "fsds/FSCar"
        direct_msg.steering = float(np.clip(steering, -self.max_direct_steering, self.max_direct_steering))
        direct_msg.throttle = float(np.clip(throttle, 0.0, self.max_direct_throttle))
        direct_msg.brake = float(np.clip(brake, 0.0, self.max_direct_brake))
        blend_msg = Float32()
        blend_msg.data = float(np.clip(blend, 0.0, 1.0))
        self.path_offset_pub.publish(offset_msg)
        self.speed_delta_pub.publish(speed_msg)
        self.direct_control_pub.publish(direct_msg)
        self.direct_blend_pub.publish(blend_msg)
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def stage_residual_scale(self) -> float:
        if self.stage <= 0:
            return 0.0
        if self.stage == 1:
            return self.stage1_residual_scale
        if self.stage == 2:
            return self.stage2_residual_scale
        if self.stage == 3:
            return self.stage3_residual_scale
        return 1.0

    def stage_direct_blend(self) -> float:
        if self.stage < 3:
            return 0.0
        if self.stage == 3:
            return self.stage3_direct_blend
        return self.stage4_direct_blend

    def direct_throttle_from_action(self, action: np.ndarray) -> float:
        return clamp(0.5 * (float(action[3]) + 1.0) * self.max_direct_throttle, 0.0, self.max_direct_throttle)

    def direct_brake_from_action(self, action: np.ndarray) -> float:
        return clamp(0.5 * (float(action[4]) + 1.0) * self.max_direct_brake, 0.0, self.max_direct_brake)

    def effective_action_for_penalty(self, action: np.ndarray) -> np.ndarray:
        residual_scale = self.stage_residual_scale()
        direct_blend = self.stage_direct_blend()
        if direct_blend <= 0.0:
            steering = 0.0
            throttle = 0.0
            brake = 0.0
        else:
            steering = float(action[2]) * direct_blend
            throttle = self.direct_throttle_from_action(action) * direct_blend
            brake = self.direct_brake_from_action(action) * direct_blend
        return np.asarray(
            [
                float(action[0]) * residual_scale,
                float(action[1]) * residual_scale,
                steering,
                throttle,
                brake,
            ],
            dtype=np.float32,
        )

    def wait_until_ready(self) -> None:
        start = time.monotonic()
        last_log = 0.0
        while rclpy.ok() and time.monotonic() - start < self.map_wait_timeout_sec:
            self.spin_for(0.1)
            if self.ready_for_learning():
                return
            now = time.monotonic()
            if now - last_log > 5.0:
                self.node.get_logger().warn(
                    "RL waiting for pose + closed-loop map: "
                    f"pose={self.pose is not None} go={bool(self.race_state.go_signal_fresh)} "
                    f"closed={bool(self.track.closed_loop)} quality={float(self.track.quality):.2f} "
                    f"center={len(self.track.centerline)} racing={len(self.track.racing_line)} "
                    f"blue={len(self.track.blue_boundary_line)} yellow={len(self.track.yellow_boundary_line)}",
                )
                last_log = now
        raise TimeoutError("Timed out waiting for live RL map/pose readiness")

    def ready_for_learning(self) -> bool:
        if self.pose is None or not self.race_state.go_signal_fresh:
            return False
        if self.require_closed_loop and not bool(self.track.closed_loop):
            return False
        if float(self.track.quality) < self.min_track_quality:
            return False
        if len(self.path_points) < 8:
            return False
        if self.require_closed_loop and (len(self.track.blue_boundary_line) < 8 or len(self.track.yellow_boundary_line) < 8):
            return False
        return True

    def call_reset(self) -> None:
        if not self.reset_client.service_is_ready():
            self.reset_client.wait_for_service(timeout_sec=1.0)
        if not self.reset_client.service_is_ready():
            self.node.get_logger().warn("/fsds/reset unavailable; continuing without episode reset")
            return
        request = Reset.Request()
        request.wait_on_last_task = False
        future = self.reset_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def random_start_active(self) -> bool:
        return (
            self.random_start_enabled
            and not self.random_start_disabled
            and int(self.stage) >= self.random_start_min_stage
        )

    def brake_before_random_start(self) -> None:
        self.publish_hard_brake()
        self.spin_for(self.random_start_pre_teleport_brake_sec)
        if self.random_start_stop_timeout_sec <= 0.0:
            return
        deadline = time.monotonic() + self.random_start_stop_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if abs(float(self.speed)) <= self.random_start_stop_speed:
                return
            self.publish_hard_brake()
            self.spin_for(min(0.05, max(0.0, deadline - time.monotonic())))
        if abs(float(self.speed)) > self.random_start_stop_speed:
            self.node.get_logger().warn(
                "RL random start proceeding while vehicle is still moving "
                f"speed={float(self.speed):.2f}m/s threshold={self.random_start_stop_speed:.2f}m/s",
                throttle_duration_sec=5.0,
            )

    def note_random_start_failure(self, message: str) -> None:
        self.random_start_failures += 1
        if self.random_start_failures >= self.random_start_disable_after_failures:
            self.random_start_disabled = True
            self.node.get_logger().warn(
                "RL random starts disabled "
                f"after {self.random_start_failures} failure(s): {message}"
            )
        else:
            self.node.get_logger().warn(f"RL random start skipped: {message}")

    def teleport_to_random_start(self) -> bool:
        if self.random_start_disabled or not bool(self.track.closed_loop):
            return False
        path = self.centerline_points or self.racing_points
        if len(path) < 3:
            return False
        try:
            client, Pose, Quaternionr, Vector3r = self.airsim_pose_client()
        except Exception as exc:  # pragma: no cover - depends on running simulator RPC
            self.note_random_start_failure(f"AirSim pose RPC unavailable: {exc}")
            return False

        candidates = self.random_start_candidates(path)
        if not candidates:
            self.node.get_logger().warn(
                "RL random start skipped; no safe blue/yellow midpoint candidates available",
                throttle_duration_sec=5.0,
            )
            return False
        index, point, width = candidates[int(self.np_random.integers(0, len(candidates)))]
        prev_p = path[(index - 1) % len(path)]
        next_p = path[(index + 1) % len(path)]
        yaw = math.atan2(float(next_p.y) - float(prev_p.y), float(next_p.x) - float(prev_p.x))
        z = float(self.pose.pose.position.z) if self.pose is not None else float(point.z)
        quat = Quaternionr(0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))
        paused = False
        try:
            if self.random_start_pause_during_teleport and hasattr(client, "simPause"):
                client.simPause(True)
                paused = True
            if hasattr(client, "enableApiControl"):
                client.enableApiControl(True, self.random_start_vehicle)
            client.simSetVehiclePose(
                Pose(
                    Vector3r(float(point.x), float(point.y), z + self.random_start_z_offset),
                    quat,
                ),
                True,
                self.random_start_vehicle,
            )
            self.random_start_failures = 0
        except Exception as exc:  # pragma: no cover - depends on running simulator RPC
            self.note_random_start_failure(f"simSetVehiclePose failed: {exc}")
            return False
        finally:
            if paused:
                try:
                    client.simPause(False)
                except Exception as exc:  # pragma: no cover - depends on running simulator RPC
                    self.node.get_logger().warn(f"RL random start could not unpause simulator: {exc}")
        self.publish_neutral()
        self.node.get_logger().info(
            "RL random start "
            f"sector index={index} x={float(point.x):.1f} y={float(point.y):.1f} "
            f"yaw={math.degrees(yaw):+.1f}deg width={width:.2f}m",
            throttle_duration_sec=5.0,
        )
        return True

    def random_start_candidates(self, path: list[Point]) -> list[tuple[int, Point, float]]:
        if not self.blue_boundary_points or not self.yellow_boundary_points:
            return []
        closed_loop = bool(self.track.closed_loop)
        reference_segments = path_segment_count(path, closed_loop=closed_loop)
        if reference_segments <= 0:
            return []
        min_width = max(2.0 * max(0.8, self.safe_clearance), 2.0)
        candidates: list[tuple[int, Point, float]] = []
        for index in range(len(path)):
            blue = point_at_reference_path_index(
                self.blue_boundary_points,
                float(index),
                reference_segments,
                closed_loop=closed_loop,
            )
            yellow = point_at_reference_path_index(
                self.yellow_boundary_points,
                float(index),
                reference_segments,
                closed_loop=closed_loop,
            )
            if blue is None or yellow is None:
                continue
            width = distance_xy(blue, yellow)
            if width < min_width:
                continue
            midpoint = Point()
            midpoint.x = 0.5 * (float(blue.x) + float(yellow.x))
            midpoint.y = 0.5 * (float(blue.y) + float(yellow.y))
            midpoint.z = 0.5 * (float(blue.z) + float(yellow.z))
            clearance = self.boundary_clearance_at(midpoint.x, midpoint.y)
            if clearance is None or float(clearance["min_clearance_m"]) < max(0.8, self.promotion_min_clearance):
                continue
            candidates.append((index, midpoint, width))
        return candidates

    def airsim_pose_client(self):
        if self.airsim_client is not None and self.airsim_pose_types is not None:
            Pose, Quaternionr, Vector3r = self.airsim_pose_types
            return self.airsim_client, Pose, Quaternionr, Vector3r

        repo_root = None
        for parent in Path(__file__).resolve().parents:
            if (parent / "AirSim" / "PythonClient" / "airsim" / "client.py").exists():
                repo_root = parent
                break
        if repo_root is None:
            raise RuntimeError("could not locate AirSim PythonClient")

        python_dir = repo_root / "python"
        airsim_client_dir = repo_root / "AirSim" / "PythonClient" / "airsim"
        for path in (str(python_dir), str(airsim_client_dir)):
            if path not in sys.path:
                sys.path.insert(0, path)

        from fsds.types import Pose, Quaternionr, Vector3r  # noqa: PLC0415
        import client as airsim_client  # noqa: PLC0415

        client = airsim_client.CarClient(
            ip=self.random_start_host,
            port=self.random_start_port,
            timeout_value=15,
        )
        if not client.ping():
            raise RuntimeError(f"could not ping AirSim RPC at {self.random_start_host}:{self.random_start_port}")
        self.airsim_client = client
        self.airsim_pose_types = (Pose, Quaternionr, Vector3r)
        return client, Pose, Quaternionr, Vector3r

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.02, max(0.0, end - time.monotonic())))

    def progress_s(self) -> float | None:
        if self.pose is None or not self.path_points or not self.arc_lengths:
            return None
        pos = self.pose.pose.position
        index = nearest_point_index(self.path_points, pos.x, pos.y)
        if index < 0:
            return None
        return self.arc_lengths[min(index, len(self.arc_lengths) - 1)]

    def progress_delta(self) -> float:
        current = self.progress_s()
        if current is None or self.prev_progress_s is None or self.path_length <= 1.0:
            self.prev_progress_s = current
            return 0.0
        delta = current - self.prev_progress_s
        if bool(self.track.closed_loop):
            if delta < -0.5 * self.path_length:
                delta += self.path_length
            elif delta > 0.5 * self.path_length:
                delta -= self.path_length
        self.prev_progress_s = current
        return float(np.clip(delta, -5.0, 8.0))

    def current_sector(self) -> int | None:
        progress = self.progress_s()
        if progress is None or self.path_length <= 1.0:
            return None
        return int((progress / self.path_length) * self.sector_count) % self.sector_count

    def update_sector_progress(self, progress: float) -> tuple[int, bool, float, list[int]]:
        current = self.current_sector()
        if current is None:
            return 0, False, 0.0, []
        if self.prev_sector is None:
            self.prev_sector = current
            self.next_sector = (current + 1) % self.sector_count
            self.last_sector_step = self.episode_step
            self.visited_sectors.add(current)
            return 0, False, 0.0, []
        forward = (current - self.prev_sector) % self.sector_count
        reverse = (self.prev_sector - current) % self.sector_count
        wrong_direction = progress < -0.05 or (reverse < forward and reverse > 0)
        new_count = 0
        reward = 0.0
        crossed_sectors = []
        if progress > 0.0 and 0 < forward <= 3:
            for offset in range(1, forward + 1):
                sector = (self.prev_sector + offset) % self.sector_count
                if self.next_sector is not None and sector == self.next_sector and sector not in self.visited_sectors:
                    self.visited_sectors.add(sector)
                    crossed_sectors.append(sector)
                    reward += self.sector_gate_reward()
                    self.last_sector_step = self.episode_step
                    self.next_sector = (sector + 1) % self.sector_count
                    new_count += 1
        self.prev_sector = current
        return new_count, wrong_direction, reward, crossed_sectors

    def path_error(self, path: list[Point]) -> float:
        if self.pose is None or not path:
            return 0.0
        pos = self.pose.pose.position
        index = nearest_point_index(path, pos.x, pos.y)
        if index < 0:
            return 0.0
        return math.hypot(float(pos.x) - float(path[index].x), float(pos.y) - float(path[index].y))

    def boundary_clearance(self) -> dict[str, float] | None:
        if self.pose is None or not self.track.centerline or not self.track.blue_boundary_line or not self.track.yellow_boundary_line:
            return None
        pos = self.pose.pose.position
        return self.boundary_clearance_at(float(pos.x), float(pos.y))

    def boundary_clearance_at(self, x: float, y: float) -> dict[str, float] | None:
        if not self.track.centerline or not self.track.blue_boundary_line or not self.track.yellow_boundary_line:
            return None
        closed_loop = bool(self.track.closed_loop)
        projection = closest_point_on_polyline_with_index(self.track.centerline, x, y, closed_loop=closed_loop)
        center_segments = path_segment_count(self.track.centerline, closed_loop=closed_loop)
        if projection is None or center_segments <= 0:
            return None
        _, _, center_distance, center_index = projection
        progress = center_index / float(center_segments)
        blue = point_at_reference_path_index(
            self.track.blue_boundary_line,
            center_index,
            center_segments,
            closed_loop=closed_loop,
        )
        yellow = point_at_reference_path_index(
            self.track.yellow_boundary_line,
            center_index,
            center_segments,
            closed_loop=closed_loop,
        )
        if blue is None or yellow is None:
            return None
        vx = float(blue.x) - float(yellow.x)
        vy = float(blue.y) - float(yellow.y)
        width = math.hypot(vx, vy)
        if width < 1e-6:
            return None
        wx = float(x) - float(yellow.x)
        wy = float(y) - float(yellow.y)
        fraction = (wx * vx + wy * vy) / (width * width)
        yellow_clearance = fraction * width
        blue_clearance = (1.0 - fraction) * width
        return {
            "yellow_clearance_m": yellow_clearance,
            "blue_clearance_m": blue_clearance,
            "min_clearance_m": min(yellow_clearance, blue_clearance),
            "track_width_m": width,
            "center_distance_m": center_distance,
            "progress_fraction": progress,
        }

    def cone_hit(self) -> bool:
        if self.extra_info is None:
            return False
        current = int(self.extra_info.doo_counter)
        if self.prev_doo_counter is None:
            self.prev_doo_counter = current
            return False
        hit = current > self.prev_doo_counter
        self.prev_doo_counter = current
        return hit

    def reset_status_bad(self) -> bool:
        if time.monotonic() - self.reset_status_time > self.reset_status_fresh_sec:
            return False
        text = self.reset_status.lower()
        safe_prefixes = (
            "armed_",
            "waiting_",
            "on_track",
            "reset_complete",
            "disabled",
        )
        if text.startswith(safe_prefixes):
            return False
        held_bad_prefixes = (
            "off_track ",
            "extreme_off_track ",
            "forbidden_area=yellow_side",
            "forbidden_area=blue_side",
            "forbidden_area=outside_loop",
        )
        if text.startswith(held_bad_prefixes):
            return self.reset_status_hold_sec(text) >= self.reset_status_hold_terminal_sec
        bad_prefixes = (
            "cone_hit ",
            "stuck ",
        )
        return text.startswith(bad_prefixes)

    @staticmethod
    def reset_status_hold_sec(text: str) -> float:
        marker = "hold="
        index = text.find(marker)
        if index < 0:
            return 0.0
        start = index + len(marker)
        end = start
        while end < len(text) and (text[end].isdigit() or text[end] in ".+-"):
            end += 1
        try:
            return float(text[start:end])
        except ValueError:
            return 0.0

    def stuck(self) -> bool:
        moving_target = self.target_speed > 0.5 or self.stage_direct_blend() > 0.0
        is_stuck = moving_target and abs(self.speed) < self.stuck_speed
        self.stuck_window.append(is_stuck)
        return len(self.stuck_window) == self.stuck_window.maxlen and all(self.stuck_window)

    def obs(self) -> np.ndarray:
        pose = pose2_from_pose_stamped(self.pose) if self.pose is not None else None
        progress = self.progress_s() or 0.0
        progress_norm = progress / max(1.0, self.path_length)
        sector = self.current_sector()
        sector_norm = float(sector if sector is not None else 0) / max(1, self.sector_count - 1)
        center_error = self.path_error(self.centerline_points)
        racing_error = self.path_error(self.racing_points)
        clear = self.boundary_clearance() or {}
        min_clearance = float(clear.get("min_clearance_m", 0.0))
        yellow_clearance = float(clear.get("yellow_clearance_m", 0.0))
        blue_clearance = float(clear.get("blue_clearance_m", 0.0))
        width = float(clear.get("track_width_m", 0.0))
        line_offsets = self.local_line_offsets()
        steering = float(self.last_control.steering)
        throttle = float(self.last_control.throttle)
        brake = float(self.last_control.brake)
        yaw = pose.yaw if pose is not None else 0.0
        base_values = np.asarray(
            [
                self.speed,
                self.target_speed,
                progress_norm,
                self.episode_progress / max(1.0, self.path_length),
                sector_norm,
                len(self.visited_sectors) / max(1.0, float(self.sector_count)),
                center_error,
                racing_error,
                min_clearance,
                yellow_clearance,
                blue_clearance,
                width,
                steering,
                throttle,
                brake,
                float(self.track.quality),
                1.0 if self.track.closed_loop else 0.0,
                1.0 if self.race_state.map_loaded else 0.0,
                1.0 if self.race_state.go_signal_fresh else 0.0,
                self.stage / 4.0,
                self.stage_direct_blend(),
                math.sin(yaw),
                math.cos(yaw),
                float(self.last_action[0]),
                float(self.last_action[1]),
                float(self.last_action[2]),
                float(self.last_action[3]),
                float(self.last_action[4]),
                line_offsets["centerline_x_m"],
                line_offsets["centerline_y_m"],
                line_offsets["racing_line_x_m"],
                line_offsets["racing_line_y_m"],
                line_offsets["yellow_line_x_m"],
                line_offsets["yellow_line_y_m"],
                line_offsets["blue_line_x_m"],
                line_offsets["blue_line_y_m"],
            ],
            dtype=np.float32,
        )
        base = np.concatenate([base_values, self.track_shape_features()]).astype(np.float32)
        return np.concatenate(
            [
                base,
                self.cone_features(),
                self.preview_features(self.centerline_points, self.center_arc_lengths),
                self.preview_features(self.racing_points, self.racing_arc_lengths),
                self.preview_features(self.yellow_boundary_points, self.yellow_arc_lengths),
                self.preview_features(self.blue_boundary_points, self.blue_arc_lengths),
            ]
        ).astype(np.float32)

    def local_line_offsets(self) -> dict[str, float]:
        center_x, center_y = self.local_offset_to_nearest(self.centerline_points)
        racing_x, racing_y = self.local_offset_to_nearest(self.racing_points)
        yellow_x, yellow_y = self.local_offset_to_nearest(self.yellow_boundary_points)
        blue_x, blue_y = self.local_offset_to_nearest(self.blue_boundary_points)
        return {
            "centerline_x_m": center_x,
            "centerline_y_m": center_y,
            "racing_line_x_m": racing_x,
            "racing_line_y_m": racing_y,
            "yellow_line_x_m": yellow_x,
            "yellow_line_y_m": yellow_y,
            "blue_line_x_m": blue_x,
            "blue_line_y_m": blue_y,
        }

    def local_offset_to_nearest(self, path: list[Point]) -> tuple[float, float]:
        if self.pose is None or not path:
            return 0.0, 0.0
        pos = self.pose.pose.position
        index = nearest_point_index(path, pos.x, pos.y)
        if index < 0:
            return 0.0, 0.0
        pose = pose2_from_pose_stamped(self.pose)
        local_x, local_y = transform_global_to_local(path[index].x, path[index].y, pose)
        return float(local_x), float(local_y)

    def cone_features(self) -> np.ndarray:
        features = np.zeros(self.cone_count * self.cone_feature_len, dtype=np.float32)
        cones = [
            cone
            for cone in self.local_cones.cones
            if float(cone.position.x) > 0.0 and abs(float(cone.position.y)) < 12.0
        ]
        cones.sort(key=lambda cone: float(cone.range) if float(cone.range) > 0.0 else math.hypot(float(cone.position.x), float(cone.position.y)))
        for index, cone in enumerate(cones[: self.cone_count]):
            color = int(cone.color)
            start = index * self.cone_feature_len
            features[start : start + self.cone_feature_len] = np.asarray(
                [
                    float(cone.position.x),
                    float(cone.position.y),
                    float(cone.range),
                    float(cone.confidence),
                    1.0 if color == ConeColor.YELLOW else 0.0,
                    1.0 if color == ConeColor.BLUE else 0.0,
                    1.0 if color in (ConeColor.ORANGE, ConeColor.LARGE_ORANGE) else 0.0,
                    1.0 if color == ConeColor.UNKNOWN else 0.0,
                ],
                dtype=np.float32,
            )
        return features

    def preview_features(self, path: list[Point], arc_lengths: list[float]) -> np.ndarray:
        features = np.zeros(self.preview_count * self.preview_feature_len, dtype=np.float32)
        if self.pose is None or not path or not arc_lengths or self.path_length <= 1.0:
            return features
        pose = pose2_from_pose_stamped(self.pose)
        current = self.progress_s() or 0.0
        for index in range(self.preview_count):
            lookahead = 2.0 + 2.0 * index
            point = self.point_at_s(path, arc_lengths, current + lookahead)
            local_x, local_y = transform_global_to_local(point.x, point.y, pose)
            start = index * self.preview_feature_len
            features[start] = float(local_x)
            features[start + 1] = float(local_y)
        return features

    def track_shape_features(self) -> np.ndarray:
        features = np.zeros(self.track_shape_feature_len, dtype=np.float32)
        if self.pose is None or self.path_length <= 1.0:
            features[12:16] = self.danger_zone_features()
            return features
        pose = pose2_from_pose_stamped(self.pose)
        station = self.progress_s() or 0.0
        width_4, center_y_4 = self.boundary_width_center_y_ahead(station, 4.0, pose)
        width_10, center_y_10 = self.boundary_width_center_y_ahead(station, 10.0, pose)
        features[:] = np.asarray(
            [
                self.path_heading_error_ahead(self.centerline_points, self.center_arc_lengths, station, 4.0, pose),
                self.path_heading_error_ahead(self.centerline_points, self.center_arc_lengths, station, 10.0, pose),
                self.path_heading_error_ahead(self.racing_points, self.racing_arc_lengths, station, 4.0, pose),
                self.path_heading_error_ahead(self.racing_points, self.racing_arc_lengths, station, 10.0, pose),
                self.path_curvature_ahead(self.centerline_points, self.center_arc_lengths, station, 4.0),
                self.path_curvature_ahead(self.centerline_points, self.center_arc_lengths, station, 10.0),
                self.path_curvature_ahead(self.racing_points, self.racing_arc_lengths, station, 4.0),
                self.path_curvature_ahead(self.racing_points, self.racing_arc_lengths, station, 10.0),
                width_4,
                center_y_4,
                width_10,
                center_y_10,
                *self.danger_zone_features().tolist(),
            ],
            dtype=np.float32,
        )
        return features

    def path_heading_error_ahead(
        self,
        path: list[Point],
        arc_lengths: list[float],
        station: float,
        lookahead: float,
        pose,
    ) -> float:
        if len(path) < 2 or len(arc_lengths) < 2:
            return 0.0
        p0 = self.point_at_s(path, arc_lengths, station + lookahead)
        p1 = self.point_at_s(path, arc_lengths, station + lookahead + 2.0)
        heading = math.atan2(float(p1.y) - float(p0.y), float(p1.x) - float(p0.x))
        return float(normalize_angle(heading - pose.yaw))

    def path_curvature_ahead(self, path: list[Point], arc_lengths: list[float], station: float, lookahead: float) -> float:
        if len(path) < 3 or len(arc_lengths) < 3:
            return 0.0
        before = max(0.0, lookahead - 2.0)
        p0 = self.point_at_s(path, arc_lengths, station + before)
        p1 = self.point_at_s(path, arc_lengths, station + lookahead)
        p2 = self.point_at_s(path, arc_lengths, station + lookahead + 2.0)
        d01 = distance_xy(p0, p1)
        d12 = distance_xy(p1, p2)
        if d01 < 1e-3 or d12 < 1e-3:
            return 0.0
        h1 = math.atan2(float(p1.y) - float(p0.y), float(p1.x) - float(p0.x))
        h2 = math.atan2(float(p2.y) - float(p1.y), float(p2.x) - float(p1.x))
        curvature = normalize_angle(h2 - h1) / max(1.0, d01 + d12)
        return float(clamp(curvature, -1.0, 1.0))

    def boundary_width_center_y_ahead(self, station: float, lookahead: float, pose) -> tuple[float, float]:
        if (
            len(self.blue_boundary_points) < 2
            or len(self.yellow_boundary_points) < 2
            or not self.blue_arc_lengths
            or not self.yellow_arc_lengths
            or self.path_length <= 1.0
        ):
            return 0.0, 0.0
        ahead_station = station + lookahead
        if bool(self.track.closed_loop):
            progress_fraction = (ahead_station % self.path_length) / self.path_length
        else:
            progress_fraction = clamp(ahead_station / self.path_length, 0.0, 1.0)
        blue = self.point_at_s(self.blue_boundary_points, self.blue_arc_lengths, progress_fraction * self.blue_arc_lengths[-1])
        yellow = self.point_at_s(
            self.yellow_boundary_points,
            self.yellow_arc_lengths,
            progress_fraction * self.yellow_arc_lengths[-1],
        )
        width = distance_xy(blue, yellow)
        center_x = 0.5 * (float(blue.x) + float(yellow.x))
        center_y = 0.5 * (float(blue.y) + float(yellow.y))
        _, local_center_y = transform_global_to_local(center_x, center_y, pose)
        return float(width), float(local_center_y)

    def danger_zone_features(self) -> np.ndarray:
        features = np.zeros(4, dtype=np.float32)
        if not self.danger_zone_enabled or self.pose is None or not self.danger_points:
            return features
        pose = pose2_from_pose_stamped(self.pose)
        pos = self.pose.pose.position
        nearest: tuple[float, float, float, float] | None = None
        for x, y, weight in self.danger_points:
            distance = math.hypot(float(pos.x) - x, float(pos.y) - y)
            if nearest is None or distance < nearest[0]:
                nearest = (distance, x, y, weight)
        if nearest is None:
            return features
        distance, x, y, weight = nearest
        local_x, local_y = transform_global_to_local(x, y, pose)
        strength = clamp(1.0 - distance / self.danger_zone_radius, 0.0, 1.0) * weight
        features[:] = np.asarray(
            [
                clamp(float(local_x), -30.0, 30.0),
                clamp(float(local_y), -30.0, 30.0),
                clamp(strength, 0.0, 1.0),
                min(1.0, len(self.danger_points) / 80.0),
            ],
            dtype=np.float32,
        )
        return features

    def point_at_s(self, path: list[Point], arc_lengths: list[float], station: float) -> Point:
        if not path:
            return Point()
        if bool(self.track.closed_loop) and arc_lengths[-1] > 1e-6:
            station = station % arc_lengths[-1]
        else:
            station = clamp(station, 0.0, arc_lengths[-1])
        for index in range(1, len(arc_lengths)):
            if arc_lengths[index] < station:
                continue
            prev_s = arc_lengths[index - 1]
            next_s = arc_lengths[index]
            span = max(1e-6, next_s - prev_s)
            ratio = clamp((station - prev_s) / span, 0.0, 1.0)
            prev_p = path[index - 1]
            next_p = path[index]
            p = Point()
            p.x = float(prev_p.x) + (float(next_p.x) - float(prev_p.x)) * ratio
            p.y = float(prev_p.y) + (float(next_p.y) - float(prev_p.y)) * ratio
            p.z = float(prev_p.z) + (float(next_p.z) - float(prev_p.z)) * ratio
            return p
        return path[-1]

    def write_reward_log(
        self,
        action: np.ndarray,
        scaled_action: dict[str, float],
        reward: float,
        reward_parts: dict[str, float],
        progress: float,
        center_error: float,
        racing_error: float,
        clearance: dict[str, float] | None,
        terminal_reason: str,
        truncated: bool,
        crossed_sectors: list[int],
    ) -> None:
        if self.reward_log_file is None:
            return

        pose_msg = self.pose.pose if self.pose is not None else None
        control = self.last_control
        row = {
            "wall_time": time.time(),
            "total_step": self.total_step_count,
            "episode_step": self.episode_step,
            "episode_step_limit": self.current_episode_step_limit,
            "stage": self.stage,
            "direct_blend": self.stage_direct_blend(),
            "reward": reward,
            "reward_parts": reward_parts,
            "progress_delta_m": float(progress),
            "episode_progress_m": float(self.episode_progress),
            "path_length_m": float(self.path_length),
            "sector": self.current_sector(),
            "next_sector": self.next_sector,
            "crossed_sectors": [int(sector) for sector in crossed_sectors],
            "visited_sector_count": len(self.visited_sectors),
            "speed_mps": float(self.speed),
            "target_speed_mps": float(self.target_speed),
            "centerline_error_m": float(center_error),
            "racing_line_error_m": float(racing_error),
            "line_offsets": self.local_line_offsets(),
            "clearance": clearance or {},
            "mistakes": {
                "budget": float(self.mistake_budget),
                "budget_limit": float(self.mistake_budget_limit),
                "offtrack_steps": int(self.offtrack_steps),
                "reset_bad_steps": int(self.reset_bad_steps),
                "low_clearance_steps": int(self.low_clearance_steps),
                "wrong_direction_steps": int(self.wrong_direction_steps),
                "no_progress_steps": int(self.no_progress_steps),
                "no_progress_grace_steps": int(self.no_progress_grace_steps),
                "low_clearance_terminal_m": float(self.low_clearance_terminal),
            },
            "reset_status": self.reset_status,
            "terminal_reason": terminal_reason,
            "truncated": bool(truncated),
            "rl_action": [float(value) for value in action.tolist()],
            "rl_scaled_action": {key: float(value) for key, value in scaled_action.items()},
            "control": {
                "steering": float(control.steering),
                "throttle": float(control.throttle),
                "brake": float(control.brake),
            },
            "pose": {
                "x": float(pose_msg.position.x) if pose_msg is not None else None,
                "y": float(pose_msg.position.y) if pose_msg is not None else None,
                "z": float(pose_msg.position.z) if pose_msg is not None else None,
                "qx": float(pose_msg.orientation.x) if pose_msg is not None else None,
                "qy": float(pose_msg.orientation.y) if pose_msg is not None else None,
                "qz": float(pose_msg.orientation.z) if pose_msg is not None else None,
                "qw": float(pose_msg.orientation.w) if pose_msg is not None else None,
            },
            "extra_info": {
                "doo_counter": int(self.extra_info.doo_counter) if self.extra_info is not None else None,
            },
        }
        self.reward_log_file.write(json.dumps(row, sort_keys=True) + "\n")
        self.reward_log_count += 1
        if self.reward_log_count % self.reward_log_flush_every == 0 or terminal_reason or truncated:
            self.reward_log_file.flush()

    def write_episode_summary(
        self,
        *,
        terminal_reason: str,
        truncated: bool,
        stage_before_update: int,
        stage_after_update: int,
    ) -> None:
        if self.episode_summary_log_file is None:
            return

        metrics = self.episode_metrics
        control_count = max(1, int(metrics.get("control_count", 0)))
        action_count = max(1, int(self.episode_step))
        path_length = float(self.path_length)
        progress_fraction = self.episode_progress / path_length if path_length > 1.0 else 0.0
        control_time = float(self.episode_step) * self.step_dt
        elapsed_wall = time.time() - float(metrics.get("start_wall_time", time.time()))
        clearance_min = float(metrics.get("clearance_min", float("inf")))
        if not math.isfinite(clearance_min):
            clearance_min = 0.0
        doo_counter = int(self.extra_info.doo_counter) if self.extra_info is not None else None
        start_doo_counter = metrics.get("start_doo_counter")
        doo_delta = None
        if doo_counter is not None and isinstance(start_doo_counter, int):
            doo_delta = max(0, doo_counter - start_doo_counter)

        row = {
            "schema": "fsds_live_rl_episode_summary_v1",
            "wall_time": time.time(),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "seed": self.seed,
            "episode_index": int(self.episode_index),
            "total_step": int(self.total_step_count),
            "start_total_step": int(metrics.get("start_total_step", 0)),
            "episode_step": int(self.episode_step),
            "episode_step_limit": int(self.current_episode_step_limit),
            "control_time_sec": control_time,
            "elapsed_wall_sec": elapsed_wall,
            "stage_start": int(metrics.get("start_stage", stage_before_update)),
            "stage_before_update": int(stage_before_update),
            "stage_after_update": int(stage_after_update),
            "stage_changed": bool(stage_after_update != stage_before_update),
            "direct_blend_start": float(metrics.get("start_direct_blend", 0.0)),
            "direct_blend_end": float(self.stage_direct_blend()),
            "direct_blend_avg": float(metrics.get("direct_blend_sum", 0.0)) / action_count,
            "terminal_reason": terminal_reason or ("truncated" if truncated else ""),
            "truncated": bool(truncated),
            "lap_complete": terminal_reason == "lap_complete",
            "progress_m": float(self.episode_progress),
            "positive_progress_m": float(metrics.get("positive_progress_m", 0.0)),
            "backward_progress_m": float(metrics.get("backward_progress_m", 0.0)),
            "path_length_m": path_length,
            "progress_fraction": float(progress_fraction),
            "progress_percent": float(100.0 * progress_fraction),
            "progress_m_per_control_sec": float(self.episode_progress / max(self.step_dt, control_time)),
            "sector_count": int(len(self.visited_sectors)),
            "sector_fraction": float(len(self.visited_sectors) / max(1, self.sector_count)),
            "start_sector": metrics.get("start_sector"),
            "end_sector": self.current_sector(),
            "reward_total": float(metrics.get("reward_total", 0.0)),
            "reward_parts": metrics.get("reward_parts", {}),
            "speed_avg_mps": self.episode_average("speed"),
            "speed_max_mps": float(metrics.get("speed_max", 0.0)),
            "target_speed_avg_mps": self.episode_average("target_speed"),
            "clearance_min_m": clearance_min,
            "clearance_avg_m": self.episode_average("clearance"),
            "safe_clearance_violation_steps": int(metrics.get("safe_clearance_violation_steps", 0)),
            "safe_clearance_violation_fraction": int(metrics.get("safe_clearance_violation_steps", 0)) / action_count,
            "center_error_avg_m": self.metric_average("center_error_sum", "center_error_count"),
            "center_error_max_m": float(metrics.get("center_error_max", 0.0)),
            "racing_error_avg_m": self.metric_average("racing_error_sum", "racing_error_count"),
            "racing_error_max_m": float(metrics.get("racing_error_max", 0.0)),
            "danger_zone_avg": float(metrics.get("danger_zone_sum", 0.0)) / action_count,
            "danger_zone_max": float(metrics.get("danger_zone_max", 0.0)),
            "mistake_budget": float(self.mistake_budget),
            "mistake_budget_limit": float(self.mistake_budget_limit),
            "offtrack_steps": int(self.offtrack_steps),
            "reset_bad_steps": int(self.reset_bad_steps),
            "low_clearance_steps": int(self.low_clearance_steps),
            "wrong_direction_steps": int(self.wrong_direction_steps),
            "no_progress_steps": int(self.no_progress_steps),
            "cone_hit": terminal_reason == "cone_hit",
            "reset_status": self.reset_status,
            "doo_counter_start": start_doo_counter,
            "doo_counter_end": doo_counter,
            "doo_counter_delta": doo_delta,
            "control": {
                "avg_abs_steering": float(metrics.get("abs_steering_sum", 0.0)) / control_count,
                "avg_throttle": float(metrics.get("throttle_sum", 0.0)) / control_count,
                "avg_brake": float(metrics.get("brake_sum", 0.0)) / control_count,
                "full_steer_step_fraction": int(metrics.get("full_steer_steps", 0)) / control_count,
                "throttle_step_fraction": int(metrics.get("throttle_steps", 0)) / control_count,
                "brake_step_fraction": int(metrics.get("brake_steps", 0)) / control_count,
            },
            "rl_action": {
                "avg_abs_raw": [float(value) / action_count for value in metrics.get("raw_action_abs_sum", [0.0] * 5)],
                "avg_abs_path_offset_m": float(metrics.get("scaled_path_offset_abs_sum", 0.0)) / action_count,
                "avg_abs_speed_delta_mps": float(metrics.get("scaled_speed_delta_abs_sum", 0.0)) / action_count,
                "avg_abs_direct_steering": float(metrics.get("scaled_steering_abs_sum", 0.0)) / action_count,
                "avg_direct_throttle": float(metrics.get("scaled_throttle_sum", 0.0)) / action_count,
                "avg_direct_brake": float(metrics.get("scaled_brake_sum", 0.0)) / action_count,
            },
            "start_pose": metrics.get("start_pose", {}),
            "end_pose": self.pose_snapshot(),
        }
        self.episode_summary_log_file.write(json.dumps(row, sort_keys=True) + "\n")
        self.episode_summary_log_count += 1
        if self.episode_summary_log_count % self.episode_summary_log_flush_every == 0:
            self.episode_summary_log_file.flush()

    def close(self) -> None:
        self.publish_neutral()
        if self.reward_log_file is not None:
            self.reward_log_file.flush()
            self.reward_log_file.close()
            self.reward_log_file = None
        if self.episode_summary_log_file is not None:
            self.episode_summary_log_file.flush()
            self.episode_summary_log_file.close()
            self.episode_summary_log_file = None
        self.node.destroy_node()


# Backwards-compatible name for older imports and experiments.
LiveResidualEnv = LiveFullControlEnv

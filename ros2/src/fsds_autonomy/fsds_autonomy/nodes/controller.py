from __future__ import annotations

import math
from statistics import median

import rclpy
from fs_msgs.msg import ControlCommand
from fsds_autonomy_msgs.msg import ConeArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String

from fsds_autonomy.constants import ConeColor, is_reliable_geometry_source
from fsds_autonomy.geometry import (
    clamp,
    closest_point_on_polyline_with_index,
    distance_xy,
    nearest_point_index,
    path_segment_count,
    point_at_reference_path_index,
    point,
    pose2_from_pose_stamped,
    transform_global_to_local,
)


class Controller(Node):
    def __init__(self) -> None:
        super().__init__("fsds_controller")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("lookahead_m", 5.0)
        self.declare_parameter("centerline_lookahead_m", 2.8)
        self.declare_parameter("racing_lookahead_m", 5.0)
        self.declare_parameter("centerline_max_lookahead_m", 4.0)
        self.declare_parameter("racing_max_lookahead_m", 8.0)
        self.declare_parameter("steering_preview_time_s", 1.0)
        self.declare_parameter("wheelbase_m", 1.20)
        self.declare_parameter("max_steering_angle_rad", 0.48)
        self.declare_parameter("steering_rate_limit_per_sec", 2.5)
        self.declare_parameter("track_half_width_m", 1.75)
        self.declare_parameter("throttle_kp", 0.25)
        self.declare_parameter("brake_kp", 0.35)
        self.declare_parameter("max_throttle", 0.55)
        self.declare_parameter("max_brake", 0.80)
        self.declare_parameter("max_path_offset_m", 1.0)
        self.declare_parameter("global_path_min_quality", 0.55)
        self.declare_parameter("centerline_path_min_quality", 0.20)
        self.declare_parameter("centerline_path_min_points", 4)
        self.declare_parameter("global_path_max_error_m", 5.0)
        self.declare_parameter("path_recovery_enabled", True)
        self.declare_parameter("path_recovery_error_m", 1.0)
        self.declare_parameter("path_recovery_lookahead_m", 1.8)
        self.declare_parameter("global_target_lateral_limit_m", 4.0)
        self.declare_parameter("path_target_memory_enabled", True)
        self.declare_parameter("path_target_search_back_points", 6)
        self.declare_parameter("path_target_search_forward_points", 55)
        self.declare_parameter("path_target_reacquire_distance_m", 4.0)
        self.declare_parameter("path_target_max_jump_points", 28)
        self.declare_parameter("target_lateral_filter_enabled", True)
        self.declare_parameter("target_lateral_filter_alpha", 0.45)
        self.declare_parameter("target_lateral_filter_max_step_m", 0.35)
        self.declare_parameter("launch_throttle", 0.50)
        self.declare_parameter("launch_speed_threshold_mps", 1.80)
        self.declare_parameter("local_cone_min_confidence", 0.50)
        self.declare_parameter("local_target_cone_clearance_m", 1.65)
        self.declare_parameter("local_target_lateral_limit_m", 3.0)
        self.declare_parameter("local_unknown_boundary_min_abs_y_m", 1.15)
        self.declare_parameter("local_unknown_boundary_max_abs_y_m", 4.5)
        self.declare_parameter("prefer_centerline_until_closed_loop", False)
        self.declare_parameter("prefer_local_cones_until_closed_loop", True)
        self.declare_parameter("close_cone_brake_enabled", True)
        self.declare_parameter("close_cone_brake_distance_m", 1.6)
        self.declare_parameter("close_cone_brake_lateral_m", 0.95)
        self.declare_parameter("close_cone_brake_min_speed_mps", 0.35)
        self.declare_parameter("close_cone_hard_stop_distance_m", 0.90)
        self.declare_parameter("close_cone_hard_stop_lateral_m", 0.30)
        self.declare_parameter("boundary_guard_enabled", True)
        self.declare_parameter("boundary_guard_margin_m", 1.65)
        self.declare_parameter("boundary_guard_gain", 1.10)
        self.declare_parameter("boundary_guard_max_shift_m", 1.40)
        self.declare_parameter("boundary_guard_min_width_m", 2.2)
        self.declare_parameter("boundary_guard_max_width_m", 8.0)
        self.declare_parameter("boundary_guard_slow_margin_m", 1.25)
        self.declare_parameter("boundary_guard_slow_speed_mps", 0.9)
        self.declare_parameter("boundary_guard_brake_margin_m", 0.80)
        self.declare_parameter("open_boundary_guard_slow_margin_m", 1.45)
        self.declare_parameter("open_boundary_guard_slow_speed_mps", 0.95)
        self.declare_parameter("open_boundary_guard_critical_margin_m", 0.95)
        self.declare_parameter("open_boundary_guard_critical_speed_mps", 0.55)
        self.declare_parameter("edge_recovery_enabled", True)
        self.declare_parameter("edge_recovery_clearance_m", 1.15)
        self.declare_parameter("edge_recovery_target_x_m", 2.0)
        self.declare_parameter("closed_loop_racing_requires_saved_map", True)
        self.declare_parameter("closed_loop_live_map_speed_mps", 1.4)
        self.declare_parameter("closed_loop_local_fallback_speed_mps", 0.9)
        self.declare_parameter("closed_loop_edge_recovery_clearance_m", 1.35)
        self.declare_parameter("closed_loop_edge_recovery_speed_mps", 0.55)
        self.declare_parameter("rl_residual_enabled", True)
        self.declare_parameter("rl_action_timeout_sec", 0.4)
        self.declare_parameter("rl_path_offset_limit_m", 0.60)
        self.declare_parameter("rl_speed_delta_limit_mps", 1.20)
        self.declare_parameter("rl_direct_control_enabled", True)
        self.declare_parameter("rl_direct_action_timeout_sec", 0.25)
        self.declare_parameter("rl_direct_blend_limit", 1.0)
        self.declare_parameter("rl_safety_overrides_enabled", True)

        self.lookahead = float(self.get_parameter("lookahead_m").value)
        self.centerline_lookahead = float(self.get_parameter("centerline_lookahead_m").value)
        self.racing_lookahead = float(self.get_parameter("racing_lookahead_m").value)
        self.centerline_max_lookahead = float(self.get_parameter("centerline_max_lookahead_m").value)
        self.racing_max_lookahead = float(self.get_parameter("racing_max_lookahead_m").value)
        self.steering_preview_time = float(self.get_parameter("steering_preview_time_s").value)
        self.wheelbase = float(self.get_parameter("wheelbase_m").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle_rad").value)
        self.steering_rate_limit = float(self.get_parameter("steering_rate_limit_per_sec").value)
        self.track_half_width = float(self.get_parameter("track_half_width_m").value)
        self.throttle_kp = float(self.get_parameter("throttle_kp").value)
        self.brake_kp = float(self.get_parameter("brake_kp").value)
        self.max_throttle = float(self.get_parameter("max_throttle").value)
        self.max_brake = float(self.get_parameter("max_brake").value)
        self.max_path_offset = float(self.get_parameter("max_path_offset_m").value)
        self.global_path_min_quality = float(self.get_parameter("global_path_min_quality").value)
        self.centerline_path_min_quality = float(self.get_parameter("centerline_path_min_quality").value)
        self.centerline_path_min_points = int(self.get_parameter("centerline_path_min_points").value)
        self.global_path_max_error = float(self.get_parameter("global_path_max_error_m").value)
        self.path_recovery_enabled = bool(self.get_parameter("path_recovery_enabled").value)
        self.path_recovery_error = float(self.get_parameter("path_recovery_error_m").value)
        self.path_recovery_lookahead = float(self.get_parameter("path_recovery_lookahead_m").value)
        self.global_target_lateral_limit = float(self.get_parameter("global_target_lateral_limit_m").value)
        self.path_target_memory_enabled = bool(self.get_parameter("path_target_memory_enabled").value)
        self.path_target_search_back_points = int(self.get_parameter("path_target_search_back_points").value)
        self.path_target_search_forward_points = int(self.get_parameter("path_target_search_forward_points").value)
        self.path_target_reacquire_distance = float(self.get_parameter("path_target_reacquire_distance_m").value)
        self.path_target_max_jump_points = int(self.get_parameter("path_target_max_jump_points").value)
        self.target_lateral_filter_enabled = bool(self.get_parameter("target_lateral_filter_enabled").value)
        self.target_lateral_filter_alpha = float(self.get_parameter("target_lateral_filter_alpha").value)
        self.target_lateral_filter_max_step = float(self.get_parameter("target_lateral_filter_max_step_m").value)
        self.launch_throttle = float(self.get_parameter("launch_throttle").value)
        self.launch_speed_threshold = float(self.get_parameter("launch_speed_threshold_mps").value)
        self.local_cone_min_confidence = float(self.get_parameter("local_cone_min_confidence").value)
        self.local_target_cone_clearance = float(self.get_parameter("local_target_cone_clearance_m").value)
        self.local_target_lateral_limit = float(self.get_parameter("local_target_lateral_limit_m").value)
        self.local_unknown_boundary_min_abs_y = float(self.get_parameter("local_unknown_boundary_min_abs_y_m").value)
        self.local_unknown_boundary_max_abs_y = float(self.get_parameter("local_unknown_boundary_max_abs_y_m").value)
        self.prefer_centerline_until_closed_loop = bool(
            self.get_parameter("prefer_centerline_until_closed_loop").value
        )
        self.prefer_local_cones_until_closed_loop = bool(
            self.get_parameter("prefer_local_cones_until_closed_loop").value
        )
        self.close_cone_brake_enabled = bool(self.get_parameter("close_cone_brake_enabled").value)
        self.close_cone_brake_distance = float(self.get_parameter("close_cone_brake_distance_m").value)
        self.close_cone_brake_lateral = float(self.get_parameter("close_cone_brake_lateral_m").value)
        self.close_cone_brake_min_speed = float(self.get_parameter("close_cone_brake_min_speed_mps").value)
        self.close_cone_hard_stop_distance = float(self.get_parameter("close_cone_hard_stop_distance_m").value)
        self.close_cone_hard_stop_lateral = float(self.get_parameter("close_cone_hard_stop_lateral_m").value)
        self.boundary_guard_enabled = bool(self.get_parameter("boundary_guard_enabled").value)
        self.boundary_guard_margin = float(self.get_parameter("boundary_guard_margin_m").value)
        self.boundary_guard_gain = float(self.get_parameter("boundary_guard_gain").value)
        self.boundary_guard_max_shift = float(self.get_parameter("boundary_guard_max_shift_m").value)
        self.boundary_guard_min_width = float(self.get_parameter("boundary_guard_min_width_m").value)
        self.boundary_guard_max_width = float(self.get_parameter("boundary_guard_max_width_m").value)
        self.boundary_guard_slow_margin = float(self.get_parameter("boundary_guard_slow_margin_m").value)
        self.boundary_guard_slow_speed = float(self.get_parameter("boundary_guard_slow_speed_mps").value)
        self.boundary_guard_brake_margin = float(self.get_parameter("boundary_guard_brake_margin_m").value)
        self.open_boundary_guard_slow_margin = float(self.get_parameter("open_boundary_guard_slow_margin_m").value)
        self.open_boundary_guard_slow_speed = float(self.get_parameter("open_boundary_guard_slow_speed_mps").value)
        self.open_boundary_guard_critical_margin = float(
            self.get_parameter("open_boundary_guard_critical_margin_m").value
        )
        self.open_boundary_guard_critical_speed = float(
            self.get_parameter("open_boundary_guard_critical_speed_mps").value
        )
        self.edge_recovery_enabled = bool(self.get_parameter("edge_recovery_enabled").value)
        self.edge_recovery_clearance = float(self.get_parameter("edge_recovery_clearance_m").value)
        self.edge_recovery_target_x = float(self.get_parameter("edge_recovery_target_x_m").value)
        self.closed_loop_racing_requires_saved_map = bool(
            self.get_parameter("closed_loop_racing_requires_saved_map").value
        )
        self.closed_loop_live_map_speed = float(self.get_parameter("closed_loop_live_map_speed_mps").value)
        self.closed_loop_local_fallback_speed = float(
            self.get_parameter("closed_loop_local_fallback_speed_mps").value
        )
        self.closed_loop_edge_recovery_clearance = float(
            self.get_parameter("closed_loop_edge_recovery_clearance_m").value
        )
        self.closed_loop_edge_recovery_speed = float(
            self.get_parameter("closed_loop_edge_recovery_speed_mps").value
        )
        self.rl_residual_enabled = bool(self.get_parameter("rl_residual_enabled").value)
        self.rl_action_timeout = float(self.get_parameter("rl_action_timeout_sec").value)
        self.rl_path_offset_limit = float(self.get_parameter("rl_path_offset_limit_m").value)
        self.rl_speed_delta_limit = float(self.get_parameter("rl_speed_delta_limit_mps").value)
        self.rl_direct_control_enabled = bool(self.get_parameter("rl_direct_control_enabled").value)
        self.rl_direct_action_timeout = float(self.get_parameter("rl_direct_action_timeout_sec").value)
        self.rl_direct_blend_limit = float(self.get_parameter("rl_direct_blend_limit").value)
        self.rl_safety_overrides_enabled = bool(self.get_parameter("rl_safety_overrides_enabled").value)

        self.pose = None
        self.speed = 0.0
        self.target_speed = 0.0
        self.race_state = RaceState()
        self.track = TrackMap()
        self.local_cones = ConeArray()
        self.path_offset = 0.0
        self.rl_path_offset = 0.0
        self.rl_speed_delta = 0.0
        self.last_rl_action_time = None
        self.rl_direct_command = ControlCommand()
        self.rl_direct_blend = 0.0
        self.last_rl_direct_command_time = None
        self.last_rl_direct_blend_time = None
        self.last_rl_direct_blend_applied = 0.0
        self.last_steering = 0.0
        self.last_steering_time = self.get_clock().now()
        self.path_cursor_source = ""
        self.path_cursor_track_id = ""
        self.path_cursor_closed_loop = False
        self.path_cursor_index: int | None = None
        self.path_target_index: int | None = None
        self.current_target_source = ""
        self.last_target_source = ""
        self.last_target_local_y: float | None = None
        self.last_target_local_x: float | None = None
        self.last_target_local_y_final: float | None = None
        self.last_path_error = float("inf")
        self.last_boundary_clearances: tuple[float, float, float] | None = None
        self.last_close_cone = False
        self.last_brake_reason = "init"

        self.control_pub = self.create_publisher(ControlCommand, "/fsds/control_command", 10)
        self.diagnostics_pub = self.create_publisher(String, "/autonomy/controller_diagnostics", 10)
        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)
        self.create_subscription(Float32, "/autonomy/speed", self.on_speed, 10)
        self.create_subscription(Float32, "/autonomy/target_speed", self.on_target_speed, 10)
        self.create_subscription(RaceState, "/autonomy/race_state", self.on_race_state, 10)
        self.create_subscription(TrackMap, "/autonomy/racing_line", self.on_track, 10)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_local_cones, 10)
        self.create_subscription(Float32, "/autonomy/path_offset", self.on_path_offset, 10)
        self.create_subscription(Float32, "/autonomy/rl_path_offset", self.on_rl_path_offset, 10)
        self.create_subscription(Float32, "/autonomy/rl_speed_delta", self.on_rl_speed_delta, 10)
        self.create_subscription(ControlCommand, "/autonomy/rl_control_command", self.on_rl_control_command, 10)
        self.create_subscription(Float32, "/autonomy/rl_direct_blend", self.on_rl_direct_blend, 10)

        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self.publish_control)

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg

    def on_speed(self, msg: Float32) -> None:
        self.speed = float(msg.data)

    def on_target_speed(self, msg: Float32) -> None:
        self.target_speed = float(msg.data)

    def on_race_state(self, msg: RaceState) -> None:
        self.race_state = msg

    def on_track(self, msg: TrackMap) -> None:
        if msg.track_id != self.path_cursor_track_id or bool(msg.closed_loop) != self.path_cursor_closed_loop:
            self.reset_path_cursor()
        self.track = msg

    def on_local_cones(self, msg: ConeArray) -> None:
        self.local_cones = msg

    def on_path_offset(self, msg: Float32) -> None:
        self.path_offset = clamp(float(msg.data), -self.max_path_offset, self.max_path_offset)

    def on_rl_path_offset(self, msg: Float32) -> None:
        if not self.rl_residual_enabled:
            return
        self.rl_path_offset = clamp(float(msg.data), -self.rl_path_offset_limit, self.rl_path_offset_limit)
        self.last_rl_action_time = self.get_clock().now()

    def on_rl_speed_delta(self, msg: Float32) -> None:
        if not self.rl_residual_enabled:
            return
        self.rl_speed_delta = clamp(float(msg.data), -self.rl_speed_delta_limit, self.rl_speed_delta_limit)
        self.last_rl_action_time = self.get_clock().now()

    def rl_action_fresh(self) -> bool:
        if not self.rl_residual_enabled or self.last_rl_action_time is None:
            return False
        age = (self.get_clock().now() - self.last_rl_action_time).nanoseconds * 1e-9
        return age <= max(0.0, self.rl_action_timeout)

    def effective_path_offset(self) -> float:
        rl_offset = self.rl_path_offset if self.rl_action_fresh() else 0.0
        return clamp(self.path_offset + rl_offset, -self.max_path_offset, self.max_path_offset)

    def effective_speed_delta(self) -> float:
        return self.rl_speed_delta if self.rl_action_fresh() else 0.0

    def on_rl_control_command(self, msg: ControlCommand) -> None:
        if not self.rl_direct_control_enabled:
            return
        self.rl_direct_command = ControlCommand()
        self.rl_direct_command.steering = clamp(float(msg.steering), -1.0, 1.0)
        self.rl_direct_command.throttle = clamp(float(msg.throttle), 0.0, 1.0)
        self.rl_direct_command.brake = clamp(float(msg.brake), 0.0, 1.0)
        self.last_rl_direct_command_time = self.get_clock().now()

    def on_rl_direct_blend(self, msg: Float32) -> None:
        if not self.rl_direct_control_enabled:
            return
        limit = clamp(self.rl_direct_blend_limit, 0.0, 1.0)
        self.rl_direct_blend = clamp(float(msg.data), 0.0, limit)
        self.last_rl_direct_blend_time = self.get_clock().now()

    def rl_direct_fresh(self) -> bool:
        if (
            not self.rl_direct_control_enabled
            or self.last_rl_direct_command_time is None
            or self.last_rl_direct_blend_time is None
        ):
            return False
        now = self.get_clock().now()
        command_age = (now - self.last_rl_direct_command_time).nanoseconds * 1e-9
        blend_age = (now - self.last_rl_direct_blend_time).nanoseconds * 1e-9
        timeout = max(0.0, self.rl_direct_action_timeout)
        return command_age <= timeout and blend_age <= timeout

    def effective_rl_direct_blend(self) -> float:
        if not self.rl_direct_fresh():
            return 0.0
        return clamp(self.rl_direct_blend, 0.0, clamp(self.rl_direct_blend_limit, 0.0, 1.0))

    def global_path_target_local(self) -> tuple[float, float] | None:
        if self.pose is None:
            return None
        selected = self.selected_drive_path()
        if selected is None:
            return None
        path, source, min_quality, min_points = selected
        if not path or len(path) < min_points or self.track.quality < min_quality:
            return None
        pose = pose2_from_pose_stamped(self.pose)
        nearest = self.path_cursor_nearest_index(path, source, pose.x, pose.y)
        if nearest < 0:
            return None
        path_error = distance_xy(path[nearest], point(pose.x, pose.y, 0.0))
        self.last_path_error = path_error
        if path_error > self.global_path_max_error:
            self.reset_path_cursor()
            return None
        lookahead = self.drive_lookahead()
        if self.path_recovery_enabled and path_error >= self.path_recovery_error:
            lookahead = min(lookahead, max(1.0, self.path_recovery_lookahead))
        target, target_index = self.path_target_after_distance(path, nearest, lookahead)
        if target is None:
            return None
        self.path_cursor_source = source
        self.path_cursor_track_id = self.track.track_id
        self.path_cursor_closed_loop = bool(self.track.closed_loop)
        self.path_cursor_index = nearest
        self.path_target_index = target_index
        local_x, local_y = transform_global_to_local(target.x, target.y, pose)
        if local_x < 0.5:
            return None
        local_y = local_y + self.effective_path_offset() + self.boundary_guard_shift_y()
        local_y = self.cone_clearance_limited_target_y(
            local_x,
            local_y,
            self.local_safety_cones(),
            lateral_limit=max(self.track_half_width, self.global_target_lateral_limit),
        )
        self.current_target_source = source
        return local_x, local_y

    def selected_drive_path(self):
        if not self.track.closed_loop:
            min_points = max(2, self.centerline_path_min_points)
            if self.prefer_centerline_until_closed_loop and len(self.track.centerline) >= min_points:
                return self.track.centerline, "centerline", self.centerline_path_min_quality, min_points
            return self.track.racing_line or self.track.centerline, "open_racing", self.centerline_path_min_quality, min_points
        if self.live_closed_loop_learning_map() and self.track.centerline:
            min_points = max(2, self.centerline_path_min_points)
            return self.track.centerline, "closed_centerline", self.centerline_path_min_quality, min_points
        return self.track.racing_line, "racing", self.global_path_min_quality, 2

    def live_closed_loop_learning_map(self) -> bool:
        if not self.track.closed_loop or not self.closed_loop_racing_requires_saved_map:
            return False
        return str(self.race_state.mode) != "race_from_map" or not bool(self.race_state.map_loaded)

    def reset_path_cursor(self) -> None:
        self.path_cursor_source = ""
        self.path_cursor_track_id = ""
        self.path_cursor_closed_loop = False
        self.path_cursor_index = None
        self.path_target_index = None

    def path_cursor_nearest_index(self, path, source: str, x: float, y: float) -> int:
        if not path:
            return -1
        full_nearest = nearest_point_index(path, x, y)
        if (
            not self.path_target_memory_enabled
            or self.path_cursor_index is None
            or source != self.path_cursor_source
            or self.track.track_id != self.path_cursor_track_id
            or bool(self.track.closed_loop) != self.path_cursor_closed_loop
        ):
            return full_nearest

        n_points = len(path)
        cursor = min(max(0, self.path_cursor_index), n_points - 1)
        candidates = self.path_cursor_candidate_indices(n_points, cursor)
        if not candidates:
            return full_nearest

        best = min(candidates, key=lambda index: (float(path[index].x) - x) ** 2 + (float(path[index].y) - y) ** 2)
        best_distance = math.hypot(float(path[best].x) - x, float(path[best].y) - y)
        if best_distance > self.path_target_reacquire_distance:
            return full_nearest

        if self.track.closed_loop:
            return best

        max_forward_jump = max(1, self.path_target_max_jump_points)
        if best > cursor + max_forward_jump:
            return cursor + max_forward_jump
        return best

    def path_cursor_candidate_indices(self, n_points: int, cursor: int) -> list[int]:
        if n_points <= 0:
            return []
        back = max(0, self.path_target_search_back_points)
        forward = max(1, self.path_target_search_forward_points)
        if self.track.closed_loop:
            return [int((cursor + offset) % n_points) for offset in range(-back, forward + 1)]
        start = max(0, cursor - back)
        end = min(n_points - 1, cursor + forward)
        return list(range(start, end + 1))

    def path_target_after_distance(self, path, start_index: int, lookahead_m: float):
        if not path:
            return None, None
        n_points = len(path)
        index = min(max(0, start_index), n_points - 1)
        travelled = 0.0
        previous = path[index]
        max_steps = n_points if self.track.closed_loop else max(0, n_points - index - 1)
        for step in range(1, max_steps + 1):
            next_index = (index + step) % n_points if self.track.closed_loop else index + step
            if next_index >= n_points:
                break
            current = path[next_index]
            segment = distance_xy(previous, current)
            if segment <= 1e-6:
                previous = current
                continue
            if travelled + segment >= lookahead_m:
                ratio = clamp((lookahead_m - travelled) / segment, 0.0, 1.0)
                return (
                    point(
                        float(previous.x) + (float(current.x) - float(previous.x)) * ratio,
                        float(previous.y) + (float(current.y) - float(previous.y)) * ratio,
                        float(previous.z) + (float(current.z) - float(previous.z)) * ratio,
                    ),
                    next_index,
                )
            travelled += segment
            previous = current
        return path[-1], n_points - 1

    def local_cone_target(self) -> tuple[float, float] | None:
        cones = self.local_safety_cones(min_x=0.8, max_x=14.0)
        if not cones:
            return None

        samples = []
        for start_x in (0.8, 3.5, 6.5, 9.5):
            end_x = start_x + 3.5
            group = [cone for cone in cones if start_x <= cone.position.x < end_x]
            if not group:
                continue
            left_real = [cone.position.y for cone in group if int(cone.color) == ConeColor.BLUE]
            right_real = [cone.position.y for cone in group if int(cone.color) == ConeColor.YELLOW]
            left_unknown = [
                cone.position.y
                for cone in group
                if int(cone.color) == ConeColor.UNKNOWN and self.local_cone_side(cone) == ConeColor.BLUE
            ]
            right_unknown = [
                cone.position.y
                for cone in group
                if int(cone.color) == ConeColor.UNKNOWN and self.local_cone_side(cone) == ConeColor.YELLOW
            ]
            left = left_real if left_real else left_unknown
            right = right_real if right_real else right_unknown
            if left and right:
                center_y = 0.5 * (median(left) + median(right))
            elif left:
                center_y = median(left) - self.track_half_width
            elif right:
                center_y = median(right) + self.track_half_width
            else:
                continue
            side_group = [cone for cone in group if self.local_cone_side(cone) in (ConeColor.BLUE, ConeColor.YELLOW)]
            center_x = median([cone.position.x for cone in side_group]) if side_group else median([cone.position.x for cone in group])
            samples.append((float(center_x), float(center_y)))

        if not samples:
            return None

        desired_x = min(self.drive_lookahead(), max(2.5, samples[-1][0]))
        target_x, target_y = min(samples, key=lambda sample: abs(sample[0] - desired_x))
        target_y += self.effective_path_offset() + self.boundary_guard_shift_y()
        target_y = self.cone_clearance_limited_target_y(
            target_x,
            target_y,
            cones,
            lateral_limit=max(self.track_half_width, self.local_target_lateral_limit),
        )
        self.current_target_source = "local_cones"
        return target_x, target_y

    def local_safety_cones(self, min_x: float = 0.25, max_x: float = 14.0):
        return [
            cone
            for cone in self.local_cones.cones
            if min_x < float(cone.position.x) < max_x
            and abs(float(cone.position.y)) < 7.0
            and float(cone.confidence) >= self.local_cone_min_confidence
            and is_reliable_geometry_source(cone.source)
        ]

    def local_cone_side(self, cone) -> int | None:
        color = int(cone.color)
        if color == ConeColor.BLUE:
            return ConeColor.BLUE
        if color == ConeColor.YELLOW:
            return ConeColor.YELLOW
        if color == ConeColor.UNKNOWN:
            lateral = float(cone.position.y)
            min_abs_y = max(0.25, self.local_unknown_boundary_min_abs_y)
            max_abs_y = max(min_abs_y, self.local_unknown_boundary_max_abs_y)
            if abs(lateral) > max_abs_y:
                return None
            if lateral >= min_abs_y:
                return ConeColor.BLUE
            if lateral <= -min_abs_y:
                return ConeColor.YELLOW
        return None

    @staticmethod
    def nearest_point(points, x: float, y: float):
        if not points:
            return None
        return min(points, key=lambda p: (float(p.x) - x) ** 2 + (float(p.y) - y) ** 2)

    def boundary_guard_shift_y(self) -> float:
        clearances = self.boundary_clearances()
        if clearances is None:
            return 0.0
        yellow_clearance, blue_clearance, width, unit_x, unit_y = clearances
        if self.pose is None:
            return 0.0
        pose = pose2_from_pose_stamped(self.pose)

        shift_x = 0.0
        shift_y = 0.0
        margin = min(self.boundary_guard_margin, max(0.0, 0.48 * width))
        if yellow_clearance < margin:
            amount = min(self.boundary_guard_max_shift, (margin - yellow_clearance) * self.boundary_guard_gain)
            shift_x += unit_x * amount
            shift_y += unit_y * amount
        if blue_clearance < margin:
            amount = min(self.boundary_guard_max_shift, (margin - blue_clearance) * self.boundary_guard_gain)
            shift_x -= unit_x * amount
            shift_y -= unit_y * amount

        local_shift_y = -math.sin(pose.yaw) * shift_x + math.cos(pose.yaw) * shift_y
        return clamp(local_shift_y, -self.boundary_guard_max_shift, self.boundary_guard_max_shift)

    def boundary_clearances(self) -> tuple[float, float, float, float, float] | None:
        if not self.boundary_guard_enabled or self.pose is None:
            return None
        if not self.track.centerline or not self.track.blue_boundary_line or not self.track.yellow_boundary_line:
            return None

        pose = pose2_from_pose_stamped(self.pose)
        closed_loop = bool(self.track.closed_loop)
        projection = closest_point_on_polyline_with_index(self.track.centerline, pose.x, pose.y, closed_loop=closed_loop)
        center_segments = path_segment_count(self.track.centerline, closed_loop=closed_loop)
        if projection is None or center_segments <= 0:
            return None
        _, _, _, center_index = projection
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
        if width < self.boundary_guard_min_width or width > self.boundary_guard_max_width:
            return None

        wx = pose.x - float(yellow.x)
        wy = pose.y - float(yellow.y)
        lateral_fraction = (wx * vx + wy * vy) / max(1e-6, width * width)
        yellow_clearance = lateral_fraction * width
        blue_clearance = (1.0 - lateral_fraction) * width
        return yellow_clearance, blue_clearance, width, vx / width, vy / width

    def drive_lookahead(self) -> float:
        if self.track.closed_loop and not self.live_closed_loop_learning_map():
            min_lookahead = max(1.0, self.racing_lookahead)
            max_lookahead = max(min_lookahead, self.racing_max_lookahead)
        else:
            min_lookahead = max(1.0, self.centerline_lookahead)
            max_lookahead = max(min_lookahead, self.centerline_max_lookahead)
        preview_distance = max(0.0, self.speed) * max(0.0, self.steering_preview_time)
        return clamp(max(min_lookahead, preview_distance), min_lookahead, max_lookahead)

    def cone_clearance_limited_target_y(
        self,
        target_x: float,
        target_y: float,
        cones,
        lateral_limit: float | None = None,
    ) -> float:
        adjusted_y = float(target_y)
        clearance = max(0.2, self.local_target_cone_clearance)
        for _ in range(4):
            moved = False
            for cone in cones:
                if abs(float(cone.position.x) - target_x) > 3.5:
                    continue
                dx = target_x - float(cone.position.x)
                dy = adjusted_y - float(cone.position.y)
                distance = math.hypot(dx, dy)
                if distance >= clearance:
                    continue
                if abs(dy) < 1e-6:
                    direction = -1.0 if float(cone.position.y) >= 0.0 else 1.0
                else:
                    direction = 1.0 if dy > 0.0 else -1.0
                adjusted_y += direction * (clearance - distance)
                moved = True
            if not moved:
                break
        limit = max(0.2, float(lateral_limit if lateral_limit is not None else self.track_half_width))
        return clamp(adjusted_y, -limit, limit)

    def steering_command(self) -> float:
        target: tuple[float, float] | None = None
        target_source = ""
        if not self.track.closed_loop:
            local_target = self.local_cone_target()
            edge_target = self.edge_recovery_target()
            edge_override = edge_target is not None and (
                local_target is None or self.edge_recovery_should_override()
            )
            if edge_override:
                target = edge_target
                target_source = "edge_recovery"
            elif self.prefer_centerline_until_closed_loop:
                target = self.global_path_target_local()
                target_source = self.current_target_source if target is not None else ""
                if target is None and local_target is not None:
                    target = local_target
                    target_source = "local_cones"
                if target is None and edge_target is not None:
                    target = edge_target
                    target_source = "edge_recovery"
            elif self.prefer_local_cones_until_closed_loop:
                if local_target is not None:
                    target = local_target
                    target_source = "local_cones"
                if target is None and edge_target is not None:
                    target = edge_target
                    target_source = "edge_recovery"
                if target is None:
                    target = self.global_path_target_local()
                    target_source = self.current_target_source if target is not None else ""
            else:
                target = self.global_path_target_local()
                target_source = self.current_target_source if target is not None else ""
                if target is None and local_target is not None:
                    target = local_target
                    target_source = "local_cones"
                if target is None and edge_target is not None:
                    target = edge_target
                    target_source = "edge_recovery"
        else:
            edge_target = self.edge_recovery_target()
            if edge_target is not None and self.edge_recovery_should_override():
                target = edge_target
                target_source = "edge_recovery"
            if target is None:
                target = self.global_path_target_local()
                target_source = self.current_target_source if target is not None else ""
            if target is None:
                target = self.local_cone_target()
                target_source = "local_cones" if target is not None else ""
            if target is None and edge_target is not None:
                target = edge_target
                target_source = "edge_recovery"
        if target is None:
            self.last_target_local_x = None
            self.last_target_local_y_final = None
            self.reset_target_filter()
            return self.rate_limited_steering(0.0)
        self.current_target_source = target_source
        local_x, local_y = self.filtered_target_local(self.current_target_source, target[0], target[1])
        self.last_target_local_x = local_x
        self.last_target_local_y_final = local_y
        desired = self.pure_pursuit_steering(local_x, local_y)
        return self.rate_limited_steering(desired)

    def filtered_target_local(self, source: str, local_x: float, local_y: float) -> tuple[float, float]:
        if not self.target_lateral_filter_enabled:
            return local_x, local_y
        if source != self.last_target_source or self.last_target_local_y is None:
            self.last_target_source = source
            self.last_target_local_y = float(local_y)
            return local_x, local_y

        alpha = clamp(self.target_lateral_filter_alpha, 0.0, 1.0)
        raw_y = float(local_y)
        previous_y = float(self.last_target_local_y)
        filtered_y = previous_y + alpha * (raw_y - previous_y)
        max_step = max(0.0, self.target_lateral_filter_max_step)
        if max_step > 0.0:
            filtered_y = clamp(filtered_y, previous_y - max_step, previous_y + max_step)
        self.last_target_source = source
        self.last_target_local_y = filtered_y
        return local_x, filtered_y

    def edge_recovery_target(self) -> tuple[float, float] | None:
        if not self.edge_recovery_enabled:
            return None
        clearances = self.boundary_clearances()
        if clearances is None:
            return None
        yellow_clearance, blue_clearance, width, _, _ = clearances
        min_clearance = min(yellow_clearance, blue_clearance)
        threshold = self.closed_loop_edge_recovery_clearance if self.track.closed_loop else self.edge_recovery_clearance
        if min_clearance >= threshold:
            return None
        limit = max(self.track_half_width, self.local_target_lateral_limit)
        target_y = limit if yellow_clearance <= blue_clearance else -limit
        target_x = max(1.2, self.edge_recovery_target_x)
        return target_x, target_y

    def edge_recovery_should_override(self) -> bool:
        clearances = self.boundary_clearances()
        if clearances is None:
            return False
        yellow_clearance, blue_clearance, _, _, _ = clearances
        threshold = (
            self.closed_loop_edge_recovery_clearance
            if self.track.closed_loop
            else self.open_boundary_guard_critical_margin
        )
        return min(yellow_clearance, blue_clearance) < threshold

    def reset_target_filter(self) -> None:
        self.current_target_source = ""
        self.last_target_source = ""
        self.last_target_local_y = None

    def pure_pursuit_steering(self, local_x: float, local_y: float) -> float:
        forward = max(0.1, float(local_x))
        lateral = float(local_y)
        lookahead_sq = max(0.25, forward * forward + lateral * lateral)
        curvature = 2.0 * lateral / lookahead_sq
        wheel_angle = math.atan(max(0.1, self.wheelbase) * curvature)
        # FSDS steering convention: negative is left, positive is right.
        return clamp(-wheel_angle / max(0.05, self.max_steering_angle), -1.0, 1.0)

    def rate_limited_steering(self, desired: float) -> float:
        now = self.get_clock().now()
        dt = (now - self.last_steering_time).nanoseconds * 1e-9
        dt = clamp(dt, 0.001, 0.20)
        max_step = max(0.0, self.steering_rate_limit) * dt
        limited = clamp(float(desired), self.last_steering - max_step, self.last_steering + max_step)
        self.last_steering = clamp(limited, -1.0, 1.0)
        self.last_steering_time = now
        return self.last_steering

    def close_cone_in_path(self) -> bool:
        if not self.close_cone_brake_enabled:
            return False
        for cone in self.local_cones.cones:
            if cone.confidence < self.local_cone_min_confidence:
                continue
            if not is_reliable_geometry_source(cone.source):
                continue
            cone_x = float(cone.position.x)
            cone_y = abs(float(cone.position.y))
            hard_stop = cone_x < self.close_cone_hard_stop_distance and cone_y < self.close_cone_hard_stop_lateral
            moving_stop = (
                abs(float(self.speed)) >= self.close_cone_brake_min_speed
                and cone_x < self.close_cone_brake_distance
                and cone_y < self.close_cone_brake_lateral
            )
            if 0.25 < cone_x and (hard_stop or moving_stop):
                return True
        return False

    def deterministic_speed_command(self, target_speed: float) -> tuple[float, float, str]:
        speed_error = target_speed - self.speed
        if target_speed <= 0.1:
            return 0.0, 0.4, "zero_target"
        if speed_error >= 0.0:
            throttle = clamp(self.throttle_kp * speed_error, 0.0, self.max_throttle)
            if target_speed > 0.4 and self.speed < self.launch_speed_threshold:
                throttle = max(throttle, min(self.launch_throttle, self.max_throttle))
            return throttle, 0.0, "accelerating"
        return 0.0, clamp(-self.brake_kp * speed_error, 0.0, self.max_brake), "speed_trim"

    def apply_rl_direct_blend(self, cmd: ControlCommand) -> bool:
        blend = self.effective_rl_direct_blend()
        self.last_rl_direct_blend_applied = blend
        if blend <= 0.0:
            return False
        cmd.steering = clamp(
            (1.0 - blend) * float(cmd.steering) + blend * float(self.rl_direct_command.steering),
            -1.0,
            1.0,
        )
        cmd.throttle = clamp(
            (1.0 - blend) * float(cmd.throttle) + blend * float(self.rl_direct_command.throttle),
            0.0,
            1.0,
        )
        cmd.brake = clamp(
            (1.0 - blend) * float(cmd.brake) + blend * float(self.rl_direct_command.brake),
            0.0,
            1.0,
        )
        return True

    def apply_safety_overrides(
        self,
        cmd: ControlCommand,
        target_speed: float,
        close_cone: bool,
        boundary_tight: bool,
        boundary_brake: bool,
    ) -> str | None:
        if not self.rl_safety_overrides_enabled:
            return None
        if self.race_state.emergency_brake or not self.race_state.go_signal_fresh:
            cmd.throttle = 0.0
            cmd.brake = 1.0 if self.speed > 0.4 else 0.2
            return "emergency_or_no_go"
        if boundary_brake and self.speed > 0.35:
            cmd.throttle = 0.0
            cmd.brake = 1.0 if self.speed > 0.6 else 0.4
            return "closed_loop_boundary"
        if close_cone:
            cmd.throttle = 0.0
            cmd.brake = 1.0 if self.speed > 0.3 else 0.4
            return "close_cone"
        if target_speed <= 0.1:
            cmd.throttle = 0.0
            cmd.brake = max(float(cmd.brake), 0.4)
            return "zero_target"
        if boundary_tight and self.speed > target_speed + 0.2:
            cmd.throttle = 0.0
            cmd.brake = max(float(cmd.brake), clamp((self.speed - target_speed) * self.brake_kp, 0.0, self.max_brake))
            return "boundary_speed_governor"
        return None

    def publish_control(self) -> None:
        cmd = ControlCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "fsds/FSCar"

        steering = self.steering_command()
        target_speed = max(0.0, self.target_speed)
        if self.rl_action_fresh():
            target_speed = max(0.0, target_speed + self.effective_speed_delta())

        close_cone = self.close_cone_in_path()
        self.last_close_cone = close_cone
        boundary_clearances = self.boundary_clearances()
        boundary_min_clearance = min(boundary_clearances[0], boundary_clearances[1]) if boundary_clearances else float("inf")
        self.last_boundary_clearances = (
            (boundary_clearances[0], boundary_clearances[1], boundary_clearances[2])
            if boundary_clearances
            else None
        )
        boundary_tight = False
        if self.track.closed_loop and boundary_min_clearance < self.boundary_guard_slow_margin:
            boundary_tight = True
            target_speed = min(target_speed, max(0.0, self.boundary_guard_slow_speed))
        elif not self.track.closed_loop and boundary_min_clearance < self.open_boundary_guard_slow_margin:
            boundary_tight = True
            open_speed = self.open_boundary_guard_slow_speed
            if boundary_min_clearance < self.open_boundary_guard_critical_margin:
                open_speed = min(open_speed, self.open_boundary_guard_critical_speed)
            target_speed = min(target_speed, max(0.0, open_speed))
        if self.track.closed_loop and self.live_closed_loop_learning_map():
            target_speed = min(target_speed, max(0.0, self.closed_loop_live_map_speed))
        if self.track.closed_loop and self.current_target_source in ("local_cones", "edge_recovery", ""):
            target_speed = min(target_speed, max(0.0, self.closed_loop_local_fallback_speed))
        if self.track.closed_loop and self.current_target_source == "edge_recovery":
            target_speed = min(target_speed, max(0.0, self.closed_loop_edge_recovery_speed))
        speed_error = target_speed - self.speed
        boundary_brake = (
            bool(self.track.closed_loop)
            and self.current_target_source != "edge_recovery"
            and boundary_min_clearance < self.boundary_guard_brake_margin
        )

        throttle, brake, base_reason = self.deterministic_speed_command(target_speed)
        cmd.steering = float(steering)
        cmd.throttle = float(throttle)
        cmd.brake = float(brake)
        if self.apply_rl_direct_blend(cmd):
            self.last_brake_reason = f"rl_direct_blend={self.last_rl_direct_blend_applied:.2f};base={base_reason}"
        else:
            self.last_brake_reason = base_reason
        safety_reason = self.apply_safety_overrides(cmd, target_speed, close_cone, boundary_tight, boundary_brake)
        if safety_reason is not None:
            self.last_brake_reason = safety_reason
        cmd.steering = clamp(float(cmd.steering), -1.0, 1.0)
        cmd.throttle = clamp(float(cmd.throttle), 0.0, 1.0)
        cmd.brake = clamp(float(cmd.brake), 0.0, 1.0)
        self.control_pub.publish(cmd)
        self.publish_diagnostics(cmd, target_speed, boundary_min_clearance, boundary_tight)

    def publish_diagnostics(
        self,
        cmd: ControlCommand,
        target_speed: float,
        boundary_min_clearance: float,
        boundary_tight: bool,
    ) -> None:
        boundary = "none"
        if self.last_boundary_clearances is not None:
            yellow_clearance, blue_clearance, width = self.last_boundary_clearances
            boundary = f"yellow={yellow_clearance:.2f} blue={blue_clearance:.2f} width={width:.2f}"
        target = "target=none"
        if self.last_target_local_x is not None and self.last_target_local_y_final is not None:
            target = (
                f"source={self.last_target_source or self.current_target_source} "
                f"x={self.last_target_local_x:.2f} y={self.last_target_local_y_final:.2f}"
            )
        msg = String()
        msg.data = (
            f"{target} steering={cmd.steering:.2f} throttle={cmd.throttle:.2f} brake={cmd.brake:.2f} "
            f"speed={self.speed:.2f} target_speed={target_speed:.2f} reason={self.last_brake_reason} "
            f"closed_loop={bool(self.track.closed_loop)} path_error={self.last_path_error:.2f} "
            f"close_cone={self.last_close_cone} boundary_tight={boundary_tight} "
            f"boundary_min={boundary_min_clearance:.2f} boundary={boundary} "
            f"rl_blend={self.last_rl_direct_blend_applied:.2f} rl_direct_fresh={self.rl_direct_fresh()}"
        )
        self.diagnostics_pub.publish(msg)


def main() -> None:
    rclpy.init()
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

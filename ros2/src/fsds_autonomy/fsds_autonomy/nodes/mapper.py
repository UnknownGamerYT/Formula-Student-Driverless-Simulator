from __future__ import annotations

from datetime import datetime, timezone
import math
from pathlib import Path
import shutil

import rclpy
from fsds_autonomy_msgs.msg import ConeArray, RaceState, TrackMap
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from fsds_autonomy.constants import ConeColor, STEREO_CAMERA_SOURCE, is_reliable_geometry_source, side_color_from_local_y
from fsds_autonomy.geometry import pose2_from_pose_stamped, speed_profile_for_path, transform_local_to_global
from fsds_autonomy.map_store import (
    ConeLandmark,
    SavedTrackMap,
    is_usable_track_map,
    load_track_map,
    load_reset_events,
    map_path,
    reset_events_path,
    save_track_map,
    track_map_sanity_reasons,
)
from fsds_autonomy.planning import (
    MIN_RACING_CONE_CLEARANCE_M,
    RACING_CAR_WIDTH_M,
    RACING_CLEARANCE_MARGIN_M,
    RACING_CONE_WIDTH_M,
    build_boundary_lines_from_cones,
    build_centerline_from_cones,
    build_racing_line,
    dedupe_cone_landmarks,
    infer_map_quality,
)
from fsds_autonomy.ros_utils import track_map_to_msg


class Mapper(Node):
    def __init__(self) -> None:
        super().__init__("fsds_mapper")
        self.declare_parameter("map_dir", "maps")
        self.declare_parameter("association_radius_m", 1.20)
        self.declare_parameter("save_period_sec", 5.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("update_loaded_map_in_race_from_map", False)
        self.declare_parameter("save_live_updates_in_race_from_map", False)
        self.declare_parameter("seed_landmarks_from_saved_map", True)
        self.declare_parameter("min_cone_confidence", 0.50)
        self.declare_parameter("min_landmark_observations", 3)
        self.declare_parameter("min_landmark_persistence_sec", 0.20)
        self.declare_parameter("tentative_landmark_timeout_sec", 1.00)
        self.declare_parameter("tentative_landmark_max_missed_frames", 10)
        self.declare_parameter("mapping_max_range_m", 14.0)
        self.declare_parameter("mapping_max_abs_y_m", 7.0)
        self.declare_parameter("require_stereo_for_mapping", True)
        self.declare_parameter("require_lidar_stereo_for_mapping", True)
        self.declare_parameter("allow_lidar_side_color_fallback", False)
        self.declare_parameter("merge_landmark_distance_m", 1.20)
        self.declare_parameter("map_refinement_enabled", True)
        self.declare_parameter("refinement_match_radius_m", 1.00)
        self.declare_parameter("refinement_position_alpha", 0.04)
        self.declare_parameter("refinement_max_position_step_m", 0.10)
        self.declare_parameter("refinement_min_changes_to_save", 4)
        self.declare_parameter("refinement_new_cone_min_observations", 10)
        self.declare_parameter("refinement_new_cone_min_persistence_sec", 2.0)
        self.declare_parameter("refinement_new_cone_min_separation_m", 1.20)
        self.declare_parameter("refinement_max_new_cones_per_save", 4)
        self.declare_parameter("refinement_max_quality_drop", 0.04)
        self.declare_parameter("refinement_save_min_interval_sec", 20.0)
        self.declare_parameter("refinement_requires_loop_completed", True)
        self.declare_parameter("loop_completion_min_travel_m", 120.0)
        self.declare_parameter("loop_completion_gate_radius_m", 4.0)
        self.declare_parameter("loop_completion_max_pose_step_m", 6.0)
        self.declare_parameter("reset_event_avoidance_enabled", True)
        self.declare_parameter("reset_event_avoidance_influence_radius_m", 7.0)
        self.declare_parameter("reset_event_avoidance_max_shift_m", 0.80)
        self.declare_parameter("reset_event_avoidance_max_events", 120)
        self.declare_parameter("reset_event_avoidance_min_line_change_m", 0.05)

        self.map_dir = Path(str(self.get_parameter("map_dir").value)).expanduser()
        self.association_radius = float(self.get_parameter("association_radius_m").value)
        self.update_loaded_map_in_race_from_map = bool(
            self.get_parameter("update_loaded_map_in_race_from_map").value
        )
        self.save_live_updates_in_race_from_map = bool(
            self.get_parameter("save_live_updates_in_race_from_map").value
        )
        self.seed_landmarks_from_saved_map = bool(self.get_parameter("seed_landmarks_from_saved_map").value)
        self.min_cone_confidence = float(self.get_parameter("min_cone_confidence").value)
        self.min_landmark_observations = int(self.get_parameter("min_landmark_observations").value)
        self.min_landmark_persistence_sec = float(
            self.get_parameter("min_landmark_persistence_sec").value
        )
        self.tentative_landmark_timeout_sec = float(
            self.get_parameter("tentative_landmark_timeout_sec").value
        )
        self.tentative_landmark_max_missed_frames = int(
            self.get_parameter("tentative_landmark_max_missed_frames").value
        )
        self.mapping_max_range = float(self.get_parameter("mapping_max_range_m").value)
        self.mapping_max_abs_y = float(self.get_parameter("mapping_max_abs_y_m").value)
        self.require_stereo_for_mapping = bool(self.get_parameter("require_stereo_for_mapping").value)
        self.require_lidar_stereo_for_mapping = bool(
            self.get_parameter("require_lidar_stereo_for_mapping").value
        )
        self.allow_lidar_side_color_fallback = bool(
            self.get_parameter("allow_lidar_side_color_fallback").value
        )
        self.merge_landmark_distance = float(self.get_parameter("merge_landmark_distance_m").value)
        self.map_refinement_enabled = bool(self.get_parameter("map_refinement_enabled").value)
        self.refinement_match_radius = float(self.get_parameter("refinement_match_radius_m").value)
        self.refinement_position_alpha = float(self.get_parameter("refinement_position_alpha").value)
        self.refinement_max_position_step = float(
            self.get_parameter("refinement_max_position_step_m").value
        )
        self.refinement_min_changes_to_save = int(
            self.get_parameter("refinement_min_changes_to_save").value
        )
        self.refinement_new_cone_min_observations = int(
            self.get_parameter("refinement_new_cone_min_observations").value
        )
        self.refinement_new_cone_min_persistence_sec = float(
            self.get_parameter("refinement_new_cone_min_persistence_sec").value
        )
        self.refinement_new_cone_min_separation = float(
            self.get_parameter("refinement_new_cone_min_separation_m").value
        )
        self.refinement_max_new_cones_per_save = int(
            self.get_parameter("refinement_max_new_cones_per_save").value
        )
        self.refinement_max_quality_drop = float(
            self.get_parameter("refinement_max_quality_drop").value
        )
        self.refinement_save_min_interval_sec = float(
            self.get_parameter("refinement_save_min_interval_sec").value
        )
        self.refinement_requires_loop_completed = bool(
            self.get_parameter("refinement_requires_loop_completed").value
        )
        self.loop_completion_min_travel = float(self.get_parameter("loop_completion_min_travel_m").value)
        self.loop_completion_gate_radius = float(self.get_parameter("loop_completion_gate_radius_m").value)
        self.loop_completion_max_pose_step = float(self.get_parameter("loop_completion_max_pose_step_m").value)
        self.reset_event_avoidance_enabled = bool(
            self.get_parameter("reset_event_avoidance_enabled").value
        )
        self.reset_event_avoidance_influence_radius = float(
            self.get_parameter("reset_event_avoidance_influence_radius_m").value
        )
        self.reset_event_avoidance_max_shift = float(
            self.get_parameter("reset_event_avoidance_max_shift_m").value
        )
        self.reset_event_avoidance_max_events = int(
            self.get_parameter("reset_event_avoidance_max_events").value
        )
        self.reset_event_avoidance_min_line_change = float(
            self.get_parameter("reset_event_avoidance_min_line_change_m").value
        )
        self.track_id = "unknown"
        self.mode = "waiting_for_go"
        self.pose = None
        self.landmarks: list[ConeLandmark] = []
        self.loaded_track_map: SavedTrackMap | None = None
        self.reference_track_map: SavedTrackMap | None = None
        self.loaded_once = False
        self.saved_map_backed_up = False
        self.frame_index = 0
        self.refined_observation_counts: dict[int, int] = {}
        self.refinement_version = 0
        self.last_refinement_saved_changes = 0
        self.last_refinement_saved_added = 0
        self.last_refinement_rejected_reason = ""
        self.last_refinement_save_sec = 0.0
        self.loop_completed = False
        self.loop_completion_status = "loop=waiting_for_orange"
        self.start_gate_x: float | None = None
        self.start_gate_y: float | None = None
        self.start_gate_observations = 0
        self.loop_distance_travelled = 0.0
        self.last_loop_pose = None
        self.reset_event_cache_track_id = ""
        self.reset_event_cache_mtime = -1.0
        self.reset_event_cache: list[dict] = []
        self.reset_event_line_update_marker: tuple[float, int] | None = None

        self.map_pub = self.create_publisher(TrackMap, "/autonomy/track_map", 10)
        self.live_map_pub = self.create_publisher(TrackMap, "/autonomy/live_track_map", 10)
        self.reference_map_pub = self.create_publisher(TrackMap, "/autonomy/reference_track_map", 10)
        self.diagnostics_pub = self.create_publisher(String, "/autonomy/mapper_diagnostics", 10)
        self.create_subscription(PoseStamped, "/autonomy/pose", self.on_pose, 10)
        self.create_subscription(ConeArray, "/autonomy/fused_cones", self.on_cones, 10)
        self.create_subscription(RaceState, "/autonomy/mission_state", self.on_mission, 10)

        publish_period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(publish_period, self.publish_map)
        self.create_timer(float(self.get_parameter("save_period_sec").value), self.save_if_ready)

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose = pose2_from_pose_stamped(msg)
        self.update_loop_distance()

    def on_mission(self, msg: RaceState) -> None:
        if msg.track_id and msg.track_id != self.track_id:
            self.track_id = msg.track_id
            self.landmarks = []
            self.loaded_track_map = None
            self.reference_track_map = None
            self.loaded_once = False
            self.saved_map_backed_up = False
            self.refined_observation_counts = {}
            self.refinement_version = 0
            self.last_refinement_saved_changes = 0
            self.last_refinement_saved_added = 0
            self.last_refinement_rejected_reason = ""
            self.last_refinement_save_sec = 0.0
            self.reset_loop_tracking()
            self.reset_event_cache_track_id = ""
            self.reset_event_cache_mtime = -1.0
            self.reset_event_cache = []
            self.reset_event_line_update_marker = None
        self.mode = msg.mode
        if not self.loaded_once:
            saved = load_track_map(self.map_dir, self.track_id)
            if saved:
                filtered_saved = self.filtered_track_map(saved)
            else:
                filtered_saved = None
            if filtered_saved and is_usable_track_map(filtered_saved, require_closed_loop=True):
                self.loaded_track_map = self.clone_track_map(filtered_saved)
                self.reference_track_map = self.clone_track_map(filtered_saved)
                self.refinement_version = int(filtered_saved.metadata.get("refinement_version", 0))
                self.initialize_start_gate_from_map(filtered_saved)
                self.loop_completed = bool(filtered_saved.metadata.get("loop_completed_after_orange_gate", False))
                if self.loop_completed:
                    self.loop_completion_status = "loop=loaded_verified"
                if self.seed_landmarks_from_saved_map:
                    self.landmarks = [self.clone_landmark(cone) for cone in filtered_saved.cones]
                self.get_logger().info(
                    f"Loaded saved map for {self.track_id}: "
                    f"reference_cones={len(filtered_saved.cones)} seeded_live_cones={len(self.landmarks)}"
                )
            self.loaded_once = True

    def on_cones(self, msg: ConeArray) -> None:
        if self.pose is None or self.mode == "waiting_for_go":
            return
        if (
            self.mode == "race_from_map"
            and self.loaded_track_map is not None
            and not self.update_loaded_map_in_race_from_map
        ):
            return
        self.frame_index += 1
        now_sec = self.get_clock().now().nanoseconds / 1e9
        seen_landmark_ids: set[int] = set()
        for cone in msg.cones:
            if cone.range > 25.0 or cone.confidence < self.min_cone_confidence:
                continue
            if cone.range > self.mapping_max_range or abs(cone.position.y) > self.mapping_max_abs_y:
                continue
            if not self.is_mapping_source_allowed(cone.source):
                continue
            color = int(cone.color)
            if color == ConeColor.UNKNOWN:
                if not self.allow_lidar_side_color_fallback:
                    continue
                color = side_color_from_local_y(cone.position.y)
                if color == ConeColor.UNKNOWN:
                    continue
            x, y = transform_local_to_global(cone.position.x, cone.position.y, self.pose)
            if color in (ConeColor.ORANGE, ConeColor.LARGE_ORANGE):
                self.observe_start_gate(x, y, now_sec)
            landmark = self.upsert_landmark(x, y, cone.position.z, color, cone.confidence, now_sec)
            seen_landmark_ids.add(id(landmark))
        self.prune_tentative_landmarks(seen_landmark_ids, now_sec)

    def upsert_landmark(
        self,
        x: float,
        y: float,
        z: float,
        color: int,
        confidence: float,
        now_sec: float,
    ) -> ConeLandmark:
        best = None
        best_dist = self.association_radius
        for landmark in self.landmarks:
            if not self.colors_match_for_association(int(landmark.color), int(color)):
                continue
            dist = ((landmark.x - x) ** 2 + (landmark.y - y) ** 2) ** 0.5
            if dist < best_dist:
                best = landmark
                best_dist = dist
        if best is None:
            landmark = ConeLandmark(
                x=x,
                y=y,
                z=z,
                color=int(color),
                confidence=float(confidence),
                observations=1,
                first_seen_sec=float(now_sec),
                last_seen_sec=float(now_sec),
                last_seen_frame=int(self.frame_index),
                hit_streak=1,
                missed_frames=0,
            )
            self.landmarks.append(landmark)
            return landmark
        same_frame_seen = int(best.last_seen_frame) == int(self.frame_index)
        n = max(1, best.observations)
        best.x = (best.x * n + x) / (n + 1)
        best.y = (best.y * n + y) / (n + 1)
        best.z = (best.z * n + z) / (n + 1)
        if color != 4 and confidence >= best.confidence:
            best.color = int(color)
        best.confidence = max(best.confidence, float(confidence))
        if not same_frame_seen:
            best.observations += 1
            best.hit_streak = max(0, int(best.hit_streak)) + 1
        best.last_seen_sec = float(now_sec)
        best.last_seen_frame = int(self.frame_index)
        best.missed_frames = 0
        if best.first_seen_sec <= 0.0:
            best.first_seen_sec = float(now_sec)
        return best

    def current_track_map(self) -> SavedTrackMap:
        if (
            self.mode == "race_from_map"
            and self.loaded_track_map is not None
            and not self.update_loaded_map_in_race_from_map
        ):
            return self.loaded_track_map
        landmarks = self.calculation_landmarks()
        live_map = self.track_map_from_landmarks(
            landmarks,
            metadata_extra={
                "mode": self.mode,
                "tentative_landmarks": self.tentative_landmark_count(),
                "confirmed_landmarks": self.confirmed_landmark_count(),
            },
        )
        if (
            self.mode == "race_from_map"
            and self.loaded_track_map is not None
            and not is_usable_track_map(live_map, require_closed_loop=True)
        ):
            return self.loaded_track_map
        return live_map

    def track_map_from_landmarks(
        self,
        landmarks: list[ConeLandmark],
        metadata_extra: dict | None = None,
    ) -> SavedTrackMap:
        centerline = build_centerline_from_cones(landmarks)
        blue_boundary, yellow_boundary = build_boundary_lines_from_cones(landmarks, centerline)
        reset_events = self.reset_events_for_planning()
        racing_line, speed_profile = build_racing_line(
            centerline,
            landmarks,
            reset_events=reset_events,
            reset_event_influence_radius_m=self.reset_event_avoidance_influence_radius,
            reset_event_max_shift_m=self.reset_event_avoidance_max_shift,
        )
        racing_line_fallback = ""
        if not self.racing_line_usable_for_map(racing_line, centerline):
            racing_line = list(centerline)
            speed_profile = speed_profile_for_path(racing_line, min_speed=2.0, max_speed=5.0, curvature_gain=24.0)
            racing_line_fallback = "centerline_full_loop_due_discontinuous_generated_line"
        quality = infer_map_quality(landmarks, centerline)
        closed_loop = self.loop_map_ready(centerline, quality)
        if closed_loop:
            blue_boundary = self.closed_loop_points(blue_boundary, max_gap_m=10.0)
            yellow_boundary = self.closed_loop_points(yellow_boundary, max_gap_m=10.0)
            centerline = self.closed_loop_points(centerline, max_gap_m=18.0)
            racing_line_was = len(racing_line)
            racing_line = self.closed_loop_points(racing_line, max_gap_m=18.0)
            if speed_profile and len(racing_line) > racing_line_was:
                speed_profile = list(speed_profile) + [float(speed_profile[0])]
        metadata = {
            "mode": self.mode,
            "min_cone_confidence": self.min_cone_confidence,
            "min_landmark_observations": self.min_landmark_observations,
            "min_landmark_persistence_sec": self.min_landmark_persistence_sec,
            "tentative_landmark_timeout_sec": self.tentative_landmark_timeout_sec,
            "tentative_landmark_max_missed_frames": self.tentative_landmark_max_missed_frames,
            "mapping_max_range_m": self.mapping_max_range,
            "mapping_max_abs_y_m": self.mapping_max_abs_y,
            "require_stereo_for_mapping": self.require_stereo_for_mapping,
            "require_lidar_stereo_for_mapping": self.require_lidar_stereo_for_mapping,
            "allow_lidar_side_color_fallback": self.allow_lidar_side_color_fallback,
            "merge_landmark_distance_m": self.merge_landmark_distance,
            "map_refinement_enabled": self.map_refinement_enabled,
            "refinement_version": self.refinement_version,
            "racing_line_vehicle_width_m": RACING_CAR_WIDTH_M,
            "racing_line_cone_width_m": RACING_CONE_WIDTH_M,
            "racing_line_clearance_margin_m": RACING_CLEARANCE_MARGIN_M,
            "racing_line_min_center_to_cone_m": MIN_RACING_CONE_CLEARANCE_M,
            "racing_line_uses_track_width": True,
            "racing_line_profile": "outside_apex_outside_width_aware",
            "loop_completed_after_orange_gate": self.loop_completed,
            "loop_completion_status": self.loop_completion_status,
            "loop_completion_min_travel_m": self.loop_completion_min_travel,
            "loop_completion_gate_radius_m": self.loop_completion_gate_radius,
            "loop_distance_travelled_m": self.loop_distance_travelled,
            "start_gate_observations": self.start_gate_observations,
            "reset_event_avoidance_enabled": self.reset_event_avoidance_enabled,
            "reset_event_avoidance_events": len(reset_events),
            "reset_event_avoidance_influence_radius_m": self.reset_event_avoidance_influence_radius,
            "reset_event_avoidance_max_shift_m": self.reset_event_avoidance_max_shift,
        }
        if racing_line_fallback:
            metadata["racing_line_fallback"] = racing_line_fallback
        if self.start_gate_x is not None and self.start_gate_y is not None:
            metadata["start_gate_x"] = self.start_gate_x
            metadata["start_gate_y"] = self.start_gate_y
        if metadata_extra:
            metadata.update(metadata_extra)
        return SavedTrackMap(
            track_id=self.track_id,
            closed_loop=closed_loop,
            quality=quality,
            cones=landmarks,
            blue_boundary_line=[(p.x, p.y) for p in blue_boundary],
            yellow_boundary_line=[(p.x, p.y) for p in yellow_boundary],
            centerline=[(p.x, p.y) for p in centerline],
            racing_line=[(p.x, p.y) for p in racing_line],
            speed_profile=speed_profile,
            metadata=metadata,
        )

    def publish_map(self) -> None:
        live_map = self.current_track_map()
        live_msg = track_map_to_msg(self, live_map)
        self.map_pub.publish(live_msg)
        self.live_map_pub.publish(live_msg)
        diagnostics = String()
        diagnostics.data = (
            f"mode={self.mode} total_landmarks={len(self.landmarks)} "
            f"tentative={self.tentative_landmark_count()} confirmed={self.confirmed_landmark_count()} "
            f"published_cones={len(live_map.cones)} quality={live_map.quality:.2f} "
            f"closed_loop={live_map.closed_loop} {self.loop_completion_status} "
            f"loop_distance={self.loop_distance_travelled:.1f}m "
            f"refinement_enabled={self.map_refinement_enabled} refinement_version={self.refinement_version} "
            f"last_refinement_changes={self.last_refinement_saved_changes} "
            f"last_refinement_added={self.last_refinement_saved_added} "
            f"last_refinement_rejected='{self.last_refinement_rejected_reason}'"
        )
        self.diagnostics_pub.publish(diagnostics)
        if self.reference_track_map is not None:
            self.reference_map_pub.publish(track_map_to_msg(self, self.reference_track_map))

    def save_if_ready(self) -> None:
        if self.mode == "waiting_for_go" or len(self.calculation_landmarks()) < 8:
            return
        if (
            self.mode == "race_from_map"
            and self.loaded_track_map is not None
        ):
            self.update_saved_line_from_reset_events_if_ready()
            if self.map_refinement_enabled:
                self.refine_saved_map_if_ready()
            if not self.save_live_updates_in_race_from_map:
                return
        track_map = self.current_track_map()
        if not is_usable_track_map(track_map, require_closed_loop=True):
            reasons = "; ".join(track_map_sanity_reasons(track_map, require_closed_loop=True))
            self.get_logger().warn(
                f"Not saving unfinished/unusable map cones={len(track_map.cones)} centerline={len(track_map.centerline)} "
                f"racing_line={len(track_map.racing_line)} quality={track_map.quality:.2f}; {reasons}",
                throttle_duration_sec=15.0,
            )
            return
        self.backup_saved_map_once()
        path = save_track_map(self.map_dir, track_map)
        if self.reference_track_map is None:
            self.reference_track_map = self.clone_track_map(track_map)
        self.get_logger().info(
            f"Saved map {path} cones={len(track_map.cones)} quality={track_map.quality:.2f}",
            throttle_duration_sec=15.0,
        )

    def reset_events_for_planning(self) -> list[dict]:
        if not self.reset_event_avoidance_enabled or not self.track_id or self.track_id == "unknown":
            return []
        path = reset_events_path(self.map_dir, self.track_id)
        try:
            mtime = path.stat().st_mtime if path.exists() else 0.0
        except OSError:
            mtime = 0.0
        if self.reset_event_cache_track_id == self.track_id and abs(mtime - self.reset_event_cache_mtime) < 1e-9:
            return self.reset_event_cache
        self.reset_event_cache_track_id = self.track_id
        self.reset_event_cache_mtime = mtime
        try:
            self.reset_event_cache = load_reset_events(
                self.map_dir,
                self.track_id,
                max_events=max(1, self.reset_event_avoidance_max_events),
            )
        except (OSError, ValueError) as exc:
            self.get_logger().warn(f"Could not load reset events for line avoidance: {exc}", throttle_duration_sec=5.0)
            self.reset_event_cache = []
        return self.reset_event_cache

    def update_saved_line_from_reset_events_if_ready(self) -> None:
        if not self.reset_event_avoidance_enabled or self.loaded_track_map is None:
            return
        events = self.reset_events_for_planning()
        if not events:
            return
        marker = (self.reset_event_cache_mtime, len(events))
        if self.reset_event_line_update_marker == marker:
            return
        regenerated = self.track_map_from_landmarks(
            [self.clone_landmark(cone) for cone in self.loaded_track_map.cones],
            metadata_extra={
                "mode": "race_from_map_reset_event_line_update",
                "reset_event_line_update_utc": datetime.now(timezone.utc).isoformat(),
                "reset_event_line_update_events": len(events),
                "reset_event_line_update_source": str(reset_events_path(self.map_dir, self.track_id)),
            },
        )
        line_change = self.mean_nearest_line_change(self.loaded_track_map.racing_line, regenerated.racing_line)
        if line_change < self.reset_event_avoidance_min_line_change:
            self.reset_event_line_update_marker = marker
            self.last_refinement_rejected_reason = f"reset_line_change_too_small:{line_change:.2f}m"
            return
        reasons = track_map_sanity_reasons(regenerated)
        if reasons:
            self.reset_event_line_update_marker = marker
            self.last_refinement_rejected_reason = "reset_line_rejected:" + "; ".join(reasons)
            self.get_logger().warn(
                f"Rejected reset-event line update change={line_change:.2f}m; "
                f"{self.last_refinement_rejected_reason}",
                throttle_duration_sec=10.0,
            )
            return
        self.backup_saved_map_once()
        path = save_track_map(self.map_dir, regenerated)
        self.loaded_track_map = self.clone_track_map(regenerated)
        self.reference_track_map = self.clone_track_map(regenerated)
        self.reset_event_line_update_marker = marker
        self.last_refinement_rejected_reason = ""
        self.get_logger().warn(
            f"Saved reset-event adjusted map line {path} events={len(events)} "
            f"mean_change={line_change:.2f}m",
            throttle_duration_sec=5.0,
        )

    @staticmethod
    def mean_nearest_line_change(
        previous_line: list[tuple[float, float]],
        next_line: list[tuple[float, float]],
    ) -> float:
        if not previous_line or not next_line:
            return 0.0
        total = 0.0
        count = 0
        for x, y in next_line:
            nearest = min(math.hypot(float(x) - float(px), float(y) - float(py)) for px, py in previous_line)
            total += nearest
            count += 1
        return total / max(1, count)

    def refine_saved_map_if_ready(self) -> None:
        if self.loaded_track_map is None:
            return
        if self.refinement_requires_loop_completed and not self.loop_completed:
            self.last_refinement_rejected_reason = "waiting_loop_completed"
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (
            self.last_refinement_save_sec > 0.0
            and now_sec - self.last_refinement_save_sec < self.refinement_save_min_interval_sec
        ):
            remaining = self.refinement_save_min_interval_sec - (now_sec - self.last_refinement_save_sec)
            self.last_refinement_rejected_reason = f"waiting_save_interval:{remaining:.1f}s"
            return
        live_landmarks = [
            landmark
            for landmark in self.landmarks
            if float(landmark.confidence) >= self.min_cone_confidence
            and self.is_confirmed_landmark(landmark)
        ]
        if not live_landmarks:
            self.last_refinement_rejected_reason = "no_confirmed_live_landmarks"
            return

        refined_cones = [self.clone_landmark(cone) for cone in self.loaded_track_map.cones]
        updated = 0
        added = 0
        applied_counts: dict[int, int] = {}
        for live_cone in sorted(
            live_landmarks,
            key=lambda cone: (int(cone.observations), float(cone.confidence)),
            reverse=True,
        ):
            cone_id = id(live_cone)
            previous_observations = self.refined_observation_counts.get(cone_id, 0)
            if int(live_cone.observations) <= previous_observations:
                continue
            match = self.nearest_same_color_landmark(
                refined_cones,
                live_cone,
                self.refinement_match_radius,
            )
            if match is not None:
                if self.apply_landmark_refinement(match, live_cone, previous_observations):
                    updated += 1
                    applied_counts[cone_id] = int(live_cone.observations)
                continue
            if added >= self.refinement_max_new_cones_per_save:
                continue
            if not self.live_landmark_is_safe_new_cone(refined_cones, live_cone):
                continue
            refined_cones.append(self.clone_landmark(live_cone))
            added += 1
            applied_counts[cone_id] = int(live_cone.observations)

        changed = updated + added
        if changed < max(1, self.refinement_min_changes_to_save):
            self.last_refinement_rejected_reason = f"not_enough_changes:{changed}"
            return

        refined_cones = dedupe_cone_landmarks(refined_cones, self.merge_landmark_distance)
        next_version = self.refinement_version + 1
        refined_map = self.track_map_from_landmarks(
            [self.clone_landmark(cone) for cone in refined_cones],
            metadata_extra={
                "mode": "race_from_map_refined",
                "refinement_version": next_version,
                "refinement_source": "confirmed_live_landmarks",
                "refinement_updated_cones": updated,
                "refinement_added_cones": added,
                "refinement_live_candidates": len(live_landmarks),
                "refinement_match_radius_m": self.refinement_match_radius,
                "refinement_position_alpha": self.refinement_position_alpha,
                "refinement_max_position_step_m": self.refinement_max_position_step,
                "refinement_new_cone_min_observations": self.refinement_new_cone_min_observations,
            "refinement_new_cone_min_persistence_sec": self.refinement_new_cone_min_persistence_sec,
            "refinement_save_min_interval_sec": self.refinement_save_min_interval_sec,
            "loop_completed_after_orange_gate": self.loop_completed,
            "loop_completion_status": self.loop_completion_status,
        },
        )
        reasons = track_map_sanity_reasons(refined_map)
        previous_quality = float(self.loaded_track_map.quality)
        if refined_map.quality + self.refinement_max_quality_drop < previous_quality:
            reasons.append(
                f"quality drop too large ({refined_map.quality:.2f} < {previous_quality:.2f} - "
                f"{self.refinement_max_quality_drop:.2f})"
            )
        if reasons:
            self.last_refinement_rejected_reason = "; ".join(reasons)
            self.get_logger().warn(
                f"Rejected refined map changes={changed} updated={updated} added={added}; "
                f"{self.last_refinement_rejected_reason}",
                throttle_duration_sec=15.0,
            )
            return

        self.backup_saved_map_once()
        path = save_track_map(self.map_dir, refined_map)
        self.loaded_track_map = self.clone_track_map(refined_map)
        self.reference_track_map = self.clone_track_map(refined_map)
        self.refinement_version = next_version
        self.refined_observation_counts.update(applied_counts)
        self.last_refinement_saved_changes = changed
        self.last_refinement_saved_added = added
        self.last_refinement_rejected_reason = ""
        self.last_refinement_save_sec = now_sec
        self.get_logger().info(
            f"Saved refined map {path} version={self.refinement_version} "
            f"updated={updated} added={added} cones={len(refined_map.cones)} "
            f"quality={refined_map.quality:.2f}",
            throttle_duration_sec=5.0,
        )

    def nearest_same_color_landmark(
        self,
        landmarks: list[ConeLandmark],
        candidate: ConeLandmark,
        max_distance: float,
    ) -> ConeLandmark | None:
        best = None
        best_distance = max_distance
        for landmark in landmarks:
            if not self.colors_match_for_association(int(landmark.color), int(candidate.color)):
                continue
            distance = math.hypot(float(landmark.x) - float(candidate.x), float(landmark.y) - float(candidate.y))
            if distance < best_distance:
                best = landmark
                best_distance = distance
        return best

    def apply_landmark_refinement(
        self,
        target: ConeLandmark,
        measurement: ConeLandmark,
        previous_observations: int,
    ) -> bool:
        delta_observations = max(1, int(measurement.observations) - int(previous_observations))
        dx = float(measurement.x) - float(target.x)
        dy = float(measurement.y) - float(target.y)
        distance = math.hypot(dx, dy)
        if distance < 1e-4:
            return False
        alpha = max(0.0, min(0.25, self.refinement_position_alpha))
        step = min(self.refinement_max_position_step, distance * alpha)
        scale = step / max(distance, 1e-6)
        target.x = float(target.x) + dx * scale
        target.y = float(target.y) + dy * scale
        target.z = (float(target.z) + float(measurement.z)) * 0.5
        if int(measurement.color) != ConeColor.UNKNOWN and float(measurement.confidence) >= float(target.confidence):
            target.color = int(measurement.color)
        target.confidence = max(float(target.confidence), float(measurement.confidence))
        target.observations = int(target.observations) + delta_observations
        return True

    def live_landmark_is_safe_new_cone(
        self,
        saved_cones: list[ConeLandmark],
        candidate: ConeLandmark,
    ) -> bool:
        if int(candidate.observations) < self.refinement_new_cone_min_observations:
            return False
        if self.landmark_age_sec(candidate) < self.refinement_new_cone_min_persistence_sec:
            return False
        nearest = self.nearest_same_color_landmark(
            saved_cones,
            candidate,
            self.refinement_new_cone_min_separation,
        )
        return nearest is None

    @staticmethod
    def landmark_age_sec(landmark: ConeLandmark) -> float:
        first_seen = float(landmark.first_seen_sec)
        last_seen = float(landmark.last_seen_sec)
        if first_seen <= 0.0 or last_seen <= 0.0:
            return 0.0
        return max(0.0, last_seen - first_seen)

    def reset_loop_tracking(self) -> None:
        self.loop_completed = False
        self.loop_completion_status = "loop=waiting_for_orange"
        self.start_gate_x = None
        self.start_gate_y = None
        self.start_gate_observations = 0
        self.loop_distance_travelled = 0.0
        self.last_loop_pose = None

    def update_loop_distance(self) -> None:
        if self.pose is None or self.mode == "waiting_for_go":
            self.last_loop_pose = self.pose
            return
        if self.last_loop_pose is None:
            self.last_loop_pose = self.pose
            return
        step = math.hypot(float(self.pose.x) - float(self.last_loop_pose.x), float(self.pose.y) - float(self.last_loop_pose.y))
        self.last_loop_pose = self.pose
        if step <= self.loop_completion_max_pose_step:
            self.loop_distance_travelled += step
            self.complete_loop_on_pose_return()
            return
        if self.mode != "waiting_for_go":
            was_completed = self.loop_completed
            self.loop_completed = False
            self.loop_distance_travelled = 0.0
            self.loop_completion_status = "loop=pose_jump_reset_waiting_orange"
            if was_completed:
                self.get_logger().warn("Loop verification cleared after pose jump/reset")

    def complete_loop_on_pose_return(self) -> None:
        if self.pose is None or self.loop_completed:
            return
        if self.start_gate_x is None or self.start_gate_y is None:
            return
        pose_distance_to_gate = math.hypot(float(self.pose.x) - self.start_gate_x, float(self.pose.y) - self.start_gate_y)
        if (
            self.loop_distance_travelled >= self.loop_completion_min_travel
            and pose_distance_to_gate <= self.loop_completion_gate_radius
        ):
            self.loop_completed = True
            self.loop_completion_status = (
                f"loop=completed_pose_return travel={self.loop_distance_travelled:.1f}m "
                f"gate_dist={pose_distance_to_gate:.1f}m"
            )
            self.get_logger().info(self.loop_completion_status)

    def initialize_start_gate_from_map(self, track_map: SavedTrackMap) -> None:
        orange = [
            cone
            for cone in track_map.cones
            if int(cone.color) in (ConeColor.ORANGE, ConeColor.LARGE_ORANGE)
        ]
        if not orange:
            return
        self.start_gate_x = sum(float(cone.x) for cone in orange) / len(orange)
        self.start_gate_y = sum(float(cone.y) for cone in orange) / len(orange)
        self.start_gate_observations = len(orange)
        if not self.loop_completed:
            self.loop_completion_status = "loop=start_gate_loaded_waiting_return"

    def observe_start_gate(self, x: float, y: float, now_sec: float) -> None:
        del now_sec
        if self.start_gate_x is None or self.start_gate_y is None:
            self.start_gate_x = float(x)
            self.start_gate_y = float(y)
            self.start_gate_observations = 1
            self.loop_completion_status = "loop=start_gate_seen_waiting_return"
            return

        distance_to_gate = math.hypot(float(x) - self.start_gate_x, float(y) - self.start_gate_y)
        if distance_to_gate <= self.loop_completion_gate_radius * 1.75:
            n = max(1, self.start_gate_observations)
            self.start_gate_x = (self.start_gate_x * n + float(x)) / (n + 1)
            self.start_gate_y = (self.start_gate_y * n + float(y)) / (n + 1)
            self.start_gate_observations = n + 1

        if self.pose is None or self.loop_completed:
            return
        pose_distance_to_gate = math.hypot(float(self.pose.x) - self.start_gate_x, float(self.pose.y) - self.start_gate_y)
        if (
            self.loop_distance_travelled >= self.loop_completion_min_travel
            and pose_distance_to_gate <= self.loop_completion_gate_radius
            and distance_to_gate <= self.loop_completion_gate_radius * 2.0
        ):
            self.loop_completed = True
            self.loop_completion_status = (
                f"loop=completed_orange_return travel={self.loop_distance_travelled:.1f}m "
                f"gate_dist={pose_distance_to_gate:.1f}m"
            )
            self.get_logger().info(self.loop_completion_status)
        else:
            self.loop_completion_status = (
                f"loop=waiting_return travel={self.loop_distance_travelled:.1f}/"
                f"{self.loop_completion_min_travel:.1f}m gate_dist={pose_distance_to_gate:.1f}m"
            )

    def loop_map_ready(self, centerline, quality: float) -> bool:
        return bool(self.loop_completed and len(centerline) > 12 and quality > 0.45)

    @staticmethod
    def closed_loop_points(points, max_gap_m: float):
        values = list(points)
        if len(values) < 2:
            return values
        gap = math.hypot(float(values[0].x) - float(values[-1].x), float(values[0].y) - float(values[-1].y))
        if 0.05 < gap <= max_gap_m:
            values.append(values[0])
        return values

    @classmethod
    def racing_line_usable_for_map(cls, racing_line, centerline) -> bool:
        if len(centerline) < 8:
            return len(racing_line) >= 8
        if len(racing_line) < max(8, int(0.75 * len(centerline))):
            return False
        if cls.point_line_endpoint_gap(racing_line) > max(18.0, cls.point_line_endpoint_gap(centerline) + 8.0):
            return False
        if cls.point_line_max_step(racing_line) > 18.0:
            return False
        return True

    @staticmethod
    def point_line_endpoint_gap(points) -> float:
        values = list(points)
        if len(values) < 2:
            return float("inf")
        return math.hypot(float(values[0].x) - float(values[-1].x), float(values[0].y) - float(values[-1].y))

    @staticmethod
    def point_line_max_step(points) -> float:
        values = list(points)
        if len(values) < 2:
            return float("inf")
        return max(
            math.hypot(float(values[index].x) - float(values[index - 1].x), float(values[index].y) - float(values[index - 1].y))
            for index in range(1, len(values))
        )

    def backup_saved_map_once(self) -> None:
        if self.saved_map_backed_up:
            return
        path = map_path(self.map_dir, self.track_id)
        self.saved_map_backed_up = True
        if not path.exists():
            return
        history_dir = self.map_dir / "history"
        history_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        backup_path = history_dir / f"{self.track_id}.{stamp}.json"
        shutil.copy2(path, backup_path)
        self.get_logger().info(f"Backed up previous saved map to {backup_path}")

    @staticmethod
    def clone_landmark(cone: ConeLandmark) -> ConeLandmark:
        return ConeLandmark(
            x=float(cone.x),
            y=float(cone.y),
            z=float(cone.z),
            color=int(cone.color),
            confidence=float(cone.confidence),
            observations=int(cone.observations),
            first_seen_sec=float(cone.first_seen_sec),
            last_seen_sec=float(cone.last_seen_sec),
            last_seen_frame=int(cone.last_seen_frame),
            hit_streak=int(cone.hit_streak),
            missed_frames=int(cone.missed_frames),
        )

    def calculation_landmarks(self) -> list[ConeLandmark]:
        return dedupe_cone_landmarks([
            self.clone_landmark(cone)
            for cone in self.landmarks
            if float(cone.confidence) >= self.min_cone_confidence
            and self.is_confirmed_landmark(cone)
        ], self.merge_landmark_distance)

    def prune_tentative_landmarks(self, seen_landmark_ids: set[int], now_sec: float) -> None:
        kept: list[ConeLandmark] = []
        pruned = 0
        for landmark in self.landmarks:
            if id(landmark) not in seen_landmark_ids:
                landmark.missed_frames = max(0, int(landmark.missed_frames)) + 1
                landmark.hit_streak = 0
            if self.is_confirmed_landmark(landmark):
                kept.append(landmark)
                continue
            stale_by_frames = int(landmark.missed_frames) >= self.tentative_landmark_max_missed_frames
            stale_by_time = (float(now_sec) - float(landmark.last_seen_sec)) >= self.tentative_landmark_timeout_sec
            if stale_by_frames and stale_by_time:
                pruned += 1
                continue
            kept.append(landmark)
        self.landmarks = kept
        if pruned:
            self.get_logger().info(
                f"Pruned {pruned} tentative cone landmark(s) before map confirmation",
                throttle_duration_sec=5.0,
            )

    def is_confirmed_landmark(self, landmark: ConeLandmark) -> bool:
        if int(landmark.observations) < max(1, self.min_landmark_observations):
            return False
        first_seen = float(landmark.first_seen_sec)
        last_seen = float(landmark.last_seen_sec)
        if first_seen <= 0.0 or last_seen <= 0.0:
            return True
        return (last_seen - first_seen) >= max(0.0, self.min_landmark_persistence_sec)

    def tentative_landmark_count(self) -> int:
        return sum(1 for landmark in self.landmarks if not self.is_confirmed_landmark(landmark))

    def confirmed_landmark_count(self) -> int:
        return sum(1 for landmark in self.landmarks if self.is_confirmed_landmark(landmark))

    def is_mapping_source_allowed(self, source: str) -> bool:
        source_text = str(source)
        if not is_reliable_geometry_source(source_text):
            return False
        if self.require_lidar_stereo_for_mapping:
            return "Lidar" in source_text and STEREO_CAMERA_SOURCE in source_text
        if self.require_stereo_for_mapping:
            return STEREO_CAMERA_SOURCE in source_text
        return True

    @staticmethod
    def colors_match_for_association(existing_color: int, candidate_color: int) -> bool:
        if int(existing_color) == int(candidate_color):
            return True
        if int(existing_color) == ConeColor.UNKNOWN or int(candidate_color) == ConeColor.UNKNOWN:
            return True
        return False

    def filtered_track_map(self, track_map: SavedTrackMap) -> SavedTrackMap:
        cones = dedupe_cone_landmarks([
            self.clone_landmark(cone)
            for cone in track_map.cones
            if float(cone.confidence) >= self.min_cone_confidence
        ], self.merge_landmark_distance)
        centerline = build_centerline_from_cones(cones)
        blue_boundary, yellow_boundary = build_boundary_lines_from_cones(cones, centerline)
        reset_events = self.reset_events_for_planning()
        racing_line, speed_profile = build_racing_line(
            centerline,
            cones,
            reset_events=reset_events,
            reset_event_influence_radius_m=self.reset_event_avoidance_influence_radius,
            reset_event_max_shift_m=self.reset_event_avoidance_max_shift,
        )
        racing_line_fallback = ""
        if not self.racing_line_usable_for_map(racing_line, centerline):
            racing_line = list(centerline)
            speed_profile = speed_profile_for_path(racing_line, min_speed=2.0, max_speed=5.0, curvature_gain=24.0)
            racing_line_fallback = "centerline_full_loop_due_discontinuous_generated_line"
        quality = infer_map_quality(cones, centerline)
        closed_loop = bool(
            track_map.metadata.get("loop_completed_after_orange_gate", False)
            and len(centerline) > 12
            and quality > 0.45
        )
        if closed_loop:
            blue_boundary = self.closed_loop_points(blue_boundary, max_gap_m=10.0)
            yellow_boundary = self.closed_loop_points(yellow_boundary, max_gap_m=10.0)
            centerline = self.closed_loop_points(centerline, max_gap_m=18.0)
            racing_line_was = len(racing_line)
            racing_line = self.closed_loop_points(racing_line, max_gap_m=18.0)
            if speed_profile and len(racing_line) > racing_line_was:
                speed_profile = list(speed_profile) + [float(speed_profile[0])]
        metadata = dict(track_map.metadata)
        metadata.update(
            {
                "filtered_from_cones": len(track_map.cones),
                "min_cone_confidence": self.min_cone_confidence,
                "racing_line_vehicle_width_m": RACING_CAR_WIDTH_M,
                "racing_line_cone_width_m": RACING_CONE_WIDTH_M,
                "racing_line_clearance_margin_m": RACING_CLEARANCE_MARGIN_M,
                "racing_line_min_center_to_cone_m": MIN_RACING_CONE_CLEARANCE_M,
                "racing_line_uses_track_width": True,
                "racing_line_profile": "outside_apex_outside_width_aware",
                "loop_completed_after_orange_gate": closed_loop,
                "reset_event_avoidance_enabled": self.reset_event_avoidance_enabled,
                "reset_event_avoidance_events": len(reset_events),
            }
        )
        if racing_line_fallback:
            metadata["racing_line_fallback"] = racing_line_fallback
        return SavedTrackMap(
            track_id=track_map.track_id,
            closed_loop=closed_loop,
            quality=quality,
            cones=cones,
            blue_boundary_line=[(p.x, p.y) for p in blue_boundary],
            yellow_boundary_line=[(p.x, p.y) for p in yellow_boundary],
            centerline=[(p.x, p.y) for p in centerline],
            racing_line=[(p.x, p.y) for p in racing_line],
            speed_profile=speed_profile,
            metadata=metadata,
        )

    @classmethod
    def clone_track_map(cls, track_map: SavedTrackMap) -> SavedTrackMap:
        return SavedTrackMap(
            track_id=track_map.track_id,
            closed_loop=bool(track_map.closed_loop),
            quality=float(track_map.quality),
            cones=[cls.clone_landmark(cone) for cone in track_map.cones],
            blue_boundary_line=list(track_map.blue_boundary_line),
            yellow_boundary_line=list(track_map.yellow_boundary_line),
            centerline=list(track_map.centerline),
            racing_line=list(track_map.racing_line),
            speed_profile=list(track_map.speed_profile),
            metadata=dict(track_map.metadata),
        )


def main() -> None:
    rclpy.init()
    rclpy.spin(Mapper())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

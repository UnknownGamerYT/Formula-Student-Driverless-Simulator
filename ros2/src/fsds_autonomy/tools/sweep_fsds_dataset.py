#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from fs_msgs.msg import Track
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


REPO_ROOT = Path(__file__).resolve().parents[4]
PYTHON_DIR = REPO_ROOT / "python"
AIRSIM_CLIENT_DIR = REPO_ROOT / "AirSim" / "PythonClient" / "airsim"
for path in (str(PYTHON_DIR), str(AIRSIM_CLIENT_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from fsds.types import Pose, Quaternionr, Vector3r  # noqa: E402
import client as airsim_client  # noqa: E402


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def yaw_to_quaternion(yaw: float) -> Quaternionr:
    return Quaternionr(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def greedy_order(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points
    remaining = list(points)
    start_index = min(range(len(remaining)), key=lambda index: distance((0.0, 0.0), remaining[index]))
    ordered = [remaining.pop(start_index)]
    while remaining:
        next_index = min(range(len(remaining)), key=lambda index: distance(ordered[-1], remaining[index]))
        ordered.append(remaining.pop(next_index))
    return ordered


def centerline_from_track(track: Track, max_pair_distance: float = 8.0) -> list[tuple[float, float]]:
    blue = [(cone.location.x, cone.location.y) for cone in track.track if int(cone.color) == 0]
    yellow = [(cone.location.x, cone.location.y) for cone in track.track if int(cone.color) == 1]
    if not blue or not yellow:
        return []
    midpoints: list[tuple[float, float]] = []
    used_yellow: set[int] = set()
    for bx, by in blue:
        best_index = None
        best_dist = max_pair_distance
        for index, yellow_point in enumerate(yellow):
            if index in used_yellow:
                continue
            dist = distance((bx, by), yellow_point)
            if dist < best_dist:
                best_dist = dist
                best_index = index
        if best_index is None:
            continue
        used_yellow.add(best_index)
        yx, yy = yellow[best_index]
        midpoints.append((0.5 * (bx + yx), 0.5 * (by + yy)))
    return greedy_order(midpoints)


def sample_centerline(points: list[tuple[float, float]], stride_m: float, max_samples: int) -> list[Pose2]:
    if len(points) < 2:
        return []
    poses: list[Pose2] = []
    carry = 0.0
    for start, end in zip(points, points[1:]):
        seg_len = distance(start, end)
        if seg_len < 1e-3:
            continue
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        step = max(0.1, stride_m - carry)
        while step <= seg_len:
            t = step / seg_len
            poses.append(Pose2(start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]), yaw))
            if len(poses) >= max_samples:
                return poses
            step += stride_m
        carry = max(0.0, seg_len - (step - stride_m))
    return poses


def offset_pose(pose: Pose2, lateral_m: float, yaw_delta: float) -> Pose2:
    return Pose2(
        x=pose.x - math.sin(pose.yaw) * lateral_m,
        y=pose.y + math.cos(pose.yaw) * lateral_m,
        yaw=pose.yaw + yaw_delta,
    )


class TrackReceiver(Node):
    def __init__(self) -> None:
        super().__init__("fsds_dataset_sweep_receiver")
        self.track: Track | None = None
        self.odom: Odometry | None = None
        track_qos = QoSProfile(depth=1)
        track_qos.reliability = ReliabilityPolicy.RELIABLE
        track_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Track, "/fsds/testing_only/track", self.on_track, track_qos)
        self.create_subscription(Odometry, "/fsds/testing_only/odom", self.on_odom, 10)

    def on_track(self, msg: Track) -> None:
        self.track = msg

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg


def wait_for_track(timeout_sec: float) -> tuple[Track, float]:
    rclpy.init()
    node = TrackReceiver()
    deadline = time.time() + timeout_sec
    try:
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.track is not None and node.odom is not None:
                z = float(node.odom.pose.pose.position.z)
                return node.track, z
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise TimeoutError("Timed out waiting for /fsds/testing_only/track and /fsds/testing_only/odom")


def main() -> None:
    parser = argparse.ArgumentParser(description="Teleport the FSDS car through the track to collect labeled camera data.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="FSCar")
    parser.add_argument("--samples", type=int, default=140)
    parser.add_argument("--stride-m", type=float, default=1.6)
    parser.add_argument("--seconds-per-pose", type=float, default=0.45)
    parser.add_argument("--lateral-offsets", default="-0.45,0.0,0.45")
    parser.add_argument("--yaw-offset-deg", default="-10,0,10")
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--wait-timeout-sec", type=float, default=8.0)
    args = parser.parse_args()

    track, current_z = wait_for_track(args.wait_timeout_sec)
    centerline = centerline_from_track(track)
    if len(centerline) < 2:
        raise SystemExit("Could not build a centerline from /fsds/testing_only/track")

    base_poses = sample_centerline(centerline, args.stride_m, args.samples)
    lateral_offsets = [float(value) for value in args.lateral_offsets.split(",") if value.strip()]
    yaw_offsets = [math.radians(float(value)) for value in args.yaw_offset_deg.split(",") if value.strip()]
    sweep_poses = [
        offset_pose(base_pose, lateral, yaw_delta)
        for base_pose in base_poses
        for lateral in lateral_offsets
        for yaw_delta in yaw_offsets
    ]

    client = airsim_client.CarClient(ip=args.host, port=args.port, timeout_value=5)
    if not client.ping():
        raise SystemExit("Could not ping FSDS/AirSim RPC server")

    print(f"Track cones={len(track.track)} centerline={len(centerline)} base_samples={len(base_poses)} sweep_poses={len(sweep_poses)}")
    print("Dataset builders should be running with dataset_enabled:=true before this sweep starts.")
    for index, pose in enumerate(sweep_poses, start=1):
        client.simSetVehiclePose(
            Pose(Vector3r(float(pose.x), float(pose.y), float(current_z + args.z_offset)), yaw_to_quaternion(pose.yaw)),
            True,
            args.vehicle,
        )
        if index == 1 or index % 25 == 0 or index == len(sweep_poses):
            print(f"pose {index}/{len(sweep_poses)} x={pose.x:.1f} y={pose.y:.1f} yaw={math.degrees(pose.yaw):+.1f}deg")
        time.sleep(max(0.05, args.seconds_per_pose))

    print("Sweep complete.")


if __name__ == "__main__":
    main()

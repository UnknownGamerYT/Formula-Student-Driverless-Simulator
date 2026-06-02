#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def control_events(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("event") == "control" and isinstance(event.get("command"), dict):
                yield event


class DriveLogReplayer:
    def __init__(self, topic: str) -> None:
        from fs_msgs.msg import ControlCommand
        from rclpy.node import Node

        class _Node(Node):
            pass

        self.ControlCommand = ControlCommand
        self.node = _Node("fsds_drive_log_replayer")
        self.pub = self.node.create_publisher(ControlCommand, topic, 10)

    def publish_event(self, event: dict) -> None:
        command = event["command"]
        msg = self.ControlCommand()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "fsds/FSCar"
        msg.throttle = float(command.get("throttle", 0.0))
        msg.brake = float(command.get("brake", 0.0))
        msg.steering = float(command.get("steering", 0.0))
        self.pub.publish(msg)

    def destroy_node(self) -> None:
        self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay /fsds/control_command entries from a drive_recorder JSONL log.")
    parser.add_argument("log", type=Path)
    parser.add_argument("--topic", default="/fsds/control_command")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier. 2.0 is twice as fast.")
    parser.add_argument("--dry-run", action="store_true", help="Print command timing without publishing.")
    parser.add_argument("--max-events", type=int, default=0)
    args = parser.parse_args()

    events = list(control_events(args.log.expanduser()))
    if args.max_events > 0:
        events = events[: args.max_events]
    print(f"Loaded {len(events)} control events from {args.log}")
    if args.dry_run:
        for event in events[:20]:
            print(event.get("seq"), event.get("dt_sec"), event.get("command"))
        return

    import rclpy

    rclpy.init()
    node = DriveLogReplayer(args.topic)
    try:
        previous_time = None
        speed = max(1e-6, float(args.speed))
        for event in events:
            event_time = event.get("time_sec")
            if previous_time is not None and isinstance(event_time, (float, int)):
                delay = max(0.0, (float(event_time) - previous_time) / speed)
                time.sleep(min(delay, 1.0))
            if isinstance(event_time, (float, int)):
                previous_time = float(event_time)
            node.publish_event(event)
            rclpy.spin_once(node.node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

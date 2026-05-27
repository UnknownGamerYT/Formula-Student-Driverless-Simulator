from __future__ import annotations

import rclpy
from fsds_autonomy_msgs.msg import TrackMap
from rclpy.node import Node

from fsds_autonomy.map_store import ConeLandmark, SavedTrackMap
from fsds_autonomy.planning import build_centerline_from_cones, build_racing_line, infer_map_quality
from fsds_autonomy.ros_utils import track_map_to_msg


class RacelinePlanner(Node):
    def __init__(self) -> None:
        super().__init__("fsds_raceline_planner")
        self.pub = self.create_publisher(TrackMap, "/autonomy/racing_line", 10)
        self.create_subscription(TrackMap, "/autonomy/track_map", self.on_track_map, 10)

    def on_track_map(self, msg: TrackMap) -> None:
        cones = [
            ConeLandmark(
                x=cone.position.x,
                y=cone.position.y,
                z=cone.position.z,
                color=cone.color,
                confidence=cone.confidence,
                observations=1,
            )
            for cone in msg.cones
        ]
        centerline = list(msg.centerline) or build_centerline_from_cones(cones)
        racing_line, speed_profile = build_racing_line(centerline)
        quality = max(float(msg.quality), infer_map_quality(cones, centerline))
        planned = SavedTrackMap(
            track_id=msg.track_id,
            closed_loop=msg.closed_loop,
            quality=quality,
            cones=cones,
            centerline=[(p.x, p.y) for p in centerline],
            racing_line=[(p.x, p.y) for p in racing_line],
            speed_profile=speed_profile,
        )
        self.pub.publish(track_map_to_msg(self, planned))


def main() -> None:
    rclpy.init()
    rclpy.spin(RacelinePlanner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

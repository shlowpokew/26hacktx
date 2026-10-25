#!/usr/bin/env python3
"""Track a person-sized cluster from the LD19 scan while follow mode is on."""

import math
import os
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class HumanTracker(Node):
    def __init__(self) -> None:
        super().__init__("human_tracker")
        self._range_min = float(os.environ.get("TRACKER_MIN_RANGE_M", "0.10"))
        self._range_max = float(os.environ.get("TRACKER_MAX_RANGE_M", "3.0"))
        self._cluster_gap = float(os.environ.get("TRACKER_CLUSTER_GAP_M", "0.15"))
        self._min_width = float(os.environ.get("TRACKER_MIN_WIDTH_M", "0.10"))
        self._max_width = float(os.environ.get("TRACKER_MAX_WIDTH_M", "0.50"))
        self._front_arc_deg = float(os.environ.get("TRACKER_FRONT_ARC_DEG", "30"))
        self._association_distance = float(
            os.environ.get("TRACKER_ASSOCIATION_DISTANCE_M", "0.60")
        )
        self._target_timeout = float(os.environ.get("TRACKER_TARGET_TIMEOUT_SEC", "0.5"))
        self._lidar_x = float(os.environ.get("LIDAR_X_M", "0.0"))
        self._lidar_y = float(os.environ.get("LIDAR_Y_M", "0.0"))
        self._lidar_yaw = float(os.environ.get("LIDAR_YAW_RAD", "0.0"))
        self._enabled = False
        self._target: Optional[tuple[float, float]] = None
        self._last_target_at: Optional[float] = None

        self._target_publisher = self.create_publisher(PoseStamped, "/human_target", 10)
        self._scan_subscription = self.create_subscription(
            LaserScan, "/scan", self._scan, 10
        )
        self._enable_subscription = self.create_subscription(
            Bool, "/follow/enable", self._set_enabled, 10
        )
        self.get_logger().info(
            f"Person cluster width {self._min_width:.2f}–{self._max_width:.2f} m; "
            f"front lock arc ±{self._front_arc_deg:g}°"
        )

    def _set_enabled(self, message: Bool) -> None:
        if self._enabled and not message.data:
            self._target = None
            self._last_target_at = None
        self._enabled = message.data

    def _scan(self, scan: LaserScan) -> None:
        if not self._enabled:
            return
        points = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not self._range_min <= distance <= self._range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment + self._lidar_yaw
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            points.append((self._lidar_x + x, self._lidar_y + y))

        clusters = self._clusters(points)
        candidates = []
        for cluster in clusters:
            width = self._cluster_width(cluster)
            if not self._min_width <= width <= self._max_width:
                continue
            x = sum(point[0] for point in cluster) / len(cluster)
            y = sum(point[1] for point in cluster) / len(cluster)
            angle = math.atan2(y, x)
            if x > 0 and abs(angle) <= math.radians(self._front_arc_deg):
                candidates.append((x, y))

        chosen = self._associate(candidates)
        now = time.monotonic()
        if chosen is None:
            if self._last_target_at is not None and now - self._last_target_at > self._target_timeout:
                self._target = None
            return

        if self._target is None:
            self._target = chosen
        else:
            self._target = (
                0.65 * chosen[0] + 0.35 * self._target[0],
                0.65 * chosen[1] + 0.35 * self._target[1],
            )
        self._last_target_at = now

        target = PoseStamped()
        target.header = scan.header
        target.header.frame_id = "base_link"
        target.pose.position.x = self._target[0]
        target.pose.position.y = self._target[1]
        target.pose.orientation.w = 1.0
        self._target_publisher.publish(target)

    def _clusters(self, points: list[tuple[float, float]]) -> list[list[tuple[float, float]]]:
        clusters: list[list[tuple[float, float]]] = []
        for point in points:
            if not clusters or math.dist(point, clusters[-1][-1]) > self._cluster_gap:
                clusters.append([point])
            else:
                clusters[-1].append(point)
        if len(clusters) > 1 and len(clusters[0]) and len(clusters[-1]):
            if math.dist(clusters[0][0], clusters[-1][-1]) <= self._cluster_gap:
                clusters[0] = clusters[-1] + clusters[0]
                clusters.pop()
        return [cluster for cluster in clusters if len(cluster) >= 3]

    @staticmethod
    def _cluster_width(cluster: list[tuple[float, float]]) -> float:
        return max(
            (math.dist(a, b) for index, a in enumerate(cluster) for b in cluster[index + 1:]),
            default=0.0,
        )

    def _associate(
        self, candidates: list[tuple[float, float]]
    ) -> Optional[tuple[float, float]]:
        if not candidates:
            return None
        if self._target is None:
            return min(candidates, key=lambda point: math.hypot(*point))
        nearest = min(candidates, key=lambda point: math.dist(point, self._target))
        if math.dist(nearest, self._target) <= self._association_distance:
            return nearest
        return None


def main() -> None:
    rclpy.init()
    node = HumanTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

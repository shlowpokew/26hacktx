#!/usr/bin/env python3
"""Apply the rover's valid distance window to LD19 LaserScan messages."""

import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarScanFilter(Node):
    def __init__(self) -> None:
        super().__init__("lidar_scan_filter")
        self._minimum = float(os.environ.get("LIDAR_MIN_RANGE_M", "0.15"))
        self._maximum = float(os.environ.get("LIDAR_MAX_RANGE_M", "8.0"))
        if self._minimum < 0 or self._maximum <= self._minimum:
            raise ValueError("LiDAR range limits must satisfy 0 <= min < max")
        self._publisher = self.create_publisher(LaserScan, "/scan", 10)
        self._subscription = self.create_subscription(
            LaserScan, "/scan/raw", self._filter, 10
        )

    def _filter(self, message: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = message.header
        filtered.angle_min = message.angle_min
        filtered.angle_max = message.angle_max
        filtered.angle_increment = message.angle_increment
        filtered.time_increment = message.time_increment
        filtered.scan_time = message.scan_time
        filtered.range_min = self._minimum
        filtered.range_max = self._maximum
        filtered.ranges = [
            value if math.isfinite(value) and self._minimum <= value <= self._maximum
            else math.nan
            for value in message.ranges
        ]
        filtered.intensities = list(message.intensities)
        self._publisher.publish(filtered)


def main() -> None:
    rclpy.init()
    node = LidarScanFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

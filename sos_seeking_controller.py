#!/usr/bin/env python3
"""Slowly approach a detected responder after the SOS return route is complete."""

import math
import os
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SosSeekingController(Node):
    def __init__(self) -> None:
        super().__init__("sos_seeking_controller")
        self._stop_distance = float(os.environ.get("SOS_SEEK_STOP_DISTANCE_M", "1.0"))
        self._max_speed = float(os.environ.get("SOS_SEEK_MAX_SPEED_MPS", "0.12"))
        self._max_turn = float(os.environ.get("SOS_SEEK_MAX_ANGULAR_RPS", "0.6"))
        self._obstacle_distance = float(os.environ.get("OBSTACLE_STOP_DISTANCE_M", "0.25"))
        self._lidar_yaw = float(os.environ.get("LIDAR_YAW_RAD", "0.0"))
        self._mode = "S0_IDLE"
        self._enabled = False
        self._targets: list[tuple[float, float]] = []
        self._targets_at: Optional[float] = None
        self._scan_at: Optional[float] = None
        self._blocked = False
        self._cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/mission/state", self._set_mode, 10)
        self.create_subscription(Bool, "/sos/seek_enable", self._set_enabled, 10)
        self.create_subscription(PoseArray, "/sos/human_targets", self._set_targets, 10)
        self.create_subscription(LaserScan, "/scan", self._set_scan, 10)
        self.create_timer(0.1, self._control)

    def _set_mode(self, message: String) -> None:
        self._mode = message.data
        if self._mode != "S3_SOS":
            self._targets = []
            self._targets_at = None

    def _set_enabled(self, message: Bool) -> None:
        self._enabled = message.data
        if not self._enabled:
            self._publish_stop()

    def _set_targets(self, message: PoseArray) -> None:
        self._targets = [
            (pose.position.x, pose.position.y) for pose in message.poses
        ]
        self._targets_at = time.monotonic()

    def _set_scan(self, scan: LaserScan) -> None:
        self._scan_at = time.monotonic()
        half_arc = math.radians(25.0)
        self._blocked = any(
            math.isfinite(distance)
            and 0 < distance < self._obstacle_distance
            and abs(math.atan2(
                math.sin(scan.angle_min + index * scan.angle_increment + self._lidar_yaw),
                math.cos(scan.angle_min + index * scan.angle_increment + self._lidar_yaw),
            )) <= half_arc
            for index, distance in enumerate(scan.ranges)
        )

    def _control(self) -> None:
        if not self._enabled or self._mode != "S3_SOS":
            return
        now = time.monotonic()
        if (
            self._blocked
            or self._scan_at is None
            or now - self._scan_at > 0.5
            or not self._targets
            or self._targets_at is None
            or now - self._targets_at > 0.5
        ):
            self._publish_stop()
            return

        x, y = min(self._targets, key=lambda point: math.hypot(*point))
        distance = math.hypot(x, y)
        angle = math.atan2(y, x)
        command = Twist()
        command.angular.z = max(-self._max_turn, min(self._max_turn, 1.2 * angle))
        if distance > self._stop_distance and abs(angle) < math.radians(45):
            command.linear.x = min(self._max_speed, 0.08 * (distance - self._stop_distance))
        self._cmd_publisher.publish(command)

    def _publish_stop(self) -> None:
        self._cmd_publisher.publish(Twist())


def main() -> None:
    rclpy.init()
    node = SosSeekingController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Follow a tracked person, with stale-data and close-obstacle stops."""

import math
import os
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class RoverController(Node):
    def __init__(self) -> None:
        super().__init__("rover_controller")
        self._follow_distance = float(os.environ.get("FOLLOW_DISTANCE_M", "1.0"))
        self._max_linear = float(os.environ.get("DRIVE_MAX_LINEAR_MPS", "0.5"))
        self._max_angular = float(os.environ.get("DRIVE_MAX_ANGULAR_RPS", "1.0"))
        self._linear_gain = float(os.environ.get("FOLLOW_LINEAR_GAIN", "0.7"))
        self._angular_gain = float(os.environ.get("FOLLOW_ANGULAR_GAIN", "1.5"))
        self._obstacle_distance = float(os.environ.get("OBSTACLE_STOP_DISTANCE_M", "0.25"))
        self._obstacle_arc_deg = float(os.environ.get("OBSTACLE_STOP_ARC_DEG", "25"))
        self._data_timeout = float(os.environ.get("FOLLOW_DATA_TIMEOUT_SEC", "0.5"))
        self._lidar_yaw = float(os.environ.get("LIDAR_YAW_RAD", "0.0"))
        if min(self._follow_distance, self._max_linear, self._max_angular,
               self._obstacle_distance, self._data_timeout) <= 0:
            raise ValueError("Follow distances, speed limits, and timeouts must be positive")
        self._enabled = False
        self._target: Optional[tuple[float, float]] = None
        self._target_at: Optional[float] = None
        self._obstacle = False
        self._scan_at: Optional[float] = None

        self._cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._target_subscription = self.create_subscription(
            PoseStamped, "/human_target", self._set_target, 10
        )
        self._scan_subscription = self.create_subscription(
            LaserScan, "/scan", self._set_scan, 10
        )
        self._enable_subscription = self.create_subscription(
            Bool, "/follow/enable", self._set_enabled, 10
        )
        self.create_timer(0.1, self._control)

    def _set_enabled(self, message: Bool) -> None:
        self._enabled = message.data
        if not self._enabled:
            self._publish_stop()

    def _set_target(self, message: PoseStamped) -> None:
        self._target = (message.pose.position.x, message.pose.position.y)
        self._target_at = time.monotonic()

    def _set_scan(self, message: LaserScan) -> None:
        self._scan_at = time.monotonic()
        half_arc = math.radians(self._obstacle_arc_deg)
        self._obstacle = False
        for index, distance in enumerate(message.ranges):
            if not math.isfinite(distance) or distance <= 0:
                continue
            angle = message.angle_min + index * message.angle_increment + self._lidar_yaw
            if abs(math.atan2(math.sin(angle), math.cos(angle))) <= half_arc and distance < self._obstacle_distance:
                self._obstacle = True
                break

    def _control(self) -> None:
        now = time.monotonic()
        if not self._enabled:
            return
        if (
            self._target is None
            or self._target_at is None
            or now - self._target_at > self._data_timeout
            or self._scan_at is None
            or now - self._scan_at > self._data_timeout
            or self._obstacle
        ):
            self._publish_stop()
            return

        x, y = self._target
        distance = math.hypot(x, y)
        angle = math.atan2(y, x)
        command = Twist()
        if abs(angle) < math.radians(60) and distance > self._follow_distance:
            command.linear.x = min(
                self._max_linear,
                self._linear_gain * (distance - self._follow_distance),
            ) * max(0.0, math.cos(angle))
        command.angular.z = max(
            -self._max_angular,
            min(self._max_angular, self._angular_gain * angle),
        )
        self._cmd_publisher.publish(command)

    def _publish_stop(self) -> None:
        self._cmd_publisher.publish(Twist())


def main() -> None:
    rclpy.init()
    node = RoverController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

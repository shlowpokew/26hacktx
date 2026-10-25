#!/usr/bin/env python3
"""Reverse-drive through the recorded map-frame breadcrumb trail."""

import json
import math
import os
from pathlib import Path
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class BacktrackNavigator(Node):
    def __init__(self) -> None:
        super().__init__("backtrack_navigator")
        self._path = Path(os.environ.get("BREADCRUMB_FILE", "/maps/breadcrumbs.json"))
        self._map_frame = os.environ.get("MAP_FRAME", "map")
        self._base_frame = os.environ.get("BASE_FRAME", "base_link")
        self._speed = float(os.environ.get("BACKTRACK_SPEED_MPS", "0.20"))
        self._angular_gain = float(os.environ.get("BACKTRACK_ANGULAR_GAIN", "1.5"))
        self._max_angular = float(os.environ.get("DRIVE_MAX_ANGULAR_RPS", "1.0"))
        self._goal_tolerance = float(os.environ.get("BACKTRACK_GOAL_TOLERANCE_M", "0.20"))
        self._obstacle_distance = float(os.environ.get("OBSTACLE_STOP_DISTANCE_M", "0.25"))
        self._obstacle_arc_deg = float(os.environ.get("OBSTACLE_STOP_ARC_DEG", "25"))
        self._lidar_yaw = float(os.environ.get("LIDAR_YAW_RAD", "0.0"))
        self._enabled = False
        self._trail: list[dict[str, float]] = []
        self._target_index = -1
        self._obstacle = False
        self._scan_at: Optional[float] = None
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._complete_publisher = self.create_publisher(Bool, "/backtrack/complete", 10)
        self.create_subscription(Bool, "/backtrack/enable", self._set_enabled, 10)
        self.create_subscription(LaserScan, "/scan", self._set_scan, 10)
        self.create_timer(0.1, self._navigate)

    def _set_enabled(self, message: Bool) -> None:
        if message.data and not self._enabled:
            self._load_trail()
            self._target_index = len(self._trail) - 2
            self._enabled = self._target_index >= 0
            if not self._enabled:
                self._publish_stop()
                self._complete_publisher.publish(Bool(data=True))
                self.get_logger().error("No saved breadcrumb route to backtrack")
                return
            self.get_logger().info(f"Backtracking {len(self._trail)} recorded poses")
        elif not message.data and self._enabled:
            self._enabled = False
            self._publish_stop()

    def _load_trail(self) -> None:
        try:
            with self._path.open(encoding="utf-8") as route_file:
                route = json.load(route_file)
            self._trail = [
                {"x": float(point["x"]), "y": float(point["y"]),
                 "theta": float(point["theta"])}
                for point in route
            ]
        except (OSError, ValueError, TypeError, KeyError) as error:
            self._trail = []
            self.get_logger().error(f"Could not load breadcrumb route: {error}")

    def _set_scan(self, scan: LaserScan) -> None:
        self._scan_at = time.monotonic()
        self._obstacle = False
        half_arc = math.radians(self._obstacle_arc_deg)
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance <= 0:
                continue
            angle = scan.angle_min + index * scan.angle_increment + self._lidar_yaw
            rear_angle = math.atan2(math.sin(angle - math.pi), math.cos(angle - math.pi))
            if abs(rear_angle) <= half_arc and distance < self._obstacle_distance:
                self._obstacle = True
                return

    def _navigate(self) -> None:
        if not self._enabled:
            return
        if (
            self._obstacle
            or self._scan_at is None
            or time.monotonic() - self._scan_at > 0.5
        ):
            self._publish_stop()
            return
        if self._target_index < 0:
            self._finish()
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time()
            )
        except TransformException:
            self._publish_stop()
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        goal = self._trail[self._target_index]
        dx = goal["x"] - translation.x
        dy = goal["y"] - translation.y
        distance = math.hypot(dx, dy)
        if distance <= self._goal_tolerance:
            self._target_index -= 1
            if self._target_index < 0:
                self._finish()
            return

        bearing_map = math.atan2(dy, dx)
        bearing_base = math.atan2(
            math.sin(bearing_map - yaw), math.cos(bearing_map - yaw)
        )
        reverse_heading_error = math.atan2(
            math.sin(bearing_base - math.pi), math.cos(bearing_base - math.pi)
        )
        command = Twist()
        command.linear.x = -self._speed
        command.angular.z = max(
            -self._max_angular,
            min(self._max_angular, self._angular_gain * reverse_heading_error),
        )
        self._cmd_publisher.publish(command)

    def _finish(self) -> None:
        self._enabled = False
        self._publish_stop()
        self._complete_publisher.publish(Bool(data=True))
        self.get_logger().info("Returned to the start of the breadcrumb trail")

    def _publish_stop(self) -> None:
        self._cmd_publisher.publish(Twist())


def main() -> None:
    rclpy.init()
    node = BacktrackNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

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
        self._detour_distance = float(os.environ.get("BACKTRACK_OBSTACLE_DISTANCE_M", "0.60"))
        self._emergency_distance = float(os.environ.get("BACKTRACK_EMERGENCY_DISTANCE_M", "0.30"))
        self._detour_arc_deg = float(os.environ.get("BACKTRACK_OBSTACLE_ARC_DEG", "45"))
        self._detour_speed = float(os.environ.get("BACKTRACK_DETOUR_SPEED_MPS", "0.08"))
        self._robot_radius = float(os.environ.get("BACKTRACK_ROBOT_RADIUS_M", "0.22"))
        self._planner_horizon = float(os.environ.get("BACKTRACK_PLANNER_HORIZON_SEC", "1.0"))
        self._lidar_x = float(os.environ.get("LIDAR_X_M", "0.0"))
        self._lidar_y = float(os.environ.get("LIDAR_Y_M", "0.0"))
        self._lidar_yaw = float(os.environ.get("LIDAR_YAW_RAD", "0.0"))
        if min(self._speed, self._goal_tolerance, self._detour_distance,
               self._emergency_distance, self._detour_arc_deg, self._detour_speed,
               self._robot_radius, self._planner_horizon) <= 0:
            raise ValueError("Backtrack speeds, distances, and planner limits must be positive")
        if self._emergency_distance >= self._detour_distance:
            raise ValueError("Emergency obstacle distance must be below the detour distance")
        self._enabled = False
        self._trail: list[dict[str, float]] = []
        self._target_index = -1
        self._scan_points: list[tuple[float, float]] = []
        self._front_clearance = math.inf
        self._rear_clearance = math.inf
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
        self._scan_points = []
        self._front_clearance = math.inf
        self._rear_clearance = math.inf
        half_arc = math.radians(self._detour_arc_deg)
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= min(scan.range_max, 3.0):
                continue
            angle = scan.angle_min + index * scan.angle_increment + self._lidar_yaw
            x = self._lidar_x + distance * math.cos(angle)
            y = self._lidar_y + distance * math.sin(angle)
            self._scan_points.append((x, y))
            if abs(angle) <= half_arc:
                self._front_clearance = min(self._front_clearance, distance)
            rear_angle = math.atan2(math.sin(angle - math.pi), math.cos(angle - math.pi))
            if abs(rear_angle) <= half_arc:
                self._rear_clearance = min(self._rear_clearance, distance)

    def _navigate(self) -> None:
        if not self._enabled:
            return
        if self._scan_at is None or time.monotonic() - self._scan_at > 0.5:
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

        if min(self._front_clearance, self._rear_clearance) < self._emergency_distance:
            self._publish_stop()
            return

        if min(self._front_clearance, self._rear_clearance) < self._detour_distance:
            bearing_map = math.atan2(dy, dx)
            goal_x_base = distance * math.cos(bearing_map - yaw)
            goal_y_base = distance * math.sin(bearing_map - yaw)
            command = self._choose_detour(goal_x_base, goal_y_base)
            if command is None:
                self._publish_stop()
            else:
                self._cmd_publisher.publish(command)
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

    def _choose_detour(self, goal_x: float, goal_y: float) -> Optional[Twist]:
        """Pick a safe reverse trajectory that makes progress toward the waypoint."""
        horizon_steps = max(1, math.ceil(self._planner_horizon / 0.1))
        dt = self._planner_horizon / horizon_steps
        best_score = -math.inf
        best_command: Optional[Twist] = None
        start_distance = math.hypot(goal_x, goal_y)
        obstacle_points = [
            point for point in self._scan_points
            if math.hypot(*point) <= self._planner_horizon * self._speed + 0.8
        ]
        for speed in (-self._detour_speed, -min(self._speed, self._detour_speed * 1.75)):
            for angular in (-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9):
                x = 0.0
                y = 0.0
                heading = 0.0
                clearance_cost = 0.0
                minimum_clearance = math.inf
                safe = True
                for _ in range(horizon_steps):
                    heading += angular * dt
                    x += speed * math.cos(heading) * dt
                    y += speed * math.sin(heading) * dt
                    for obstacle_x, obstacle_y in obstacle_points:
                        clearance = math.hypot(x - obstacle_x, y - obstacle_y) - self._robot_radius
                        minimum_clearance = min(minimum_clearance, clearance)
                        if clearance <= 0.02:
                            safe = False
                            break
                        if clearance < self._detour_distance:
                            clearance_cost += (self._detour_distance - clearance) ** 2
                    if not safe:
                        break
                if not safe:
                    continue
                progress = start_distance - math.hypot(goal_x - x, goal_y - y)
                score = (
                    4.0 * progress
                    + 0.15 * min(minimum_clearance, 1.0)
                    - 3.0 * clearance_cost / horizon_steps
                )
                if score > best_score:
                    best_score = score
                    best_command = Twist()
                    best_command.linear.x = speed
                    best_command.angular.z = angular
        return best_command

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

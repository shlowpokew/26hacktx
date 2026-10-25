#!/usr/bin/env python3
"""Record sparse map-frame poses during the outbound follow mission."""

import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class BreadcrumbManager(Node):
    def __init__(self) -> None:
        super().__init__("breadcrumb_manager")
        self._path = Path(os.environ.get("BREADCRUMB_FILE", "/maps/breadcrumbs.json"))
        self._map_frame = os.environ.get("MAP_FRAME", "map")
        self._base_frame = os.environ.get("BASE_FRAME", "base_link")
        self._spacing = float(os.environ.get("BREADCRUMB_SPACING_M", "0.25"))
        self._turn_spacing = math.radians(float(os.environ.get("BREADCRUMB_TURN_DEG", "15")))
        if self._spacing <= 0 or self._turn_spacing <= 0:
            raise ValueError("Breadcrumb spacing thresholds must be positive")
        self._recording = False
        self._breadcrumbs: list[dict[str, float]] = []
        self._last_recorded: Optional[dict[str, float]] = None
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(Bool, "/breadcrumb/recording", self._set_recording, 10)
        self.create_timer(0.1, self._record_pose)

    def _set_recording(self, message: Bool) -> None:
        if message.data and not self._recording:
            self._breadcrumbs = []
            self._last_recorded = None
            self._save()
        self._recording = message.data

    def _record_pose(self) -> None:
        if not self._recording:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time()
            )
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        current = {
            "x": translation.x,
            "y": translation.y,
            "theta": yaw,
            "timestamp": time.time(),
        }
        if self._last_recorded is not None:
            distance = math.hypot(
                current["x"] - self._last_recorded["x"],
                current["y"] - self._last_recorded["y"],
            )
            turn = abs(self._angle_delta(current["theta"], self._last_recorded["theta"]))
            if distance < self._spacing and turn < self._turn_spacing:
                return
        self._breadcrumbs.append(current)
        self._last_recorded = current
        self._save()

    @staticmethod
    def _angle_delta(first: float, second: float) -> float:
        return math.atan2(math.sin(first - second), math.cos(first - second))

    def _save(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        temporary_name = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=self._path.parent,
                prefix=f".{self._path.name}.", delete=False,
            ) as temporary:
                temporary_name = temporary.name
                json.dump(self._breadcrumbs, temporary, separators=(",", ":"))
                temporary.flush()
                os.fsync(temporary.fileno())
            os.replace(temporary_name, self._path)
            directory_fd = os.open(self._path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary_name and os.path.exists(temporary_name):
                os.unlink(temporary_name)


def main() -> None:
    rclpy.init()
    node = BreadcrumbManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

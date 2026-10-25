#!/usr/bin/env python3
"""Run the UPO LFE-PPN 2D-LiDAR people detector while follow mode is enabled."""

import math
import os
import time
from typing import Optional

import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import String


class HumanTracker(Node):
    def __init__(self) -> None:
        super().__init__("human_tracker")
        self._range_min = float(os.environ.get("TRACKER_MIN_RANGE_M", "0.10"))
        self._range_max = float(os.environ.get("TRACKER_MAX_RANGE_M", "3.0"))
        self._front_arc_deg = float(os.environ.get("TRACKER_FRONT_ARC_DEG", "60"))
        self._association_distance = float(
            os.environ.get("TRACKER_ASSOCIATION_DISTANCE_M", "0.60")
        )
        self._target_timeout = float(os.environ.get("TRACKER_TARGET_TIMEOUT_SEC", "0.5"))
        self._score_threshold = float(os.environ.get("HUMAN_DETECTOR_SCORE_THRESHOLD", "0.937660"))
        self._scan_near = float(os.environ.get("HUMAN_DETECTOR_SCAN_NEAR_M", "0.02"))
        self._scan_far = float(os.environ.get("HUMAN_DETECTOR_SCAN_FAR_M", "10.0"))
        self._sos_near_human_range = float(
            os.environ.get("SOS_NEAR_HUMAN_RANGE_M", "3.0")
        )
        self._lidar_x = float(os.environ.get("LIDAR_X_M", "0.0"))
        self._lidar_y = float(os.environ.get("LIDAR_Y_M", "0.0"))
        self._lidar_yaw = float(os.environ.get("LIDAR_YAW_RAD", "0.0"))
        model_path = os.environ.get("HUMAN_DETECTOR_MODEL", "/opt/robot/models/LFE-PPN.onnx")
        if not 0.0 <= self._score_threshold <= 1.0:
            raise ValueError("HUMAN_DETECTOR_SCORE_THRESHOLD must be in [0, 1]")
        if self._scan_near < 0 or self._scan_far <= self._scan_near:
            raise ValueError("Detector scan limits must satisfy 0 <= near < far")
        if min(self._range_min, self._range_max, self._association_distance,
               self._target_timeout) <= 0:
            raise ValueError("Tracker ranges, association distance, and timeout must be positive")

        options = ort.SessionOptions()
        options.intra_op_num_threads = max(1, int(os.environ.get("HUMAN_DETECTOR_THREADS", "2")))
        options.inter_op_num_threads = 1
        # The Raspberry Pi 5 has no CUDA device; use ONNX Runtime's ARM64 CPU provider.
        self._session = ort.InferenceSession(
            model_path, sess_options=options, providers=["CPUExecutionProvider"]
        )
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        self._mode = "S0_IDLE"
        self._target: Optional[tuple[float, float]] = None
        self._last_target_at: Optional[float] = None

        self._target_publisher = self.create_publisher(PoseStamped, "/human_target", 10)
        self._sos_targets_publisher = self.create_publisher(PoseArray, "/sos/human_targets", 10)
        self._sos_nearby_publisher = self.create_publisher(Bool, "/sos/human_nearby", 10)
        self._scan_subscription = self.create_subscription(
            LaserScan, "/scan", self._scan, 10
        )
        self._mode_subscription = self.create_subscription(
            String, "/mission/state", self._set_mode, 10
        )
        self.get_logger().info(
            f"UPO LFE-PPN loaded from {model_path}; CPU threads={options.intra_op_num_threads}, "
            f"score threshold={self._score_threshold:.4f}"
        )

    def _set_mode(self, message: String) -> None:
        if message.data != self._mode:
            self._target = None
            self._last_target_at = None
        self._mode = message.data

    def _scan(self, scan: LaserScan) -> None:
        if self._mode not in ("S1_FOLLOW_MAP", "S3_SOS"):
            return

        distances = np.asarray(scan.ranges, dtype=np.float32)
        # UPO's LFE-PPN input is 1 - range / scan_far; invalid beams represent
        # no return and therefore use scan_far (zero after normalization).
        distances = np.nan_to_num(
            distances, nan=self._scan_far, posinf=self._scan_far, neginf=self._scan_far
        )
        distances = np.clip(distances, 0.0, self._scan_far)
        model_input = (1.0 - distances / self._scan_far).reshape(1, -1, 1)
        grid = self._session.run(
            [self._output_name], {self._input_name: model_input}
        )[0]
        candidates = self._decode_detections(
            grid, scan, front_only=self._mode == "S1_FOLLOW_MAP"
        )
        if self._mode == "S3_SOS":
            targets = PoseArray()
            targets.header = scan.header
            for x, y in candidates:
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.orientation.w = 1.0
                targets.poses.append(pose)
            self._sos_targets_publisher.publish(targets)
            nearby = any(
                math.hypot(x, y) <= self._sos_near_human_range
                for x, y in candidates
            )
            self._sos_nearby_publisher.publish(Bool(data=nearby))
            return

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

    def _decode_detections(
        self, grid: np.ndarray, scan: LaserScan, *, front_only: bool
    ) -> list[tuple[float, float]]:
        if grid.ndim != 4 or grid.shape[0] != 1 or grid.shape[3] != 3:
            self.get_logger().error(f"Unexpected LFE-PPN output shape: {grid.shape}")
            return []
        sectors, anchors = grid.shape[1], grid.shape[2]
        anchor_depth = (self._scan_far - self._scan_near) / anchors
        sector_width = (scan.angle_max - scan.angle_min) / sectors
        flat = grid.reshape(-1, 3)
        order = np.flatnonzero(
            np.all(np.isfinite(flat), axis=1) & (flat[:, 0] >= self._score_threshold)
        )
        order = order[np.argsort(flat[order, 0])[::-1]]
        people: list[tuple[float, float]] = []
        suppressed: list[tuple[float, float]] = []
        for index in order:
            score, distance_offset, angle_offset = flat[index]
            sector = int(index) // anchors
            anchor = int(index) % anchors
            anchor_distance = self._scan_near + (0.5 + anchor) * anchor_depth
            angle = scan.angle_min + (0.5 + sector) * sector_width
            distance = anchor_distance + float(distance_offset) * anchor_depth
            angle += float(angle_offset) * anchor_depth / max(anchor_distance, 1e-6)
            lidar_x = distance * math.cos(angle)
            lidar_y = distance * math.sin(angle)

            # Match the UPO detector's non-maximum suppression radius (0.4 m).
            if any(math.hypot(lidar_x - x, lidar_y - y) < 0.8 for x, y in suppressed):
                continue
            suppressed.append((lidar_x, lidar_y))
            base_x = self._lidar_x + lidar_x * math.cos(self._lidar_yaw) - lidar_y * math.sin(self._lidar_yaw)
            base_y = self._lidar_y + lidar_x * math.sin(self._lidar_yaw) + lidar_y * math.cos(self._lidar_yaw)
            base_distance = math.hypot(base_x, base_y)
            base_angle = math.atan2(base_y, base_x)
            minimum_range = 0.60 if self._mode == "S3_SOS" else self._range_min
            if not minimum_range <= base_distance <= self._range_max:
                continue
            if front_only and (
                base_x <= 0 or abs(base_angle) > math.radians(self._front_arc_deg)
            ):
                continue
            people.append((base_x, base_y))
        return people

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

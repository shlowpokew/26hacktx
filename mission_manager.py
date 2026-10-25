#!/usr/bin/env python3
"""Small mission state machine for follow, backtrack, idle, and SOS."""

from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MissionState(str, Enum):
    IDLE = "S0_IDLE"
    FOLLOW = "S1_FOLLOW_MAP"
    BACKTRACK = "S2_BACKTRACK"
    SOS = "S3_SOS"


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")
        self._state = MissionState.IDLE
        self._follow_pub = self.create_publisher(Bool, "/follow/enable", 10)
        self._record_pub = self.create_publisher(Bool, "/breadcrumb/recording", 10)
        self._backtrack_pub = self.create_publisher(Bool, "/backtrack/enable", 10)
        self._state_pub = self.create_publisher(String, "/mission/state", 10)
        self.create_subscription(Bool, "/hardware/button_a", self._button_a, 10)
        self.create_subscription(Bool, "/hardware/button_b", self._button_b, 10)
        self.create_subscription(Bool, "/hardware/sos", self._sos, 10)
        self.create_subscription(Bool, "/backtrack/complete", self._backtrack_complete, 10)
        self._publish_state()

    def _button_a(self, message: Bool) -> None:
        if not message.data:
            return
        if self._state in (MissionState.IDLE, MissionState.SOS):
            self._set_state(MissionState.FOLLOW if self._state == MissionState.IDLE else MissionState.IDLE)

    def _button_b(self, message: Bool) -> None:
        if not message.data:
            return
        if self._state == MissionState.FOLLOW:
            self._set_state(MissionState.BACKTRACK)
        elif self._state in (MissionState.BACKTRACK, MissionState.SOS):
            self._set_state(MissionState.IDLE)

    def _sos(self, message: Bool) -> None:
        if message.data:
            self._set_state(MissionState.SOS)

    def _backtrack_complete(self, message: Bool) -> None:
        if message.data and self._state == MissionState.BACKTRACK:
            self._set_state(MissionState.IDLE)

    def _set_state(self, state: MissionState) -> None:
        if state == self._state:
            return
        self._state = state
        self._follow_pub.publish(Bool(data=state == MissionState.FOLLOW))
        self._record_pub.publish(Bool(data=state == MissionState.FOLLOW))
        self._backtrack_pub.publish(Bool(data=state == MissionState.BACKTRACK))
        self._publish_state()

    def _publish_state(self) -> None:
        message = String()
        message.data = self._state.value
        self._state_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

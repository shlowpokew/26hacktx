#!/usr/bin/env python3
import json
import os
import socket
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ArduinoBridge(Node):
    def __init__(self) -> None:
        super().__init__("arduino_bridge")
        self._arduino_ip = os.environ.get("ARDUINO_IP", "").strip()
        self._arduino_port = int(os.environ.get("ARDUINO_PORT", "4210"))
        self._local_port = int(os.environ.get("ARDUINO_LOCAL_PORT", "4211"))
        self._wheel_base = float(os.environ.get("DRIVE_WHEEL_BASE_M", "0.32"))
        self._max_linear_speed = float(os.environ.get("DRIVE_MAX_LINEAR_MPS", "0.5"))
        self._cmd_vel_timeout = float(os.environ.get("CMD_VEL_TIMEOUT_SEC", "0.5"))
        if self._wheel_base <= 0 or self._max_linear_speed <= 0 or self._cmd_vel_timeout <= 0:
            raise ValueError("Drive dimensions, speed, and command timeout must be positive")
        self._last_cmd_vel_at: Optional[float] = None
        self._cmd_vel_timed_out = True
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", self._local_port))
        self._socket.setblocking(False)

        self._telemetry = self.create_publisher(String, "/arduino/telemetry", 10)
        self._commands = self.create_subscription(
            String, "/arduino/command", self._send_command, 10
        )
        self._cmd_vel = self.create_subscription(Twist, "/cmd_vel", self._send_cmd_vel, 10)
        self.create_timer(0.05, self._receive_messages)
        self.create_timer(0.25, self._send_ping)
        self.create_timer(0.1, self._check_cmd_vel_timeout)
        if self._arduino_ip:
            self.get_logger().info(
                f"Arduino UDP bridge listening on {self._local_port}; "
                f"sending to {self._arduino_ip}:{self._arduino_port}"
            )
        else:
            self.get_logger().warn(
                "ARDUINO_IP is not set; listening for telemetry only. "
                "Set it before publishing commands."
            )

    def _send(self, payload: dict) -> None:
        if not self._arduino_ip:
            self.get_logger().warn("Cannot send to Arduino: ARDUINO_IP is not set")
            return
        message = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
        try:
            self._socket.sendto(message, (self._arduino_ip, self._arduino_port))
        except OSError as error:
            self.get_logger().error(f"Could not send UDP message to Arduino: {error}")

    def _send_command(self, message: String) -> None:
        self._send({"type": "command", "data": message.data})

    def _send_cmd_vel(self, message: Twist) -> None:
        half_track = self._wheel_base / 2.0
        left = message.linear.x - message.angular.z * half_track
        right = message.linear.x + message.angular.z * half_track
        peak = max(abs(left), abs(right))
        if peak > self._max_linear_speed:
            ratio = self._max_linear_speed / peak
            left *= ratio
            right *= ratio
        left_pwm = round(left / self._max_linear_speed * 255)
        right_pwm = round(right / self._max_linear_speed * 255)
        self._send({"type": "command", "data": f"drive {left_pwm} {right_pwm}"})
        self._last_cmd_vel_at = time.monotonic()
        self._cmd_vel_timed_out = False

    def _check_cmd_vel_timeout(self) -> None:
        if self._last_cmd_vel_at is None or self._cmd_vel_timed_out:
            return
        if time.monotonic() - self._last_cmd_vel_at > self._cmd_vel_timeout:
            self._send({"type": "command", "data": "drive 0 0"})
            self._cmd_vel_timed_out = True
            self.get_logger().warn("No /cmd_vel received before timeout; stopping motors")

    def _send_ping(self) -> None:
        if self._arduino_ip:
            self._send({"type": "ping"})

    def _receive_messages(self) -> None:
        while True:
            try:
                data, address = self._socket.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError as error:
                self.get_logger().error(f"Could not receive UDP message: {error}")
                return
            try:
                text = data.decode("utf-8").strip()
            except UnicodeDecodeError as error:
                self.get_logger().error(f"Ignoring non-UTF-8 Arduino message: {error}")
                continue
            if not text:
                continue
            self.get_logger().debug(f"Received Arduino message from {address}: {text}")
            output = String()
            output.data = text
            self._telemetry.publish(output)

    def destroy_node(self) -> bool:
        self._socket.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import json
import os
import socket
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String


class ArduinoBridge(Node):
    def __init__(self) -> None:
        super().__init__("arduino_bridge")
        self._arduino_ip = os.environ.get("ARDUINO_IP", "").strip()
        self._arduino_port = int(os.environ.get("ARDUINO_PORT", "4210"))
        self._local_port = int(os.environ.get("ARDUINO_LOCAL_PORT", "4211"))
        self._wheel_base = float(os.environ.get("DRIVE_WHEEL_BASE_M", "0.32"))
        self._max_linear_speed = float(os.environ.get("DRIVE_MAX_LINEAR_MPS", "0.5"))
        self._cmd_vel_timeout = float(os.environ.get("CMD_VEL_TIMEOUT_SEC", "0.5"))
        self._encoder_timeout = float(os.environ.get("ENCODER_FEEDBACK_TIMEOUT_SEC", "0.3"))
        self._pid_kp = float(os.environ.get("DRIVE_PID_KP", "100.0"))
        self._pid_ki = float(os.environ.get("DRIVE_PID_KI", "40.0"))
        self._pid_kd = float(os.environ.get("DRIVE_PID_KD", "0.0"))
        self._integral_limit = float(os.environ.get("DRIVE_PID_INTEGRAL_LIMIT", "0.5"))
        if min(self._wheel_base, self._max_linear_speed, self._cmd_vel_timeout,
               self._encoder_timeout, self._integral_limit) <= 0:
            raise ValueError("Drive dimensions, speed, and command timeout must be positive")
        self._last_cmd_vel_at: Optional[float] = None
        self._last_wheel_velocity_at: Optional[float] = None
        self._last_scan_at: Optional[float] = None
        self._target_wheel_speeds = [0.0, 0.0]
        self._measured_wheel_speeds = [0.0, 0.0]
        self._pid_integrals = [0.0, 0.0]
        self._previous_errors = [0.0, 0.0]
        self._previous_control_at = time.monotonic()
        self._cmd_vel_timed_out = True
        self._stop_sent = False
        self._encoder_timeout_reported = False
        self._safety_warning = ""
        self._mission_mode = "S0_IDLE"
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", self._local_port))
        self._socket.setblocking(False)

        self._telemetry = self.create_publisher(String, "/arduino/telemetry", 10)
        self._safety_publisher = self.create_publisher(String, "/safety/warning", 10)
        self._mission_state = self.create_subscription(
            String, "/mission/state", self._set_mission_state, 10
        )
        self._commands = self.create_subscription(
            String, "/arduino/command", self._send_command, 10
        )
        self._cmd_vel = self.create_subscription(Twist, "/cmd_vel", self._send_cmd_vel, 10)
        self._wheel_velocity = self.create_subscription(
            Float32MultiArray, "/wheel/velocity", self._set_wheel_velocity, 10
        )
        self._scan = self.create_subscription(LaserScan, "/scan", self._set_scan, 10)
        self.create_timer(0.05, self._receive_messages)
        self.create_timer(0.25, self._send_ping)
        self.create_timer(0.05, self._control_motors)
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
        self._target_wheel_speeds = [left, right]
        self._last_cmd_vel_at = time.monotonic()
        self._cmd_vel_timed_out = False
        self._stop_sent = False

    def _set_scan(self, _message: LaserScan) -> None:
        self._last_scan_at = time.monotonic()

    def _set_mission_state(self, message: String) -> None:
        self._mission_mode = message.data
        if self._mission_mode == "S0_IDLE":
            self._set_safety_warning("")

    def _set_safety_warning(self, warning: str) -> None:
        if warning == self._safety_warning:
            return
        self._safety_warning = warning
        message = String()
        message.data = warning
        self._safety_publisher.publish(message)

    def _set_wheel_velocity(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        self._measured_wheel_speeds = [float(message.data[0]), float(message.data[1])]
        self._last_wheel_velocity_at = time.monotonic()
        self._encoder_timeout_reported = False

    def _control_motors(self) -> None:
        now = time.monotonic()
        dt = max(0.001, min(now - self._previous_control_at, 0.25))
        self._previous_control_at = now
        command_stale = (
            self._last_cmd_vel_at is None
            or now - self._last_cmd_vel_at > self._cmd_vel_timeout
        )
        mode_active = self._mission_mode != "S0_IDLE"
        if command_stale and (mode_active or self._last_cmd_vel_at is not None):
            command_was_moving = any(
                abs(speed) > 1e-3 for speed in self._target_wheel_speeds
            )
            if mode_active or command_was_moving:
                self._set_safety_warning("Command timeout")
            elif self._safety_warning == "Command timeout":
                self._set_safety_warning("")
        if command_stale:
            self._target_wheel_speeds = [0.0, 0.0]
            if not self._cmd_vel_timed_out:
                self._cmd_vel_timed_out = True
                self.get_logger().warn("No /cmd_vel received before timeout; stopping motors")

        moving = any(abs(speed) > 1e-3 for speed in self._target_wheel_speeds)
        scan_stale = self._last_scan_at is None or now - self._last_scan_at > 0.5
        if mode_active and scan_stale:
            self._set_safety_warning("LiDAR scan timeout")
        if moving and scan_stale:
            self._reset_pid()
            if not self._stop_sent:
                self._send({"type": "command", "data": "drive 0 0"})
                self._stop_sent = True
            return
        encoder_stale = (
            self._last_wheel_velocity_at is None
            or now - self._last_wheel_velocity_at > self._encoder_timeout
        )
        if moving and encoder_stale:
            self._set_safety_warning("Encoder feedback timeout")
            self._reset_pid()
            if not self._stop_sent:
                self._send({"type": "command", "data": "drive 0 0"})
                self._stop_sent = True
            if not self._encoder_timeout_reported:
                self.get_logger().error("Wheel encoder feedback stale; stopping motors")
                self._encoder_timeout_reported = True
            return

        if not moving:
            if not mode_active or (not command_stale and not scan_stale):
                self._set_safety_warning("")
            self._reset_pid()
            if not self._stop_sent:
                self._send({"type": "command", "data": "drive 0 0"})
                self._stop_sent = True
            return

        left_pwm = self._pid_pwm(0, dt)
        right_pwm = self._pid_pwm(1, dt)
        self._set_safety_warning("")
        self._stop_sent = False
        self._send({
            "type": "command",
            "data": f"drive {left_pwm} {right_pwm}",
        })

    def _pid_pwm(self, wheel: int, dt: float) -> int:
        target = self._target_wheel_speeds[wheel]
        measured = self._measured_wheel_speeds[wheel]
        error = target - measured
        derivative = (error - self._previous_errors[wheel]) / dt
        candidate_integral = max(
            -self._integral_limit,
            min(self._integral_limit, self._pid_integrals[wheel] + error * dt),
        )
        feed_forward = target / self._max_linear_speed * 255.0
        candidate = (
            feed_forward + self._pid_kp * error
            + self._pid_ki * candidate_integral + self._pid_kd * derivative
        )
        if abs(candidate) <= 255.0 or candidate * error < 0:
            self._pid_integrals[wheel] = candidate_integral
        self._previous_errors[wheel] = error
        output = (
            feed_forward + self._pid_kp * error
            + self._pid_ki * self._pid_integrals[wheel] + self._pid_kd * derivative
        )
        return round(max(-255.0, min(255.0, output)))

    def _reset_pid(self) -> None:
        self._pid_integrals = [0.0, 0.0]
        self._previous_errors = [0.0, 0.0]

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
        self._send({"type": "command", "data": "drive 0 0"})
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

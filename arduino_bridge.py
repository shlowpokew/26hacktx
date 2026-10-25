#!/usr/bin/env python3
"""Drive the Arduino Uno over USB serial.

Every 50 ms this sends one control frame: wheel PWM from the /cmd_vel PID
loop, the mission state, and the nearest LiDAR return per 45 degree sector,
which the Uno uses for its own collision stop. The frame encoding lives in
uno_link.py. Wheel encoder counts from the Uno go out on /wheel/ticks.
"""

import os
import termios
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int64MultiArray, String

from uno_link import (
    MISSION_CODES,
    NO_RETURN,
    SECTORS,
    encode_control,
    parse_encoder_line,
    parse_manual_command,
    sector_minimums,
    split_lines,
)


SCAN_TIMEOUT_SEC = 0.5
# Opening the port resets the Uno; give its bootloader time to hand over.
UNO_BOOT_SEC = 2.0


class ArduinoBridge(Node):
    def __init__(self) -> None:
        super().__init__("arduino_bridge")
        self._port = os.environ.get("UNO_SERIAL_PORT", "/dev/uno")
        self._baud = int(os.environ.get("UNO_BAUD", "57600"))
        self._yaw = float(os.environ.get("LIDAR_YAW_RAD", "0.0"))
        self._manual_duration = float(os.environ.get("MANUAL_COMMAND_SEC", "2.0"))
        self._wheel_base = float(os.environ.get("DRIVE_WHEEL_BASE_M", "0.32"))
        self._max_linear_speed = float(os.environ.get("DRIVE_MAX_LINEAR_MPS", "0.5"))
        self._cmd_vel_timeout = float(os.environ.get("CMD_VEL_TIMEOUT_SEC", "0.5"))
        self._encoder_timeout = float(os.environ.get("ENCODER_FEEDBACK_TIMEOUT_SEC", "0.3"))
        self._pid_kp = float(os.environ.get("DRIVE_PID_KP", "100.0"))
        self._pid_ki = float(os.environ.get("DRIVE_PID_KI", "40.0"))
        self._pid_kd = float(os.environ.get("DRIVE_PID_KD", "0.0"))
        self._integral_limit = float(os.environ.get("DRIVE_PID_INTEGRAL_LIMIT", "0.5"))
        if min(self._wheel_base, self._max_linear_speed, self._cmd_vel_timeout,
               self._encoder_timeout, self._integral_limit, self._manual_duration) <= 0:
            raise ValueError("Drive dimensions, speed, and timeouts must be positive")
        self._last_cmd_vel_at: Optional[float] = None
        self._last_wheel_velocity_at: Optional[float] = None
        self._last_scan_at: Optional[float] = None
        self._sectors = [NO_RETURN] * SECTORS
        self._target_wheel_speeds = [0.0, 0.0]
        self._measured_wheel_speeds = [0.0, 0.0]
        self._pid_integrals = [0.0, 0.0]
        self._previous_errors = [0.0, 0.0]
        self._previous_control_at = time.monotonic()
        self._cmd_vel_timed_out = True
        self._encoder_timeout_reported = False
        self._manual: Optional[tuple[int, int, float]] = None
        self._safety_warning = ""
        self._mission_mode = "S0_IDLE"
        self._fd: Optional[int] = None
        self._opened_at = 0.0
        self._next_open_attempt = 0.0
        self._rx = bytearray()

        self._telemetry = self.create_publisher(String, "/arduino/telemetry", 10)
        self._ticks = self.create_publisher(Int64MultiArray, "/wheel/ticks", 10)
        self._safety_publisher = self.create_publisher(String, "/safety/warning", 10)
        self._mission_state = self.create_subscription(
            String, "/mission/state", self._set_mission_state, 10
        )
        self._commands = self.create_subscription(
            String, "/arduino/command", self._set_manual_command, 10
        )
        self._cmd_vel = self.create_subscription(Twist, "/cmd_vel", self._send_cmd_vel, 10)
        self._wheel_velocity = self.create_subscription(
            Float32MultiArray, "/wheel/velocity", self._set_wheel_velocity, 10
        )
        self._scan = self.create_subscription(LaserScan, "/scan", self._set_scan, 10)
        self.create_timer(0.05, self._receive_messages)
        self.create_timer(0.05, self._control_motors)
        self.get_logger().info(f"Arduino Uno bridge using {self._port} at {self._baud} baud")

    def _set_manual_command(self, message: String) -> None:
        wheels = parse_manual_command(message.data)
        if wheels is None:
            self.get_logger().warn(f"Unknown /arduino/command: {message.data!r}")
            return
        if wheels == (0, 0):
            self._manual = None
        else:
            self._manual = (*wheels, time.monotonic() + self._manual_duration)

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
        self._manual = None

    def _set_scan(self, message: LaserScan) -> None:
        self._sectors = sector_minimums(
            message.angle_min, message.angle_increment, message.ranges, self._yaw
        )
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
        left, right = self._wheel_pwm(now, dt)
        if self._manual is not None:
            if now <= self._manual[2]:
                left, right = self._manual[0], self._manual[1]
            else:
                self._manual = None
        scan_fresh = self._last_scan_at is not None and now - self._last_scan_at <= SCAN_TIMEOUT_SEC
        self._write(encode_control(
            scan_fresh,
            bool(self._safety_warning),
            MISSION_CODES.get(self._mission_mode, 0),
            left,
            right,
            self._sectors,
        ))

    def _wheel_pwm(self, now: float, dt: float) -> tuple[int, int]:
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
        scan_stale = self._last_scan_at is None or now - self._last_scan_at > SCAN_TIMEOUT_SEC
        if mode_active and scan_stale:
            self._set_safety_warning("LiDAR scan timeout")
        if moving and scan_stale:
            self._reset_pid()
            return 0, 0
        encoder_stale = (
            self._last_wheel_velocity_at is None
            or now - self._last_wheel_velocity_at > self._encoder_timeout
        )
        if moving and encoder_stale:
            self._set_safety_warning("Encoder feedback timeout")
            self._reset_pid()
            if not self._encoder_timeout_reported:
                self.get_logger().error("Wheel encoder feedback stale; stopping motors")
                self._encoder_timeout_reported = True
            return 0, 0

        if not moving:
            if not mode_active or (not command_stale and not scan_stale):
                self._set_safety_warning("")
            self._reset_pid()
            return 0, 0

        self._set_safety_warning("")
        return self._pid_pwm(0, dt), self._pid_pwm(1, dt)

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

    def _open(self) -> bool:
        if self._fd is not None:
            return True
        now = time.monotonic()
        if now < self._next_open_attempt:
            return False
        self._next_open_attempt = now + 2.0
        fd = None
        try:
            fd = os.open(self._port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(fd)
            speed = getattr(termios, f"B{self._baud}")
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = speed
            attrs[5] = speed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except (OSError, AttributeError, termios.error) as error:
            if fd is not None:
                os.close(fd)
            self.get_logger().warn(
                f"Uno not available on {self._port} ({error}); is it plugged in and "
                "UNO_DEVICE set in .env? Retrying.",
                throttle_duration_sec=30.0,
            )
            return False
        self._fd = fd
        self._opened_at = now
        self.get_logger().info(f"Uno connected on {self._port}")
        return True

    def _close(self, error: OSError) -> None:
        self.get_logger().warn(f"Lost Uno serial link: {error}")
        if self._fd is not None:
            os.close(self._fd)
        self._fd = None
        self._rx.clear()

    def _write(self, frame: bytes) -> None:
        if not self._open() or time.monotonic() - self._opened_at < UNO_BOOT_SEC:
            return
        try:
            os.write(self._fd, frame)
        except BlockingIOError:
            pass
        except OSError as error:
            self._close(error)

    def _receive_messages(self) -> None:
        if self._fd is None:
            return
        try:
            chunk = os.read(self._fd, 512)
        except BlockingIOError:
            return
        except OSError as error:
            self._close(error)
            return
        for text in split_lines(self._rx, chunk):
            encoder = parse_encoder_line(text)
            if encoder is not None:
                ticks = Int64MultiArray()
                ticks.data = list(encoder)
                self._ticks.publish(ticks)
                continue
            output = String()
            output.data = text
            self._telemetry.publish(output)

    def destroy_node(self) -> bool:
        if self._fd is not None:
            try:
                os.write(self._fd, encode_control(False, False, 0, 0, 0, [NO_RETURN] * SECTORS))
            except OSError:
                pass
            os.close(self._fd)
            self._fd = None
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

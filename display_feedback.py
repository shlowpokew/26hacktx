#!/usr/bin/env python3
"""Animate eyes and mission status on the Grove 16x2 RGB I2C LCD."""

import math
import os
import random
import time
from typing import Optional

from smbus2 import SMBus, i2c_msg
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


EYE_OPEN = {
    -1: [0b00000, 0b01110, 0b10001, 0b10111, 0b10111, 0b10001, 0b01110, 0b00000],
    0: [0b00000, 0b01110, 0b10001, 0b10101, 0b10101, 0b10001, 0b01110, 0b00000],
    1: [0b00000, 0b01110, 0b10001, 0b11101, 0b11101, 0b10001, 0b01110, 0b00000],
}
EYE_CLOSED = [0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000]
EYE_WIDE = [0b01110, 0b10001, 0b10101, 0b10101, 0b10101, 0b10101, 0b10001, 0b01110]


class GroveRgbLcd:
    """Small Seeed Grove RGB LCD driver supporting V4 and V5 RGB addresses."""

    LCD_ADDRESS = 0x3E

    def __init__(self, bus: SMBus) -> None:
        self._bus = bus
        self._rgb_address, self._rgb_v5 = self._find_rgb_controller()
        time.sleep(0.05)
        self._command(0x28)  # two lines, 5x8 font
        time.sleep(0.005)
        self._command(0x28)
        self._command(0x0C)  # display on, cursor off
        self._command(0x01)
        time.sleep(0.002)
        self._command(0x06)  # advance cursor, no display shift
        if self._rgb_v5:
            self._rgb_write(0x00, 0x07)  # reset V5 RGB controller
            time.sleep(0.001)
            self._rgb_write(0x04, 0x15)  # enable RGB output channels
        else:
            self._rgb_write(0x00, 0x00)
            self._rgb_write(0x08, 0xFF)
            self._rgb_write(0x01, 0x20)

    def _find_rgb_controller(self) -> tuple[int, bool]:
        for address, is_v5 in ((0x30, True), (0x62, False)):
            try:
                write = i2c_msg.write(address, [0x00])
                read = i2c_msg.read(address, 1)
                self._bus.i2c_rdwr(write, read)
                return address, is_v5
            except OSError:
                continue
        raise OSError("Grove LCD RGB controller not found at 0x30 or 0x62")

    def _command(self, value: int) -> None:
        self._bus.write_byte_data(self.LCD_ADDRESS, 0x80, value)

    def _rgb_write(self, register: int, value: int) -> None:
        self._bus.write_byte_data(self._rgb_address, register, value)

    def set_color(self, red: int, green: int, blue: int) -> None:
        if self._rgb_v5:
            registers = (0x06, 0x07, 0x08)
        else:
            registers = (0x04, 0x03, 0x02)
        for register, value in zip(registers, (red, green, blue)):
            self._rgb_write(register, max(0, min(255, int(value))))

    def set_char(self, slot: int, rows: list[int]) -> None:
        self._command(0x40 | ((slot & 0x07) << 3))
        self._bus.write_i2c_block_data(self.LCD_ADDRESS, 0x40, rows[:8])

    def write_line(self, row: int, text: str) -> None:
        self._command(0x80 | (0x40 if row else 0x00))
        data = text[:16].ljust(16).encode("ascii", "replace")
        self._bus.write_i2c_block_data(self.LCD_ADDRESS, 0x40, list(data))


class DisplayFeedback(Node):
    def __init__(self) -> None:
        super().__init__("display_feedback")
        self._bus_number = int(os.environ.get("LCD_I2C_BUS", "1"))
        self._mode = "S0_IDLE"
        self._warning = ""
        self._target_angle = 0.0
        self._bus: Optional[SMBus] = None
        self._lcd: Optional[GroveRgbLcd] = None
        self._last_line1: Optional[str] = None
        self._last_line2: Optional[str] = None
        self._last_color: Optional[tuple[int, int, int]] = None
        self._last_gaze: Optional[int] = None
        self._last_glyph: Optional[list[int]] = None
        self._next_connect_attempt = 0.0
        self._next_blink = time.monotonic() + random.uniform(2.0, 6.0)
        self._blink_until = 0.0
        self._next_idle_gaze = time.monotonic() + random.uniform(0.8, 2.5)
        self._idle_gaze = 0
        self.create_subscription(String, "/mission/state", self._set_mode, 10)
        self.create_subscription(String, "/safety/warning", self._set_warning, 10)
        self.create_subscription(PoseStamped, "/human_target", self._set_target, 10)
        self.create_timer(1.0 / 30.0, self._render)

    def _set_mode(self, message: String) -> None:
        self._mode = message.data

    def _set_warning(self, message: String) -> None:
        self._warning = message.data

    def _set_target(self, message: PoseStamped) -> None:
        self._target_angle = math.atan2(
            message.pose.position.y, message.pose.position.x
        )

    def _connect(self) -> bool:
        try:
            self._bus = SMBus(self._bus_number)
            self._lcd = GroveRgbLcd(self._bus)
            self.get_logger().info(
                f"Grove RGB LCD ready on I2C bus {self._bus_number} "
                f"(RGB address 0x{self._lcd._rgb_address:02x})"
            )
            return True
        except OSError as error:
            if self._bus is not None:
                self._bus.close()
            self._bus = None
            self._lcd = None
            self.get_logger().warn(f"LCD unavailable; retrying: {error}")
            return False

    def _render(self) -> None:
        try:
            self._render_frame()
        except OSError as error:
            self.get_logger().warn(f"LCD I2C write failed; reconnecting: {error}")
            if self._bus is not None:
                self._bus.close()
            self._bus = None
            self._lcd = None
            self._next_connect_attempt = time.monotonic() + 2.0

    def _render_frame(self) -> None:
        now = time.monotonic()
        if self._lcd is None:
            if now < self._next_connect_attempt:
                return
            self._next_connect_attempt = now + 2.0
            if not self._connect():
                return
        assert self._lcd is not None

        if now >= self._next_blink:
            self._blink_until = now + 0.16
            self._next_blink = now + random.uniform(2.0, 6.0)
        blinking = now < self._blink_until
        if self._mode == "S0_IDLE":
            if now >= self._next_idle_gaze:
                self._idle_gaze = random.choice((-1, 0, 1))
                self._next_idle_gaze = now + random.uniform(0.8, 2.5)
            gaze = self._idle_gaze
            line2, color = "READY / PRESS A", (0, 100, 180)
            glyph = EYE_CLOSED if blinking else EYE_OPEN[gaze]
        elif self._mode == "S1_FOLLOW_MAP":
            gaze = -1 if self._target_angle < -0.15 else 1 if self._target_angle > 0.15 else 0
            line2, color = "FOLLOW  /  1.0m", (0, 180, 40)
            glyph = EYE_CLOSED if blinking else EYE_OPEN[gaze]
        elif self._mode == "S2_BACKTRACK":
            gaze = (-1, 0, 1, 0)[int(now / 0.6) % 4]
            line2, color = "RETURNING TO BASE", (200, 100, 0)
            glyph = EYE_CLOSED if blinking else EYE_OPEN[gaze]
        else:
            gaze = 0
            flash = int(now / 0.5) % 2 == 0
            line2 = "!!! SOS !!!" if flash else "PLEASE HELP!"
            color = (255, 0, 0) if flash else (80, 0, 0)
            glyph = EYE_CLOSED if blinking else EYE_WIDE

        if self._warning:
            warning_text = {
                "LiDAR scan timeout": "STOP: SCAN STALE",
                "Command timeout": "STOP: CMD STALE",
                "Encoder feedback timeout": "STOP: ENCODER",
            }.get(self._warning, "STOP: SAFETY FAULT")
            line2 = warning_text[:16]
            color = (255, 0, 0)

        line1 = f"   {chr(0)}        {chr(1)}   "
        if glyph != self._last_glyph:
            self._lcd.set_char(0, glyph)
            self._lcd.set_char(1, glyph)
            self._last_glyph = glyph
        if (line1, line2) != (self._last_line1, self._last_line2):
            self._lcd.write_line(0, line1)
            self._lcd.write_line(1, line2)
            self._last_line1, self._last_line2 = line1, line2
        if color != self._last_color:
            self._lcd.set_color(*color)
            self._last_color = color

    def destroy_node(self) -> bool:
        if self._bus is not None:
            self._bus.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DisplayFeedback()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

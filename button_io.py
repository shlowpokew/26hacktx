#!/usr/bin/env python3
"""Publish debounced Pi hardware button events for the mission manager."""

import os

import rclpy
from gpiozero import Button
from gpiozero.pins.lgpio import LGPIOFactory
from rclpy.node import Node
from std_msgs.msg import Bool


class ButtonIo(Node):
    def __init__(self) -> None:
        super().__init__("button_io")
        chip = int(os.environ.get("GPIO_CHIP", "4"))
        self._pin_factory = LGPIOFactory(chip=chip)
        debounce = float(os.environ.get("BUTTON_DEBOUNCE_SEC", "0.05"))
        self._button_a_pub = self.create_publisher(Bool, "/hardware/button_a", 10)
        self._button_b_pub = self.create_publisher(Bool, "/hardware/button_b", 10)
        self._sos_pub = self.create_publisher(Bool, "/hardware/sos", 10)
        self._button_a = Button(
            int(os.environ.get("BUTTON_A_BCM", "5")),
            pull_up=True,
            bounce_time=debounce,
            pin_factory=self._pin_factory,
        )
        self._button_b = Button(
            int(os.environ.get("BUTTON_B_BCM", "6")),
            pull_up=True,
            bounce_time=debounce,
            hold_time=float(os.environ.get("SOS_HOLD_SEC", "3.0")),
            pin_factory=self._pin_factory,
        )
        self._button_a.when_pressed = lambda: self._publish(self._button_a_pub)
        self._button_b.when_pressed = lambda: self._publish(self._button_b_pub)
        self._button_b.when_held = lambda: self._publish(self._sos_pub)

    @staticmethod
    def _publish(publisher) -> None:
        publisher.publish(Bool(data=True))

    def destroy_node(self) -> bool:
        self._button_a.close()
        self._button_b.close()
        self._pin_factory.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ButtonIo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

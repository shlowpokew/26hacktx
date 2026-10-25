#!/usr/bin/env python3
"""Publish differential-drive odometry from two quadrature wheel encoders."""

import math
import os
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from gpiozero import RotaryEncoder
from gpiozero.pins.lgpio import LGPIOFactory


class EncoderOdometry(Node):
    def __init__(self) -> None:
        super().__init__("encoder_odometry")
        self._left_a = int(os.environ.get("LEFT_ENCODER_A_BCM", "17"))
        self._left_b = int(os.environ.get("LEFT_ENCODER_B_BCM", "18"))
        self._right_a = int(os.environ.get("RIGHT_ENCODER_A_BCM", "22"))
        self._right_b = int(os.environ.get("RIGHT_ENCODER_B_BCM", "23"))
        self._ticks_per_rev = float(os.environ.get("ENCODER_TICKS_PER_REV", "240"))
        self._wheel_radius = float(os.environ.get("DRIVE_WHEEL_RADIUS_M", "0.05"))
        self._wheel_base = float(os.environ.get("DRIVE_WHEEL_BASE_M", "0.32"))
        self._left_sign = int(os.environ.get("LEFT_ENCODER_SIGN", "1"))
        self._right_sign = int(os.environ.get("RIGHT_ENCODER_SIGN", "1"))
        self._publish_rate = float(os.environ.get("ODOM_PUBLISH_RATE_HZ", "50"))
        self._odom_frame = os.environ.get("ODOM_FRAME", "odom")
        self._base_frame = os.environ.get("BASE_FRAME", "base_link")
        if min(self._ticks_per_rev, self._wheel_radius, self._wheel_base, self._publish_rate) <= 0:
            raise ValueError("Encoder and drive dimensions and publish rate must be positive")
        if self._left_sign not in (-1, 1) or self._right_sign not in (-1, 1):
            raise ValueError("Encoder direction signs must be -1 or 1")

        gpio_chip = int(os.environ.get("GPIO_CHIP", "4"))
        self._pin_factory = LGPIOFactory(chip=gpio_chip)
        self._left_encoder = RotaryEncoder(
            self._left_a, self._left_b, max_steps=0, pin_factory=self._pin_factory
        )
        self._right_encoder = RotaryEncoder(
            self._right_a, self._right_b, max_steps=0, pin_factory=self._pin_factory
        )
        self._previous_left = self._left_encoder.steps
        self._previous_right = self._right_encoder.steps
        self._previous_time = time.monotonic()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._linear_speed = 0.0
        self._angular_speed = 0.0

        self._odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self._wheel_velocity_publisher = self.create_publisher(
            Float32MultiArray, "/wheel/velocity", 10
        )
        self._transform_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / self._publish_rate, self._update)
        self.get_logger().info(
            f"Encoder odometry at {self._publish_rate:g} Hz; "
            f"left GPIO {self._left_a}/{self._left_b}, "
            f"right GPIO {self._right_a}/{self._right_b}"
        )

    def _update(self) -> None:
        now = time.monotonic()
        dt = now - self._previous_time
        if dt <= 0:
            return

        left_steps = self._left_encoder.steps
        right_steps = self._right_encoder.steps
        left_delta = (left_steps - self._previous_left) * self._left_sign
        right_delta = (right_steps - self._previous_right) * self._right_sign
        self._previous_left = left_steps
        self._previous_right = right_steps
        self._previous_time = now

        meters_per_tick = 2.0 * math.pi * self._wheel_radius / self._ticks_per_rev
        left_distance = left_delta * meters_per_tick
        right_distance = right_delta * meters_per_tick
        wheel_velocity = Float32MultiArray()
        wheel_velocity.data = [left_distance / dt, right_distance / dt]
        self._wheel_velocity_publisher.publish(wheel_velocity)
        distance = (left_distance + right_distance) / 2.0
        heading_change = (right_distance - left_distance) / self._wheel_base
        midpoint_heading = self._yaw + heading_change / 2.0
        self._x += distance * math.cos(midpoint_heading)
        self._y += distance * math.sin(midpoint_heading)
        self._yaw = math.atan2(
            math.sin(self._yaw + heading_change),
            math.cos(self._yaw + heading_change),
        )
        self._linear_speed = distance / dt
        self._angular_speed = heading_change / dt

        stamp = self.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self._odom_frame
        odometry.child_frame_id = self._base_frame
        odometry.pose.pose.position.x = self._x
        odometry.pose.pose.position.y = self._y
        odometry.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        odometry.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        odometry.twist.twist.linear.x = self._linear_speed
        odometry.twist.twist.angular.z = self._angular_speed
        odometry.pose.covariance[0] = 0.02
        odometry.pose.covariance[7] = 0.02
        odometry.pose.covariance[35] = 0.05
        odometry.twist.covariance[0] = 0.05
        odometry.twist.covariance[35] = 0.1
        self._odom_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.rotation = odometry.pose.pose.orientation
        self._transform_broadcaster.sendTransform(transform)

    def destroy_node(self) -> bool:
        self._left_encoder.close()
        self._right_encoder.close()
        self._pin_factory.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = EncoderOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

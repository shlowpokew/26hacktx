"""Tests for arduino_bridge.py, run without ROS or an Uno.

ROS is replaced by small fakes, the clock is controlled by the test, and the
Uno is a pseudo-terminal, so these check the exact bytes the bridge writes.
"""

import math
import os
import struct
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


class FakeLogger:
    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def _log(self, level: str, message: str, **_kwargs) -> None:
        self.records.append((level, message))

    def debug(self, message: str, **kwargs) -> None:
        self._log("debug", message, **kwargs)

    def info(self, message: str, **kwargs) -> None:
        self._log("info", message, **kwargs)

    def warn(self, message: str, **kwargs) -> None:
        self._log("warn", message, **kwargs)

    def error(self, message: str, **kwargs) -> None:
        self._log("error", message, **kwargs)


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeNode:
    def __init__(self, _name: str) -> None:
        self.publishers: dict[str, FakePublisher] = {}
        self.callbacks: dict[str, object] = {}
        self._logger = FakeLogger()

    def create_publisher(self, _type, topic: str, _qos) -> FakePublisher:
        self.publishers[topic] = FakePublisher()
        return self.publishers[topic]

    def create_subscription(self, _type, topic: str, callback, _qos) -> None:
        self.callbacks[topic] = callback

    def create_timer(self, _period, _callback) -> None:
        pass

    def get_logger(self) -> FakeLogger:
        return self._logger

    def destroy_node(self) -> bool:
        return True


class String:
    def __init__(self, data: str = "") -> None:
        self.data = data


class Float32MultiArray:
    def __init__(self, data=None) -> None:
        self.data = list(data or [])


class Int64MultiArray(Float32MultiArray):
    pass


class Twist:
    def __init__(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        self.linear = types.SimpleNamespace(x=linear_x, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=angular_z)


class LaserScan:
    def __init__(self, ranges: list[float]) -> None:
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / len(ranges)
        self.ranges = ranges


def install_fake_ros() -> None:
    def module(name: str, **attrs) -> types.ModuleType:
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod
        return mod

    module("rclpy", init=lambda **_: None, spin=lambda _: None, shutdown=lambda: None)
    module("rclpy.node", Node=FakeNode)
    module("geometry_msgs")
    module("geometry_msgs.msg", Twist=Twist)
    module("sensor_msgs")
    module("sensor_msgs.msg", LaserScan=LaserScan)
    module("std_msgs")
    module("std_msgs.msg", String=String, Float32MultiArray=Float32MultiArray,
           Int64MultiArray=Int64MultiArray)


install_fake_ros()
import arduino_bridge  # noqa: E402

FRAME = struct.Struct("<2sBBBBhh8HB")
NO_RETURN = 0xFFFF


def decode_frames(data: bytes) -> list[dict]:
    frames = []
    while len(data) >= FRAME.size:
        start = data.find(b"\xa5\x5a")
        if start < 0 or len(data) - start < FRAME.size:
            break
        sync, kind, length, flags, mission, left, right, *rest = FRAME.unpack_from(data, start)
        sectors, crc = rest[:8], rest[8]
        assert (kind, length) == (0x02, 22)
        assert crc == arduino_bridge.encode_control(
            bool(flags & 1), bool(flags & 2), mission, left, right, list(sectors)
        )[-1]
        frames.append({
            "scan_fresh": bool(flags & 1),
            "warning": bool(flags & 2),
            "mission": mission,
            "left": left,
            "right": right,
            "sectors": list(sectors),
        })
        data = data[start + FRAME.size:]
    return frames


def scan_with_obstacle_ahead(distance_m: float) -> LaserScan:
    ranges = [math.nan] * 360
    ranges[180] = distance_m
    return LaserScan(ranges)


class Harness:
    """An ArduinoBridge talking to a pseudo-terminal, on a fake clock."""

    def __init__(self, env: dict | None = None, wait_for_boot: bool = True) -> None:
        self.now = 1000.0
        self.master, self.slave = os.openpty()
        os.set_blocking(self.master, False)
        environment = {"UNO_SERIAL_PORT": os.ttyname(self.slave), **(env or {})}
        self._patches = [
            mock.patch.dict(os.environ, environment),
            mock.patch.object(arduino_bridge.time, "monotonic", lambda: self.now),
        ]
        for patch in self._patches:
            patch.start()
        self.node = arduino_bridge.ArduinoBridge()
        if wait_for_boot:
            self.node._control_motors()  # opens the port
            self.advance(arduino_bridge.UNO_BOOT_SEC + 0.01)
            self.sent()

    def close(self) -> None:
        for patch in self._patches:
            patch.stop()
        if self.node._fd is not None:
            os.close(self.node._fd)
            self.node._fd = None
        os.close(self.master)
        os.close(self.slave)

    def advance(self, seconds: float) -> None:
        self.now += seconds

    def publish(self, topic: str, message) -> None:
        self.node.callbacks[topic](message)

    def published(self, topic: str) -> list:
        return [m.data for m in self.node.publishers[topic].messages]

    def sent(self) -> list[dict]:
        data = b""
        while True:
            try:
                chunk = os.read(self.master, 4096)
            except BlockingIOError:
                break
            if not chunk:
                break
            data += chunk
        return decode_frames(data)

    def tick(self, cmd_vel: Twist | None = None, scan: bool = True, encoders: bool = True) -> dict:
        """Advance one 50 ms control period with healthy inputs; return the frame."""
        self.advance(0.05)
        if cmd_vel is not None:
            self.publish("/cmd_vel", cmd_vel)
        if scan:
            self.publish("/scan", scan_with_obstacle_ahead(2.0))
        if encoders:
            self.publish("/wheel/velocity", Float32MultiArray([0.0, 0.0]))
        self.node._control_motors()
        frames = self.sent()
        assert len(frames) == 1, frames
        return frames[0]


class BridgeTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.bridge = Harness()

    def tearDown(self) -> None:
        self.bridge.close()


class LinkTest(unittest.TestCase):
    def test_waits_for_the_uno_bootloader_before_sending(self) -> None:
        bridge = Harness(wait_for_boot=False)
        try:
            bridge.node._control_motors()
            bridge.advance(1.0)
            bridge.node._control_motors()
            self.assertEqual(bridge.sent(), [])
            bridge.advance(arduino_bridge.UNO_BOOT_SEC)
            bridge.node._control_motors()
            self.assertEqual(len(bridge.sent()), 1)
        finally:
            bridge.close()

    def test_missing_port_does_not_crash(self) -> None:
        bridge = Harness(env={"UNO_SERIAL_PORT": "/dev/null"}, wait_for_boot=False)
        try:
            bridge.node._control_motors()
            self.assertIsNone(bridge.node._fd)
            warnings = [m for level, m in bridge.node.get_logger().records if level == "warn"]
            self.assertTrue(any("UNO_DEVICE" in m for m in warnings), warnings)
        finally:
            bridge.close()


class HeartbeatTest(BridgeTestCase):
    def test_idle_sends_stop_frames(self) -> None:
        frame = self.bridge.tick(scan=False, encoders=False)
        self.assertEqual((frame["left"], frame["right"], frame["mission"]), (0, 0, 0))
        self.assertFalse(frame["scan_fresh"])

    def test_scan_sectors_and_freshness(self) -> None:
        self.bridge.publish("/scan", scan_with_obstacle_ahead(0.5))
        frame = self.bridge.tick(scan=False)
        self.assertTrue(frame["scan_fresh"])
        self.assertEqual(frame["sectors"][0], 50)
        self.assertEqual(frame["sectors"][4], NO_RETURN)
        self.bridge.advance(arduino_bridge.SCAN_TIMEOUT_SEC)
        self.assertFalse(self.bridge.tick(scan=False)["scan_fresh"])

    def test_mission_state_is_forwarded(self) -> None:
        for state, code in [("S1_FOLLOW_MAP", 1), ("S2_BACKTRACK", 2), ("S3_SOS", 3),
                            ("S0_IDLE", 0), ("SOMETHING_NEW", 0)]:
            with self.subTest(state=state):
                self.bridge.publish("/mission/state", String(state))
                self.assertEqual(self.bridge.tick()["mission"], code)

    def test_telemetry_from_the_uno_is_published(self) -> None:
        os.write(self.bridge.master, b"st=idle mission=idle\r\nst=dri")
        self.bridge.node._receive_messages()
        self.assertEqual(self.bridge.published("/arduino/telemetry"), ["st=idle mission=idle"])

    def test_encoder_counts_go_to_wheel_ticks_not_telemetry(self) -> None:
        os.write(self.bridge.master, b"enc 1020 15 -7\r\nst=idle\r\n")
        self.bridge.node._receive_messages()
        self.assertEqual(self.bridge.published("/wheel/ticks"), [[1020, 15, -7]])
        self.assertEqual(self.bridge.published("/arduino/telemetry"), ["st=idle"])

    def test_shutdown_sends_a_stop_frame(self) -> None:
        self.bridge.publish("/arduino/command", String("motor forward 100"))
        self.bridge.tick()
        self.bridge.node.destroy_node()
        frames = self.bridge.sent()
        self.assertEqual((frames[-1]["left"], frames[-1]["right"]), (0, 0))


class DriveTest(BridgeTestCase):
    def test_forward(self) -> None:
        frame = self.bridge.tick(Twist(linear_x=0.2))
        self.assertGreater(frame["left"], 0)
        self.assertEqual(frame["left"], frame["right"])

    def test_turning_left_speeds_up_the_right_wheel(self) -> None:
        frame = self.bridge.tick(Twist(linear_x=0.1, angular_z=1.0))
        self.assertGreater(frame["right"], frame["left"])

    def test_reverse(self) -> None:
        frame = self.bridge.tick(Twist(linear_x=-0.2))
        self.assertLess(frame["left"], 0)
        self.assertLess(frame["right"], 0)

    def test_pwm_stays_in_range(self) -> None:
        for _ in range(40):
            frame = self.bridge.tick(Twist(linear_x=5.0, angular_z=-5.0))
            self.assertLessEqual(abs(frame["left"]), 255)
            self.assertLessEqual(abs(frame["right"]), 255)


class SafetyTest(BridgeTestCase):
    def test_cmd_vel_timeout_stops_and_warns(self) -> None:
        self.bridge.publish("/mission/state", String("S1_FOLLOW_MAP"))
        self.assertGreater(self.bridge.tick(Twist(linear_x=0.2))["left"], 0)
        self.bridge.advance(arduino_bridge.SCAN_TIMEOUT_SEC)  # past CMD_VEL_TIMEOUT_SEC too
        frame = self.bridge.tick()
        self.assertEqual((frame["left"], frame["right"]), (0, 0))
        self.assertEqual(self.bridge.published("/safety/warning")[-1], "Command timeout")
        self.assertTrue(frame["warning"], "warning flag reaches the Uno during a mission")

    def test_cmd_vel_timeout_in_idle_still_stops(self) -> None:
        self.bridge.tick(Twist(linear_x=0.2))
        self.bridge.advance(arduino_bridge.SCAN_TIMEOUT_SEC)
        frame = self.bridge.tick()
        self.assertEqual((frame["left"], frame["right"]), (0, 0))

    def test_missing_encoder_feedback_stops(self) -> None:
        frame = self.bridge.tick(Twist(linear_x=0.2), encoders=False)
        self.assertEqual((frame["left"], frame["right"]), (0, 0))
        self.assertIn("Encoder feedback timeout", self.bridge.published("/safety/warning"))

    def test_stale_scan_stops(self) -> None:
        frame = self.bridge.tick(Twist(linear_x=0.2), scan=False)
        self.assertEqual((frame["left"], frame["right"]), (0, 0))
        self.assertFalse(frame["scan_fresh"])

    def test_warning_clears_when_healthy_again(self) -> None:
        self.bridge.tick(Twist(linear_x=0.2), encoders=False)
        self.bridge.tick(Twist(linear_x=0.2))
        self.assertEqual(self.bridge.published("/safety/warning")[-1], "")


class ManualCommandTest(BridgeTestCase):
    def test_runs_for_its_duration_then_stops(self) -> None:
        self.bridge.publish("/arduino/command", String("drive 120 -80"))
        frame = self.bridge.tick()
        self.assertEqual((frame["left"], frame["right"]), (120, -80))
        self.bridge.advance(2.0)
        frame = self.bridge.tick()
        self.assertEqual((frame["left"], frame["right"]), (0, 0))

    def test_duration_is_configurable(self) -> None:
        self.bridge.close()
        self.bridge = Harness(env={"MANUAL_COMMAND_SEC": "0.5"})
        self.bridge.publish("/arduino/command", String("motor forward 100"))
        self.bridge.advance(0.5)
        self.assertEqual(self.bridge.tick()["left"], 0)

    def test_motor_stop_cancels(self) -> None:
        self.bridge.publish("/arduino/command", String("motor reverse 100"))
        self.assertEqual(self.bridge.tick()["left"], -100)
        self.bridge.publish("/arduino/command", String("motor stop"))
        self.assertEqual(self.bridge.tick()["left"], 0)

    def test_cmd_vel_takes_over(self) -> None:
        self.bridge.publish("/arduino/command", String("motor reverse 100"))
        self.bridge.tick()
        frame = self.bridge.tick(Twist(linear_x=0.2))
        self.assertGreater(frame["left"], 0)

    def test_unknown_command_is_ignored(self) -> None:
        self.bridge.publish("/arduino/command", String("do a flip"))
        self.assertEqual(self.bridge.tick()["left"], 0)
        warnings = [m for level, m in self.bridge.node.get_logger().records if level == "warn"]
        self.assertTrue(any("do a flip" in m for m in warnings))


if __name__ == "__main__":
    unittest.main()

"""Tests for uno_link.py: the Pi side of the Uno USB protocol."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from uno_link import (  # noqa: E402
    NO_RETURN,
    crc8,
    encode_control,
    parse_manual_command,
    sector_minimums,
    split_lines,
)


def ranges_with(points: dict[int, float], count: int = 360) -> list[float]:
    """A 1-degree scan starting at -180 degrees, NaN except `points`."""
    ranges = [math.nan] * count
    for degrees, distance in points.items():
        ranges[degrees + 180] = distance
    return ranges


def sectors(ranges: list[float], yaw: float = 0.0) -> list[int]:
    return sector_minimums(-math.pi, math.radians(1), ranges, yaw)


class Crc8Test(unittest.TestCase):
    def test_standard_check_value(self) -> None:
        # CRC-8/SMBUS check value from the CRC catalogue.
        self.assertEqual(crc8(b"123456789"), 0xF4)

    def test_empty(self) -> None:
        self.assertEqual(crc8(b""), 0)


class EncodeControlTest(unittest.TestCase):
    def test_matches_the_rust_decoder_test_vector(self) -> None:
        # Same bytes as protocol::tests::matches_python_encoder in the brain crate.
        frame = encode_control(True, False, 1, 140, -120, [120, 300, NO_RETURN, 45, 80, 999, 20, 65])
        self.assertEqual(frame, bytes([
            0xa5, 0x5a, 0x02, 0x16, 0x01, 0x01, 0x8c, 0x00, 0x88, 0xff, 0x78, 0x00, 0x2c, 0x01,
            0xff, 0xff, 0x2d, 0x00, 0x50, 0x00, 0xe7, 0x03, 0x14, 0x00, 0x41, 0x00, 0xdf,
        ]))

    def test_flags(self) -> None:
        sectors_cm = [NO_RETURN] * 8
        self.assertEqual(encode_control(False, False, 0, 0, 0, sectors_cm)[4], 0x00)
        self.assertEqual(encode_control(True, False, 0, 0, 0, sectors_cm)[4], 0x01)
        self.assertEqual(encode_control(False, True, 0, 0, 0, sectors_cm)[4], 0x02)
        self.assertEqual(encode_control(True, True, 0, 0, 0, sectors_cm)[4], 0x03)

    def test_length_and_crc_cover_the_body(self) -> None:
        frame = encode_control(True, True, 3, -255, 255, [1, 2, 3, 4, 5, 6, 7, 8])
        self.assertEqual(len(frame), 27)
        self.assertEqual(frame[3], len(frame) - 5)
        self.assertEqual(frame[-1], crc8(frame[2:-1]))

    def test_rejects_out_of_range_values(self) -> None:
        with self.assertRaises(Exception):
            encode_control(True, False, 0, 40000, 0, [0] * 8)


class SectorMinimumsTest(unittest.TestCase):
    def test_nothing_seen(self) -> None:
        self.assertEqual(sectors(ranges_with({})), [NO_RETURN] * 8)

    def test_each_direction_lands_in_its_sector(self) -> None:
        # 0 ahead, 90 left, -90 right, 180 behind (counter-clockwise).
        got = sectors(ranges_with({0: 1.0, 45: 2.0, 90: 0.5, -180: 3.0, -90: 0.7, -45: 1.2}))
        self.assertEqual(got, [100, 200, 50, NO_RETURN, 300, NO_RETURN, 70, 120])

    def test_sector_edges_are_plus_minus_22_5_degrees(self) -> None:
        got = sectors(ranges_with({22: 1.0, 23: 2.0}))
        self.assertEqual(got[0], 100)
        self.assertEqual(got[1], 200)

    def test_keeps_the_closest_point(self) -> None:
        got = sectors(ranges_with({-10: 2.0, 0: 0.4, 10: 1.0}))
        self.assertEqual(got[0], 40)

    def test_yaw_rotates_into_the_base_frame(self) -> None:
        # LiDAR mounted turned 90 degrees left: its "ahead" is the rover's left.
        got = sectors(ranges_with({0: 1.0}), yaw=math.pi / 2)
        self.assertEqual(got[2], 100)
        self.assertEqual(got[0], NO_RETURN)

    def test_ignores_invalid_readings(self) -> None:
        got = sectors(ranges_with({0: math.inf, 1: 0.0, 2: -1.0, 3: math.nan}))
        self.assertEqual(got[0], NO_RETURN)

    def test_huge_distance_never_reads_as_no_return(self) -> None:
        got = sectors(ranges_with({0: 1000.0}))
        self.assertEqual(got[0], NO_RETURN - 1)


class ParseManualCommandTest(unittest.TestCase):
    def test_valid(self) -> None:
        cases = {
            "motor stop": (0, 0),
            "motor brake": (0, 0),
            "motor forward 120": (120, 120),
            "motor reverse 90": (-90, -90),
            "drive -50 60": (-50, 60),
            "  drive   255  -255 ": (255, -255),
        }
        for text, wheels in cases.items():
            with self.subTest(text=text):
                self.assertEqual(parse_manual_command(text), wheels)

    def test_invalid(self) -> None:
        for text in [
            "", "spin", "motor", "motor forward", "motor forward -1", "motor forward 256",
            "motor reverse x", "drive 1", "drive 1 2 3", "drive 300 0", "drive a b",
        ]:
            with self.subTest(text=text):
                self.assertIsNone(parse_manual_command(text))


class SplitLinesTest(unittest.TestCase):
    def test_holds_partial_lines_until_newline(self) -> None:
        buffer = bytearray()
        self.assertEqual(split_lines(buffer, b"st=idle l=0"), [])
        self.assertEqual(split_lines(buffer, b" r=0\r\nst=dri"), ["st=idle l=0 r=0"])
        self.assertEqual(split_lines(buffer, b"ving\n\n"), ["st=driving"])
        self.assertEqual(buffer, bytearray())

    def test_survives_garbage(self) -> None:
        buffer = bytearray()
        self.assertEqual(split_lines(buffer, b"\xff\xfeok\n"), ["��ok"])

    def test_drops_runaway_line(self) -> None:
        buffer = bytearray()
        split_lines(buffer, b"x" * 2000)
        self.assertEqual(buffer, bytearray())


if __name__ == "__main__":
    unittest.main()

"""Pi side of the Arduino Uno USB link, without ROS so it can be unit tested.

Frame format is documented in arduino_uno_rs/brain/src/protocol.rs.
"""

import math
import struct
from typing import Optional


SYNC = b"\xa5\x5a"
MSG_CONTROL = 0x02
SECTORS = 8
NO_RETURN = 0xFFFF
SECTOR_WIDTH = 2.0 * math.pi / SECTORS
FLAG_SCAN_FRESH = 0x01
FLAG_WARNING = 0x02
MISSION_CODES = {"S0_IDLE": 0, "S1_FOLLOW_MAP": 1, "S2_BACKTRACK": 2, "S3_SOS": 3}


def crc8(data: bytes) -> int:
    """CRC-8/SMBUS (poly 0x07, init 0), matching protocol::crc8."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_control(
    scan_fresh: bool, warning: bool, mission: int, left: int, right: int, sectors_cm: list[int]
) -> bytes:
    flags = (FLAG_SCAN_FRESH if scan_fresh else 0) | (FLAG_WARNING if warning else 0)
    body = struct.pack(
        "<BBBBhh8H", MSG_CONTROL, 6 + 2 * SECTORS, flags, mission, left, right, *sectors_cm
    )
    return SYNC + body + bytes([crc8(body)])


def sector_minimums(
    angle_min: float, angle_increment: float, ranges, yaw: float
) -> list[int]:
    """Closest return per 45 degree sector in cm; sector 0 is centered ahead."""
    closest = [NO_RETURN] * SECTORS
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance <= 0.0:
            continue
        angle = angle_min + index * angle_increment + yaw
        sector = int(math.floor((angle + SECTOR_WIDTH / 2.0) / SECTOR_WIDTH)) % SECTORS
        closest[sector] = min(closest[sector], min(NO_RETURN - 1, round(distance * 100.0)))
    return closest


def parse_manual_command(text: str) -> Optional[tuple[int, int]]:
    """`motor stop|brake`, `motor forward|reverse <0-255>`, `drive <l> <r>`."""
    words = text.split()
    try:
        if words in (["motor", "stop"], ["motor", "brake"]):
            return 0, 0
        if len(words) == 3 and words[:2] in (["motor", "forward"], ["motor", "reverse"]):
            speed = int(words[2])
            if 0 <= speed <= 255:
                return (speed, speed) if words[1] == "forward" else (-speed, -speed)
        if len(words) == 3 and words[0] == "drive":
            left, right = int(words[1]), int(words[2])
            if -255 <= left <= 255 and -255 <= right <= 255:
                return left, right
    except ValueError:
        pass
    return None


def parse_encoder_line(text: str) -> Optional[tuple[int, int, int]]:
    """`enc <uno ms> <left> <right>` from the Uno: running counts since boot."""
    words = text.split()
    if len(words) != 4 or words[0] != "enc":
        return None
    try:
        return int(words[1]), int(words[2]), int(words[3])
    except ValueError:
        return None


def split_lines(buffer: bytearray, chunk: bytes) -> list[str]:
    """Append serial bytes to `buffer`, return the complete non-empty lines."""
    buffer.extend(chunk)
    lines = []
    while b"\n" in buffer:
        line, _, rest = buffer.partition(b"\n")
        buffer[:] = rest
        text = line.decode("ascii", "replace").strip()
        if text:
            lines.append(text)
    if len(buffer) > 1024:
        buffer.clear()
    return lines

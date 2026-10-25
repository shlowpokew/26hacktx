#!/usr/bin/env python3
"""Generate short offline alert tones for the rover's audio feedback node."""

import math
from pathlib import Path
import struct
import sys
import wave


SAMPLE_RATE = 22050


def write_tone(path: Path, notes: list[tuple[float, float]], amplitude: float = 0.28) -> None:
    samples = bytearray()
    for frequency, duration in notes:
        count = int(SAMPLE_RATE * duration)
        for index in range(count):
            envelope = min(1.0, index / 160.0, (count - index) / 240.0)
            value = amplitude * envelope * math.sin(2.0 * math.pi * frequency * index / SAMPLE_RATE)
            samples.extend(struct.pack("<h", int(value * 32767)))
        samples.extend(b"\0\0" * int(SAMPLE_RATE * 0.025))
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(SAMPLE_RATE)
        output.writeframes(samples)


def main() -> None:
    output = Path(sys.argv[1] if len(sys.argv) > 1 else "/opt/robot/audio")
    write_tone(output / "start_chime.wav", [(740.0, 0.10), (988.0, 0.16)])
    write_tone(output / "status_beep.wav", [(880.0, 0.12)])


if __name__ == "__main__":
    main()

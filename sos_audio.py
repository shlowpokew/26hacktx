#!/usr/bin/env python3
"""Play offline mission tones and a looping SOS voice prompt through ALSA."""

import os
from pathlib import Path
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SosAudio(Node):
    def __init__(self) -> None:
        super().__init__("sos_audio")
        audio_dir = Path(os.environ.get("ROVER_AUDIO_DIR", "/opt/robot/audio"))
        self._start_chime = audio_dir / "start_chime.wav"
        self._status_beep = audio_dir / "status_beep.wav"
        self._sos_normal = audio_dir / "sos.wav"
        self._sos_loud = audio_dir / "sos_loud.wav"
        self._mode = "S0_IDLE"
        self._nearby_human = False
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._sos_thread: Optional[threading.Thread] = None
        self._sos_process: Optional[subprocess.Popen] = None
        self._one_shot: Optional[subprocess.Popen] = None
        self._audio_unavailable = False
        self._next_status_beep = time.monotonic()
        self.create_subscription(String, "/mission/state", self._set_mode, 10)
        self.create_subscription(Bool, "/sos/human_nearby", self._set_nearby, 10)
        self.create_timer(0.1, self._periodic_audio)

    def _set_mode(self, message: String) -> None:
        previous = self._mode
        self._mode = message.data
        if self._mode == "S3_SOS" and previous != "S3_SOS":
            if self._one_shot is not None and self._one_shot.poll() is None:
                self._one_shot.terminate()
            self._stop.clear()
            self._sos_thread = threading.Thread(
                target=self._sos_loop, name="sos-audio-loop", daemon=True
            )
            self._sos_thread.start()
        elif previous == "S3_SOS" and self._mode != "S3_SOS":
            self._stop.set()
            with self._lock:
                process = self._sos_process
            if process is not None and process.poll() is None:
                process.terminate()
        if self._mode == "S1_FOLLOW_MAP" and previous != "S1_FOLLOW_MAP":
            self._play_once(self._start_chime)
        if self._mode != "S2_BACKTRACK":
            self._next_status_beep = time.monotonic()

    def _set_nearby(self, message: Bool) -> None:
        self._nearby_human = message.data

    def _periodic_audio(self) -> None:
        if self._mode != "S2_BACKTRACK" or time.monotonic() < self._next_status_beep:
            return
        self._next_status_beep = time.monotonic() + 6.0
        self._play_once(self._status_beep)

    def _play_once(self, path: Path) -> None:
        if self._audio_unavailable or not path.exists():
            return
        try:
            if self._one_shot is not None and self._one_shot.poll() is None:
                self._one_shot.terminate()
            self._one_shot = subprocess.Popen(
                ["aplay", "-q", str(path)], stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as error:
            self._audio_unavailable = True
            self.get_logger().error(f"Could not start ALSA playback: {error}")

    def _sos_loop(self) -> None:
        while not self._stop.is_set():
            path = self._sos_loud if self._nearby_human else self._sos_normal
            if self._audio_unavailable or not path.exists():
                return
            try:
                process = subprocess.Popen(
                    ["aplay", "-q", str(path)], stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except OSError as error:
                self._audio_unavailable = True
                self.get_logger().error(f"Could not start SOS audio playback: {error}")
                return
            with self._lock:
                self._sos_process = process
            while process.poll() is None and not self._stop.wait(0.1):
                pass
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    process.kill()
            with self._lock:
                if self._sos_process is process:
                    self._sos_process = None
            pause = 0.3 if self._nearby_human else 1.5
            self._stop.wait(pause)

    def destroy_node(self) -> bool:
        self._stop.set()
        with self._lock:
            process = self._sos_process
        if process is not None and process.poll() is None:
            process.terminate()
        if self._one_shot is not None and self._one_shot.poll() is None:
            self._one_shot.terminate()
        if self._sos_thread is not None:
            self._sos_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SosAudio()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

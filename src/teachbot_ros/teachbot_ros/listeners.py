# listeners.py
"""
TCP listener threads for TOS Teachbot encoders and RS-485 trigger/buttons.
Connects to teachbot TCP server to receive encoder and control data.
"""

from __future__ import annotations

import socket
import threading
import time
from collections import deque
from typing import Dict, Optional


# -----------------------------------------------------------------------------
# Global protocol constants
# -----------------------------------------------------------------------------
MAX_POSITION = 1 << 17  # 131072 counts (17-bit)
START_BYTE = 0xAA       # RS-485 trigger frame start

# Potentiometer calibration defaults
POT_IDLE_RAW = 811
POT_FULL_RAW = 58


def pot_raw_to_percent(raw: int, idle: int = POT_IDLE_RAW, full: int = POT_FULL_RAW) -> int:
    """Map raw pot reading so idle→0%, full→100%, with clamping."""
    if idle == full:
        return 0
    t = (raw - idle) / (full - idle)
    t = max(0.0, min(1.0, t))
    return int(round(t * 100.0))


class _FreqMeter:
    """Utility to compute message frequency over the most-recent N samples."""

    def __init__(self, maxlen: int = 100) -> None:
        self._ts: deque = deque(maxlen=maxlen)
        self.freq_hz: float = 0.0

    def tick(self) -> None:
        now = time.perf_counter()
        self._ts.append(now)
        if len(self._ts) >= 2:
            dt = self._ts[-1] - self._ts[0]
            self.freq_hz = (len(self._ts) - 1) / dt if dt else 0.0


class EncoderListener(threading.Thread):
    """Connects to teachbot TCP server and receives 3-byte AksIM-2 frames."""

    def __init__(self, remote_ip: str, port: int, *, packet_size: int = 3) -> None:
        super().__init__(daemon=True)
        self._remote_ip = remote_ip
        self.port = port
        self._packet_size = packet_size
        self._sock: Optional[socket.socket] = None
        self._connected = False

        self._lock = threading.Lock()
        self.raw_pos: int = 0
        self.error: bool = False
        self.warning: bool = False
        self._running = threading.Event()
        self._freq = _FreqMeter()

    def _connect(self) -> bool:
        """Attempt to connect to the TCP server."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(2.0)
            self._sock.connect((self._remote_ip, self.port))
            self._sock.settimeout(1.0)  # Shorter timeout for recv
            self._connected = True
            return True
        except (socket.timeout, socket.error, OSError):
            self._connected = False
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
            return False

    def run(self) -> None:
        self._running.set()
        leftover = b""

        while self._running.is_set():
            # Try to connect if not connected
            if not self._connected:
                if not self._connect():
                    time.sleep(1.0)  # Wait before retry
                    continue

            try:
                packet = self._sock.recv(1024)  # TCP can batch data
            except socket.timeout:
                continue
            except OSError:
                self._connected = False
                continue

            if not packet:  # Connection closed
                self._connected = False
                continue

            leftover += packet
            while len(leftover) >= self._packet_size:
                frame, leftover = leftover[:self._packet_size], leftover[self._packet_size:]
                self._decode(frame)

    def _decode(self, frame: bytes) -> None:
        raw_val = (frame[0] << 16) | (frame[1] << 8) | frame[2]
        w_bit = raw_val & 0x01
        e_bit = (raw_val >> 1) & 0x01
        pos = raw_val >> 7

        with self._lock:
            self.raw_pos = pos
            self.error = (e_bit == 0)
            self.warning = (w_bit == 0)
            self._freq.tick()

    def stop(self) -> None:
        self._running.clear()
        self._connected = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def snapshot(self) -> Dict[str, float]:
        with self._lock:
            return {
                "pos": self.raw_pos,
                "error": self.error,
                "warning": self.warning,
                "freq": self._freq.freq_hz,
            }


class RS485Listener(threading.Thread):
    """Connects to teachbot TCP server and receives 5-byte trigger frames."""

    def __init__(self, remote_ip: str, port: int = 5011) -> None:
        super().__init__(daemon=True)
        self._remote_ip = remote_ip
        self.port = port
        self._packet_size = 5
        self._sock: Optional[socket.socket] = None
        self._connected = False

        self._lock = threading.Lock()
        self.pot: int = 0       # 0-1023
        self.btn1: bool = False
        self.btn2: bool = False
        self._freq = _FreqMeter()
        self._running = threading.Event()

    def _connect(self) -> bool:
        """Attempt to connect to the TCP server."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(2.0)
            self._sock.connect((self._remote_ip, self.port))
            self._sock.settimeout(1.0)  # Shorter timeout for recv
            self._connected = True
            return True
        except (socket.timeout, socket.error, OSError):
            self._connected = False
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
            return False

    def run(self) -> None:
        self._running.set()
        leftover = b""

        while self._running.is_set():
            # Try to connect if not connected
            if not self._connected:
                if not self._connect():
                    time.sleep(1.0)  # Wait before retry
                    continue

            try:
                data = self._sock.recv(1024)  # TCP can batch data
            except socket.timeout:
                continue
            except OSError:
                self._connected = False
                continue

            if not data:  # Connection closed
                self._connected = False
                continue

            leftover += data

            # TCP stream framing: search for start byte 0xAA to resynchronize
            while len(leftover) >= self._packet_size:
                # Find the start byte
                start_idx = leftover.find(bytes([START_BYTE]))
                if start_idx == -1:
                    # No start byte found, discard buffer
                    leftover = b""
                    break
                elif start_idx > 0:
                    # Discard bytes before start byte (resync)
                    leftover = leftover[start_idx:]
                    continue

                # We have a potential frame starting at index 0
                if len(leftover) < self._packet_size:
                    break  # Wait for more data

                frame = leftover[:self._packet_size]

                # Validate checksum before consuming
                if (sum(frame[:4]) & 0xFF) == frame[4]:
                    # Valid frame, consume and decode
                    leftover = leftover[self._packet_size:]
                    self._decode(frame)
                else:
                    # Invalid checksum - this 0xAA wasn't a real start byte
                    # Skip this byte and search for next 0xAA
                    leftover = leftover[1:]

    def _decode(self, buf: bytes) -> None:
        pot = (buf[2] << 8) | buf[1]
        btn = buf[3]

        with self._lock:
            self.pot = pot & 0x3FF  # 10-bit
            self.btn1 = bool(btn & 0x01)
            self.btn2 = bool(btn & 0x02)
            self._freq.tick()

    def stop(self) -> None:
        self._running.clear()
        self._connected = False
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def snapshot(self) -> Dict[str, float]:
        with self._lock:
            return {
                "pot": self.pot,
                "btn1": self.btn1,
                "btn2": self.btn2,
                "freq": self._freq.freq_hz,
            }

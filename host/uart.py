"""Thin UART line client for robot_drive firmware (no local command FIFO)."""

from __future__ import annotations

import re
import threading
import time
from typing import Callable, Dict, Optional

import serial

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

MODE_MAP = {
    "I": "idle",
    "F": "forward",
    "B": "backward",
    "L": "turn_left",
    "R": "turn_right",
}

STOP_MAP = {
    "-": "none",
    "C": "command",
    "W": "watchdog",
    "T": "travel_limit",
    "U": "turn_done",
    "D": "duration",
    "K": "boot",
    "Q": "queue_full",
}

# Compact status: S e=… m=F es=0 sp=80 tl=0 q=0 ls=D hb=0 cs=1 ss=0 tr=0 v=7500
STATUS_RE = re.compile(
    r"^S e=(?P<event>\S+)"
    r" m=(?P<m>[IFBLR])"
    r" es=(?P<estop>\d+)"
    r" sp=(?P<speed>-?\d+)"
    r" tl=(?P<ttl_left_ms>\d+)"
    r" q=(?P<qlen>\d+)"
    r" ls=(?P<ls>.)"
    r" hb=(?P<hb>\d+)"
    r" cs=(?P<cmd_seq>\d+)"
    r" ss=(?P<stop_seq>\d+)"
    r" tr=(?P<travel>-?\d+)"
    r" v=(?P<bat_mV>\d+)"
)

# Short ACK: A e=fw m=B es=0 q=0 hb=0
ACK_RE = re.compile(
    r"^A e=(?P<event>\S+)"
    r" m=(?P<m>[IFBLR])"
    r" es=(?P<estop>\d+)"
    r" q=(?P<qlen>\d+)"
    r" hb=(?P<hb>\d+)"
)


def parse_status(line: str) -> Optional[Dict]:
    raw = line.strip()
    if raw.startswith("D "):
        return {"event": "debug", "debug": True, "raw": raw}

    m = STATUS_RE.match(raw)
    if m:
        d = m.groupdict()
        return {
            "event": d["event"],
            "mode": MODE_MAP.get(d["m"], "idle"),
            "estop": d["estop"] == "1",
            "speed": int(d["speed"]),
            "ttl_left_ms": int(d["ttl_left_ms"]),
            "queue_len": int(d["qlen"]),
            "last_stop": STOP_MAP.get(d["ls"], d["ls"]),
            "need_hb": d["hb"] == "1",
            "cmd_seq": int(d["cmd_seq"]),
            "stop_seq": int(d["stop_seq"]),
            "travel": int(d["travel"]),
            "battery_mV": int(d["bat_mV"]),
            "raw": line,
        }

    m = ACK_RE.match(raw)
    if m:
        d = m.groupdict()
        return {
            "event": d["event"],
            "mode": MODE_MAP.get(d["m"], "idle"),
            "estop": d["estop"] == "1",
            "queue_len": int(d["qlen"]),
            "need_hb": d["hb"] == "1",
            "raw": line,
            "ack": True,
        }
    return None


class RobotUart:
    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        on_status: Optional[Callable[[Dict], None]] = None,
    ) -> None:
        self.port = port
        self.baud = baud
        self.on_status = on_status
        self._ser: Optional[serial.Serial] = None
        self.last_status: Dict = {}
        self.last_raw = ""
        self._lock = threading.Lock()
        self._reader: Optional[threading.Thread] = None
        self._running = False

    def open(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.25)
        self._ser.reset_input_buffer()
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._running = False
        if self._reader:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def __enter__(self) -> "RobotUart":
        self.open()
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def _read_loop(self) -> None:
        while self._running:
            try:
                if not self._ser or not self._ser.is_open:
                    time.sleep(0.05)
                    continue
                line = self._ser.readline().decode("ascii", errors="replace").strip()
                if not line:
                    continue
                self.last_raw = line
                parsed = parse_status(line)
                if parsed:
                    if parsed.get("debug"):
                        # Do not clobber last_status; still notify listener.
                        if self.on_status:
                            self.on_status(dict(parsed))
                        continue
                    # Merge ACK into last_status without wiping telemetry fields.
                    if parsed.get("ack"):
                        self.last_status.update(parsed)
                    else:
                        self.last_status = parsed
                    if self.on_status:
                        self.on_status(dict(self.last_status))
            except Exception:
                time.sleep(0.05)

    def write_line(self, line: str) -> None:
        with self._lock:
            if not self._ser or not self._ser.is_open:
                raise RuntimeError("UART not open")
            self._ser.write((line.strip() + "\n").encode("ascii"))
            self._ser.flush()

    def send(self, line: str) -> None:
        self.write_line(line)

    def stop(self) -> None:
        self.write_line("STOP")

    def clear(self) -> None:
        self.write_line("CLEAR")

    def idle(self) -> None:
        self.write_line("IDLE")

    def status(self) -> None:
        self.write_line("ST")

    def heartbeat(self, ttl_ms: int = 300) -> None:
        self.write_line(f"HB {ttl_ms}")

    def forward(self, speed: int = 100, ttl_ms: int = 300, dur_ms: int = 0) -> None:
        self.write_line(f"FW {speed} {ttl_ms} {dur_ms}")

    def backward(self, speed: int = 100, ttl_ms: int = 300, dur_ms: int = 0) -> None:
        self.write_line(f"BK {speed} {ttl_ms} {dur_ms}")

    def turn_left(
        self,
        speed: int = 80,
        ttl_ms: int = 300,
        counts: int = 0,
        dur_ms: int = 0,
    ) -> None:
        self.write_line(f"LT {speed} {ttl_ms} {counts} {dur_ms}")

    def turn_right(
        self,
        speed: int = 80,
        ttl_ms: int = 300,
        counts: int = 0,
        dur_ms: int = 0,
    ) -> None:
        self.write_line(f"RT {speed} {ttl_ms} {counts} {dur_ms}")

    def reset_odometry(self) -> None:
        self.write_line("RESET")

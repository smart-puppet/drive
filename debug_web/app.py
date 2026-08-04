#!/usr/bin/env python3
"""
Debug-only web pad for robot_drive.

Serves a browser UI (arrows + STOP). Publishes MQTT drive commands and
refreshes the MCU watchdog (HB) while a direction is held.
Requires host/mqtt_bridge.py (or equivalent) to be running.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import paho.mqtt.client as mqtt
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field


def _lan_ipv4s() -> List[str]:
    """Board LAN IPv4 addresses (excludes loopback and typical docker bridges)."""
    skip_prefixes = ("127.", "172.17.", "172.18.")
    found: List[str] = []
    try:
        out = subprocess.check_output(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            text=True,
            timeout=2,
        )
        for line in out.splitlines():
            parts = line.split()
            if "inet" not in parts:
                continue
            ip = parts[parts.index("inet") + 1].split("/")[0]
            if any(ip.startswith(p) for p in skip_prefixes):
                continue
            if ip not in found:
                found.append(ip)
    except Exception:
        pass
    if found:
        return found
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        primary = s.getsockname()[0]
        s.close()
        if primary and not any(primary.startswith(p) for p in skip_prefixes):
            found.append(primary)
    except Exception:
        pass
    return found


def _print_access_urls(host: str, port: int) -> None:
    urls = [f"http://127.0.0.1:{port}", f"http://localhost:{port}"]
    if host in ("0.0.0.0", "::", ""):
        for ip in _lan_ipv4s():
            urls.append(f"http://{ip}:{port}")
    elif host not in ("127.0.0.1", "localhost"):
        urls.append(f"http://{host}:{port}")
    print("Debug web pad listening — open:")
    for u in urls:
        print(f"  {u}")


STATIC_DIR = Path(__file__).resolve().parent / "static"
HB_PERIOD_S = 0.15

app = FastAPI(title="robot_drive debug pad", version="0.1.0")
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


class HoldRequest(BaseModel):
    direction: str = Field(..., pattern="^(forward|backward|left|right)$")
    speed: int = Field(100, ge=1, le=200)
    ttl_ms: int = Field(300, ge=50, le=1000)


class Bridge:
    def __init__(self) -> None:
        self.prefix = "robot/drive"
        self.broker = "127.0.0.1"
        self.broker_port = 1883
        self.username: Optional[str] = None
        self.password: Optional[str] = None
        self.client: Optional[mqtt.Client] = None
        self.last_status: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._holding = False
        self._hold_dir: Optional[str] = None
        self._hold_speed = 100
        self._hold_ttl = 300
        self._hb_thread: Optional[threading.Thread] = None
        self._running = False

    def configure(self, broker: str, port: int, prefix: str,
                  username: Optional[str], password: Optional[str]) -> None:
        self.broker = broker
        self.broker_port = port
        self.prefix = prefix.rstrip("/")
        self.username = username
        self.password = password

    def start(self) -> None:
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"robot_debug_web_{os.getpid()}",
        )
        if self.username:
            self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(self.broker, self.broker_port, keepalive=30)
        self.client.loop_start()
        self._running = True
        self._hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_thread.start()

    def stop(self) -> None:
        self._running = False
        self._holding = False
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            self.client = None

    def _topic(self, name: str) -> str:
        return f"{self.prefix}/{name}"

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        client.subscribe(self._topic("status"))

    def _on_message(self, client, userdata, msg):
        try:
            self.last_status = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            pass

    def _publish_cmd(self, payload: Dict[str, Any]) -> None:
        if not self.client:
            raise RuntimeError("mqtt not connected")
        self.client.publish(self._topic("cmd"), json.dumps(payload), qos=1)

    def _publish_stop(self) -> None:
        if not self.client:
            raise RuntimeError("mqtt not connected")
        self.client.publish(self._topic("stop"), "{}", qos=1)

    def _hb_loop(self) -> None:
        while self._running:
            with self._lock:
                holding = self._holding
                ttl = self._hold_ttl
            if holding:
                try:
                    self._publish_cmd({"cmd": "heartbeat", "ttl": ttl})
                except Exception:
                    pass
            time.sleep(HB_PERIOD_S)

    def hold(self, direction: str, speed: int, ttl_ms: int) -> Dict[str, Any]:
        cmd_map = {
            "forward": "forward",
            "backward": "backward",
            "left": "turn_left",
            "right": "turn_right",
        }
        cmd = cmd_map[direction]
        with self._lock:
            self._holding = True
            self._hold_dir = direction
            self._hold_speed = speed
            self._hold_ttl = ttl_ms
        body: Dict[str, Any] = {"cmd": cmd, "speed": speed, "ttl": ttl_ms, "dur": 0}
        if direction in ("left", "right"):
            body["counts"] = 0  # continuous spin while held
        self._publish_cmd(body)
        # Immediate HB so TTL starts fresh
        self._publish_cmd({"cmd": "heartbeat", "ttl": ttl_ms})
        return {"ok": True, "holding": direction}

    def release(self) -> Dict[str, Any]:
        with self._lock:
            self._holding = False
            self._hold_dir = None
        self._publish_cmd({"cmd": "idle"})
        return {"ok": True, "holding": None}

    def emergency_stop(self) -> Dict[str, Any]:
        with self._lock:
            self._holding = False
            self._hold_dir = None
        self._publish_stop()
        return {"ok": True, "estop": True}

    def clear(self) -> Dict[str, Any]:
        self._publish_cmd({"cmd": "clear"})
        return {"ok": True}

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            holding = self._hold_dir if self._holding else None
        return {
            "holding": holding,
            "status": self.last_status,
            "prefix": self.prefix,
        }


bridge = Bridge()


@app.on_event("startup")
def _startup() -> None:
    # Config applied in main() before uvicorn loads, or via env defaults.
    if bridge.client is None:
        bridge.configure(
            broker=os.environ.get("MQTT_BROKER", "127.0.0.1"),
            port=int(os.environ.get("MQTT_PORT", "1883")),
            prefix=os.environ.get("ROBOT_MQTT_PREFIX", "robot/drive"),
            username=os.environ.get("MQTT_USERNAME"),
            password=os.environ.get("MQTT_PASSWORD"),
        )
        bridge.start()


@app.on_event("shutdown")
def _shutdown() -> None:
    try:
        bridge.emergency_stop()
    except Exception:
        pass
    bridge.stop()


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/state")
def api_state() -> Dict[str, Any]:
    return bridge.snapshot()


@app.post("/api/hold")
def api_hold(body: HoldRequest) -> Dict[str, Any]:
    try:
        return bridge.hold(body.direction, body.speed, body.ttl_ms)
    except Exception as e:
        raise HTTPException(503, str(e)) from e


@app.post("/api/release")
def api_release() -> Dict[str, Any]:
    try:
        return bridge.release()
    except Exception as e:
        raise HTTPException(503, str(e)) from e


@app.post("/api/stop")
def api_stop() -> Dict[str, Any]:
    try:
        return bridge.emergency_stop()
    except Exception as e:
        raise HTTPException(503, str(e)) from e


@app.post("/api/clear")
def api_clear() -> Dict[str, Any]:
    try:
        return bridge.clear()
    except Exception as e:
        raise HTTPException(503, str(e)) from e


def main() -> None:
    p = argparse.ArgumentParser(description="robot_drive debug web pad")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8090)
    p.add_argument("--broker", default=os.environ.get("MQTT_BROKER", "127.0.0.1"))
    p.add_argument("--broker-port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")))
    p.add_argument("--prefix", default=os.environ.get("ROBOT_MQTT_PREFIX", "robot/drive"))
    p.add_argument("--username", default=os.environ.get("MQTT_USERNAME"))
    p.add_argument("--password", default=os.environ.get("MQTT_PASSWORD"))
    args = p.parse_args()

    bridge.configure(
        broker=args.broker,
        port=args.broker_port,
        prefix=args.prefix,
        username=args.username,
        password=args.password,
    )

    import uvicorn

    _print_access_urls(args.host, args.port)
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()

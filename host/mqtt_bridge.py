#!/usr/bin/env python3
"""
MQTT ↔ UART bridge for robot_drive.

Subscribes to drive commands on MQTT, forwards line protocol to the MCU.
Publishes MCU status back to MQTT. Sends HB while the robot is moving
(MCU FIFO + FUSA watchdog live on the firmware).
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from typing import Any, Dict, Optional

import paho.mqtt.client as mqtt

from uart import DEFAULT_BAUD, DEFAULT_PORT, RobotUart

DEFAULT_PREFIX = os.environ.get("ROBOT_MQTT_PREFIX", "robot/drive")
HB_PERIOD_S = 0.15
# ACKs only carry mode/estop/qlen; speed, ttl_left, last_stop, travel and
# battery live in the full `S` line, which the MCU sends only on `ST`.
STATUS_POLL_S = 0.5


class MqttUartBridge:
    def __init__(
        self,
        broker: str,
        port: int,
        prefix: str,
        serial_port: str,
        serial_baud: int,
        username: Optional[str] = None,
        password: Optional[str] = None,
    ) -> None:
        self.prefix = prefix.rstrip("/")
        self.topic_cmd = f"{self.prefix}/cmd"
        self.topic_stop = f"{self.prefix}/stop"
        self.topic_status = f"{self.prefix}/status"
        self.topic_debug = f"{self.prefix}/debug"
        self.topic_hb = f"{self.prefix}/hb"

        self.uart = RobotUart(
            port=serial_port,
            baud=serial_baud,
            on_status=self._on_uart_status,
        )
        self._hb_ttl = 300
        self._running = False
        self._hb_thread: Optional[threading.Thread] = None
        self._status_thread: Optional[threading.Thread] = None

        self.mqtt = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"robot_drive_{os.getpid()}",
        )
        if username:
            self.mqtt.username_pw_set(username, password)
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_message
        self._broker = broker
        self._broker_port = port

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        client.subscribe(self.topic_cmd)
        client.subscribe(self.topic_stop)
        client.subscribe(self.topic_hb)
        client.publish(
            self.topic_status,
            json.dumps({"event": "bridge_online", "prefix": self.prefix}),
            qos=1,
        )

    def _on_uart_status(self, status: Dict) -> None:
        try:
            if status.get("debug"):
                raw = status.get("raw", "")
                print(raw, flush=True)
                self.mqtt.publish(
                    self.topic_debug,
                    json.dumps({"event": "debug", "raw": raw}),
                    qos=0,
                )
                return
            self.mqtt.publish(self.topic_status, json.dumps(status), qos=0)
        except Exception:
            pass

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = msg.payload.decode("utf-8").strip()
        except Exception:
            return

        if topic == self.topic_stop:
            self.uart.stop()
            return

        data: Dict[str, Any] = {}
        if payload:
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                # allow bare command word: forward / stop / ...
                data = {"cmd": payload}

        if topic == self.topic_hb:
            ttl = int(data.get("ttl", self._hb_ttl))
            self._hb_ttl = ttl
            self.uart.heartbeat(ttl)
            return

        cmd = str(data.get("cmd", "")).lower().strip()
        self._handle_cmd(cmd, data)

    def _handle_cmd(self, cmd: str, data: Dict[str, Any]) -> None:
        speed = int(data.get("speed", 100))
        ttl = int(data.get("ttl", 300))
        dur = int(data.get("dur", data.get("duration_ms", 0)))
        counts = int(data.get("counts", 0))
        self._hb_ttl = ttl

        if cmd in ("stop",):
            self.uart.stop()
        elif cmd in ("clear", "arm"):
            self.uart.clear()
        elif cmd in ("idle",):
            self.uart.idle()
        elif cmd in ("status",):
            self.uart.status()
        elif cmd in ("reset", "reset_odometry"):
            self.uart.reset_odometry()
        elif cmd in ("dbg", "debug"):
            on = int(data.get("on", data.get("enable", 1)))
            self.uart.send(f"DBG {1 if on else 0}")
        elif cmd in ("hb", "heartbeat"):
            self.uart.heartbeat(ttl)
        elif cmd in ("forward", "fw"):
            self.uart.forward(speed=speed, ttl_ms=ttl, dur_ms=dur)
        elif cmd in ("backward", "back", "bk"):
            self.uart.backward(speed=speed, ttl_ms=ttl, dur_ms=dur)
        elif cmd in ("turn_left", "left", "lt"):
            self.uart.turn_left(
                speed=int(data.get("speed", 120)),
                ttl_ms=ttl,
                counts=counts,
                dur_ms=dur,
            )
        elif cmd in ("turn_right", "right", "rt"):
            self.uart.turn_right(
                speed=int(data.get("speed", 120)),
                ttl_ms=ttl,
                counts=counts,
                dur_ms=dur,
            )
        else:
            self.mqtt.publish(
                self.topic_status,
                json.dumps({"event": "error", "error": "unknown_cmd", "cmd": cmd}),
            )

    def _hb_loop(self) -> None:
        """Pet MCU watchdog only when status says hb=1 (untimed latch)."""
        while self._running:
            st = self.uart.last_status
            if st.get("need_hb") and not st.get("estop", False):
                try:
                    self.uart.heartbeat(self._hb_ttl)
                except Exception:
                    pass
            time.sleep(HB_PERIOD_S)

    def _status_loop(self) -> None:
        """Poll full telemetry; ACKs alone leave most status fields unset."""
        while self._running:
            try:
                self.uart.status()
            except Exception:
                pass
            time.sleep(STATUS_POLL_S)

    def start(self) -> None:
        self.uart.open()
        self._running = True
        self._hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_thread.start()
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self._status_thread.start()
        self.mqtt.connect(self._broker, self._broker_port, keepalive=30)
        self.mqtt.loop_start()

    def stop(self) -> None:
        self._running = False
        try:
            self.uart.stop()
        except Exception:
            pass
        self.mqtt.loop_stop()
        try:
            self.mqtt.disconnect()
        except Exception:
            pass
        self.uart.close()


def main() -> int:
    p = argparse.ArgumentParser(description="robot_drive MQTT↔UART bridge")
    p.add_argument("--broker", default=os.environ.get("MQTT_BROKER", "127.0.0.1"))
    p.add_argument("--broker-port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")))
    p.add_argument("--prefix", default=DEFAULT_PREFIX)
    p.add_argument("--serial", default=os.environ.get("ROBOT_SERIAL", DEFAULT_PORT))
    p.add_argument("--baud", type=int, default=int(os.environ.get("ROBOT_BAUD", str(DEFAULT_BAUD))))
    p.add_argument("--username", default=os.environ.get("MQTT_USERNAME"))
    p.add_argument("--password", default=os.environ.get("MQTT_PASSWORD"))
    args = p.parse_args()

    bridge = MqttUartBridge(
        broker=args.broker,
        port=args.broker_port,
        prefix=args.prefix,
        serial_port=args.serial,
        serial_baud=args.baud,
        username=args.username,
        password=args.password,
    )

    def _shutdown(*_args):
        bridge.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    bridge.start()
    print(
        f"MQTT bridge up: {args.broker}:{args.broker_port} "
        f"prefix={args.prefix} serial={args.serial}",
        flush=True,
    )
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    raise SystemExit(main())

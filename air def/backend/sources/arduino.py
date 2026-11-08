"""Serial reader/writer for the ESP32 radar.

ESP32 -> backend: timestamp_ms,bearing_deg,distance_cm
backend -> ESP32: START <duration_ms>\n | STOP\n | LOCK <bearing>\n | UNLOCK\n
Bearing is relative to the front of the sensor: -80° left, 0° forward,
+80° right. A duration of zero means continuous scanning.
"""
from __future__ import annotations

import os
import threading
import time
from typing import Iterator

import serial
import serial.tools.list_ports

_PORT_HINTS = ("arduino", "ch340", "cp210", "usb serial", "silicon labs", "usb jtag")
_serial_lock = threading.Lock()
_serial_handle: serial.Serial | None = None


def find_arduino_port() -> str | None:
    configured = os.getenv("ARDUINO_PORT")
    if configured:
        return configured

    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        description = f"{port.description or ''} {port.manufacturer or ''} {port.product or ''}".lower()
        if any(hint in description for hint in _PORT_HINTS):
            return port.device
    return ports[0].device if len(ports) == 1 else None


def send_command(command: str) -> tuple[bool, str]:
    """Send a single allow-listed scan/lock command to the connected ESP32."""
    global _serial_handle
    cmd = command.strip()
    if not (cmd == "STOP"
            or cmd == "UNLOCK"
            or cmd.startswith("START ")
            or cmd.startswith("LOCK ")):
        return False, "unsupported serial command"
    handle = _serial_handle
    if handle is None or not handle.is_open:
        return False, "serial port is not open yet; check ARDUINO_PORT and close PlatformIO Serial Monitor"
    try:
        with _serial_lock:
            handle.write((cmd + "\n").encode("ascii"))
            handle.flush()
        return True, "command sent"
    except (serial.SerialException, OSError, ValueError) as exc:
        _serial_handle = None
        return False, str(exc)


def read_arduino(port: str | None = None) -> Iterator[dict]:
    """Yield parsed measurements and reconnect after serial failures."""
    global _serial_handle

    while True:
        selected_port = port or find_arduino_port()
        if not selected_port:
            raise RuntimeError("No serial port found. Set ARDUINO_PORT=COM3 (or your actual port).")

        try:
            print(f"[esp32] opening {selected_port} at 115200 baud")
            with serial.Serial(selected_port, 115200, timeout=1) as handle:
                _serial_handle = handle
                time.sleep(2.0)  # ESP32 commonly resets when the serial port opens.
                handle.reset_input_buffer()

                while True:
                    raw = handle.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if not line or line.startswith("#"):
                        continue

                    parts = line.split(",")
                    if len(parts) != 3:
                        continue
                    try:
                        timestamp, bearing, distance = parts
                        yield {
                            "timestamp_ms": int(float(timestamp)),
                            "angle_deg": float(bearing),
                            "distance_cm": float(distance),
                        }
                    except (ValueError, TypeError):
                        continue
        except (serial.SerialException, OSError) as exc:
            print(f"[esp32] serial error on {selected_port}: {exc}; retrying in 2s")
            _serial_handle = None
            port = None
            time.sleep(2.0)
        finally:
            if _serial_handle is not None and not _serial_handle.is_open:
                _serial_handle = None
"""Flask API for the ESP32 ultrasonic radar and its browser dashboard."""
from __future__ import annotations

import json
import os
import threading
import time
from typing import Any

import flask

from process import get_latest_result, set_error, update_latest_result

app = flask.Flask(__name__)

# Both hardware and simulation remain idle until the dashboard starts a scan.
_scan_lock = threading.Lock()
_scan_state: dict[str, Any] = {
    "running": False,
    "duration_s": 0,
    "started_at": time.time(),
    "ends_at": None,
    "reason": "startup",
    "version": 0,
}

# Lock is a dashboard selection that also drives the physical servo in serial mode.
_visual_lock_guard = threading.Lock()
_visual_lock: dict[str, Any] = {"mode": "scan", "target_id": None}

# Re-lock control: how far the track must drift before we re-aim the servo,
# and the minimum time between serial LOCK commands so we don't flood the port.
_RELOCK_THRESHOLD_DEG = 3.0
_RELOCK_COOLDOWN_S = 0.08
_last_relock_at = 0.0
_last_locked_bearing = None
_relock_guard = threading.Lock()


def get_scan_status() -> dict[str, Any]:
    """Return a copy of the scan state and expire timed scans consistently."""
    with _scan_lock:
        now = time.time()
        ends_at = _scan_state["ends_at"]
        if _scan_state["running"] and ends_at is not None and now >= ends_at:
            _scan_state["running"] = False
            _scan_state["reason"] = "duration_elapsed"
            _scan_state["duration_s"] = 0
            _scan_state["ends_at"] = None
            _scan_state["version"] += 1

        remaining = None
        if _scan_state["running"] and _scan_state["ends_at"] is not None:
            remaining = max(0.0, _scan_state["ends_at"] - now)

        return {
            "running": bool(_scan_state["running"]),
            "duration_s": int(_scan_state["duration_s"]),
            "remaining_s": round(remaining, 1) if remaining is not None else None,
            "reason": _scan_state["reason"],
            "version": _scan_state["version"],
            "sector": "FRONT ±80°" if os.getenv("RADAR_SOURCE", "sim").lower() == "serial" else "360° SIMULATION",
        }


def scan_is_running() -> bool:
    return get_scan_status()["running"]


def _set_scan_state(running: bool, duration_s: int = 0, reason: str = "command") -> None:
    now = time.time()
    with _scan_lock:
        _scan_state["running"] = running
        _scan_state["duration_s"] = duration_s if running else 0
        _scan_state["started_at"] = now if running else None
        _scan_state["ends_at"] = now + duration_s if running and duration_s > 0 else None
        _scan_state["reason"] = reason
        _scan_state["version"] += 1


def get_visual_lock() -> dict[str, Any]:
    with _visual_lock_guard:
        return dict(_visual_lock)


def _send_lock(bearing_deg: int) -> tuple[bool, str]:
    """Send a LOCK command; returns (ok, message). No-op in sim mode."""
    source = os.getenv("RADAR_SOURCE", "sim").lower()
    if source != "serial":
        return True, "sim"
    try:
        from sources.arduino import send_command
        return send_command(f"LOCK {bearing_deg}")
    except Exception as exc:  # noqa: BLE001
        return False, str(exc)


def _maybe_relock(track: dict[str, Any]) -> None:
    """Re-aim the servo if the locked track has drifted past the threshold."""
    global _last_relock_at, _last_locked_bearing

    bearing = track.get("angle_deg")
    if bearing is None:
        return

    with _relock_guard:
        now = time.time()
        if _last_locked_bearing is not None:
            drift = abs(((bearing - _last_locked_bearing + 180.0) % 360.0) - 180.0)
            if drift < _RELOCK_THRESHOLD_DEG:
                return
        if now - _last_relock_at < _RELOCK_COOLDOWN_S:
            return

        ok, msg = _send_lock(int(round(bearing)))
        if ok:
            _last_relock_at = now
            _last_locked_bearing = bearing
        else:
            print(f"[!] Re-lock failed: {msg}")


def _build_source():
    mode = os.getenv("RADAR_SOURCE", "sim").lower()
    print(f"[*] Radar source: {mode}")
    if mode == "sim":
        from sources.simulate_radar import generate_measurements
        return generate_measurements()
    if mode == "serial":
        from sources.arduino import read_arduino
        return read_arduino()
    raise ValueError(f"Unknown RADAR_SOURCE: {mode!r}")


def data_ingestion_worker() -> None:
    """Read sensor data; publish during scans and locks; re-aim on drift."""
    global _last_locked_bearing

    while True:
        try:
            for measurement in _build_source():
                lock = get_visual_lock()

                if lock["mode"] == "lock" and lock["target_id"] is not None:
                    # Publish first so the tracker updates the locked track's bearing.
                    update_latest_result(measurement)
                    set_error(None)

                    state = get_latest_result()
                    track = next(
                        (t for t in state["tracked_objects"] if t["id"] == lock["target_id"]),
                        None,
                    )
                    if track is None:
                        # Track expired; release the lock so the user can re-acquire.
                        with _visual_lock_guard:
                            _visual_lock.update({"mode": "scan", "target_id": None})
                        _last_locked_bearing = None
                        continue

                    _maybe_relock(track)
                elif scan_is_running():
                    update_latest_result(measurement)
                    set_error(None)
        except Exception as exc:  # noqa: BLE001
            message = str(exc)
            print(f"[!] Ingestion error: {message}. Retrying in 2s…")
            set_error(message)
            time.sleep(2)


@app.after_request
def add_cors_headers(response: flask.Response) -> flask.Response:
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type,Authorization"
    response.headers["Access-Control-Allow-Methods"] = "GET,POST,OPTIONS"
    return response


def _public_state() -> dict[str, Any]:
    state = get_latest_result()
    state["scan"] = get_scan_status()
    state["lock"] = get_visual_lock()
    return state


@app.route("/api/health", methods=["GET"])
def health():
    return flask.jsonify({"status": "ok", "source": os.getenv("RADAR_SOURCE", "sim").lower()}), 200


@app.route("/api/radar/latest", methods=["GET"])
def radar_latest():
    return flask.jsonify(_public_state()), 200


@app.route("/api/radar/stream", methods=["GET"])
def radar_stream():
    """Send state at 10 Hz so controls and countdowns update while paused too."""
    def generate():
        while True:
            yield "data: " + json.dumps(_public_state(), separators=(",", ":")) + "\n\n"
            time.sleep(0.1)

    return flask.Response(
        generate(),
        mimetype="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no",
            "Connection": "keep-alive",
        },
    )


@app.route("/api/radar/control", methods=["POST", "OPTIONS"])
def radar_control():
    """Start a timed/continuous scan or stop the physical radar servo."""
    global _last_locked_bearing

    if flask.request.method == "OPTIONS":
        return ("", 204)

    data = flask.request.get_json(silent=True) or {}
    action = str(data.get("action", "")).strip().lower()
    source = os.getenv("RADAR_SOURCE", "sim").lower()

    if action == "stop":
        if source == "serial":
            try:
                from sources.arduino import send_command
                ok, message = send_command("STOP")
            except Exception as exc:  # noqa: BLE001
                return flask.jsonify({"status": "error", "message": str(exc)}), 502
            if not ok:
                return flask.jsonify({"status": "error", "message": message}), 502
        with _visual_lock_guard:
            _visual_lock.update({"mode": "scan", "target_id": None})
        _last_locked_bearing = None
        _set_scan_state(False, reason="manual_stop")
        return flask.jsonify({"status": "success", "scan": get_scan_status()}), 200

    if action == "start":
        raw_duration = data.get("duration_s", 0)
        try:
            duration_s = int(raw_duration)
        except (TypeError, ValueError):
            return flask.jsonify({"status": "error", "message": "duration_s must be an integer"}), 400
        if duration_s < 0 or duration_s > 3600:
            return flask.jsonify({"status": "error", "message": "duration_s must be between 0 and 3600"}), 400

        if source == "serial":
            try:
                from sources.arduino import send_command
                ok, message = send_command(f"START {duration_s * 1000}")
            except Exception as exc:  # noqa: BLE001
                return flask.jsonify({"status": "error", "message": str(exc)}), 502
            if not ok:
                return flask.jsonify({"status": "error", "message": message}), 502

        with _visual_lock_guard:
            _visual_lock.update({"mode": "scan", "target_id": None})
        _last_locked_bearing = None
        _set_scan_state(True, duration_s=duration_s, reason="manual_start")
        return flask.jsonify({"status": "success", "scan": get_scan_status()}), 200

    return flask.jsonify({"status": "error", "message": "action must be 'start' or 'stop'"}), 400


@app.route("/api/lock", methods=["POST", "OPTIONS"])
def visual_lock():
    """Store a dashboard selection and drive the servo when in serial mode."""
    global _last_locked_bearing

    if flask.request.method == "OPTIONS":
        return ("", 204)

    data = flask.request.get_json(silent=True) or {}
    mode = str(data.get("mode", "lock")).lower()
    if mode not in ("lock", "scan"):
        return flask.jsonify({"status": "error", "message": "mode must be 'lock' or 'scan'"}), 400

    source = os.getenv("RADAR_SOURCE", "sim").lower()

    if mode == "scan":
        if source == "serial":
            try:
                from sources.arduino import send_command
                ok, message = send_command("UNLOCK")
            except Exception as exc:  # noqa: BLE001
                return flask.jsonify({"status": "error", "message": str(exc)}), 502
            if not ok:
                return flask.jsonify({"status": "error", "message": message}), 502
        with _visual_lock_guard:
            _visual_lock.update({"mode": "scan", "target_id": None})
        _last_locked_bearing = None
        return flask.jsonify({"status": "success", "lock": get_visual_lock()}), 200

    target_id = data.get("target_id")
    if target_id is None:
        return flask.jsonify({"status": "error", "message": "target_id is required"}), 400

    state = get_latest_result()
    track = next((t for t in state["tracked_objects"] if t["id"] == target_id), None)
    if track is None:
        return flask.jsonify({"status": "error", "message": "track not found or expired"}), 404

    bearing = int(round(track["angle_deg"]))
    ok, message = _send_lock(bearing)
    if not ok:
        return flask.jsonify({"status": "error", "message": message}), 502

    with _visual_lock_guard:
        _visual_lock.update({"mode": "lock", "target_id": target_id})
    _last_locked_bearing = track["angle_deg"]
    return flask.jsonify({"status": "success", "lock": get_visual_lock()}), 200


if __name__ == "__main__":
    threading.Thread(target=data_ingestion_worker, daemon=True, name="radar-ingestion").start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
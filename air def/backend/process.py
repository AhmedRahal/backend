"""Polar->cartesian conversion plus multi-target tracker with size estimation."""
import math
import os
import threading
import time

_state_lock = threading.Lock()

# ----------------------------- tuning ---------------------------------
# Widened so a target doesn't get orphaned into a "new track" every sweep.
_ANGLE_GATE_DEG = 25.0   # was 12.0
_DIST_GATE_CM   = 50.0   # was 25.0
_TRACK_TTL_MS   = 6000   # was 4000 — survives the gap between sweep passes
_VEL_ALPHA      = 0.35
_VEL_MAX_DT_MS  = 500

# -------- size classification tuning --------
_SENSOR_BEAM_DEG = float(os.getenv(
    "SENSOR_BEAM_DEG",
    "0" if os.getenv("RADAR_SOURCE", "sim").lower() == "sim" else "15",
))

_WIDTH_ALPHA = 0.30

_SIZE_THRESHOLDS = [
    ("small",  0.0,          20.0),
    ("medium", 20.0,         60.0),
    ("large",  60.0, float("inf")),
]

# ----------------------------- state ----------------------------------
_state = {
    "seq": 0,
    "last_update_ms": None,
    "last_measurement": None,
    "tracked_objects": [],
    "error": None,
}
_tracks = []
_next_track_id = 1

_prev_sweep_id = None
_prev_sweep_angle = None
_sweep_direction = 0


# --------------------------- conversion -------------------------------
def _ang_diff(a, b):
    """Signed shortest difference a - b in degrees, in (-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def _polar_to_xy(angle_deg, distance_cm):
    r = math.radians(angle_deg)
    return distance_cm * math.sin(r), distance_cm * math.cos(r)


def process_measurement(measurement):
    ts       = measurement.get("timestamp_ms")
    angle    = measurement.get("angle_deg")
    distance = measurement.get("distance_cm")

    x = y = 0.0
    if distance is not None and distance > 0 and angle is not None:
        x, y = _polar_to_xy(angle, distance)

    return {
        "timestamp_ms": ts,
        "angle_deg":    angle,
        "distance_cm":  distance,
        "x":            round(x, 2),
        "y":            round(y, 2),
        "sweep_id":     measurement.get("sweep_id"),
        "sweep_dir":    measurement.get("sweep_dir"),
        "snr_db":       measurement.get("snr_db"),
        "amplitude":    measurement.get("amplitude"),
    }


def iter_processed(measurement):
    dets = measurement.get("detections")
    if not dets:
        yield process_measurement(measurement)
        return
    for d in dets:
        yield process_measurement({**measurement, **d})


# --------------------------- size helpers -----------------------------
def _classify_width(w_cm):
    for name, lo, hi in _SIZE_THRESHOLDS:
        if lo <= w_cm < hi:
            return name
    return "unknown"


def _width_from_span(span_deg, distance_cm):
    corrected = max(0.0, span_deg - _SENSOR_BEAM_DEG)
    if corrected <= 0:
        return 0.0
    return 2.0 * distance_cm * math.tan(math.radians(corrected / 2.0))


# ---------------------------- tracker ---------------------------------
def _associate(angle_deg, distance_cm):
    best, best_cost = None, float("inf")
    for t in _tracks:
        da = abs(_ang_diff(angle_deg, t["angle_deg"]))
        dd = abs(distance_cm - t["distance_cm"])
        if da > _ANGLE_GATE_DEG or dd > _DIST_GATE_CM:
            continue
        cost = (da / _ANGLE_GATE_DEG) ** 2 + (dd / _DIST_GATE_CM) ** 2
        if cost < best_cost:
            best, best_cost = t, cost
    return best


def _finalize_spans():
    for t in _tracks:
        span = t["span_max_deg"] - t["span_min_deg"]
        if span <= 0:
            continue

        w = _width_from_span(span, t["distance_cm"])

        if t["width_cm"] is None:
            t["width_cm"] = w
        else:
            a = _WIDTH_ALPHA
            t["width_cm"] = (1 - a) * t["width_cm"] + a * w

        t["size_class"] = _classify_width(t["width_cm"])

        t["span_min_deg"] = t["angle_deg"]
        t["span_max_deg"] = t["angle_deg"]


def _detect_reversal(processed, angle):
    global _prev_sweep_id, _prev_sweep_angle, _sweep_direction

    sweep_id = processed.get("sweep_id")
    if sweep_id is not None:
        if _prev_sweep_id is not None and sweep_id != _prev_sweep_id:
            _finalize_spans()
        _prev_sweep_id = sweep_id
        return

    if _prev_sweep_angle is not None:
        delta = angle - _prev_sweep_angle
        if delta != 0:
            new_dir = 1 if delta > 0 else -1
            if _sweep_direction != 0 and new_dir != _sweep_direction:
                _finalize_spans()
            _sweep_direction = new_dir
    _prev_sweep_angle = angle


def _update_tracker(processed, now_ms):
    global _next_track_id

    angle    = processed["angle_deg"]
    distance = processed["distance_cm"]

    if angle is not None:
        _detect_reversal(processed, angle)

    if distance is not None and distance > 0:
        t = _associate(angle, distance)
        if t is None:
            t = {
                "id":             _next_track_id,
                "angle_deg":      angle,
                "distance_cm":    distance,
                "x":              processed["x"],
                "y":              processed["y"],
                "v_angle":        0.0,
                "v_distance":     0.0,
                "hits":           0,
                "first_seen_ms":  now_ms,
                "last_update_ms": now_ms,
                "span_min_deg":   angle,
                "span_max_deg":   angle,
                "width_cm":       None,
                "size_class":     "unknown",
                "snr_db":         processed.get("snr_db"),
                "amplitude":      processed.get("amplitude"),
            }
            _next_track_id += 1
            _tracks.append(t)
        else:
            dt = now_ms - t["last_update_ms"]
            if 0 < dt <= _VEL_MAX_DT_MS:
                a = _VEL_ALPHA
                t["v_angle"]    = (1 - a) * t["v_angle"]    + a * _ang_diff(angle, t["angle_deg"]) * 1000.0 / dt
                t["v_distance"] = (1 - a) * t["v_distance"] + a * (distance - t["distance_cm"]) * 1000.0 / dt

            t["angle_deg"]      = angle
            t["distance_cm"]    = distance
            t["x"]              = processed["x"]
            t["y"]              = processed["y"]
            t["last_update_ms"] = now_ms

            snr = processed.get("snr_db")
            if snr is not None:
                t["snr_db"] = snr
            amp = processed.get("amplitude")
            if amp is not None:
                t["amplitude"] = amp

            if angle < t["span_min_deg"]:
                t["span_min_deg"] = angle
            if angle > t["span_max_deg"]:
                t["span_max_deg"] = angle

        t["hits"] += 1

    _tracks[:] = [
        t for t in _tracks
        if now_ms - t["last_update_ms"] <= _TRACK_TTL_MS
    ]


def _publish_tracks():
    now_ms = int(time.time() * 1000)
    _state["tracked_objects"] = [
        {
            "id":          t["id"],
            "angle_deg":   t["angle_deg"],
            "distance_cm": t["distance_cm"],
            "x":           t["x"],
            "y":           t["y"],
            "v_angle":     round(t["v_angle"], 2),
            "v_distance":  round(t["v_distance"], 2),
            "hits":        t["hits"],
            "age_ms":      now_ms - t["first_seen_ms"],
            "width_cm":    round(t["width_cm"], 1) if t["width_cm"] is not None else None,
            "size_class":  t["size_class"],
            "snr_db":      t.get("snr_db"),
            "amplitude":   t.get("amplitude"),
        }
        for t in _tracks
    ]


def update_latest_result(measurement):
    now_ms = int(time.time() * 1000)
    nearest = None

    with _state_lock:
        for processed in iter_processed(measurement):
            _update_tracker(processed, now_ms)
            d = processed["distance_cm"]
            if d is not None and d > 0:
                if nearest is None or d < nearest["distance_cm"]:
                    nearest = processed
        if nearest is None:
            nearest = process_measurement(measurement)

        _state["seq"] += 1
        _state["last_update_ms"] = now_ms
        _state["last_measurement"] = nearest
        _publish_tracks()


def get_latest_result():
    with _state_lock:
        return {
            "seq":               _state["seq"],
            "last_update_ms":    _state["last_update_ms"],
            "last_measurement":  _state["last_measurement"],
            "tracked_objects":   list(_state["tracked_objects"]),
            "error":             _state["error"],
        }


def set_error(msg):
    with _state_lock:
        _state["error"] = msg

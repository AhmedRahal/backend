"""Simulated 360 degree radar sweep -- a pure source module.

Pipeline position
-----------------
    simulate_radar.py  ->  process.py  ->  backend  ->  HTML/CSS/JS radar
      (this file)          (tracker)     (transport)    (display)

This module only produces raw beam samples.  It never talks to the
network, never assumes how it will be rendered, and exposes nothing
beyond :func:`generate_measurements` (the iterator ``process.py``
consumes) and :func:`default_scene` / :class:`SimConfig` for setup.

Sample payload (one per beam position)
--------------------------------------
::

    {
        "timestamp_ms":   int,        # ms since simulator start
        "angle_deg":      float,      # current beam bearing (always sent)
        "distance_cm":    float,      # nearest echo, -1.0 if none
        "truth_id":       int | None, # ground truth, debug/scoring only
        "snr_db":         float|None, # nearest echo strength
        "detections":     [ ... ],    # every echo in this beam
        "sweep_id":       int,        # increments on every pass reversal
        "sweep_dir":      +1 | -1,    # +1 rising, -1 falling
        "sweep_progress": float,      # 0..1 across the sector
    }

``detections`` entries are ``{"distance_cm", "truth_id", "snr_db",
"amplitude"}``.  The top-level ``distance_cm`` / ``truth_id`` / ``snr_db``
mirror the *nearest* entry so consumers that only read those three keys
keep working unchanged.

Truth data (``truth_id``, ``width_cm``) is exported for scoring only --
the tracker in ``process.py`` never sees it and still has to estimate
width from the observed angular span.
"""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from typing import Iterator, Optional


# ======================================================================
# Tunables
# ======================================================================
@dataclass
class SimConfig:
    """Everything tunable about the simulated front end."""

    # -- sweep geometry -------------------------------------------------
    min_angle_deg: float = 0.0
    max_angle_deg: float = 360.0
    step_deg: float = 1.0                # angular resolution of the beam
    sweep_period_s: float = 0.015        # dwell time per beam position
    min_range_cm: float = 2.0
    max_range_cm: float = 400.0

    # -- pacing (for headless tests) ------------------------------------
    realtime: bool = True                # False -> run flat out
    max_steps: Optional[int] = None      # None -> run forever

    # -- detection model ------------------------------------------------
    snr_ref_db: float = 20.0             # SNR of the reference target ...
    ref_range_cm: float = 100.0          # ... at this range ...
    ref_width_cm: float = 50.0           # ... with this physical width
    snr_threshold_db: float = 6.0        # ~50 % single-dwell Pd
    snr_softness_db: float = 2.0         # logistic slope (dB)
    swerling: bool = True                # per-dwell echo fluctuation
    swerling_sigma_db: float = 3.0       # 1-sigma fluctuation (dB)
    miss_rate: float = 0.0               # flat dropout on top of the model
    clutter_rate: float = 0.0            # expected false alarms per dwell

    # -- measurement noise ---------------------------------------------
    noise_cm: float = 1.5                # 1-sigma range noise at ref range
    noise_range_gain: float = 0.0        # extra sigma per ref_range_cm

    # -- output ---------------------------------------------------------
    seed: Optional[int] = None           # None -> non-reproducible

    def __post_init__(self) -> None:
        if self.step_deg <= 0.0:
            raise ValueError("step_deg must be > 0")
        if self.max_angle_deg <= self.min_angle_deg:
            raise ValueError("max_angle_deg must be > min_angle_deg")
        if self.max_range_cm <= self.min_range_cm:
            raise ValueError("max_range_cm must be > min_range_cm")
        if self.sweep_period_s < 0.0:
            raise ValueError("sweep_period_s must be >= 0")


# ======================================================================
# Scene objects (ground truth)
# ======================================================================
@dataclass
class RadarObject:
    """A single truth target.

    ``width_cm`` is ground truth only -- the tracker never sees it.
    Set ``angle_rate_dps`` for constant bearing drift, ``range_rate_cms``
    for range motion, and ``osc_amp_deg`` for a sinusoidal patrol that
    rides on top of the drift.
    """

    truth_id: int
    angle_deg: float
    distance_cm: float
    width_cm: float

    angle_rate_dps: float = 0.0          # constant bearing drift
    range_rate_cms: float = 0.0          # + closing, - opening
    osc_amp_deg: float = 0.0             # sinusoidal bearing patrol
    osc_period_s: float = 6.0
    osc_phase: float = 0.0

    base_angle_deg: float = field(init=False, repr=False, default=0.0)

    def __post_init__(self) -> None:
        self.base_angle_deg = float(self.angle_deg)

    @property
    def half_width_deg(self) -> float:
        """Angular half-width subtended by the disc at its current range."""
        return math.degrees(math.atan2(self.width_cm, 2.0 * self.distance_cm))

    def advance(self, dt_s: float, t_s: float) -> None:
        """Integrate motion forward by ``dt_s`` seconds (elapsed: ``t_s``)."""
        self.base_angle_deg += self.angle_rate_dps * dt_s
        self.distance_cm = max(1.0, self.distance_cm + self.range_rate_cms * dt_s)

        offset = 0.0
        if self.osc_amp_deg:
            offset = self.osc_amp_deg * math.sin(
                2.0 * math.pi * t_s / self.osc_period_s + self.osc_phase
            )
        self.angle_deg = self.base_angle_deg + offset


def default_scene() -> list[RadarObject]:
    """The original four-target scene.

    Width buckets (per ``process.py`` defaults):
        <20 cm   -> small
        20-60 cm -> medium
        >60 cm   -> large
    """
    return [
        RadarObject(1, 30.0, 100.0, 25.0),                                    # medium
        RadarObject(2, 85.0, 180.0, 50.0, osc_amp_deg=8.0, osc_period_s=6.0), # medium, patrolling
        RadarObject(3, 145.0, 75.0, 15.0),                                    # small
        RadarObject(4, 55.0, 60.0, 90.0),                                     # LARGE
        RadarObject(5, 230.0, 120.0, 30.0),                                   # medium (rear sector)
        RadarObject(6, 305.0, 80.0, 15.0),                                    # small (rear sector)
    ]


# ======================================================================
# Simulator
# ======================================================================
class RadarSimulator:
    """Ping-pong 360 degree sweep over a set of truth objects."""

    def __init__(
        self,
        config: Optional[SimConfig] = None,
        objects: Optional[list[RadarObject]] = None,
        rng: Optional[random.Random] = None,
    ) -> None:
        self.cfg = config or SimConfig()
        self.objects = list(objects) if objects is not None else default_scene()
        self.rng = rng or random.Random(self.cfg.seed)
        self._last_t = 0.0

    # -- detection model ------------------------------------------------
    def _snr_db(self, obj: RadarObject) -> float:
        """Radar-equation style SNR: RCS ~ width^2, two-way spread ~ 1/R^4."""
        if obj.distance_cm <= 0.0 or obj.width_cm <= 0.0:
            return -math.inf
        cfg = self.cfg
        return (
            cfg.snr_ref_db
            + 20.0 * math.log10(obj.width_cm / cfg.ref_width_cm)
            - 40.0 * math.log10(obj.distance_cm / cfg.ref_range_cm)
        )

    def _fluctuate(self, snr_db: float) -> float:
        """Swerling-I style per-dwell echo fluctuation."""
        if not self.cfg.swerling:
            return snr_db
        return snr_db + self.rng.gauss(0.0, self.cfg.swerling_sigma_db)

    def _pd(self, snr_db: float) -> float:
        """Single-dwell probability of detection, logistic in SNR."""
        cfg = self.cfg
        x = (snr_db - cfg.snr_threshold_db) / max(cfg.snr_softness_db, 1e-9)
        x = max(-60.0, min(60.0, x))          # keep exp() finite
        return (1.0 / (1.0 + math.exp(-x))) * (1.0 - cfg.miss_rate)

    # -- one beam position ----------------------------------------------
    def _beam_sample(
        self,
        angle_deg: float,
        timestamp_ms: int,
        sweep_id: int,
        sweep_dir: int,
    ) -> dict:
        cfg, rng = self.cfg, self.rng
        detections: list[dict] = []

        for obj in self.objects:
            if not (cfg.min_range_cm <= obj.distance_cm <= cfg.max_range_cm):
                continue
            if abs(angle_deg - obj.angle_deg) > obj.half_width_deg:
                continue

            snr_db = self._fluctuate(self._snr_db(obj))
            if rng.random() > self._pd(snr_db):
                continue

            sigma = cfg.noise_cm * (
                1.0 + cfg.noise_range_gain * obj.distance_cm / cfg.ref_range_cm
            )
            distance_cm = obj.distance_cm + rng.gauss(0.0, sigma)
            distance_cm = min(max(distance_cm, cfg.min_range_cm), cfg.max_range_cm)

            detections.append({
                "distance_cm": round(distance_cm, 2),
                "truth_id":    obj.truth_id,
                "snr_db":      round(snr_db, 2),
                # 0..1, handy for display opacity / blip size
                "amplitude":   round(1.0 / (1.0 + math.exp(
                    -(snr_db - cfg.snr_threshold_db) / max(cfg.snr_softness_db, 1e-9)
                )), 3),
            })

        # False alarm: random blip with no truth object behind it.
        if cfg.clutter_rate > 0.0 and rng.random() < cfg.clutter_rate:
            detections.append({
                "distance_cm": round(rng.uniform(cfg.min_range_cm, cfg.max_range_cm), 2),
                "truth_id":    None,
                "snr_db":      None,
                "amplitude":   0.0,
            })

        # Nearest echo becomes the legacy top-level scalar fields.
        if detections:
            nearest = min(detections, key=lambda d: d["distance_cm"])
            distance_cm = nearest["distance_cm"]
            truth_id    = nearest["truth_id"]
            snr_db      = nearest["snr_db"]
        else:
            distance_cm = -1.0
            truth_id    = None
            snr_db      = None

        span = cfg.max_angle_deg - cfg.min_angle_deg
        return {
            "timestamp_ms":   timestamp_ms,
            "angle_deg":      round(angle_deg, 3),
            "distance_cm":    distance_cm,        # backward compat
            "truth_id":       truth_id,           # backward compat (debug)
            "snr_db":         snr_db,
            "detections":     detections,
            "sweep_id":       sweep_id,
            "sweep_dir":      sweep_dir,
            "sweep_progress": round((angle_deg - cfg.min_angle_deg) / span, 4),
        }

    # -- main loop ------------------------------------------------------
    def measurements(self) -> Iterator[dict]:
        """Yield one beam sample per beam position.  Never terminates
        unless ``SimConfig.max_steps`` is set."""
        cfg = self.cfg

        angle = float(cfg.min_angle_deg)
        direction = 1
        sweep_id = 1
        steps = 0

        t0 = time.monotonic()
        next_tick = t0

        while cfg.max_steps is None or steps < cfg.max_steps:
            now = time.monotonic()
            t_s = now - t0
            dt_s = t_s - self._last_t
            self._last_t = t_s

            for obj in self.objects:
                obj.advance(dt_s, t_s)

            yield self._beam_sample(
                angle_deg=angle,
                timestamp_ms=int(t_s * 1000.0),
                sweep_id=sweep_id,
                sweep_dir=direction,
            )
            steps += 1

            # --- advance the beam one step, bouncing off sector edges ---
            angle += cfg.step_deg * direction
            if angle >= cfg.max_angle_deg:
                angle, direction, sweep_id = float(cfg.max_angle_deg), -1, sweep_id + 1
            elif angle <= cfg.min_angle_deg:
                angle, direction, sweep_id = float(cfg.min_angle_deg), 1, sweep_id + 1

            # --- absolute-deadline pacing (no cumulative drift) ---------
            if not cfg.realtime:
                continue
            next_tick += cfg.sweep_period_s
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()   # fell behind; resync


# ======================================================================
# Public entry point used by process.py
# ======================================================================
def generate_measurements(
    config: Optional[SimConfig] = None,
    objects: Optional[list[RadarObject]] = None,
) -> Iterator[dict]:
    """Drop-in replacement for the original generator.

    Signature and yielded scalar keys are unchanged, so ``process.py``
    can keep doing ``for m in generate_measurements(): ...`` verbatim.
    """
    yield from RadarSimulator(config, objects).measurements()

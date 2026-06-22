"""Dedicated attitude-board sample tracking and leveling reference management."""

from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import datetime


@dataclass(frozen=True)
class AttitudeSample:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    timestamp_local: datetime


class AttitudeService:
    """Stores latest absolute/relative attitude from the dedicated attitude board."""

    REFERENCE_CAPTURE_SECONDS = 3.0

    def __init__(self) -> None:
        self._connected = False
        self._latest_absolute: AttitudeSample | None = None
        self._latest_relative: AttitudeSample | None = None
        self._reference: AttitudeSample | None = None
        self._reference_started_s: float | None = None
        self._reference_count = 0
        self._reference_roll_sum = 0.0
        self._reference_pitch_sum = 0.0

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True
        self._latest_absolute = None
        self._latest_relative = None
        self._reference = None
        self._reference_started_s = time.monotonic()
        self._reference_count = 0
        self._reference_roll_sum = 0.0
        self._reference_pitch_sum = 0.0

    def disconnect(self) -> None:
        self._connected = False
        self._latest_absolute = None
        self._latest_relative = None
        self._reference = None
        self._reference_started_s = None
        self._reference_count = 0
        self._reference_roll_sum = 0.0
        self._reference_pitch_sum = 0.0

    def ingest_sample(self, sample: AttitudeSample) -> None:
        if not self._connected:
            return
        self._latest_absolute = sample
        self._maybe_update_reference(sample)

        reference = self._reference
        if reference is None:
            return
        self._latest_relative = AttitudeSample(
            roll_deg=float(sample.roll_deg - reference.roll_deg),
            pitch_deg=float(sample.pitch_deg - reference.pitch_deg),
            yaw_deg=float(sample.yaw_deg - reference.yaw_deg),
            timestamp_local=sample.timestamp_local,
        )

    def latest_attitude(self) -> AttitudeSample | None:
        return self._latest_relative

    def latest_absolute_attitude(self) -> AttitudeSample | None:
        return self._latest_absolute

    def attitude_reference(self) -> AttitudeSample | None:
        return self._reference

    def attitude_reference_ready(self) -> bool:
        return self._reference is not None

    def _maybe_update_reference(self, sample: AttitudeSample) -> None:
        if self._reference is not None:
            return

        self._reference_roll_sum += float(sample.roll_deg)
        self._reference_pitch_sum += float(sample.pitch_deg)
        self._reference_count += 1

        started_s = self._reference_started_s
        if started_s is None:
            started_s = time.monotonic()
            self._reference_started_s = started_s

        elapsed_s = time.monotonic() - started_s
        if elapsed_s < self.REFERENCE_CAPTURE_SECONDS:
            return

        if self._reference_count <= 0:
            self._reference = AttitudeSample(
                roll_deg=float(sample.roll_deg),
                pitch_deg=float(sample.pitch_deg),
                yaw_deg=float(sample.yaw_deg),
                timestamp_local=sample.timestamp_local,
            )
            return

        self._reference = AttitudeSample(
            roll_deg=self._reference_roll_sum / float(self._reference_count),
            pitch_deg=self._reference_pitch_sum / float(self._reference_count),
            yaw_deg=0.0,
            timestamp_local=sample.timestamp_local,
        )

"""Adaptive attitude-driven excitation controller for auto blackbox tuning sessions."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from statistics import median


class AdaptiveSessionState(str, Enum):
    idle = "idle"
    preflight = "preflight"
    adaptive_run = "adaptive_run"
    recovery = "recovery"
    finalize = "finalize"
    import_analyze = "import_analyze"
    report_ready = "report_ready"
    aborted = "aborted"


@dataclass(frozen=True)
class AdaptiveSessionConfig:
    soft_limit_deg: float = 35.0
    hard_limit_deg: float = 45.0
    recovery_entry_deg: float = 43.0
    recovery_exit_deg: float = 20.0
    telemetry_stale_s: float = 1.0
    control_interval_s: float = 0.070
    settle_deadband_deg: float = 2.5
    force_min_us: int = 100
    force_max_us: int = 360
    hold_min_s: float = 0.25
    hold_max_s: float = 0.65
    roll_force_us: int = 425
    pitch_force_us: int = 425
    roll_hold_s: float = 0.45
    pitch_hold_s: float = 0.35
    roll_target_peak_deg: float = 25.0
    pitch_target_peak_deg: float = 25.0
    settle_max_s: float = 0.85
    min_runtime_s: float = 45.0
    max_runtime_s: float = 180.0
    target_peak_min_deg: float = 8.0
    target_peak_max_deg: float = 25.0
    target_valid_events: int = 6
    target_settle_ratio: float = 0.80
    throttle_start_us: int = 1350
    throttle_max_us: int = 1600
    throttle_step_us: int = 25
    throttle_boost_peak_deg: float = 4.0
    throttle_trim_peak_deg: float = 28.0

    def axis_force_us(self, axis: str) -> int:
        return self.roll_force_us if axis == "roll" else self.pitch_force_us

    def axis_hold_s(self, axis: str) -> float:
        return self.roll_hold_s if axis == "roll" else self.pitch_hold_s

    def axis_target_peak_max_deg(self, axis: str) -> float:
        return self.roll_target_peak_deg if axis == "roll" else self.pitch_target_peak_deg

    def axis_target_peak_min_deg(self, axis: str) -> float:
        return min(self.target_peak_min_deg, self.axis_target_peak_max_deg(axis))


@dataclass(frozen=True)
class AdaptiveCommand:
    axis: str
    direction: int
    force_us: int
    hold_s: float
    settle_s: float
    recovery: bool
    reason: str


@dataclass(frozen=True)
class ExcitationEvent:
    axis: str
    direction: int
    force_us: int
    hold_s: float
    settle_s: float
    baseline_angle_deg: float
    peak_delta_deg: float
    settle_success: bool
    response_delay_s: float | None
    final_error_deg: float


@dataclass(frozen=True)
class DirectionSnapshot:
    total_count: int
    valid_count: int
    settle_ratio: float
    median_peak_deg: float
    confidence: float
    target_met: bool


@dataclass(frozen=True)
class CoverageMetrics:
    direction: dict[str, DirectionSnapshot]
    axis_confidence: dict[str, float]


@dataclass
class _DirectionCoverage:
    peak_angles: deque[float] = field(default_factory=lambda: deque(maxlen=64))
    response_delays: deque[float] = field(default_factory=lambda: deque(maxlen=64))
    total_count: int = 0
    valid_count: int = 0
    settle_success_count: int = 0

    def register_event(self, event: ExcitationEvent, cfg: AdaptiveSessionConfig) -> None:
        self.total_count += 1
        peak = abs(float(event.peak_delta_deg))
        self.peak_angles.append(peak)
        if event.response_delay_s is not None:
            self.response_delays.append(float(event.response_delay_s))

        in_band = cfg.axis_target_peak_min_deg(event.axis) <= peak <= cfg.axis_target_peak_max_deg(event.axis)
        if in_band:
            self.valid_count += 1
        if event.settle_success:
            self.settle_success_count += 1

    def settle_ratio(self) -> float:
        if self.total_count <= 0:
            return 0.0
        return self.settle_success_count / float(self.total_count)

    def median_peak(self) -> float:
        if not self.peak_angles:
            return 0.0
        return float(median(self.peak_angles))

    def target_met(self, cfg: AdaptiveSessionConfig, axis: str) -> bool:
        target_min = cfg.axis_target_peak_min_deg(axis)
        target_max = cfg.axis_target_peak_max_deg(axis)
        return (
            self.valid_count >= cfg.target_valid_events
            and target_min <= self.median_peak() <= target_max
            and self.settle_ratio() >= cfg.target_settle_ratio
        )

    def confidence(self, cfg: AdaptiveSessionConfig, axis: str) -> float:
        target_min = cfg.axis_target_peak_min_deg(axis)
        target_max = cfg.axis_target_peak_max_deg(axis)
        valid_score = min(1.0, self.valid_count / float(max(1, cfg.target_valid_events)))
        peak = self.median_peak()
        if peak <= 0:
            peak_score = 0.0
        elif target_min <= peak <= target_max:
            peak_score = 1.0
        elif peak < target_min:
            peak_score = max(0.0, peak / max(0.1, target_min))
        else:
            overflow = peak - target_max
            span = max(1.0, target_max)
            peak_score = max(0.0, 1.0 - (overflow / span))
        settle_score = min(1.0, self.settle_ratio() / max(0.01, cfg.target_settle_ratio))
        return max(0.0, min(1.0, (valid_score * 0.5) + (peak_score * 0.25) + (settle_score * 0.25)))


class AdaptiveExcitationController:
    """Closed-loop excitation selection using live attitude and coverage confidence."""

    def __init__(self, config: AdaptiveSessionConfig | None = None) -> None:
        self.config = config or AdaptiveSessionConfig()
        self._coverage: dict[tuple[str, int], _DirectionCoverage] = {
            ("roll", -1): _DirectionCoverage(),
            ("roll", +1): _DirectionCoverage(),
            ("pitch", -1): _DirectionCoverage(),
            ("pitch", +1): _DirectionCoverage(),
        }

    def record_event(self, event: ExcitationEvent) -> None:
        key = (event.axis, 1 if event.direction >= 0 else -1)
        coverage = self._coverage.get(key)
        if coverage is None:
            return
        coverage.register_event(event, self.config)

    def initial_throttle(self, current_throttle_us: int) -> tuple[int, str]:
        current = max(1000, min(2000, int(current_throttle_us)))
        target = max(self.config.throttle_start_us, current)
        target = min(self.config.throttle_max_us, target)
        if target == current:
            return current, ""
        if target > current:
            return target, f"auto throttle floor {target}us"
        return target, f"auto throttle capped {target}us"

    def throttle_after_event(self, current_throttle_us: int, event: ExcitationEvent) -> tuple[int, str]:
        current = max(1000, min(2000, int(current_throttle_us)))
        peak = abs(float(event.peak_delta_deg))
        if peak < self.config.throttle_boost_peak_deg:
            target = min(self.config.throttle_max_us, max(current, self.config.throttle_start_us) + self.config.throttle_step_us)
            if target > current:
                return target, f"boost throttle to {target}us; weak {peak:.1f}deg response"
        elif peak > self.config.throttle_trim_peak_deg:
            target = max(self.config.throttle_start_us, current - self.config.throttle_step_us)
            if target < current:
                return target, f"trim throttle to {target}us; large {peak:.1f}deg response"
        return current, ""

    def axis_confidence(self, axis: str) -> float:
        pos = self._coverage[(axis, +1)].confidence(self.config, axis)
        neg = self._coverage[(axis, -1)].confidence(self.config, axis)
        return (pos + neg) / 2.0

    def direction_confidence(self, axis: str, direction: int) -> float:
        return self._coverage[(axis, 1 if direction >= 0 else -1)].confidence(self.config, axis)

    def coverage_metrics(self) -> CoverageMetrics:
        direction: dict[str, DirectionSnapshot] = {}
        for axis in ("roll", "pitch"):
            for direction_value, direction_name in ((-1, "neg"), (1, "pos")):
                raw = self._coverage[(axis, direction_value)]
                direction[f"{axis}_{direction_name}"] = DirectionSnapshot(
                    total_count=raw.total_count,
                    valid_count=raw.valid_count,
                    settle_ratio=raw.settle_ratio(),
                    median_peak_deg=raw.median_peak(),
                    confidence=raw.confidence(self.config, axis),
                    target_met=raw.target_met(self.config, axis),
                )
        return CoverageMetrics(
            direction=direction,
            axis_confidence={
                "roll": self.axis_confidence("roll"),
                "pitch": self.axis_confidence("pitch"),
            },
        )

    def stop_ready(self, elapsed_s: float) -> tuple[bool, str, str]:
        if elapsed_s >= self.config.max_runtime_s:
            return True, "Max adaptive runtime reached.", "Reached max runtime before confidence targets were met."
        if elapsed_s < self.config.min_runtime_s:
            return False, "", ""

        roll_ok = self._coverage[("roll", -1)].target_met(self.config, "roll") and self._coverage[
            ("roll", +1)
        ].target_met(self.config, "roll")
        pitch_ok = self._coverage[("pitch", -1)].target_met(self.config, "pitch") and self._coverage[
            ("pitch", +1)
        ].target_met(self.config, "pitch")
        if roll_ok and pitch_ok:
            return True, "Coverage confidence targets reached.", ""
        return False, "", ""

    def should_abort(self, roll_deg: float, pitch_deg: float) -> tuple[bool, str]:
        if abs(float(roll_deg)) >= self.config.hard_limit_deg or abs(float(pitch_deg)) >= self.config.hard_limit_deg:
            return True, f"Hard safety limit exceeded (roll={roll_deg:+.1f}, pitch={pitch_deg:+.1f})."
        return False, ""

    def should_recover(self, roll_deg: float, pitch_deg: float) -> bool:
        return abs(float(roll_deg)) >= self.config.recovery_entry_deg or abs(float(pitch_deg)) >= self.config.recovery_entry_deg

    def recovery_complete(self, roll_deg: float, pitch_deg: float) -> bool:
        return abs(float(roll_deg)) < self.config.recovery_exit_deg and abs(float(pitch_deg)) < self.config.recovery_exit_deg

    def next_command(self, roll_deg: float, pitch_deg: float, recovery_mode: bool) -> AdaptiveCommand | None:
        if recovery_mode:
            return self._recovery_command(roll_deg, pitch_deg)

        axis = self._choose_axis()
        if axis is None:
            return None

        current_angle = float(roll_deg if axis == "roll" else pitch_deg)
        direction = self._choose_direction(axis, current_angle)
        force = self._compute_force(axis, direction, current_angle)
        if force <= 0:
            return None

        hold = self.config.axis_hold_s(axis)
        settle = self.config.settle_max_s
        return AdaptiveCommand(
            axis=axis,
            direction=direction,
            force_us=force,
            hold_s=hold,
            settle_s=settle,
            recovery=False,
            reason="coverage",
        )

    def _choose_axis(self) -> str | None:
        roll_conf = self.axis_confidence("roll")
        pitch_conf = self.axis_confidence("pitch")
        if abs(roll_conf - pitch_conf) < 0.02:
            roll_events = self._coverage[("roll", -1)].total_count + self._coverage[("roll", +1)].total_count
            pitch_events = self._coverage[("pitch", -1)].total_count + self._coverage[("pitch", +1)].total_count
            return "roll" if roll_events <= pitch_events else "pitch"
        return "roll" if roll_conf <= pitch_conf else "pitch"

    def _choose_direction(self, axis: str, current_angle: float) -> int:
        if current_angle > self.config.recovery_exit_deg:
            return -1
        if current_angle < -self.config.recovery_exit_deg:
            return +1

        neg_conf = self.direction_confidence(axis, -1)
        pos_conf = self.direction_confidence(axis, +1)
        return -1 if neg_conf <= pos_conf else +1

    def _compute_force(self, axis: str, direction: int, current_angle: float) -> int:
        # Use the configured axis pulse, but damp commands that would push farther from center near limits.
        force = int(self.config.axis_force_us(axis))

        # If commanding away from center and already tilted, reduce assertiveness.
        away_from_center = (current_angle >= 0 and direction > 0) or (current_angle < 0 and direction < 0)
        abs_angle = abs(current_angle)

        if away_from_center and abs_angle >= self.config.soft_limit_deg:
            soft_span = max(0.1, self.config.hard_limit_deg - self.config.soft_limit_deg)
            over = min(soft_span, abs_angle - self.config.soft_limit_deg)
            scale = max(0.15, 1.0 - (over / soft_span))
            force = int(round(force * scale))

        # Predicted angle guard: never command away from center near/over hard limit.
        predicted_delta = force / 15.0
        if away_from_center and (abs_angle + predicted_delta) >= self.config.hard_limit_deg:
            return 0

        return max(1, min(500, force))

    def _recovery_command(self, roll_deg: float, pitch_deg: float) -> AdaptiveCommand:
        if abs(roll_deg) >= abs(pitch_deg):
            axis = "roll"
            angle = float(roll_deg)
        else:
            axis = "pitch"
            angle = float(pitch_deg)

        direction = -1 if angle > 0 else +1
        force = max(1, min(500, int(self.config.axis_force_us(axis))))
        hold = self.config.axis_hold_s(axis)
        settle = self.config.settle_max_s
        return AdaptiveCommand(
            axis=axis,
            direction=direction,
            force_us=force,
            hold_s=hold,
            settle_s=settle,
            recovery=True,
            reason="recovery",
        )


def axis_channel_index(axis: str) -> int:
    if axis == "roll":
        return 0
    if axis == "pitch":
        return 1
    raise ValueError(f"Unsupported axis: {axis}")

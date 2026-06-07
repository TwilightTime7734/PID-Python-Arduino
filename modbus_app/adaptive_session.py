"""Randomized attitude-bounded excitation controller for auto-tune sessions."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import math
import random
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
    safety_margin_deg: float = 2.0
    recovery_entry_deg: float = 10.0
    recovery_exit_deg: float = 4.0
    telemetry_stale_s: float = 1.0
    control_interval_s: float = 0.070
    settle_deadband_deg: float = 2.5
    force_min_us: int = 100
    force_max_us: int = 220
    recovery_force_us: int = 180
    hold_min_s: float = 0.25
    hold_max_s: float = 0.35
    recovery_hold_s: float = 0.18
    roll_force_us: int = 220
    pitch_force_us: int = 220
    roll_hold_s: float = 0.25
    pitch_hold_s: float = 0.22
    roll_target_peak_deg: float = 12.0
    pitch_target_peak_deg: float = 12.0
    settle_max_s: float = 0.45
    recovery_settle_s: float = 0.05
    max_runtime_s: float = 60.0
    target_peak_min_deg: float = 4.0
    target_peak_max_deg: float = 12.0
    target_valid_events: int = 6
    target_settle_ratio: float = 0.80
    throttle_start_us: int = 1260
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
    target_peak_deg: float = 0.0


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
    total_count: int = 0
    valid_count: int = 0
    settle_success_count: int = 0

    def register_event(self, event: ExcitationEvent, cfg: AdaptiveSessionConfig) -> None:
        self.total_count += 1
        peak = abs(float(event.peak_delta_deg))
        self.peak_angles.append(peak)
        if cfg.axis_target_peak_min_deg(event.axis) <= peak <= cfg.axis_target_peak_max_deg(event.axis):
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

    def confidence(self, cfg: AdaptiveSessionConfig, axis: str) -> float:
        target = max(1, cfg.target_valid_events)
        valid_score = min(1.0, self.valid_count / float(target))
        settle_score = min(1.0, self.settle_ratio() / max(0.01, cfg.target_settle_ratio))
        return max(0.0, min(1.0, (valid_score * 0.65) + (settle_score * 0.35)))

    def target_met(self, cfg: AdaptiveSessionConfig, axis: str) -> bool:
        return self.valid_count >= cfg.target_valid_events and self.settle_ratio() >= cfg.target_settle_ratio


class AdaptiveExcitationController:
    """Random roll/pitch excitation bounded by live attitude and configured runtime."""

    def __init__(self, config: AdaptiveSessionConfig | None = None, rng: random.Random | None = None) -> None:
        self.config = config or AdaptiveSessionConfig()
        self._rng = rng or random.Random()
        self._pending_commands: deque[AdaptiveCommand] = deque()
        self._coverage: dict[tuple[str, int], _DirectionCoverage] = {
            ("roll", -1): _DirectionCoverage(),
            ("roll", +1): _DirectionCoverage(),
            ("pitch", -1): _DirectionCoverage(),
            ("pitch", +1): _DirectionCoverage(),
        }

    def record_event(self, event: ExcitationEvent) -> None:
        key = (event.axis, 1 if event.direction >= 0 else -1)
        coverage = self._coverage.get(key)
        if coverage is not None:
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

    def throttle_after_event(self, current_throttle_us: int, _event: ExcitationEvent) -> tuple[int, str]:
        return max(1000, min(2000, int(current_throttle_us))), ""

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
            runtime_s = max(0.0, float(self.config.max_runtime_s))
            if abs(runtime_s - round(runtime_s)) < 0.05:
                duration_text = f"{runtime_s:.0f}-second"
            else:
                duration_text = f"{runtime_s:.1f}-second"
            return True, f"Randomized {duration_text} auto tune complete.", ""
        return False, "", ""

    def should_abort(self, roll_deg: float, pitch_deg: float) -> tuple[bool, str]:
        if abs(float(roll_deg)) >= self.config.hard_limit_deg or abs(float(pitch_deg)) >= self.config.hard_limit_deg:
            return True, f"Hard safety limit exceeded (roll={roll_deg:+.1f}, pitch={pitch_deg:+.1f})."
        return False, ""

    def should_recover(self, roll_deg: float, pitch_deg: float) -> bool:
        return (
            abs(float(roll_deg)) >= self.config.recovery_entry_deg
            or abs(float(pitch_deg)) >= self.config.recovery_entry_deg
        )

    def recovery_complete(self, roll_deg: float, pitch_deg: float) -> bool:
        return (
            abs(float(roll_deg)) <= self.config.recovery_exit_deg
            and abs(float(pitch_deg)) <= self.config.recovery_exit_deg
        )

    def next_command(self, roll_deg: float, pitch_deg: float, recovery_mode: bool = False) -> AdaptiveCommand | None:
        if recovery_mode:
            self._pending_commands.clear()
            return self._recovery_command(float(roll_deg), float(pitch_deg))
        if not self._pending_commands:
            self._queue_random_cycle(float(roll_deg), float(pitch_deg))
        if not self._pending_commands:
            return None
        return self._pending_commands.popleft()

    def _queue_random_cycle(self, roll_deg: float, pitch_deg: float) -> None:
        directions = {
            "roll": self._rng.choice((-1, 1)),
            "pitch": self._rng.choice((-1, 1)),
        }
        axes = ["roll", "pitch"]
        self._rng.shuffle(axes)
        for axis in axes:
            angle = roll_deg if axis == "roll" else pitch_deg
            command = self._random_command_for_axis(axis, angle, directions[axis])
            if command is not None:
                self._pending_commands.append(command)

    def _random_command_for_axis(self, axis: str, current_angle: float, direction: int) -> AdaptiveCommand | None:
        safe_direction = 1 if direction >= 0 else -1
        max_delta = self._safe_delta_deg(current_angle, safe_direction)
        min_delta = self.config.axis_target_peak_min_deg(axis)
        if max_delta < min_delta:
            safe_direction = self._toward_center_direction(current_angle)
            max_delta = self._safe_delta_deg(current_angle, safe_direction)
        if max_delta < min_delta:
            return None

        max_delta = min(max_delta, self.config.axis_target_peak_max_deg(axis))
        if max_delta < min_delta:
            return None

        command = self._try_random_force_time(axis, safe_direction, min_delta, max_delta)
        if command is not None:
            return command
        return self._fallback_force_time(axis, safe_direction, min_delta, max_delta)

    def _try_random_force_time(
        self, axis: str, direction: int, min_delta_deg: float, max_delta_deg: float
    ) -> AdaptiveCommand | None:
        base_hold = max(0.05, self.config.axis_hold_s(axis))
        axis_force_max = min(self.config.force_max_us, int(self.config.axis_force_us(axis)))
        for _ in range(80):
            hold_s = self._rng.uniform(self.config.hold_min_s, self.config.hold_max_s)
            min_force = math.ceil(min_delta_deg * 15.0 * base_hold / hold_s)
            max_force = math.floor(max_delta_deg * 15.0 * base_hold / hold_s)
            min_force = max(self.config.force_min_us, min_force)
            max_force = min(axis_force_max, max_force)
            if min_force > max_force:
                continue
            force_us = self._rng.randint(min_force, max_force)
            target_peak = self._predicted_peak_deg(axis, force_us, hold_s)
            return AdaptiveCommand(
                axis=axis,
                direction=direction,
                force_us=force_us,
                hold_s=hold_s,
                settle_s=self.config.settle_max_s,
                recovery=False,
                reason="random bounded",
                target_peak_deg=target_peak,
            )
        return None

    def _fallback_force_time(
        self, axis: str, direction: int, min_delta_deg: float, max_delta_deg: float
    ) -> AdaptiveCommand | None:
        hold_s = max(self.config.hold_min_s, min(self.config.hold_max_s, self.config.axis_hold_s(axis)))
        base_hold = max(0.05, self.config.axis_hold_s(axis))
        axis_force_max = min(self.config.force_max_us, int(self.config.axis_force_us(axis)))
        target_delta = min(max_delta_deg, max(min_delta_deg, (min_delta_deg + max_delta_deg) / 2.0))
        force_us = round(target_delta * 15.0 * base_hold / hold_s)
        force_us = max(self.config.force_min_us, min(axis_force_max, force_us))
        target_peak = self._predicted_peak_deg(axis, force_us, hold_s)
        if target_peak < min_delta_deg or target_peak > max_delta_deg:
            return None
        return AdaptiveCommand(
            axis=axis,
            direction=direction,
            force_us=force_us,
            hold_s=hold_s,
            settle_s=self.config.settle_max_s,
            recovery=False,
            reason="random bounded fallback",
            target_peak_deg=target_peak,
        )

    def _safe_delta_deg(self, current_angle: float, direction: int) -> float:
        limit = max(0.0, self.config.hard_limit_deg - self.config.safety_margin_deg)
        if direction >= 0:
            return max(0.0, limit - current_angle)
        return max(0.0, current_angle + limit)

    def _toward_center_direction(self, current_angle: float) -> int:
        if current_angle > 0:
            return -1
        if current_angle < 0:
            return 1
        return self._rng.choice((-1, 1))

    def _recovery_command(self, roll_deg: float, pitch_deg: float) -> AdaptiveCommand | None:
        axis = "roll" if abs(roll_deg) >= abs(pitch_deg) else "pitch"
        angle = roll_deg if axis == "roll" else pitch_deg
        if abs(angle) <= self.config.recovery_exit_deg:
            other_axis = "pitch" if axis == "roll" else "roll"
            other_angle = pitch_deg if axis == "roll" else roll_deg
            if abs(other_angle) <= self.config.recovery_exit_deg:
                return None
            axis = other_axis
            angle = other_angle

        direction = self._toward_center_direction(angle)
        force_us = round(abs(angle) * 8.0)
        force_us = max(self.config.force_min_us, min(int(self.config.recovery_force_us), force_us))
        target_peak = max(0.0, abs(angle) - self.config.recovery_exit_deg)
        return AdaptiveCommand(
            axis=axis,
            direction=direction,
            force_us=force_us,
            hold_s=max(0.05, self.config.recovery_hold_s),
            settle_s=max(0.0, self.config.recovery_settle_s),
            recovery=True,
            reason="attitude recovery",
            target_peak_deg=target_peak,
        )

    def _predicted_peak_deg(self, axis: str, force_us: int, hold_s: float) -> float:
        base_hold = max(0.05, self.config.axis_hold_s(axis))
        return max(0.0, (float(force_us) / 15.0) * (float(hold_s) / base_hold))


def axis_channel_index(axis: str) -> int:
    if axis == "roll":
        return 0
    if axis == "pitch":
        return 1
    raise ValueError(f"Unsupported axis: {axis}")

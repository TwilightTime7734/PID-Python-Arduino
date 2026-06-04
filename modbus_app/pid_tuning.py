"""PID response analysis and tuning suggestion helpers."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class PIDRecommendation:
    """Response metrics plus a conservative plain-language suggestion."""

    oscillation_count: int
    settling_time: float | None
    steady_state_error: float
    overshoot_ratio: float
    recommendation: str


class PIDAnalyzer:
    """Analyze one response trace and suggest the next small PID tuning step."""

    def analyze(self, time_data: Sequence[float], value_data: Sequence[float], target_value: float) -> PIDRecommendation:
        if len(time_data) != len(value_data):
            raise ValueError("time_data and value_data must have the same length")
        if len(time_data) < 3:
            raise ValueError("Need at least three samples to analyze a response")

        times = [float(v) for v in time_data]
        values = [float(v) for v in value_data]
        target = float(target_value)
        errors = [target - value for value in values]
        tolerance = max(abs(target) * 0.05, 1.0)

        oscillations = self._count_oscillations(errors, tolerance)
        settling_time = None
        for i in range(len(errors)):
            if all(abs(error) <= tolerance for error in errors[i:]):
                settling_time = float(times[i])
                break

        final_window = errors[-min(20, len(errors)) :]
        steady_state_error = self._mean(final_window)
        overshoot_ratio = self._overshoot_ratio(values, target)

        recommendation = self._recommend(
            oscillations,
            settling_time,
            steady_state_error,
            overshoot_ratio,
            float(times[-1]),
            tolerance,
        )
        return PIDRecommendation(oscillations, settling_time, steady_state_error, overshoot_ratio, recommendation)

    def _mean(self, values: Sequence[float]) -> float:
        return sum(values) / len(values)

    def _count_oscillations(self, errors: Sequence[float], tolerance: float) -> int:
        # Ignore tiny sign flips caused by quantization/jitter near the target.
        deadband = max(tolerance * 0.35, 0.25)
        signs: list[int] = []
        for error in errors:
            if error > deadband:
                sign = 1
            elif error < -deadband:
                sign = -1
            else:
                continue
            if not signs or sign != signs[-1]:
                signs.append(sign)
        return sum(1 for prev, curr in zip(signs, signs[1:]) if prev != curr)

    def _overshoot_ratio(self, values: Sequence[float], target: float) -> float:
        target_abs = max(abs(target), 0.001)
        if target >= 0:
            peak = max(values)
            overshoot = max(0.0, peak - target)
        else:
            peak = min(values)
            overshoot = max(0.0, target - peak)
        return overshoot / target_abs

    def _recommend(
        self,
        oscillations: int,
        settling_time: float | None,
        steady_state_error: float,
        overshoot_ratio: float,
        run_duration: float,
        tolerance: float,
    ) -> str:
        if oscillations >= 6 or overshoot_ratio > 0.35:
            return "Oscillation is high. Reduce P slightly, then increase D in small steps."
        if settling_time is None or settling_time > run_duration * 0.75:
            return "Response is slow to settle. Increase P slightly; add D if overshoot appears."
        if abs(steady_state_error) > tolerance:
            return "Steady-state error remains. Add a small amount of I or increase I gradually."
        if overshoot_ratio > 0.15:
            return "Overshoot is visible. Increase D slightly; reduce P a touch if needed."
        if oscillations <= 1 and overshoot_ratio < 0.10:
            return "Response is stable. Try a small P increase if you want faster tracking."
        return "Tune is close. Make one small change at a time and re-run this axis."

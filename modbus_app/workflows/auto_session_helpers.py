"""Shared helpers for the adaptive auto-session runtime."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

from ..adaptive_session import AdaptiveCommand, AdaptiveSessionState, ExcitationEvent
from ..constants import PULSE_STATUS_REJECTED, THROTTLE_CHANNEL_INDEX
from ..tasks.worker_tasks import cancel_active_pulse as worker_cancel_active_pulse


class AutoSessionHelpers:
    ARDUINO_MILLIS_WRAP = 1 << 32

    def __init__(
        self,
        app,
        *,
        auto_is_running: Callable[[], bool],
        auto_abort: Callable[..., None],
        schedule_auto_tick: Callable[..., None],
        set_live_channel_outputs: Callable[[list[int]], None],
        arduino_output_connected: Callable[[], bool],
        queue_live_channel_update: Callable[..., None],
        parse_offset_values_with_defaults: Callable[[], list[int]],
        apply_auto_base_outputs: Callable[..., None],
    ) -> None:
        self.app = app
        self.auto_is_running = auto_is_running
        self.auto_abort = auto_abort
        self.schedule_auto_tick = schedule_auto_tick
        self.set_live_channel_outputs = set_live_channel_outputs
        self.arduino_output_connected = arduino_output_connected
        self.queue_live_channel_update = queue_live_channel_update
        self.parse_offset_values_with_defaults = parse_offset_values_with_defaults
        self.apply_auto_base_outputs = apply_auto_base_outputs

    @staticmethod
    def pulse_axis_value(sample: Any, axis: str) -> float:
        if axis == "roll":
            return float(sample.roll_deg)
        return float(sample.pitch_deg)

    @classmethod
    def arduino_elapsed_s(cls, start_millis: int, current_millis: int) -> float:
        if start_millis is None or current_millis is None:
            raise ValueError("Arduino movement_millis is required for attitude timing.")
        delta_ms = (int(current_millis) - int(start_millis)) % cls.ARDUINO_MILLIS_WRAP
        return max(0.0, float(delta_ms) / 1000.0)

    def elapsed_s(self, now_s: float | None = None) -> float:
        app = self.app
        if app.auto_session_start_s is None:
            return 0.0
        current = time.monotonic() if now_s is None else now_s
        return max(0.0, current - app.auto_session_start_s)

    def set_state(self, next_state: AdaptiveSessionState, safety_text: str = "") -> None:
        app = self.app
        app.auto_state = next_state
        if safety_text and safety_text != "--":
            app.status.set(safety_text)

    def payload(self) -> dict[str, object]:
        app = self.app
        metrics: dict[str, object] = {}
        if app.auto_controller is not None:
            snapshot = app.auto_controller.coverage_metrics()
            metrics = {
                "axis_confidence": snapshot.axis_confidence,
                "direction": {
                    key: {
                        "total_count": value.total_count,
                        "valid_count": value.valid_count,
                        "settle_ratio": value.settle_ratio,
                        "median_peak_deg": value.median_peak_deg,
                        "confidence": value.confidence,
                        "target_met": value.target_met,
                    }
                    for key, value in snapshot.direction.items()
                },
            }
        return {
            "state": app.auto_state.value,
            "stop_reason": app.auto_stop_reason,
            "warning": app.auto_warning,
            "elapsed_s": self.elapsed_s(),
            "metrics": metrics,
            "start_throttle_us": app.auto_start_throttle_us,
            "current_throttle_us": app.auto_current_throttle_us,
            "peak_throttle_us": app.auto_peak_throttle_us,
        }

    def record_sample(self, sample: Any) -> None:
        app = self.app
        app.auto_last_sample_s = time.monotonic()
        command = app.auto_active_command
        if command is None:
            return
        if app.auto_controller is not None and app.auto_pulse_inflight:
            stop_limit = max(0.0, app.auto_controller.config.hard_limit_deg - app.auto_controller.config.safety_margin_deg)
            if abs(float(sample.roll_deg)) >= stop_limit or abs(float(sample.pitch_deg)) >= stop_limit:
                self.auto_abort(
                    f"Emergency attitude limit reached (roll={sample.roll_deg:+.1f}, pitch={sample.pitch_deg:+.1f}).",
                    continue_pipeline=False,
                )
                return
        axis_value = self.pulse_axis_value(sample, command.axis)
        signed_delta = axis_value - app.auto_event_baseline
        abs_delta = abs(signed_delta)
        if abs_delta > app.auto_event_abs_peak_delta:
            app.auto_event_abs_peak_delta = float(abs_delta)
            app.auto_event_signed_peak_delta = float(signed_delta)
        directed_delta = signed_delta * float(command.direction)
        if directed_delta > app.auto_event_peak_delta:
            app.auto_event_peak_delta = float(directed_delta)
        start_millis = app.auto_event_start_millis
        if start_millis is None:
            raise RuntimeError("Auto attitude timing requires Arduino movement_millis at command start.")
        sample_elapsed_s = self.arduino_elapsed_s(start_millis, sample.movement_millis)
        within_hold_window = sample_elapsed_s <= command.hold_s
        target_peak_deg = command.target_peak_deg
        if target_peak_deg <= 0 and app.auto_controller is not None:
            target_peak_deg = app.auto_controller.config.axis_target_peak_max_deg(command.axis)
        if app.auto_controller is not None:
            cfg = app.auto_controller.config
            emergency_peak_deg = cfg.probe_emergency_peak_deg if command.calibration else cfg.emergency_peak_deg
            if emergency_peak_deg > 0 and abs_delta >= emergency_peak_deg:
                self.auto_abort(
                    f"Emergency {command.axis} movement limit exceeded ({abs_delta:.1f} deg).",
                    continue_pipeline=False,
                )
                return
        if app.auto_controller is not None and app.auto_pulse_inflight and within_hold_window:
            if target_peak_deg > 0 and abs_delta >= target_peak_deg:
                self.request_angle_hold_end(command)
                return
        if (
            app.auto_controller is not None
            and app.auto_pulse_inflight
            and within_hold_window
            and target_peak_deg > 0
            and directed_delta >= target_peak_deg
        ):
            self.request_angle_hold_end(command)
        if app.auto_event_response_delay_s is None:
            threshold_deg = max(2.0, (command.force_us / 15.0) * 0.35)
            if directed_delta >= threshold_deg:
                app.auto_event_response_delay_s = sample_elapsed_s

    def cancel_hold_timer(self) -> None:
        app = self.app
        if app.auto_hold_after_id is not None:
            try:
                app.root.after_cancel(app.auto_hold_after_id)
            except Exception:
                pass
            finally:
                app.auto_hold_after_id = None

    def begin_observe_window(self, command: AdaptiveCommand) -> None:
        app = self.app
        if not self.auto_is_running():
            return
        app.auto_pulse_inflight = False
        self.set_live_channel_outputs(app.base_channel_outputs)
        app.auto_settle_until_s = time.monotonic() + command.settle_s
        self.schedule_auto_tick(delay_ms=round(command.settle_s * 1000.0))

    def request_angle_hold_end(self, command: AdaptiveCommand) -> None:
        app = self.app
        if app.auto_hold_end_requested or not self.auto_is_running() or not self.arduino_output_connected():
            return
        app.auto_hold_end_requested = True
        self.cancel_hold_timer()
        self.set_live_channel_outputs(app.base_channel_outputs)
        self.begin_observe_window(command)

        def on_auto_hold_end_done(ok: bool, res: object) -> None:
            if not self.auto_is_running():
                return
            if not ok:
                self.auto_abort(
                    "Unable to end adaptive pulse on angle threshold.",
                    warning=str(res) if not isinstance(res, Exception) else str(res),
                )
                return
            if not isinstance(res, int):
                self.auto_abort("Unexpected hold-end result from worker.")
                return
            if res == PULSE_STATUS_REJECTED:
                self.auto_abort("Firmware rejected adaptive hold-end command.")
                return
            if self.arduino_output_connected():
                self.queue_live_channel_update(
                    app.base_channel_outputs.copy(),
                    self.parse_offset_values_with_defaults(),
                )

        app.worker.submit(worker_cancel_active_pulse, callback=on_auto_hold_end_done)

    def adjust_throttle_after_event(self, event: ExcitationEvent, recovery_event: bool) -> None:
        app = self.app
        if app.auto_controller is None or recovery_event:
            return
        current = app.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        target, reason = app.auto_controller.throttle_after_event(current, event)
        if target == current:
            return
        channels = app.base_channel_outputs.copy()
        channels[THROTTLE_CHANNEL_INDEX] = target
        self.apply_auto_base_outputs(channels, reason)

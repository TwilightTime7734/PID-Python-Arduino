"""Auto-level workflow for the Level button."""

from __future__ import annotations

import time
from collections.abc import Callable

from ..constants import (
    ARDUINO_FIXED_PULSE_HOLD_S,
    LEVEL_CENTER_US,
    LEVEL_DEADBAND_DEG,
    PITCH_CHANNEL_INDEX,
    PULSE_STATUS_REJECTED,
    ROLL_CHANNEL_INDEX,
    THROTTLE_CHANNEL_INDEX,
)
from ..tasks.worker_tasks import pulse_channel_force as worker_pulse_channel_force


class LevelWorkflow:
    """Owns the Level button state and auto-level pulse loop."""

    # Fixed-dose corrections. Angle feedback is used only to choose the
    # direction, choose coarse vs fine mode, and decide whether to continue.
    # It is never used to scale into a big pulse, which avoids the overshoot
    # seen with the older angle-chasing level logic.
    COARSE_FORCE_US = 35
    FINE_FORCE_US = 14
    FINE_TUNE_ENTER_DEG = 4.0
    CORRECTION_SETTLE_MS = 250
    EMERGENCY_ATTITUDE_DEG = 70.0
    # The old timeout was too short for large initial errors. One press of
    # Level should keep nudging until it is centered, the safety limit is hit,
    # or the pulse budget is exhausted.
    MAX_LEVEL_RUN_S = 45.0
    MAX_CORRECTION_PULSES = 90

    def __init__(
        self,
        app,
        *,
        set_live_channel_outputs: Callable[[list[int]], None],
        queue_live_channel_update: Callable[..., None],
        update_link_indicators: Callable[[], None],
        arduino_output_connected: Callable[[], bool],
        parse_offsets: Callable[[], list[int]],
        get_test_throttle_us: Callable[[], int],
        set_error: Callable[[str, Exception], None],
    ) -> None:
        self.app = app
        self.set_live_channel_outputs = set_live_channel_outputs
        self.queue_live_channel_update = queue_live_channel_update
        self.update_link_indicators = update_link_indicators
        self.arduino_output_connected = arduino_output_connected
        self.parse_offsets = parse_offsets
        self.get_test_throttle_us = get_test_throttle_us
        self.set_error = set_error

    def refresh_button_state(self, attitude_connected: bool) -> None:
        app = self.app
        level_ready = app.controller.is_connected and attitude_connected
        if app.level_active and not level_ready:
            self.stop(update_status=False)
        app.level_button.config(
            state="normal" if level_ready else "disabled",
            relief="sunken" if app.level_active else "raised",
        )

    def cancel_timer(self) -> None:
        app = self.app
        if app.level_after_id is not None:
            try:
                app.root.after_cancel(app.level_after_id)
            except Exception:
                pass
            finally:
                app.level_after_id = None

    def stop(self, update_status: bool = False, reason: str = "Auto-level stopped.") -> None:
        app = self.app
        was_active = app.level_active
        self.cancel_timer()
        app.level_active = False
        app.level_pulse_inflight = False
        app.level_timeout_deadline_s = None
        self.set_live_channel_outputs(app.base_channel_outputs)
        self.update_link_indicators()
        if update_status and was_active and not app.is_closing:
            app.status.set(reason)

    def correction_from_angle(self, angle_deg: float) -> tuple[int, int, float, str] | None:
        abs_angle = abs(float(angle_deg))
        if abs_angle <= LEVEL_DEADBAND_DEG:
            return None
        if abs_angle <= self.FINE_TUNE_ENTER_DEG:
            delta = self.FINE_FORCE_US
            mode = "fine"
        else:
            delta = self.COARSE_FORCE_US
            mode = "coarse"
        if angle_deg > 0:
            return LEVEL_CENTER_US - delta, delta, ARDUINO_FIXED_PULSE_HOLD_S, mode
        return LEVEL_CENTER_US + delta, delta, ARDUINO_FIXED_PULSE_HOLD_S, mode

    def attitude_is_settled(self, roll_deg: float, pitch_deg: float) -> bool:
        return abs(roll_deg) <= LEVEL_DEADBAND_DEG and abs(pitch_deg) <= LEVEL_DEADBAND_DEG

    def schedule_step(self, delay_ms: int | None = None) -> None:
        app = self.app
        self.cancel_timer()
        delay = self.CORRECTION_SETTLE_MS if delay_ms is None else delay_ms
        app.level_after_id = app.root.after(max(1, delay), self.run_step)

    def run_step(self) -> None:
        app = self.app
        app.level_after_id = None
        if not app.level_active:
            return
        if not self.arduino_output_connected():
            self.stop(update_status=True, reason="Auto-level stopped: output is not running.")
            return
        if not app.attitude_service.is_connected:
            self.stop(update_status=True, reason="Auto-level stopped: attitude board is disconnected.")
            return
        if app.level_timeout_deadline_s is not None and time.monotonic() >= app.level_timeout_deadline_s:
            self.stop(
                update_status=True,
                reason=(
                    f"Auto-level stopped after {self.MAX_LEVEL_RUN_S:.0f}s. "
                    "Press Level again only if it is still safely away from the hard stops."
                ),
            )
            return
        pulse_count = int(getattr(app, "level_pulse_count", 0))
        if pulse_count >= self.MAX_CORRECTION_PULSES:
            self.stop(
                update_status=True,
                reason=(
                    f"Auto-level stopped after {self.MAX_CORRECTION_PULSES} correction nudges. "
                    "Press Level again only if it is still safely away from the hard stops."
                ),
            )
            return
        if app.level_pulse_inflight:
            self.schedule_step()
            return

        sample = app.attitude_service.latest_attitude()
        if sample is None:
            self.schedule_step()
            return
        absolute_sample = None
        try:
            absolute_sample = app.attitude_service.latest_absolute_attitude()
        except Exception:
            absolute_sample = None
        safety_sample = absolute_sample or sample
        if (
            abs(float(safety_sample.roll_deg)) >= self.EMERGENCY_ATTITUDE_DEG
            or abs(float(safety_sample.pitch_deg)) >= self.EMERGENCY_ATTITUDE_DEG
        ):
            self.stop(
                update_status=True,
                reason=(
                    f"Auto-level stopped: attitude safety limit reached "
                    f"(roll={float(safety_sample.roll_deg):+.1f}, "
                    f"pitch={float(safety_sample.pitch_deg):+.1f})."
                ),
            )
            return
        if self.attitude_is_settled(sample.roll_deg, sample.pitch_deg):
            self.stop(update_status=True, reason="Auto-level complete: roll and pitch are centered.")
            return

        roll_correction = self.correction_from_angle(sample.roll_deg)
        pitch_correction = self.correction_from_angle(sample.pitch_deg)
        axis_targets: list[tuple[int, int, float, int, float, str]] = []
        if roll_correction is not None:
            target_us, force_us, hold_s, mode = roll_correction
            axis_targets.append((ROLL_CHANNEL_INDEX, target_us, abs(sample.roll_deg), force_us, hold_s, mode))
        if pitch_correction is not None:
            target_us, force_us, hold_s, mode = pitch_correction
            axis_targets.append((PITCH_CHANNEL_INDEX, target_us, abs(sample.pitch_deg), force_us, hold_s, mode))
        if not axis_targets:
            self.stop(update_status=True, reason="Auto-level complete: roll and pitch are centered.")
            return
        channel_index, target_us, angle_error_deg, force_us, hold_s, correction_mode = max(
            axis_targets, key=lambda item: item[2]
        )

        if len(app.base_channel_outputs) <= channel_index:
            self.stop(update_status=False)
            self.set_error("Level error", RuntimeError("Base channel output list is too short for auto-level pulse"))
            return
        base_value = int(app.base_channel_outputs[channel_index])
        signed_force_us = target_us - base_value

        active_outputs = app.base_channel_outputs.copy()
        active_outputs[channel_index] = target_us
        self.set_live_channel_outputs(active_outputs)
        axis_name = "roll" if channel_index == ROLL_CHANNEL_INDEX else "pitch"
        direction = target_us - LEVEL_CENTER_US
        direction_text = "+" if direction >= 0 else ""
        pulse_count = int(getattr(app, "level_pulse_count", 0)) + 1
        app.level_pulse_count = pulse_count
        app.status.set(
            f"Auto-level {correction_mode} nudge {pulse_count}/{self.MAX_CORRECTION_PULSES}: "
            f"{axis_name} CH{channel_index + 1} {LEVEL_CENTER_US}->{target_us} "
            f"({direction_text}{direction}us) for board-fixed {ARDUINO_FIXED_PULSE_HOLD_S:.2f}s; "
            f"error {angle_error_deg:.1f} deg; waiting {self.CORRECTION_SETTLE_MS}ms."
        )
        app.level_pulse_inflight = True

        def on_level_pulse_done(ok: bool, res: object) -> None:
            app.level_pulse_inflight = False
            if not app.level_active:
                return
            if not ok:
                self.stop(update_status=False)
                self.set_error("Level error", res if isinstance(res, Exception) else RuntimeError(res))
                return
            if not isinstance(res, int):
                self.stop(update_status=False)
                self.set_error("Level error", RuntimeError("Unexpected worker result from level task"))
                return
            if res == PULSE_STATUS_REJECTED:
                self.stop(update_status=False)
                self.set_error("Level error", RuntimeError("Firmware rejected auto-level pulse"))
                return
            self.schedule_step()

        app.worker.submit(
            worker_pulse_channel_force,
            channel_index,
            signed_force_us,
            callback=on_level_pulse_done,
        )

    def _prepare_shared_test_throttle_and_start(self) -> None:
        app = self.app
        if len(app.base_channel_outputs) <= THROTTLE_CHANNEL_INDEX:
            raise RuntimeError("Channel output list is too short to set test throttle.")
        test_throttle_us = max(1000, min(2000, int(self.get_test_throttle_us())))
        channels = app.base_channel_outputs.copy()
        channels[THROTTLE_CHANNEL_INDEX] = test_throttle_us
        app.base_channel_outputs = channels.copy()
        self.set_live_channel_outputs(channels)
        offsets = self.parse_offsets()

        def on_test_throttle_ready(ok: bool, res: object) -> None:
            if not app.level_active:
                return
            if not ok:
                self.stop(update_status=False)
                self.set_error("Level error", res if isinstance(res, Exception) else RuntimeError(str(res)))
                return
            reference = None
            try:
                reference = app.attitude_service.attitude_reference()
            except Exception:
                reference = None
            ref_text = ""
            if reference is not None:
                ref_text = f" Level reference roll={reference.roll_deg:+.1f}, pitch={reference.pitch_deg:+.1f}."
            app.status.set(
                f"Auto-level active at shared test throttle {test_throttle_us}us. "
                f"Coarse +/-{self.COARSE_FORCE_US}us above "
                f"{self.FINE_TUNE_ENTER_DEG:.1f} deg; fine +/-{self.FINE_FORCE_US}us "
                f"near center; board pulse duration {ARDUINO_FIXED_PULSE_HOLD_S:.2f}s; "
                f"settle {self.CORRECTION_SETTLE_MS}ms. "
                f"Leveling is relative to the captured quiet reference.{ref_text} "
                f"One press continues until centered, safety stop, {self.MAX_CORRECTION_PULSES} nudges, "
                f"or {self.MAX_LEVEL_RUN_S:.0f}s. Press Level again to stop."
            )
            self.run_step()

        self.queue_live_channel_update(channels, offsets, after_update=on_test_throttle_ready)

    def toggle(self) -> None:
        app = self.app
        try:
            if app.level_active:
                self.stop(update_status=True)
                return
            if not self.arduino_output_connected():
                raise RuntimeError("Press Connect Arduino before using Level.")
            if not app.attitude_service.is_connected:
                raise RuntimeError("Connect Arduino/attitude board before using Level.")
            reference_ready = True
            try:
                reference_ready = app.attitude_service.attitude_reference_ready()
            except Exception:
                reference_ready = app.attitude_service.latest_attitude() is not None
            if not reference_ready or app.attitude_service.latest_attitude() is None:
                raise RuntimeError(
                    "No attitude-board level reference yet. Keep the drone still for 3 seconds after Arduino connect, "
                    "then press Level again."
                )
            app.level_active = True
            app.level_pulse_count = 0
            app.level_timeout_deadline_s = time.monotonic() + self.MAX_LEVEL_RUN_S
            self.update_link_indicators()
            self._prepare_shared_test_throttle_and_start()
        except Exception as exc:
            self.stop(update_status=False)
            self.set_error("Level error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

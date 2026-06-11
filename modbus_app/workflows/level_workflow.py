"""Auto-level workflow for the Level button."""

from __future__ import annotations

import time
from collections.abc import Callable

from ..constants import (
    LEVEL_CENTER_US,
    LEVEL_DEADBAND_DEG,
    LEVEL_FULL_SCALE_DEG,
    LEVEL_LOOP_INTERVAL_MS,
    LEVEL_MAX_DELTA_US,
    LEVEL_MIN_DELTA_US,
    LEVEL_PULSE_TIMEOUT_S,
    LEVEL_TIMEOUT_DEFAULT_S,
    PITCH_CHANNEL_INDEX,
    PULSE_STATUS_REJECTED,
    ROLL_CHANNEL_INDEX,
    THROTTLE_CHANNEL_INDEX,
)
from ..tasks.worker_tasks import hold_channel_until_stop as worker_hold_channel_until_stop


class LevelWorkflow:
    """Owns the Level button state and auto-level pulse loop."""

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

    def refresh_button_state(self, fc_connected: bool) -> None:
        app = self.app
        level_ready = app.controller.is_connected and fc_connected
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

    def target_from_angle(self, angle_deg: float) -> int | None:
        abs_angle = abs(angle_deg)
        if abs_angle <= LEVEL_DEADBAND_DEG:
            return None
        ratio = min(1.0, abs_angle / LEVEL_FULL_SCALE_DEG)
        delta = max(LEVEL_MIN_DELTA_US, round(LEVEL_MAX_DELTA_US * ratio))
        if angle_deg > 0:
            return LEVEL_CENTER_US - delta
        return LEVEL_CENTER_US + delta

    def attitude_is_settled(self, roll_deg: float, pitch_deg: float) -> bool:
        return abs(roll_deg) <= LEVEL_DEADBAND_DEG and abs(pitch_deg) <= LEVEL_DEADBAND_DEG

    def schedule_step(self, delay_ms: int = LEVEL_LOOP_INTERVAL_MS) -> None:
        app = self.app
        self.cancel_timer()
        app.level_after_id = app.root.after(max(1, delay_ms), self.run_step)

    def run_step(self) -> None:
        app = self.app
        app.level_after_id = None
        if not app.level_active:
            return
        if not self.arduino_output_connected():
            self.stop(update_status=True, reason="Auto-level stopped: output is not running.")
            return
        if not app.fc_service.is_connected:
            self.stop(update_status=True, reason="Auto-level stopped: FC is disconnected.")
            return
        if app.level_timeout_deadline_s is not None and time.monotonic() >= app.level_timeout_deadline_s:
            self.stop(update_status=True, reason=f"Auto-level timed out after {LEVEL_TIMEOUT_DEFAULT_S:.3g}s.")
            return
        if app.level_pulse_inflight:
            self.schedule_step()
            return

        sample = app.fc_service.latest_attitude()
        if sample is None:
            self.schedule_step()
            return
        if self.attitude_is_settled(sample.roll_deg, sample.pitch_deg):
            self.stop(update_status=True, reason="Auto-level complete: roll and pitch are centered.")
            return

        roll_target_us = self.target_from_angle(sample.roll_deg)
        pitch_target_us = self.target_from_angle(sample.pitch_deg)
        axis_targets: list[tuple[int, int, float]] = []
        if roll_target_us is not None:
            axis_targets.append((ROLL_CHANNEL_INDEX, roll_target_us, abs(sample.roll_deg)))
        if pitch_target_us is not None:
            axis_targets.append((PITCH_CHANNEL_INDEX, pitch_target_us, abs(sample.pitch_deg)))
        if not axis_targets:
            self.stop(update_status=True, reason="Auto-level complete: roll and pitch are centered.")
            return
        channel_index, target_us, _ = max(axis_targets, key=lambda item: item[2])

        try:
            offsets = self.parse_offsets()
        except Exception as exc:
            self.stop(update_status=False)
            self.set_error("Level error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))
            return

        active_outputs = app.base_channel_outputs.copy()
        active_outputs[channel_index] = target_us
        self.set_live_channel_outputs(active_outputs)
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
            worker_hold_channel_until_stop,
            channel_index,
            target_us,
            offsets[channel_index],
            LEVEL_PULSE_TIMEOUT_S,
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
            app.status.set(
                f"Auto-level active at shared test throttle {test_throttle_us}us "
                f"({LEVEL_TIMEOUT_DEFAULT_S:.3g}s timeout). Press Level again to stop."
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
            if not app.fc_service.is_connected:
                raise RuntimeError("Connect FC before using Level.")
            if app.fc_service.latest_attitude() is None:
                raise RuntimeError("No FC attitude sample yet. Wait a moment, then press Level again.")
            app.level_active = True
            app.level_timeout_deadline_s = time.monotonic() + LEVEL_TIMEOUT_DEFAULT_S
            self.update_link_indicators()
            self._prepare_shared_test_throttle_and_start()
        except Exception as exc:
            self.stop(update_status=False)
            self.set_error("Level error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

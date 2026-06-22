"""Deterministic Fly/Log paired-pulse workflow.

This gives PID-plan Fly/Log a simple, repeatable path that is closer to the
old exact-pulse exciter, but avoids the oversized one-way pulse that pushed the
stand into the hard stop. The sequence uses measured doses in both directions:

    marker on -> six sample groups -> marker off

Each sample group is:

    pitch +, pitch -, pitch -, pitch + -> center check/slow adjust ->
    roll +, roll -, roll -, roll + -> center check/slow adjust

Attitude feedback is used only as a safety/quality check and for slow centering
between sample sections. It is not used to stop the main test pulses. The
adaptive/random engine remains available elsewhere, but PID-plan Fly/Log can use
this as a safer default while debugging.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

from ..adaptive_session import AdaptiveSessionState
from ..constants import (
    BEEPER_MARKER_CHANNEL_INDEX,
    BEEPER_MARKER_OFF_US,
    BEEPER_MARKER_ON_US,
    BEEPER_MARKER_SPINUP_DELAY_MS,
    LEVEL_CENTER_US,
    PITCH_CHANNEL_INDEX,
    ROLL_CHANNEL_INDEX,
    PULSE_STATUS_REJECTED,
    THROTTLE_CHANNEL_INDEX,
)
from ..tasks.worker_tasks import hold_channel_until_stop as worker_hold_channel_until_stop


@dataclass(frozen=True)
class _FlyLogPulseStep:
    axis_name: str
    channel_index: int
    signed_force_us: int
    hold_s: float


class DeterministicFlyLogWorkflow:
    """Runs exact pitch/roll pulse pairs bracketed by the CH8 marker."""

    # Shared/generated plan throttle is preferred. This is only a fallback.
    DEFAULT_TEST_THROTTLE_US = 1250

    # Current useful movement settings from test-stand logs.
    PITCH_FORCE_US = 125
    PITCH_HOLD_S = 0.35
    ROLL_FORCE_US = 125
    ROLL_HOLD_S = 0.35

    # Run the full Pitch/Roll sample group multiple times so one Blackbox file
    # contains enough clean sections for PID analysis.
    SAMPLE_GROUP_COUNT = 6

    # Wait after each pulse before the next command. Important: the Arduino pulse
    # command returns after it is accepted, not after the timed hold completes, so
    # code below waits for hold time + this neutral period before continuing.
    NEUTRAL_WAIT_AFTER_PULSE_MS = 300
    GROUP_SETTLE_WAIT_MS = 1000
    MARKER_SETTLE_MS = 150
    EMERGENCY_ATTITUDE_DEG = 45.0

    # Center checks between pitch/roll and between groups. Below the warning
    # value, the test continues. Above the abort value, it stops before the next
    # axis/group. Between those values, it performs small fixed center nudges and
    # then continues.
    CENTER_CHECK_WARNING_DEG = 8.0
    CENTER_CHECK_ABORT_DEG = 25.0
    CENTER_ADJUST_FORCE_US = 55
    CENTER_ADJUST_HOLD_S = 0.06
    CENTER_ADJUST_SETTLE_MS = 245
    CENTER_ADJUST_MAX_PULSES = 12

    def __init__(
        self,
        *,
        app,
        auto_is_running: Callable[[], bool],
        auto_abort: Callable[..., None],
        set_auto_state: Callable[[AdaptiveSessionState, str], None],
        queue_live_channel_update: Callable[..., None],
        parse_offset_values_with_defaults: Callable[[], list[int]],
        set_live_channel_outputs: Callable[[list[int]], None],
        get_test_throttle_us: Callable[[], int],
        begin_fly_log_marker_off_and_complete: Callable[[], None],
        refresh_fly_log_button_state: Callable[[], None],
        open_pid_progress_window: Callable[[], None],
        update_pid_progress_window: Callable[[], None],
        set_auto_report_text: Callable[[str], None],
    ) -> None:
        self.app = app
        self.auto_is_running = auto_is_running
        self.auto_abort = auto_abort
        self.set_auto_state = set_auto_state
        self.queue_live_channel_update = queue_live_channel_update
        self.parse_offset_values_with_defaults = parse_offset_values_with_defaults
        self.set_live_channel_outputs = set_live_channel_outputs
        self.get_test_throttle_us = get_test_throttle_us
        self.begin_fly_log_marker_off_and_complete = begin_fly_log_marker_off_and_complete
        self.refresh_fly_log_button_state = refresh_fly_log_button_state
        self.open_pid_progress_window = open_pid_progress_window
        self.update_pid_progress_window = update_pid_progress_window
        self.set_auto_report_text = set_auto_report_text

    def _trace(self, message: str) -> None:
        app = self.app
        line = f"[FlyLog deterministic] {message}"
        app.status.set(line)
        try:
            app.auto_report_text.config(state="normal")
            app.auto_report_text.insert("end", line + "\n")
            app.auto_report_text.see("end")
            app.auto_report_text.config(state="disabled")
        except Exception:
            pass

    def _schedule(self, delay_ms: int, callback: Callable[[], None]) -> None:
        app = self.app
        if app.fly_log_marker_after_id is not None:
            try:
                app.root.after_cancel(app.fly_log_marker_after_id)
            except Exception:
                pass
        app.fly_log_marker_after_id = app.root.after(max(1, delay_ms), callback)

    def _sample_is_emergency(self) -> tuple[bool, str]:
        # Relative attitude is used for normal center quality. Absolute attitude
        # is preferred for the hard safety limit because the relative reference can
        # hide a tilted startup position.
        sample = None
        try:
            sample = self.app.fc_service.latest_absolute_attitude()
        except Exception:
            sample = None
        if sample is None:
            sample = self.app.fc_service.latest_attitude()
        if sample is None:
            return False, ""
        roll = float(sample.roll_deg)
        pitch = float(sample.pitch_deg)
        if abs(roll) >= self.EMERGENCY_ATTITUDE_DEG or abs(pitch) >= self.EMERGENCY_ATTITUDE_DEG:
            return True, f"Emergency attitude limit reached (roll={roll:+.1f}, pitch={pitch:+.1f})."
        return False, ""

    def _test_throttle_us(self) -> int:
        try:
            return max(1000, min(2000, int(self.get_test_throttle_us())))
        except Exception:
            return self.DEFAULT_TEST_THROTTLE_US

    def _channels_with_marker_state(self, channels: list[int], *, active: bool) -> list[int]:
        """Return a channel list that explicitly carries the CH8 marker value.

        The hardware controller also has a marker flag, but keeping the marker
        value in the actual channel list makes the app display and the Arduino
        frame agree. This avoids a hidden marker state where CH8 can appear low
        in the live outputs even though the marker flag is true.
        """
        output = channels.copy()
        while len(output) <= BEEPER_MARKER_CHANNEL_INDEX:
            output.append(BEEPER_MARKER_OFF_US)
        output[BEEPER_MARKER_CHANNEL_INDEX] = BEEPER_MARKER_ON_US if active else BEEPER_MARKER_OFF_US
        return output

    def _set_base_marker_state(self, *, active: bool) -> None:
        app = self.app
        app.base_channel_outputs = self._channels_with_marker_state(app.base_channel_outputs, active=active)
        self.set_live_channel_outputs(app.base_channel_outputs.copy())

    def _pitch_steps(self) -> list[_FlyLogPulseStep]:
        return [
            _FlyLogPulseStep("pitch", PITCH_CHANNEL_INDEX, self.PITCH_FORCE_US, self.PITCH_HOLD_S),
            _FlyLogPulseStep("pitch", PITCH_CHANNEL_INDEX, -self.PITCH_FORCE_US, self.PITCH_HOLD_S),
            _FlyLogPulseStep("pitch", PITCH_CHANNEL_INDEX, -self.PITCH_FORCE_US, self.PITCH_HOLD_S),
            _FlyLogPulseStep("pitch", PITCH_CHANNEL_INDEX, self.PITCH_FORCE_US, self.PITCH_HOLD_S),
        ]

    def _roll_steps(self) -> list[_FlyLogPulseStep]:
        return [
            _FlyLogPulseStep("roll", ROLL_CHANNEL_INDEX, self.ROLL_FORCE_US, self.ROLL_HOLD_S),
            _FlyLogPulseStep("roll", ROLL_CHANNEL_INDEX, -self.ROLL_FORCE_US, self.ROLL_HOLD_S),
            _FlyLogPulseStep("roll", ROLL_CHANNEL_INDEX, -self.ROLL_FORCE_US, self.ROLL_HOLD_S),
            _FlyLogPulseStep("roll", ROLL_CHANNEL_INDEX, self.ROLL_FORCE_US, self.ROLL_HOLD_S),
        ]

    def start(self) -> None:
        app = self.app
        app.auto_controller = None
        app.auto_original_base_outputs = app.base_channel_outputs.copy()
        test_throttle_us = self._test_throttle_us()
        if len(app.base_channel_outputs) > THROTTLE_CHANNEL_INDEX:
            app.base_channel_outputs[THROTTLE_CHANNEL_INDEX] = test_throttle_us
        app.auto_start_throttle_us = (
            app.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
            if len(app.base_channel_outputs) > THROTTLE_CHANNEL_INDEX
            else test_throttle_us
        )
        app.auto_current_throttle_us = app.auto_start_throttle_us
        app.auto_peak_throttle_us = app.auto_start_throttle_us
        app.auto_stop_reason = ""
        app.auto_warning = ""
        app.auto_session_start_s = time.monotonic()
        app.auto_last_tick_s = app.auto_session_start_s
        app.auto_last_sample_s = app.auto_session_start_s
        app.pid_plan_fly_log_active = True
        app.fly_log_finishing = False
        app.auto_stop_after_recovery = False
        app.auto_probe_axes_pending = []
        app.auto_pulse_inflight = False
        app.auto_hold_end_requested = False
        self.set_auto_state(AdaptiveSessionState.adaptive_run, "Deterministic Fly/Log 6-group pulse active")
        app.beeper_marker_active = False
        self._set_base_marker_state(active=False)
        app.auto_session_button.config(text="Fly/Log Active", state="disabled")
        app.cancel_auto_session_button.config(state="normal")
        self.refresh_fly_log_button_state()

        spinup_delay_s = BEEPER_MARKER_SPINUP_DELAY_MS / 1000.0
        self.set_auto_report_text(
            "Deterministic Fly/Log active\n\n"
            f"Candidate: {app.pid_plan_current_candidate_title or 'current PID plan step'}\n"
            f"Mode: {self.SAMPLE_GROUP_COUNT} exact bidirectional pitch/roll sample groups, "
            f"shared test throttle {test_throttle_us}us, pitch and roll +/-125us for 0.35s each\n"
            f"Neutral wait after every pulse: {self.NEUTRAL_WAIT_AFTER_PULSE_MS}ms\n"
            f"Slow center adjust when 8-25 deg off: +/-{self.CENTER_ADJUST_FORCE_US}us for "
            f"{self.CENTER_ADJUST_HOLD_S:.3g}s, up to {self.CENTER_ADJUST_MAX_PULSES} nudges\n"
            f"Settle wait between sample groups: {self.GROUP_SETTLE_WAIT_MS}ms\n"
            f"Spin-up before CH8 marker: {spinup_delay_s:.1f}s\n"
            "Sequence per group: pitch + - - + -> center adjust/check -> "
            "roll + - - + -> center adjust/check.\n"
            "Attitude is checked only as safety/center quality, not used to stop the main pulses.\n"
            "The adaptive/random movement engine is bypassed for this Fly/Log run.\n\n"
        )
        self.open_pid_progress_window()
        self.update_pid_progress_window()
        self._trace(
            f"Preparing base outputs before CH8 marker. Shared test throttle={test_throttle_us}us; "
            f"actual throttle={app.auto_start_throttle_us}us."
        )

        def on_spinup_outputs_prepared(ok: bool, res: object) -> None:
            if not app.pid_plan_fly_log_active:
                return
            if not ok:
                self.auto_abort(
                    "Unable to prepare deterministic Fly/Log outputs before marker.",
                    warning=str(res) if not isinstance(res, Exception) else str(res),
                )
                return
            self._trace(
                f"Base outputs prepared at throttle {app.base_channel_outputs[THROTTLE_CHANNEL_INDEX]}us. "
                f"Marker starts in {spinup_delay_s:.1f}s."
            )
            self._schedule(max(1, BEEPER_MARKER_SPINUP_DELAY_MS), self._enable_marker)

        self.queue_live_channel_update(
            app.base_channel_outputs.copy(),
            self.parse_offset_values_with_defaults(),
            after_update=on_spinup_outputs_prepared,
        )

    def _enable_marker(self) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        app.beeper_marker_active = True
        self._set_base_marker_state(active=True)
        self._trace("Enabling CH8 beeper marker on channel 8 (2000us).")

        def on_marker_enabled(ok: bool, res: object) -> None:
            if not app.pid_plan_fly_log_active:
                return
            if not ok:
                self.auto_abort(
                    "Unable to enable deterministic Fly/Log CH8 marker.",
                    warning=str(res) if not isinstance(res, Exception) else str(res),
                )
                return
            now_s = time.monotonic()
            app.auto_session_start_s = now_s
            app.auto_last_tick_s = now_s
            app.auto_last_sample_s = now_s
            self._trace("CH8 marker is ON. Scheduling 6 pitch/roll sample groups.")
            self._schedule(self.MARKER_SETTLE_MS, lambda: self._run_sample_group(0))

        self.queue_live_channel_update(
            app.base_channel_outputs.copy(),
            self.parse_offset_values_with_defaults(),
            after_update=on_marker_enabled,
        )

    def _run_sample_group(self, group_index: int) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        if group_index >= self.SAMPLE_GROUP_COUNT:
            self._finish_sequence()
            return
        self._trace(f"Starting sample group {group_index + 1}/{self.SAMPLE_GROUP_COUNT}: Pitch + - - +.")
        self._run_step_list(
            self._pitch_steps(),
            0,
            next_callback=lambda: self._center_or_adjust(
                context=f"group {group_index + 1} before roll",
                next_callback=lambda: self._run_roll_section(group_index),
            ),
        )

    def _run_roll_section(self, group_index: int) -> None:
        self._trace(f"Sample group {group_index + 1}/{self.SAMPLE_GROUP_COUNT}: Roll + - - +.")
        self._run_step_list(
            self._roll_steps(),
            0,
            next_callback=lambda: self._center_or_adjust(
                context=f"group {group_index + 1} after roll",
                next_callback=lambda: self._finish_group(group_index),
            ),
        )

    def _finish_group(self, group_index: int) -> None:
        if group_index + 1 >= self.SAMPLE_GROUP_COUNT:
            self._finish_sequence()
            return
        self._trace(
            f"Sample group {group_index + 1}/{self.SAMPLE_GROUP_COUNT} complete; "
            f"settling {self.GROUP_SETTLE_WAIT_MS}ms before next group."
        )
        self._schedule(self.GROUP_SETTLE_WAIT_MS, lambda: self._run_sample_group(group_index + 1))

    def _run_step_list(
        self,
        steps: list[_FlyLogPulseStep],
        step_index: int,
        *,
        next_callback: Callable[[], None],
    ) -> None:
        if step_index >= len(steps):
            next_callback()
            return
        step = steps[step_index]
        direction_label = "+" if step.signed_force_us >= 0 else "-"
        self._issue_axis_pulse(
            step=step,
            label=f"{direction_label}{step.axis_name}",
            next_callback=lambda: self._run_step_list(steps, step_index + 1, next_callback=next_callback),
        )

    def _center_or_adjust(self, *, context: str, next_callback: Callable[[], None]) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        emergency, reason = self._sample_is_emergency()
        if emergency:
            self.auto_abort(reason, continue_pipeline=False)
            return

        sample = app.fc_service.latest_attitude()
        if sample is None:
            self._trace(f"Center check {context}: no attitude sample available; continuing.")
            next_callback()
            return

        roll = float(sample.roll_deg)
        pitch = float(sample.pitch_deg)
        max_abs = max(abs(roll), abs(pitch))
        if max_abs >= self.CENTER_CHECK_ABORT_DEG:
            self.auto_abort(
                f"Center check {context} failed; drone is too far from center "
                f"(roll={roll:+.1f}, pitch={pitch:+.1f}).",
                continue_pipeline=False,
            )
            return
        if max_abs < self.CENTER_CHECK_WARNING_DEG:
            self._trace(f"Center check {context} OK (roll={roll:+.1f}, pitch={pitch:+.1f}).")
            next_callback()
            return

        self._trace(
            f"Center check {context}: roll={roll:+.1f}, pitch={pitch:+.1f}; "
            "running slow center adjustment before continuing."
        )
        self._run_center_adjustment(context=context, pulse_index=1, next_callback=next_callback)

    def _run_center_adjustment(
        self,
        *,
        context: str,
        pulse_index: int,
        next_callback: Callable[[], None],
    ) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        emergency, reason = self._sample_is_emergency()
        if emergency:
            self.auto_abort(reason, continue_pipeline=False)
            return

        sample = app.fc_service.latest_attitude()
        if sample is None:
            self._trace(f"Center adjust {context}: no attitude sample available; continuing.")
            next_callback()
            return
        roll = float(sample.roll_deg)
        pitch = float(sample.pitch_deg)
        max_abs = max(abs(roll), abs(pitch))
        if max_abs >= self.CENTER_CHECK_ABORT_DEG:
            self.auto_abort(
                f"Center adjust {context} stopped; drone is too far from center "
                f"(roll={roll:+.1f}, pitch={pitch:+.1f}).",
                continue_pipeline=False,
            )
            return
        if max_abs < self.CENTER_CHECK_WARNING_DEG:
            self._trace(
                f"Center adjust {context} complete (roll={roll:+.1f}, pitch={pitch:+.1f}); continuing test."
            )
            next_callback()
            return
        if pulse_index > self.CENTER_ADJUST_MAX_PULSES:
            self._trace(
                f"Center adjust {context} used {self.CENTER_ADJUST_MAX_PULSES} nudges and is still off-center "
                f"(roll={roll:+.1f}, pitch={pitch:+.1f}); continuing because it is under "
                f"{self.CENTER_CHECK_ABORT_DEG:.0f} deg."
            )
            next_callback()
            return

        if abs(roll) >= abs(pitch):
            channel_index = ROLL_CHANNEL_INDEX
            axis_name = "roll"
            angle = roll
        else:
            channel_index = PITCH_CHANNEL_INDEX
            axis_name = "pitch"
            angle = pitch

        center_us = LEVEL_CENTER_US
        if len(app.base_channel_outputs) > channel_index:
            try:
                center_us = int(app.base_channel_outputs[channel_index])
            except Exception:
                center_us = LEVEL_CENTER_US
        signed_force_us = -self.CENTER_ADJUST_FORCE_US if angle > 0 else self.CENTER_ADJUST_FORCE_US
        target = max(1000, min(2000, center_us + signed_force_us))

        try:
            offsets = self.parse_offset_values_with_defaults()
        except Exception as exc:
            self.auto_abort("Could not parse offsets for deterministic Fly/Log center adjust.", warning=str(exc))
            return
        if len(offsets) <= channel_index:
            self.auto_abort("Offset list is too short for deterministic Fly/Log center adjust.")
            return

        active_outputs = app.base_channel_outputs.copy()
        active_outputs[channel_index] = target
        self.set_live_channel_outputs(active_outputs)
        app.auto_pulse_inflight = True
        direction_text = "+" if signed_force_us >= 0 else ""
        hold_ms = max(1, round(self.CENTER_ADJUST_HOLD_S * 1000))
        total_wait_ms = hold_ms + max(0, int(self.CENTER_ADJUST_SETTLE_MS))
        self._trace(
            f"Center adjust {context} nudge {pulse_index}/{self.CENTER_ADJUST_MAX_PULSES}: "
            f"{axis_name} CH{channel_index + 1} {center_us}->{target} "
            f"({direction_text}{signed_force_us}us) for {self.CENTER_ADJUST_HOLD_S:.3g}s; "
            f"roll={roll:+.1f}, pitch={pitch:+.1f}."
        )

        def on_center_pulse_done(ok: bool, res: object) -> None:
            if not app.pid_plan_fly_log_active:
                return
            if not ok:
                self.auto_abort(
                    "Deterministic Fly/Log center-adjust command failed.",
                    warning=str(res) if not isinstance(res, Exception) else str(res),
                )
                return
            if not isinstance(res, int):
                self.auto_abort("Unexpected deterministic Fly/Log center-adjust result from worker.")
                return
            if res == PULSE_STATUS_REJECTED:
                self.auto_abort("Firmware rejected deterministic Fly/Log center-adjust command.")
                return
            self._schedule(
                total_wait_ms,
                lambda: self._complete_center_adjust_delay(
                    context=context,
                    pulse_index=pulse_index,
                    next_callback=next_callback,
                ),
            )

        app.worker.submit(
            worker_hold_channel_until_stop,
            channel_index,
            target,
            offsets[channel_index],
            self.CENTER_ADJUST_HOLD_S,
            callback=on_center_pulse_done,
        )

    def _complete_center_adjust_delay(
        self,
        *,
        context: str,
        pulse_index: int,
        next_callback: Callable[[], None],
    ) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        app.auto_pulse_inflight = False
        self.set_live_channel_outputs(app.base_channel_outputs.copy())
        self._run_center_adjustment(
            context=context,
            pulse_index=pulse_index + 1,
            next_callback=next_callback,
        )

    def _issue_axis_pulse(
        self,
        *,
        step: _FlyLogPulseStep,
        label: str,
        next_callback: Callable[[], None],
    ) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        emergency, reason = self._sample_is_emergency()
        if emergency:
            self.auto_abort(reason, continue_pipeline=False)
            return

        channel_index = step.channel_index
        if len(app.base_channel_outputs) <= channel_index:
            self.auto_abort("Base channel output list is too short for deterministic Fly/Log pulse.")
            return
        target = max(1000, min(2000, int(app.base_channel_outputs[channel_index]) + int(step.signed_force_us)))
        try:
            offsets = self.parse_offset_values_with_defaults()
        except Exception as exc:
            self.auto_abort("Could not parse offsets for deterministic Fly/Log pulse.", warning=str(exc))
            return
        if len(offsets) <= channel_index:
            self.auto_abort("Offset list is too short for deterministic Fly/Log pulse.")
            return

        active_outputs = app.base_channel_outputs.copy()
        active_outputs[channel_index] = target
        self.set_live_channel_outputs(active_outputs)
        app.auto_pulse_inflight = True
        direction_text = "+" if step.signed_force_us >= 0 else ""
        hold_ms = max(1, round(step.hold_s * 1000))
        wait_ms = max(0, int(self.NEUTRAL_WAIT_AFTER_PULSE_MS))
        next_total_delay_ms = hold_ms + wait_ms
        self._trace(
            f"Issuing {label} pulse: CH{channel_index + 1} "
            f"{app.base_channel_outputs[channel_index]} -> {target} "
            f"({direction_text}{step.signed_force_us}us) for {step.hold_s:.3g}s; "
            f"then {wait_ms}ms neutral wait."
        )

        def on_pulse_done(ok: bool, res: object) -> None:
            if not app.pid_plan_fly_log_active:
                return
            if not ok:
                self.auto_abort(
                    "Deterministic Fly/Log pulse command failed.",
                    warning=str(res) if not isinstance(res, Exception) else str(res),
                )
                return
            if not isinstance(res, int):
                self.auto_abort("Unexpected deterministic Fly/Log pulse result from worker.")
                return
            if res == PULSE_STATUS_REJECTED:
                self.auto_abort("Firmware rejected deterministic Fly/Log pulse command.")
                return
            self._trace(
                f"{label.title()} pulse command accepted. Firmware hold is {hold_ms}ms; "
                f"next step in {next_total_delay_ms}ms."
            )
            self._schedule(
                next_total_delay_ms,
                lambda: self._complete_pulse_delay(label=label, next_callback=next_callback),
            )

        app.worker.submit(
            worker_hold_channel_until_stop,
            channel_index,
            target,
            offsets[channel_index],
            step.hold_s,
            callback=on_pulse_done,
        )

    def _complete_pulse_delay(self, *, label: str, next_callback: Callable[[], None]) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        app.auto_pulse_inflight = False
        self.set_live_channel_outputs(app.base_channel_outputs.copy())
        emergency, reason = self._sample_is_emergency()
        if emergency:
            self.auto_abort(reason, continue_pipeline=False)
            return
        self._trace(f"{label.title()} pulse/neutral wait complete; continuing sequence.")
        next_callback()

    def _finish_sequence(self) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active:
            return
        app.auto_pulse_inflight = False
        self.set_live_channel_outputs(app.base_channel_outputs.copy())
        emergency, reason = self._sample_is_emergency()
        if emergency:
            self.auto_abort(reason, continue_pipeline=False)
            return
        self._trace("All deterministic Fly/Log sample groups complete; marker will turn off.")
        self.begin_fly_log_marker_off_and_complete()

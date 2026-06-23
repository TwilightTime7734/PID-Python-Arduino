"""Deterministic Fly/Log one-axis paired-pulse workflow.

The Fly/Log path keeps the test narrow and repeatable:

    CH8 marker on -> 10 pulses positive -> 10 pulses negative -> CH8 marker off

Only the selected axis is driven. Attitude is watched for safety stops, but no
other axis is commanded during the pulse sequence.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

from ..adaptive_session import AdaptiveSessionState
from ..ch8_marker import channels_with_pid_test_ch8
from ..constants import (
    PITCH_CHANNEL_INDEX,
    PID_TEST_CH8_SPINUP_DELAY_MS,
    ROLL_CHANNEL_INDEX,
    PULSE_STATUS_REJECTED,
    THROTTLE_CHANNEL_INDEX,
    ARDUINO_FIXED_PULSE_HOLD_S,
    ARDUINO_FIXED_PULSE_HOLD_MS,
)
from ..pid_tuning_workflow import PStartInputs, TestPulseProfile, estimate_test_pulse_profile
from ..tasks.worker_tasks import pulse_channel_force as worker_pulse_channel_force


@dataclass(frozen=True)
class _FlyLogPulseStep:
    axis_name: str
    channel_index: int
    signed_force_us: int
    hold_s: float


class DeterministicFlyLogWorkflow:
    """Runs exact one-axis pulse pairs bracketed by the CH8 marker."""

    # Shared/generated plan throttle is preferred. This is only a fallback.
    DEFAULT_TEST_THROTTLE_US = 1250

    # Main pulse force/hold comes from the generated PID plan's aircraft profile.

    # Ten + pulses and ten - pulses for the selected axis.
    PULSE_PAIR_COUNT = 10

    # Wait after each pulse before the next command. Important: the Arduino pulse
    # command returns after it is accepted, not after the timed hold completes, so
    # code below waits for hold time + this neutral period before continuing.
    # Neutral wait after each pulse also comes from the generated PID plan.
    GROUP_SETTLE_WAIT_MS = 1000
    MARKER_SETTLE_MS = 150
    EMERGENCY_ATTITUDE_DEG = 45.0

    # Safety stop threshold for attitude excursions during the pulse sequence.
    CENTER_CHECK_ABORT_DEG = 25.0

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
        get_test_pulse_profile: Callable[[], TestPulseProfile],
        begin_fly_log_marker_off_and_complete: Callable[[], None],
        refresh_fly_log_button_state: Callable[[], None],
        open_pid_progress_window: Callable[[], None],
        update_pid_progress_window: Callable[[], None],
        publish_auto_report: Callable[[str], None],
    ) -> None:
        self.app = app
        self.auto_is_running = auto_is_running
        self.auto_abort = auto_abort
        self.set_auto_state = set_auto_state
        self.queue_live_channel_update = queue_live_channel_update
        self.parse_offset_values_with_defaults = parse_offset_values_with_defaults
        self.set_live_channel_outputs = set_live_channel_outputs
        self.get_test_throttle_us = get_test_throttle_us
        self.get_test_pulse_profile = get_test_pulse_profile
        self.begin_fly_log_marker_off_and_complete = begin_fly_log_marker_off_and_complete
        self.refresh_fly_log_button_state = refresh_fly_log_button_state
        self.open_pid_progress_window = open_pid_progress_window
        self.update_pid_progress_window = update_pid_progress_window
        self.publish_auto_report = publish_auto_report
        self._active_test_pulse_profile: TestPulseProfile | None = None

    def _trace(self, message: str) -> None:
        app = self.app
        app.status.set(f"[FlyLog deterministic] {message}")

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
            sample = self.app.attitude_service.latest_absolute_attitude()
        except Exception:
            sample = None
        if sample is None:
            sample = self.app.attitude_service.latest_attitude()
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

    def _test_pulse_profile(self) -> TestPulseProfile:
        profile = self._active_test_pulse_profile
        if profile is not None:
            return profile
        try:
            return self.get_test_pulse_profile()
        except Exception:
            return estimate_test_pulse_profile(PStartInputs())

    def _neutral_wait_after_pulse_ms(self) -> int:
        return max(0, int(self._test_pulse_profile().neutral_wait_ms))

    def _fly_log_axis(self) -> str:
        axis = str(self._test_pulse_profile().test_axis).strip().lower()
        return axis if axis in {"roll", "pitch"} else "roll"

    @staticmethod
    def _axis_channel_index(axis: str) -> int:
        return ROLL_CHANNEL_INDEX if axis == "roll" else PITCH_CHANNEL_INDEX

    def _set_base_marker_state(self, *, active: bool) -> None:
        app = self.app
        app.base_channel_outputs = channels_with_pid_test_ch8(app.base_channel_outputs, active=active)
        self.set_live_channel_outputs(app.base_channel_outputs.copy())

    def _axis_pair_steps(self) -> list[_FlyLogPulseStep]:
        profile = self._test_pulse_profile()
        axis = self._fly_log_axis()
        channel_index = self._axis_channel_index(axis)
        force_us = int(profile.main_force_us)
        hold_s = ARDUINO_FIXED_PULSE_HOLD_S
        return [
            _FlyLogPulseStep(axis, channel_index, force_us, hold_s),
            _FlyLogPulseStep(axis, channel_index, -force_us, hold_s),
        ]

    def start(self) -> None:
        app = self.app
        self._active_test_pulse_profile = self.get_test_pulse_profile()
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
        axis = self._fly_log_axis()
        self.set_auto_state(AdaptiveSessionState.adaptive_run, "Deterministic Fly/Log one-axis pulse active")
        self._set_base_marker_state(active=False)
        self.refresh_fly_log_button_state()

        spinup_delay_s = PID_TEST_CH8_SPINUP_DELAY_MS / 1000.0
        pulse_profile = self._test_pulse_profile()
        self.publish_auto_report(
            "Deterministic Fly/Log active\n\n"
            f"Candidate: {app.pid_plan_current_candidate_title or 'current PID plan step'}\n"
            f"Mode: one-axis {axis} test, {self.PULSE_PAIR_COUNT} positive and "
            f"{self.PULSE_PAIR_COUNT} negative main pulses, "
            f"shared test throttle {test_throttle_us}us, {pulse_profile.aircraft_name} pulse profile, "
            f"{axis} +/-{pulse_profile.main_force_us}us for board-fixed {ARDUINO_FIXED_PULSE_HOLD_S:.2f}s each\n"
            f"Start probe reference: {axis} +/-{pulse_profile.probe_force_us}us "
            f"for board-fixed {ARDUINO_FIXED_PULSE_HOLD_S:.2f}s\n"
            f"Neutral wait after every pulse: {self._neutral_wait_after_pulse_ms()}ms\n"
            f"Settle wait between pulse pairs: {self.GROUP_SETTLE_WAIT_MS}ms\n"
            f"Spin-up before CH8 marker: {spinup_delay_s:.1f}s\n"
            f"Sequence per pair: {axis} + -> {axis} -.\n"
            "Attitude is checked only as a safety stop, not used to command the other axis.\n\n"
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
                f"Start probes begin in {spinup_delay_s:.1f}s."
            )
            self._schedule(max(1, PID_TEST_CH8_SPINUP_DELAY_MS), self._run_start_probe_sequence)

        self.queue_live_channel_update(
            app.base_channel_outputs.copy(),
            self.parse_offset_values_with_defaults(),
            after_update=on_spinup_outputs_prepared,
        )

    def _start_probe_steps(self) -> list[_FlyLogPulseStep]:
        profile = self._test_pulse_profile()
        axis = self._fly_log_axis()
        channel_index = self._axis_channel_index(axis)
        force_us = max(0, int(profile.probe_force_us))
        hold_s = ARDUINO_FIXED_PULSE_HOLD_S
        if force_us <= 0:
            return []
        return [
            _FlyLogPulseStep(axis, channel_index, force_us, hold_s),
            _FlyLogPulseStep(axis, channel_index, -force_us, hold_s),
        ]

    def _run_start_probe_sequence(self) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        steps = self._start_probe_steps()
        if not steps:
            self._enable_marker()
            return
        self._trace("Running small pre-marker start probes before CH8 marker.")
        self._run_step_list(steps, 0, next_callback=self._enable_marker)

    def _enable_marker(self) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        self._set_base_marker_state(active=True)
        self._trace("Setting CH8 to 2000us for the PID test.")

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
            self._trace(
                f"CH8 marker is ON. Scheduling {self.PULSE_PAIR_COUNT} {self._fly_log_axis()} pulse pairs."
            )
            self._schedule(self.MARKER_SETTLE_MS, lambda: self._run_pulse_pair(0))

        self.queue_live_channel_update(
            app.base_channel_outputs.copy(),
            self.parse_offset_values_with_defaults(),
            after_update=on_marker_enabled,
        )

    def _run_pulse_pair(self, pair_index: int) -> None:
        app = self.app
        app.fly_log_marker_after_id = None
        if not app.pid_plan_fly_log_active or not self.auto_is_running():
            return
        if pair_index >= self.PULSE_PAIR_COUNT:
            self._finish_sequence()
            return
        axis = self._fly_log_axis()
        self._trace(f"Starting {axis} pulse pair {pair_index + 1}/{self.PULSE_PAIR_COUNT}: + then -.")
        self._run_step_list(
            self._axis_pair_steps(),
            0,
            next_callback=lambda: self._finish_pair(pair_index),
        )

    def _finish_pair(self, pair_index: int) -> None:
        if pair_index + 1 >= self.PULSE_PAIR_COUNT:
            self._finish_sequence()
            return
        self._trace(
            f"Pulse pair {pair_index + 1}/{self.PULSE_PAIR_COUNT} complete; "
            f"settling {self.GROUP_SETTLE_WAIT_MS}ms before next pair."
        )
        self._schedule(self.GROUP_SETTLE_WAIT_MS, lambda: self._run_pulse_pair(pair_index + 1))

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
        active_outputs = app.base_channel_outputs.copy()
        active_outputs[channel_index] = target
        self.set_live_channel_outputs(active_outputs)
        app.auto_pulse_inflight = True
        direction_text = "+" if step.signed_force_us >= 0 else ""
        hold_ms = ARDUINO_FIXED_PULSE_HOLD_MS
        wait_ms = self._neutral_wait_after_pulse_ms()
        next_total_delay_ms = hold_ms + wait_ms
        self._trace(
            f"Issuing {label} pulse: CH{channel_index + 1} "
            f"{app.base_channel_outputs[channel_index]} -> {target} "
            f"({direction_text}{step.signed_force_us}us) for board-fixed {ARDUINO_FIXED_PULSE_HOLD_S:.2f}s; "
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
            worker_pulse_channel_force,
            channel_index,
            step.signed_force_us,
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
        self._trace("All deterministic Fly/Log one-axis pulse pairs complete; marker will turn off.")
        self._active_test_pulse_profile = None
        self.begin_fly_log_marker_off_and_complete()

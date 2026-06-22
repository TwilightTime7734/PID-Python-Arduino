"""Live auto-session tick/pulse engine."""

from __future__ import annotations

import time
from collections.abc import Callable

from ..adaptive_session import AdaptiveCommand, AdaptiveSessionState, ExcitationEvent, axis_channel_index
from ..constants import PULSE_STATUS_REJECTED
from ..tasks.worker_tasks import hold_channel_until_stop as worker_hold_channel_until_stop


class AutoSessionEngine:
    def __init__(
        self,
        *,
        app,
        auto_is_running: Callable[[], bool],
        auto_abort: Callable[..., None],
        set_auto_state: Callable[[AdaptiveSessionState, str], None],
        auto_elapsed_s: Callable[..., float],
        pulse_axis_value: Callable[[object, str], float],
        arduino_output_connected: Callable[[], bool],
        parse_offset_values_with_defaults: Callable[[], list[int]],
        set_live_channel_outputs: Callable[[list[int]], None],
        begin_auto_observe_window: Callable[[AdaptiveCommand], None],
        begin_fly_log_marker_off_and_complete: Callable[[], None],
        finish_pid_plan_fly_log: Callable[..., None],
        adjust_auto_throttle_after_event: Callable[[ExcitationEvent, bool], None],
    ) -> None:
        self.app = app
        self.auto_is_running = auto_is_running
        self.auto_abort = auto_abort
        self.set_auto_state = set_auto_state
        self.auto_elapsed_s = auto_elapsed_s
        self.pulse_axis_value = pulse_axis_value
        self.arduino_output_connected = arduino_output_connected
        self.parse_offset_values_with_defaults = parse_offset_values_with_defaults
        self.set_live_channel_outputs = set_live_channel_outputs
        self.begin_auto_observe_window = begin_auto_observe_window
        self.begin_fly_log_marker_off_and_complete = begin_fly_log_marker_off_and_complete
        self.finish_pid_plan_fly_log = finish_pid_plan_fly_log
        self.adjust_auto_throttle_after_event = adjust_auto_throttle_after_event

    def schedule_tick(self, delay_ms: int | None = None) -> None:
        app = self.app
        if not self.auto_is_running():
            return
        if app.auto_tick_after_id is not None:
            try:
                app.root.after_cancel(app.auto_tick_after_id)
            except Exception:
                pass
        cadence_ms = max(10, round(app.auto_config.control_interval_s * 1000.0))
        app.auto_tick_after_id = app.root.after(cadence_ms if delay_ms is None else max(1, delay_ms), self.run_tick)

    def finalize_event(self) -> None:
        app = self.app
        if app.auto_controller is None or app.auto_active_command is None:
            return
        sample = app.attitude_service.latest_attitude()
        if sample is None:
            return
        recovery_event = app.auto_active_command.recovery
        axis_value = self.pulse_axis_value(sample, app.auto_active_command.axis)
        final_error = axis_value - app.auto_event_baseline
        settle_success = abs(final_error) <= app.auto_config.settle_deadband_deg
        if app.auto_active_command.calibration:
            axis = app.auto_active_command.axis
            signed_peak = app.auto_event_signed_peak_delta
            abs_peak = app.auto_event_abs_peak_delta
            if abs_peak < app.auto_config.probe_min_response_deg:
                self.auto_abort(
                    f"{axis.title()} probe produced too little attitude response ({abs_peak:.2f} deg).",
                    warning="Fly/Log stopped before roll/pitch calibration could finish.",
                )
                return
            app.auto_axis_output_sign[axis] = 1 if signed_peak >= 0 else -1
            app.auto_controller.record_axis_response(
                axis,
                app.auto_active_command.force_us,
                app.auto_active_command.hold_s,
                abs_peak,
            )
            app.status.set(
                f"{axis.title()} output calibrated: +PWM gives {'+' if signed_peak >= 0 else '-'}{axis} response."
            )
            app.auto_active_command = None
            app.auto_hold_end_requested = False
            app.auto_event_peak_delta = 0.0
            app.auto_event_abs_peak_delta = 0.0
            app.auto_event_signed_peak_delta = 0.0
            app.auto_event_response_delay_s = None
            app.auto_event_baseline = 0.0
            app.auto_event_start_s = 0.0
            if app.pid_plan_fly_log_active and not app.auto_probe_axes_pending:
                self.begin_fly_log_marker_off_and_complete()
            return
        event = ExcitationEvent(
            axis=app.auto_active_command.axis,
            direction=app.auto_active_command.direction,
            force_us=app.auto_active_command.force_us,
            hold_s=app.auto_active_command.hold_s,
            settle_s=app.auto_active_command.settle_s,
            baseline_angle_deg=app.auto_event_baseline,
            peak_delta_deg=app.auto_event_peak_delta,
            settle_success=settle_success,
            response_delay_s=app.auto_event_response_delay_s,
            final_error_deg=final_error,
            recovery=recovery_event,
        )
        app.auto_controller.record_event(event)
        self.adjust_auto_throttle_after_event(event, recovery_event)
        app.auto_active_command = None
        app.auto_hold_end_requested = False
        app.auto_event_peak_delta = 0.0
        app.auto_event_abs_peak_delta = 0.0
        app.auto_event_signed_peak_delta = 0.0
        app.auto_event_response_delay_s = None
        app.auto_event_baseline = 0.0
        app.auto_event_start_s = 0.0

    def issue_command(self, command: AdaptiveCommand) -> None:
        app = self.app
        if not self.arduino_output_connected():
            raise RuntimeError("Arduino output is disconnected.")
        if app.auto_controller is None:
            raise RuntimeError("Adaptive controller is not initialized.")

        sample = app.attitude_service.latest_attitude()
        if sample is None:
            raise RuntimeError("No attitude-board sample available.")

        channel_index = axis_channel_index(command.axis)
        output_sign = 1 if command.calibration else app.auto_axis_output_sign.get(command.axis, 1)
        pwm_direction = command.direction * output_sign
        target = app.base_channel_outputs[channel_index] + (pwm_direction * command.force_us)
        target = max(1000, min(2000, target))
        offsets = self.parse_offset_values_with_defaults()

        active_outputs = app.base_channel_outputs.copy()
        active_outputs[channel_index] = target
        self.set_live_channel_outputs(active_outputs)
        app.auto_pulse_inflight = True
        app.auto_hold_end_requested = False
        app.auto_settle_until_s = None
        app.auto_active_command = command
        app.auto_event_peak_delta = 0.0
        app.auto_event_abs_peak_delta = 0.0
        app.auto_event_signed_peak_delta = 0.0
        app.auto_event_response_delay_s = None
        app.auto_event_baseline = self.pulse_axis_value(sample, command.axis)
        app.auto_event_start_s = time.monotonic()

        def on_auto_hold_elapsed() -> None:
            app.auto_hold_after_id = None
            if not self.auto_is_running() or app.auto_active_command is not command:
                return
            self.begin_auto_observe_window(command)

        def on_auto_hold_done(ok: bool, res: object) -> None:
            if not self.auto_is_running():
                return
            if not ok:
                app.auto_pulse_inflight = False
                self.auto_abort(
                    "Pulse command failed during auto session.",
                    warning=str(res) if not isinstance(res, Exception) else str(res),
                )
                return
            if not isinstance(res, int):
                app.auto_pulse_inflight = False
                self.auto_abort("Unexpected pulse result from worker.")
                return
            if res == PULSE_STATUS_REJECTED:
                app.auto_pulse_inflight = False
                self.auto_abort("Firmware rejected adaptive pulse command.")
                return
            if app.auto_hold_end_requested:
                return
            app.auto_hold_after_id = app.root.after(round(command.hold_s * 1000.0), on_auto_hold_elapsed)

        app.worker.submit(
            worker_hold_channel_until_stop,
            channel_index,
            target,
            offsets[channel_index],
            command.hold_s,
            callback=on_auto_hold_done,
        )

    def issue_next_probe(self) -> bool:
        app = self.app
        if app.auto_controller is None or not app.auto_probe_axes_pending:
            return False
        axis = app.auto_probe_axes_pending.pop(0)
        command = AdaptiveCommand(
            axis=axis,
            direction=1,
            force_us=max(1, int(app.auto_config.probe_force_us)),
            hold_s=max(0.01, float(app.auto_config.probe_hold_s)),
            settle_s=max(0.0, float(app.auto_config.probe_settle_s)),
            recovery=False,
            reason="output sign/gain probe",
            target_peak_deg=max(0.0, float(app.auto_config.probe_target_peak_deg)),
            calibration=True,
        )
        app.status.set(f"Calibrating {axis} output with a small Fly/Log probe.")
        self.issue_command(command)
        return True

    def run_tick(self) -> None:
        app = self.app
        app.auto_tick_after_id = None
        if not self.auto_is_running():
            return
        if app.auto_controller is None:
            self.auto_abort("Adaptive controller was not initialized.")
            return
        if not self.arduino_output_connected():
            self.auto_abort("Arduino output disconnected during auto session.")
            return
        if not app.attitude_service.is_connected:
            self.auto_abort("Attitude board disconnected during auto session.")
            return

        now = time.monotonic()
        if app.auto_last_sample_s is None or (now - app.auto_last_sample_s) > app.auto_config.telemetry_stale_s:
            self.auto_abort("FC telemetry became stale.", continue_pipeline=False)
            return

        sample = app.attitude_service.latest_attitude()
        if sample is None:
            self.schedule_tick()
            return

        abort, abort_reason = app.auto_controller.should_abort(sample.roll_deg, sample.pitch_deg)
        if abort:
            self.auto_abort(abort_reason, continue_pipeline=False)
            return

        calibration_in_progress = bool(app.auto_probe_axes_pending) or (
            app.auto_active_command is not None and app.auto_active_command.calibration
        )
        if calibration_in_progress and app.auto_controller.should_recover(sample.roll_deg, sample.pitch_deg):
            self.auto_abort(
                f"Attitude exceeded the Fly/Log calibration window (roll={sample.roll_deg:+.1f}, pitch={sample.pitch_deg:+.1f}).",
                continue_pipeline=False,
            )
            return

        if app.auto_controller.should_recover(sample.roll_deg, sample.pitch_deg):
            app.auto_recovery_mode = True
            self.set_auto_state(AdaptiveSessionState.recovery, "Recovery mode")
        elif app.auto_recovery_mode and app.auto_controller.recovery_complete(sample.roll_deg, sample.pitch_deg):
            app.auto_recovery_mode = False
            if app.auto_stop_after_recovery:
                app.auto_stop_after_recovery = False
                self.auto_abort("Auto session stopped after recovering from hard safety limit.", continue_pipeline=False)
                return
            self.set_auto_state(AdaptiveSessionState.adaptive_run, "Active")

        if app.auto_recovery_mode and not app.auto_pulse_inflight:
            app.auto_active_command = None
            app.auto_hold_end_requested = False
            app.auto_settle_until_s = None

        if app.auto_pulse_inflight:
            self.schedule_tick()
            return

        if app.auto_active_command is not None and app.auto_settle_until_s is not None and now < app.auto_settle_until_s:
            self.schedule_tick()
            return

        if app.auto_active_command is not None and app.auto_settle_until_s is not None and now >= app.auto_settle_until_s:
            self.finalize_event()
            app.auto_settle_until_s = None
            if not self.auto_is_running():
                return

        if not app.auto_recovery_mode and app.auto_probe_axes_pending:
            try:
                if self.issue_next_probe():
                    self.schedule_tick()
                    return
            except Exception as exc:
                self.auto_abort("Unable to issue Fly/Log calibration probe.", warning=str(exc))
                return

        if app.pid_plan_fly_log_active:
            ready, stop_reason, warning = app.auto_controller.stop_ready(self.auto_elapsed_s(now))
            if ready:
                self.finish_pid_plan_fly_log(stop_reason)
                return
            self.schedule_tick()
            return

        ready, stop_reason, warning = app.auto_controller.stop_ready(self.auto_elapsed_s(now))
        if ready:
            self.auto_abort(
                "Adaptive movement ended without an active Fly/Log session.",
                warning=stop_reason,
                continue_pipeline=False,
            )
            return

        command = app.auto_controller.next_command(sample.roll_deg, sample.pitch_deg, app.auto_recovery_mode)
        if command is None:
            self.schedule_tick()
            return

        try:
            self.issue_command(command)
        except Exception as exc:
            self.auto_abort("Unable to issue adaptive command.", warning=str(exc))
            return
        self.schedule_tick()

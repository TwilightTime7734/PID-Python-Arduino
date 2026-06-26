"""Application runtime orchestration."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import queue
import tkinter as tk
from tkinter import messagebox

from serialUSB.inav_serial_service import InavSerialService

from .constants import (
    ARDUINO_FIXED_PULSE_HOLD_S,
    ARDUINO_FIXED_PULSE_HOLD_MS,
    FC_PORT_DEFAULT,
    FLY_LOG_SAFETY_TIMEOUT_S,
    PITCH_CHANNEL_INDEX,
    PID_PLAN_FLY_LOG_RUNTIME_S,
    PORT_DEFAULT,
    PULSE_STATUS_REJECTED,
    ROLL_CHANNEL_INDEX,
    THROTTLE_CHANNEL_INDEX,
)
from .adaptive_session import (
    AdaptiveCommand,
    AdaptiveSessionConfig,
    AdaptiveSessionState,
    ExcitationEvent,
)
from .ch8_marker import channels_with_pid_test_ch8
from .blackbox_import import BlackboxImportResult
from .pid_tuning_workflow import (
    LoadedPIDTuningPlan,
    PStartInputs,
    TestPulseProfile,
    estimate_test_pulse_profile,
    load_pid_tuning_plan,
)

from .ui import (
    build_main_gui,
    normalize_channel_value,
)
from .controllers import HardwareStateMixin, RuntimeStateController, WidgetBindings
from .workflows.fly_log_workflow import FlyLogWorkflow
from .workflows.pid_plan_workflow import PidPlanWorkflow
from .workflows.blackbox_workflow import BlackboxWorkflow
from .workflows.fc_pid_ff_workflow import FcPidFfWorkflow
from .workflows.level_workflow import LevelWorkflow
from .workflows.connection_workflow import ConnectionWorkflow
from .workflows.channel_output_workflow import ChannelOutputWorkflow
from .workflows.auto_session_helpers import AutoSessionHelpers
from .workflows.auto_session_workflow import AutoSessionWorkflow
from .workflows.auto_session_completion_workflow import AutoSessionCompletionWorkflow
from .workflows.auto_session_engine import AutoSessionEngine
from .workflows.deterministic_fly_log_workflow import DeterministicFlyLogWorkflow
from .workflows.fly_log_pid_isolation_workflow import (
    FlyLogPidIsolationResult,
    restore_fly_log_pid_isolation as restore_fly_log_pid_isolation_direct,
)
from .tasks.worker_tasks import (
    analyze_blackbox_logs as worker_analyze_blackbox_logs,
    analyze_specific_blackbox_log as worker_analyze_specific_blackbox_log,
    enter_msc_and_import_blackbox_logs as worker_enter_msc_and_import_blackbox_logs,
    generate_auto_report as worker_generate_auto_report,
    prepare_fly_log_pid_isolation as worker_prepare_fly_log_pid_isolation,
    pulse_channel_force as worker_pulse_channel_force,
    read_movement_attitude as worker_read_movement_attitude,
    restore_fly_log_pid_isolation as worker_restore_fly_log_pid_isolation,
)


DEFAULT_TEST_THROTTLE_US = 1250



class ModbusApp(HardwareStateMixin):
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.ui = build_main_gui(root)
        WidgetBindings.attach_to(self, self.ui)
        RuntimeStateController(self).initialize()

    def run(self) -> None:

        def port() -> str:
            return self.port_entry.get().strip() or PORT_DEFAULT

        def parse_baud_entry(entry) -> int:
            return int(entry.get().strip())

        def arduino_baud() -> int:
            return parse_baud_entry(self.port_baud_entry)

        def fc_port() -> str:
            return self.fc_port_entry.get().strip() or FC_PORT_DEFAULT

        def fc_baud() -> int:
            return parse_baud_entry(self.fc_baud_entry)

        def pulse_axis_value(sample, axis: str) -> float:
            return auto_session_helpers.pulse_axis_value(sample, axis)

        def clamp_test_throttle_us(value: int | None) -> int:
            if value is None:
                value = DEFAULT_TEST_THROTTLE_US
            return normalize_channel_value(value)

        if not hasattr(self, "test_throttle_us"):
            self.test_throttle_us = DEFAULT_TEST_THROTTLE_US

        def get_test_throttle_us() -> int:
            return clamp_test_throttle_us(getattr(self, "test_throttle_us", DEFAULT_TEST_THROTTLE_US))

        def set_test_throttle_us(value: int | None, source: str = "") -> int:
            throttle = clamp_test_throttle_us(value)
            self.test_throttle_us = throttle

            # Keep the visible Throttle channel box in sync with the shared test throttle.
            # This only updates the GUI/internal base value; it does not immediately send
            # a live throttle command to the Arduino/FC just because a plan was generated.
            if len(self.ch_entries) > THROTTLE_CHANNEL_INDEX:
                self.ch_entries[THROTTLE_CHANNEL_INDEX].set(str(throttle))
            if len(getattr(self, "base_channel_outputs", [])) > THROTTLE_CHANNEL_INDEX:
                self.base_channel_outputs[THROTTLE_CHANNEL_INDEX] = throttle
            if len(getattr(self, "live_channel_outputs", [])) > THROTTLE_CHANNEL_INDEX:
                self.live_channel_outputs[THROTTLE_CHANNEL_INDEX] = throttle

            if source:
                self.status.set(f"Test throttle set to {throttle}us from {source}.")
            return throttle

        def _combo_values(combo) -> list[str]:
            values = combo.cget("values")
            if isinstance(values, str):
                return [str(value) for value in self.root.tk.splitlist(values)]
            return [str(value) for value in values]

        def _set_combo_value(combo, value_text: str, *, numeric_key) -> None:
            values = _combo_values(combo)
            if value_text not in values:
                values.append(value_text)
                values.sort(key=numeric_key)
                combo.configure(values=tuple(values))
            combo.set(value_text)

        def _set_axis_combo_value(combo, value_text: str) -> None:
            values = _combo_values(combo)
            if value_text not in values:
                values.append(value_text)
                combo.configure(values=tuple(values))
            combo.set(value_text)

        def _format_pulse_force_us(value: int | float) -> str:
            return str(int(round(float(value))))

        def _format_pulse_hold_s(value: int | float) -> str:
            return f"{float(value):.2f}"

        def set_test_pulse_profile(profile: TestPulseProfile | None, source: str = "") -> TestPulseProfile:
            if profile is None:
                profile = estimate_test_pulse_profile(PStartInputs())
            self.test_pulse_profile_base = profile
            _set_axis_combo_value(self.pulse_axis_combo, profile.test_axis.title())
            _set_combo_value(
                self.pulse_force_combo,
                _format_pulse_force_us(profile.main_force_us),
                numeric_key=lambda item: int(round(float(item))),
            )
            _set_combo_value(
                self.pulse_time_combo,
                _format_pulse_hold_s(ARDUINO_FIXED_PULSE_HOLD_S),
                numeric_key=lambda item: float(item),
            )
            return profile

        def read_pulse_force_us() -> int:
            force_us = int(round(float(self.pulse_force_combo.get().strip())))
            _set_combo_value(
                self.pulse_force_combo,
                _format_pulse_force_us(force_us),
                numeric_key=lambda item: int(round(float(item))),
            )
            return force_us

        def read_pulse_hold_s() -> float:
            _set_combo_value(
                self.pulse_time_combo,
                _format_pulse_hold_s(ARDUINO_FIXED_PULSE_HOLD_S),
                numeric_key=lambda item: float(item),
            )
            return ARDUINO_FIXED_PULSE_HOLD_S

        def read_pulse_axis() -> str:
            axis = self.pulse_axis_combo.get().strip().lower()
            if axis == "pitch":
                _set_axis_combo_value(self.pulse_axis_combo, "Pitch")
                return "pitch"
            _set_axis_combo_value(self.pulse_axis_combo, "Roll")
            return "roll"

        def sync_pulse_dropdowns_from_user(event=None) -> None:
            axis = read_pulse_axis()
            force_us = read_pulse_force_us()
            hold_s = read_pulse_hold_s()
            self.status.set(
                f"Fly/Log main pulse set to {axis.title()} +/-{force_us}us; "
                f"board duration is fixed at {hold_s:.2f}s."
            )

        def current_plan_test_pulse_profile() -> TestPulseProfile | None:
            plan = getattr(self, "pid_plan", None)
            return getattr(plan, "test_pulse_profile", None)

        def current_test_pulse_profile() -> TestPulseProfile:
            profile = current_plan_test_pulse_profile()
            if profile is None:
                profile = getattr(self, "test_pulse_profile_base", None)
            if profile is None:
                profile = estimate_test_pulse_profile(PStartInputs())
            return replace(
                profile,
                main_force_us=read_pulse_force_us(),
                main_hold_s=ARDUINO_FIXED_PULSE_HOLD_S,
                probe_hold_s=ARDUINO_FIXED_PULSE_HOLD_S,
            )

        set_test_pulse_profile(current_plan_test_pulse_profile())

        def read_auto_tune_config() -> AdaptiveSessionConfig:
            config = AdaptiveSessionConfig(throttle_start_us=get_test_throttle_us())
            pulse = current_test_pulse_profile()
            return replace(
                config,
                force_max_us=max(config.force_max_us, int(pulse.main_force_us)),
                roll_force_us=int(pulse.main_force_us),
                pitch_force_us=int(pulse.main_force_us),
                roll_hold_s=ARDUINO_FIXED_PULSE_HOLD_S,
                pitch_hold_s=ARDUINO_FIXED_PULSE_HOLD_S,
                hold_min_s=ARDUINO_FIXED_PULSE_HOLD_S,
                hold_max_s=ARDUINO_FIXED_PULSE_HOLD_S,
                probe_force_us=int(pulse.probe_force_us),
                probe_hold_s=ARDUINO_FIXED_PULSE_HOLD_S,
                recovery_hold_s=ARDUINO_FIXED_PULSE_HOLD_S,
            )

        fly_log_workflow = FlyLogWorkflow(
            app=self,
            start_pid_plan_fly_log=lambda: start_pid_plan_fly_log(),
            set_error=lambda title, exc: set_error(title, exc),
        )

        def refresh_fly_log_button_state() -> None:
            fly_log_workflow.refresh_button_state()

        fc_pid_ff_workflow = FcPidFfWorkflow(
            app=self,
            set_error=lambda title, exc: set_error(title, exc),
            ensure_disarmed_before_pid_write=lambda: ensure_disarmed_before_pid_write(),
            format_pid_values=lambda values: format_pid_values(values),
        )

        def format_pid_ff_value(value: float) -> str:
            return fc_pid_ff_workflow.format_value(value)

        def clear_pid_ff_displays() -> None:
            fc_pid_ff_workflow.clear_displays()

        def set_pid_ff_displays(roll_values, pitch_values) -> None:
            fc_pid_ff_workflow.set_displays(roll_values, pitch_values)

        def pid_ff_var(axis: str, gain: str) -> tk.StringVar:
            return fc_pid_ff_workflow.var(axis, gain)

        def parse_pid_ff_var(axis: str, gain: str) -> int:
            return fc_pid_ff_workflow.parse_var(axis, gain)

        def set_pid_ff_var(axis: str, gain: str, value: int) -> None:
            fc_pid_ff_workflow.set_var(axis, gain, value)

        def staged_roll_pitch_pid_ff_values() -> dict[str, dict[str, int]]:
            return fc_pid_ff_workflow.staged_roll_pitch_values()

        def publish_auto_report(text: str) -> None:
            summary = " ".join(line.strip() for line in text.splitlines() if line.strip())
            if summary:
                self.status.set(summary[:240])

        def auto_elapsed_s(now_s: float | None = None) -> float:
            return auto_session_helpers.elapsed_s(now_s)

        def set_auto_state(next_state: AdaptiveSessionState, safety_text: str = "") -> None:
            auto_session_helpers.set_state(next_state, safety_text)

        def auto_session_payload() -> dict[str, object]:
            return auto_session_helpers.payload()

        def refresh_pid_ff_from_fc(update_status: bool = False) -> bool:
            return fc_pid_ff_workflow.refresh_from_fc(update_status)

        def queue_fc_pid_ff_refresh(connected_port: str, connected_baud: int) -> None:
            fc_pid_ff_workflow.queue_refresh(connected_port, connected_baud)

        def do_load_pid_ff_from_fc() -> None:
            fc_pid_ff_workflow.load_from_fc()

        def write_staged_pid_ff_values_to_fc(target: dict[str, dict[str, int]]) -> None:
            fc_pid_ff_workflow.write_staged_values_to_fc(target)

        def do_save_pid_ff_to_fc() -> None:
            fc_pid_ff_workflow.save_to_fc()

        def record_auto_session_sample(sample) -> None:
            auto_session_helpers.record_sample(sample)

        def cancel_auto_hold_timer() -> None:
            auto_session_helpers.cancel_hold_timer()

        def cancel_fly_log_marker_timer() -> None:
            if self.fly_log_marker_after_id is not None:
                try:
                    self.root.after_cancel(self.fly_log_marker_after_id)
                except Exception:
                    pass
                finally:
                    self.fly_log_marker_after_id = None

        def begin_auto_observe_window(command: AdaptiveCommand) -> None:
            auto_session_helpers.begin_observe_window(command)

        def request_auto_angle_hold_end(command: AdaptiveCommand) -> None:
            auto_session_helpers.request_angle_hold_end(command)

        channel_output_workflow = ChannelOutputWorkflow(
            app=self,
            set_error=lambda title, exc: set_error(title, exc),
        )

        def draw_channel_output(index: int, value: int) -> None:
            channel_output_workflow.draw_channel_output(index, value)

        def parse_channel_values_with_defaults() -> list[int]:
            return channel_output_workflow.parse_channel_values_with_defaults()

        def parse_offset_values_with_defaults() -> list[int]:
            return channel_output_workflow.parse_offset_values_with_defaults()

        def adjust_channel_value(index: int, delta: int) -> None:
            channel_output_workflow.adjust_channel_value(index, delta)

        def cancel_adjust_repeat() -> None:
            channel_output_workflow.cancel_adjust_repeat()

        def on_adjust_press(adjust_handler, index: int, event: tk.Event, step: int = 5) -> None:
            channel_output_workflow.on_adjust_press(adjust_handler, index, event, step)

        def adjust_pid_ff_value(index: int, delta: int) -> None:
            fc_pid_ff_workflow.adjust_value(index, delta)

        def on_adjust_release(event: tk.Event) -> None:
            channel_output_workflow.on_adjust_release(event)

        def set_live_channel_outputs(values: list[int]) -> None:
            channel_output_workflow.set_live_channel_outputs(values)

        def arduino_output_connected() -> bool:
            return channel_output_workflow.arduino_output_connected()

        def restore_base_outputs_after_hold(offsets: list[int] | None = None) -> None:
            channel_output_workflow.restore_base_outputs_after_hold(offsets)

        def refresh_channel_outputs() -> None:
            channel_output_workflow.refresh_channel_outputs()

        def queue_live_channel_update(channels: list[int], offsets: list[int], after_update=None) -> None:
            channel_output_workflow.queue_live_channel_update(channels, offsets, after_update)

        def set_channel_entry_value(index: int, value: int) -> None:
            channel_output_workflow.set_channel_entry_value(index, value)

        def apply_auto_base_outputs(channels: list[int], safety_text: str = "", send_update: bool = True) -> None:
            channel_output_workflow.apply_auto_base_outputs(channels, safety_text, send_update)

        def restore_auto_original_base_outputs() -> None:
            channel_output_workflow.restore_auto_original_base_outputs()

        def lower_throttle_for_abort() -> None:
            channel_output_workflow.lower_throttle_for_abort()

        auto_session_helpers = AutoSessionHelpers(
            app=self,
            auto_is_running=lambda: auto_is_running(),
            auto_abort=lambda *args, **kwargs: auto_abort(*args, **kwargs),
            schedule_auto_tick=lambda *args, **kwargs: schedule_auto_tick(*args, **kwargs),
            set_live_channel_outputs=set_live_channel_outputs,
            arduino_output_connected=arduino_output_connected,
            queue_live_channel_update=queue_live_channel_update,
            parse_offset_values_with_defaults=parse_offset_values_with_defaults,
            apply_auto_base_outputs=apply_auto_base_outputs,
        )

        def adjust_auto_throttle_after_event(event: ExcitationEvent, recovery_event: bool) -> None:
            auto_session_helpers.adjust_throttle_after_event(event, recovery_event)

        def on_output_inputs_changed() -> None:
            channel_output_workflow.on_output_inputs_changed()

        def scan_fc_ports(update_status: bool = True) -> None:
            connection_workflow.scan_fc_ports(update_status)

        blackbox_workflow = BlackboxWorkflow(
            app=self,
            fc_port=fc_port,
            fc_baud=fc_baud,
            auto_is_running=lambda: auto_is_running(),
            publish_auto_report=publish_auto_report,
            set_error=lambda title, exc: set_error(title, exc),
            disconnect_fc=lambda *args, **kwargs: do_fc_disconnect(*args, **kwargs),
        )

        def format_blackbox_report(result: BlackboxImportResult) -> str:
            return blackbox_workflow.format_blackbox_report(result)

        pid_plan_workflow = PidPlanWorkflow(
            app=self,
            auto_is_running=lambda: auto_is_running(),
            publish_auto_report=publish_auto_report,
            set_error=lambda title, exc: set_error(title, exc),
            refresh_fly_log_button_state=refresh_fly_log_button_state,
            update_progress_window=lambda: update_pid_progress_window(),
            open_progress_window=lambda: open_pid_progress_window(),
            ensure_disarmed_before_pid_write=lambda: ensure_disarmed_before_pid_write(),
            stage_pid_ff_var=set_pid_ff_var,
            set_test_throttle_us=set_test_throttle_us,
            set_test_pulse_profile=set_test_pulse_profile,
        )

        def do_pid_tuning_plan() -> None:
            pid_plan_workflow.generate_plan()

        def locate_pid_tuning_plan_file() -> Path:
            return pid_plan_workflow.locate_plan_file()

        def read_fc_pid_ff_values(axes: tuple[str, ...] = ("roll", "pitch", "yaw")) -> dict[str, dict[str, int]]:
            return pid_plan_workflow.read_fc_pid_ff_values(axes)

        def format_pid_values(values: dict[str, dict[str, int]]) -> str:
            return pid_plan_workflow.format_pid_values(values)

        def format_pid_target_check(
            current: dict[str, dict[str, int]],
            target: dict[str, dict[str, int]],
        ) -> str:
            return pid_plan_workflow.format_pid_target_check(current, target)

        def set_pid_plan_report_text(
            plan: LoadedPIDTuningPlan,
            title: str,
            target: dict[str, dict[str, int]] | None = None,
            current: dict[str, dict[str, int]] | None = None,
        ) -> None:
            pid_plan_workflow.set_plan_report_text(plan, title, target, current)

        def update_pid_progress_window() -> None:
            pass

        def open_pid_progress_window() -> None:
            pass

        def ensure_disarmed_before_pid_write() -> bool:
            while True:
                try:
                    is_armed = self.fc_service.is_armed(timeout_seconds=0.8)
                except Exception as exc:
                    prompt = (
                        "Could not verify whether the drone is armed.\n\n"
                        f"{exc}\n\n"
                        "Cancel to stop, or continue only if you have confirmed the drone is disarmed."
                    )
                    return messagebox.askokcancel("Arm State Unknown", prompt, icon="warning", parent=self.root)

                if not is_armed:
                    return True

                retry = messagebox.askretrycancel(
                    "Drone Armed",
                    "The FC reports the drone is armed.\n\n"
                    "Disarm it before writing PID/FF values, then click Retry.",
                    icon="warning",
                    parent=self.root,
                )
                if not retry:
                    return False

        def stage_pid_ff_values(target: dict[str, dict[str, int]]) -> None:
            pid_plan_workflow.stage_pid_ff_values(target)

        def roll_pitch_target(
            roll_p: int,
            pitch_p: int,
            roll_d: int,
            pitch_d: int,
            roll_i: int,
            pitch_i: int,
            roll_ff: int,
            pitch_ff: int,
        ) -> dict[str, dict[str, int]]:
            return pid_plan_workflow.roll_pitch_target(
                roll_p, pitch_p, roll_d, pitch_d, roll_i, pitch_i, roll_ff, pitch_ff
            )

        def pid_plan_d_candidates() -> tuple[int, ...]:
            return pid_plan_workflow.d_candidates()

        def pid_plan_p_candidates() -> tuple[dict[str, int], ...]:
            return pid_plan_workflow.p_candidates()

        def pid_plan_d_recheck_candidates() -> tuple[int, ...]:
            return pid_plan_workflow.d_recheck_candidates()

        def complete_pid_tuning_plan(message: str) -> None:
            pid_plan_workflow.complete(message)

        def prepare_pid_plan_next_step() -> bool:
            return pid_plan_workflow.prepare_next_step()

        def advance_pid_plan_after_step() -> None:
            pid_plan_workflow.advance_after_step()

        def run_pid_plan_final_write() -> None:
            pid_plan_workflow.run_final_write()

        def continue_pid_tuning_plan() -> None:
            pid_plan_workflow.continue_plan()

        def start_pid_tuning_plan_session() -> None:
            pid_plan_workflow.start_session()

        def read_fc_armed_state_for_blackbox_import(selected_port: str, selected_baud: int) -> bool:
            return blackbox_workflow.read_fc_armed_state_for_import(selected_port, selected_baud)

        def ensure_disarmed_before_blackbox_import(selected_port: str, selected_baud: int) -> bool:
            return blackbox_workflow.ensure_disarmed_before_import(selected_port, selected_baud)

        def do_pull_blackbox_logs() -> None:
            blackbox_workflow.pull_blackbox_logs()

        def do_analyze_blackbox_logs() -> None:
            blackbox_workflow.analyze_blackbox_logs()

        def do_step_response_report() -> None:
            blackbox_workflow.generate_step_response_report()

        def auto_is_running() -> bool:
            return auto_session_workflow.is_running()

        def set_test_pulse_buttons_state(state: str) -> None:
            self.pulse_positive_button.config(state=state)
            self.pulse_negative_button.config(state=state)

        def do_test_pulse(direction: int) -> None:
            try:
                if getattr(self, "manual_pulse_inflight", False):
                    self.status.set("Test pulse already in progress.")
                    return
                if auto_is_running():
                    self.status.set("Wait for the active Auto/FlyLog task to finish first.")
                    return
                if self.level_active:
                    self.status.set("Stop auto-level before testing the pulse.")
                    return
                if not arduino_output_connected():
                    raise RuntimeError("Connect Arduino output before testing the pulse.")

                profile = current_test_pulse_profile()
                axis = read_pulse_axis()
                channel_index = PITCH_CHANNEL_INDEX if axis == "pitch" else ROLL_CHANNEL_INDEX
                signed_force_us = int(profile.main_force_us) * (1 if direction >= 0 else -1)
                hold_s = float(profile.main_hold_s)
                if len(self.base_channel_outputs) <= channel_index:
                    raise RuntimeError("Base channel output list is too short for a test pulse.")

                base_value = int(self.base_channel_outputs[channel_index])
                target = max(1000, min(2000, base_value + signed_force_us))
                active_outputs = self.base_channel_outputs.copy()
                active_outputs[channel_index] = target
                set_live_channel_outputs(active_outputs)
                self.manual_pulse_inflight = True
                set_test_pulse_buttons_state("disabled")
                hold_ms = ARDUINO_FIXED_PULSE_HOLD_MS
                direction_text = "+" if signed_force_us >= 0 else ""
                complete_status = (
                    f"Test pulse complete: {axis} {direction_text}{signed_force_us}us for board-fixed {hold_s:.2f}s."
                )
                self.status.set(
                    f"Testing {axis} pulse: CH{channel_index + 1} {base_value}->{target} "
                    f"({direction_text}{signed_force_us}us) for board-fixed {hold_s:.2f}s."
                )

                def finish_test_pulse(status_text: str) -> None:
                    self.manual_pulse_inflight = False
                    set_test_pulse_buttons_state("normal")
                    set_live_channel_outputs(self.base_channel_outputs.copy())
                    self.status.set(status_text)

                def complete_test_pulse() -> None:
                    finish_test_pulse(complete_status)

                def on_test_pulse_done(ok: bool, res: object) -> None:
                    if not ok:
                        finish_test_pulse("Test pulse failed.")
                        set_error("Test pulse error", res if isinstance(res, Exception) else RuntimeError(str(res)))
                        return
                    if not isinstance(res, int):
                        finish_test_pulse("Test pulse failed.")
                        set_error("Test pulse error", RuntimeError("Unexpected test pulse result from worker."))
                        return
                    if res == PULSE_STATUS_REJECTED:
                        finish_test_pulse("Test pulse rejected by firmware.")
                        set_error("Test pulse error", RuntimeError("Firmware rejected the test pulse command."))
                        return
                    self.status.set(
                        f"Test pulse accepted; firmware hold is {hold_ms}ms. Returning to neutral after hold."
                    )
                    self.root.after(hold_ms, complete_test_pulse)

                self.worker.submit(
                    worker_pulse_channel_force,
                    channel_index,
                    signed_force_us,
                    callback=on_test_pulse_done,
                )
            except Exception as exc:
                self.manual_pulse_inflight = False
                set_test_pulse_buttons_state("normal")
                set_live_channel_outputs(self.base_channel_outputs.copy())
                set_error("Test pulse error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

        def schedule_auto_tick(delay_ms: int | None = None) -> None:
            auto_session_engine.schedule_tick(delay_ms)

        def stop_auto_session_runtime(restore_outputs: bool = True) -> None:
            auto_session_completion.stop_runtime(restore_outputs)

        def complete_auto_session(
            next_state: AdaptiveSessionState,
            reason: str,
            warning: str = "",
            lower_throttle: bool = False,
        ) -> None:
            auto_session_completion.complete(next_state, reason, warning, lower_throttle)

        def clear_fly_log_pid_isolation_state() -> None:
            self.fly_log_pid_isolation_snapshot = None
            self.fly_log_pid_isolation_restoring = False
            self.fly_log_pid_isolation_run_complete = False
            if self.fly_log_pid_restore_after_id is not None:
                try:
                    self.root.after_cancel(self.fly_log_pid_restore_after_id)
                except Exception:
                    pass
                finally:
                    self.fly_log_pid_restore_after_id = None

        def format_fly_log_pid_isolation_axes(snapshot) -> str:
            axes = tuple(getattr(snapshot, "isolated_axes", ()) or ())
            if not axes:
                fallback_axis = getattr(snapshot, "isolated_axis", "")
                axes = (fallback_axis,) if fallback_axis else ()
            names = [str(axis).strip().title() for axis in axes if str(axis).strip()]
            return "/".join(names) if names else "isolated axes"

        def finalize_pid_plan_fly_log_complete(
            completed_title: str,
            reason: str,
            *,
            restored_axis: str = "",
        ) -> None:
            self.pid_plan_waiting_for_fly_log = False
            self.pid_plan_current_candidate_title = ""
            self.pid_plan_current_candidate_phase = ""
            self.pid_plan_current_candidate_target = None
            refresh_fly_log_button_state()
            suffix = f" Restored {restored_axis} PID/FF." if restored_axis else ""
            self.status.set(f"Fly/Log complete: {completed_title or reason}.{suffix}".strip())
            update_pid_progress_window()
            lines = [
                "Fly/Log is complete. The CH8 PID test marker bracketed the calibration probes in Blackbox."
            ]
            if restored_axis:
                lines.append(f"Restored {restored_axis} PID/FF.")
            lines.append("Press Next PID Plan Step when ready.")
            messagebox.showinfo(
                "Fly/Log Complete",
                "\n\n".join(lines),
                parent=self.root,
            )

        def schedule_pid_isolation_restore_after_disarm(completed_title: str, reason: str) -> None:
            snapshot = self.fly_log_pid_isolation_snapshot
            if snapshot is None:
                finalize_pid_plan_fly_log_complete(completed_title, reason)
                return
            if self.fly_log_pid_isolation_restoring:
                return

            def check_disarmed() -> None:
                self.fly_log_pid_restore_after_id = None
                if self.is_closing:
                    return
                snapshot_inner = self.fly_log_pid_isolation_snapshot
                if snapshot_inner is None:
                    finalize_pid_plan_fly_log_complete(completed_title, reason)
                    return
                if not self.fc_service.is_connected:
                    self.status.set("Reconnect FC while disarmed to restore Fly/Log PID/FF isolation.")
                    self.fly_log_pid_restore_after_id = self.root.after(1000, check_disarmed)
                    return
                try:
                    is_armed = self.fc_service.is_armed(timeout_seconds=0.8)
                except Exception as exc:
                    self.status.set(f"Waiting to restore PID/FF isolation; arm check failed: {exc}")
                    self.fly_log_pid_restore_after_id = self.root.after(1000, check_disarmed)
                    return
                if is_armed:
                    axes_text = format_fly_log_pid_isolation_axes(snapshot_inner)
                    self.status.set(
                        f"Fly/Log complete. Disarm to restore {axes_text} PID/FF."
                    )
                    self.fly_log_pid_restore_after_id = self.root.after(1000, check_disarmed)
                    return

                self.fly_log_pid_isolation_restoring = True
                refresh_fly_log_button_state()
                axes_text = format_fly_log_pid_isolation_axes(snapshot_inner)
                self.status.set(f"Restoring {axes_text} PID/FF after Fly/Log...")

                def on_restore_done(ok: bool, res: object) -> None:
                    self.fly_log_pid_isolation_restoring = False
                    refresh_fly_log_button_state()
                    if not ok:
                        set_error(
                            "Fly/Log PID/FF restore error",
                            res if isinstance(res, Exception) else RuntimeError(str(res)),
                        )
                        self.fly_log_pid_restore_after_id = self.root.after(1000, check_disarmed)
                        return
                    if not isinstance(res, FlyLogPidIsolationResult):
                        set_error("Fly/Log PID/FF restore error", RuntimeError("Unexpected restore result."))
                        self.fly_log_pid_restore_after_id = self.root.after(1000, check_disarmed)
                        return
                    set_pid_ff_displays(res.roll_values, res.pitch_values)
                    restored_axis = format_fly_log_pid_isolation_axes(res.snapshot)
                    clear_fly_log_pid_isolation_state()
                    finalize_pid_plan_fly_log_complete(completed_title, reason, restored_axis=restored_axis)

                self.fc_worker.submit(
                    worker_restore_fly_log_pid_isolation,
                    self.fc_service,
                    snapshot_inner,
                    callback=on_restore_done,
                )

            check_disarmed()

        def auto_abort(reason: str, warning: str = "", continue_pipeline: bool = False) -> None:
            was_fly_log = self.pid_plan_fly_log_active
            completed_title = self.pid_plan_current_candidate_title
            auto_session_workflow.abort(reason, warning, continue_pipeline)
            if was_fly_log and self.fly_log_pid_isolation_snapshot is not None:
                self.fly_log_pid_isolation_run_complete = True
                refresh_fly_log_button_state()
                schedule_pid_isolation_restore_after_disarm(completed_title, reason)

        def finish_pid_plan_fly_log(reason: str = "Fly/Log calibration complete.") -> None:
            if not self.pid_plan_fly_log_active:
                return
            self.fly_log_finishing = False
            complete_auto_session(AdaptiveSessionState.report_ready, reason)
            self.pid_plan_fly_log_active = False
            completed_title = self.pid_plan_current_candidate_title
            refresh_fly_log_button_state()
            update_pid_progress_window()
            if self.fly_log_pid_isolation_snapshot is not None:
                self.fly_log_pid_isolation_run_complete = True
                axes_text = format_fly_log_pid_isolation_axes(self.fly_log_pid_isolation_snapshot)
                refresh_fly_log_button_state()
                self.status.set(f"Fly/Log complete: {completed_title or reason}. Disarm to restore {axes_text} PID/FF.")
                messagebox.showinfo(
                    "Fly/Log Complete",
                    "Fly/Log pulses are complete. The CH8 PID test marker bracketed the calibration probes in Blackbox.\n\n"
                    f"Disarm the drone now. After the FC reports disarmed, the app will restore {axes_text} PID/FF automatically.",
                    parent=self.root,
                )
                schedule_pid_isolation_restore_after_disarm(completed_title, reason)
                return

            finalize_pid_plan_fly_log_complete(completed_title, reason)
            return

        def begin_fly_log_marker_off_and_complete() -> None:
            if not self.pid_plan_fly_log_active or self.fly_log_finishing:
                return
            self.fly_log_finishing = True
            self.base_channel_outputs = channels_with_pid_test_ch8(self.base_channel_outputs, active=False)
            set_live_channel_outputs(self.base_channel_outputs.copy())
            self.status.set("Setting CH8 back to 1000us...")

            def on_marker_disabled(ok: bool, res: object) -> None:
                self.fly_log_finishing = False
                if not self.pid_plan_fly_log_active:
                    return
                if not ok:
                    auto_abort(
                        "Unable to set CH8 back to 1000us.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                finish_pid_plan_fly_log()

            queue_live_channel_update(
                self.base_channel_outputs.copy(),
                parse_offset_values_with_defaults(),
                after_update=on_marker_disabled,
            )

        auto_session_engine = AutoSessionEngine(
            app=self,
            auto_is_running=auto_is_running,
            auto_abort=auto_abort,
            set_auto_state=set_auto_state,
            auto_elapsed_s=auto_elapsed_s,
            pulse_axis_value=pulse_axis_value,
            arduino_output_connected=arduino_output_connected,
            set_live_channel_outputs=set_live_channel_outputs,
            begin_auto_observe_window=begin_auto_observe_window,
            begin_fly_log_marker_off_and_complete=begin_fly_log_marker_off_and_complete,
            finish_pid_plan_fly_log=finish_pid_plan_fly_log,
            adjust_auto_throttle_after_event=adjust_auto_throttle_after_event,
        )

        def start_auto_session() -> None:
            if self.blackbox_import_inflight:
                raise RuntimeError("Blackbox import/analyze is in progress.")
            if not self.fc_service.is_connected:
                raise RuntimeError("Connect FC before starting auto session.")
            if self.level_active:
                raise RuntimeError("Stop auto-level before starting auto session.")
            if self.attitude_service.latest_attitude() is None:
                raise RuntimeError("No attitude-board sample yet. Wait for movement telemetry then retry.")
            self.auto_config = read_auto_tune_config()

            prompt = (
                "Confirm preflight:\n"
                "- FC is connected and attitude-board telemetry is live\n"
                "- Drone is disarmed before pressing Save for PID/FF values\n"
                "- Guided plan steps only stage PID/FF values in the boxes\n"
                "- You are ready to prepare Fly/Log while disarmed, then arm and run one candidate at a time\n"
                "- You will land and disarm so the app can restore isolated PID/FF before pressing Next PID Plan Step\n\n"
                "The app will load pid_tuning_plan.txt, compare the current FC PID/FF values "
                "to the next plan target, and ask before staging each step in the PID boxes.\n\n"
                "It will not run randomized stick pulses and it will not write or save PID/FF values automatically.\n\n"
                "Start guided PID tuning plan now?"
            )
            if not messagebox.askyesno("Start Auto Session", prompt):
                self.status.set("Auto session start canceled.")
                return
            start_pid_tuning_plan_session()

        def finalize_auto_event() -> None:
            auto_session_engine.finalize_event()

        def issue_auto_command(command: AdaptiveCommand) -> None:
            auto_session_engine.issue_command(command)

        def issue_next_auto_probe() -> bool:
            return auto_session_engine.issue_next_probe()

        def run_auto_tick() -> None:
            auto_session_engine.run_tick()

        def begin_auto_pipeline() -> None:
            auto_session_completion.begin_pipeline()

        def fc_is_armed_for_fly_log() -> bool:
            try:
                if self.fc_service.is_armed(timeout_seconds=0.8):
                    return True
            except Exception as exc:
                messagebox.showerror("Fly/Log Arm Check", f"Could not verify armed state:\n\n{exc}", parent=self.root)
                return False
            messagebox.showwarning(
                "Fly/Log Requires Armed",
                "The FC reports the drone is not armed.\n\n"
                "Arm the drone first, then press Fly/Log.",
                parent=self.root,
            )
            return False

        def start_pid_plan_fly_log() -> None:
            if not self.pid_plan_active or not self.pid_plan_waiting_for_fly_log:
                self.status.set("No PID plan candidate is ready for Fly/Log.")
                return
            if self.blackbox_import_inflight:
                self.status.set("Blackbox/report task already in progress.")
                return
            if auto_is_running():
                self.status.set("Auto/FlyLog task already running.")
                return
            if not arduino_output_connected():
                raise RuntimeError("Connect Arduino output before Fly/Log.")
            if not self.fc_service.is_connected:
                raise RuntimeError("Connect FC before Fly/Log.")
            if self.level_active:
                raise RuntimeError("Stop auto-level before Fly/Log.")
            if self.attitude_service.latest_attitude() is None:
                raise RuntimeError("No attitude-board sample yet. Wait for movement telemetry then retry.")

            self.auto_config = replace(read_auto_tune_config(), max_runtime_s=FLY_LOG_SAFETY_TIMEOUT_S)
            pulse = current_test_pulse_profile()
            test_axis = str(pulse.test_axis).strip().lower()
            snapshot = self.fly_log_pid_isolation_snapshot
            if snapshot is None:
                if not ensure_disarmed_before_pid_write():
                    self.status.set("Fly/Log PID/FF isolation canceled; disarm before preparing.")
                    return
                isolated_axis = "pitch" if test_axis == "roll" else "roll"
                axes_text = "/".join(axis.title() for axis in (isolated_axis, "yaw"))
                prompt = (
                    f"Prepare Fly/Log candidate: {self.pid_plan_current_candidate_title or 'current PID plan step'}\n\n"
                    f"This will save the current Roll/Pitch/Yaw PID/FF values, then write and save {axes_text} "
                    "P/I/D/FF = 0 for this test only.\n\n"
                    f"The tested axis remains active: {test_axis.title()}.\n\n"
                    "After preparation finishes, arm the drone and press Fly/Log again to run the pulse sequence. "
                    "After the sequence, disarm and the app will restore the zeroed PID/FF values automatically.\n\n"
                    "INAV motor configuration will not be changed."
                )
                if not messagebox.askokcancel("Prepare Fly/Log", prompt, icon="warning", parent=self.root):
                    self.status.set("Fly/Log preparation canceled.")
                    return

                self.fly_log_button.config(state="disabled")
                self.status.set(f"Preparing Fly/Log PID isolation: zeroing {axes_text} P/I/D/FF...")

                def on_prepare_done(ok: bool, res: object) -> None:
                    if not ok:
                        clear_fly_log_pid_isolation_state()
                        refresh_fly_log_button_state()
                        set_error(
                            "Fly/Log PID/FF preparation error",
                            res if isinstance(res, Exception) else RuntimeError(str(res)),
                        )
                        return
                    if not isinstance(res, FlyLogPidIsolationResult):
                        clear_fly_log_pid_isolation_state()
                        refresh_fly_log_button_state()
                        set_error("Fly/Log PID/FF preparation error", RuntimeError("Unexpected preparation result."))
                        return
                    self.fly_log_pid_isolation_snapshot = res.snapshot
                    self.fly_log_pid_isolation_run_complete = False
                    set_pid_ff_displays(res.roll_values, res.pitch_values)
                    prepared_axes_text = format_fly_log_pid_isolation_axes(res.snapshot)
                    refresh_fly_log_button_state()
                    self.status.set(
                        f"Fly/Log prepared: {prepared_axes_text} P/I/D/FF are zeroed. Arm, then press Fly/Log again."
                    )
                    messagebox.showinfo(
                        "Fly/Log Prepared",
                        f"{prepared_axes_text} P/I/D/FF are zeroed and saved for this Fly/Log test.\n\n"
                        "Arm the drone, press Fly/Log again, then disarm after the pulse sequence so the app can restore them.",
                        parent=self.root,
                    )

                self.fc_worker.submit(
                    worker_prepare_fly_log_pid_isolation,
                    self.fc_service,
                    test_axis,
                    callback=on_prepare_done,
                )
                return

            if self.fly_log_pid_isolation_run_complete:
                axes_text = format_fly_log_pid_isolation_axes(snapshot)
                self.status.set(f"Fly/Log already ran. Disarm to restore {axes_text} PID/FF.")
                return
            if str(snapshot.test_axis).strip().lower() != test_axis:
                self.status.set("Fly/Log prepared for a different test axis. Restore or reconnect before changing axis.")
                messagebox.showwarning(
                    "Fly/Log Axis Changed",
                    f"PID isolation was prepared for {snapshot.test_axis.title()}, but the current pulse axis is "
                    f"{test_axis.title()}.\n\n"
                    "Restore the current isolation before changing the test axis.",
                    parent=self.root,
                )
                return
            if not fc_is_armed_for_fly_log():
                return

            pulse_text = (
                f"{pulse.aircraft_name}: {pulse.test_axis} start probe +/-{pulse.probe_force_us}us; "
                f"main sequence 10 positive and 10 negative {pulse.test_axis} pulses at "
                f"dropdown-selected +/-{pulse.main_force_us}us; board pulse duration fixed at "
                f"{ARDUINO_FIXED_PULSE_HOLD_S:.2f}s; neutral wait {pulse.neutral_wait_ms}ms."
            )
            axes_text = format_fly_log_pid_isolation_axes(snapshot)
            prompt = (
                f"Fly/Log candidate: {self.pid_plan_current_candidate_title or 'current PID plan step'}\n\n"
                "The FC reports ARMED.\n\n"
                f"PID isolation is active: {axes_text} P/I/D/FF are zeroed until this run is complete and the FC is disarmed.\n\n"
                "Pressing OK will run this sequence:\n"
                "1. Brief spin-up on the current outputs\n"
                "2. Small pre-marker start probes\n"
                "3. CH8 PID test marker ON (BEEPERON in Blackbox)\n"
                "4. Marked deterministic one-axis pulse pairs\n"
                "5. CH8 PID test marker OFF (BEEPEROFF in Blackbox)\n\n"
                f"Pulse profile: {pulse_text}\n\n"
                "Chart Step Response analyzes the marker bracket, not a fixed time window. "
                "You can also bracket your own logs of any length with CH8 markers.\n\n"
                "INAV motor configuration will not be changed.\n"
                "No PID/FF values will be written while armed.\n\n"
                "Keep the drone secured, keep the area clear, and be ready to disarm."
            )
            if not messagebox.askokcancel("Start Fly/Log", prompt, icon="warning", parent=self.root):
                self.status.set("Fly/Log canceled.")
                return

            deterministic_fly_log_workflow.start()
        def do_auto_session_toggle() -> None:
            auto_session_workflow.toggle()

        def set_error(title: str, exc: Exception) -> None:
            if self.is_closing:
                return
            self.status.set("Error")
            messagebox.showerror(title, str(exc))

        def update_link_indicators() -> None:
            connection_workflow.update_link_indicators()

        # Workflow wiring starts here. Keep these constructors below the callback functions
        # they receive. Use lambdas for callbacks that reference workflow objects created later.
        auto_session_workflow = AutoSessionWorkflow(
            app=self,
            start_auto_session=start_auto_session,
            open_pid_progress_window=open_pid_progress_window,
            continue_pid_tuning_plan=continue_pid_tuning_plan,
            complete_auto_session=complete_auto_session,
            refresh_fly_log_button_state=refresh_fly_log_button_state,
            complete_pid_tuning_plan=complete_pid_tuning_plan,
            update_link_indicators=lambda: update_link_indicators(),
            update_pid_progress_window=update_pid_progress_window,
            begin_auto_pipeline=begin_auto_pipeline,
            set_error=lambda title, exc: set_error(title, exc),
        )

        level_workflow = LevelWorkflow(
            app=self,
            set_live_channel_outputs=set_live_channel_outputs,
            queue_live_channel_update=queue_live_channel_update,
            update_link_indicators=lambda: update_link_indicators(),
            arduino_output_connected=arduino_output_connected,
            parse_offsets=parse_offset_values_with_defaults,
            get_test_throttle_us=get_test_throttle_us,
            set_error=lambda title, exc: set_error(title, exc),
        )

        def cancel_level_timer() -> None:
            level_workflow.cancel_timer()

        def stop_level_loop(update_status: bool = False, reason: str = "Auto-level stopped.") -> None:
            level_workflow.stop(update_status=update_status, reason=reason)

        def refresh_level_button_state(fc_connected: bool) -> None:
            level_workflow.refresh_button_state(fc_connected)

        def do_level() -> None:
            level_workflow.toggle()

        # ConnectionWorkflow creates do_fc_disconnect below. AutoSessionCompletionWorkflow
        # depends on that wrapper, so completion wiring must stay after this block.
        connection_workflow = ConnectionWorkflow(
            app=self,
            port=port,
            arduino_baud=arduino_baud,
            fc_port=fc_port,
            fc_baud=fc_baud,
            auto_is_running=lambda: auto_is_running(),
            refresh_level_button_state=refresh_level_button_state,
            refresh_fly_log_button_state=refresh_fly_log_button_state,
            clear_pid_ff_displays=clear_pid_ff_displays,
            queue_fc_pid_ff_refresh=queue_fc_pid_ff_refresh,
            set_live_channel_outputs=set_live_channel_outputs,
            parse_channel_values_with_defaults=parse_channel_values_with_defaults,
            parse_offset_values_with_defaults=parse_offset_values_with_defaults,
            set_error=lambda title, exc: set_error(title, exc),
            auto_abort=lambda *args, **kwargs: auto_abort(*args, **kwargs),
        )

        def do_fc_connect() -> None:
            connection_workflow.connect_fc()

        def do_fc_disconnect(update_status: bool = True) -> None:
            connection_workflow.disconnect_fc(update_status=update_status)

        def do_fc_toggle() -> None:
            connection_workflow.toggle_fc()

        def do_arduino_toggle() -> None:
            connection_workflow.toggle_arduino()

        # Keep this after do_fc_disconnect is defined. Passing do_fc_disconnect before
        # this point raises UnboundLocalError because run() has not bound the wrapper yet.
        auto_session_completion = AutoSessionCompletionWorkflow(
            app=self,
            fc_port=fc_port,
            fc_baud=fc_baud,
            ensure_disarmed_before_blackbox_import=ensure_disarmed_before_blackbox_import,
            do_fc_disconnect=do_fc_disconnect,
            set_auto_state=set_auto_state,
            publish_auto_report=publish_auto_report,
            format_blackbox_report=format_blackbox_report,
            auto_session_payload=auto_session_payload,
            auto_abort=lambda *args, **kwargs: auto_abort(*args, **kwargs),
            cancel_auto_hold_timer=cancel_auto_hold_timer,
            cancel_fly_log_marker_timer=cancel_fly_log_marker_timer,
            restore_auto_original_base_outputs=restore_auto_original_base_outputs,
            lower_throttle_for_abort=lower_throttle_for_abort,
            arduino_output_connected=arduino_output_connected,
            restore_base_outputs_after_hold=restore_base_outputs_after_hold,
            worker_enter_msc_and_import_blackbox_logs=worker_enter_msc_and_import_blackbox_logs,
            worker_analyze_specific_blackbox_log=worker_analyze_specific_blackbox_log,
            worker_analyze_blackbox_logs=worker_analyze_blackbox_logs,
            worker_generate_auto_report=worker_generate_auto_report,
        )

        deterministic_fly_log_workflow = DeterministicFlyLogWorkflow(
            app=self,
            auto_is_running=lambda: auto_is_running(),
            auto_abort=lambda *args, **kwargs: auto_abort(*args, **kwargs),
            set_auto_state=set_auto_state,
            queue_live_channel_update=queue_live_channel_update,
            parse_offset_values_with_defaults=parse_offset_values_with_defaults,
            set_live_channel_outputs=set_live_channel_outputs,
            get_test_throttle_us=get_test_throttle_us,
            get_test_pulse_profile=current_test_pulse_profile,
            begin_fly_log_marker_off_and_complete=begin_fly_log_marker_off_and_complete,
            refresh_fly_log_button_state=refresh_fly_log_button_state,
            open_pid_progress_window=open_pid_progress_window,
            update_pid_progress_window=update_pid_progress_window,
            publish_auto_report=publish_auto_report,
        )

        def poll_attitude() -> None:
            try:
                if self.controller.is_connected and not self.attitude_poll_inflight:
                    self.attitude_poll_inflight = True

                    def on_attitude_sample(ok: bool, res: object) -> None:
                        self.attitude_poll_inflight = False
                        if not ok:
                            return
                        if res is None:
                            return
                        if self.attitude_service.ingest_sample(res):
                            self.attitude_sample_updated = True

                    self.worker.submit(worker_read_movement_attitude, callback=on_attitude_sample)

                sample = self.attitude_service.latest_attitude()
                if sample is not None:
                    if self.attitude_sample_updated:
                        record_auto_session_sample(sample)
                        self.attitude_sample_updated = False
                    self.horizon.set_attitude(sample.roll_deg, sample.pitch_deg)
                    self.roll_text.set(f"Roll: {sample.roll_deg:6.1f} deg")
                    self.pitch_text.set(f"Pitch: {sample.pitch_deg:6.1f} deg")
            except Exception:
                pass
            self.fc_poll_after_id = self.root.after(60, poll_attitude)

        def poll_results() -> None:
            for worker in (self.worker, self.fc_worker):
                while True:
                    try:
                        cb, ok, res = worker.results.get_nowait()
                    except queue.Empty:
                        break
                    if cb:
                        try:
                            cb(ok, res)
                        except Exception as e:
                            set_error("Callback error", e)
            self.root.after(50, poll_results)

        def on_close() -> None:
            self.is_closing = True
            cancel_adjust_repeat()
            if auto_is_running():
                lower_throttle_for_abort()
                stop_auto_session_runtime(restore_outputs=False)
            else:
                stop_auto_session_runtime()
            snapshot = self.fly_log_pid_isolation_snapshot
            if (
                snapshot is not None
                and not self.fly_log_pid_isolation_restoring
                and self.fc_service.is_connected
            ):
                try:
                    if not self.fc_service.is_armed(timeout_seconds=0.8):
                        res = restore_fly_log_pid_isolation_direct(self.fc_service, snapshot)
                        set_pid_ff_displays(res.roll_values, res.pitch_values)
                        clear_fly_log_pid_isolation_state()
                except Exception:
                    pass
            if self.fc_poll_after_id is not None:
                try:
                    self.root.after_cancel(self.fc_poll_after_id)
                except Exception:
                    pass
                finally:
                    self.fc_poll_after_id = None
            self.attitude_service.disconnect()
            self.attitude_sample_updated = False
            self.attitude_poll_inflight = False

            def on_stop_and_close(ok: bool, res: object) -> None:
                do_fc_disconnect(update_status=False)
                try:
                    self.worker.stop()
                except Exception:
                    pass
                try:
                    self.fc_worker.stop()
                except Exception:
                    pass
                self.root.destroy()

            try:
                self.controller.shutdown(callback=on_stop_and_close)
            except Exception:
                on_stop_and_close(False, None)

        scan_fc_ports(update_status=False)

        for pulse_combo in (self.pulse_force_combo, self.pulse_time_combo):
            pulse_combo.bind("<<ComboboxSelected>>", sync_pulse_dropdowns_from_user)
        self.pulse_axis_combo.bind("<<ComboboxSelected>>", sync_pulse_dropdowns_from_user)

        self.scan_fc_button.config(command=scan_fc_ports)
        self.connect_fc_button.config(command=do_fc_toggle)
        self.import_blackbox_button.config(command=do_pull_blackbox_logs)
        self.analyze_blackbox_button.config(command=do_analyze_blackbox_logs)
        self.fly_log_button.config(command=fly_log_workflow.toggle)
        self.pulse_positive_button.config(command=lambda: do_test_pulse(1))
        self.pulse_negative_button.config(command=lambda: do_test_pulse(-1))
        self.load_pid_ff_button.config(command=do_load_pid_ff_from_fc)
        self.save_pid_ff_button.config(command=do_save_pid_ff_to_fc)
        self.step_response_button.config(command=do_step_response_report)
        self.pid_tuning_plan_button.config(command=do_pid_tuning_plan)
        self.arduino_button.config(command=do_arduino_toggle)
        self.level_button.config(command=do_level)
        for i, canvas in enumerate(self.channel_adjust_canvases):
            canvas.bind("<ButtonPress-1>", lambda event, i=i: on_adjust_press(adjust_channel_value, i, event, 25))
            canvas.bind("<ButtonRelease-1>", on_adjust_release)
            canvas.bind("<Leave>", on_adjust_release)
        for i, canvas in enumerate(self.pid_ff_adjust_canvases):
            canvas.bind("<ButtonPress-1>", lambda event, i=i: on_adjust_press(adjust_pid_ff_value, i, event, 1))
            canvas.bind("<ButtonRelease-1>", on_adjust_release)
            canvas.bind("<Leave>", on_adjust_release)
        for entry in self.ch_entries:
            entry.bind("<KeyRelease>", lambda _event: on_output_inputs_changed())
            entry.bind("<FocusOut>", lambda _event: on_output_inputs_changed())
            entry.bind("<<ComboboxSelected>>", lambda _event: on_output_inputs_changed())
        set_auto_state(AdaptiveSessionState.idle)
        set_live_channel_outputs(parse_channel_values_with_defaults())
        update_link_indicators()
        self.root.after(50, poll_results)
        self.fc_poll_after_id = self.root.after(60, poll_attitude)
        self.root.protocol("WM_DELETE_WINDOW", on_close)

        self.root.mainloop()


def main() -> None:
    root = tk.Tk()
    root.attributes("-topmost", True)
    root.after_idle(lambda: root.attributes("-topmost", True))
    app = ModbusApp(root)
    app.run()

if __name__ == "__main__":
    main()

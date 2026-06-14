"""Application runtime orchestration."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path
import queue
import time
import tkinter as tk
from tkinter import messagebox

from serialUSB.inav_serial_service import (
    AxisPidFf,
    FF_SETTING_NAME,
    InavSerialService,
    PID_SETTING_NAME,
)

from .constants import (
    FC_PORT_DEFAULT,
    LEVEL_CENTER_US,
    LEVEL_DEADBAND_DEG,
    LEVEL_FULL_SCALE_DEG,
    LEVEL_LOOP_INTERVAL_MS,
    LEVEL_MAX_DELTA_US,
    LEVEL_MIN_DELTA_US,
    LEVEL_PULSE_TIMEOUT_S,
    LEVEL_TIMEOUT_DEFAULT_S,
    PITCH_CHANNEL_INDEX,
    FLY_LOG_SAFETY_TIMEOUT_S,
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
    axis_channel_index,
)
from .ch8_marker import channels_with_pid_test_ch8
from .auto_tune_report import AutoTuneReport
from .blackbox_import import BlackboxImportResult
from .pid_tuning_workflow import (
    LoadedPIDTuningPlan,
    load_pid_tuning_plan,
)
from .step_response_report import (
    MAX_STEP_RESPONSE_LOGS,
    StepResponseReport,
    format_step_response_report,
)
from .ui import (
    build_main_gui,
    parse_entries,
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
from .presenters.pid_progress_presenter import PidProgressPresenter
from .tasks.worker_tasks import (
    analyze_blackbox_logs as worker_analyze_blackbox_logs,
    analyze_specific_blackbox_log as worker_analyze_specific_blackbox_log,
    enter_msc_and_import_blackbox_logs as worker_enter_msc_and_import_blackbox_logs,
    generate_auto_report as worker_generate_auto_report,
    generate_step_response_report as worker_generate_step_response_report,
    read_fc_pid_ff as worker_read_fc_pid_ff,
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

        def fc_port() -> str:
            return self.fc_port_entry.get().strip() or FC_PORT_DEFAULT

        def fc_baud() -> int:
            try:
                value = int(self.fc_baud_entry.get().strip())
            except ValueError as exc:
                raise RuntimeError("FC baud must be an integer.") from exc
            if value <= 0:
                raise RuntimeError("FC baud must be > 0.")
            return value

        def pulse_axis_value(sample, axis: str) -> float:
            return auto_session_helpers.pulse_axis_value(sample, axis)

        def clamp_test_throttle_us(value: int | None) -> int:
            if value is None:
                value = DEFAULT_TEST_THROTTLE_US
            return max(1000, min(2000, int(value)))

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
                self.ch_entries[THROTTLE_CHANNEL_INDEX].delete(0, tk.END)
                self.ch_entries[THROTTLE_CHANNEL_INDEX].insert(0, str(throttle))
            if len(getattr(self, "base_channel_outputs", [])) > THROTTLE_CHANNEL_INDEX:
                self.base_channel_outputs[THROTTLE_CHANNEL_INDEX] = throttle
            if len(getattr(self, "live_channel_outputs", [])) > THROTTLE_CHANNEL_INDEX:
                self.live_channel_outputs[THROTTLE_CHANNEL_INDEX] = throttle

            if source:
                self.status.set(f"Test throttle set to {throttle}us from {source}.")
            return throttle

        def read_auto_tune_config() -> AdaptiveSessionConfig:
            return AdaptiveSessionConfig(throttle_start_us=get_test_throttle_us())

        def simulation_mode_enabled() -> bool:
            return bool(self.simulation_mode_var.get())

        fly_log_workflow = FlyLogWorkflow(
            app=self,
            simulation_mode_enabled=simulation_mode_enabled,
            start_pid_plan_fly_log=lambda: start_pid_plan_fly_log(),
            start_simulated_fly_log=lambda: start_simulated_fly_log(),
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

        def set_auto_report_text(text: str) -> None:
            self.auto_report_text.config(state="normal")
            self.auto_report_text.delete("1.0", tk.END)
            self.auto_report_text.insert("1.0", text.strip() + ("\n" if text and not text.endswith("\n") else ""))
            self.auto_report_text.config(state="disabled")

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
            simulation_mode_enabled=simulation_mode_enabled,
            auto_is_running=lambda: auto_is_running(),
            set_auto_report_text=set_auto_report_text,
            set_error=lambda title, exc: set_error(title, exc),
            disconnect_fc=lambda *args, **kwargs: do_fc_disconnect(*args, **kwargs),
        )

        def format_blackbox_report(result: BlackboxImportResult) -> str:
            return blackbox_workflow.format_blackbox_report(result)

        pid_plan_workflow = PidPlanWorkflow(
            app=self,
            auto_is_running=lambda: auto_is_running(),
            set_auto_report_text=set_auto_report_text,
            set_error=lambda title, exc: set_error(title, exc),
            set_auto_button_idle=lambda: set_auto_button_idle(),
            refresh_fly_log_button_state=refresh_fly_log_button_state,
            update_progress_window=lambda: update_pid_progress_window(),
            open_progress_window=lambda: open_pid_progress_window(),
            ensure_disarmed_before_pid_write=lambda: ensure_disarmed_before_pid_write(),
            stage_pid_ff_var=set_pid_ff_var,
            stop_simulated_auto_session=lambda *args, **kwargs: stop_simulated_auto_session(*args, **kwargs),
            set_test_throttle_us=set_test_throttle_us,
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

        def current_pid_plan_step() -> tuple[str, str, dict[str, dict[str, int]]] | None:
            return pid_plan_workflow.current_step()

        pid_progress_presenter = PidProgressPresenter(
            app=self,
            current_pid_plan_step=lambda: current_pid_plan_step(),
            format_pid_values=format_pid_values,
            set_error=lambda title, exc: set_error(title, exc),
        )

        def update_pid_progress_window() -> None:
            pid_progress_presenter.update_window()

        def close_pid_progress_window() -> None:
            pid_progress_presenter.close_window()

        def open_pid_progress_window() -> None:
            pid_progress_presenter.open_window()

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

        def schedule_auto_tick(delay_ms: int | None = None) -> None:
            auto_session_engine.schedule_tick(delay_ms)

        def stop_auto_session_runtime(restore_outputs: bool = True) -> None:
            auto_session_completion.stop_runtime(restore_outputs)

        def set_auto_button_idle() -> None:
            auto_session_workflow.set_button_idle()

        def complete_auto_session(
            next_state: AdaptiveSessionState,
            reason: str,
            warning: str = "",
            lower_throttle: bool = False,
        ) -> None:
            auto_session_completion.complete(next_state, reason, warning, lower_throttle)

        def auto_abort(reason: str, warning: str = "", continue_pipeline: bool = False) -> None:
            auto_session_workflow.abort(reason, warning, continue_pipeline)

        def finish_pid_plan_fly_log(reason: str = "Fly/Log calibration complete.") -> None:
            if not self.pid_plan_fly_log_active:
                return
            self.fly_log_finishing = False
            complete_auto_session(AdaptiveSessionState.report_ready, reason)
            self.pid_plan_fly_log_active = False
            self.pid_plan_waiting_for_fly_log = False
            completed_title = self.pid_plan_current_candidate_title
            self.pid_plan_current_candidate_title = ""
            self.pid_plan_current_candidate_phase = ""
            self.pid_plan_current_candidate_target = None
            self.auto_session_button.config(text="Next PID Plan Step", state="normal")
            refresh_fly_log_button_state()
            self.status.set(f"Fly/Log complete: {completed_title or reason}")
            update_pid_progress_window()
            messagebox.showinfo(
                "Fly/Log Complete",
                "Fly/Log is complete. The CH8 PID test marker bracketed the calibration probes in Blackbox.\n\n"
                "Disarm the drone now. After it is disarmed, press Next PID Plan Step.",
                parent=self.root,
            )

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
            parse_offset_values_with_defaults=parse_offset_values_with_defaults,
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
            if self.fc_service.latest_attitude() is None:
                raise RuntimeError("No FC attitude sample yet. Wait for telemetry then retry.")
            self.auto_config = read_auto_tune_config()

            prompt = (
                "Confirm preflight:\n"
                "- FC is connected and attitude telemetry is live\n"
                "- Drone is disarmed before pressing Save for PID/FF values\n"
                "- Guided plan steps only stage PID/FF values in the boxes\n"
                "- You are ready to arm and press Fly/Log for one candidate at a time\n"
                "- You will land and disarm before pressing Next PID Plan Step\n\n"
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
            if self.sim_active:
                self.status.set("Stop simulation before Fly/Log.")
                return
            if not arduino_output_connected():
                raise RuntimeError("Connect Arduino output before Fly/Log.")
            if not self.fc_service.is_connected:
                raise RuntimeError("Connect FC before Fly/Log.")
            if self.level_active:
                raise RuntimeError("Stop auto-level before Fly/Log.")
            if self.fc_service.latest_attitude() is None:
                raise RuntimeError("No FC attitude sample yet. Wait for telemetry then retry.")
            if not fc_is_armed_for_fly_log():
                return

            self.auto_config = replace(read_auto_tune_config(), max_runtime_s=FLY_LOG_SAFETY_TIMEOUT_S)
            prompt = (
                f"Fly/Log candidate: {self.pid_plan_current_candidate_title or 'current PID plan step'}\n\n"
                "The FC reports ARMED.\n\n"
                "Pressing OK will run this sequence:\n"
                "1. Brief spin-up on the current outputs\n"
                "2. CH8 PID test marker ON (BEEPERON in Blackbox)\n"
                "3. Small roll/pitch calibration probes\n"
                "4. CH8 PID test marker OFF (BEEPEROFF in Blackbox)\n\n"
                "Chart Step Response analyzes the marker bracket, not a fixed time window. "
                "You can also bracket your own logs of any length with CH8 markers.\n\n"
                "No PID/FF values will be written while armed.\n\n"
                "Keep the drone secured, keep the area clear, and be ready to disarm."
            )
            if not messagebox.askokcancel("Start Fly/Log", prompt, icon="warning", parent=self.root):
                self.status.set("Fly/Log canceled.")
                return

            deterministic_fly_log_workflow.start()

        def set_sim_attitude_display() -> None:
            self.horizon.set_attitude(self.sim_roll_deg, self.sim_pitch_deg)
            self.roll_text.set(f"Roll: {self.sim_roll_deg:6.1f} deg")
            self.pitch_text.set(f"Pitch: {self.sim_pitch_deg:6.1f} deg")

        def restore_neutral_sim_display() -> None:
            self.horizon.set_attitude(0.0, 0.0)
            self.roll_text.set("Roll: 0.0 deg")
            self.pitch_text.set("Pitch: 0.0 deg")
            clear_pid_ff_displays()

        def set_sim_pid_ff_display(target: dict[str, dict[str, int]]) -> None:
            for label, gain, var in zip(self.pid_ff_labels, ("p", "i", "d", "ff"), self.roll_pidff_vars):
                value = target.get("roll", {}).get(gain)
                var.set(f"{label}: --" if value is None else f"{label}: {value}")
            for label, gain, var in zip(self.pid_ff_labels, ("p", "i", "d", "ff"), self.pitch_pidff_vars):
                value = target.get("pitch", {}).get(gain)
                var.set(f"{label}: --" if value is None else f"{label}: {value}")

        def simulation_hardware_is_disconnected() -> bool:
            if self.start_pending or self.controller.is_connected:
                messagebox.showwarning(
                    "Simulation Requires No Hardware",
                    "Disconnect Arduino output before using the simulator.",
                    parent=self.root,
                )
                return False
            if self.fc_service.is_connected:
                messagebox.showwarning(
                    "Simulation Requires No Hardware",
                    "Disconnect the FC before using the simulator. Simulation uses synthetic attitude and PID boxes only.",
                    parent=self.root,
                )
                return False
            return True

        def _sim_preview_d(plan: LoadedPIDTuningPlan) -> int:
            if len(plan.d_sweep) >= 2:
                return int(plan.d_sweep[1])
            if plan.d_sweep:
                return int(plan.d_sweep[0])
            return 17

        def _sim_preview_pair(rows: tuple[dict[str, int], ...], fallback: dict[str, int]) -> dict[str, int]:
            if not rows:
                return dict(fallback)
            return dict(rows[min(1, len(rows) - 1)])

        def build_simulated_pid_plan_steps(plan: LoadedPIDTuningPlan) -> list[dict[str, object]]:
            start_d = int(plan.d_sweep[0]) if plan.d_sweep else 17
            preview_d = _sim_preview_d(plan)
            preview_p = _sim_preview_pair(tuple({"roll": r["roll"], "pitch": r["pitch"]} for r in pid_plan_p_candidates_for(plan)), plan.start_p)
            preview_i = _sim_preview_pair(plan.i_sweep, {"roll": 60, "pitch": 65})
            preview_ff = _sim_preview_pair(plan.ff_sweep, {"roll": 86, "pitch": 89})
            steps: list[dict[str, object]] = []

            def add(title: str, instruction: str, stage: str, target: dict[str, dict[str, int]], note: str = "") -> None:
                steps.append({"title": title, "instruction": instruction, "stage": stage, "target": target, "note": note})

            add(
                "Safe start / first D log",
                f"Start P with Roll I {plan.start_i['roll']}, Pitch I {plan.start_i['pitch']}, FF = 0, and the first D value.",
                "d",
                roll_pitch_target(
                    plan.start_p["roll"],
                    plan.start_p["pitch"],
                    start_d,
                    start_d,
                    plan.start_i["roll"],
                    plan.start_i["pitch"],
                    0,
                    0,
                ),
            )
            for index, d_value in enumerate(plan.d_sweep[1:], start=2):
                add(
                    f"D sweep {index}/{len(plan.d_sweep)}",
                    f"Compare damping with Roll/Pitch D {d_value}.",
                    "d",
                    roll_pitch_target(
                        plan.start_p["roll"],
                        plan.start_p["pitch"],
                        int(d_value),
                        int(d_value),
                        plan.start_i["roll"],
                        plan.start_i["pitch"],
                        0,
                        0,
                    ),
                )
            if plan.optional_d is not None and plan.optional_d not in plan.d_sweep:
                add(
                    "Optional D sweep",
                    f"Optional comparison at Roll/Pitch D {plan.optional_d}.",
                    "d",
                    roll_pitch_target(
                        plan.start_p["roll"],
                        plan.start_p["pitch"],
                        int(plan.optional_d),
                        int(plan.optional_d),
                        plan.start_i["roll"],
                        plan.start_i["pitch"],
                        0,
                        0,
                    ),
                    "Real tuning should only run this if needed and motors stay cool.",
                )
            for index, row in enumerate(pid_plan_p_candidates_for(plan), start=1):
                add(
                    f"P sweep {index}/{len(pid_plan_p_candidates_for(plan))}",
                    f"Compare tracking with Roll P {row['roll']} and Pitch P {row['pitch']}.",
                    "p",
                    roll_pitch_target(
                        row["roll"],
                        row["pitch"],
                        preview_d,
                        preview_d,
                        plan.start_i["roll"],
                        plan.start_i["pitch"],
                        0,
                        0,
                    ),
                    f"Simulation uses preview D {preview_d}; real tuning uses the D you choose from logs.",
                )
            for index, d_value in enumerate(simulated_d_recheck_values(preview_d), start=1):
                add(
                    f"D re-check {index}/3",
                    f"Re-check damping with chosen P and Roll/Pitch D {d_value}.",
                    "d",
                    roll_pitch_target(
                        preview_p["roll"],
                        preview_p["pitch"],
                        d_value,
                        d_value,
                        plan.start_i["roll"],
                        plan.start_i["pitch"],
                        0,
                        0,
                    ),
                    f"Simulation uses preview P {preview_p['roll']}/{preview_p['pitch']}.",
                )
            for index, row in enumerate(plan.i_sweep, start=1):
                add(
                    f"I sweep {index}/{len(plan.i_sweep)}",
                    f"Compare hold/recenter with Roll I {row['roll']} and Pitch I {row['pitch']}.",
                    "i",
                    roll_pitch_target(preview_p["roll"], preview_p["pitch"], preview_d, preview_d, row["roll"], row["pitch"], 0, 0),
                    "I is subtle in this short visual preview; real logs still decide the winner.",
                )
            for index, row in enumerate(plan.ff_sweep, start=1):
                add(
                    f"FF sweep {index}/{len(plan.ff_sweep)}",
                    f"Compare initial response with Roll FF {row['roll']} and Pitch FF {row['pitch']}.",
                    "ff",
                    roll_pitch_target(
                        preview_p["roll"],
                        preview_p["pitch"],
                        preview_d,
                        preview_d,
                        preview_i["roll"],
                        preview_i["pitch"],
                        row["roll"],
                        row["pitch"],
                    ),
                    f"Simulation uses preview P/D/I {preview_p['roll']}/{preview_p['pitch']} / {preview_d} / {preview_i['roll']}/{preview_i['pitch']}.",
                )
            final_target = roll_pitch_target(
                preview_p["roll"],
                preview_p["pitch"],
                preview_d,
                preview_d,
                preview_i["roll"],
                preview_i["pitch"],
                preview_ff["roll"],
                preview_ff["pitch"],
            )
            add(
                "Final preview",
                "Preview the conservative final roll/pitch set. Yaw is listed in the plan but not shown in these boxes.",
                "final",
                final_target,
                f"Yaw recommendation remains P {plan.yaw_final_pid_ff['p']}, I {plan.yaw_final_pid_ff['i']}, D {plan.yaw_final_pid_ff['d']}, FF {plan.yaw_final_pid_ff['ff']}.",
            )
            return steps

        def pid_plan_p_candidates_for(plan: LoadedPIDTuningPlan) -> tuple[dict[str, int], ...]:
            return tuple(
                {"roll": int(roll), "pitch": int(pitch)}
                for roll, pitch in zip(plan.p_sweep.get("roll", ()), plan.p_sweep.get("pitch", ()))
            )

        def simulated_d_recheck_values(selected_d: int) -> tuple[int, ...]:
            return tuple(dict.fromkeys(max(0, min(255, int(value))) for value in (selected_d - 5, selected_d, selected_d + 5)))

        def current_sim_step() -> dict[str, object] | None:
            if self.sim_plan_step_index < 0 or self.sim_plan_step_index >= len(self.sim_plan_steps):
                return None
            return self.sim_plan_steps[self.sim_plan_step_index]

        def _sim_axis_wave(elapsed_s: float, start_s: float, direction: int, gains: dict[str, int], stage: str) -> float:
            local_s = elapsed_s - start_s
            if local_s < 0.0:
                return 0.0
            p_value = float(gains.get("p", 0))
            d_value = float(gains.get("d", 0))
            i_value = float(gains.get("i", 0))
            ff_value = float(gains.get("ff", 0))
            damping = max(0.0, min(1.0, (d_value - 15.0) / 27.0))
            tracking = max(0.62, min(1.18, 0.78 + ((p_value - 35.0) * 0.012) + (ff_value * 0.0008)))
            target_deg = 18.0 * tracking
            rise_rate = 1.65 + (p_value / 45.0) + min(1.2, ff_value / 120.0)
            overshoot = max(0.02, 0.36 - (damping * 0.27) + max(0.0, p_value - 50.0) * 0.012)
            if stage == "ff":
                overshoot += max(0.0, ff_value - 129.0) * 0.0016
            if stage == "i":
                overshoot += max(0.0, 60.0 - i_value) * 0.002

            if local_s <= 2.35:
                rise = 1.0 - math.exp(-rise_rate * local_s)
                ring = target_deg * overshoot * math.exp(-(1.0 + damping * 2.2) * local_s) * math.sin(local_s * 7.0)
                return float(direction) * (target_deg * rise + ring)

            release_s = local_s - 2.35
            release_rate = 1.15 + (damping * 2.8) + min(0.65, i_value / 180.0)
            residual = target_deg * math.exp(-release_rate * release_s)
            bounce = target_deg * overshoot * 0.55 * math.exp(-(0.8 + damping * 2.0) * release_s) * math.sin(release_s * 8.5)
            return float(direction) * (residual + bounce)

        def _sim_repeating_axis_wave(elapsed_s: float, start_s: float, direction: int, gains: dict[str, int], stage: str) -> float:
            cycle_s = max(1.0, PID_PLAN_FLY_LOG_RUNTIME_S / 4.0)
            if elapsed_s < start_s:
                return 0.0
            cycle_index = int((elapsed_s - start_s) // cycle_s)
            cycle_start_s = start_s + (cycle_index * cycle_s)
            cycle_direction = direction if cycle_index % 2 == 0 else -direction
            return _sim_axis_wave(elapsed_s, cycle_start_s, cycle_direction, gains, stage)

        def update_simulated_plan_attitude(elapsed_s: float, step: dict[str, object]) -> None:
            target = step["target"]
            if not isinstance(target, dict):
                self.sim_roll_deg = 0.0
                self.sim_pitch_deg = 0.0
                return
            stage = str(step.get("stage", ""))
            roll_gains = target.get("roll", {}) if isinstance(target.get("roll", {}), dict) else {}
            pitch_gains = target.get("pitch", {}) if isinstance(target.get("pitch", {}), dict) else {}
            self.sim_roll_deg = _sim_repeating_axis_wave(elapsed_s, 0.25, 1, roll_gains, stage)
            self.sim_pitch_deg = _sim_repeating_axis_wave(elapsed_s, 3.85, -1, pitch_gains, stage)
            limit = 35.0
            self.sim_roll_deg = max(-limit, min(limit, self.sim_roll_deg))
            self.sim_pitch_deg = max(-limit, min(limit, self.sim_pitch_deg))

        def refresh_sim_report(
            elapsed_s: float,
            step: dict[str, object],
            fly_log_running: bool = False,
            step_number: int | None = None,
        ) -> None:
            target = step["target"]
            target_text = format_pid_values(target) if isinstance(target, dict) else ""
            note = str(step.get("note", "") or "")
            mode_text = "Simulated Fly/Log movement running" if fly_log_running else "Simulated values staged"
            display_step_number = self.sim_plan_step_index + 1 if step_number is None else step_number
            lines = [
                "Simulated PID tuning plan step",
                f"Step {display_step_number} of {len(self.sim_plan_steps)}: {step['title']}",
                f"Plan file: {self.sim_plan.text_path if self.sim_plan is not None else '--'}",
                f"State: {mode_text}",
                "",
                str(step["instruction"]),
                "",
                "Hardware is intentionally disconnected for simulation. These values are staged only in the UI PID boxes.",
                "",
                "Real-world sequence for this step:",
                "1. Disarm before saving these PID/FF values.",
                "2. Press Save in the FC / INAV section only while disarmed.",
                "3. Arm, then press Fly/Log for the candidate.",
                "4. Land and disarm before moving to the next plan step.",
                "",
                "Simulated PID/FF boxes",
                target_text,
                "",
                f"Elapsed: {elapsed_s:4.1f}s / {PID_PLAN_FLY_LOG_RUNTIME_S:.1f}s",
                f"Roll:  {self.sim_roll_deg:+5.1f} deg",
                f"Pitch: {self.sim_pitch_deg:+5.1f} deg",
            ]
            if note:
                lines.extend(["", note])
            if fly_log_running:
                lines.extend(["", "When simulated Fly/Log finishes, press Next Sim Step to preview the next tuning candidate."])
            else:
                lines.extend(["", "Press Fly/Log to stimulate the simulated drone for this candidate."])
            set_auto_report_text("\n".join(lines))

        def stop_simulated_auto_session(message: str = "", restore_display: bool = True, clear_walkthrough: bool = False) -> None:
            if self.sim_after_id is not None:
                try:
                    self.root.after_cancel(self.sim_after_id)
                except Exception:
                    pass
            self.sim_active = False
            self.sim_fly_log_active = False
            self.sim_after_id = None
            self.sim_step_started_s = None
            self.sim_last_report_second = -1
            if clear_walkthrough:
                self.sim_plan = None
                self.sim_plan_steps = []
                self.sim_plan_step_index = 0
                self.sim_waiting_for_fly_log = False
            update_link_indicators()
            if restore_display:
                restore_neutral_sim_display()
            if message and not self.is_closing:
                self.status.set(message)

        def finish_simulated_plan_step(step: dict[str, object]) -> None:
            completed_step_number = self.sim_plan_step_index + 1
            self.sim_active = False
            self.sim_fly_log_active = False
            self.sim_waiting_for_fly_log = False
            self.sim_after_id = None
            self.sim_step_started_s = None
            self.sim_plan_step_index += 1
            if self.sim_plan_step_index >= len(self.sim_plan_steps):
                update_link_indicators()
                self.status.set("PID plan simulation complete.")
                refresh_sim_report(PID_PLAN_FLY_LOG_RUNTIME_S, step, fly_log_running=False, step_number=completed_step_number)
                return
            update_link_indicators()
            self.status.set(f"Simulation step complete. Next: {self.sim_plan_steps[self.sim_plan_step_index]['title']}")
            refresh_sim_report(PID_PLAN_FLY_LOG_RUNTIME_S, step, fly_log_running=False, step_number=completed_step_number)

        def run_simulated_auto_tick() -> None:
            self.sim_after_id = None
            if not self.sim_active:
                return
            step = current_sim_step()
            if step is None or self.sim_step_started_s is None:
                stop_simulated_auto_session("Simulation stopped: no plan step is loaded.", clear_walkthrough=True)
                return

            elapsed_s = max(0.0, time.monotonic() - self.sim_step_started_s)
            update_simulated_plan_attitude(elapsed_s, step)
            set_sim_attitude_display()
            report_second = int(elapsed_s)
            if report_second != self.sim_last_report_second:
                self.sim_last_report_second = report_second
                refresh_sim_report(elapsed_s, step, fly_log_running=True)
            if elapsed_s >= PID_PLAN_FLY_LOG_RUNTIME_S:
                finish_simulated_plan_step(step)
                return
            self.sim_after_id = self.root.after(40, run_simulated_auto_tick)

        def start_simulated_plan_step() -> None:
            if not simulation_hardware_is_disconnected():
                return
            if auto_is_running() or self.pid_plan_active or self.blackbox_import_inflight:
                self.status.set("Wait for the active auto/PID/log task to finish before simulating.")
                return
            step = current_sim_step()
            if step is None:
                stop_simulated_auto_session("PID plan simulation complete.", restore_display=True, clear_walkthrough=True)
                return
            target = step["target"]
            if isinstance(target, dict):
                set_sim_pid_ff_display(target)
            self.sim_active = False
            self.sim_fly_log_active = False
            self.sim_waiting_for_fly_log = True
            self.sim_step_started_s = None
            self.sim_roll_deg = 0.0
            self.sim_pitch_deg = 0.0
            self.sim_last_report_second = -1
            update_link_indicators()
            self.status.set(f"Simulated values staged: {step['title']}. Press Fly/Log.")
            refresh_sim_report(0.0, step, fly_log_running=False)
            set_sim_attitude_display()

        def start_simulated_fly_log() -> None:
            if not simulation_hardware_is_disconnected():
                return
            step = current_sim_step()
            if step is None or not self.sim_waiting_for_fly_log:
                self.status.set("No simulated candidate is staged for Fly/Log.")
                return
            self.sim_active = True
            self.sim_fly_log_active = True
            self.sim_step_started_s = time.monotonic()
            self.sim_roll_deg = 0.0
            self.sim_pitch_deg = 0.0
            self.sim_last_report_second = -1
            update_link_indicators()
            self.status.set(f"Simulated Fly/Log running: {step['title']}")
            refresh_sim_report(0.0, step, fly_log_running=True)
            set_sim_attitude_display()
            run_simulated_auto_tick()

        def start_simulated_auto_session() -> None:
            if not simulation_hardware_is_disconnected():
                return
            if self.pid_plan_active:
                raise RuntimeError("Cancel the active guided PID tuning plan before starting simulation.")
            plan_path = locate_pid_tuning_plan_file()
            self.sim_plan = load_pid_tuning_plan(plan_path)
            self.sim_plan_steps = build_simulated_pid_plan_steps(self.sim_plan)
            self.sim_plan_step_index = 0
            if not self.sim_plan_steps:
                raise RuntimeError("PID tuning plan has no steps to simulate.")
            start_simulated_plan_step()

        def do_simulated_auto_session_toggle() -> None:
            try:
                if self.sim_active or self.sim_fly_log_active:
                    self.status.set("Use Cancel Auto Session to stop the simulation.")
                    return
                if self.sim_waiting_for_fly_log:
                    messagebox.showinfo(
                        "Simulated Fly/Log Needed",
                        "The simulated PID/FF values are staged.\n\nPress Fly/Log to stimulate the simulated drone before moving to the next step.",
                        parent=self.root,
                    )
                    self.status.set("Press Fly/Log before the next simulated step.")
                    return
                if self.sim_plan is not None and self.sim_plan_step_index < len(self.sim_plan_steps):
                    start_simulated_plan_step()
                    return
                start_simulated_auto_session()
            except Exception as exc:
                stop_simulated_auto_session("", restore_display=True, clear_walkthrough=True)
                set_error("Simulation error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

        def do_auto_session_toggle() -> None:
            auto_session_workflow.toggle()

        def auto_session_cancel_available() -> bool:
            return auto_session_workflow.cancel_available()

        def do_cancel_auto_session() -> None:
            auto_session_workflow.cancel()

        def on_simulation_mode_changed() -> None:
            try:
                if simulation_mode_enabled():
                    if self.pid_plan_active:
                        self.simulation_mode_var.set(False)
                        messagebox.showwarning(
                            "Simulation Blocked",
                            "Cancel or complete the active guided PID tuning plan before enabling simulation.",
                            parent=self.root,
                        )
                        self.status.set("Simulation requires no active guided PID plan.")
                        update_link_indicators()
                        return
                    if self.start_pending or self.controller.is_connected or self.fc_service.is_connected:
                        self.simulation_mode_var.set(False)
                        messagebox.showwarning(
                            "Simulation Requires No Hardware",
                            "Disconnect Arduino output and FC before enabling simulation mode.",
                            parent=self.root,
                        )
                        self.status.set("Simulation mode requires Arduino and FC disconnected.")
                        update_link_indicators()
                        return
                    self.status.set("Simulation mode enabled. Press Start Auto Session to run the simulator.")
                else:
                    if self.sim_active or self.sim_fly_log_active or self.sim_waiting_for_fly_log or self.sim_plan is not None:
                        stop_simulated_auto_session("Simulation mode disabled.", restore_display=True, clear_walkthrough=True)
                        return
                    self.status.set("Simulation mode disabled.")
                update_link_indicators()
            except Exception as exc:
                self.simulation_mode_var.set(False)
                set_error("Simulation mode error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

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
            simulation_mode_enabled=simulation_mode_enabled,
            do_simulated_auto_session_toggle=do_simulated_auto_session_toggle,
            start_auto_session=start_auto_session,
            open_pid_progress_window=open_pid_progress_window,
            continue_pid_tuning_plan=continue_pid_tuning_plan,
            complete_auto_session=complete_auto_session,
            refresh_fly_log_button_state=refresh_fly_log_button_state,
            complete_pid_tuning_plan=complete_pid_tuning_plan,
            stop_simulated_auto_session=stop_simulated_auto_session,
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
            parse_offsets=lambda: parse_entries(self.off_entries, int, "Offset"),
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
            fc_port=fc_port,
            fc_baud=fc_baud,
            simulation_mode_enabled=simulation_mode_enabled,
            auto_is_running=lambda: auto_is_running(),
            auto_session_cancel_available=lambda: auto_session_cancel_available(),
            refresh_level_button_state=refresh_level_button_state,
            refresh_fly_log_button_state=refresh_fly_log_button_state,
            clear_pid_ff_displays=clear_pid_ff_displays,
            queue_fc_pid_ff_refresh=queue_fc_pid_ff_refresh,
            set_live_channel_outputs=set_live_channel_outputs,
            parse_channel_values_with_defaults=parse_channel_values_with_defaults,
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
            simulation_mode_enabled=simulation_mode_enabled,
            fc_port=fc_port,
            fc_baud=fc_baud,
            ensure_disarmed_before_blackbox_import=ensure_disarmed_before_blackbox_import,
            do_fc_disconnect=do_fc_disconnect,
            set_auto_state=set_auto_state,
            set_auto_button_idle=set_auto_button_idle,
            set_auto_report_text=set_auto_report_text,
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
            begin_fly_log_marker_off_and_complete=begin_fly_log_marker_off_and_complete,
            refresh_fly_log_button_state=refresh_fly_log_button_state,
            open_pid_progress_window=open_pid_progress_window,
            update_pid_progress_window=update_pid_progress_window,
            set_auto_report_text=set_auto_report_text,
        )


        def poll_fc_attitude() -> None:
            try:
                sample = self.fc_service.latest_attitude()
                if sample is not None:
                    record_auto_session_sample(sample)
                    if not self.sim_active:
                        self.horizon.set_attitude(sample.roll_deg, sample.pitch_deg)
                        self.roll_text.set(f"Roll: {sample.roll_deg:6.1f} deg")
                        self.pitch_text.set(f"Pitch: {sample.pitch_deg:6.1f} deg")
            except Exception:
                pass
            self.fc_poll_after_id = self.root.after(60, poll_fc_attitude)

        def poll_results() -> None:
            while True:
                try:
                    cb, ok, res = self.worker.results.get_nowait()
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
            stop_simulated_auto_session("", restore_display=False)
            if self.fc_poll_after_id is not None:
                try:
                    self.root.after_cancel(self.fc_poll_after_id)
                except Exception:
                    pass
                finally:
                    self.fc_poll_after_id = None

            def on_stop_and_close(ok: bool, res: object) -> None:
                do_fc_disconnect(update_status=False)
                try:
                    self.worker.stop()
                except Exception:
                    pass
                self.root.destroy()

            try:
                self.controller.shutdown(callback=on_stop_and_close)
            except Exception:
                on_stop_and_close(False, None)

        scan_fc_ports(update_status=False)

        self.scan_fc_button.config(command=scan_fc_ports)
        self.connect_fc_button.config(command=do_fc_toggle)
        self.import_blackbox_button.config(command=do_pull_blackbox_logs)
        self.analyze_blackbox_button.config(command=do_analyze_blackbox_logs)
        self.auto_session_button.config(command=do_auto_session_toggle)
        self.fly_log_button.config(command=fly_log_workflow.toggle)
        self.simulation_mode_checkbutton.config(command=on_simulation_mode_changed)
        self.pid_progress_button.config(command=open_pid_progress_window)
        self.cancel_auto_session_button.config(command=do_cancel_auto_session)
        self.load_pid_ff_button.config(command=do_load_pid_ff_from_fc)
        self.save_pid_ff_button.config(command=do_save_pid_ff_to_fc)
        self.step_response_button.config(command=do_step_response_report)
        self.pid_tuning_plan_button.config(command=do_pid_tuning_plan)
        self.arduino_button.config(command=do_arduino_toggle)
        self.level_button.config(command=do_level)
        for i, canvas in enumerate(self.channel_adjust_canvases):
            canvas.bind("<ButtonPress-1>", lambda event, i=i: on_adjust_press(adjust_channel_value, i, event))
            canvas.bind("<ButtonRelease-1>", on_adjust_release)
            canvas.bind("<Leave>", on_adjust_release)
        for i, canvas in enumerate(self.pid_ff_adjust_canvases):
            canvas.bind("<ButtonPress-1>", lambda event, i=i: on_adjust_press(adjust_pid_ff_value, i, event, 1))
            canvas.bind("<ButtonRelease-1>", on_adjust_release)
            canvas.bind("<Leave>", on_adjust_release)
        for entry in self.ch_entries:
            entry.bind("<KeyRelease>", lambda _event: on_output_inputs_changed())
            entry.bind("<FocusOut>", lambda _event: on_output_inputs_changed())
        set_auto_state(AdaptiveSessionState.idle)
        set_live_channel_outputs(parse_channel_values_with_defaults())
        update_link_indicators()
        self.root.after(50, poll_results)
        self.fc_poll_after_id = self.root.after(60, poll_fc_attitude)
        self.root.protocol("WM_DELETE_WINDOW", on_close)

        self.root.mainloop()


def main() -> None:
    root = tk.Tk()
    app = ModbusApp(root)
    app.run()

if __name__ == "__main__":
    main()

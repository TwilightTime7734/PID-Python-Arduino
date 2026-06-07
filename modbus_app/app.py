"""Application runtime orchestration."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path
import queue
import time
import tkinter as tk
from collections.abc import Callable, Sequence
from tkinter import filedialog, messagebox, simpledialog

import serial
from serial.tools import list_ports

from serialUSB.inav_serial_service import (
    AxisPidFf,
    FF_SETTING_NAME,
    InavSerialService,
    PID_SETTING_NAME,
    send_cli_msc_command,
)

from .constants import (
    ADJUST_REPEAT_INITIAL_MS,
    ADJUST_REPEAT_INTERVAL_MS,
    BEEPER_MARKER_SPINUP_DELAY_MS,
    CHANNEL_DEFAULTS,
    FC_DEVICE_ID,
    FC_DEVICE_PID,
    FC_DEVICE_VID,
    FC_PORT_DEFAULT,
    LEVEL_CENTER_US,
    LEVEL_DEADBAND_DEG,
    LEVEL_FULL_SCALE_DEG,
    LEVEL_LOOP_INTERVAL_MS,
    LEVEL_MAX_DELTA_US,
    LEVEL_MIN_DELTA_US,
    LEVEL_PULSE_TIMEOUT_S,
    LEVEL_TIMEOUT_DEFAULT_S,
    OFFSET_DEFAULTS,
    PITCH_CHANNEL_INDEX,
    PID_PLAN_FLY_LOG_RUNTIME_S,
    PORT_DEFAULT,
    PULSE_STATUS_REJECTED,
    REG_QUANT,
    ROLL_CHANNEL_INDEX,
    THROTTLE_CHANNEL_INDEX,
)
from .serial_protocol import (
    end_hold_on_serial,
    open_serial,
    read_pulse_status_on_serial,
    read_regs,
    run_ppm_on_serial,
    set_channel_until_stop_on_serial,
    stop_ppm_on_serial,
)
from .adaptive_session import (
    AdaptiveCommand,
    AdaptiveExcitationController,
    AdaptiveSessionConfig,
    AdaptiveSessionState,
    ExcitationEvent,
    axis_channel_index,
)
from .auto_tune_report import AutoTuneReport, generate_auto_tune_report
from .blackbox_import import (
    BlackboxImportResult,
    analyze_blackbox_log,
    analyze_pulled_blackbox_logs,
    import_blackbox_logs_from_msc,
)
from .pid_tuning_workflow import (
    LoadedPIDTuningPlan,
    PAVO_PICO_II_PRESET_INPUTS,
    PStartInputs,
    find_latest_pid_tuning_plan,
    generate_pid_tuning_plan_report,
    load_pid_tuning_plan,
    safe_p_start_information_needed,
    suggest_starting_p,
)
from .step_response_report import (
    MAX_STEP_RESPONSE_LOGS,
    StepResponseReport,
    format_step_response_report,
    generate_step_response_report,
)
from .ui import (
    build_main_gui,
    parse_entries,
    require_range,
)
from .worker import SerialWorker
from .hardware_controller import HardwareController


class ModbusApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.ui = build_main_gui(root)
        self.port_entry = self.ui.port_entry
        self.channel_adjust_canvases = self.ui.channel_adjust_canvases
        self.ch_entries = self.ui.ch_entries
        self.off_entries = self.ui.off_entries
        self.channel_output_canvases = self.ui.channel_output_canvases
        self.channel_output_fill_ids = self.ui.channel_output_fill_ids
        self.level_button = self.ui.level_button
        self.status = self.ui.status
        self.pc_link_box = self.ui.pc_link_box
        self.horizon = self.ui.horizon
        self.roll_text = self.ui.roll_text
        self.pitch_text = self.ui.pitch_text
        self.roll_pidff_vars = self.ui.roll_pidff_vars
        self.pitch_pidff_vars = self.ui.pitch_pidff_vars
        self.pid_ff_adjust_canvases = self.ui.pid_ff_adjust_canvases
        self.fc_port_entry = self.ui.fc_port_entry
        self.fc_baud_entry = self.ui.fc_baud_entry
        self.scan_fc_button = self.ui.scan_fc_button
        self.connect_fc_button = self.ui.connect_fc_button
        self.import_blackbox_button = self.ui.import_blackbox_button
        self.analyze_blackbox_button = self.ui.analyze_blackbox_button
        self.arduino_button = self.ui.arduino_button
        self.auto_session_button = self.ui.auto_session_button
        self.auto_report_text = self.ui.auto_report_text
        self.fly_log_button = self.ui.fly_log_button
        self.simulation_mode_var = self.ui.simulation_mode_var
        self.simulation_mode_checkbutton = self.ui.simulation_mode_checkbutton
        self.pid_progress_button = self.ui.pid_progress_button
        self.step_response_button = self.ui.step_response_button
        self.pid_tuning_plan_button = self.ui.pid_tuning_plan_button

        self.start_pending = False
        self.is_closing = False
        self.controller = HardwareController()
        self.run_active = False
        self.run_port = PORT_DEFAULT
        self.run_ser: serial.Serial | None = None
        self.run_quant: int | None = None
        self.run_max_count: int | None = None
        self.adjust_repeat_after_id: str | None = None
        self.adjust_repeat_handler: Callable[[int, int], None] | None = None
        self.adjust_repeat_index: int | None = None
        self.adjust_repeat_delta = 0
        self.base_channel_outputs = CHANNEL_DEFAULTS.copy()
        self.live_channel_outputs = self.base_channel_outputs.copy()
        self.beeper_marker_active = False
        self.worker = self.controller.worker
        self.fc_service = InavSerialService()
        self.fc_poll_after_id: str | None = None
        self.level_active = False
        self.level_after_id: str | None = None
        self.level_pulse_inflight = False
        self.level_timeout_deadline_s: float | None = None
        self.auto_config = AdaptiveSessionConfig()
        self.auto_controller: AdaptiveExcitationController | None = None
        self.auto_state = AdaptiveSessionState.idle
        self.auto_stop_reason = ""
        self.auto_warning = ""
        self.auto_session_start_s: float | None = None
        self.auto_last_tick_s: float | None = None
        self.auto_last_sample_s: float | None = None
        self.auto_tick_after_id: str | None = None
        self.auto_hold_after_id: str | None = None
        self.fly_log_marker_after_id: str | None = None
        self.auto_pulse_inflight = False
        self.auto_hold_end_requested = False
        self.auto_settle_until_s: float | None = None
        self.auto_recovery_mode = False
        self.auto_active_command: AdaptiveCommand | None = None
        self.auto_event_peak_delta = 0.0
        self.auto_event_response_delay_s: float | None = None
        self.auto_event_baseline = 0.0
        self.auto_event_start_s = 0.0
        self.auto_original_base_outputs: list[int] | None = None
        self.auto_start_throttle_us = self.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        self.auto_current_throttle_us = self.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        self.auto_peak_throttle_us = self.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        self.auto_latest_report: AutoTuneReport | None = None
        self.auto_import_result: BlackboxImportResult | None = None
        self.auto_latest_imported_log: str = ""
        self.sim_active = False
        self.sim_after_id: str | None = None
        self.sim_plan: LoadedPIDTuningPlan | None = None
        self.sim_plan_steps: list[dict[str, object]] = []
        self.sim_plan_step_index = 0
        self.sim_waiting_for_fly_log = False
        self.sim_fly_log_active = False
        self.sim_step_started_s: float | None = None
        self.sim_roll_deg = 0.0
        self.sim_pitch_deg = 0.0
        self.sim_last_report_second = -1
        self.blackbox_import_inflight = False
        self.blackbox_import_dir = (Path(__file__).resolve().parent.parent / "blackbox_imports").resolve()
        self.blackbox_msc_mount_timeout_s = 12.0
        self.blackbox_msc_mount_poll_s = 1.0
        self.requested_pid_plan_path = (
            self.blackbox_import_dir / "reports" / "pid_tuning_plan_20260605_201036" / "pid_tuning_plan.txt"
        )
        self.pid_plan_active = False
        self.pid_plan: LoadedPIDTuningPlan | None = None
        self.pid_plan_phase = "idle"
        self.pid_plan_index = 0
        self.pid_plan_selected_d: int | None = None
        self.pid_plan_selected_p: dict[str, int] | None = None
        self.pid_plan_selected_i: dict[str, int] | None = None
        self.pid_plan_selected_ff: dict[str, int] | None = None
        self.pid_plan_waiting_for_fly_log = False
        self.pid_plan_current_candidate_title = ""
        self.pid_plan_current_candidate_phase = ""
        self.pid_plan_current_candidate_target: dict[str, dict[str, int]] | None = None
        self.pid_plan_fly_log_active = False
        self.pid_progress_window: tk.Toplevel | None = None
        self.pid_progress_phase_labels: dict[str, tk.Label] = {}
        self.pid_progress_current_var = tk.StringVar(value="No PID tuning plan is active.")
        self.pid_progress_action_var = tk.StringVar(value="Generate or start a PID tuning plan.")
        self.pid_progress_selection_var = tk.StringVar(value="")
        self.pid_progress_plan_var = tk.StringVar(value="")
        self.pid_progress_target_text: tk.Text | None = None
        self.pid_ff_labels = ("P", "I", "D", "FF")
        self.pid_ff_adjust_fields = [
            ("roll", "p"),
            ("pitch", "p"),
            ("roll", "i"),
            ("pitch", "i"),
            ("roll", "d"),
            ("pitch", "d"),
            ("roll", "ff"),
            ("pitch", "ff"),
        ]

    @property
    def run_active(self) -> bool:
        return self.controller.run_active

    @run_active.setter
    def run_active(self, value: bool) -> None:
        self.controller.run_active = value

    @property
    def run_port(self) -> str:
        return self.controller.run_port

    @run_port.setter
    def run_port(self, value: str) -> None:
        self.controller.run_port = value

    @property
    def run_ser(self) -> serial.Serial | None:
        return self.controller.run_ser

    @run_ser.setter
    def run_ser(self, value: serial.Serial | None) -> None:
        self.controller.run_ser = value

    @property
    def run_quant(self) -> int | None:
        return self.controller.run_quant

    @run_quant.setter
    def run_quant(self, value: int | None) -> None:
        self.controller.run_quant = value

    @property
    def run_max_count(self) -> int | None:
        return self.controller.run_max_count

    @run_max_count.setter
    def run_max_count(self, value: int | None) -> None:
        self.controller.run_max_count = value

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
            if axis == "roll":
                return float(sample.roll_deg)
            return float(sample.pitch_deg)

        def read_auto_tune_config() -> AdaptiveSessionConfig:
            return AdaptiveSessionConfig()

        def format_pid_ff_value(value: float) -> str:
            rounded = round(value)
            if abs(value - rounded) < 1e-6:
                return str(int(rounded))
            return f"{value:.2f}".rstrip("0").rstrip(".")

        def clear_pid_ff_displays() -> None:
            for label, var in zip(self.pid_ff_labels, self.roll_pidff_vars):
                var.set(f"{label}: --")
            for label, var in zip(self.pid_ff_labels, self.pitch_pidff_vars):
                var.set(f"{label}: --")

        def simulation_mode_enabled() -> bool:
            return bool(self.simulation_mode_var.get())

        def refresh_fly_log_button_state() -> None:
            if self.pid_plan_fly_log_active:
                self.fly_log_button.config(text="Abort Fly/Log", state="normal")
            elif simulation_mode_enabled() and self.sim_fly_log_active:
                self.fly_log_button.config(text="Stop Sim Fly/Log", state="normal")
            elif self.pid_plan_active and self.pid_plan_waiting_for_fly_log:
                self.fly_log_button.config(text="Fly/Log", state="normal")
            elif simulation_mode_enabled() and self.sim_plan is not None and self.sim_waiting_for_fly_log:
                self.fly_log_button.config(text="Fly/Log", state="normal")
            else:
                self.fly_log_button.config(text="Fly/Log", state="disabled")

        def set_pid_ff_displays(roll_values: AxisPidFf, pitch_values: AxisPidFf) -> None:
            roll_series = (roll_values.p, roll_values.i, roll_values.d, roll_values.ff)
            pitch_series = (pitch_values.p, pitch_values.i, pitch_values.d, pitch_values.ff)
            for label, value, var in zip(self.pid_ff_labels, roll_series, self.roll_pidff_vars):
                var.set(f"{label}: {format_pid_ff_value(value)}")
            for label, value, var in zip(self.pid_ff_labels, pitch_series, self.pitch_pidff_vars):
                var.set(f"{label}: {format_pid_ff_value(value)}")

        def set_auto_report_text(text: str) -> None:
            self.auto_report_text.config(state="normal")
            self.auto_report_text.delete("1.0", tk.END)
            self.auto_report_text.insert("1.0", text.strip() + ("\n" if text and not text.endswith("\n") else ""))
            self.auto_report_text.config(state="disabled")

        def auto_elapsed_s(now_s: float | None = None) -> float:
            if self.auto_session_start_s is None:
                return 0.0
            current = time.monotonic() if now_s is None else now_s
            return max(0.0, current - self.auto_session_start_s)

        def set_auto_state(next_state: AdaptiveSessionState, safety_text: str = "") -> None:
            self.auto_state = next_state
            if safety_text and safety_text != "--":
                self.status.set(safety_text)

        def auto_session_payload() -> dict[str, object]:
            metrics: dict[str, object] = {}
            if self.auto_controller is not None:
                snapshot = self.auto_controller.coverage_metrics()
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
                "state": self.auto_state.value,
                "stop_reason": self.auto_stop_reason,
                "warning": self.auto_warning,
                "elapsed_s": auto_elapsed_s(),
                "metrics": metrics,
                "start_throttle_us": self.auto_start_throttle_us,
                "current_throttle_us": self.auto_current_throttle_us,
                "peak_throttle_us": self.auto_peak_throttle_us,
            }

        def refresh_pid_ff_from_fc(update_status: bool = False) -> bool:
            if not self.fc_service.is_connected:
                clear_pid_ff_displays()
                if update_status:
                    self.status.set("FC is disconnected.")
                return False
            try:
                roll_values, pitch_values = self.fc_service.read_roll_pitch_pid_ff(timeout_seconds=1.2)
                set_pid_ff_displays(roll_values, pitch_values)
                if update_status:
                    self.status.set("PID/FF refreshed from FC.")
                return True
            except Exception as exc:
                clear_pid_ff_displays()
                if update_status:
                    set_error("PID/FF read error", exc)
                return False

        def queue_fc_pid_ff_refresh(connected_port: str, connected_baud: int) -> None:
            if not self.fc_service.is_connected:
                return

            def on_pid_ff_read_done(ok: bool, res: object) -> None:
                if not self.fc_service.is_connected:
                    return
                if not ok:
                    clear_pid_ff_displays()
                    self.status.set(f"FC connected: {connected_port} @ {connected_baud}. PID/FF read failed.")
                    return
                if (
                    not isinstance(res, tuple)
                    or len(res) != 2
                    or not isinstance(res[0], AxisPidFf)
                    or not isinstance(res[1], AxisPidFf)
                ):
                    clear_pid_ff_displays()
                    self.status.set(f"FC connected: {connected_port} @ {connected_baud}. PID/FF read failed.")
                    return
                set_pid_ff_displays(res[0], res[1])
                self.status.set(f"FC connected: {connected_port} @ {connected_baud}. PID/FF loaded.")

            self.worker.submit(_task_fc_read_pid_ff, callback=on_pid_ff_read_done)

        def record_auto_session_sample(sample) -> None:
            self.auto_last_sample_s = time.monotonic()
            command = self.auto_active_command
            if command is None:
                return
            if self.auto_controller is not None and self.auto_pulse_inflight:
                stop_limit = max(0.0, self.auto_controller.config.hard_limit_deg - self.auto_controller.config.safety_margin_deg)
                if abs(float(sample.roll_deg)) >= stop_limit or abs(float(sample.pitch_deg)) >= stop_limit:
                    request_auto_angle_hold_end(command)
                    return
            axis_value = pulse_axis_value(sample, command.axis)
            directed_delta = (axis_value - self.auto_event_baseline) * float(command.direction)
            if directed_delta > self.auto_event_peak_delta:
                self.auto_event_peak_delta = float(directed_delta)
            within_hold_window = (time.monotonic() - self.auto_event_start_s) <= command.hold_s
            target_peak_deg = command.target_peak_deg
            if target_peak_deg <= 0 and self.auto_controller is not None:
                target_peak_deg = self.auto_controller.config.axis_target_peak_max_deg(command.axis)
            if (
                self.auto_controller is not None
                and self.auto_pulse_inflight
                and within_hold_window
                and target_peak_deg > 0
                and directed_delta >= target_peak_deg
            ):
                request_auto_angle_hold_end(command)
            if self.auto_event_response_delay_s is None:
                threshold_deg = max(2.0, (command.force_us / 15.0) * 0.35)
                if directed_delta >= threshold_deg:
                    self.auto_event_response_delay_s = max(0.0, time.monotonic() - self.auto_event_start_s)

        def cancel_auto_hold_timer() -> None:
            if self.auto_hold_after_id is not None:
                try:
                    self.root.after_cancel(self.auto_hold_after_id)
                except Exception:
                    pass
                finally:
                    self.auto_hold_after_id = None

        def cancel_fly_log_marker_timer() -> None:
            if self.fly_log_marker_after_id is not None:
                try:
                    self.root.after_cancel(self.fly_log_marker_after_id)
                except Exception:
                    pass
                finally:
                    self.fly_log_marker_after_id = None

        def begin_auto_observe_window(command: AdaptiveCommand) -> None:
            if not auto_is_running():
                return
            self.auto_pulse_inflight = False
            set_live_channel_outputs(self.base_channel_outputs)
            self.auto_settle_until_s = time.monotonic() + command.settle_s
            schedule_auto_tick(delay_ms=round(command.settle_s * 1000.0))

        def request_auto_angle_hold_end(command: AdaptiveCommand) -> None:
            if self.auto_hold_end_requested or not auto_is_running() or not arduino_output_connected():
                return
            self.auto_hold_end_requested = True
            cancel_auto_hold_timer()
            set_live_channel_outputs(self.base_channel_outputs)
            begin_auto_observe_window(command)
            channel_index = axis_channel_index(command.axis)
            target_angle = command.target_peak_deg
            if target_angle <= 0 and self.auto_controller is not None:
                target_angle = self.auto_controller.config.axis_target_peak_max_deg(command.axis)
            def on_auto_hold_end_done(ok: bool, res: object) -> None:
                if not auto_is_running():
                    return
                if not ok:
                    auto_abort(
                        "Unable to end adaptive pulse on angle threshold.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                if not isinstance(res, int):
                    auto_abort("Unexpected hold-end result from worker.")
                    return
                if res == PULSE_STATUS_REJECTED:
                    auto_abort("Firmware rejected adaptive hold-end command.")
                    return
                if arduino_output_connected():
                    queue_live_channel_update(self.base_channel_outputs.copy(), parse_offset_values_with_defaults())

            self.worker.submit(_task_hold_end, channel_index, callback=on_auto_hold_end_done)

        def draw_channel_output(index: int, value: int) -> None:
            clamped = max(1000, min(2000, value))
            canvas = self.channel_output_canvases[index]
            fill_id = self.channel_output_fill_ids[index]

            left = 2.0
            right = 94.0
            center = (left + right) / 2.0
            y1 = 3.0
            y2 = 13.0

            if clamped < 1500:
                ratio = (1500 - clamped) / 500.0
                x = center - (center - left) * ratio
                canvas.coords(fill_id, x, y1, center, y2)
                canvas.itemconfig(fill_id, fill="#E38C8C")
            elif clamped > 1500:
                ratio = (clamped - 1500) / 500.0
                x = center + (right - center) * ratio
                canvas.coords(fill_id, center, y1, x, y2)
                canvas.itemconfig(fill_id, fill="#94D98F")
            else:
                canvas.coords(fill_id, center, y1, center, y2)
                canvas.itemconfig(fill_id, fill="#94D98F")

        def parse_channel_values_with_defaults() -> list[int]:
            values: list[int] = []
            for i, entry in enumerate(self.ch_entries):
                try:
                    values.append(int(entry.get().strip()))
                except ValueError:
                    values.append(CHANNEL_DEFAULTS[i])
            return values

        def parse_offset_values_with_defaults() -> list[int]:
            values: list[int] = []
            for i, entry in enumerate(self.off_entries):
                try:
                    values.append(int(entry.get().strip()))
                except ValueError:
                    values.append(OFFSET_DEFAULTS[i])
            return values

        def adjust_channel_value(index: int, delta: int) -> None:
            try:
                current = int(self.ch_entries[index].get().strip())
            except ValueError:
                current = CHANNEL_DEFAULTS[index]
            updated = max(1000, min(2000, current + delta))
            self.ch_entries[index].delete(0, tk.END)
            self.ch_entries[index].insert(0, str(updated))
            on_output_inputs_changed()

        def get_adjust_delta(event: tk.Event, step: int = 5) -> int:
            width = int(event.widget.cget("width"))
            mid_x = width / 2
            return -step if event.x <= mid_x else step

        def cancel_adjust_repeat() -> None:
            if self.adjust_repeat_after_id is not None:
                try:
                    self.root.after_cancel(self.adjust_repeat_after_id)
                except Exception:
                    pass
                finally:
                    self.adjust_repeat_after_id = None
            self.adjust_repeat_handler = None
            self.adjust_repeat_index = None
            self.adjust_repeat_delta = 0

        def schedule_adjust_repeat() -> None:
            if self.adjust_repeat_handler is None or self.adjust_repeat_index is None or self.adjust_repeat_delta == 0:
                self.adjust_repeat_after_id = None
                return
            self.adjust_repeat_handler(self.adjust_repeat_index, self.adjust_repeat_delta)
            self.adjust_repeat_after_id = self.root.after(ADJUST_REPEAT_INTERVAL_MS, schedule_adjust_repeat)

        def on_adjust_press(
            adjust_handler: Callable[[int, int], None],
            index: int,
            event: tk.Event,
            step: int = 5,
        ) -> None:
            cancel_adjust_repeat()
            delta = get_adjust_delta(event, step=step)
            adjust_handler(index, delta)
            self.adjust_repeat_handler = adjust_handler
            self.adjust_repeat_index = index
            self.adjust_repeat_delta = delta
            self.adjust_repeat_after_id = self.root.after(ADJUST_REPEAT_INITIAL_MS, schedule_adjust_repeat)

        def adjust_pid_ff_value(index: int, delta: int) -> None:
            if index < 0 or index >= len(self.pid_ff_adjust_fields):
                return
            if delta == 0:
                return
            if not self.fc_service.is_connected:
                self.status.set("Connect FC before adjusting PID/FF.")
                return
            axis, gain = self.pid_ff_adjust_fields[index]
            setting_name = FF_SETTING_NAME[axis] if gain == "ff" else PID_SETTING_NAME[(axis, gain)]
            try:
                current = int(self.fc_service.get_setting_int(setting_name, timeout_seconds=0.8))
                target = max(0, min(255, current + delta))
                if target == current:
                    return
                _ = self.fc_service.set_setting_int(setting_name, target, timeout_seconds=0.9)
                if not refresh_pid_ff_from_fc(update_status=False):
                    raise RuntimeError("Failed to refresh PID/FF from FC after update.")
                self.status.set(
                    f"{axis.title()} {gain.upper()} set to {target} on FC."
                )
            except Exception as exc:
                set_error("PID/FF adjust error", exc)

        def on_adjust_release(_event: tk.Event) -> None:
            cancel_adjust_repeat()

        def set_live_channel_outputs(values: list[int]) -> None:
            self.live_channel_outputs = values[: len(self.channel_output_canvases)].copy()
            refresh_channel_outputs()

        def arduino_output_connected() -> bool:
            return self.controller.is_connected

        def restore_base_outputs_after_hold(offsets: list[int] | None = None) -> None:
            if not arduino_output_connected():
                return
            restore_offsets = offsets.copy() if offsets is not None else parse_offset_values_with_defaults()
            set_live_channel_outputs(self.base_channel_outputs)
            queue_live_channel_update(self.base_channel_outputs.copy(), restore_offsets)

        def refresh_channel_outputs() -> None:
            for i, value in enumerate(self.live_channel_outputs[: len(self.channel_output_canvases)]):
                draw_channel_output(i, value)

        def queue_live_channel_update(
            channels: list[int],
            offsets: list[int],
            after_update: Callable[[bool, object], None] | None = None,
        ) -> None:
            if not arduino_output_connected():
                if after_update is not None:
                    after_update(False, RuntimeError("Arduino output is disconnected."))
                return

            def on_live_update_done(ok: bool, res: object) -> None:
                if not ok:
                    set_error("Live update error", res if isinstance(res, Exception) else RuntimeError(res))
                else:
                    if (
                        not isinstance(res, tuple)
                        or len(res) != 3
                        or not isinstance(res[0], int)
                        or not isinstance(res[1], int)
                        or not isinstance(res[2], list)
                    ):
                        set_error("Live update error", RuntimeError("Unexpected worker result from live update task"))
                    else:
                        self.run_quant = res[0]
                        self.run_max_count = res[1]
                        sent_channels = [int(v) for v in res[2]]
                        self.base_channel_outputs = sent_channels
                        set_live_channel_outputs(sent_channels)

                if after_update is not None:
                    try:
                        after_update(ok, res)
                    except Exception as exc:
                        set_error("Live update callback error", exc)

            self.controller.queue_live_channel_update(
                channels.copy(),
                offsets.copy(),
                self.beeper_marker_active,
                callback=on_live_update_done,
            )

        def set_channel_entry_value(index: int, value: int) -> None:
            self.ch_entries[index].delete(0, tk.END)
            self.ch_entries[index].insert(0, str(value))

        def apply_auto_base_outputs(channels: list[int], safety_text: str = "", send_update: bool = True) -> None:
            clamped = [max(1000, min(2000, int(value))) for value in channels[: len(self.ch_entries)]]
            self.base_channel_outputs = clamped.copy()
            if self.auto_original_base_outputs is not None:
                self.auto_current_throttle_us = clamped[THROTTLE_CHANNEL_INDEX]
                self.auto_peak_throttle_us = max(self.auto_peak_throttle_us, self.auto_current_throttle_us)
            for index, value in enumerate(clamped[: len(self.ch_entries)]):
                set_channel_entry_value(index, value)
            set_live_channel_outputs(clamped)
            if send_update and arduino_output_connected():
                queue_live_channel_update(clamped.copy(), parse_offset_values_with_defaults())
            if safety_text:
                self.status.set(safety_text)

        def restore_auto_original_base_outputs() -> None:
            if self.auto_original_base_outputs is None:
                return
            original = self.auto_original_base_outputs
            self.auto_original_base_outputs = None
            apply_auto_base_outputs(original, "restored pre-auto outputs")

        def prepare_auto_throttle(send_update: bool = True) -> bool:
            if self.auto_controller is None:
                return False
            current = self.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
            target, reason = self.auto_controller.initial_throttle(current)
            if target == current:
                return False
            channels = self.base_channel_outputs.copy()
            channels[THROTTLE_CHANNEL_INDEX] = target
            apply_auto_base_outputs(channels, reason, send_update=send_update)
            return True

        def adjust_auto_throttle_after_event(event: ExcitationEvent, recovery_event: bool) -> None:
            if self.auto_controller is None or recovery_event:
                return
            current = self.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
            target, reason = self.auto_controller.throttle_after_event(current, event)
            if target == current:
                return
            channels = self.base_channel_outputs.copy()
            channels[THROTTLE_CHANNEL_INDEX] = target
            apply_auto_base_outputs(channels, reason)

        def on_output_inputs_changed() -> None:
            if not arduino_output_connected():
                set_live_channel_outputs(parse_channel_values_with_defaults())
                return

            try:
                channels = parse_entries(self.ch_entries, int, "Channel")
                require_range(channels, "Channel", 1000, 2000)
                offsets = parse_entries(self.off_entries, int, "Offset")
            except Exception:
                return

            set_live_channel_outputs(channels)
            self.base_channel_outputs = channels.copy()
            queue_live_channel_update(channels, offsets)

        def select_fc_port(port_infos: Sequence[object]) -> str:
            target_id = FC_DEVICE_ID.upper()
            for info in port_infos:
                vid = getattr(info, "vid", None)
                pid = getattr(info, "pid", None)
                if vid == FC_DEVICE_VID and pid == FC_DEVICE_PID:
                    device = str(getattr(info, "device", "") or "").strip()
                    if device:
                        return device
                hwid = str(getattr(info, "hwid", "") or "")
                if target_id in hwid.upper():
                    device = str(getattr(info, "device", "") or "").strip()
                    if device:
                        return device
            return FC_PORT_DEFAULT

        def list_scanned_ports(port_infos: Sequence[object]) -> list[str]:
            ports = [str(getattr(p, "device", "") or "").strip() for p in port_infos]
            return [p for p in ports if p]

        def populate_port_dropdowns(ports: Sequence[str]) -> None:
            values = tuple(ports)
            self.port_entry.config(values=values)
            self.fc_port_entry.config(values=values)

        def scan_fc_ports(update_status: bool = True) -> None:
            port_infos = sorted(
                list_ports.comports(),
                key=lambda p: str(getattr(p, "device", "") or "").upper(),
            )
            ports = list_scanned_ports(port_infos)
            populate_port_dropdowns(ports)
            selected_port = select_fc_port(port_infos)
            self.fc_port_entry.delete(0, tk.END)
            self.fc_port_entry.insert(0, selected_port)
            if update_status:
                if ports:
                    self.status.set(f"Detected ports: {', '.join(ports)}. FC port set to {selected_port}.")
                else:
                    self.status.set(f"No serial ports detected. FC port set to {selected_port}.")

        def format_blackbox_report(result: BlackboxImportResult) -> str:
            lines: list[str] = [result.analysis_summary]
            if result.pid_report is not None:
                if result.pid_report.headline:
                    lines.append(result.pid_report.headline)
                if result.pid_report.highlights:
                    lines.append("PID recommendations:")
                    for line in result.pid_report.highlights[:6]:
                        lines.append(f"- {line}")
                if result.pid_report.cli_commands:
                    lines.append("Suggested CLI settings:")
                    for command in result.pid_report.cli_commands[:10]:
                        lines.append(f"  {command}")
                if result.pid_report.advisory:
                    lines.append("Notes:")
                    for note in result.pid_report.advisory[:4]:
                        lines.append(f"- {note}")
            if result.analysis_source:
                lines.append(f"Source: {result.analysis_source}")
            if result.scanned_roots:
                lines.append("Scanned: " + ", ".join(result.scanned_roots))
            if result.warnings:
                lines.append("Warnings:")
                for warning in result.warnings[:3]:
                    lines.append(f"- {warning}")
            return "\n".join(lines)

        def parse_optional_float_input(value: str, label: str) -> float | None:
            text = value.strip()
            if not text:
                return None
            try:
                parsed = float(text)
            except ValueError as exc:
                raise RuntimeError(f"{label} must be a number or blank.") from exc
            if parsed <= 0:
                raise RuntimeError(f"{label} must be greater than zero or blank.")
            return parsed

        def parse_optional_int_input(value: str, label: str) -> int | None:
            text = value.strip()
            if not text:
                return None
            try:
                parsed = int(text)
            except ValueError as exc:
                raise RuntimeError(f"{label} must be an integer or blank.") from exc
            if parsed <= 0:
                raise RuntimeError(f"{label} must be greater than zero or blank.")
            return parsed

        def ask_pid_tuning_inputs() -> PStartInputs | None:
            dialog = tk.Toplevel(self.root)
            dialog.title("PID Tuning Plan")
            dialog.transient(self.root)
            dialog.resizable(False, False)
            dialog.grab_set()

            result: dict[str, PStartInputs | None] = {"value": None}
            body = tk.Frame(dialog, padx=12, pady=10)
            body.grid(row=0, column=0, sticky="nsew")
            body.grid_columnconfigure(1, weight=1)

            needed = "\n".join(f"- {item}" for item in safe_p_start_information_needed())
            tk.Label(
                body,
                text=(
                    "The plan estimates a first safe P from build specs, tunes roll/pitch only, "
                    "and gives yaw a conservative final PID/FF value without testing yaw.\n\n"
                    f"Useful inputs:\n{needed}"
                ),
                justify="left",
                wraplength=560,
            ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 10))

            auw_var = tk.StringVar()
            motor_count_var = tk.StringVar(value="4")
            motor_kv_var = tk.StringVar()
            battery_cells_var = tk.StringVar()
            prop_var = tk.StringVar()
            pitch_var = tk.StringVar()
            chemistry_var = tk.StringVar(value="LiPo")
            chemistry_options = {"LiPo": "lipo", "LiHV": "lihv", "Li-ion": "liion"}
            chemistry_labels = {value: label for label, value in chemistry_options.items()}
            pavo_pico_ii_var = tk.BooleanVar(value=False)

            def apply_pavo_pico_ii_preset() -> None:
                if not pavo_pico_ii_var.get():
                    return
                preset = PAVO_PICO_II_PRESET_INPUTS
                auw_var.set("" if preset.all_up_weight_g is None else str(preset.all_up_weight_g))
                motor_count_var.set(str(preset.motor_count))
                motor_kv_var.set("" if preset.motor_kv is None else str(preset.motor_kv))
                battery_cells_var.set("" if preset.battery_cells is None else str(preset.battery_cells))
                prop_var.set("" if preset.prop_diameter_in is None else f"{preset.prop_diameter_in:g}")
                pitch_var.set("" if preset.prop_pitch_in is None else f"{preset.prop_pitch_in:g}")
                chemistry_var.set(chemistry_labels.get(preset.battery_chemistry, "LiPo"))

            tk.Label(body, text="AUW grams").grid(row=1, column=0, sticky="e", padx=(0, 6), pady=2)
            tk.Entry(body, width=10, textvariable=auw_var).grid(row=1, column=1, sticky="w", pady=2)
            tk.Label(body, text="Motors").grid(row=1, column=2, sticky="e", padx=(8, 6), pady=2)
            tk.Entry(body, width=10, textvariable=motor_count_var).grid(row=1, column=3, sticky="w", pady=2)

            tk.Label(body, text="Motor KV").grid(row=2, column=0, sticky="e", padx=(0, 6), pady=2)
            tk.Entry(body, width=10, textvariable=motor_kv_var).grid(row=2, column=1, sticky="w", pady=2)
            tk.Label(body, text="Battery S").grid(row=2, column=2, sticky="e", padx=(8, 6), pady=2)
            tk.Entry(body, width=10, textvariable=battery_cells_var).grid(row=2, column=3, sticky="w", pady=2)

            tk.Label(body, text="Prop dia (in)").grid(row=3, column=0, sticky="e", padx=(0, 6), pady=2)
            tk.Entry(body, width=10, textvariable=prop_var).grid(row=3, column=1, sticky="w", pady=2)
            tk.Label(body, text="Prop pitch (in)").grid(row=3, column=2, sticky="e", padx=(8, 6), pady=2)
            tk.Entry(body, width=10, textvariable=pitch_var).grid(row=3, column=3, sticky="w", pady=2)

            tk.Label(body, text="Chemistry").grid(row=4, column=0, sticky="e", padx=(0, 6), pady=2)
            chemistry_menu = tk.OptionMenu(body, chemistry_var, *chemistry_options.keys())
            chemistry_menu.config(width=10)
            chemistry_menu.grid(row=4, column=1, sticky="w", pady=2)
            tk.Checkbutton(
                body,
                text="Pavo Pico 2",
                variable=pavo_pico_ii_var,
                command=apply_pavo_pico_ii_preset,
            ).grid(row=4, column=2, sticky="w", padx=(8, 6), pady=2)
            tk.Label(
                body,
                text="BETAFPV O4 + LAVA II 580mAh",
                fg="#374151",
            ).grid(row=4, column=3, sticky="w", pady=2)

            tk.Label(
                body,
                text="Blank fields keep the instruction baselines. Motor count defaults to 4.",
                justify="left",
                wraplength=560,
                fg="#374151",
            ).grid(row=5, column=0, columnspan=4, sticky="w", pady=(8, 0))

            buttons = tk.Frame(body)
            buttons.grid(row=6, column=0, columnspan=4, sticky="e", pady=(10, 0))

            def on_cancel() -> None:
                result["value"] = None
                dialog.destroy()

            def on_ok() -> None:
                try:
                    motor_count = parse_optional_int_input(motor_count_var.get(), "Motors")
                    result["value"] = PStartInputs(
                        all_up_weight_g=parse_optional_int_input(auw_var.get(), "AUW grams"),
                        motor_kv=parse_optional_int_input(motor_kv_var.get(), "Motor KV"),
                        prop_diameter_in=parse_optional_float_input(prop_var.get(), "Prop inches"),
                        prop_pitch_in=parse_optional_float_input(pitch_var.get(), "Prop pitch"),
                        battery_cells=parse_optional_int_input(battery_cells_var.get(), "Battery S"),
                        battery_chemistry=chemistry_options[chemistry_var.get()],
                        motor_count=4 if motor_count is None else motor_count,
                    )
                except Exception as exc:
                    messagebox.showerror("PID tuning input", str(exc), parent=dialog)
                    return
                dialog.destroy()

            tk.Button(buttons, text="Cancel", width=10, command=on_cancel).pack(side="right", padx=(6, 0))
            tk.Button(buttons, text="Generate Plan", width=14, command=on_ok).pack(side="right")
            dialog.protocol("WM_DELETE_WINDOW", on_cancel)
            dialog.wait_window()
            return result["value"]

        def do_pid_tuning_plan() -> None:
            try:
                if self.blackbox_import_inflight:
                    self.status.set("Blackbox/report task already in progress.")
                    return
                if auto_is_running():
                    self.status.set("Wait for the auto session/pipeline to finish first.")
                    return

                inputs = ask_pid_tuning_inputs()
                if inputs is None:
                    self.status.set("PID tuning plan canceled.")
                    return

                recommendation = suggest_starting_p(inputs)
                report = generate_pid_tuning_plan_report(self.blackbox_import_dir, recommendation)
                set_auto_report_text(Path(report.text_path).read_text(encoding="utf-8", errors="replace"))
                self.status.set(f"PID tuning plan generated: {report.report_dir}")
            except Exception as exc:
                set_error("PID tuning plan error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

        def locate_pid_tuning_plan_file() -> Path:
            if self.requested_pid_plan_path.exists():
                return self.requested_pid_plan_path
            latest = find_latest_pid_tuning_plan(self.blackbox_import_dir)
            if latest is not None:
                return latest
            raise RuntimeError(
                "No PID tuning plan file was found. Generate a PID Tuning Plan first."
            )

        def read_fc_pid_ff_values(axes: tuple[str, ...] = ("roll", "pitch", "yaw")) -> dict[str, dict[str, int]]:
            values: dict[str, dict[str, int]] = {}
            for axis in axes:
                values[axis] = {
                    "p": int(self.fc_service.get_setting_int(PID_SETTING_NAME[(axis, "p")], timeout_seconds=1.0)),
                    "i": int(self.fc_service.get_setting_int(PID_SETTING_NAME[(axis, "i")], timeout_seconds=1.0)),
                    "d": int(self.fc_service.get_setting_int(PID_SETTING_NAME[(axis, "d")], timeout_seconds=1.0)),
                    "ff": int(self.fc_service.get_setting_int(FF_SETTING_NAME[axis], timeout_seconds=1.0)),
                }
            return values

        def format_pid_values(values: dict[str, dict[str, int]]) -> str:
            lines: list[str] = []
            for axis in ("roll", "pitch", "yaw"):
                gains = values.get(axis)
                if not gains:
                    continue
                lines.append(
                    f"{axis.title():5} P {gains['p']:3d}, I {gains['i']:3d}, "
                    f"D {gains['d']:3d}, FF {gains['ff']:3d}"
                )
            return "\n".join(lines)

        def format_pid_target_check(
            current: dict[str, dict[str, int]],
            target: dict[str, dict[str, int]],
        ) -> str:
            lines: list[str] = []
            for axis in ("roll", "pitch", "yaw"):
                if axis not in target:
                    continue
                parts: list[str] = []
                for gain in ("p", "i", "d", "ff"):
                    target_value = int(target[axis][gain])
                    current_value = current.get(axis, {}).get(gain)
                    if current_value == target_value:
                        parts.append(f"{gain.upper()} {target_value} OK")
                    else:
                        parts.append(f"{gain.upper()} {current_value} -> {target_value}")
                lines.append(f"{axis.title()}: " + ", ".join(parts))
            return "\n".join(lines)

        def set_pid_plan_report_text(
            plan: LoadedPIDTuningPlan,
            title: str,
            target: dict[str, dict[str, int]] | None = None,
            current: dict[str, dict[str, int]] | None = None,
        ) -> None:
            lines = [
                title,
                f"Plan file: {plan.text_path}",
            ]
            if current:
                lines.extend(["", "Current FC PID/FF", format_pid_values(current)])
            if target:
                lines.extend(["", "Target for this step", format_pid_values(target)])
            lines.extend(["", plan.text])
            set_auto_report_text("\n".join(lines))

        PID_PROGRESS_PHASES = (
            ("safe_start", "Safe Start"),
            ("d_sweep", "D Sweep"),
            ("p_sweep", "P Sweep"),
            ("d_recheck", "D Re-check"),
            ("i_sweep", "I Sweep"),
            ("ff_sweep", "FF Sweep"),
            ("final_write", "Final"),
        )
        PID_PROGRESS_PHASE_INDEX = {phase: index for index, (phase, _label) in enumerate(PID_PROGRESS_PHASES)}

        def normalize_pid_progress_phase(phase: str) -> str:
            if phase == "d_optional":
                return "d_sweep"
            return phase

        def pid_progress_active_phase() -> str:
            if self.pid_plan_phase == "complete":
                return "complete"
            if self.pid_plan_waiting_for_fly_log or self.pid_plan_fly_log_active:
                return normalize_pid_progress_phase(self.pid_plan_current_candidate_phase or self.pid_plan_phase)
            return normalize_pid_progress_phase(self.pid_plan_phase)

        def pid_progress_target() -> dict[str, dict[str, int]] | None:
            if self.pid_plan_waiting_for_fly_log or self.pid_plan_fly_log_active:
                return self.pid_plan_current_candidate_target
            if self.pid_plan is None or self.pid_plan_phase in ("idle", "complete"):
                return None
            try:
                step = current_pid_plan_step()
            except Exception:
                return None
            if step is None:
                return None
            return step[2]

        def pid_progress_title() -> str:
            if self.pid_plan is None:
                return "No PID tuning plan is active."
            if self.pid_plan_phase == "complete":
                return "PID tuning plan complete."
            if self.pid_plan_waiting_for_fly_log or self.pid_plan_fly_log_active:
                return self.pid_plan_current_candidate_title or "Current candidate"
            if self.pid_plan_phase == "final_write":
                return "Final values"
            try:
                step = current_pid_plan_step()
            except Exception:
                return "PID tuning plan"
            if step is None:
                return "Choose the next winner or stage."
            return step[0]

        def pid_progress_action() -> str:
            if self.pid_plan is None:
                return "Start Auto Session to begin the guided PID plan."
            if self.pid_plan_phase == "complete":
                return "Review final values and save in INAV only when you are satisfied."
            if self.pid_plan_fly_log_active:
                return "Fly/Log movement is active. Keep the drone controlled, then land and disarm."
            if self.pid_plan_waiting_for_fly_log:
                return "Arm, press Fly/Log, land, disarm, review the log, then press Next PID Plan Step."
            if self.pid_plan_phase == "final_write":
                return "Choose final values. The app will verify disarmed state before writing."
            return "Press Next PID Plan Step and follow the prompt. Write/check values only while disarmed."

        def format_pid_progress_selection() -> str:
            selected_p = (
                "--"
                if self.pid_plan_selected_p is None
                else f"Roll {self.pid_plan_selected_p['roll']} / Pitch {self.pid_plan_selected_p['pitch']}"
            )
            selected_i = (
                "--"
                if self.pid_plan_selected_i is None
                else f"Roll {self.pid_plan_selected_i['roll']} / Pitch {self.pid_plan_selected_i['pitch']}"
            )
            selected_ff = (
                "--"
                if self.pid_plan_selected_ff is None
                else f"Roll {self.pid_plan_selected_ff['roll']} / Pitch {self.pid_plan_selected_ff['pitch']}"
            )
            return (
                f"Chosen D: {'--' if self.pid_plan_selected_d is None else self.pid_plan_selected_d}\n"
                f"Chosen P: {selected_p}\n"
                f"Chosen I: {selected_i}\n"
                f"Chosen FF: {selected_ff}"
            )

        def set_pid_progress_target_text(text: str) -> None:
            if self.pid_progress_target_text is None:
                return
            self.pid_progress_target_text.config(state="normal")
            self.pid_progress_target_text.delete("1.0", tk.END)
            self.pid_progress_target_text.insert("1.0", text)
            self.pid_progress_target_text.config(state="disabled")

        def update_pid_progress_window() -> None:
            if self.pid_progress_window is None or not self.pid_progress_window.winfo_exists():
                return

            active_phase = pid_progress_active_phase()
            active_index = PID_PROGRESS_PHASE_INDEX.get(active_phase)
            for phase, label_text in PID_PROGRESS_PHASES:
                label = self.pid_progress_phase_labels.get(phase)
                if label is None:
                    continue
                phase_index = PID_PROGRESS_PHASE_INDEX[phase]
                if self.pid_plan_phase == "complete":
                    state_text = "Done"
                    bg = "#DDEFE1"
                    fg = "#153B1A"
                elif phase == active_phase:
                    state_text = "Active"
                    bg = "#CFE8FF"
                    fg = "#12385D"
                elif active_index is not None and phase_index < active_index:
                    state_text = "Done"
                    bg = "#E3E8EF"
                    fg = "#26313D"
                else:
                    state_text = "Pending"
                    bg = "#F3F4F6"
                    fg = "#374151"
                label.config(text=f"{label_text}\n{state_text}", bg=bg, fg=fg)

            self.pid_progress_current_var.set(pid_progress_title())
            self.pid_progress_action_var.set(pid_progress_action())
            self.pid_progress_selection_var.set(format_pid_progress_selection())
            self.pid_progress_plan_var.set("" if self.pid_plan is None else f"Plan file: {self.pid_plan.text_path}")

            target = pid_progress_target()
            target_text = "No target values are staged yet."
            if target:
                target_text = format_pid_values(target)
            set_pid_progress_target_text(target_text)

        def close_pid_progress_window() -> None:
            if self.pid_progress_window is not None:
                try:
                    self.pid_progress_window.destroy()
                except Exception:
                    pass
            self.pid_progress_window = None
            self.pid_progress_target_text = None
            self.pid_progress_phase_labels.clear()

        def open_pid_progress_window() -> None:
            if self.pid_progress_window is not None and self.pid_progress_window.winfo_exists():
                self.pid_progress_window.lift()
                update_pid_progress_window()
                return

            window = tk.Toplevel(self.root)
            window.withdraw()
            try:
                window.title("PID Tuning Progress")
                window.resizable(False, False)
                window.grid_rowconfigure(0, weight=1)
                window.grid_columnconfigure(0, weight=1)
                window.protocol("WM_DELETE_WINDOW", close_pid_progress_window)

                outer = tk.Frame(window, padx=10, pady=10)
                outer.grid(row=0, column=0, sticky="nsew")
                outer.grid_columnconfigure(0, weight=1)

                phase_frame = tk.LabelFrame(outer, text="Flow", padx=6, pady=6)
                phase_frame.grid(row=0, column=0, sticky="we")
                for column, (phase, label_text) in enumerate(PID_PROGRESS_PHASES):
                    phase_frame.grid_columnconfigure(column, weight=1)
                    label = tk.Label(
                        phase_frame,
                        text=f"{label_text}\nPending",
                        width=12,
                        height=2,
                        relief="groove",
                        bd=1,
                        justify="center",
                        bg="#F3F4F6",
                    )
                    label.grid(row=0, column=column, padx=2, sticky="we")
                    self.pid_progress_phase_labels[phase] = label

                current_frame = tk.LabelFrame(outer, text="Current Step", padx=8, pady=8)
                current_frame.grid(row=1, column=0, sticky="we", pady=(8, 0))
                current_frame.grid_columnconfigure(0, weight=1)
                tk.Label(
                    current_frame,
                    textvariable=self.pid_progress_current_var,
                    anchor="w",
                    justify="left",
                    font=("Segoe UI", 10, "bold"),
                    width=82,
                    wraplength=680,
                ).grid(row=0, column=0, sticky="w")
                tk.Label(
                    current_frame,
                    textvariable=self.pid_progress_action_var,
                    anchor="w",
                    justify="left",
                    width=82,
                    wraplength=680,
                ).grid(row=1, column=0, sticky="w", pady=(6, 0))
                tk.Label(
                    current_frame,
                    textvariable=self.pid_progress_plan_var,
                    anchor="w",
                    justify="left",
                    width=82,
                    wraplength=680,
                    fg="#374151",
                ).grid(row=2, column=0, sticky="w", pady=(6, 0))

                target_frame = tk.LabelFrame(outer, text="Target Values", padx=8, pady=8)
                target_frame.grid(row=2, column=0, sticky="we", pady=(8, 0))
                target_frame.grid_columnconfigure(0, weight=1)
                self.pid_progress_target_text = tk.Text(target_frame, width=82, height=5, wrap="none")
                self.pid_progress_target_text.grid(row=0, column=0, sticky="we")
                self.pid_progress_target_text.config(state="disabled")

                selection_frame = tk.LabelFrame(outer, text="Selected Winners", padx=8, pady=8)
                selection_frame.grid(row=3, column=0, sticky="we", pady=(8, 0))
                tk.Label(
                    selection_frame,
                    textvariable=self.pid_progress_selection_var,
                    anchor="w",
                    justify="left",
                    width=82,
                ).grid(row=0, column=0, sticky="w")

                buttons = tk.Frame(outer)
                buttons.grid(row=4, column=0, sticky="e", pady=(8, 0))
                tk.Button(buttons, text="Refresh", width=10, command=update_pid_progress_window).pack(side="right")
                tk.Button(buttons, text="Close", width=10, command=close_pid_progress_window).pack(
                    side="right", padx=(0, 6)
                )

                self.pid_progress_window = window
                update_pid_progress_window()
                window.deiconify()
                window.lift()
                window.update_idletasks()
            except Exception as exc:
                self.pid_progress_window = None
                self.pid_progress_target_text = None
                self.pid_progress_phase_labels.clear()
                try:
                    window.destroy()
                except Exception:
                    pass
                set_error("PID progress error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

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

        def write_fc_pid_ff_values(target: dict[str, dict[str, int]]) -> None:
            for axis in ("roll", "pitch", "yaw"):
                gains = target.get(axis)
                if not gains:
                    continue
                for gain in ("p", "i", "d", "ff"):
                    value = int(gains[gain])
                    if value < 0 or value > 255:
                        raise RuntimeError(f"{axis.title()} {gain.upper()} target {value} is outside 0-255.")
                    setting_name = FF_SETTING_NAME[axis] if gain == "ff" else PID_SETTING_NAME[(axis, gain)]
                    confirmed = int(self.fc_service.set_setting_int(setting_name, value, timeout_seconds=1.2))
                    if confirmed != value:
                        raise RuntimeError(
                            f"{axis.title()} {gain.upper()} write verified as {confirmed}, expected {value}."
                        )

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
            return {
                "roll": {"p": int(roll_p), "i": int(roll_i), "d": int(roll_d), "ff": int(roll_ff)},
                "pitch": {"p": int(pitch_p), "i": int(pitch_i), "d": int(pitch_d), "ff": int(pitch_ff)},
            }

        def ask_pid_value(title: str, prompt: str, initial: int) -> int | None:
            return simpledialog.askinteger(
                title,
                prompt,
                initialvalue=int(initial),
                minvalue=0,
                maxvalue=255,
                parent=self.root,
            )

        def ask_pid_pair(title: str, gain: str, initial_roll: int, initial_pitch: int) -> dict[str, int] | None:
            roll_value = ask_pid_value(title, f"Enter chosen Roll {gain.upper()} value.", initial_roll)
            if roll_value is None:
                return None
            pitch_value = ask_pid_value(title, f"Enter chosen Pitch {gain.upper()} value.", initial_pitch)
            if pitch_value is None:
                return None
            return {"roll": int(roll_value), "pitch": int(pitch_value)}

        def pid_plan_d_candidates() -> tuple[int, ...]:
            if self.pid_plan is None:
                return ()
            if len(self.pid_plan.d_sweep) <= 1:
                return ()
            return tuple(int(value) for value in self.pid_plan.d_sweep[1:])

        def pid_plan_p_candidates() -> tuple[dict[str, int], ...]:
            if self.pid_plan is None:
                return ()
            return tuple(
                {"roll": int(roll), "pitch": int(pitch)}
                for roll, pitch in zip(self.pid_plan.p_sweep.get("roll", ()), self.pid_plan.p_sweep.get("pitch", ()))
            )

        def pid_plan_d_recheck_candidates() -> tuple[int, ...]:
            if self.pid_plan_selected_d is None:
                return ()
            delta = 5
            values = (self.pid_plan_selected_d - delta, self.pid_plan_selected_d, self.pid_plan_selected_d + delta)
            return tuple(dict.fromkeys(max(0, min(255, int(value))) for value in values))

        def complete_pid_tuning_plan(message: str) -> None:
            self.pid_plan_active = False
            self.pid_plan_phase = "complete"
            self.pid_plan_index = 0
            self.pid_plan_waiting_for_fly_log = False
            self.pid_plan_current_candidate_title = ""
            self.pid_plan_current_candidate_phase = ""
            self.pid_plan_current_candidate_target = None
            set_auto_button_idle()
            refresh_fly_log_button_state()
            self.status.set(message)
            update_pid_progress_window()

        def prepare_pid_plan_next_step() -> bool:
            if self.pid_plan is None:
                raise RuntimeError("PID tuning plan is not loaded.")

            while True:
                if self.pid_plan_phase == "safe_start":
                    return True

                if self.pid_plan_phase == "d_sweep" and self.pid_plan_index >= len(pid_plan_d_candidates()):
                    optional_d = self.pid_plan.optional_d
                    if optional_d is not None and optional_d not in self.pid_plan.d_sweep:
                        if messagebox.askyesno(
                            "Optional D Step",
                            f"The normal D sweep is complete.\n\nRun optional D {optional_d} before choosing D?",
                            parent=self.root,
                        ):
                            self.pid_plan_phase = "d_optional"
                            self.pid_plan_index = 0
                            return True
                    chosen = ask_pid_value("Choose D", "Enter the best Roll/Pitch D from the D sweep.", self.pid_plan.d_sweep[0])
                    if chosen is None:
                        self.status.set("PID plan paused; D winner is required before P sweep.")
                        return False
                    self.pid_plan_selected_d = int(chosen)
                    self.pid_plan_phase = "p_sweep"
                    self.pid_plan_index = 0
                    update_pid_progress_window()
                    continue

                if self.pid_plan_phase == "d_optional" and self.pid_plan_index >= 1:
                    initial = self.pid_plan.optional_d if self.pid_plan.optional_d is not None else self.pid_plan.d_sweep[0]
                    chosen = ask_pid_value("Choose D", "Enter the best Roll/Pitch D from the D sweep.", int(initial))
                    if chosen is None:
                        self.status.set("PID plan paused; D winner is required before P sweep.")
                        return False
                    self.pid_plan_selected_d = int(chosen)
                    self.pid_plan_phase = "p_sweep"
                    self.pid_plan_index = 0
                    update_pid_progress_window()
                    continue

                if self.pid_plan_phase == "p_sweep" and self.pid_plan_index >= len(pid_plan_p_candidates()):
                    candidates = pid_plan_p_candidates()
                    initial = candidates[-1] if candidates else self.pid_plan.start_p
                    selected = ask_pid_pair("Choose P", "P", initial["roll"], initial["pitch"])
                    if selected is None:
                        self.status.set("PID plan paused; P winners are required before D re-check.")
                        return False
                    self.pid_plan_selected_p = selected
                    self.pid_plan_phase = "d_recheck"
                    self.pid_plan_index = 0
                    update_pid_progress_window()
                    continue

                if self.pid_plan_phase == "d_recheck" and self.pid_plan_index >= len(pid_plan_d_recheck_candidates()):
                    initial = self.pid_plan_selected_d if self.pid_plan_selected_d is not None else self.pid_plan.d_sweep[0]
                    chosen = ask_pid_value("Choose Final D", "Enter the best Roll/Pitch D after re-check.", int(initial))
                    if chosen is None:
                        self.status.set("PID plan paused; final D is required before I sweep.")
                        return False
                    self.pid_plan_selected_d = int(chosen)
                    self.pid_plan_phase = "i_sweep"
                    self.pid_plan_index = 0
                    update_pid_progress_window()
                    continue

                if self.pid_plan_phase == "i_sweep" and self.pid_plan_index >= len(self.pid_plan.i_sweep):
                    initial = self.pid_plan.i_sweep[-1] if self.pid_plan.i_sweep else {"roll": 60, "pitch": 65}
                    selected = ask_pid_pair("Choose I", "I", initial["roll"], initial["pitch"])
                    if selected is None:
                        self.status.set("PID plan paused; I winners are required before FF sweep.")
                        return False
                    self.pid_plan_selected_i = selected
                    self.pid_plan_phase = "ff_sweep"
                    self.pid_plan_index = 0
                    update_pid_progress_window()
                    continue

                if self.pid_plan_phase == "ff_sweep" and self.pid_plan_index >= len(self.pid_plan.ff_sweep):
                    initial = self.pid_plan.ff_sweep[-1] if self.pid_plan.ff_sweep else {"roll": 86, "pitch": 89}
                    selected = ask_pid_pair("Choose FF", "FF", initial["roll"], initial["pitch"])
                    if selected is None:
                        self.status.set("PID plan paused; FF winners are required before final write.")
                        return False
                    self.pid_plan_selected_ff = selected
                    self.pid_plan_phase = "final_write"
                    self.pid_plan_index = 0
                    update_pid_progress_window()
                    continue

                return True

        def current_pid_plan_step() -> tuple[str, str, dict[str, dict[str, int]]] | None:
            if self.pid_plan is None:
                raise RuntimeError("PID tuning plan is not loaded.")
            start_d = int(self.pid_plan.d_sweep[0]) if self.pid_plan.d_sweep else 17

            if self.pid_plan_phase == "safe_start":
                target = roll_pitch_target(
                    self.pid_plan.start_p["roll"],
                    self.pid_plan.start_p["pitch"],
                    start_d,
                    start_d,
                    0,
                    0,
                    0,
                    0,
                )
                return (
                    "Safe start / first D log",
                    "This writes the safe starting P values with I = 0, FF = 0, and the first D value.",
                    target,
                )

            if self.pid_plan_phase == "d_sweep":
                candidates = pid_plan_d_candidates()
                if self.pid_plan_index >= len(candidates):
                    return None
                d_value = candidates[self.pid_plan_index]
                target = roll_pitch_target(
                    self.pid_plan.start_p["roll"],
                    self.pid_plan.start_p["pitch"],
                    d_value,
                    d_value,
                    0,
                    0,
                    0,
                    0,
                )
                return (f"D sweep {self.pid_plan_index + 2}/{len(self.pid_plan.d_sweep)}", f"Log Roll/Pitch D {d_value}.", target)

            if self.pid_plan_phase == "d_optional":
                if self.pid_plan.optional_d is None or self.pid_plan_index >= 1:
                    return None
                d_value = int(self.pid_plan.optional_d)
                target = roll_pitch_target(
                    self.pid_plan.start_p["roll"],
                    self.pid_plan.start_p["pitch"],
                    d_value,
                    d_value,
                    0,
                    0,
                    0,
                    0,
                )
                return ("Optional D sweep", f"Log optional Roll/Pitch D {d_value}.", target)

            if self.pid_plan_phase == "p_sweep":
                if self.pid_plan_selected_d is None:
                    return None
                candidates = pid_plan_p_candidates()
                if self.pid_plan_index >= len(candidates):
                    return None
                row = candidates[self.pid_plan_index]
                target = roll_pitch_target(row["roll"], row["pitch"], self.pid_plan_selected_d, self.pid_plan_selected_d, 0, 0, 0, 0)
                return (
                    f"P sweep {self.pid_plan_index + 1}/{len(candidates)}",
                    f"Log Roll P {row['roll']} and Pitch P {row['pitch']} with D {self.pid_plan_selected_d}.",
                    target,
                )

            if self.pid_plan_phase == "d_recheck":
                if self.pid_plan_selected_p is None:
                    return None
                candidates = pid_plan_d_recheck_candidates()
                if self.pid_plan_index >= len(candidates):
                    return None
                d_value = candidates[self.pid_plan_index]
                target = roll_pitch_target(
                    self.pid_plan_selected_p["roll"],
                    self.pid_plan_selected_p["pitch"],
                    d_value,
                    d_value,
                    0,
                    0,
                    0,
                    0,
                )
                return (
                    f"D re-check {self.pid_plan_index + 1}/{len(candidates)}",
                    f"Log Roll/Pitch D {d_value} with chosen P.",
                    target,
                )

            if self.pid_plan_phase == "i_sweep":
                if self.pid_plan_selected_p is None or self.pid_plan_selected_d is None:
                    return None
                if self.pid_plan_index >= len(self.pid_plan.i_sweep):
                    return None
                row = self.pid_plan.i_sweep[self.pid_plan_index]
                target = roll_pitch_target(
                    self.pid_plan_selected_p["roll"],
                    self.pid_plan_selected_p["pitch"],
                    self.pid_plan_selected_d,
                    self.pid_plan_selected_d,
                    row["roll"],
                    row["pitch"],
                    0,
                    0,
                )
                return (
                    f"I sweep {self.pid_plan_index + 1}/{len(self.pid_plan.i_sweep)}",
                    f"Log Roll I {row['roll']} and Pitch I {row['pitch']}.",
                    target,
                )

            if self.pid_plan_phase == "ff_sweep":
                if self.pid_plan_selected_p is None or self.pid_plan_selected_d is None or self.pid_plan_selected_i is None:
                    return None
                if self.pid_plan_index >= len(self.pid_plan.ff_sweep):
                    return None
                row = self.pid_plan.ff_sweep[self.pid_plan_index]
                target = roll_pitch_target(
                    self.pid_plan_selected_p["roll"],
                    self.pid_plan_selected_p["pitch"],
                    self.pid_plan_selected_d,
                    self.pid_plan_selected_d,
                    self.pid_plan_selected_i["roll"],
                    self.pid_plan_selected_i["pitch"],
                    row["roll"],
                    row["pitch"],
                )
                return (
                    f"FF sweep {self.pid_plan_index + 1}/{len(self.pid_plan.ff_sweep)}",
                    f"Log Roll FF {row['roll']} and Pitch FF {row['pitch']}.",
                    target,
                )

            return None

        def advance_pid_plan_after_step() -> None:
            if self.pid_plan_phase == "safe_start":
                self.pid_plan_phase = "d_sweep"
                self.pid_plan_index = 0
                return
            self.pid_plan_index += 1

        def run_pid_plan_final_write() -> None:
            if (
                self.pid_plan is None
                or self.pid_plan_selected_p is None
                or self.pid_plan_selected_d is None
                or self.pid_plan_selected_i is None
                or self.pid_plan_selected_ff is None
            ):
                raise RuntimeError("PID plan final values are incomplete.")

            roll_pitch = roll_pitch_target(
                self.pid_plan_selected_p["roll"],
                self.pid_plan_selected_p["pitch"],
                self.pid_plan_selected_d,
                self.pid_plan_selected_d,
                self.pid_plan_selected_i["roll"],
                self.pid_plan_selected_i["pitch"],
                self.pid_plan_selected_ff["roll"],
                self.pid_plan_selected_ff["pitch"],
            )
            with_yaw = dict(roll_pitch)
            with_yaw["yaw"] = dict(self.pid_plan.yaw_final_pid_ff)
            current = read_fc_pid_ff_values()
            set_pid_plan_report_text(self.pid_plan, "PID plan final write", with_yaw, current)
            prompt = (
                "The roll/pitch sweeps are complete.\n\n"
                "DISARM before writing final PID/FF values. The app will verify disarmed state before writing.\n\n"
                "Yes: write chosen roll/pitch values and the conservative yaw recommendation while disarmed.\n"
                "No: write chosen roll/pitch values only while disarmed.\n"
                "Cancel: stop without writing final values.\n\n"
                "Current vs target:\n"
                f"{format_pid_target_check(current, with_yaw)}"
            )
            choice = messagebox.askyesnocancel("PID Plan Final Values", prompt, parent=self.root)
            if choice is None:
                complete_pid_tuning_plan("PID tuning plan stopped before final write.")
                return
            if not ensure_disarmed_before_pid_write():
                self.status.set("PID final write canceled; disarm before writing values.")
                update_pid_progress_window()
                return
            write_fc_pid_ff_values(with_yaw if choice else roll_pitch)
            refresh_pid_ff_from_fc(update_status=False)
            complete_pid_tuning_plan("PID tuning plan complete.")
            messagebox.showinfo(
                "PID Plan Complete",
                "Final selected values were written. Review and save in INAV only when you are satisfied.",
                parent=self.root,
            )

        def continue_pid_tuning_plan() -> None:
            if not self.pid_plan_active:
                return
            if self.pid_plan is None:
                raise RuntimeError("PID tuning plan is not loaded.")
            if self.pid_plan_waiting_for_fly_log:
                open_pid_progress_window()
                messagebox.showinfo(
                    "Fly/Log Needed",
                    f"{self.pid_plan_current_candidate_title or 'The current candidate'} is ready.\n\n"
                    "Arm the drone, press Fly/Log, then disarm the drone before pressing Next PID Plan Step.",
                    parent=self.root,
                )
                self.status.set("Press Fly/Log for the current candidate before moving to the next step.")
                update_pid_progress_window()
                return
            if not prepare_pid_plan_next_step():
                update_pid_progress_window()
                return
            if self.pid_plan_phase == "final_write":
                update_pid_progress_window()
                run_pid_plan_final_write()
                return

            step = current_pid_plan_step()
            if step is None:
                self.status.set("PID plan is waiting for the next stage choice.")
                update_pid_progress_window()
                return
            title, instruction, target = step
            step_phase = self.pid_plan_phase
            update_pid_progress_window()
            if self.pid_plan_phase == "safe_start":
                set_pid_plan_report_text(self.pid_plan, f"PID plan step: {title}", target)
                if not ensure_disarmed_before_pid_write():
                    self.status.set("Safe-start PID write waiting; disarm the drone before starting the plan.")
                    update_pid_progress_window()
                    return
                current = read_fc_pid_ff_values(tuple(target.keys()))
                set_pid_plan_report_text(self.pid_plan, f"PID plan step: {title}", target, current)
                write_fc_pid_ff_values(target)
                refresh_pid_ff_from_fc(update_status=False)
                advance_pid_plan_after_step()
                self.pid_plan_waiting_for_fly_log = True
                self.pid_plan_current_candidate_title = title
                self.pid_plan_current_candidate_phase = step_phase
                self.pid_plan_current_candidate_target = target
                self.auto_session_button.config(text="Next PID Plan Step", state="normal")
                refresh_fly_log_button_state()
                self.status.set("Safe-start PID/FF values written while disarmed.")
                update_pid_progress_window()
                messagebox.showinfo(
                    "Safe Start Ready",
                    "Safe-start PID/FF values were written while the drone was disarmed.\n\n"
                    "Now arm the drone, press Fly/Log, then disarm the drone before pressing Next PID Plan Step.",
                    parent=self.root,
                )
                return

            if not ensure_disarmed_before_pid_write():
                self.status.set("PID plan paused; disarm the drone before moving to the next candidate.")
                update_pid_progress_window()
                return
            current = read_fc_pid_ff_values(tuple(target.keys()))
            set_pid_plan_report_text(self.pid_plan, f"PID plan step: {title}", target, current)
            prompt = (
                f"{title}\n\n"
                f"{instruction}\n\n"
                "Required sequence for this candidate:\n"
                "1. DISARM the drone.\n"
                "2. Write/check these PID/FF values only while disarmed.\n"
                "3. Arm, then press Fly/Log for this candidate.\n"
                "4. Land and DISARM before pressing Next PID Plan Step.\n\n"
                "Yes: write these target values now after the app confirms the FC is disarmed.\n"
                "No: skip this write and mark the step done.\n"
                "Cancel: stop the guided PID plan.\n\n"
                "Current vs target:\n"
                f"{format_pid_target_check(current, target)}"
            )
            choice = messagebox.askyesnocancel("PID Plan Step", prompt, parent=self.root)
            if choice is None:
                complete_pid_tuning_plan("PID tuning plan stopped by user.")
                return
            if choice:
                if not ensure_disarmed_before_pid_write():
                    self.status.set("PID write canceled; disarm before writing values.")
                    update_pid_progress_window()
                    return
                write_fc_pid_ff_values(target)
                refresh_pid_ff_from_fc(update_status=False)
                self.pid_plan_waiting_for_fly_log = True
                self.pid_plan_current_candidate_title = title
                self.pid_plan_current_candidate_phase = step_phase
                self.pid_plan_current_candidate_target = target
                self.status.set(f"PID plan step written: {title}")
            else:
                self.pid_plan_waiting_for_fly_log = False
                self.pid_plan_current_candidate_title = ""
                self.pid_plan_current_candidate_phase = ""
                self.pid_plan_current_candidate_target = None
                self.status.set(f"PID plan step skipped: {title}")
            advance_pid_plan_after_step()
            self.auto_session_button.config(text="Next PID Plan Step", state="normal")
            refresh_fly_log_button_state()
            update_pid_progress_window()
            messagebox.showinfo(
                "PID Plan Step Ready",
                (
                    "Values are ready for this candidate.\n\n"
                    "Now arm the drone, press Fly/Log, then disarm the drone before pressing Next PID Plan Step."
                    if choice
                    else "This candidate was skipped. Press Next PID Plan Step when ready for the next candidate."
                ),
                parent=self.root,
            )

        def start_pid_tuning_plan_session() -> None:
            plan_path = locate_pid_tuning_plan_file()
            self.pid_plan = load_pid_tuning_plan(plan_path)
            self.pid_plan_active = True
            self.pid_plan_phase = "safe_start"
            self.pid_plan_index = 0
            self.pid_plan_selected_d = None
            self.pid_plan_selected_p = None
            self.pid_plan_selected_i = None
            self.pid_plan_selected_ff = None
            self.pid_plan_waiting_for_fly_log = False
            self.pid_plan_current_candidate_title = ""
            self.pid_plan_current_candidate_phase = ""
            self.pid_plan_current_candidate_target = None
            set_pid_plan_report_text(self.pid_plan, "PID tuning plan loaded")
            self.auto_session_button.config(text="Next PID Plan Step", state="normal")
            refresh_fly_log_button_state()
            self.status.set(f"PID tuning plan loaded: {self.pid_plan.text_path}")
            open_pid_progress_window()
            continue_pid_tuning_plan()

        def read_fc_armed_state_for_blackbox_import(selected_port: str, selected_baud: int) -> bool:
            if not self.fc_service.is_connected:
                raise RuntimeError("Connect FC manually before pulling Blackbox logs.")
            return self.fc_service.is_armed(timeout_seconds=0.8)

        def ensure_disarmed_before_blackbox_import(selected_port: str, selected_baud: int) -> bool:
            if not self.fc_service.is_connected:
                messagebox.showwarning(
                    "FC Not Connected",
                    "Connect FC manually before pulling Blackbox logs. The app will not auto-connect to the FC.",
                    parent=self.root,
                )
                return False
            while True:
                try:
                    is_armed = read_fc_armed_state_for_blackbox_import(selected_port, selected_baud)
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
                    "Disarm it before pulling Blackbox logs or entering MSC mode, then click Retry.",
                    icon="warning",
                    parent=self.root,
                )
                if not retry:
                    return False

        def do_pull_blackbox_logs() -> None:
            try:
                if simulation_mode_enabled():
                    self.status.set("Turn off Simulate before pulling Blackbox logs.")
                    return
                if self.blackbox_import_inflight:
                    self.status.set("Blackbox import already in progress.")
                    return

                selected_port = fc_port()
                selected_baud = fc_baud()
                if not ensure_disarmed_before_blackbox_import(selected_port, selected_baud):
                    self.status.set("Blackbox import canceled; disarm the drone before pulling logs.")
                    return
                if self.fc_service.is_connected:
                    do_fc_disconnect(update_status=False)

                self.blackbox_import_inflight = True
                self.status.set(f"Requesting FC MSC mode on {selected_port} @ {selected_baud}, then scanning mounted volumes...")

                def on_pull_done(ok: bool, res: object) -> None:
                    self.blackbox_import_inflight = False
                    if not ok:
                        set_error("Blackbox import error", res if isinstance(res, Exception) else RuntimeError(res))
                        return
                    if not isinstance(res, BlackboxImportResult):
                        set_error("Blackbox import error", RuntimeError("Unexpected import task result."))
                        return

                    imported_count = len(res.imported_files)
                    if imported_count == 0:
                        if res.skipped_count > 0:
                            self.status.set(
                                f"No new Blackbox logs were copied ({res.skipped_count} duplicate file(s) skipped)."
                            )
                        else:
                            self.status.set("No new Blackbox logs were imported from MSC volumes.")
                    else:
                        if res.skipped_count > 0:
                            self.status.set(
                                f"Imported {imported_count} Blackbox file(s) to {self.blackbox_import_dir} "
                                f"({res.skipped_count} duplicate file(s) skipped)."
                            )
                        else:
                            self.status.set(
                                f"Imported {imported_count} Blackbox file(s) to {self.blackbox_import_dir}."
                            )

                    set_auto_report_text(format_blackbox_report(res))

                self.worker.submit(_task_enter_msc_and_import_blackbox_logs, selected_port, selected_baud, callback=on_pull_done)
            except Exception as exc:
                self.blackbox_import_inflight = False
                set_error("Blackbox import error", exc)

        def do_analyze_blackbox_logs() -> None:
            try:
                if self.blackbox_import_inflight:
                    self.status.set("Blackbox import already in progress.")
                    return

                initial_dir = self.blackbox_import_dir if self.blackbox_import_dir.exists() else Path.cwd()
                selected_log = filedialog.askopenfilename(
                    parent=self.root,
                    title="Select Blackbox Log to Analyze",
                    initialdir=str(initial_dir),
                    filetypes=(
                        ("Blackbox logs", "*.bbl *.bfl *.bbs *.txt *.csv"),
                        ("All files", "*.*"),
                    ),
                )
                if not selected_log:
                    self.status.set("Blackbox analysis canceled.")
                    return

                self.blackbox_import_inflight = True
                selected_name = Path(selected_log).name
                self.status.set(f"Analyzing Blackbox log: {selected_name}...")

                def on_analyze_done(ok: bool, res: object) -> None:
                    if not ok:
                        self.blackbox_import_inflight = False
                        set_error("Blackbox analyze error", res if isinstance(res, Exception) else RuntimeError(res))
                        return
                    if not isinstance(res, BlackboxImportResult):
                        self.blackbox_import_inflight = False
                        set_error("Blackbox analyze error", RuntimeError("Unexpected analysis task result."))
                        return

                    summary = res.analysis_summary
                    summary_head = summary.split("|", 1)[0].strip()
                    if res.pid_report is not None and res.pid_report.headline:
                        summary_head = res.pid_report.headline
                    self.status.set(f"Blackbox analysis complete: {summary_head}. Generating report...")
                    set_auto_report_text(format_blackbox_report(res))
                    self.auto_latest_report = None

                    session_payload = {
                        "state": "manual_analyze",
                        "stop_reason": "Manual Analyze Logs run",
                        "warning": "",
                        "elapsed_s": 0.0,
                        "metrics": {},
                    }

                    def on_report_done(ok2: bool, res2: object) -> None:
                        self.blackbox_import_inflight = False
                        if not ok2:
                            error_text = str(res2) if not isinstance(res2, Exception) else str(res2)
                            if "Could not resolve a primary Blackbox CSV for" in error_text:
                                error_text += (
                                    "\nTip: the selected log did not resolve to a usable CSV. "
                                    "Try selecting a CSV log directly, or import/decode the raw log first."
                                )
                            elif "No module named" in error_text:
                                error_text += (
                                    "\nTip: this Python environment may be missing required packages "
                                    "(for the HTML chart viewer: numpy and plotly)."
                                )
                            self.status.set("Blackbox analysis complete, but report generation failed.")
                            set_auto_report_text(f"{format_blackbox_report(res)}\n\nReport generation error: {error_text}")
                            return
                        if not isinstance(res2, AutoTuneReport):
                            set_error("Blackbox report error", RuntimeError("Unexpected report task result."))
                            return

                        self.auto_latest_report = res2
                        try:
                            report_text = Path(res2.summary_txt).read_text(encoding="utf-8", errors="replace")
                        except Exception:
                            report_text = f"Report generated at {res2.report_dir}\nSummary file: {res2.summary_txt}"
                        set_auto_report_text(report_text)
                        self.status.set(f"Blackbox report generated: {res2.report_dir}")

                    self.worker.submit(
                        _task_generate_auto_report,
                        res,
                        session_payload,
                        selected_log,
                        callback=on_report_done,
                    )

                self.worker.submit(_task_analyze_specific_blackbox_log, selected_log, callback=on_analyze_done)
            except Exception as exc:
                self.blackbox_import_inflight = False
                set_error("Blackbox analyze error", exc)

        def do_step_response_report() -> None:
            try:
                if self.blackbox_import_inflight:
                    self.status.set("Blackbox import/analyze already in progress.")
                    return
                if auto_is_running():
                    raise RuntimeError("Wait for the auto session/pipeline to finish first.")

                initial_dir = self.blackbox_import_dir if self.blackbox_import_dir.exists() else Path.cwd()
                selected_logs = filedialog.askopenfilenames(
                    parent=self.root,
                    title=f"Select Blackbox Logs for Step Response (max {MAX_STEP_RESPONSE_LOGS})",
                    initialdir=str(initial_dir),
                    filetypes=(
                        ("Blackbox logs", "*.bbl *.bfl *.bbs *.txt *.csv"),
                        ("All files", "*.*"),
                    ),
                )
                if not selected_logs:
                    self.status.set("Step response canceled.")
                    return
                if len(selected_logs) > MAX_STEP_RESPONSE_LOGS:
                    raise RuntimeError(f"Select at most {MAX_STEP_RESPONSE_LOGS} Blackbox logs.")

                self.blackbox_import_inflight = True
                self.step_response_button.config(state="disabled")
                count = len(selected_logs)
                self.status.set(f"Generating step response report for {count} log file(s)...")
                set_auto_report_text(
                    f"Step response generation started for {count} log file(s).\n"
                    "Raw logs will be decoded with tools/blackbox_decode_INAV.exe."
                )

                def on_step_response_done(ok: bool, res: object) -> None:
                    self.blackbox_import_inflight = False
                    self.step_response_button.config(state="normal")
                    if not ok:
                        set_error("Step response error", res if isinstance(res, Exception) else RuntimeError(res))
                        return
                    if not isinstance(res, StepResponseReport):
                        set_error("Step response error", RuntimeError("Unexpected step-response task result."))
                        return

                    set_auto_report_text(format_step_response_report(res))
                    self.status.set(f"Step response report generated: {res.report_dir}")

                self.worker.submit(_task_generate_step_response_report, list(selected_logs), callback=on_step_response_done)
            except Exception as exc:
                self.blackbox_import_inflight = False
                self.step_response_button.config(state="normal")
                set_error("Step response error", exc)

        def auto_is_running() -> bool:
            return self.auto_state in {
                AdaptiveSessionState.adaptive_run,
                AdaptiveSessionState.recovery,
                AdaptiveSessionState.finalize,
                AdaptiveSessionState.import_analyze,
            }

        def schedule_auto_tick(delay_ms: int | None = None) -> None:
            if not auto_is_running():
                return
            if self.auto_tick_after_id is not None:
                try:
                    self.root.after_cancel(self.auto_tick_after_id)
                except Exception:
                    pass
            cadence_ms = max(10, round(self.auto_config.control_interval_s * 1000.0))
            self.auto_tick_after_id = self.root.after(cadence_ms if delay_ms is None else max(1, delay_ms), run_auto_tick)

        def stop_auto_session_runtime() -> None:
            if self.auto_tick_after_id is not None:
                try:
                    self.root.after_cancel(self.auto_tick_after_id)
                except Exception:
                    pass
                self.auto_tick_after_id = None
            cancel_auto_hold_timer()
            cancel_fly_log_marker_timer()
            restore_auto_original_base_outputs()
            self.auto_pulse_inflight = False
            self.auto_hold_end_requested = False
            self.auto_settle_until_s = None
            self.auto_active_command = None

        def set_auto_button_idle() -> None:
            self.auto_session_button.config(text="Start Auto Session", state="normal")

        def complete_auto_session(next_state: AdaptiveSessionState, reason: str, warning: str = "") -> None:
            self.auto_stop_reason = reason
            self.auto_warning = warning
            self.beeper_marker_active = False
            stop_auto_session_runtime()
            if arduino_output_connected():
                try:
                    restore_base_outputs_after_hold()
                except Exception:
                    pass
            set_auto_state(next_state, warning or reason)

        def auto_abort(reason: str, warning: str = "", continue_pipeline: bool = False) -> None:
            complete_auto_session(AdaptiveSessionState.aborted, reason, warning)
            self.pid_plan_fly_log_active = False
            refresh_fly_log_button_state()
            self.status.set(f"Auto session aborted: {reason}")
            set_auto_button_idle()
            if self.pid_plan_active:
                self.auto_session_button.config(text="Next PID Plan Step", state="normal")
            update_pid_progress_window()
            if continue_pipeline:
                begin_auto_pipeline()

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
                "- Drone is disarmed before any PID/FF write\n"
                "- You will write/check values only while disarmed\n"
                "- You are ready to arm and press Fly/Log for one candidate at a time\n"
                "- You will land and disarm before pressing Next PID Plan Step\n\n"
                "The app will load pid_tuning_plan.txt, compare the current FC PID/FF values "
                "to the next plan target, and ask before writing each step.\n\n"
                "It will not run randomized stick pulses and it will not save final values automatically.\n\n"
                "Start guided PID tuning plan now?"
            )
            if not messagebox.askyesno("Start Auto Session", prompt):
                self.status.set("Auto session start canceled.")
                return
            start_pid_tuning_plan_session()

        def finalize_auto_event() -> None:
            if self.auto_controller is None or self.auto_active_command is None:
                return
            sample = self.fc_service.latest_attitude()
            if sample is None:
                return
            recovery_event = self.auto_active_command.recovery
            axis_value = pulse_axis_value(sample, self.auto_active_command.axis)
            final_error = axis_value - self.auto_event_baseline
            settle_success = abs(final_error) <= self.auto_config.settle_deadband_deg
            event = ExcitationEvent(
                axis=self.auto_active_command.axis,
                direction=self.auto_active_command.direction,
                force_us=self.auto_active_command.force_us,
                hold_s=self.auto_active_command.hold_s,
                settle_s=self.auto_active_command.settle_s,
                baseline_angle_deg=self.auto_event_baseline,
                peak_delta_deg=self.auto_event_peak_delta,
                settle_success=settle_success,
                response_delay_s=self.auto_event_response_delay_s,
                final_error_deg=final_error,
            )
            self.auto_controller.record_event(event)
            adjust_auto_throttle_after_event(event, recovery_event)
            self.auto_active_command = None
            self.auto_hold_end_requested = False
            self.auto_event_peak_delta = 0.0
            self.auto_event_response_delay_s = None
            self.auto_event_baseline = 0.0
            self.auto_event_start_s = 0.0

        def issue_auto_command(command: AdaptiveCommand) -> None:
            if not arduino_output_connected():
                raise RuntimeError("Arduino output is disconnected.")
            if self.auto_controller is None:
                raise RuntimeError("Adaptive controller is not initialized.")

            sample = self.fc_service.latest_attitude()
            if sample is None:
                raise RuntimeError("No FC attitude sample available.")

            channel_index = axis_channel_index(command.axis)
            target = self.base_channel_outputs[channel_index] + (command.direction * command.force_us)
            target = max(1000, min(2000, target))
            offsets = parse_offset_values_with_defaults()

            active_outputs = self.base_channel_outputs.copy()
            active_outputs[channel_index] = target
            set_live_channel_outputs(active_outputs)
            self.auto_pulse_inflight = True
            self.auto_hold_end_requested = False
            self.auto_settle_until_s = None
            self.auto_active_command = command
            self.auto_event_peak_delta = 0.0
            self.auto_event_response_delay_s = None
            self.auto_event_baseline = pulse_axis_value(sample, command.axis)
            self.auto_event_start_s = time.monotonic()

            def on_auto_hold_elapsed() -> None:
                self.auto_hold_after_id = None
                if not auto_is_running() or self.auto_active_command is not command:
                    return
                begin_auto_observe_window(command)

            def on_auto_hold_done(ok: bool, res: object) -> None:
                if not auto_is_running():
                    return
                if not ok:
                    self.auto_pulse_inflight = False
                    auto_abort(
                        "Pulse command failed during auto session.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                if not isinstance(res, int):
                    self.auto_pulse_inflight = False
                    auto_abort("Unexpected pulse result from worker.")
                    return
                if res == PULSE_STATUS_REJECTED:
                    self.auto_pulse_inflight = False
                    auto_abort("Firmware rejected adaptive pulse command.")
                    return
                if self.auto_hold_end_requested:
                    return
                self.auto_hold_after_id = self.root.after(round(command.hold_s * 1000.0), on_auto_hold_elapsed)

            self.worker.submit(_task_hold, channel_index, target, offsets[channel_index], command.hold_s, callback=on_auto_hold_done)

        def run_auto_tick() -> None:
            self.auto_tick_after_id = None
            if not auto_is_running():
                return
            if self.auto_controller is None:
                auto_abort("Adaptive controller was not initialized.")
                return
            if not arduino_output_connected():
                auto_abort("Arduino output disconnected during auto session.")
                return
            if not self.fc_service.is_connected:
                auto_abort("FC disconnected during auto session.")
                return

            now = time.monotonic()
            if self.auto_last_sample_s is None or (now - self.auto_last_sample_s) > self.auto_config.telemetry_stale_s:
                auto_abort("FC telemetry became stale.", continue_pipeline=False)
                return

            sample = self.fc_service.latest_attitude()
            if sample is None:
                schedule_auto_tick()
                return

            abort, abort_reason = self.auto_controller.should_abort(sample.roll_deg, sample.pitch_deg)
            if abort:
                auto_abort(abort_reason, continue_pipeline=not self.pid_plan_fly_log_active)
                return

            if self.auto_controller.should_recover(sample.roll_deg, sample.pitch_deg):
                self.auto_recovery_mode = True
                set_auto_state(AdaptiveSessionState.recovery, "Recovery mode")
            elif self.auto_recovery_mode and self.auto_controller.recovery_complete(sample.roll_deg, sample.pitch_deg):
                self.auto_recovery_mode = False
                set_auto_state(AdaptiveSessionState.adaptive_run, "Active")

            if self.auto_pulse_inflight:
                schedule_auto_tick()
                return

            if self.auto_active_command is not None and self.auto_settle_until_s is not None and now < self.auto_settle_until_s:
                schedule_auto_tick()
                return

            if self.auto_active_command is not None and self.auto_settle_until_s is not None and now >= self.auto_settle_until_s:
                finalize_auto_event()
                self.auto_settle_until_s = None

            ready, stop_reason, warning = self.auto_controller.stop_ready(auto_elapsed_s(now))
            if ready:
                if self.pid_plan_fly_log_active:
                    complete_auto_session(AdaptiveSessionState.report_ready, stop_reason, warning)
                    self.pid_plan_fly_log_active = False
                    self.pid_plan_waiting_for_fly_log = False
                    completed_title = self.pid_plan_current_candidate_title
                    self.pid_plan_current_candidate_title = ""
                    self.pid_plan_current_candidate_phase = ""
                    self.pid_plan_current_candidate_target = None
                    self.auto_session_button.config(text="Next PID Plan Step", state="normal")
                    refresh_fly_log_button_state()
                    self.status.set(f"Fly/Log complete: {completed_title or stop_reason}")
                    update_pid_progress_window()
                    messagebox.showinfo(
                        "Fly/Log Complete",
                        "Fly/Log movement is complete.\n\n"
                        "Disarm the drone now. After it is disarmed, press Next PID Plan Step.",
                        parent=self.root,
                    )
                else:
                    complete_auto_session(AdaptiveSessionState.finalize, stop_reason, warning)
                    self.status.set(f"Auto session complete: {stop_reason}")
                    begin_auto_pipeline()
                return

            command = self.auto_controller.next_command(sample.roll_deg, sample.pitch_deg, self.auto_recovery_mode)
            if command is None:
                schedule_auto_tick()
                return

            try:
                issue_auto_command(command)
            except Exception as exc:
                auto_abort("Unable to issue adaptive command.", warning=str(exc))
                return
            schedule_auto_tick()

        def begin_auto_pipeline() -> None:
            if simulation_mode_enabled():
                self.status.set("Auto blackbox pipeline skipped: simulation mode is enabled.")
                return
            if self.blackbox_import_inflight:
                auto_warning_text = "Blackbox pipeline already in progress."
                self.status.set(auto_warning_text)
                return
            try:
                selected_port = fc_port()
                selected_baud = fc_baud()
                if not ensure_disarmed_before_blackbox_import(selected_port, selected_baud):
                    set_auto_button_idle()
                    self.status.set("Auto blackbox pipeline canceled; disarm the drone before pulling logs.")
                    return
                if self.fc_service.is_connected:
                    do_fc_disconnect(update_status=False)
                self.blackbox_import_inflight = True
                set_auto_state(AdaptiveSessionState.import_analyze, "Import/analyze running")
                self.auto_session_button.config(text="Running Analysis...", state="disabled")
                self.status.set("Auto session finished. Pulling and analyzing blackbox logs...")

                def on_auto_pull_done(ok: bool, res: object) -> None:
                    if not ok:
                        self.blackbox_import_inflight = False
                        auto_abort(
                            "Auto pipeline failed while pulling blackbox logs.",
                            warning=str(res) if not isinstance(res, Exception) else str(res),
                        )
                        return
                    if not isinstance(res, BlackboxImportResult):
                        self.blackbox_import_inflight = False
                        auto_abort("Unexpected pull result in auto pipeline.")
                        return
                    self.auto_latest_imported_log = ""
                    if res.imported_files:
                        newest = max(res.imported_files, key=lambda item: item.modified_epoch_s)
                        self.auto_latest_imported_log = newest.local_path
                    elif res.analysis_source:
                        self.auto_latest_imported_log = res.analysis_source
                    if self.auto_latest_imported_log:
                        self.worker.submit(_task_analyze_specific_blackbox_log, self.auto_latest_imported_log, callback=on_auto_analyze_done)
                    else:
                        self.worker.submit(_task_analyze_blackbox_logs, callback=on_auto_analyze_done)

                def on_auto_analyze_done(ok: bool, res: object) -> None:
                    if not ok:
                        self.blackbox_import_inflight = False
                        auto_abort(
                            "Auto pipeline failed while analyzing blackbox logs.",
                            warning=str(res) if not isinstance(res, Exception) else str(res),
                        )
                        return
                    if not isinstance(res, BlackboxImportResult):
                        self.blackbox_import_inflight = False
                        auto_abort("Unexpected analyze result in auto pipeline.")
                        return
                    self.auto_import_result = res
                    set_auto_report_text(format_blackbox_report(res))
                    self.worker.submit(
                        _task_generate_auto_report,
                        res,
                        auto_session_payload(),
                        self.auto_latest_imported_log,
                        callback=on_auto_report_done,
                    )

                def on_auto_report_done(ok: bool, res: object) -> None:
                    self.blackbox_import_inflight = False
                    set_auto_button_idle()
                    if not ok:
                        auto_abort(
                            "Auto pipeline failed while generating report artifacts.",
                            warning=str(res) if not isinstance(res, Exception) else str(res),
                        )
                        return
                    if not isinstance(res, AutoTuneReport):
                        auto_abort("Unexpected report generation result.")
                        return
                    self.auto_latest_report = res
                    try:
                        report_text = Path(res.summary_txt).read_text(encoding="utf-8", errors="replace")
                    except Exception:
                        report_text = f"Report generated at {res.report_dir}\nSummary file: {res.summary_txt}"
                    set_auto_report_text(report_text)
                    set_auto_state(AdaptiveSessionState.report_ready, "Ready")
                    self.status.set(f"Auto report ready: {res.report_dir}")

                self.worker.submit(_task_enter_msc_and_import_blackbox_logs, selected_port, selected_baud, callback=on_auto_pull_done)
            except Exception as exc:
                self.blackbox_import_inflight = False
                auto_abort("Unable to start auto blackbox pipeline.", warning=str(exc))

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

            self.auto_config = replace(read_auto_tune_config(), max_runtime_s=PID_PLAN_FLY_LOG_RUNTIME_S)
            prompt = (
                f"Fly/Log candidate: {self.pid_plan_current_candidate_title or 'current PID plan step'}\n\n"
                "The FC reports ARMED.\n\n"
                "Pressing OK will send bounded roll/pitch stick movement through the Arduino output "
                f"for {PID_PLAN_FLY_LOG_RUNTIME_S:.0f} seconds after the CH8 marker turns on so Blackbox "
                "has movement to log.\n\n"
                "The app will first let the throttle settle briefly, then channel 8 will switch "
                "high as the beeper log marker. Channel 8 switches low when Fly/Log completes "
                "or aborts.\n\n"
                "No PID/FF values will be written while armed.\n\n"
                "Keep the drone secured, keep the area clear, and be ready to disarm."
            )
            if not messagebox.askokcancel("Start Fly/Log", prompt, icon="warning", parent=self.root):
                self.status.set("Fly/Log canceled.")
                return

            self.auto_controller = AdaptiveExcitationController(self.auto_config)
            self.auto_original_base_outputs = self.base_channel_outputs.copy()
            self.auto_start_throttle_us = self.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
            self.auto_current_throttle_us = self.auto_start_throttle_us
            self.auto_peak_throttle_us = self.auto_start_throttle_us
            self.auto_stop_reason = ""
            self.auto_warning = ""
            self.auto_session_start_s = time.monotonic()
            self.auto_last_tick_s = self.auto_session_start_s
            self.auto_last_sample_s = self.auto_session_start_s
            self.pid_plan_fly_log_active = True
            set_auto_state(AdaptiveSessionState.adaptive_run, "Fly/Log active")
            throttle_prepared = prepare_auto_throttle(send_update=False)
            self.beeper_marker_active = False
            self.auto_session_button.config(text="Abort Fly/Log", state="normal")
            refresh_fly_log_button_state()
            spinup_delay_s = BEEPER_MARKER_SPINUP_DELAY_MS / 1000.0
            self.status.set(f"Preparing Fly/Log outputs. CH8 marker starts after {spinup_delay_s:.1f}s spin-up.")
            open_pid_progress_window()
            update_pid_progress_window()
            set_auto_report_text(
                "Fly/Log active\n\n"
                f"Candidate: {self.pid_plan_current_candidate_title or 'current PID plan step'}\n"
                f"The app is waiting {spinup_delay_s:.1f}s for spin-up before enabling the channel 8 marker.\n"
                f"After the marker turns on, the app sends bounded roll/pitch movement for "
                f"{PID_PLAN_FLY_LOG_RUNTIME_S:.0f}s of Blackbox data.\n"
                "No PID/FF values are being written while armed.\n\n"
                "When complete, disarm the drone before pressing Next PID Plan Step."
            )

            def on_beeper_marker_enabled(ok: bool, res: object) -> None:
                if not self.pid_plan_fly_log_active:
                    return
                if not ok:
                    auto_abort(
                        "Unable to enable channel 8 beeper log marker.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                now_s = time.monotonic()
                self.auto_session_start_s = now_s
                self.auto_last_tick_s = now_s
                self.auto_last_sample_s = now_s
                self.status.set("Fly/Log movement active. Channel 8 beeper marker is ON.")
                schedule_auto_tick(delay_ms=250 if throttle_prepared else None)

            def enable_beeper_marker_after_spinup() -> None:
                self.fly_log_marker_after_id = None
                if not self.pid_plan_fly_log_active or not auto_is_running():
                    return
                self.beeper_marker_active = True
                self.status.set("Enabling channel 8 beeper log marker...")
                queue_live_channel_update(
                    self.base_channel_outputs.copy(),
                    parse_offset_values_with_defaults(),
                    after_update=on_beeper_marker_enabled,
                )

            def on_spinup_outputs_prepared(ok: bool, res: object) -> None:
                if not self.pid_plan_fly_log_active:
                    return
                if not ok:
                    auto_abort(
                        "Unable to prepare Fly/Log outputs before marker.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                self.status.set(f"Fly/Log spin-up wait: channel 8 marker starts in {spinup_delay_s:.1f}s.")
                self.fly_log_marker_after_id = self.root.after(
                    max(1, BEEPER_MARKER_SPINUP_DELAY_MS),
                    enable_beeper_marker_after_spinup,
                )

            queue_live_channel_update(
                self.base_channel_outputs.copy(),
                parse_offset_values_with_defaults(),
                after_update=on_spinup_outputs_prepared,
            )

        def do_pid_plan_fly_log_toggle() -> None:
            try:
                if self.sim_fly_log_active:
                    stop_simulated_auto_session("Simulated Fly/Log stopped.", restore_display=False)
                    return
                if self.sim_plan is not None and self.sim_waiting_for_fly_log:
                    start_simulated_fly_log()
                    return
                if self.pid_plan_fly_log_active:
                    auto_abort("Fly/Log aborted by user.", continue_pipeline=False)
                    self.pid_plan_fly_log_active = False
                    refresh_fly_log_button_state()
                    update_pid_progress_window()
                    return
                start_pid_plan_fly_log()
            except Exception as exc:
                set_error("Fly/Log error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

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
                "Start P with I = 0, FF = 0, and the first D value.",
                "d",
                roll_pitch_target(plan.start_p["roll"], plan.start_p["pitch"], start_d, start_d, 0, 0, 0, 0),
            )
            for index, d_value in enumerate(plan.d_sweep[1:], start=2):
                add(
                    f"D sweep {index}/{len(plan.d_sweep)}",
                    f"Compare damping with Roll/Pitch D {d_value}.",
                    "d",
                    roll_pitch_target(plan.start_p["roll"], plan.start_p["pitch"], int(d_value), int(d_value), 0, 0, 0, 0),
                )
            if plan.optional_d is not None and plan.optional_d not in plan.d_sweep:
                add(
                    "Optional D sweep",
                    f"Optional comparison at Roll/Pitch D {plan.optional_d}.",
                    "d",
                    roll_pitch_target(plan.start_p["roll"], plan.start_p["pitch"], int(plan.optional_d), int(plan.optional_d), 0, 0, 0, 0),
                    "Real tuning should only run this if needed and motors stay cool.",
                )
            for index, row in enumerate(pid_plan_p_candidates_for(plan), start=1):
                add(
                    f"P sweep {index}/{len(pid_plan_p_candidates_for(plan))}",
                    f"Compare tracking with Roll P {row['roll']} and Pitch P {row['pitch']}.",
                    "p",
                    roll_pitch_target(row["roll"], row["pitch"], preview_d, preview_d, 0, 0, 0, 0),
                    f"Simulation uses preview D {preview_d}; real tuning uses the D you choose from logs.",
                )
            for index, d_value in enumerate(simulated_d_recheck_values(preview_d), start=1):
                add(
                    f"D re-check {index}/3",
                    f"Re-check damping with chosen P and Roll/Pitch D {d_value}.",
                    "d",
                    roll_pitch_target(preview_p["roll"], preview_p["pitch"], d_value, d_value, 0, 0, 0, 0),
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
                "Hardware is intentionally disconnected for simulation. These values are written only to the UI PID boxes.",
                "",
                "Real-world sequence for this step:",
                "1. Disarm before writing/checking these PID/FF values.",
                "2. Write values only while disarmed.",
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
            plan_path = locate_pid_tuning_plan_file()
            self.sim_plan = load_pid_tuning_plan(plan_path)
            self.sim_plan_steps = build_simulated_pid_plan_steps(self.sim_plan)
            self.sim_plan_step_index = 0
            if not self.sim_plan_steps:
                raise RuntimeError("PID tuning plan has no steps to simulate.")
            start_simulated_plan_step()

        def do_simulated_auto_session_toggle() -> None:
            try:
                if self.sim_active:
                    stop_simulated_auto_session("Simulation stopped.")
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
            try:
                if simulation_mode_enabled():
                    if auto_is_running():
                        auto_abort("Auto session aborted by user.", continue_pipeline=not self.pid_plan_fly_log_active)
                        return
                    do_simulated_auto_session_toggle()
                    return
                if self.sim_active:
                    stop_simulated_auto_session("Simulation stopped.", clear_walkthrough=True)
                if auto_is_running():
                    auto_abort("Auto session aborted by user.", continue_pipeline=not self.pid_plan_fly_log_active)
                    return
                if self.auto_state == AdaptiveSessionState.import_analyze:
                    self.status.set("Auto pipeline is running; wait for completion.")
                    return
                if self.pid_plan_active:
                    open_pid_progress_window()
                    continue_pid_tuning_plan()
                    return
                start_auto_session()
            except Exception as exc:
                set_error("Auto session error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

        def on_simulation_mode_changed() -> None:
            try:
                if simulation_mode_enabled():
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
            sim_mode = simulation_mode_enabled()
            if self.controller.is_connected:
                self.pc_link_box.config(text="PC-ARD OPEN", bg="#2E7D32", fg="white")
            else:
                self.pc_link_box.config(text="PC-ARD CLOSED", bg="#8B1E1E", fg="white")
            fc_connected = self.fc_service.is_connected
            if fc_connected:
                self.connect_fc_button.config(
                    text="Disconnect FC",
                    state="normal",
                    bg="#BEEAC4",
                    activebackground="#A6E1AE",
                    fg="#0E2F11",
                    activeforeground="#0E2F11",
                )
            else:
                self.connect_fc_button.config(
                    text="Connect FC",
                    state="normal",
                    bg="#F3C1C1",
                    activebackground="#ECA8A8",
                    fg="#3A1111",
                    activeforeground="#3A1111",
                )
            if sim_mode:
                self.connect_fc_button.config(state="disabled")
            arduino_connected = self.controller.is_connected
            if self.start_pending:
                self.arduino_button.config(
                    text="Connecting...",
                    state="disabled",
                    bg="#F3E6B3",
                    activebackground="#EBD997",
                    fg="#3F3210",
                    activeforeground="#3F3210",
                )
            elif arduino_connected:
                self.arduino_button.config(
                    text="Disconnect Arduino",
                    state="normal",
                    bg="#BEEAC4",
                    activebackground="#A6E1AE",
                    fg="#0E2F11",
                    activeforeground="#0E2F11",
                )
            else:
                self.arduino_button.config(
                    text="Connect Arduino",
                    state="normal",
                    bg="#F3C1C1",
                    activebackground="#ECA8A8",
                    fg="#3A1111",
                    activeforeground="#3A1111",
                )
            if sim_mode:
                self.arduino_button.config(state="disabled")
            simulation_blocked = self.start_pending or arduino_connected or fc_connected
            self.simulation_mode_checkbutton.config(state="normal" if sim_mode or not simulation_blocked else "disabled")
            level_ready = self.controller.is_connected and fc_connected
            if self.level_active and not level_ready:
                stop_level_loop(update_status=False)
            self.level_button.config(
                state="normal" if level_ready else "disabled",
                relief="sunken" if self.level_active else "raised",
            )
            if self.auto_state == AdaptiveSessionState.import_analyze:
                self.auto_session_button.config(text="Running Analysis...", state="disabled")
            elif auto_is_running():
                self.auto_session_button.config(text="Abort Fly/Log" if self.pid_plan_fly_log_active else "Abort Auto Session", state="normal")
            elif sim_mode:
                if self.sim_active or self.sim_fly_log_active:
                    self.auto_session_button.config(text="Stop Simulation", state="normal")
                elif self.sim_waiting_for_fly_log:
                    self.auto_session_button.config(text="Next Sim Step", state="disabled")
                elif self.sim_plan is not None and self.sim_plan_step_index < len(self.sim_plan_steps):
                    self.auto_session_button.config(text="Next Sim Step", state="normal")
                else:
                    self.auto_session_button.config(text="Start Auto Session", state="normal")
            elif self.pid_plan_active:
                self.auto_session_button.config(text="Next PID Plan Step", state="normal")
            else:
                self.auto_session_button.config(text="Start Auto Session", state="normal")
            refresh_fly_log_button_state()

        def cancel_level_timer() -> None:
            if self.level_after_id is not None:
                try:
                    self.root.after_cancel(self.level_after_id)
                except Exception:
                    pass
                finally:
                    self.level_after_id = None

        def stop_level_loop(update_status: bool = False, reason: str = "Auto-level stopped.") -> None:
            was_active = self.level_active
            cancel_level_timer()
            self.level_active = False
            self.level_pulse_inflight = False
            self.level_timeout_deadline_s = None
            set_live_channel_outputs(self.base_channel_outputs)
            update_link_indicators()
            if update_status and was_active and not self.is_closing:
                self.status.set(reason)

        def level_target_from_angle(angle_deg: float) -> int | None:
            abs_angle = abs(angle_deg)
            if abs_angle <= LEVEL_DEADBAND_DEG:
                return None
            ratio = min(1.0, abs_angle / LEVEL_FULL_SCALE_DEG)
            delta = max(LEVEL_MIN_DELTA_US, round(LEVEL_MAX_DELTA_US * ratio))
            if angle_deg > 0:
                return LEVEL_CENTER_US - delta
            return LEVEL_CENTER_US + delta

        def is_level_attitude_settled(roll_deg: float, pitch_deg: float) -> bool:
            return abs(roll_deg) <= LEVEL_DEADBAND_DEG and abs(pitch_deg) <= LEVEL_DEADBAND_DEG

        def schedule_level_step(delay_ms: int = LEVEL_LOOP_INTERVAL_MS) -> None:
            cancel_level_timer()
            self.level_after_id = self.root.after(max(1, delay_ms), run_level_step)

        def run_level_step() -> None:
            self.level_after_id = None
            if not self.level_active:
                return
            if not arduino_output_connected():
                stop_level_loop(update_status=True, reason="Auto-level stopped: output is not running.")
                return
            if not self.fc_service.is_connected:
                stop_level_loop(update_status=True, reason="Auto-level stopped: FC is disconnected.")
                return
            if self.level_timeout_deadline_s is not None and time.monotonic() >= self.level_timeout_deadline_s:
                stop_level_loop(update_status=True, reason=f"Auto-level timed out after {LEVEL_TIMEOUT_DEFAULT_S:.3g}s.")
                return
            if self.level_pulse_inflight:
                schedule_level_step()
                return

            sample = self.fc_service.latest_attitude()
            if sample is None:
                schedule_level_step()
                return
            if is_level_attitude_settled(sample.roll_deg, sample.pitch_deg):
                stop_level_loop(update_status=True, reason="Auto-level complete: roll and pitch are centered.")
                return

            roll_target_us = level_target_from_angle(sample.roll_deg)
            pitch_target_us = level_target_from_angle(sample.pitch_deg)
            axis_targets: list[tuple[int, int, float]] = []
            if roll_target_us is not None:
                axis_targets.append((ROLL_CHANNEL_INDEX, roll_target_us, abs(sample.roll_deg)))
            if pitch_target_us is not None:
                axis_targets.append((PITCH_CHANNEL_INDEX, pitch_target_us, abs(sample.pitch_deg)))
            if not axis_targets:
                stop_level_loop(update_status=True, reason="Auto-level complete: roll and pitch are centered.")
                return
            channel_index, target_us, _ = max(axis_targets, key=lambda item: item[2])

            try:
                offsets = parse_entries(self.off_entries, int, "Offset")
            except Exception as exc:
                stop_level_loop(update_status=False)
                set_error("Level error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))
                return

            active_outputs = self.base_channel_outputs.copy()
            active_outputs[channel_index] = target_us
            set_live_channel_outputs(active_outputs)
            self.level_pulse_inflight = True

            def on_level_pulse_done(ok: bool, res: object) -> None:
                self.level_pulse_inflight = False
                if not self.level_active:
                    return
                if not ok:
                    stop_level_loop(update_status=False)
                    set_error("Level error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, int):
                    stop_level_loop(update_status=False)
                    set_error("Level error", RuntimeError("Unexpected worker result from level task"))
                    return
                if res == PULSE_STATUS_REJECTED:
                    stop_level_loop(update_status=False)
                    set_error("Level error", RuntimeError("Firmware rejected auto-level pulse"))
                    return
                schedule_level_step()

            self.worker.submit(
                _task_hold,
                channel_index,
                target_us,
                offsets[channel_index],
                LEVEL_PULSE_TIMEOUT_S,
                callback=on_level_pulse_done,
            )

        def do_level() -> None:
            try:
                if self.level_active:
                    stop_level_loop(update_status=True)
                    return
                if not arduino_output_connected():
                    raise RuntimeError("Press Connect Arduino before using Level.")
                if not self.fc_service.is_connected:
                    raise RuntimeError("Connect FC before using Level.")
                if self.fc_service.latest_attitude() is None:
                    raise RuntimeError("No FC attitude sample yet. Wait a moment, then press Level again.")
                self.level_active = True
                self.level_timeout_deadline_s = time.monotonic() + LEVEL_TIMEOUT_DEFAULT_S
                update_link_indicators()
                self.status.set(f"Auto-level active ({LEVEL_TIMEOUT_DEFAULT_S:.3g}s timeout). Press Level again to stop.")
                run_level_step()
            except Exception as exc:
                stop_level_loop(update_status=False)
                set_error("Level error", exc)

        def do_fc_connect() -> None:
            try:
                if simulation_mode_enabled():
                    raise RuntimeError("Turn off Simulate before connecting FC.")
                if self.fc_service.is_connected:
                    return
                selected_port = fc_port()
                selected_baud = fc_baud()
                self.fc_service.connect(selected_port, selected_baud)
                # Mirror Usb2Arduino flow: verify telemetry immediately, then load PID/FF asynchronously.
                _ = self.fc_service.read_attitude(timeout_seconds=2.0)
                update_link_indicators()
                self.status.set(f"FC connected: {selected_port} @ {selected_baud}. Loading PID/FF...")
                queue_fc_pid_ff_refresh(selected_port, selected_baud)
            except Exception as exc:
                set_error("FC connect error", exc)

        def do_fc_disconnect(update_status: bool = True) -> None:
            if self.auto_state in (AdaptiveSessionState.adaptive_run, AdaptiveSessionState.recovery):
                auto_abort("FC disconnected during adaptive session.", continue_pipeline=False)
            try:
                self.fc_service.disconnect()
            except Exception as exc:
                if not self.is_closing:
                    set_error("FC disconnect error", exc)
            finally:
                self.horizon.set_attitude(0.0, 0.0)
                self.roll_text.set("Roll: 0.0 deg")
                self.pitch_text.set("Pitch: 0.0 deg")
                clear_pid_ff_displays()
                update_link_indicators()
                if update_status and not self.is_closing:
                    self.status.set("FC disconnected.")

        def do_fc_toggle() -> None:
            if simulation_mode_enabled():
                self.status.set("Turn off Simulate before connecting FC.")
                return
            if self.fc_service.is_connected:
                do_fc_disconnect()
            else:
                do_fc_connect()

        def do_arduino_toggle() -> None:
            if simulation_mode_enabled():
                self.status.set("Turn off Simulate before connecting Arduino.")
                return
            if self.start_pending:
                return
            if arduino_output_connected():
                do_stop()
            else:
                do_start()

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

        def _task_hold(worker_self: SerialWorker, i: int, target: int, offset: int, timeout_s: float):
            if worker_self.ser is None:
                raise RuntimeError("Serial not open")
            quant, max_count = read_regs(worker_self.ser, REG_QUANT, 2)
            set_channel_until_stop_on_serial(worker_self.ser, quant, max_count, i, target, offset, timeout_s)
            return read_pulse_status_on_serial(worker_self.ser, max_count)

        def _task_fc_read_pid_ff(_worker_self: SerialWorker):
            return self.fc_service.read_roll_pitch_pid_ff(timeout_seconds=1.2)

        def _task_enter_msc_and_import_blackbox_logs(_worker_self: SerialWorker, fc_port_name: str, fc_baud_rate: int):
            msc_warnings: list[str] = []
            try:
                send_cli_msc_command(fc_port_name, fc_baud_rate)
            except Exception as exc:
                msc_warnings.append(f"Could not send CLI 'msc' on {fc_port_name}: {exc}")

            deadline = time.monotonic() + self.blackbox_msc_mount_timeout_s
            result: BlackboxImportResult | None = None
            while True:
                result = import_blackbox_logs_from_msc(self.blackbox_import_dir)
                if result.scanned_roots:
                    break
                if time.monotonic() >= deadline:
                    break
                time.sleep(self.blackbox_msc_mount_poll_s)

            if result is None:
                result = import_blackbox_logs_from_msc(self.blackbox_import_dir)
            if not msc_warnings:
                return result

            merged_warnings: list[str] = []
            for warning in [*msc_warnings, *result.warnings]:
                if warning and warning not in merged_warnings:
                    merged_warnings.append(warning)
            return BlackboxImportResult(
                scanned_roots=result.scanned_roots,
                imported_files=result.imported_files,
                skipped_count=result.skipped_count,
                warnings=tuple(merged_warnings),
                analysis_summary=result.analysis_summary,
                analysis_source=result.analysis_source,
                pid_report=result.pid_report,
            )

        def _task_analyze_blackbox_logs(_worker_self: SerialWorker):
            return analyze_pulled_blackbox_logs(self.blackbox_import_dir)

        def _task_analyze_specific_blackbox_log(_worker_self: SerialWorker, log_path: str):
            return analyze_blackbox_log(log_path, decode_destination_dir=self.blackbox_import_dir)

        def _is_auxiliary_blackbox_csv(path: Path) -> bool:
            lower = path.name.lower()
            return lower.endswith(".gps.csv") or lower.endswith(".event.csv") or lower.endswith(".events.csv")

        def _resolve_chart_source_path(analysis_result: BlackboxImportResult, preferred_log_path: str) -> str | None:
            candidates: list[Path] = []
            if preferred_log_path:
                candidates.append(Path(preferred_log_path))
            if analysis_result.analysis_source:
                candidates.append(Path(analysis_result.analysis_source))

            # Prefer any explicit, existing non-aux CSV path first.
            for candidate in candidates:
                if candidate.exists() and candidate.is_file() and candidate.suffix.lower() == ".csv":
                    if not _is_auxiliary_blackbox_csv(candidate):
                        return str(candidate)

            search_dirs: list[Path] = []
            for candidate in candidates:
                parent = candidate.parent
                if parent.exists() and parent not in search_dirs:
                    search_dirs.append(parent)
            if self.blackbox_import_dir.exists() and self.blackbox_import_dir not in search_dirs:
                search_dirs.append(self.blackbox_import_dir)

            stems: list[str] = []
            for candidate in candidates:
                stem = candidate.stem.strip()
                if stem and stem not in stems:
                    stems.append(stem)

            for search_dir in search_dirs:
                csv_candidates: list[Path] = []
                for stem in stems:
                    csv_candidates.extend(p for p in search_dir.glob(f"{stem}*.csv") if p.is_file())
                csv_candidates.extend(p for p in search_dir.glob("*.csv") if p.is_file())
                preferred_csvs = [p for p in csv_candidates if not _is_auxiliary_blackbox_csv(p)]
                if preferred_csvs:
                    return str(max(preferred_csvs, key=lambda p: p.stat().st_mtime))

            if analysis_result.analysis_source:
                return analysis_result.analysis_source
            return preferred_log_path or None

        def _task_generate_auto_report(
            _worker_self: SerialWorker,
            analysis_result: BlackboxImportResult,
            session_payload: dict[str, object],
            preferred_log_path: str,
        ):
            source_path = _resolve_chart_source_path(analysis_result, preferred_log_path)
            return generate_auto_tune_report(
                self.blackbox_import_dir,
                analysis_result,
                session_payload,
                source_path,
            )

        def _task_generate_step_response_report(_worker_self: SerialWorker, log_paths: list[str]):
            return generate_step_response_report(log_paths, self.blackbox_import_dir)

        def _task_hold_end(worker_self: SerialWorker, i: int):
            if worker_self.ser is None:
                raise RuntimeError("Serial not open")
            _, max_count = read_regs(worker_self.ser, REG_QUANT, 2)
            end_hold_on_serial(worker_self.ser, max_count, i)
            return read_pulse_status_on_serial(worker_self.ser, max_count)

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

        def do_start() -> None:
            try:
                if simulation_mode_enabled():
                    raise RuntimeError("Turn off Simulate before connecting Arduino.")
                if self.start_pending:
                    raise RuntimeError("Start is already in progress.")
                channels = parse_entries(self.ch_entries, int, "Channel")
                require_range(channels, "Channel", 1000, 2000)
                offsets = parse_entries(self.off_entries, int, "Offset")
                selected_port = port()
                if self.controller.is_connected and selected_port != self.controller.run_port:
                    raise RuntimeError(f"Output is active on {self.controller.run_port}. Press Disconnect Arduino before switching ports.")
                self.beeper_marker_active = False

                def on_start_done(ok: bool, res: object) -> None:
                    self.start_pending = False
                    if not ok:
                        update_link_indicators()
                        set_error("Start error", res if isinstance(res, Exception) else RuntimeError(res))
                        return
                    if (
                        not isinstance(res, tuple)
                        or len(res) != 3
                        or not isinstance(res[0], int)
                        or not isinstance(res[1], int)
                        or (res[2] is not None and not isinstance(res[2], str))
                    ):
                        update_link_indicators()
                        set_error("Start error", RuntimeError("Unexpected worker result from start task"))
                        return
                    self.base_channel_outputs = channels.copy()
                    set_live_channel_outputs(self.base_channel_outputs)
                    update_link_indicators()
                    version_warning = res[2]
                    if version_warning:
                        self.status.set(version_warning)
                        messagebox.showwarning("Firmware version", version_warning)
                    else:
                        self.status.set("PPM output configured and started.")

                self.start_pending = True
                update_link_indicators()
                self.controller.start_output(
                    selected_port,
                    channels,
                    offsets,
                    self.beeper_marker_active,
                    callback=on_start_done,
                )

            except Exception as exc:
                self.start_pending = False
                update_link_indicators()
                set_error("Start error", exc)

        def do_stop() -> None:
            try:
                def on_stop_done(ok: bool, res: object) -> None:
                    if not ok:
                        set_error("Stop error", res if isinstance(res, Exception) else RuntimeError(res))
                        return
                    if res is not None:
                        set_error("Stop error", RuntimeError("Unexpected worker result from stop task"))
                        return
                    self.beeper_marker_active = False
                    set_live_channel_outputs(parse_channel_values_with_defaults())
                    update_link_indicators()
                    self.status.set("PPM output stopped.")

                self.controller.stop_output(callback=on_stop_done)
            except Exception as exc:
                set_error("Stop error", exc)

        def on_close() -> None:
            self.is_closing = True
            cancel_adjust_repeat()
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
        self.fly_log_button.config(command=do_pid_plan_fly_log_toggle)
        self.simulation_mode_checkbutton.config(command=on_simulation_mode_changed)
        self.pid_progress_button.config(command=open_pid_progress_window)
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

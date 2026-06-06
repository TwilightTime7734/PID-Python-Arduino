"""Application runtime orchestration."""

from __future__ import annotations

import os
import math
from pathlib import Path
import queue
import shutil
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
    CHANNEL_DEFAULTS,
    FC_DEVICE_ID,
    FC_DEVICE_PID,
    FC_DEVICE_VID,
    FC_PORT_DEFAULT,
    HOLD_ANGLE_CHECK_MS,
    HOLD_TIMEOUT_POLL_MS,
    LEVEL_CENTER_US,
    LEVEL_DEADBAND_DEG,
    LEVEL_FULL_SCALE_DEG,
    LEVEL_LOOP_INTERVAL_MS,
    LEVEL_MAX_DELTA_US,
    LEVEL_MIN_DELTA_US,
    LEVEL_PULSE_TIMEOUT_S,
    LEVEL_TIMEOUT_MAX_S,
    LEVEL_TIMEOUT_MIN_S,
    OFFSET_DEFAULTS,
    PITCH_CHANNEL_INDEX,
    PORT_DEFAULT,
    PULSE_DURATION_DEFAULTS,
    PULSE_STATUS_HOLD_ENDED,
    PULSE_STATUS_REJECTED,
    PULSE_STATUS_TIMEOUT_RESTORED,
    PULSE_TARGET_DEFAULTS,
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
    set_channel_with_human_profile_until_stop_on_serial,
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
    require_duration_range,
    require_range,
)
from .worker import SerialWorker


def main() -> None:
    root = tk.Tk()
    ui = build_main_gui(root)
    port_entry = ui.port_entry
    channel_adjust_canvases = ui.channel_adjust_canvases
    target_adjust_canvases = ui.target_adjust_canvases
    ch_entries = ui.ch_entries
    off_entries = ui.off_entries
    target_entries = ui.target_entries
    dur_entries = ui.dur_entries
    angle_entries = ui.angle_entries
    channel_output_canvases = ui.channel_output_canvases
    channel_output_fill_ids = ui.channel_output_fill_ids
    hold_send_canvases = ui.hold_send_canvases
    level_button = ui.level_button
    status = ui.status
    pc_link_box = ui.pc_link_box
    horizon = ui.horizon
    roll_text = ui.roll_text
    pitch_text = ui.pitch_text
    roll_pidff_vars = ui.roll_pidff_vars
    pitch_pidff_vars = ui.pitch_pidff_vars
    pid_ff_adjust_canvases = ui.pid_ff_adjust_canvases
    fc_port_entry = ui.fc_port_entry
    fc_baud_entry = ui.fc_baud_entry
    scan_fc_button = ui.scan_fc_button
    connect_fc_button = ui.connect_fc_button
    import_blackbox_button = ui.import_blackbox_button
    analyze_blackbox_button = ui.analyze_blackbox_button
    arduino_button = ui.arduino_button
    auto_session_button = ui.auto_session_button
    auto_report_text = ui.auto_report_text
    auto_report_listbox = ui.auto_report_listbox
    auto_open_selected_button = ui.auto_open_selected_button
    auto_open_all_button = ui.auto_open_all_button
    auto_clear_reports_button = ui.auto_clear_reports_button
    fly_log_button = ui.fly_log_button
    simulate_auto_session_button = ui.simulate_auto_session_button
    step_response_button = ui.step_response_button
    pid_tuning_plan_button = ui.pid_tuning_plan_button

    run_active = False
    start_pending = False
    is_closing = False
    run_port = PORT_DEFAULT
    run_ser: serial.Serial | None = None
    run_quant: int | None = None
    run_max_count: int | None = None
    hold_timeout_after_id: str | None = None
    hold_command_inflight = False
    channel_update_inflight = False
    pending_channel_update_channels: list[int] | None = None
    pending_channel_update_offsets: list[int] | None = None
    adjust_repeat_after_id: str | None = None
    adjust_repeat_handler: Callable[[int, int], None] | None = None
    adjust_repeat_index: int | None = None
    adjust_repeat_delta = 0
    base_channel_outputs = CHANNEL_DEFAULTS.copy()
    live_channel_outputs = base_channel_outputs.copy()
    worker = SerialWorker()
    fc_service = InavSerialService()
    fc_poll_after_id: str | None = None
    level_active = False
    level_after_id: str | None = None
    level_pulse_inflight = False
    level_timeout_deadline_s: float | None = None
    level_timeout_s = max(PULSE_DURATION_DEFAULTS[ROLL_CHANNEL_INDEX], PULSE_DURATION_DEFAULTS[PITCH_CHANNEL_INDEX])
    auto_config = AdaptiveSessionConfig()
    auto_controller: AdaptiveExcitationController | None = None
    auto_state = AdaptiveSessionState.idle
    auto_stop_reason = ""
    auto_warning = ""
    auto_session_start_s: float | None = None
    auto_last_tick_s: float | None = None
    auto_last_sample_s: float | None = None
    auto_tick_after_id: str | None = None
    auto_hold_after_id: str | None = None
    auto_pulse_inflight = False
    auto_hold_end_requested = False
    auto_settle_until_s: float | None = None
    auto_recovery_mode = False
    auto_active_command: AdaptiveCommand | None = None
    auto_event_peak_delta = 0.0
    auto_event_response_delay_s: float | None = None
    auto_event_baseline = 0.0
    auto_event_start_s = 0.0
    auto_original_base_outputs: list[int] | None = None
    auto_start_throttle_us = base_channel_outputs[THROTTLE_CHANNEL_INDEX]
    auto_current_throttle_us = base_channel_outputs[THROTTLE_CHANNEL_INDEX]
    auto_peak_throttle_us = base_channel_outputs[THROTTLE_CHANNEL_INDEX]
    auto_latest_report: AutoTuneReport | None = None
    auto_report_files: list[str] = []
    auto_import_result: BlackboxImportResult | None = None
    auto_latest_imported_log: str = ""
    sim_active = False
    sim_after_id: str | None = None
    sim_plan: LoadedPIDTuningPlan | None = None
    sim_plan_steps: list[dict[str, object]] = []
    sim_plan_step_index = 0
    sim_waiting_for_fly_log = False
    sim_fly_log_active = False
    sim_step_started_s: float | None = None
    sim_step_duration_s = 7.5
    sim_roll_deg = 0.0
    sim_pitch_deg = 0.0
    sim_last_report_second = -1
    blackbox_import_inflight = False
    blackbox_import_dir = (Path(__file__).resolve().parent.parent / "blackbox_imports").resolve()
    blackbox_msc_mount_timeout_s = 12.0
    blackbox_msc_mount_poll_s = 1.0
    requested_pid_plan_path = (
        blackbox_import_dir / "reports" / "pid_tuning_plan_20260605_201036" / "pid_tuning_plan.txt"
    )
    pid_plan_active = False
    pid_plan: LoadedPIDTuningPlan | None = None
    pid_plan_phase = "idle"
    pid_plan_index = 0
    pid_plan_selected_d: int | None = None
    pid_plan_selected_p: dict[str, int] | None = None
    pid_plan_selected_i: dict[str, int] | None = None
    pid_plan_selected_ff: dict[str, int] | None = None
    pid_plan_waiting_for_fly_log = False
    pid_plan_current_candidate_title = ""
    pid_plan_fly_log_active = False
    pid_ff_labels = ("P", "I", "D", "FF")
    pid_ff_adjust_fields = [
        ("roll", "p"),
        ("pitch", "p"),
        ("roll", "i"),
        ("pitch", "i"),
        ("roll", "d"),
        ("pitch", "d"),
        ("roll", "ff"),
        ("pitch", "ff"),
    ]
    def port() -> str:
        return port_entry.get().strip() or PORT_DEFAULT

    def fc_port() -> str:
        return fc_port_entry.get().strip() or FC_PORT_DEFAULT

    def fc_baud() -> int:
        try:
            value = int(fc_baud_entry.get().strip())
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
        for label, var in zip(pid_ff_labels, roll_pidff_vars):
            var.set(f"{label}: --")
        for label, var in zip(pid_ff_labels, pitch_pidff_vars):
            var.set(f"{label}: --")

    def refresh_fly_log_button_state() -> None:
        if pid_plan_fly_log_active:
            fly_log_button.config(text="Abort Fly/Log", state="normal")
        elif sim_fly_log_active:
            fly_log_button.config(text="Stop Sim Fly/Log", state="normal")
        elif pid_plan_active and pid_plan_waiting_for_fly_log:
            fly_log_button.config(text="Fly/Log", state="normal")
        elif sim_plan is not None and sim_waiting_for_fly_log:
            fly_log_button.config(text="Fly/Log", state="normal")
        else:
            fly_log_button.config(text="Fly/Log", state="disabled")

    def set_pid_ff_displays(roll_values: AxisPidFf, pitch_values: AxisPidFf) -> None:
        roll_series = (roll_values.p, roll_values.i, roll_values.d, roll_values.ff)
        pitch_series = (pitch_values.p, pitch_values.i, pitch_values.d, pitch_values.ff)
        for label, value, var in zip(pid_ff_labels, roll_series, roll_pidff_vars):
            var.set(f"{label}: {format_pid_ff_value(value)}")
        for label, value, var in zip(pid_ff_labels, pitch_series, pitch_pidff_vars):
            var.set(f"{label}: {format_pid_ff_value(value)}")

    def set_auto_report_text(text: str) -> None:
        auto_report_text.config(state="normal")
        auto_report_text.delete("1.0", tk.END)
        auto_report_text.insert("1.0", text.strip() + ("\n" if text and not text.endswith("\n") else ""))
        auto_report_text.config(state="disabled")

    def refresh_auto_report_file_list() -> None:
        auto_report_listbox.delete(0, tk.END)
        for file_path in auto_report_files:
            auto_report_listbox.insert(tk.END, file_path)

    def open_local_path(path: str) -> None:
        if not path:
            return
        target = Path(path)
        if not target.exists():
            raise RuntimeError(f"File not found: {target}")
        if os.name == "nt":
            os.startfile(str(target))  # type: ignore[attr-defined]
            return
        raise RuntimeError("Opening files is currently supported only on Windows in this app.")

    def open_selected_report_file() -> None:
        try:
            selected = auto_report_listbox.curselection()
            if not selected:
                status.set("Select a report file first.")
                return
            file_path = auto_report_listbox.get(selected[0])
            open_local_path(file_path)
        except Exception as exc:
            set_error("Open report file error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def open_all_report_files() -> None:
        try:
            if not auto_report_files:
                status.set("No report files available yet.")
                return
            for file_path in auto_report_files:
                open_local_path(file_path)
        except Exception as exc:
            set_error("Open report files error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def remove_blackbox_child_path(path: Path, root_dir: Path) -> int:
        resolved_root = root_dir.resolve()
        if path.is_symlink() or path.is_file():
            path.unlink()
            return 1
        if path.is_dir():
            resolved_path = path.resolve()
            if resolved_root != resolved_path and resolved_root not in resolved_path.parents:
                raise RuntimeError(f"Refusing to remove path outside blackbox_imports: {path}")
            count = sum(1 for item in path.rglob("*") if item.is_file() or item.is_symlink())
            shutil.rmtree(path)
            return max(1, count)
        return 0

    def clear_blackbox_import_contents() -> int:
        root_dir = blackbox_import_dir.resolve()
        reports_dir = (root_dir / "reports").resolve()
        if root_dir.name.lower() != "blackbox_imports":
            raise RuntimeError(f"Unexpected blackbox import directory: {root_dir}")
        if reports_dir.parent != root_dir:
            raise RuntimeError(f"Unexpected reports directory: {reports_dir}")

        root_dir.mkdir(parents=True, exist_ok=True)
        reports_dir.mkdir(parents=True, exist_ok=True)

        removed_count = 0
        for child in list(root_dir.iterdir()):
            if child.resolve() == reports_dir:
                continue
            removed_count += remove_blackbox_child_path(child, root_dir)

        reports_dir.mkdir(parents=True, exist_ok=True)
        for child in list(reports_dir.iterdir()):
            removed_count += remove_blackbox_child_path(child, root_dir)
        reports_dir.mkdir(parents=True, exist_ok=True)
        return removed_count

    def do_clear_report_files() -> None:
        nonlocal auto_latest_report, auto_report_files, auto_import_result, auto_latest_imported_log
        try:
            if blackbox_import_inflight:
                status.set("Blackbox import/analyze is in progress; wait before clearing files.")
                return
            if auto_is_running():
                status.set("Auto session/pipeline is running; wait before clearing files.")
                return
            prompt = (
                f"Delete all files under:\n{blackbox_import_dir}\n\n"
                "This also clears generated reports. The folders themselves will be kept.\n"
                "This cannot be undone."
            )
            if not messagebox.askyesno("Clear Blackbox Files", prompt, icon="warning", parent=root):
                status.set("Clear canceled.")
                return

            removed_count = clear_blackbox_import_contents()
            auto_latest_report = None
            auto_report_files = []
            auto_import_result = None
            auto_latest_imported_log = ""
            refresh_auto_report_file_list()
            set_auto_report_text("Blackbox imports and generated reports cleared.")
            status.set(f"Cleared {removed_count} item(s) from blackbox_imports and reports.")
        except Exception as exc:
            set_error("Clear report files error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def auto_elapsed_s(now_s: float | None = None) -> float:
        if auto_session_start_s is None:
            return 0.0
        current = time.monotonic() if now_s is None else now_s
        return max(0.0, current - auto_session_start_s)

    def set_auto_state(next_state: AdaptiveSessionState, safety_text: str = "") -> None:
        nonlocal auto_state
        auto_state = next_state
        if safety_text and safety_text != "--":
            status.set(safety_text)

    def auto_session_payload() -> dict[str, object]:
        metrics: dict[str, object] = {}
        if auto_controller is not None:
            snapshot = auto_controller.coverage_metrics()
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
            "state": auto_state.value,
            "stop_reason": auto_stop_reason,
            "warning": auto_warning,
            "elapsed_s": auto_elapsed_s(),
            "metrics": metrics,
            "start_throttle_us": auto_start_throttle_us,
            "current_throttle_us": auto_current_throttle_us,
            "peak_throttle_us": auto_peak_throttle_us,
        }

    def refresh_pid_ff_from_fc(update_status: bool = False) -> bool:
        if not fc_service.is_connected:
            clear_pid_ff_displays()
            if update_status:
                status.set("FC is disconnected.")
            return False
        try:
            roll_values, pitch_values = fc_service.read_roll_pitch_pid_ff(timeout_seconds=1.2)
            set_pid_ff_displays(roll_values, pitch_values)
            if update_status:
                status.set("PID/FF refreshed from FC.")
            return True
        except Exception as exc:
            clear_pid_ff_displays()
            if update_status:
                set_error("PID/FF read error", exc)
            return False

    def queue_fc_pid_ff_refresh(connected_port: str, connected_baud: int) -> None:
        if not fc_service.is_connected:
            return

        def on_pid_ff_read_done(ok: bool, res: object) -> None:
            if not fc_service.is_connected:
                return
            if not ok:
                clear_pid_ff_displays()
                status.set(f"FC connected: {connected_port} @ {connected_baud}. PID/FF read failed.")
                return
            if (
                not isinstance(res, tuple)
                or len(res) != 2
                or not isinstance(res[0], AxisPidFf)
                or not isinstance(res[1], AxisPidFf)
            ):
                clear_pid_ff_displays()
                status.set(f"FC connected: {connected_port} @ {connected_baud}. PID/FF read failed.")
                return
            set_pid_ff_displays(res[0], res[1])
            status.set(f"FC connected: {connected_port} @ {connected_baud}. PID/FF loaded.")

        worker.submit(_task_fc_read_pid_ff, callback=on_pid_ff_read_done)

    def record_auto_session_sample(sample) -> None:
        nonlocal auto_last_sample_s, auto_event_peak_delta, auto_event_response_delay_s
        auto_last_sample_s = time.monotonic()
        command = auto_active_command
        if command is None:
            return
        if auto_controller is not None and auto_pulse_inflight:
            stop_limit = max(0.0, auto_controller.config.hard_limit_deg - auto_controller.config.safety_margin_deg)
            if abs(float(sample.roll_deg)) >= stop_limit or abs(float(sample.pitch_deg)) >= stop_limit:
                request_auto_angle_hold_end(command)
                return
        axis_value = pulse_axis_value(sample, command.axis)
        directed_delta = (axis_value - auto_event_baseline) * float(command.direction)
        if directed_delta > auto_event_peak_delta:
            auto_event_peak_delta = float(directed_delta)
        within_hold_window = (time.monotonic() - auto_event_start_s) <= command.hold_s
        target_peak_deg = command.target_peak_deg
        if target_peak_deg <= 0 and auto_controller is not None:
            target_peak_deg = auto_controller.config.axis_target_peak_max_deg(command.axis)
        if (
            auto_controller is not None
            and auto_pulse_inflight
            and within_hold_window
            and target_peak_deg > 0
            and directed_delta >= target_peak_deg
        ):
            request_auto_angle_hold_end(command)
        if auto_event_response_delay_s is None:
            threshold_deg = max(2.0, (command.force_us / 15.0) * 0.35)
            if directed_delta >= threshold_deg:
                auto_event_response_delay_s = max(0.0, time.monotonic() - auto_event_start_s)

    def cancel_auto_hold_timer() -> None:
        nonlocal auto_hold_after_id
        if auto_hold_after_id is not None:
            try:
                root.after_cancel(auto_hold_after_id)
            except Exception:
                pass
            finally:
                auto_hold_after_id = None

    def begin_auto_observe_window(command: AdaptiveCommand) -> None:
        nonlocal auto_pulse_inflight, auto_settle_until_s
        if not auto_is_running():
            return
        auto_pulse_inflight = False
        set_live_channel_outputs(base_channel_outputs)
        auto_settle_until_s = time.monotonic() + command.settle_s
        schedule_auto_tick(delay_ms=round(command.settle_s * 1000.0))

    def request_auto_angle_hold_end(command: AdaptiveCommand) -> None:
        nonlocal auto_hold_end_requested
        if auto_hold_end_requested or not auto_is_running() or run_ser is None:
            return
        auto_hold_end_requested = True
        cancel_auto_hold_timer()
        set_live_channel_outputs(base_channel_outputs)
        begin_auto_observe_window(command)
        channel_index = axis_channel_index(command.axis)
        target_angle = command.target_peak_deg
        if target_angle <= 0 and auto_controller is not None:
            target_angle = auto_controller.config.axis_target_peak_max_deg(command.axis)
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
            if run_active and run_ser is not None:
                queue_live_channel_update(base_channel_outputs.copy(), parse_offset_values_with_defaults())

        worker.submit(_task_hold_end, channel_index, callback=on_auto_hold_end_done)

    def draw_channel_output(index: int, value: int) -> None:
        clamped = max(1000, min(2000, value))
        canvas = channel_output_canvases[index]
        fill_id = channel_output_fill_ids[index]

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
        for i, entry in enumerate(ch_entries):
            try:
                values.append(int(entry.get().strip()))
            except ValueError:
                values.append(CHANNEL_DEFAULTS[i])
        return values

    def parse_offset_values_with_defaults() -> list[int]:
        values: list[int] = []
        for i, entry in enumerate(off_entries):
            try:
                values.append(int(entry.get().strip()))
            except ValueError:
                values.append(OFFSET_DEFAULTS[i])
        return values

    def adjust_channel_value(index: int, delta: int) -> None:
        try:
            current = int(ch_entries[index].get().strip())
        except ValueError:
            current = CHANNEL_DEFAULTS[index]
        updated = max(1000, min(2000, current + delta))
        ch_entries[index].delete(0, tk.END)
        ch_entries[index].insert(0, str(updated))
        on_output_inputs_changed()

    def adjust_target_value(index: int, delta: int) -> None:
        try:
            current = int(target_entries[index].get().strip())
        except ValueError:
            current = PULSE_TARGET_DEFAULTS[index]
        updated = max(0, min(500, current + delta))
        target_entries[index].delete(0, tk.END)
        target_entries[index].insert(0, str(updated))

    def get_adjust_delta(event: tk.Event, step: int = 5) -> int:
        width = int(event.widget.cget("width"))
        mid_x = width / 2
        return -step if event.x <= mid_x else step

    def get_pulse_action(event: tk.Event) -> str:
        width = int(event.widget.cget("width"))
        third_x = width / 3
        if event.x < third_x:
            return "negative"
        if event.x < (third_x * 2):
            return "end"
        return "positive"

    def cancel_adjust_repeat() -> None:
        nonlocal adjust_repeat_after_id, adjust_repeat_handler, adjust_repeat_index, adjust_repeat_delta
        if adjust_repeat_after_id is not None:
            try:
                root.after_cancel(adjust_repeat_after_id)
            except Exception:
                pass
            finally:
                adjust_repeat_after_id = None
        adjust_repeat_handler = None
        adjust_repeat_index = None
        adjust_repeat_delta = 0

    def schedule_adjust_repeat() -> None:
        nonlocal adjust_repeat_after_id
        if adjust_repeat_handler is None or adjust_repeat_index is None or adjust_repeat_delta == 0:
            adjust_repeat_after_id = None
            return
        adjust_repeat_handler(adjust_repeat_index, adjust_repeat_delta)
        adjust_repeat_after_id = root.after(ADJUST_REPEAT_INTERVAL_MS, schedule_adjust_repeat)

    def on_adjust_press(
        adjust_handler: Callable[[int, int], None],
        index: int,
        event: tk.Event,
        step: int = 5,
    ) -> None:
        nonlocal adjust_repeat_after_id, adjust_repeat_handler, adjust_repeat_index, adjust_repeat_delta
        cancel_adjust_repeat()
        delta = get_adjust_delta(event, step=step)
        adjust_handler(index, delta)
        adjust_repeat_handler = adjust_handler
        adjust_repeat_index = index
        adjust_repeat_delta = delta
        adjust_repeat_after_id = root.after(ADJUST_REPEAT_INITIAL_MS, schedule_adjust_repeat)

    def adjust_pid_ff_value(index: int, delta: int) -> None:
        if index < 0 or index >= len(pid_ff_adjust_fields):
            return
        if delta == 0:
            return
        if not fc_service.is_connected:
            status.set("Connect FC before adjusting PID/FF.")
            return
        axis, gain = pid_ff_adjust_fields[index]
        setting_name = FF_SETTING_NAME[axis] if gain == "ff" else PID_SETTING_NAME[(axis, gain)]
        try:
            current = int(fc_service.get_setting_int(setting_name, timeout_seconds=0.8))
            target = max(0, min(255, current + delta))
            if target == current:
                return
            _ = fc_service.set_setting_int(setting_name, target, timeout_seconds=0.9)
            if not refresh_pid_ff_from_fc(update_status=False):
                raise RuntimeError("Failed to refresh PID/FF from FC after update.")
            status.set(
                f"{axis.title()} {gain.upper()} set to {target} on FC."
            )
        except Exception as exc:
            set_error("PID/FF adjust error", exc)

    def on_adjust_release(_event: tk.Event) -> None:
        cancel_adjust_repeat()

    def set_live_channel_outputs(values: list[int]) -> None:
        nonlocal live_channel_outputs
        live_channel_outputs = values.copy()
        refresh_channel_outputs()

    def restore_base_outputs_after_hold(offsets: list[int] | None = None) -> None:
        if not run_active or run_ser is None:
            return
        restore_offsets = offsets.copy() if offsets is not None else parse_offset_values_with_defaults()
        set_live_channel_outputs(base_channel_outputs)
        queue_live_channel_update(base_channel_outputs.copy(), restore_offsets)

    def refresh_channel_outputs() -> None:
        for i, value in enumerate(live_channel_outputs):
            draw_channel_output(i, value)

    def queue_live_channel_update(channels: list[int], offsets: list[int]) -> None:
        nonlocal channel_update_inflight, pending_channel_update_channels, pending_channel_update_offsets
        if not run_active or run_ser is None:
            return
        if channel_update_inflight:
            pending_channel_update_channels = channels.copy()
            pending_channel_update_offsets = offsets.copy()
            return

        channel_update_inflight = True

        def on_live_update_done(ok: bool, res: object) -> None:
            nonlocal channel_update_inflight, pending_channel_update_channels, pending_channel_update_offsets
            nonlocal run_quant, run_max_count, base_channel_outputs
            channel_update_inflight = False
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
                    run_quant = res[0]
                    run_max_count = res[1]
                    sent_channels = [int(v) for v in res[2]]
                    base_channel_outputs = sent_channels
                    if hold_timeout_after_id is None:
                        set_live_channel_outputs(sent_channels)

            if not run_active or run_ser is None:
                pending_channel_update_channels = None
                pending_channel_update_offsets = None
                return

            if pending_channel_update_channels is None or pending_channel_update_offsets is None:
                return

            next_channels = pending_channel_update_channels
            next_offsets = pending_channel_update_offsets
            pending_channel_update_channels = None
            pending_channel_update_offsets = None
            queue_live_channel_update(next_channels, next_offsets)

        worker.submit(_task_update_channels, channels.copy(), offsets.copy(), callback=on_live_update_done)

    def set_channel_entry_value(index: int, value: int) -> None:
        ch_entries[index].delete(0, tk.END)
        ch_entries[index].insert(0, str(value))

    def apply_auto_base_outputs(channels: list[int], safety_text: str = "") -> None:
        nonlocal base_channel_outputs, auto_current_throttle_us, auto_peak_throttle_us
        clamped = [max(1000, min(2000, int(value))) for value in channels]
        base_channel_outputs = clamped.copy()
        if auto_original_base_outputs is not None:
            auto_current_throttle_us = clamped[THROTTLE_CHANNEL_INDEX]
            auto_peak_throttle_us = max(auto_peak_throttle_us, auto_current_throttle_us)
        for index, value in enumerate(clamped):
            set_channel_entry_value(index, value)
        set_live_channel_outputs(clamped)
        if run_active and run_ser is not None:
            queue_live_channel_update(clamped.copy(), parse_offset_values_with_defaults())
        if safety_text:
            status.set(safety_text)

    def restore_auto_original_base_outputs() -> None:
        nonlocal auto_original_base_outputs
        if auto_original_base_outputs is None:
            return
        original = auto_original_base_outputs
        auto_original_base_outputs = None
        apply_auto_base_outputs(original, "restored pre-auto outputs")

    def prepare_auto_throttle() -> bool:
        if auto_controller is None:
            return False
        current = base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        target, reason = auto_controller.initial_throttle(current)
        if target == current:
            return False
        channels = base_channel_outputs.copy()
        channels[THROTTLE_CHANNEL_INDEX] = target
        apply_auto_base_outputs(channels, reason)
        return True

    def adjust_auto_throttle_after_event(event: ExcitationEvent, recovery_event: bool) -> None:
        if auto_controller is None or recovery_event:
            return
        current = base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        target, reason = auto_controller.throttle_after_event(current, event)
        if target == current:
            return
        channels = base_channel_outputs.copy()
        channels[THROTTLE_CHANNEL_INDEX] = target
        apply_auto_base_outputs(channels, reason)

    def on_output_inputs_changed() -> None:
        nonlocal base_channel_outputs
        if not run_active or run_ser is None:
            set_live_channel_outputs(parse_channel_values_with_defaults())
            return

        try:
            channels = parse_entries(ch_entries, int, "Channel")
            require_range(channels, "Channel", 1000, 2000)
            offsets = parse_entries(off_entries, int, "Offset")
        except Exception:
            return

        set_live_channel_outputs(channels)
        base_channel_outputs = channels.copy()
        queue_live_channel_update(channels, offsets)

    def channel_angle_value(channel_index: int) -> float | None:
        sample = fc_service.latest_attitude()
        if sample is None:
            return None
        if channel_index == 0:
            return sample.roll_deg
        if channel_index == 1:
            return sample.pitch_deg
        if channel_index == 3:
            return sample.yaw_deg
        return None

    def is_angle_threshold_reached(channel_index: int, threshold_deg: float) -> bool:
        if threshold_deg == 0:
            return False
        measured = channel_angle_value(channel_index)
        if measured is None:
            return False
        if threshold_deg > 0:
            return measured >= threshold_deg
        return measured <= threshold_deg

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
        port_entry.config(values=values)
        fc_port_entry.config(values=values)

    def scan_fc_ports(update_status: bool = True) -> None:
        port_infos = sorted(
            list_ports.comports(),
            key=lambda p: str(getattr(p, "device", "") or "").upper(),
        )
        ports = list_scanned_ports(port_infos)
        populate_port_dropdowns(ports)
        selected_port = select_fc_port(port_infos)
        fc_port_entry.delete(0, tk.END)
        fc_port_entry.insert(0, selected_port)
        if update_status:
            if ports:
                status.set(f"Detected ports: {', '.join(ports)}. FC port set to {selected_port}.")
            else:
                status.set(f"No serial ports detected. FC port set to {selected_port}.")

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
        dialog = tk.Toplevel(root)
        dialog.title("PID Tuning Plan")
        dialog.transient(root)
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
        nonlocal auto_report_files
        try:
            if blackbox_import_inflight:
                status.set("Blackbox/report task already in progress.")
                return
            if auto_is_running():
                status.set("Wait for the auto session/pipeline to finish first.")
                return

            inputs = ask_pid_tuning_inputs()
            if inputs is None:
                status.set("PID tuning plan canceled.")
                return

            recommendation = suggest_starting_p(inputs)
            report = generate_pid_tuning_plan_report(blackbox_import_dir, recommendation)
            auto_report_files = [report.text_path, report.summary_json]
            refresh_auto_report_file_list()
            set_auto_report_text(Path(report.text_path).read_text(encoding="utf-8", errors="replace"))
            status.set(f"PID tuning plan generated: {report.report_dir}")
        except Exception as exc:
            set_error("PID tuning plan error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def locate_pid_tuning_plan_file() -> Path:
        if requested_pid_plan_path.exists():
            return requested_pid_plan_path
        latest = find_latest_pid_tuning_plan(blackbox_import_dir)
        if latest is not None:
            return latest
        raise RuntimeError(
            "No PID tuning plan file was found. Generate a PID Tuning Plan first."
        )

    def read_fc_pid_ff_values(axes: tuple[str, ...] = ("roll", "pitch", "yaw")) -> dict[str, dict[str, int]]:
        values: dict[str, dict[str, int]] = {}
        for axis in axes:
            values[axis] = {
                "p": int(fc_service.get_setting_int(PID_SETTING_NAME[(axis, "p")], timeout_seconds=1.0)),
                "i": int(fc_service.get_setting_int(PID_SETTING_NAME[(axis, "i")], timeout_seconds=1.0)),
                "d": int(fc_service.get_setting_int(PID_SETTING_NAME[(axis, "d")], timeout_seconds=1.0)),
                "ff": int(fc_service.get_setting_int(FF_SETTING_NAME[axis], timeout_seconds=1.0)),
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

    def ensure_disarmed_before_pid_write() -> bool:
        while True:
            try:
                is_armed = fc_service.is_armed(timeout_seconds=0.8)
            except Exception as exc:
                prompt = (
                    "Could not verify whether the drone is armed.\n\n"
                    f"{exc}\n\n"
                    "Cancel to stop, or continue only if you have confirmed the drone is disarmed."
                )
                return messagebox.askokcancel("Arm State Unknown", prompt, icon="warning", parent=root)

            if not is_armed:
                return True

            retry = messagebox.askretrycancel(
                "Drone Armed",
                "The FC reports the drone is armed.\n\n"
                "Disarm it before writing PID/FF values, then click Retry.",
                icon="warning",
                parent=root,
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
                confirmed = int(fc_service.set_setting_int(setting_name, value, timeout_seconds=1.2))
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
            parent=root,
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
        if pid_plan is None:
            return ()
        if len(pid_plan.d_sweep) <= 1:
            return ()
        return tuple(int(value) for value in pid_plan.d_sweep[1:])

    def pid_plan_p_candidates() -> tuple[dict[str, int], ...]:
        if pid_plan is None:
            return ()
        return tuple(
            {"roll": int(roll), "pitch": int(pitch)}
            for roll, pitch in zip(pid_plan.p_sweep.get("roll", ()), pid_plan.p_sweep.get("pitch", ()))
        )

    def pid_plan_d_recheck_candidates() -> tuple[int, ...]:
        if pid_plan_selected_d is None:
            return ()
        delta = 6
        values = (pid_plan_selected_d - delta, pid_plan_selected_d, pid_plan_selected_d + delta)
        return tuple(dict.fromkeys(max(0, min(255, int(value))) for value in values))

    def complete_pid_tuning_plan(message: str) -> None:
        nonlocal pid_plan_active, pid_plan_phase, pid_plan_index, pid_plan_waiting_for_fly_log
        nonlocal pid_plan_current_candidate_title
        pid_plan_active = False
        pid_plan_phase = "complete"
        pid_plan_index = 0
        pid_plan_waiting_for_fly_log = False
        pid_plan_current_candidate_title = ""
        set_auto_button_idle()
        refresh_fly_log_button_state()
        status.set(message)

    def prepare_pid_plan_next_step() -> bool:
        nonlocal pid_plan_phase, pid_plan_index, pid_plan_selected_d, pid_plan_selected_p
        nonlocal pid_plan_selected_i, pid_plan_selected_ff
        if pid_plan is None:
            raise RuntimeError("PID tuning plan is not loaded.")

        while True:
            if pid_plan_phase == "safe_start":
                return True

            if pid_plan_phase == "d_sweep" and pid_plan_index >= len(pid_plan_d_candidates()):
                optional_d = pid_plan.optional_d
                if optional_d is not None and optional_d not in pid_plan.d_sweep:
                    if messagebox.askyesno(
                        "Optional D Step",
                        f"The normal D sweep is complete.\n\nRun optional D {optional_d} before choosing D?",
                        parent=root,
                    ):
                        pid_plan_phase = "d_optional"
                        pid_plan_index = 0
                        return True
                chosen = ask_pid_value("Choose D", "Enter the best Roll/Pitch D from the D sweep.", pid_plan.d_sweep[0])
                if chosen is None:
                    status.set("PID plan paused; D winner is required before P sweep.")
                    return False
                pid_plan_selected_d = int(chosen)
                pid_plan_phase = "p_sweep"
                pid_plan_index = 0
                continue

            if pid_plan_phase == "d_optional" and pid_plan_index >= 1:
                initial = pid_plan.optional_d if pid_plan.optional_d is not None else pid_plan.d_sweep[0]
                chosen = ask_pid_value("Choose D", "Enter the best Roll/Pitch D from the D sweep.", int(initial))
                if chosen is None:
                    status.set("PID plan paused; D winner is required before P sweep.")
                    return False
                pid_plan_selected_d = int(chosen)
                pid_plan_phase = "p_sweep"
                pid_plan_index = 0
                continue

            if pid_plan_phase == "p_sweep" and pid_plan_index >= len(pid_plan_p_candidates()):
                candidates = pid_plan_p_candidates()
                initial = candidates[-1] if candidates else pid_plan.start_p
                selected = ask_pid_pair("Choose P", "P", initial["roll"], initial["pitch"])
                if selected is None:
                    status.set("PID plan paused; P winners are required before D re-check.")
                    return False
                pid_plan_selected_p = selected
                pid_plan_phase = "d_recheck"
                pid_plan_index = 0
                continue

            if pid_plan_phase == "d_recheck" and pid_plan_index >= len(pid_plan_d_recheck_candidates()):
                initial = pid_plan_selected_d if pid_plan_selected_d is not None else pid_plan.d_sweep[0]
                chosen = ask_pid_value("Choose Final D", "Enter the best Roll/Pitch D after re-check.", int(initial))
                if chosen is None:
                    status.set("PID plan paused; final D is required before I sweep.")
                    return False
                pid_plan_selected_d = int(chosen)
                pid_plan_phase = "i_sweep"
                pid_plan_index = 0
                continue

            if pid_plan_phase == "i_sweep" and pid_plan_index >= len(pid_plan.i_sweep):
                initial = pid_plan.i_sweep[-1] if pid_plan.i_sweep else {"roll": 60, "pitch": 65}
                selected = ask_pid_pair("Choose I", "I", initial["roll"], initial["pitch"])
                if selected is None:
                    status.set("PID plan paused; I winners are required before FF sweep.")
                    return False
                pid_plan_selected_i = selected
                pid_plan_phase = "ff_sweep"
                pid_plan_index = 0
                continue

            if pid_plan_phase == "ff_sweep" and pid_plan_index >= len(pid_plan.ff_sweep):
                initial = pid_plan.ff_sweep[-1] if pid_plan.ff_sweep else {"roll": 86, "pitch": 89}
                selected = ask_pid_pair("Choose FF", "FF", initial["roll"], initial["pitch"])
                if selected is None:
                    status.set("PID plan paused; FF winners are required before final write.")
                    return False
                pid_plan_selected_ff = selected
                pid_plan_phase = "final_write"
                pid_plan_index = 0
                continue

            return True

    def current_pid_plan_step() -> tuple[str, str, dict[str, dict[str, int]]] | None:
        if pid_plan is None:
            raise RuntimeError("PID tuning plan is not loaded.")
        start_d = int(pid_plan.d_sweep[0]) if pid_plan.d_sweep else 17

        if pid_plan_phase == "safe_start":
            target = roll_pitch_target(
                pid_plan.start_p["roll"],
                pid_plan.start_p["pitch"],
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

        if pid_plan_phase == "d_sweep":
            candidates = pid_plan_d_candidates()
            if pid_plan_index >= len(candidates):
                return None
            d_value = candidates[pid_plan_index]
            target = roll_pitch_target(
                pid_plan.start_p["roll"],
                pid_plan.start_p["pitch"],
                d_value,
                d_value,
                0,
                0,
                0,
                0,
            )
            return (f"D sweep {pid_plan_index + 2}/{len(pid_plan.d_sweep)}", f"Log Roll/Pitch D {d_value}.", target)

        if pid_plan_phase == "d_optional":
            if pid_plan.optional_d is None or pid_plan_index >= 1:
                return None
            d_value = int(pid_plan.optional_d)
            target = roll_pitch_target(
                pid_plan.start_p["roll"],
                pid_plan.start_p["pitch"],
                d_value,
                d_value,
                0,
                0,
                0,
                0,
            )
            return ("Optional D sweep", f"Log optional Roll/Pitch D {d_value}.", target)

        if pid_plan_phase == "p_sweep":
            if pid_plan_selected_d is None:
                return None
            candidates = pid_plan_p_candidates()
            if pid_plan_index >= len(candidates):
                return None
            row = candidates[pid_plan_index]
            target = roll_pitch_target(row["roll"], row["pitch"], pid_plan_selected_d, pid_plan_selected_d, 0, 0, 0, 0)
            return (
                f"P sweep {pid_plan_index + 1}/{len(candidates)}",
                f"Log Roll P {row['roll']} and Pitch P {row['pitch']} with D {pid_plan_selected_d}.",
                target,
            )

        if pid_plan_phase == "d_recheck":
            if pid_plan_selected_p is None:
                return None
            candidates = pid_plan_d_recheck_candidates()
            if pid_plan_index >= len(candidates):
                return None
            d_value = candidates[pid_plan_index]
            target = roll_pitch_target(
                pid_plan_selected_p["roll"],
                pid_plan_selected_p["pitch"],
                d_value,
                d_value,
                0,
                0,
                0,
                0,
            )
            return (
                f"D re-check {pid_plan_index + 1}/{len(candidates)}",
                f"Log Roll/Pitch D {d_value} with chosen P.",
                target,
            )

        if pid_plan_phase == "i_sweep":
            if pid_plan_selected_p is None or pid_plan_selected_d is None:
                return None
            if pid_plan_index >= len(pid_plan.i_sweep):
                return None
            row = pid_plan.i_sweep[pid_plan_index]
            target = roll_pitch_target(
                pid_plan_selected_p["roll"],
                pid_plan_selected_p["pitch"],
                pid_plan_selected_d,
                pid_plan_selected_d,
                row["roll"],
                row["pitch"],
                0,
                0,
            )
            return (
                f"I sweep {pid_plan_index + 1}/{len(pid_plan.i_sweep)}",
                f"Log Roll I {row['roll']} and Pitch I {row['pitch']}.",
                target,
            )

        if pid_plan_phase == "ff_sweep":
            if pid_plan_selected_p is None or pid_plan_selected_d is None or pid_plan_selected_i is None:
                return None
            if pid_plan_index >= len(pid_plan.ff_sweep):
                return None
            row = pid_plan.ff_sweep[pid_plan_index]
            target = roll_pitch_target(
                pid_plan_selected_p["roll"],
                pid_plan_selected_p["pitch"],
                pid_plan_selected_d,
                pid_plan_selected_d,
                pid_plan_selected_i["roll"],
                pid_plan_selected_i["pitch"],
                row["roll"],
                row["pitch"],
            )
            return (
                f"FF sweep {pid_plan_index + 1}/{len(pid_plan.ff_sweep)}",
                f"Log Roll FF {row['roll']} and Pitch FF {row['pitch']}.",
                target,
            )

        return None

    def advance_pid_plan_after_step() -> None:
        nonlocal pid_plan_phase, pid_plan_index
        if pid_plan_phase == "safe_start":
            pid_plan_phase = "d_sweep"
            pid_plan_index = 0
            return
        pid_plan_index += 1

    def run_pid_plan_final_write() -> None:
        if (
            pid_plan is None
            or pid_plan_selected_p is None
            or pid_plan_selected_d is None
            or pid_plan_selected_i is None
            or pid_plan_selected_ff is None
        ):
            raise RuntimeError("PID plan final values are incomplete.")

        roll_pitch = roll_pitch_target(
            pid_plan_selected_p["roll"],
            pid_plan_selected_p["pitch"],
            pid_plan_selected_d,
            pid_plan_selected_d,
            pid_plan_selected_i["roll"],
            pid_plan_selected_i["pitch"],
            pid_plan_selected_ff["roll"],
            pid_plan_selected_ff["pitch"],
        )
        with_yaw = dict(roll_pitch)
        with_yaw["yaw"] = dict(pid_plan.yaw_final_pid_ff)
        current = read_fc_pid_ff_values()
        set_pid_plan_report_text(pid_plan, "PID plan final write", with_yaw, current)
        prompt = (
            "The roll/pitch sweeps are complete.\n\n"
            "DISARM before writing final PID/FF values. The app will verify disarmed state before writing.\n\n"
            "Yes: write chosen roll/pitch values and the conservative yaw recommendation while disarmed.\n"
            "No: write chosen roll/pitch values only while disarmed.\n"
            "Cancel: stop without writing final values.\n\n"
            "Current vs target:\n"
            f"{format_pid_target_check(current, with_yaw)}"
        )
        choice = messagebox.askyesnocancel("PID Plan Final Values", prompt, parent=root)
        if choice is None:
            complete_pid_tuning_plan("PID tuning plan stopped before final write.")
            return
        if not ensure_disarmed_before_pid_write():
            status.set("PID final write canceled; disarm before writing values.")
            return
        write_fc_pid_ff_values(with_yaw if choice else roll_pitch)
        refresh_pid_ff_from_fc(update_status=False)
        complete_pid_tuning_plan("PID tuning plan complete.")
        messagebox.showinfo(
            "PID Plan Complete",
            "Final selected values were written. Review and save in INAV only when you are satisfied.",
            parent=root,
        )

    def continue_pid_tuning_plan() -> None:
        nonlocal pid_plan_waiting_for_fly_log, pid_plan_current_candidate_title
        if not pid_plan_active:
            return
        if pid_plan is None:
            raise RuntimeError("PID tuning plan is not loaded.")
        if pid_plan_waiting_for_fly_log:
            messagebox.showinfo(
                "Fly/Log Needed",
                f"{pid_plan_current_candidate_title or 'The current candidate'} is ready.\n\n"
                "Arm the drone, press Fly/Log, then disarm the drone before pressing Next PID Plan Step.",
                parent=root,
            )
            status.set("Press Fly/Log for the current candidate before moving to the next step.")
            return
        if not prepare_pid_plan_next_step():
            return
        if pid_plan_phase == "final_write":
            run_pid_plan_final_write()
            return

        step = current_pid_plan_step()
        if step is None:
            status.set("PID plan is waiting for the next stage choice.")
            return
        title, instruction, target = step
        if pid_plan_phase == "safe_start":
            set_pid_plan_report_text(pid_plan, f"PID plan step: {title}", target)
            if not ensure_disarmed_before_pid_write():
                status.set("Safe-start PID write waiting; disarm the drone before starting the plan.")
                return
            current = read_fc_pid_ff_values(tuple(target.keys()))
            set_pid_plan_report_text(pid_plan, f"PID plan step: {title}", target, current)
            write_fc_pid_ff_values(target)
            refresh_pid_ff_from_fc(update_status=False)
            advance_pid_plan_after_step()
            pid_plan_waiting_for_fly_log = True
            pid_plan_current_candidate_title = title
            auto_session_button.config(text="Next PID Plan Step", state="normal")
            refresh_fly_log_button_state()
            status.set("Safe-start PID/FF values written while disarmed.")
            messagebox.showinfo(
                "Safe Start Ready",
                "Safe-start PID/FF values were written while the drone was disarmed.\n\n"
                "Now arm the drone, press Fly/Log, then disarm the drone before pressing Next PID Plan Step.",
                parent=root,
            )
            return

        if not ensure_disarmed_before_pid_write():
            status.set("PID plan paused; disarm the drone before moving to the next candidate.")
            return
        current = read_fc_pid_ff_values(tuple(target.keys()))
        set_pid_plan_report_text(pid_plan, f"PID plan step: {title}", target, current)
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
        choice = messagebox.askyesnocancel("PID Plan Step", prompt, parent=root)
        if choice is None:
            complete_pid_tuning_plan("PID tuning plan stopped by user.")
            return
        if choice:
            if not ensure_disarmed_before_pid_write():
                status.set("PID write canceled; disarm before writing values.")
                return
            write_fc_pid_ff_values(target)
            refresh_pid_ff_from_fc(update_status=False)
            pid_plan_waiting_for_fly_log = True
            pid_plan_current_candidate_title = title
            status.set(f"PID plan step written: {title}")
        else:
            pid_plan_waiting_for_fly_log = False
            pid_plan_current_candidate_title = ""
            status.set(f"PID plan step skipped: {title}")
        advance_pid_plan_after_step()
        auto_session_button.config(text="Next PID Plan Step", state="normal")
        refresh_fly_log_button_state()
        messagebox.showinfo(
            "PID Plan Step Ready",
            (
                "Values are ready for this candidate.\n\n"
                "Now arm the drone, press Fly/Log, then disarm the drone before pressing Next PID Plan Step."
                if choice
                else "This candidate was skipped. Press Next PID Plan Step when ready for the next candidate."
            ),
            parent=root,
        )

    def start_pid_tuning_plan_session() -> None:
        nonlocal pid_plan_active, pid_plan, pid_plan_phase, pid_plan_index
        nonlocal pid_plan_selected_d, pid_plan_selected_p, pid_plan_selected_i, pid_plan_selected_ff
        nonlocal pid_plan_waiting_for_fly_log, pid_plan_current_candidate_title
        nonlocal auto_report_files
        plan_path = locate_pid_tuning_plan_file()
        pid_plan = load_pid_tuning_plan(plan_path)
        pid_plan_active = True
        pid_plan_phase = "safe_start"
        pid_plan_index = 0
        pid_plan_selected_d = None
        pid_plan_selected_p = None
        pid_plan_selected_i = None
        pid_plan_selected_ff = None
        pid_plan_waiting_for_fly_log = False
        pid_plan_current_candidate_title = ""
        auto_report_files = [
            path for path in (pid_plan.text_path, pid_plan.summary_json) if path
        ]
        refresh_auto_report_file_list()
        set_pid_plan_report_text(pid_plan, "PID tuning plan loaded")
        auto_session_button.config(text="Next PID Plan Step", state="normal")
        refresh_fly_log_button_state()
        status.set(f"PID tuning plan loaded: {pid_plan.text_path}")
        continue_pid_tuning_plan()

    def read_fc_armed_state_for_blackbox_import(selected_port: str, selected_baud: int) -> bool:
        if fc_service.is_connected:
            return fc_service.is_armed(timeout_seconds=0.8)

        checker = InavSerialService()
        try:
            checker.connect(selected_port, selected_baud)
            return checker.is_armed(timeout_seconds=0.8)
        finally:
            checker.disconnect()

    def ensure_disarmed_before_blackbox_import(selected_port: str, selected_baud: int) -> bool:
        while True:
            try:
                is_armed = read_fc_armed_state_for_blackbox_import(selected_port, selected_baud)
            except Exception as exc:
                prompt = (
                    "Could not verify whether the drone is armed.\n\n"
                    f"{exc}\n\n"
                    "Cancel to stop, or continue only if you have confirmed the drone is disarmed."
                )
                return messagebox.askokcancel("Arm State Unknown", prompt, icon="warning", parent=root)

            if not is_armed:
                return True

            retry = messagebox.askretrycancel(
                "Drone Armed",
                "The FC reports the drone is armed.\n\n"
                "Disarm it before pulling Blackbox logs or entering MSC mode, then click Retry.",
                icon="warning",
                parent=root,
            )
            if not retry:
                return False

    def do_pull_blackbox_logs() -> None:
        nonlocal blackbox_import_inflight
        try:
            if blackbox_import_inflight:
                status.set("Blackbox import already in progress.")
                return

            selected_port = fc_port()
            selected_baud = fc_baud()
            if not ensure_disarmed_before_blackbox_import(selected_port, selected_baud):
                status.set("Blackbox import canceled; disarm the drone before pulling logs.")
                return
            if fc_service.is_connected:
                do_fc_disconnect(update_status=False)

            blackbox_import_inflight = True
            status.set(f"Requesting FC MSC mode on {selected_port} @ {selected_baud}, then scanning mounted volumes...")

            def on_pull_done(ok: bool, res: object) -> None:
                nonlocal blackbox_import_inflight
                blackbox_import_inflight = False
                if not ok:
                    set_error("Blackbox import error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, BlackboxImportResult):
                    set_error("Blackbox import error", RuntimeError("Unexpected import task result."))
                    return

                imported_count = len(res.imported_files)
                if imported_count == 0:
                    if res.skipped_count > 0:
                        status.set(
                            f"No new Blackbox logs were copied ({res.skipped_count} duplicate file(s) skipped)."
                        )
                    else:
                        status.set("No new Blackbox logs were imported from MSC volumes.")
                else:
                    if res.skipped_count > 0:
                        status.set(
                            f"Imported {imported_count} Blackbox file(s) to {blackbox_import_dir} "
                            f"({res.skipped_count} duplicate file(s) skipped)."
                        )
                    else:
                        status.set(
                            f"Imported {imported_count} Blackbox file(s) to {blackbox_import_dir}."
                        )

                set_auto_report_text(format_blackbox_report(res))

            worker.submit(_task_enter_msc_and_import_blackbox_logs, selected_port, selected_baud, callback=on_pull_done)
        except Exception as exc:
            blackbox_import_inflight = False
            set_error("Blackbox import error", exc)

    def do_analyze_blackbox_logs() -> None:
        nonlocal blackbox_import_inflight, auto_latest_report, auto_report_files
        try:
            if blackbox_import_inflight:
                status.set("Blackbox import already in progress.")
                return

            initial_dir = blackbox_import_dir if blackbox_import_dir.exists() else Path.cwd()
            selected_log = filedialog.askopenfilename(
                parent=root,
                title="Select Blackbox Log to Analyze",
                initialdir=str(initial_dir),
                filetypes=(
                    ("Blackbox logs", "*.bbl *.bfl *.bbs *.txt *.csv"),
                    ("All files", "*.*"),
                ),
            )
            if not selected_log:
                status.set("Blackbox analysis canceled.")
                return

            blackbox_import_inflight = True
            selected_name = Path(selected_log).name
            status.set(f"Analyzing Blackbox log: {selected_name}...")

            def on_analyze_done(ok: bool, res: object) -> None:
                nonlocal blackbox_import_inflight, auto_latest_report, auto_report_files
                if not ok:
                    blackbox_import_inflight = False
                    set_error("Blackbox analyze error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, BlackboxImportResult):
                    blackbox_import_inflight = False
                    set_error("Blackbox analyze error", RuntimeError("Unexpected analysis task result."))
                    return

                summary = res.analysis_summary
                summary_head = summary.split("|", 1)[0].strip()
                if res.pid_report is not None and res.pid_report.headline:
                    summary_head = res.pid_report.headline
                status.set(f"Blackbox analysis complete: {summary_head}. Generating report...")
                set_auto_report_text(format_blackbox_report(res))
                auto_latest_report = None
                auto_report_files = []
                refresh_auto_report_file_list()

                session_payload = {
                    "state": "manual_analyze",
                    "stop_reason": "Manual Analyze Logs run",
                    "warning": "",
                    "elapsed_s": 0.0,
                    "metrics": {},
                }

                def on_report_done(ok2: bool, res2: object) -> None:
                    nonlocal blackbox_import_inflight, auto_latest_report, auto_report_files
                    blackbox_import_inflight = False
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
                        status.set("Blackbox analysis complete, but report generation failed.")
                        set_auto_report_text(f"{format_blackbox_report(res)}\n\nReport generation error: {error_text}")
                        return
                    if not isinstance(res2, AutoTuneReport):
                        set_error("Blackbox report error", RuntimeError("Unexpected report task result."))
                        return

                    auto_latest_report = res2
                    report_files = [
                        res2.summary_txt,
                        res2.summary_json,
                        res2.combined_chart_sheet,
                        *list(res2.chart_paths),
                    ]
                    auto_report_files = [path for path in dict.fromkeys(item for item in report_files if item)]
                    refresh_auto_report_file_list()
                    try:
                        report_text = Path(res2.summary_txt).read_text(encoding="utf-8", errors="replace")
                    except Exception:
                        report_text = f"Report generated at {res2.report_dir}\nSummary file: {res2.summary_txt}"
                    set_auto_report_text(report_text)
                    status.set(f"Blackbox report generated: {res2.report_dir}")

                worker.submit(
                    _task_generate_auto_report,
                    res,
                    session_payload,
                    selected_log,
                    callback=on_report_done,
                )

            worker.submit(_task_analyze_specific_blackbox_log, selected_log, callback=on_analyze_done)
        except Exception as exc:
            blackbox_import_inflight = False
            set_error("Blackbox analyze error", exc)

    def do_step_response_report() -> None:
        nonlocal blackbox_import_inflight, auto_report_files
        try:
            if blackbox_import_inflight:
                status.set("Blackbox import/analyze already in progress.")
                return
            if auto_is_running():
                raise RuntimeError("Wait for the auto session/pipeline to finish first.")

            initial_dir = blackbox_import_dir if blackbox_import_dir.exists() else Path.cwd()
            selected_logs = filedialog.askopenfilenames(
                parent=root,
                title=f"Select Blackbox Logs for Step Response (max {MAX_STEP_RESPONSE_LOGS})",
                initialdir=str(initial_dir),
                filetypes=(
                    ("Blackbox logs", "*.bbl *.bfl *.bbs *.txt *.csv"),
                    ("All files", "*.*"),
                ),
            )
            if not selected_logs:
                status.set("Step response canceled.")
                return
            if len(selected_logs) > MAX_STEP_RESPONSE_LOGS:
                raise RuntimeError(f"Select at most {MAX_STEP_RESPONSE_LOGS} Blackbox logs.")

            blackbox_import_inflight = True
            step_response_button.config(state="disabled")
            count = len(selected_logs)
            status.set(f"Generating step response report for {count} log file(s)...")
            set_auto_report_text(
                f"Step response generation started for {count} log file(s).\n"
                "Raw logs will be decoded with tools/blackbox_decode_INAV.exe."
            )

            def on_step_response_done(ok: bool, res: object) -> None:
                nonlocal blackbox_import_inflight, auto_report_files
                blackbox_import_inflight = False
                step_response_button.config(state="normal")
                if not ok:
                    set_error("Step response error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, StepResponseReport):
                    set_error("Step response error", RuntimeError("Unexpected step-response task result."))
                    return

                report_files = [
                    res.html_path,
                    res.summary_json,
                    *list(res.decoded_csv_paths),
                ]
                auto_report_files = [path for path in dict.fromkeys(item for item in report_files if item)]
                refresh_auto_report_file_list()
                set_auto_report_text(format_step_response_report(res))
                status.set(f"Step response report generated: {res.report_dir}")

            worker.submit(_task_generate_step_response_report, list(selected_logs), callback=on_step_response_done)
        except Exception as exc:
            blackbox_import_inflight = False
            step_response_button.config(state="normal")
            set_error("Step response error", exc)

    def auto_is_running() -> bool:
        return auto_state in {
            AdaptiveSessionState.adaptive_run,
            AdaptiveSessionState.recovery,
            AdaptiveSessionState.finalize,
            AdaptiveSessionState.import_analyze,
        }

    def schedule_auto_tick(delay_ms: int | None = None) -> None:
        nonlocal auto_tick_after_id
        if not auto_is_running():
            return
        if auto_tick_after_id is not None:
            try:
                root.after_cancel(auto_tick_after_id)
            except Exception:
                pass
        cadence_ms = max(10, round(auto_config.control_interval_s * 1000.0))
        auto_tick_after_id = root.after(cadence_ms if delay_ms is None else max(1, delay_ms), run_auto_tick)

    def stop_auto_session_runtime() -> None:
        nonlocal auto_tick_after_id, auto_pulse_inflight, auto_hold_end_requested, auto_settle_until_s, auto_active_command
        if auto_tick_after_id is not None:
            try:
                root.after_cancel(auto_tick_after_id)
            except Exception:
                pass
            auto_tick_after_id = None
        cancel_auto_hold_timer()
        restore_auto_original_base_outputs()
        auto_pulse_inflight = False
        auto_hold_end_requested = False
        auto_settle_until_s = None
        auto_active_command = None

    def set_auto_button_idle() -> None:
        auto_session_button.config(text="Start Auto Session", state="normal")

    def complete_auto_session(next_state: AdaptiveSessionState, reason: str, warning: str = "") -> None:
        nonlocal auto_stop_reason, auto_warning
        auto_stop_reason = reason
        auto_warning = warning
        stop_auto_session_runtime()
        if run_active and run_ser is not None and hold_timeout_after_id is None:
            try:
                restore_base_outputs_after_hold()
            except Exception:
                pass
        set_auto_state(next_state, warning or reason)

    def auto_abort(reason: str, warning: str = "", continue_pipeline: bool = False) -> None:
        nonlocal pid_plan_fly_log_active
        complete_auto_session(AdaptiveSessionState.aborted, reason, warning)
        pid_plan_fly_log_active = False
        refresh_fly_log_button_state()
        status.set(f"Auto session aborted: {reason}")
        set_auto_button_idle()
        if pid_plan_active:
            auto_session_button.config(text="Next PID Plan Step", state="normal")
        if continue_pipeline:
            begin_auto_pipeline()

    def start_auto_session() -> None:
        nonlocal auto_config, auto_controller, auto_session_start_s, auto_last_tick_s
        nonlocal auto_last_sample_s, auto_stop_reason, auto_warning, auto_latest_report
        nonlocal auto_report_files, auto_import_result, auto_latest_imported_log, auto_original_base_outputs
        nonlocal auto_start_throttle_us, auto_current_throttle_us, auto_peak_throttle_us
        if blackbox_import_inflight:
            raise RuntimeError("Blackbox import/analyze is in progress.")
        if not fc_service.is_connected:
            raise RuntimeError("Connect FC before starting auto session.")
        if level_active:
            raise RuntimeError("Stop auto-level before starting auto session.")
        if fc_service.latest_attitude() is None:
            raise RuntimeError("No FC attitude sample yet. Wait for telemetry then retry.")
        auto_config = read_auto_tune_config()

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
            status.set("Auto session start canceled.")
            return
        start_pid_tuning_plan_session()

    def finalize_auto_event() -> None:
        nonlocal auto_active_command, auto_hold_end_requested, auto_event_peak_delta, auto_event_response_delay_s
        nonlocal auto_event_baseline, auto_event_start_s
        if auto_controller is None or auto_active_command is None:
            return
        sample = fc_service.latest_attitude()
        if sample is None:
            return
        recovery_event = auto_active_command.recovery
        axis_value = pulse_axis_value(sample, auto_active_command.axis)
        final_error = axis_value - auto_event_baseline
        settle_success = abs(final_error) <= auto_config.settle_deadband_deg
        event = ExcitationEvent(
            axis=auto_active_command.axis,
            direction=auto_active_command.direction,
            force_us=auto_active_command.force_us,
            hold_s=auto_active_command.hold_s,
            settle_s=auto_active_command.settle_s,
            baseline_angle_deg=auto_event_baseline,
            peak_delta_deg=auto_event_peak_delta,
            settle_success=settle_success,
            response_delay_s=auto_event_response_delay_s,
            final_error_deg=final_error,
        )
        auto_controller.record_event(event)
        adjust_auto_throttle_after_event(event, recovery_event)
        auto_active_command = None
        auto_hold_end_requested = False
        auto_event_peak_delta = 0.0
        auto_event_response_delay_s = None
        auto_event_baseline = 0.0
        auto_event_start_s = 0.0

    def issue_auto_command(command: AdaptiveCommand) -> None:
        nonlocal auto_hold_after_id, auto_pulse_inflight, auto_hold_end_requested, auto_active_command, auto_settle_until_s
        nonlocal auto_event_peak_delta, auto_event_response_delay_s, auto_event_baseline, auto_event_start_s
        if run_ser is None:
            raise RuntimeError("Arduino output is disconnected.")
        if auto_controller is None:
            raise RuntimeError("Adaptive controller is not initialized.")

        sample = fc_service.latest_attitude()
        if sample is None:
            raise RuntimeError("No FC attitude sample available.")

        channel_index = axis_channel_index(command.axis)
        target = base_channel_outputs[channel_index] + (command.direction * command.force_us)
        target = max(1000, min(2000, target))
        offsets = parse_offset_values_with_defaults()

        active_outputs = base_channel_outputs.copy()
        active_outputs[channel_index] = target
        set_live_channel_outputs(active_outputs)
        auto_pulse_inflight = True
        auto_hold_end_requested = False
        auto_settle_until_s = None
        auto_active_command = command
        auto_event_peak_delta = 0.0
        auto_event_response_delay_s = None
        auto_event_baseline = pulse_axis_value(sample, command.axis)
        auto_event_start_s = time.monotonic()

        def on_auto_hold_elapsed() -> None:
            nonlocal auto_hold_after_id
            auto_hold_after_id = None
            if not auto_is_running() or auto_active_command is not command:
                return
            begin_auto_observe_window(command)

        def on_auto_hold_done(ok: bool, res: object) -> None:
            nonlocal auto_hold_after_id, auto_pulse_inflight
            if not auto_is_running():
                return
            if not ok:
                auto_pulse_inflight = False
                auto_abort(
                    "Pulse command failed during auto session.",
                    warning=str(res) if not isinstance(res, Exception) else str(res),
                )
                return
            if not isinstance(res, int):
                auto_pulse_inflight = False
                auto_abort("Unexpected pulse result from worker.")
                return
            if res == PULSE_STATUS_REJECTED:
                auto_pulse_inflight = False
                auto_abort("Firmware rejected adaptive pulse command.")
                return
            if auto_hold_end_requested:
                return
            auto_hold_after_id = root.after(round(command.hold_s * 1000.0), on_auto_hold_elapsed)

        worker.submit(_task_hold, channel_index, target, offsets[channel_index], command.hold_s, callback=on_auto_hold_done)

    def run_auto_tick() -> None:
        nonlocal auto_tick_after_id, auto_recovery_mode, pid_plan_fly_log_active, pid_plan_waiting_for_fly_log
        nonlocal pid_plan_current_candidate_title
        auto_tick_after_id = None
        if not auto_is_running():
            return
        if auto_controller is None:
            auto_abort("Adaptive controller was not initialized.")
            return
        if not run_active or run_ser is None:
            auto_abort("Arduino output disconnected during auto session.")
            return
        if not fc_service.is_connected:
            auto_abort("FC disconnected during auto session.")
            return

        now = time.monotonic()
        if auto_last_sample_s is None or (now - auto_last_sample_s) > auto_config.telemetry_stale_s:
            auto_abort("FC telemetry became stale.", continue_pipeline=False)
            return

        sample = fc_service.latest_attitude()
        if sample is None:
            schedule_auto_tick()
            return

        abort, abort_reason = auto_controller.should_abort(sample.roll_deg, sample.pitch_deg)
        if abort:
            auto_abort(abort_reason, continue_pipeline=not pid_plan_fly_log_active)
            return

        if auto_controller.should_recover(sample.roll_deg, sample.pitch_deg):
            auto_recovery_mode = True
            set_auto_state(AdaptiveSessionState.recovery, "Recovery mode")
        elif auto_recovery_mode and auto_controller.recovery_complete(sample.roll_deg, sample.pitch_deg):
            auto_recovery_mode = False
            set_auto_state(AdaptiveSessionState.adaptive_run, "Active")

        if auto_pulse_inflight:
            schedule_auto_tick()
            return

        if auto_active_command is not None and auto_settle_until_s is not None and now < auto_settle_until_s:
            schedule_auto_tick()
            return

        if auto_active_command is not None and auto_settle_until_s is not None and now >= auto_settle_until_s:
            finalize_auto_event()
            auto_settle_until_s = None

        ready, stop_reason, warning = auto_controller.stop_ready(auto_elapsed_s(now))
        if ready:
            if pid_plan_fly_log_active:
                complete_auto_session(AdaptiveSessionState.report_ready, stop_reason, warning)
                pid_plan_fly_log_active = False
                pid_plan_waiting_for_fly_log = False
                completed_title = pid_plan_current_candidate_title
                pid_plan_current_candidate_title = ""
                auto_session_button.config(text="Next PID Plan Step", state="normal")
                refresh_fly_log_button_state()
                status.set(f"Fly/Log complete: {completed_title or stop_reason}")
                messagebox.showinfo(
                    "Fly/Log Complete",
                    "Fly/Log movement is complete.\n\n"
                    "Disarm the drone now. After it is disarmed, press Next PID Plan Step.",
                    parent=root,
                )
            else:
                complete_auto_session(AdaptiveSessionState.finalize, stop_reason, warning)
                status.set(f"Auto session complete: {stop_reason}")
                begin_auto_pipeline()
            return

        command = auto_controller.next_command(sample.roll_deg, sample.pitch_deg, auto_recovery_mode)
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
        nonlocal blackbox_import_inflight
        if blackbox_import_inflight:
            auto_warning_text = "Blackbox pipeline already in progress."
            status.set(auto_warning_text)
            return
        try:
            selected_port = fc_port()
            selected_baud = fc_baud()
            if not ensure_disarmed_before_blackbox_import(selected_port, selected_baud):
                set_auto_button_idle()
                status.set("Auto blackbox pipeline canceled; disarm the drone before pulling logs.")
                return
            if fc_service.is_connected:
                do_fc_disconnect(update_status=False)
            blackbox_import_inflight = True
            set_auto_state(AdaptiveSessionState.import_analyze, "Import/analyze running")
            auto_session_button.config(text="Running Analysis...", state="disabled")
            status.set("Auto session finished. Pulling and analyzing blackbox logs...")

            def on_auto_pull_done(ok: bool, res: object) -> None:
                nonlocal blackbox_import_inflight, auto_latest_imported_log
                if not ok:
                    blackbox_import_inflight = False
                    auto_abort(
                        "Auto pipeline failed while pulling blackbox logs.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                if not isinstance(res, BlackboxImportResult):
                    blackbox_import_inflight = False
                    auto_abort("Unexpected pull result in auto pipeline.")
                    return
                auto_latest_imported_log = ""
                if res.imported_files:
                    newest = max(res.imported_files, key=lambda item: item.modified_epoch_s)
                    auto_latest_imported_log = newest.local_path
                elif res.analysis_source:
                    auto_latest_imported_log = res.analysis_source
                if auto_latest_imported_log:
                    worker.submit(_task_analyze_specific_blackbox_log, auto_latest_imported_log, callback=on_auto_analyze_done)
                else:
                    worker.submit(_task_analyze_blackbox_logs, callback=on_auto_analyze_done)

            def on_auto_analyze_done(ok: bool, res: object) -> None:
                nonlocal blackbox_import_inflight, auto_import_result
                if not ok:
                    blackbox_import_inflight = False
                    auto_abort(
                        "Auto pipeline failed while analyzing blackbox logs.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                if not isinstance(res, BlackboxImportResult):
                    blackbox_import_inflight = False
                    auto_abort("Unexpected analyze result in auto pipeline.")
                    return
                auto_import_result = res
                set_auto_report_text(format_blackbox_report(res))
                worker.submit(
                    _task_generate_auto_report,
                    res,
                    auto_session_payload(),
                    auto_latest_imported_log,
                    callback=on_auto_report_done,
                )

            def on_auto_report_done(ok: bool, res: object) -> None:
                nonlocal blackbox_import_inflight, auto_latest_report, auto_report_files
                blackbox_import_inflight = False
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
                auto_latest_report = res
                report_files = [
                    res.summary_txt,
                    res.summary_json,
                    res.combined_chart_sheet,
                    *list(res.chart_paths),
                ]
                auto_report_files = [path for path in dict.fromkeys(item for item in report_files if item)]
                refresh_auto_report_file_list()
                try:
                    report_text = Path(res.summary_txt).read_text(encoding="utf-8", errors="replace")
                except Exception:
                    report_text = f"Report generated at {res.report_dir}\nSummary file: {res.summary_txt}"
                set_auto_report_text(report_text)
                set_auto_state(AdaptiveSessionState.report_ready, "Ready")
                status.set(f"Auto report ready: {res.report_dir}")

            worker.submit(_task_enter_msc_and_import_blackbox_logs, selected_port, selected_baud, callback=on_auto_pull_done)
        except Exception as exc:
            blackbox_import_inflight = False
            auto_abort("Unable to start auto blackbox pipeline.", warning=str(exc))

    def fc_is_armed_for_fly_log() -> bool:
        try:
            if fc_service.is_armed(timeout_seconds=0.8):
                return True
        except Exception as exc:
            messagebox.showerror("Fly/Log Arm Check", f"Could not verify armed state:\n\n{exc}", parent=root)
            return False
        messagebox.showwarning(
            "Fly/Log Requires Armed",
            "The FC reports the drone is not armed.\n\n"
            "Arm the drone first, then press Fly/Log.",
            parent=root,
        )
        return False

    def start_pid_plan_fly_log() -> None:
        nonlocal auto_config, auto_controller, auto_session_start_s, auto_last_tick_s, auto_last_sample_s
        nonlocal auto_stop_reason, auto_warning, auto_original_base_outputs
        nonlocal auto_start_throttle_us, auto_current_throttle_us, auto_peak_throttle_us
        nonlocal pid_plan_fly_log_active
        if not pid_plan_active or not pid_plan_waiting_for_fly_log:
            status.set("No PID plan candidate is ready for Fly/Log.")
            return
        if blackbox_import_inflight:
            status.set("Blackbox/report task already in progress.")
            return
        if auto_is_running():
            status.set("Auto/FlyLog task already running.")
            return
        if sim_active:
            status.set("Stop simulation before Fly/Log.")
            return
        if not run_active or run_ser is None:
            raise RuntimeError("Connect Arduino output before Fly/Log.")
        if not fc_service.is_connected:
            raise RuntimeError("Connect FC before Fly/Log.")
        if level_active:
            raise RuntimeError("Stop auto-level before Fly/Log.")
        if hold_command_inflight or hold_timeout_after_id is not None:
            raise RuntimeError("Wait for active pulse/hold command to complete first.")
        if fc_service.latest_attitude() is None:
            raise RuntimeError("No FC attitude sample yet. Wait for telemetry then retry.")
        if not fc_is_armed_for_fly_log():
            return

        auto_config = read_auto_tune_config()
        prompt = (
            f"Fly/Log candidate: {pid_plan_current_candidate_title or 'current PID plan step'}\n\n"
            "The FC reports ARMED.\n\n"
            "Pressing OK will send bounded roll/pitch stick movement through the Arduino output "
            "for about 60 seconds so Blackbox has movement to log.\n\n"
            "No PID/FF values will be written while armed.\n\n"
            "Keep the drone secured, keep the area clear, and be ready to disarm."
        )
        if not messagebox.askokcancel("Start Fly/Log", prompt, icon="warning", parent=root):
            status.set("Fly/Log canceled.")
            return

        auto_controller = AdaptiveExcitationController(auto_config)
        auto_original_base_outputs = base_channel_outputs.copy()
        auto_start_throttle_us = base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        auto_current_throttle_us = auto_start_throttle_us
        auto_peak_throttle_us = auto_start_throttle_us
        auto_stop_reason = ""
        auto_warning = ""
        auto_session_start_s = time.monotonic()
        auto_last_tick_s = auto_session_start_s
        auto_last_sample_s = auto_session_start_s
        pid_plan_fly_log_active = True
        set_auto_state(AdaptiveSessionState.adaptive_run, "Fly/Log active")
        throttle_prepared = prepare_auto_throttle()
        auto_session_button.config(text="Abort Fly/Log", state="normal")
        refresh_fly_log_button_state()
        status.set("Fly/Log movement active. Disarm after it completes.")
        set_auto_report_text(
            "Fly/Log active\n\n"
            f"Candidate: {pid_plan_current_candidate_title or 'current PID plan step'}\n"
            "The app is sending bounded roll/pitch movement for Blackbox logging.\n"
            "No PID/FF values are being written while armed.\n\n"
            "When complete, disarm the drone before pressing Next PID Plan Step."
        )
        schedule_auto_tick(delay_ms=250 if throttle_prepared else None)

    def do_pid_plan_fly_log_toggle() -> None:
        nonlocal pid_plan_fly_log_active
        try:
            if sim_fly_log_active:
                stop_simulated_auto_session("Simulated Fly/Log stopped.", restore_display=False)
                return
            if sim_plan is not None and sim_waiting_for_fly_log:
                start_simulated_fly_log()
                return
            if pid_plan_fly_log_active:
                auto_abort("Fly/Log aborted by user.", continue_pipeline=False)
                pid_plan_fly_log_active = False
                refresh_fly_log_button_state()
                return
            start_pid_plan_fly_log()
        except Exception as exc:
            set_error("Fly/Log error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def set_sim_attitude_display() -> None:
        horizon.set_attitude(sim_roll_deg, sim_pitch_deg)
        roll_text.set(f"Roll: {sim_roll_deg:6.1f} deg")
        pitch_text.set(f"Pitch: {sim_pitch_deg:6.1f} deg")

    def restore_neutral_sim_display() -> None:
        horizon.set_attitude(0.0, 0.0)
        roll_text.set("Roll: 0.0 deg")
        pitch_text.set("Pitch: 0.0 deg")
        clear_pid_ff_displays()

    def set_sim_pid_ff_display(target: dict[str, dict[str, int]]) -> None:
        for label, gain, var in zip(pid_ff_labels, ("p", "i", "d", "ff"), roll_pidff_vars):
            value = target.get("roll", {}).get(gain)
            var.set(f"{label}: --" if value is None else f"{label}: {value}")
        for label, gain, var in zip(pid_ff_labels, ("p", "i", "d", "ff"), pitch_pidff_vars):
            value = target.get("pitch", {}).get(gain)
            var.set(f"{label}: --" if value is None else f"{label}: {value}")

    def simulation_hardware_is_disconnected() -> bool:
        if run_active or run_ser is not None:
            messagebox.showwarning(
                "Simulation Requires No Hardware",
                "Disconnect Arduino output before using the simulator.",
                parent=root,
            )
            return False
        if fc_service.is_connected:
            messagebox.showwarning(
                "Simulation Requires No Hardware",
                "Disconnect the FC before using the simulator. Simulation uses synthetic attitude and PID boxes only.",
                parent=root,
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
        return tuple(dict.fromkeys(max(0, min(255, int(value))) for value in (selected_d - 6, selected_d, selected_d + 6)))

    def current_sim_step() -> dict[str, object] | None:
        if sim_plan_step_index < 0 or sim_plan_step_index >= len(sim_plan_steps):
            return None
        return sim_plan_steps[sim_plan_step_index]

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

    def update_simulated_plan_attitude(elapsed_s: float, step: dict[str, object]) -> None:
        nonlocal sim_roll_deg, sim_pitch_deg
        target = step["target"]
        if not isinstance(target, dict):
            sim_roll_deg = 0.0
            sim_pitch_deg = 0.0
            return
        stage = str(step.get("stage", ""))
        roll_gains = target.get("roll", {}) if isinstance(target.get("roll", {}), dict) else {}
        pitch_gains = target.get("pitch", {}) if isinstance(target.get("pitch", {}), dict) else {}
        sim_roll_deg = _sim_axis_wave(elapsed_s, 0.25, 1, roll_gains, stage)
        sim_pitch_deg = _sim_axis_wave(elapsed_s, 3.85, -1, pitch_gains, stage)
        limit = 35.0
        sim_roll_deg = max(-limit, min(limit, sim_roll_deg))
        sim_pitch_deg = max(-limit, min(limit, sim_pitch_deg))

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
        display_step_number = sim_plan_step_index + 1 if step_number is None else step_number
        lines = [
            "Simulated PID tuning plan step",
            f"Step {display_step_number} of {len(sim_plan_steps)}: {step['title']}",
            f"Plan file: {sim_plan.text_path if sim_plan is not None else '--'}",
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
            f"Elapsed: {elapsed_s:4.1f}s / {sim_step_duration_s:.1f}s",
            f"Roll:  {sim_roll_deg:+5.1f} deg",
            f"Pitch: {sim_pitch_deg:+5.1f} deg",
        ]
        if note:
            lines.extend(["", note])
        if fly_log_running:
            lines.extend(["", "When simulated Fly/Log finishes, press Next Sim Step to preview the next tuning candidate."])
        else:
            lines.extend(["", "Press Fly/Log to stimulate the simulated drone for this candidate."])
        set_auto_report_text("\n".join(lines))

    def stop_simulated_auto_session(message: str = "", restore_display: bool = True, clear_walkthrough: bool = False) -> None:
        nonlocal sim_active, sim_after_id, sim_step_started_s, sim_last_report_second
        nonlocal sim_plan, sim_plan_steps, sim_plan_step_index, sim_waiting_for_fly_log, sim_fly_log_active
        if sim_after_id is not None:
            try:
                root.after_cancel(sim_after_id)
            except Exception:
                pass
        sim_active = False
        sim_fly_log_active = False
        sim_after_id = None
        sim_step_started_s = None
        sim_last_report_second = -1
        if clear_walkthrough:
            sim_plan = None
            sim_plan_steps = []
            sim_plan_step_index = 0
            sim_waiting_for_fly_log = False
        if sim_waiting_for_fly_log:
            simulate_auto_session_button.config(text="Next Sim Step", state="disabled")
        elif sim_plan is not None and sim_plan_step_index < len(sim_plan_steps):
            simulate_auto_session_button.config(text="Next Sim Step", state="normal")
        else:
            simulate_auto_session_button.config(text="Simulate", state="normal")
        refresh_fly_log_button_state()
        if restore_display:
            restore_neutral_sim_display()
        if message and not is_closing:
            status.set(message)

    def finish_simulated_plan_step(step: dict[str, object]) -> None:
        nonlocal sim_active, sim_after_id, sim_step_started_s, sim_plan_step_index
        nonlocal sim_waiting_for_fly_log, sim_fly_log_active
        completed_step_number = sim_plan_step_index + 1
        sim_active = False
        sim_fly_log_active = False
        sim_waiting_for_fly_log = False
        sim_after_id = None
        sim_step_started_s = None
        sim_plan_step_index += 1
        if sim_plan_step_index >= len(sim_plan_steps):
            simulate_auto_session_button.config(text="Simulate", state="normal")
            refresh_fly_log_button_state()
            status.set("PID plan simulation complete.")
            refresh_sim_report(sim_step_duration_s, step, fly_log_running=False, step_number=completed_step_number)
            return
        simulate_auto_session_button.config(text="Next Sim Step", state="normal")
        refresh_fly_log_button_state()
        status.set(f"Simulation step complete. Next: {sim_plan_steps[sim_plan_step_index]['title']}")
        refresh_sim_report(sim_step_duration_s, step, fly_log_running=False, step_number=completed_step_number)

    def run_simulated_auto_tick() -> None:
        nonlocal sim_after_id, sim_last_report_second
        sim_after_id = None
        if not sim_active:
            return
        step = current_sim_step()
        if step is None or sim_step_started_s is None:
            stop_simulated_auto_session("Simulation stopped: no plan step is loaded.", clear_walkthrough=True)
            return

        elapsed_s = max(0.0, time.monotonic() - sim_step_started_s)
        update_simulated_plan_attitude(elapsed_s, step)
        set_sim_attitude_display()
        report_second = int(elapsed_s)
        if report_second != sim_last_report_second:
            sim_last_report_second = report_second
            refresh_sim_report(elapsed_s, step, fly_log_running=True)
        if elapsed_s >= sim_step_duration_s:
            finish_simulated_plan_step(step)
            return
        sim_after_id = root.after(40, run_simulated_auto_tick)

    def start_simulated_plan_step() -> None:
        nonlocal sim_active, sim_step_started_s, sim_roll_deg, sim_pitch_deg, sim_last_report_second
        nonlocal sim_waiting_for_fly_log, sim_fly_log_active
        if not simulation_hardware_is_disconnected():
            return
        if auto_is_running() or pid_plan_active or blackbox_import_inflight:
            status.set("Wait for the active auto/PID/log task to finish before simulating.")
            return
        step = current_sim_step()
        if step is None:
            stop_simulated_auto_session("PID plan simulation complete.", restore_display=True, clear_walkthrough=True)
            return
        target = step["target"]
        if isinstance(target, dict):
            set_sim_pid_ff_display(target)
        sim_active = False
        sim_fly_log_active = False
        sim_waiting_for_fly_log = True
        sim_step_started_s = None
        sim_roll_deg = 0.0
        sim_pitch_deg = 0.0
        sim_last_report_second = -1
        simulate_auto_session_button.config(text="Next Sim Step", state="disabled")
        refresh_fly_log_button_state()
        status.set(f"Simulated values staged: {step['title']}. Press Fly/Log.")
        refresh_sim_report(0.0, step, fly_log_running=False)
        set_sim_attitude_display()

    def start_simulated_fly_log() -> None:
        nonlocal sim_active, sim_step_started_s, sim_roll_deg, sim_pitch_deg, sim_last_report_second
        nonlocal sim_waiting_for_fly_log, sim_fly_log_active
        if not simulation_hardware_is_disconnected():
            return
        step = current_sim_step()
        if step is None or not sim_waiting_for_fly_log:
            status.set("No simulated candidate is staged for Fly/Log.")
            return
        sim_active = True
        sim_fly_log_active = True
        sim_step_started_s = time.monotonic()
        sim_roll_deg = 0.0
        sim_pitch_deg = 0.0
        sim_last_report_second = -1
        simulate_auto_session_button.config(text="Next Sim Step", state="disabled")
        refresh_fly_log_button_state()
        status.set(f"Simulated Fly/Log running: {step['title']}")
        refresh_sim_report(0.0, step, fly_log_running=True)
        set_sim_attitude_display()
        run_simulated_auto_tick()

    def start_simulated_auto_session() -> None:
        nonlocal sim_plan, sim_plan_steps, sim_plan_step_index
        if not simulation_hardware_is_disconnected():
            return
        plan_path = locate_pid_tuning_plan_file()
        sim_plan = load_pid_tuning_plan(plan_path)
        sim_plan_steps = build_simulated_pid_plan_steps(sim_plan)
        sim_plan_step_index = 0
        if not sim_plan_steps:
            raise RuntimeError("PID tuning plan has no steps to simulate.")
        auto_report_files[:] = [path for path in (sim_plan.text_path, sim_plan.summary_json) if path]
        refresh_auto_report_file_list()
        start_simulated_plan_step()

    def do_simulated_auto_session_toggle() -> None:
        try:
            if sim_active:
                stop_simulated_auto_session("Simulation stopped.")
                return
            if sim_waiting_for_fly_log:
                messagebox.showinfo(
                    "Simulated Fly/Log Needed",
                    "The simulated PID/FF values are staged.\n\nPress Fly/Log to stimulate the simulated drone before moving to the next step.",
                    parent=root,
                )
                status.set("Press Fly/Log before the next simulated step.")
                return
            if sim_plan is not None and sim_plan_step_index < len(sim_plan_steps):
                start_simulated_plan_step()
                return
            start_simulated_auto_session()
        except Exception as exc:
            stop_simulated_auto_session("", restore_display=True, clear_walkthrough=True)
            set_error("Simulation error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def do_auto_session_toggle() -> None:
        try:
            if sim_active:
                stop_simulated_auto_session("Simulation stopped.", clear_walkthrough=True)
            if auto_is_running():
                auto_abort("Auto session aborted by user.", continue_pipeline=not pid_plan_fly_log_active)
                return
            if auto_state == AdaptiveSessionState.import_analyze:
                status.set("Auto pipeline is running; wait for completion.")
                return
            if pid_plan_active:
                continue_pid_tuning_plan()
                return
            start_auto_session()
        except Exception as exc:
            set_error("Auto session error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def set_error(title: str, exc: Exception) -> None:
        if is_closing:
            return
        status.set("Error")
        messagebox.showerror(title, str(exc))

    def update_link_indicators() -> None:
        if run_ser is not None:
            pc_link_box.config(text="PC-ARD OPEN", bg="#2E7D32", fg="white")
        else:
            pc_link_box.config(text="PC-ARD CLOSED", bg="#8B1E1E", fg="white")
        fc_connected = fc_service.is_connected
        if fc_connected:
            connect_fc_button.config(
                text="Disconnect FC",
                state="normal",
                bg="#BEEAC4",
                activebackground="#A6E1AE",
                fg="#0E2F11",
                activeforeground="#0E2F11",
            )
        else:
            connect_fc_button.config(
                text="Connect FC",
                state="normal",
                bg="#F3C1C1",
                activebackground="#ECA8A8",
                fg="#3A1111",
                activeforeground="#3A1111",
            )
        arduino_connected = run_active and run_ser is not None
        if start_pending:
            arduino_button.config(
                text="Connecting...",
                state="disabled",
                bg="#F3E6B3",
                activebackground="#EBD997",
                fg="#3F3210",
                activeforeground="#3F3210",
            )
        elif arduino_connected:
            arduino_button.config(
                text="Disconnect Arduino",
                state="normal",
                bg="#BEEAC4",
                activebackground="#A6E1AE",
                fg="#0E2F11",
                activeforeground="#0E2F11",
            )
        else:
            arduino_button.config(
                text="Connect Arduino",
                state="normal",
                bg="#F3C1C1",
                activebackground="#ECA8A8",
                fg="#3A1111",
                activeforeground="#3A1111",
            )
        angle_state = "normal" if fc_connected else "disabled"
        for entry in angle_entries:
            entry.config(state=angle_state)
        level_ready = run_ser is not None and fc_connected
        if level_active and not level_ready:
            stop_level_loop(update_status=False)
        level_button.config(
            state="normal" if level_ready else "disabled",
            relief="sunken" if level_active else "raised",
        )
        if auto_state == AdaptiveSessionState.import_analyze:
            auto_session_button.config(text="Running Analysis...", state="disabled")
        elif auto_is_running():
            auto_session_button.config(text="Abort Fly/Log" if pid_plan_fly_log_active else "Abort Auto Session", state="normal")
        elif pid_plan_active:
            auto_session_button.config(text="Next PID Plan Step", state="normal")
        else:
            auto_session_button.config(text="Start Auto Session", state="normal")
        refresh_fly_log_button_state()

    def cancel_hold_timeout() -> None:
        nonlocal hold_timeout_after_id
        if hold_timeout_after_id is not None:
            try:
                root.after_cancel(hold_timeout_after_id)
            except Exception:
                pass
            finally:
                hold_timeout_after_id = None

    def cancel_level_timer() -> None:
        nonlocal level_after_id
        if level_after_id is not None:
            try:
                root.after_cancel(level_after_id)
            except Exception:
                pass
            finally:
                level_after_id = None

    def stop_level_loop(update_status: bool = False, reason: str = "Auto-level stopped.") -> None:
        nonlocal level_active, level_pulse_inflight, level_timeout_deadline_s
        was_active = level_active
        cancel_level_timer()
        level_active = False
        level_pulse_inflight = False
        level_timeout_deadline_s = None
        if hold_timeout_after_id is None:
            set_live_channel_outputs(base_channel_outputs)
        update_link_indicators()
        if update_status and was_active and not is_closing:
            status.set(reason)

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
        nonlocal level_after_id
        cancel_level_timer()
        level_after_id = root.after(max(1, delay_ms), run_level_step)

    def run_level_step() -> None:
        nonlocal level_after_id, level_pulse_inflight
        level_after_id = None
        if not level_active:
            return
        if not run_active or run_ser is None:
            stop_level_loop(update_status=True, reason="Auto-level stopped: output is not running.")
            return
        if not fc_service.is_connected:
            stop_level_loop(update_status=True, reason="Auto-level stopped: FC is disconnected.")
            return
        if level_timeout_deadline_s is not None and time.monotonic() >= level_timeout_deadline_s:
            stop_level_loop(update_status=True, reason=f"Auto-level timed out after {level_timeout_s:.3g}s.")
            return
        if hold_timeout_after_id is not None or level_pulse_inflight:
            schedule_level_step()
            return

        sample = fc_service.latest_attitude()
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
            offsets = parse_entries(off_entries, int, "Offset")
        except Exception as exc:
            stop_level_loop(update_status=False)
            set_error("Level error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))
            return

        active_outputs = base_channel_outputs.copy()
        active_outputs[channel_index] = target_us
        set_live_channel_outputs(active_outputs)
        level_pulse_inflight = True

        def on_level_pulse_done(ok: bool, res: object) -> None:
            nonlocal level_pulse_inflight
            level_pulse_inflight = False
            if not level_active:
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

        worker.submit(
            _task_hold,
            channel_index,
            target_us,
            offsets[channel_index],
            LEVEL_PULSE_TIMEOUT_S,
            callback=on_level_pulse_done,
        )

    def do_level() -> None:
        nonlocal level_active, level_timeout_deadline_s, level_timeout_s
        try:
            if level_active:
                stop_level_loop(update_status=True)
                return
            if not run_active or run_ser is None:
                raise RuntimeError("Press Connect Arduino before using Level.")
            if not fc_service.is_connected:
                raise RuntimeError("Connect FC before using Level.")
            if hold_command_inflight:
                raise RuntimeError("Wait for active Pulse command to finish.")
            if hold_timeout_after_id is not None:
                raise RuntimeError("Wait for active Hold to finish or press ∅/Stop.")
            if fc_service.latest_attitude() is None:
                raise RuntimeError("No FC attitude sample yet. Wait a moment, then press Level again.")
            durations = parse_entries(dur_entries, float, "Duration")
            level_timeout_s = max(durations[ROLL_CHANNEL_INDEX], durations[PITCH_CHANNEL_INDEX])
            if level_timeout_s < LEVEL_TIMEOUT_MIN_S or level_timeout_s > LEVEL_TIMEOUT_MAX_S:
                raise RuntimeError(
                    f"Duration CH1/CH2 must be between {LEVEL_TIMEOUT_MIN_S:.3g}s and {LEVEL_TIMEOUT_MAX_S:.3g}s."
                )
            level_active = True
            level_timeout_deadline_s = time.monotonic() + level_timeout_s
            update_link_indicators()
            status.set(f"Auto-level active ({level_timeout_s:.3g}s timeout). Press Level again to stop.")
            run_level_step()
        except Exception as exc:
            stop_level_loop(update_status=False)
            set_error("Level error", exc)

    def close_run_connection() -> None:
        nonlocal run_ser, run_quant, run_max_count, run_active, hold_command_inflight
        nonlocal channel_update_inflight, pending_channel_update_channels, pending_channel_update_offsets
        if run_ser is not None:
            try:
                run_ser.close()
            except Exception:
                pass
            finally:
                run_ser = None
                run_quant = None
                run_max_count = None
                run_active = False
                hold_command_inflight = False
                channel_update_inflight = False
                pending_channel_update_channels = None
                pending_channel_update_offsets = None
                update_link_indicators()

    def do_fc_connect() -> None:
        try:
            if fc_service.is_connected:
                return
            selected_port = fc_port()
            selected_baud = fc_baud()
            fc_service.connect(selected_port, selected_baud)
            # Mirror Usb2Arduino flow: verify telemetry immediately, then load PID/FF asynchronously.
            _ = fc_service.read_attitude(timeout_seconds=2.0)
            update_link_indicators()
            status.set(f"FC connected: {selected_port} @ {selected_baud}. Loading PID/FF...")
            queue_fc_pid_ff_refresh(selected_port, selected_baud)
        except Exception as exc:
            set_error("FC connect error", exc)

    def do_fc_disconnect(update_status: bool = True) -> None:
        if auto_state in (AdaptiveSessionState.adaptive_run, AdaptiveSessionState.recovery):
            auto_abort("FC disconnected during adaptive session.", continue_pipeline=False)
        try:
            fc_service.disconnect()
        except Exception as exc:
            if not is_closing:
                set_error("FC disconnect error", exc)
        finally:
            horizon.set_attitude(0.0, 0.0)
            roll_text.set("Roll: 0.0 deg")
            pitch_text.set("Pitch: 0.0 deg")
            clear_pid_ff_displays()
            update_link_indicators()
            if update_status and not is_closing:
                status.set("FC disconnected.")

    def do_fc_toggle() -> None:
        if fc_service.is_connected:
            do_fc_disconnect()
        else:
            do_fc_connect()

    def do_arduino_toggle() -> None:
        if start_pending:
            return
        if run_active and run_ser is not None:
            do_stop()
        else:
            do_start()

    def poll_fc_attitude() -> None:
        nonlocal fc_poll_after_id
        try:
            sample = fc_service.latest_attitude()
            if sample is not None:
                record_auto_session_sample(sample)
                if not sim_active:
                    horizon.set_attitude(sample.roll_deg, sample.pitch_deg)
                    roll_text.set(f"Roll: {sample.roll_deg:6.1f} deg")
                    pitch_text.set(f"Pitch: {sample.pitch_deg:6.1f} deg")
        except Exception:
            pass
        fc_poll_after_id = root.after(60, poll_fc_attitude)

    def _task_open_and_start(worker_self: SerialWorker, port: str, channels: list[int], offsets: list[int]):
        ser = open_serial(port)
        worker_self.ser = ser
        try:
            quant, max_count, version_warning = run_ppm_on_serial(ser, channels, offsets)
        except Exception:
            ser.close()
            worker_self.ser = None
            raise
        return (quant, max_count, version_warning)

    def _task_stop(worker_self: SerialWorker, port: str):
        if worker_self.ser is not None:
            try:
                stop_ppm_on_serial(worker_self.ser)
            finally:
                try:
                    worker_self.ser.close()
                finally:
                    worker_self.ser = None
            return None
        else:
            with open_serial(port) as ser:
                stop_ppm_on_serial(ser)
            return None

    def _task_hold(worker_self: SerialWorker, i: int, target: int, offset: int, timeout_s: float):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        quant, max_count = read_regs(worker_self.ser, REG_QUANT, 2)
        set_channel_until_stop_on_serial(worker_self.ser, quant, max_count, i, target, offset, timeout_s)
        return read_pulse_status_on_serial(worker_self.ser, max_count)

    def _task_fc_read_pid_ff(_worker_self: SerialWorker):
        return fc_service.read_roll_pitch_pid_ff(timeout_seconds=1.2)

    def _task_enter_msc_and_import_blackbox_logs(_worker_self: SerialWorker, fc_port_name: str, fc_baud_rate: int):
        msc_warnings: list[str] = []
        try:
            send_cli_msc_command(fc_port_name, fc_baud_rate)
        except Exception as exc:
            msc_warnings.append(f"Could not send CLI 'msc' on {fc_port_name}: {exc}")

        deadline = time.monotonic() + blackbox_msc_mount_timeout_s
        result: BlackboxImportResult | None = None
        while True:
            result = import_blackbox_logs_from_msc(blackbox_import_dir)
            if result.scanned_roots:
                break
            if time.monotonic() >= deadline:
                break
            time.sleep(blackbox_msc_mount_poll_s)

        if result is None:
            result = import_blackbox_logs_from_msc(blackbox_import_dir)
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
        return analyze_pulled_blackbox_logs(blackbox_import_dir)

    def _task_analyze_specific_blackbox_log(_worker_self: SerialWorker, log_path: str):
        return analyze_blackbox_log(log_path, decode_destination_dir=blackbox_import_dir)

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
        if blackbox_import_dir.exists() and blackbox_import_dir not in search_dirs:
            search_dirs.append(blackbox_import_dir)

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
            blackbox_import_dir,
            analysis_result,
            session_payload,
            source_path,
        )

    def _task_generate_step_response_report(_worker_self: SerialWorker, log_paths: list[str]):
        return generate_step_response_report(log_paths, blackbox_import_dir)

    def _task_hold_humanized(
        worker_self: SerialWorker,
        i: int,
        target: int,
        offset: int,
        timeout_s: float,
        channels: list[int],
        offsets: list[int],
    ):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        quant, max_count = read_regs(worker_self.ser, REG_QUANT, 2)
        set_channel_with_human_profile_until_stop_on_serial(
            worker_self.ser,
            quant,
            max_count,
            channels,
            offsets,
            i,
            target,
            offset,
            timeout_s,
        )
        return read_pulse_status_on_serial(worker_self.ser, max_count)

    def _task_hold_end(worker_self: SerialWorker, i: int):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        _, max_count = read_regs(worker_self.ser, REG_QUANT, 2)
        end_hold_on_serial(worker_self.ser, max_count, i)
        return read_pulse_status_on_serial(worker_self.ser, max_count)

    def _task_run_ppm_on_existing(worker_self: SerialWorker, channels: list[int], offsets: list[int]):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        return run_ppm_on_serial(worker_self.ser, channels, offsets)

    def _task_update_channels(worker_self: SerialWorker, channels: list[int], offsets: list[int]):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        quant, max_count, _ = run_ppm_on_serial(worker_self.ser, channels, offsets)
        return (quant, max_count, channels)

    def _task_read_pulse_status(worker_self: SerialWorker, max_count: int):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        return read_pulse_status_on_serial(worker_self.ser, max_count)

    def _task_shutdown(worker_self: SerialWorker):
        if worker_self.ser is None:
            return None
        try:
            stop_ppm_on_serial(worker_self.ser)
        except Exception:
            pass
        finally:
            try:
                worker_self.ser.close()
            finally:
                worker_self.ser = None
        return None

    def poll_results() -> None:
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
        root.after(50, poll_results)

    def do_start() -> None:
        nonlocal run_active, run_port, run_ser, run_quant, run_max_count, start_pending, base_channel_outputs
        nonlocal channel_update_inflight, pending_channel_update_channels, pending_channel_update_offsets
        try:
            if start_pending:
                raise RuntimeError("Start is already in progress.")
            if hold_command_inflight:
                raise RuntimeError("Wait for active Pulse command to finish.")
            if hold_timeout_after_id is not None:
                raise RuntimeError("Wait for active Hold to finish or press ∅/Stop.")
            channels = parse_entries(ch_entries, int, "Channel")
            require_range(channels, "Channel", 1000, 2000)
            offsets = parse_entries(off_entries, int, "Offset")
            selected_port = port()

            def on_start_done(ok: bool, res: object) -> None:
                nonlocal run_active, run_port, run_ser, run_quant, run_max_count, start_pending, base_channel_outputs
                nonlocal channel_update_inflight, pending_channel_update_channels, pending_channel_update_offsets
                start_pending = False
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
                # success
                run_port = selected_port
                run_quant = res[0]
                run_max_count = res[1]
                run_ser = worker.ser
                run_active = True
                channel_update_inflight = False
                pending_channel_update_channels = None
                pending_channel_update_offsets = None
                base_channel_outputs = channels.copy()
                set_live_channel_outputs(base_channel_outputs)
                update_link_indicators()
                version_warning = res[2]
                if version_warning:
                    status.set(version_warning)
                    messagebox.showwarning("Firmware version", version_warning)
                else:
                    status.set("PPM output configured and started.")

            if run_ser is None:
                start_pending = True
                update_link_indicators()
                worker.submit(_task_open_and_start, selected_port, channels, offsets, callback=on_start_done)
            else:
                if selected_port != run_port:
                    raise RuntimeError(f"Output is active on {run_port}. Press Disconnect Arduino before switching ports.")
                start_pending = True
                update_link_indicators()
                worker.submit(_task_run_ppm_on_existing, channels, offsets, callback=on_start_done)

        except Exception as exc:
            start_pending = False
            update_link_indicators()
            set_error("Start error", exc)

    def do_stop() -> None:
        try:
            if hold_command_inflight:
                raise RuntimeError("Pulse command is in progress. Wait a moment, then try again.")
            cancel_hold_timeout()
            def on_stop_done(ok: bool, res: object) -> None:
                nonlocal run_ser, run_quant, run_max_count, run_active, hold_command_inflight
                nonlocal channel_update_inflight, pending_channel_update_channels, pending_channel_update_offsets
                if not ok:
                    set_error("Stop error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if res is not None:
                    set_error("Stop error", RuntimeError("Unexpected worker result from stop task"))
                    return
                run_ser = None
                run_quant = None
                run_max_count = None
                run_active = False
                hold_command_inflight = False
                channel_update_inflight = False
                pending_channel_update_channels = None
                pending_channel_update_offsets = None
                set_live_channel_outputs(parse_channel_values_with_defaults())
                update_link_indicators()
                status.set("PPM output stopped.")

            worker.submit(_task_stop, port(), callback=on_stop_done)
        except Exception as exc:
            set_error("Stop error", exc)

    def do_hold_send(i: int, direction: int) -> None:
        nonlocal run_max_count, hold_timeout_after_id, hold_command_inflight
        try:
            if not run_active or run_ser is None:
                raise RuntimeError("Press Connect Arduino before using Hold.")
            if level_active:
                stop_level_loop(update_status=False)
            if hold_command_inflight:
                raise RuntimeError("A pulse command is already in progress.")
            if hold_timeout_after_id is not None:
                raise RuntimeError("A hold command is already active. Wait for timeout or press ∅.")

            channels = parse_entries(ch_entries, int, "Channel")
            require_range(channels, "Channel", 1000, 2000)
            offsets = parse_entries(off_entries, int, "Offset")
            targets = parse_entries(target_entries, int, "Target")
            require_range(targets, "Target", 0, 500)
            durations = parse_entries(dur_entries, float, "Duration")
            require_duration_range(durations, 0.05, 60.0)
            timeout_s = durations[i]
            signed_direction = 1 if direction >= 0 else -1
            target_delta_us = signed_direction * targets[i]
            pulse_target_us = channels[i] + target_delta_us
            if pulse_target_us < 1000 or pulse_target_us > 2000:
                raise RuntimeError(
                    f"Computed pulse value CH{i + 1} is {pulse_target_us}. "
                    "Adjust Channel/Target so output stays between 1000 and 2000."
                )
            angle_threshold = 0.0
            angle_state = str(angle_entries[i].cget("state"))
            if angle_state == "normal":
                raw_threshold = angle_entries[i].get().strip()
                if raw_threshold:
                    try:
                        angle_magnitude = float(raw_threshold)
                    except ValueError as exc:
                        raise RuntimeError(f"Angle CH{i + 1} must be a number.") from exc
                    if angle_magnitude < 0 or angle_magnitude > 45:
                        raise RuntimeError(f"Angle CH{i + 1} must be between 0 and 45.")
                    if angle_magnitude > 0:
                        angle_threshold = float(signed_direction) * angle_magnitude

            def restore_after_hold_failure() -> None:
                if not run_active or run_ser is None:
                    return
                restore_base_outputs_after_hold(offsets)

            def on_hold_done(ok: bool, res: object) -> None:
                nonlocal hold_timeout_after_id, run_max_count, hold_command_inflight
                hold_command_inflight = False
                if not ok:
                    restore_after_hold_failure()
                    set_error("Hold error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, int):
                    restore_after_hold_failure()
                    set_error("Hold error", RuntimeError("Unexpected worker result from hold task"))
                    return
                pulse_status = res
                if pulse_status == PULSE_STATUS_REJECTED:
                    restore_after_hold_failure()
                    set_error("Hold error", RuntimeError("Firmware rejected hold command"))
                    return
                active_outputs = base_channel_outputs.copy()
                active_outputs[i] = pulse_target_us
                set_live_channel_outputs(active_outputs)

                timeout_ms = max(1, round(timeout_s * 1000))
                chan_label = i + 1
                deadline_s = time.monotonic() + timeout_s
                direction_label = "positive" if signed_direction > 0 else "negative"

                def schedule_timeout_status_check() -> None:
                    def cb(ok2: bool, res2: object) -> None:
                        nonlocal hold_timeout_after_id
                        if not ok2:
                            set_error("Hold timeout error", res2 if isinstance(res2, Exception) else RuntimeError(res2))
                            hold_timeout_after_id = None
                            return
                        if not isinstance(res2, int):
                            set_error("Hold timeout error", RuntimeError("Unexpected worker result from pulse-status task"))
                            hold_timeout_after_id = None
                            return
                        pulse_status_now = res2
                        if pulse_status_now not in (PULSE_STATUS_TIMEOUT_RESTORED, PULSE_STATUS_HOLD_ENDED):
                            hold_timeout_after_id = root.after(HOLD_TIMEOUT_POLL_MS, schedule_timeout_status_check)
                            return
                        hold_timeout_after_id = None
                        restore_base_outputs_after_hold(offsets)
                        if pulse_status_now == PULSE_STATUS_TIMEOUT_RESTORED:
                            status.set(f"CH{chan_label} hold timed out; channel restored.")
                        else:
                            status.set(f"CH{chan_label} hold ended; channel restored.")

                    worker.submit(_task_read_pulse_status, run_max_count or 0, callback=cb)

                def on_angle_hold_end_done(ok3: bool, res3: object) -> None:
                    nonlocal hold_timeout_after_id
                    if not ok3:
                        set_error("Hold end error", res3 if isinstance(res3, Exception) else RuntimeError(res3))
                        hold_timeout_after_id = root.after(HOLD_ANGLE_CHECK_MS, schedule_angle_or_timeout_check)
                        return
                    if not isinstance(res3, int):
                        set_error("Hold end error", RuntimeError("Unexpected worker result from hold-end task"))
                        hold_timeout_after_id = root.after(HOLD_ANGLE_CHECK_MS, schedule_angle_or_timeout_check)
                        return
                    if res3 == PULSE_STATUS_REJECTED:
                        set_error("Hold end error", RuntimeError("Firmware rejected hold-end command"))
                        hold_timeout_after_id = root.after(HOLD_ANGLE_CHECK_MS, schedule_angle_or_timeout_check)
                        return
                    cancel_hold_timeout()
                    restore_base_outputs_after_hold(offsets)
                    status.set(f"CH{chan_label} hold ended on angle threshold; channel restored.")

                def schedule_angle_or_timeout_check() -> None:
                    nonlocal hold_timeout_after_id
                    if hold_timeout_after_id is None:
                        return

                    angle_entry_enabled = str(angle_entries[i].cget("state")) == "normal"
                    if angle_entry_enabled and angle_threshold != 0 and is_angle_threshold_reached(i, angle_threshold):
                        worker.submit(_task_hold_end, i, callback=on_angle_hold_end_done)
                        return

                    if time.monotonic() >= deadline_s:
                        schedule_timeout_status_check()
                        return

                    remaining_ms = max(1, round((deadline_s - time.monotonic()) * 1000))
                    hold_timeout_after_id = root.after(min(HOLD_ANGLE_CHECK_MS, remaining_ms), schedule_angle_or_timeout_check)

                hold_timeout_after_id = root.after(min(HOLD_ANGLE_CHECK_MS, timeout_ms), schedule_angle_or_timeout_check)
                status.set(
                    f"CH{chan_label} hold active ({direction_label}, {target_delta_us:+d}us). "
                    f"Press ∅ for early restore (auto in {timeout_s:.3g}s)."
                )

            hold_command_inflight = True
            try:
                worker.submit(
                    _task_hold_humanized,
                    i,
                    pulse_target_us,
                    offsets[i],
                    timeout_s,
                    channels.copy(),
                    offsets.copy(),
                    callback=on_hold_done,
                )
            except Exception:
                hold_command_inflight = False
                raise
        except Exception as exc:
            set_error("Hold error", exc)

    def do_hold_end(i: int) -> None:
        try:
            if not run_active or run_ser is None:
                raise RuntimeError("Press Connect Arduino before ending Hold.")
            if hold_command_inflight:
                raise RuntimeError("Pulse command is still ramping in. Wait a moment, then try ∅.")
            if hold_timeout_after_id is None:
                raise RuntimeError("No active Hold to end.")

            def on_hold_end_done(ok: bool, res: object) -> None:
                if not ok:
                    set_error("Hold end error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, int):
                    set_error("Hold end error", RuntimeError("Unexpected worker result from hold-end task"))
                    return
                if res == PULSE_STATUS_REJECTED:
                    set_error("Hold end error", RuntimeError("Firmware rejected hold-end command"))
                    return
                cancel_hold_timeout()
                restore_base_outputs_after_hold()
                status.set(f"CH{i + 1} hold ended; channel restored.")

            worker.submit(_task_hold_end, i, callback=on_hold_end_done)
        except Exception as exc:
            set_error("Hold end error", exc)

    def on_close() -> None:
        nonlocal is_closing, fc_poll_after_id
        is_closing = True
        cancel_adjust_repeat()
        cancel_hold_timeout()
        stop_auto_session_runtime()
        stop_simulated_auto_session("", restore_display=False)
        if fc_poll_after_id is not None:
            try:
                root.after_cancel(fc_poll_after_id)
            except Exception:
                pass
            finally:
                fc_poll_after_id = None

        def on_stop_and_close(ok: bool, res: object) -> None:
            do_fc_disconnect(update_status=False)
            try:
                worker.stop()
            except Exception:
                pass
            close_run_connection()
            root.destroy()

        try:
            worker.submit(_task_shutdown, callback=on_stop_and_close)
        except Exception:
            on_stop_and_close(False, None)

    scan_fc_ports(update_status=False)

    scan_fc_button.config(command=scan_fc_ports)
    connect_fc_button.config(command=do_fc_toggle)
    import_blackbox_button.config(command=do_pull_blackbox_logs)
    analyze_blackbox_button.config(command=do_analyze_blackbox_logs)
    auto_session_button.config(command=do_auto_session_toggle)
    fly_log_button.config(command=do_pid_plan_fly_log_toggle)
    simulate_auto_session_button.config(command=do_simulated_auto_session_toggle)
    step_response_button.config(command=do_step_response_report)
    pid_tuning_plan_button.config(command=do_pid_tuning_plan)
    auto_open_selected_button.config(command=open_selected_report_file)
    auto_open_all_button.config(command=open_all_report_files)
    auto_clear_reports_button.config(command=do_clear_report_files)
    arduino_button.config(command=do_arduino_toggle)
    for i, canvas in enumerate(hold_send_canvases):
        def on_hold_press(event: tk.Event, i: int = i) -> None:
            action = get_pulse_action(event)
            if action == "end":
                do_hold_end(i)
                return
            direction = 1 if action == "positive" else -1
            do_hold_send(i, direction)

        canvas.bind("<ButtonPress-1>", on_hold_press)
    level_button.config(command=do_level)
    for i, canvas in enumerate(channel_adjust_canvases):
        canvas.bind("<ButtonPress-1>", lambda event, i=i: on_adjust_press(adjust_channel_value, i, event))
        canvas.bind("<ButtonRelease-1>", on_adjust_release)
        canvas.bind("<Leave>", on_adjust_release)
    for i, canvas in enumerate(target_adjust_canvases):
        canvas.bind("<ButtonPress-1>", lambda event, i=i: on_adjust_press(adjust_target_value, i, event))
        canvas.bind("<ButtonRelease-1>", on_adjust_release)
        canvas.bind("<Leave>", on_adjust_release)
    for i, canvas in enumerate(pid_ff_adjust_canvases):
        canvas.bind("<ButtonPress-1>", lambda event, i=i: on_adjust_press(adjust_pid_ff_value, i, event, 1))
        canvas.bind("<ButtonRelease-1>", on_adjust_release)
        canvas.bind("<Leave>", on_adjust_release)
    for entry in ch_entries:
        entry.bind("<KeyRelease>", lambda _event: on_output_inputs_changed())
        entry.bind("<FocusOut>", lambda _event: on_output_inputs_changed())
    set_auto_state(AdaptiveSessionState.idle)
    set_live_channel_outputs(parse_channel_values_with_defaults())
    update_link_indicators()
    root.after(50, poll_results)
    fc_poll_after_id = root.after(60, poll_fc_attitude)
    root.protocol("WM_DELETE_WINDOW", on_close)

    root.mainloop()


if __name__ == "__main__":
    main()

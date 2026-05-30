"""Application runtime orchestration."""

from __future__ import annotations

import io
import queue
import time
import tkinter as tk
from collections.abc import Callable, Sequence
from tkinter import messagebox

import serial
from serial.tools import list_ports

from serialUSB.inav_serial_service import (
    AxisPidFf,
    FF_SETTING_NAME,
    InavSerialService,
    PID_SETTING_NAME,
)

from .constants import (
    ADJUST_REPEAT_INITIAL_MS,
    ADJUST_REPEAT_INTERVAL_MS,
    CHANNEL_DEFAULTS,
    FC_BAUD_DEFAULT,
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
    PITCH_CHANNEL_INDEX,
    PORT_DEFAULT,
    PULSE_CHART_MAX_POINTS,
    PULSE_CHART_SETTLE_S,
    PULSE_DURATION_DEFAULTS,
    PULSE_STATUS_HOLD_ENDED,
    PULSE_STATUS_REJECTED,
    PULSE_STATUS_TIMEOUT_RESTORED,
    PULSE_TARGET_DEFAULTS,
    REG_QUANT,
    ROLL_CHANNEL_INDEX,
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
from .ui import (
    PulseChartCapture,
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
    hold_end_buttons = ui.hold_end_buttons
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
    arduino_button = ui.arduino_button
    chart_status = ui.chart_status
    chart_strip_canvas = ui.chart_strip_canvas
    chart_strip_frame = ui.chart_strip_frame
    attitude_chart = ui.attitude_chart

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
    pulse_chart_finalize_after_id: str | None = None
    pulse_chart_active: PulseChartCapture | None = None
    pulse_chart_photos: list[tk.PhotoImage] = []
    pulse_chart_count = 0
    level_active = False
    level_after_id: str | None = None
    level_pulse_inflight = False
    level_timeout_deadline_s: float | None = None
    level_timeout_s = max(PULSE_DURATION_DEFAULTS[ROLL_CHANNEL_INDEX], PULSE_DURATION_DEFAULTS[PITCH_CHANNEL_INDEX])
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

    def pulse_axis_for_channel(channel_index: int) -> str | None:
        if channel_index == 0:
            return "roll"
        if channel_index == 1:
            return "pitch"
        return None

    def pulse_axis_value(sample, axis: str) -> float:
        if axis == "roll":
            return float(sample.roll_deg)
        return float(sample.pitch_deg)

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

    def set_pid_ff_displays(roll_values: AxisPidFf, pitch_values: AxisPidFf) -> None:
        roll_series = (roll_values.p, roll_values.i, roll_values.d, roll_values.ff)
        pitch_series = (pitch_values.p, pitch_values.i, pitch_values.d, pitch_values.ff)
        for label, value, var in zip(pid_ff_labels, roll_series, roll_pidff_vars):
            var.set(f"{label}: {format_pid_ff_value(value)}")
        for label, value, var in zip(pid_ff_labels, pitch_series, pitch_pidff_vars):
            var.set(f"{label}: {format_pid_ff_value(value)}")

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

    def blend_hex(start_hex: str, end_hex: str, amount: float) -> str:
        clamped = max(0.0, min(1.0, float(amount)))
        start = start_hex.lstrip("#")
        end = end_hex.lstrip("#")
        s_r, s_g, s_b = int(start[0:2], 16), int(start[2:4], 16), int(start[4:6], 16)
        e_r, e_g, e_b = int(end[0:2], 16), int(end[2:4], 16), int(end[4:6], 16)
        r = round(s_r + (e_r - s_r) * clamped)
        g = round(s_g + (e_g - s_g) * clamped)
        b = round(s_b + (e_b - s_b) * clamped)
        return f"#{r:02X}{g:02X}{b:02X}"

    def pulse_direction_label(axis: str, delta_us: int) -> str:
        if delta_us > 0:
            return "Right" if axis == "roll" else "Forward"
        if delta_us < 0:
            return "Left" if axis == "roll" else "Back"
        return "Neutral"

    def pulse_visuals(axis: str, delta_us: int) -> tuple[str, str, str, str]:
        magnitude_ratio = min(1.0, abs(delta_us) / 450.0)
        if delta_us > 0:
            card_bg = blend_hex("#EFF6FF", "#DBEAFE", magnitude_ratio)
            accent_color = blend_hex("#60A5FA", "#1D4ED8", magnitude_ratio)
            badge_color = blend_hex("#2563EB", "#1E3A8A", magnitude_ratio)
        elif delta_us < 0:
            card_bg = blend_hex("#FFF7ED", "#FFEDD5", magnitude_ratio)
            accent_color = blend_hex("#FDBA74", "#C2410C", magnitude_ratio)
            badge_color = blend_hex("#EA580C", "#7C2D12", magnitude_ratio)
        else:
            card_bg = "#F8FAFC"
            accent_color = "#94A3B8"
            badge_color = "#64748B"
        if axis == "roll":
            line_color = blend_hex("#38BDF8", "#0369A1", magnitude_ratio)
        else:
            line_color = blend_hex("#34D399", "#047857", magnitude_ratio)
        return card_bg, accent_color, badge_color, line_color

    def cancel_pulse_chart_finalize_timer() -> None:
        nonlocal pulse_chart_finalize_after_id
        if pulse_chart_finalize_after_id is None:
            return
        try:
            root.after_cancel(pulse_chart_finalize_after_id)
        except Exception:
            pass
        pulse_chart_finalize_after_id = None

    def pulse_chart_baseline(capture: PulseChartCapture) -> float:
        if not capture.samples:
            return 0.0
        if capture.restore_monotonic is not None:
            restore_elapsed = max(0.0, capture.restore_monotonic - capture.start_monotonic)
            settled_values = [value for t, value in capture.samples if t >= restore_elapsed]
            if settled_values:
                return sum(settled_values) / len(settled_values)
        tail = capture.samples[-min(10, len(capture.samples)) :]
        return sum(value for _, value in tail) / len(tail)

    def append_pulse_chart_card(capture: PulseChartCapture, baseline_deg: float) -> None:
        nonlocal pulse_chart_count, pulse_chart_photos
        if len(capture.samples) < 2:
            chart_status.set("Pulse chart skipped: not enough FC samples were collected.")
            return
        try:
            import matplotlib

            matplotlib.use("Agg", force=True)
            from matplotlib.backends.backend_agg import FigureCanvasAgg as LocalFigureCanvasAgg
            from matplotlib.figure import Figure as LocalFigure
        except Exception as exc:
            chart_status.set(
                "Matplotlib is unavailable in this interpreter, so pulse charts cannot be rendered. "
                f"Install it in .venv to enable charts. Error: {exc}"
            )
            return

        axis_title = capture.axis.title()
        direction_label = pulse_direction_label(capture.axis, capture.command_delta_us)
        card_bg, accent_color, badge_color, line_color = pulse_visuals(capture.axis, capture.command_delta_us)
        time_data = [point[0] for point in capture.samples]
        movement_data = [point[1] - baseline_deg for point in capture.samples]
        restore_elapsed = None
        if capture.restore_monotonic is not None:
            restore_elapsed = max(0.0, capture.restore_monotonic - capture.start_monotonic)
        magnitude_pct = min(100, round((abs(capture.command_delta_us) / 500.0) * 100))

        fig = LocalFigure(figsize=(4.9, 2.75), dpi=100, facecolor=card_bg)
        ax = fig.add_subplot(111, facecolor="#ECF3F9")
        ax.plot(time_data, movement_data, linewidth=1.9, color=line_color)
        ax.axhline(0.0, linewidth=1.0, color="#334155", linestyle="--", alpha=0.75)
        if restore_elapsed is not None:
            ax.axvline(restore_elapsed, linewidth=1.0, color=accent_color, linestyle=":", alpha=0.9)
        ax.set_title(
            f"{axis_title} CH{capture.channel_index + 1} target {capture.target_us} us ({direction_label} {capture.command_delta_us:+d} us)",
            fontsize=9,
        )
        ax.set_xlabel("Time from pulse command (s)", fontsize=8)
        ax.set_ylabel(f"{axis_title} vs settle baseline (deg)", fontsize=8)
        if capture.command_delta_us < 0:
            # Mirror vertical readout for negative/reverse pulse captures.
            ax.invert_yaxis()
        ax.tick_params(axis="both", labelsize=8)
        ax.grid(True, alpha=0.3, color="#94A3B8")
        fig.subplots_adjust(left=0.13, right=0.98, top=0.86, bottom=0.22)

        canvas = LocalFigureCanvasAgg(fig)
        buffer = io.BytesIO()
        canvas.print_png(buffer)
        photo = tk.PhotoImage(data=buffer.getvalue())
        pulse_chart_photos.append(photo)

        pulse_chart_count += 1
        card = tk.Frame(chart_strip_frame, bg=card_bg, bd=1, relief="solid", padx=4, pady=4)
        tk.Frame(card, bg=accent_color, height=4).pack(fill="x", pady=(0, 4))
        header = tk.Frame(card, bg=card_bg)
        header.pack(fill="x", pady=(0, 2))
        tk.Label(header, text=f"{axis_title} pulse #{pulse_chart_count}", bg=card_bg, fg="#0F172A", font=("Segoe UI", 9, "bold")).pack(
            side="left", anchor="w"
        )
        tk.Label(
            header,
            text=f"{direction_label} {capture.command_delta_us:+d}us",
            bg=badge_color,
            fg="white",
            font=("Segoe UI", 8, "bold"),
            padx=6,
            pady=1,
        ).pack(side="right", anchor="e")
        tk.Label(card, image=photo, bg=card_bg).pack(anchor="w", pady=(2, 2))
        reason = capture.restore_reason or "pulse completed"
        tk.Label(
            card,
            text=f"Baseline {baseline_deg:+.2f} deg ({reason}) | Magnitude {magnitude_pct}%",
            bg=card_bg,
            anchor="w",
            justify="left",
            font=("Segoe UI", 8),
        ).pack(anchor="w")
        card.pack(side="left", padx=(0, 8), pady=(0, 4), anchor="n")

        chart_strip_canvas.configure(scrollregion=chart_strip_canvas.bbox("all"))
        chart_strip_canvas.xview_moveto(1.0)
        chart_status.set(
            f"Added {axis_title} pulse chart #{pulse_chart_count}. "
            "Charts stay left-to-right in the horizontal strip."
        )

    def finalize_pulse_chart(force: bool = False) -> None:
        nonlocal pulse_chart_active
        cancel_pulse_chart_finalize_timer()
        capture = pulse_chart_active
        if capture is None:
            return
        if capture.restore_monotonic is None and not force:
            return
        baseline_deg = pulse_chart_baseline(capture)
        append_pulse_chart_card(capture, baseline_deg)
        pulse_chart_active = None

    def schedule_pulse_chart_finalize() -> None:
        nonlocal pulse_chart_finalize_after_id
        cancel_pulse_chart_finalize_timer()
        delay_ms = max(200, round(PULSE_CHART_SETTLE_S * 1000))

        def on_finalize() -> None:
            nonlocal pulse_chart_finalize_after_id
            pulse_chart_finalize_after_id = None
            finalize_pulse_chart(force=False)

        pulse_chart_finalize_after_id = root.after(delay_ms, on_finalize)

    def mark_pulse_chart_restore(reason: str) -> None:
        capture = pulse_chart_active
        if capture is None:
            return
        if capture.restore_monotonic is not None:
            return
        capture.restore_monotonic = time.monotonic()
        capture.restore_reason = reason
        chart_status.set(
            f"{capture.axis.title()} pulse restore detected; collecting {PULSE_CHART_SETTLE_S:.1f}s settle baseline."
        )
        schedule_pulse_chart_finalize()

    def start_pulse_chart_capture(channel_index: int, target_us: int, timeout_s: float, reference_us: int) -> None:
        nonlocal pulse_chart_active
        axis = pulse_axis_for_channel(channel_index)
        if axis is None:
            return
        if not fc_service.is_connected:
            chart_status.set(
                "Pulse sent on Roll/Pitch, but FC is disconnected so no movement chart could be recorded."
            )
            return
        if pulse_chart_active is not None:
            finalize_pulse_chart(force=True)
        capture = PulseChartCapture(
            axis=axis,
            channel_index=channel_index,
            target_us=target_us,
            reference_us=reference_us,
            command_delta_us=int(target_us) - int(reference_us),
            timeout_s=timeout_s,
            start_monotonic=time.monotonic(),
        )
        latest_sample = fc_service.latest_attitude()
        if latest_sample is not None:
            capture.samples.append((0.0, pulse_axis_value(latest_sample, axis)))
        pulse_chart_active = capture
        chart_status.set(
            f"Recording {axis.title()} pulse trace on CH{channel_index + 1}; chart will append after neutral settle."
        )

    def record_pulse_chart_sample(sample) -> None:
        capture = pulse_chart_active
        if capture is None:
            return
        elapsed = max(0.0, time.monotonic() - capture.start_monotonic)
        capture.samples.append((elapsed, pulse_axis_value(sample, capture.axis)))
        if len(capture.samples) > PULSE_CHART_MAX_POINTS:
            capture.samples = capture.samples[-PULSE_CHART_MAX_POINTS:]

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
        level_button.config(state="normal" if level_ready else "disabled", relief="sunken" if level_active else "raised")
        attitude_chart.set_connection_state(fc_connected)

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
                raise RuntimeError("Wait for active Hold to finish or press End/Stop.")
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
                if pulse_chart_active is not None:
                    mark_pulse_chart_restore("PPM link closed")
                    finalize_pulse_chart(force=True)
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
            attitude_chart.clear("FC connected. Waiting for first attitude sample...")
            update_link_indicators()
            status.set(f"FC connected: {selected_port} @ {selected_baud}. Loading PID/FF...")
            queue_fc_pid_ff_refresh(selected_port, selected_baud)
        except Exception as exc:
            set_error("FC connect error", exc)

    def do_fc_disconnect(update_status: bool = True) -> None:
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
            attitude_chart.set_connection_state(False)
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
                record_pulse_chart_sample(sample)
                horizon.set_attitude(sample.roll_deg, sample.pitch_deg)
                roll_text.set(f"Roll: {sample.roll_deg:6.1f} deg")
                pitch_text.set(f"Pitch: {sample.pitch_deg:6.1f} deg")
                attitude_chart.add_sample(sample)
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
                raise RuntimeError("Wait for active Hold to finish or press End/Stop.")
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
                raise RuntimeError("A hold command is already active. Wait for timeout or press End.")

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
                set_live_channel_outputs(base_channel_outputs)
                queue_live_channel_update(base_channel_outputs.copy(), offsets.copy())

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
                start_pulse_chart_capture(i, pulse_target_us, timeout_s, base_channel_outputs[i])

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
                        set_live_channel_outputs(base_channel_outputs)
                        if pulse_status_now == PULSE_STATUS_TIMEOUT_RESTORED:
                            mark_pulse_chart_restore(f"CH{chan_label} timeout restore")
                            status.set(f"CH{chan_label} hold timed out; channel restored.")
                        else:
                            mark_pulse_chart_restore(f"CH{chan_label} hold end restore")
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
                    set_live_channel_outputs(base_channel_outputs)
                    mark_pulse_chart_restore(f"CH{chan_label} angle threshold restore")
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
                    f"Press End for early restore (auto in {timeout_s:.3g}s)."
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
                raise RuntimeError("Pulse command is still ramping in. Wait a moment, then try End.")
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
                set_live_channel_outputs(base_channel_outputs)
                mark_pulse_chart_restore(f"CH{i + 1} manual end restore")
                status.set(f"CH{i + 1} hold ended; channel restored.")

            worker.submit(_task_hold_end, i, callback=on_hold_end_done)
        except Exception as exc:
            set_error("Hold end error", exc)

    def on_close() -> None:
        nonlocal is_closing, fc_poll_after_id
        is_closing = True
        cancel_adjust_repeat()
        cancel_hold_timeout()
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
    arduino_button.config(command=do_arduino_toggle)
    for i, canvas in enumerate(hold_send_canvases):
        def on_hold_press(event: tk.Event, i: int = i) -> None:
            delta = get_adjust_delta(event, step=1)
            direction = 1 if delta > 0 else -1
            do_hold_send(i, direction)

        canvas.bind("<ButtonPress-1>", on_hold_press)
    for i, button in enumerate(hold_end_buttons):
        button.config(command=lambda i=i: do_hold_end(i))
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
    set_live_channel_outputs(parse_channel_values_with_defaults())
    update_link_indicators()
    root.after(50, poll_results)
    fc_poll_after_id = root.after(60, poll_fc_attitude)
    root.protocol("WM_DELETE_WINDOW", on_close)

    root.mainloop()


if __name__ == "__main__":
    main()

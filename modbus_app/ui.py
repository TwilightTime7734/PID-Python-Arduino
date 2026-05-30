"""Tkinter UI components and layout builders."""

from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk
from bisect import bisect_left, bisect_right
from collections.abc import Sequence
from dataclasses import dataclass, field

from serialUSB.inav_serial_service import AttitudeSample

from .constants import (
    ATTITUDE_CHART_DRAW_INTERVAL_S,
    ATTITUDE_CHART_SCROLL_UNIT_S,
    ATTITUDE_CHART_VIEW_WINDOW_S,
    BAUDRATE,
    CHANNEL_DEFAULTS,
    FC_BAUD_DEFAULT,
    FC_PORT_DEFAULT,
    LEVEL_DEADBAND_DEG,
    OFFSET_DEFAULTS,
    PORT_DEFAULT,
    PULSE_CHART_MAX_POINTS,
    PULSE_DURATION_DEFAULTS,
    PULSE_TARGET_DEFAULTS,
)


def parse_entries(entries: list[tk.Entry], cast, label: str) -> list:
    try:
        return [cast(e.get().strip()) for e in entries]
    except ValueError:
        kind = "integers" if cast is int else "numbers"
        raise RuntimeError(f"{label} values must be {kind}")


def require_range(values: Sequence[int], label: str, min_value: int, max_value: int) -> None:
    for idx, value in enumerate(values, 1):
        if value < min_value or value > max_value:
            raise RuntimeError(f"{label} CH{idx} must be between {min_value} and {max_value}")


def require_duration_range(values: Sequence[float], min_s: float, max_s: float) -> None:
    for idx, value in enumerate(values, 1):
        if value < min_s or value > max_s:
            raise RuntimeError(f"Duration CH{idx} must be between {min_s:.3g}s and {max_s:.3g}s")


def make_row(root: tk.Misc, row: int, label: str, defaults: Sequence[int | float]) -> list[tk.Entry]:
    tk.Label(root, text=label).grid(row=row, column=0, padx=6, pady=(0, 2), sticky="e")
    out: list[tk.Entry] = []
    for i, value in enumerate(defaults, 1):
        entry = tk.Entry(root, width=8)
        entry.insert(0, str(value))
        entry.grid(row=row, column=i, padx=4, pady=(0, 2))
        out.append(entry)
    return out


class ArtificialHorizon(tk.Canvas):
    def __init__(self, parent: tk.Misc, size: int = 180) -> None:
        super().__init__(parent, width=size, height=size, bg="#101417", highlightthickness=0)
        self._size = size
        self._cx = size / 2
        self._cy = size / 2
        self._extent = size * 2.0
        self._pitch_px_per_deg = size / 90.0

        self._sky = self.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#4B95D9", outline="")
        self._ground = self.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#8C4F2A", outline="")
        self._horizon_line = self.create_line(0, 0, 0, 0, fill="white", width=2)

        self.create_oval(2, 2, size - 2, size - 2, outline="#C9D1D9", width=2)
        self.create_line(self._cx - 22, self._cy, self._cx + 22, self._cy, fill="#F6D32D", width=3)
        self.create_line(self._cx, self._cy - 8, self._cx, self._cy + 8, fill="#F6D32D", width=2)
        for deg in (-30, -20, -10, 10, 20, 30):
            y = self._cy - deg * self._pitch_px_per_deg
            self.create_line(self._cx - 10, y, self._cx + 10, y, fill="#FFFFFF", width=1)

        self.set_attitude(roll_deg=0.0, pitch_deg=0.0)

    def set_attitude(self, roll_deg: float, pitch_deg: float) -> None:
        roll_rad = math.radians(roll_deg)
        dx = math.cos(roll_rad)
        dy = math.sin(roll_rad)
        nx = -dy
        ny = dx

        pitch_shift = max(-45.0, min(45.0, pitch_deg)) * self._pitch_px_per_deg
        px = self._cx + nx * pitch_shift
        py = self._cy + ny * pitch_shift

        half = self._extent
        x1 = px - dx * half
        y1 = py - dy * half
        x2 = px + dx * half
        y2 = py + dy * half

        sx1 = x1 - nx * half
        sy1 = y1 - ny * half
        sx2 = x2 - nx * half
        sy2 = y2 - ny * half

        gx1 = x1 + nx * half
        gy1 = y1 + ny * half
        gx2 = x2 + nx * half
        gy2 = y2 + ny * half

        self.coords(self._sky, sx1, sy1, sx2, sy2, x2, y2, x1, y1)
        self.coords(self._ground, x1, y1, x2, y2, gx2, gy2, gx1, gy1)
        self.coords(self._horizon_line, x1, y1, x2, y2)


class AttitudeChartPanel:
    def __init__(self, parent: tk.Misc, figure_size: tuple[float, float] = (8.5, 2.8)) -> None:
        self._view_window_seconds = ATTITUDE_CHART_VIEW_WINDOW_S
        self._times: list[float] = []
        self._rolls: list[float] = []
        self._pitches: list[float] = []
        self._start_time_s: float | None = None
        self._last_draw_s = 0.0
        self._view_start_s = 0.0
        self._auto_follow = True
        self._frozen = False

        self._status = tk.StringVar(value="FC chart idle. Connect FC to stream roll/pitch.")
        header = tk.Frame(parent)
        header.pack(fill=tk.X, pady=(0, 4))
        tk.Label(header, textvariable=self._status, anchor="w").pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._freeze_button = tk.Button(header, text="Freeze", width=8, command=self.toggle_freeze)
        self._freeze_button.pack(side=tk.RIGHT, padx=(6, 0))
        self._live_button = tk.Button(header, text="Live", width=8, command=self.jump_to_live)
        self._live_button.pack(side=tk.RIGHT, padx=(6, 0))
        self._clear_button = tk.Button(header, text="Clear", width=8, command=self.clear)
        self._clear_button.pack(side=tk.RIGHT, padx=(6, 0))

        self._canvas = None
        self._axis = None
        self._line_roll = None
        self._line_pitch = None
        self._x_scrollbar = None

        try:
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
        except Exception as exc:
            self._clear_button.config(state="disabled")
            self._live_button.config(state="disabled")
            self._freeze_button.config(state="disabled")
            self._status.set("Chart unavailable: matplotlib is not available in this Python environment.")
            tk.Label(
                parent,
                text=f"Matplotlib load error: {exc}",
                fg="#8B1E1E",
                anchor="w",
                justify="left",
            ).pack(fill=tk.X)
            return

        figure = Figure(figsize=figure_size, dpi=100, facecolor="#172033")
        axis = figure.add_subplot(111, facecolor="#111827")
        line_roll, = axis.plot([], [], linewidth=1.8, color="#38bdf8", label="Roll")
        line_pitch, = axis.plot([], [], linewidth=1.8, color="#34d399", label="Pitch")
        axis.axhline(0.0, linewidth=1, alpha=0.65, color="#d1d5db", label="Baseline")
        axis.axhline(LEVEL_DEADBAND_DEG, linestyle="--", linewidth=1.0, color="#f59e0b", label="+ Deadband")
        axis.axhline(-LEVEL_DEADBAND_DEG, linestyle="--", linewidth=1.0, color="#f59e0b", label="- Deadband")
        axis.set_title("Live FC Attitude (Roll/Pitch)", fontsize=10, color="#FDFDFD")
        axis.set_xlabel("Time (s)", fontsize=8, color="#FDFDFD")
        axis.set_ylabel("Angle (deg)", fontsize=8, color="#FDFDFD")
        axis.tick_params(axis="both", labelsize=8, colors="#FDFDFD")
        for spine in axis.spines.values():
            spine.set_color("#374151")
        axis.grid(True, alpha=0.35, color="#4b5563")
        legend = axis.legend(loc="upper left", fontsize=7, ncol=2)
        legend.get_frame().set_facecolor("#1f2937")
        legend.get_frame().set_edgecolor("#374151")
        for text in legend.get_texts():
            text.set_color("#FDFDFD")
        axis.set_xlim(0.0, self._view_window_seconds)
        axis.set_ylim(-20.0, 20.0)

        canvas = FigureCanvasTkAgg(figure, master=parent)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.configure(highlightthickness=0, bd=0)
        canvas_widget.pack(fill=tk.BOTH, expand=True)

        x_scrollbar = tk.Scrollbar(parent, orient="horizontal", command=self._on_scrollbar)
        x_scrollbar.pack(fill=tk.X, pady=(4, 0))
        x_scrollbar.set(0.0, 1.0)
        canvas.draw_idle()

        self._canvas = canvas
        self._axis = axis
        self._line_roll = line_roll
        self._line_pitch = line_pitch
        self._x_scrollbar = x_scrollbar

    def clear(self, status_text: str | None = None) -> None:
        self._times.clear()
        self._rolls.clear()
        self._pitches.clear()
        self._start_time_s = None
        self._last_draw_s = 0.0
        self._view_start_s = 0.0
        self._auto_follow = True
        self._frozen = False
        self._freeze_button.config(text="Freeze")
        if status_text is not None:
            self._status.set(status_text)
        if self._axis is None or self._canvas is None or self._line_roll is None or self._line_pitch is None:
            return
        self._line_roll.set_data([], [])
        self._line_pitch.set_data([], [])
        self._axis.set_xlim(0.0, self._view_window_seconds)
        self._axis.set_ylim(-20.0, 20.0)
        self._update_scrollbar_thumb()
        self._canvas.draw_idle()

    def set_connection_state(self, connected: bool) -> None:
        if self._axis is None:
            return
        if connected:
            if not self._times:
                self._status.set("FC connected. Waiting for first attitude sample...")
            return
        if self._times:
            self._status.set("FC disconnected. Chart paused. Press Clear to reset.")
        else:
            self._status.set("FC chart idle. Connect FC to stream roll/pitch.")

    def jump_to_live(self) -> None:
        if not self._times:
            return
        self._auto_follow = True
        self._view_start_s = self._max_view_start()
        self._redraw(force=True)

    def toggle_freeze(self) -> None:
        if self._axis is None or self._canvas is None:
            return
        self._frozen = not self._frozen
        if self._frozen:
            self._freeze_button.config(text="Resume")
            if self._times:
                self._status.set("Chart frozen. Press Resume to continue updates.")
            else:
                self._status.set("Chart frozen. Waiting for first sample.")
            return

        self._freeze_button.config(text="Freeze")
        self._redraw(force=True)

    def _max_view_start(self) -> float:
        if not self._times:
            return 0.0
        return max(0.0, self._times[-1] - self._view_window_seconds)

    def _on_scrollbar(self, *args: str) -> None:
        if not self._times:
            return

        latest_time = self._times[-1]
        max_start = self._max_view_start()
        if max_start <= 0.0:
            self._auto_follow = True
            self._view_start_s = 0.0
            self._redraw(force=True)
            return

        next_start = self._view_start_s
        command = args[0] if args else ""
        if command == "moveto" and len(args) >= 2:
            try:
                fraction = float(args[1])
            except ValueError:
                return
            total_range = max(self._view_window_seconds, latest_time)
            next_start = fraction * total_range
        elif command == "scroll" and len(args) >= 3:
            try:
                steps = int(args[1])
            except ValueError:
                return
            mode = args[2]
            delta = self._view_window_seconds * 0.85 if mode == "pages" else ATTITUDE_CHART_SCROLL_UNIT_S
            next_start += steps * delta
        else:
            return

        next_start = max(0.0, min(max_start, next_start))
        self._view_start_s = next_start
        self._auto_follow = next_start >= (max_start - 1e-6)
        self._redraw(force=True)

    def _update_scrollbar_thumb(self) -> None:
        if self._x_scrollbar is None:
            return
        if not self._times:
            self._x_scrollbar.set(0.0, 1.0)
            return

        latest_time = self._times[-1]
        if latest_time <= self._view_window_seconds:
            self._x_scrollbar.set(0.0, 1.0)
            return

        total_range = latest_time
        lo = max(0.0, min(1.0, self._view_start_s / total_range))
        hi = max(lo, min(1.0, (self._view_start_s + self._view_window_seconds) / total_range))
        self._x_scrollbar.set(lo, hi)

    def _redraw(self, force: bool = False) -> None:
        if self._axis is None or self._canvas is None or self._line_roll is None or self._line_pitch is None:
            return

        now = time.monotonic()
        if not force and (now - self._last_draw_s < ATTITUDE_CHART_DRAW_INTERVAL_S):
            return
        self._last_draw_s = now

        if not self._times:
            self._line_roll.set_data([], [])
            self._line_pitch.set_data([], [])
            self._axis.set_xlim(0.0, self._view_window_seconds)
            self._axis.set_ylim(-20.0, 20.0)
            self._update_scrollbar_thumb()
            self._canvas.draw_idle()
            return

        if self._auto_follow:
            self._view_start_s = self._max_view_start()

        view_end_s = self._view_start_s + self._view_window_seconds
        left = bisect_left(self._times, self._view_start_s)
        right = bisect_right(self._times, view_end_s)
        if left > 0:
            left -= 1
        if right < len(self._times):
            right += 1

        x_values = self._times[left:right]
        roll_values = self._rolls[left:right]
        pitch_values = self._pitches[left:right]
        self._line_roll.set_data(x_values, roll_values)
        self._line_pitch.set_data(x_values, pitch_values)

        visible_values = roll_values + pitch_values
        max_abs = max(abs(v) for v in visible_values) if visible_values else 0.0
        y_limit = max(10.0, LEVEL_DEADBAND_DEG + 2.0, max_abs + 2.0)
        self._axis.set_xlim(self._view_start_s, max(view_end_s, self._view_start_s + 0.1))
        self._axis.set_ylim(-y_limit, y_limit)

        self._update_scrollbar_thumb()
        self._canvas.draw_idle()

        latest_roll = self._rolls[-1]
        latest_pitch = self._pitches[-1]
        if self._auto_follow:
            self._status.set(
                f"Live FC chart: Roll {latest_roll:+5.1f} deg, Pitch {latest_pitch:+5.1f} deg (t={self._times[-1]:.1f}s)"
            )
        else:
            self._status.set(
                f"History view {self._view_start_s:.1f}s to {view_end_s:.1f}s. Last sample Roll {latest_roll:+5.1f}, Pitch {latest_pitch:+5.1f}."
            )

    def add_sample(self, sample: AttitudeSample) -> None:
        if self._axis is None or self._canvas is None or self._line_roll is None or self._line_pitch is None:
            return
        sample_time = time.monotonic()
        if self._start_time_s is None:
            self._start_time_s = sample_time
        t_s = sample_time - self._start_time_s

        self._times.append(t_s)
        self._rolls.append(sample.roll_deg)
        self._pitches.append(sample.pitch_deg)
        if self._frozen:
            return
        self._redraw()


@dataclass
class MainUi:
    port_entry: tk.Entry | ttk.Combobox
    channel_adjust_canvases: list[tk.Canvas]
    target_adjust_canvases: list[tk.Canvas]
    ch_entries: list[tk.Entry]
    off_entries: list[tk.Entry]
    target_entries: list[tk.Entry]
    dur_entries: list[tk.Entry]
    angle_entries: list[tk.Entry]
    channel_output_canvases: list[tk.Canvas]
    channel_output_fill_ids: list[int]
    hold_send_canvases: list[tk.Canvas]
    hold_end_buttons: list[tk.Button]
    level_button: tk.Button
    status: tk.StringVar
    pc_link_box: tk.Label
    horizon: ArtificialHorizon
    roll_text: tk.StringVar
    pitch_text: tk.StringVar
    roll_pidff_vars: list[tk.StringVar]
    pitch_pidff_vars: list[tk.StringVar]
    pid_ff_adjust_canvases: list[tk.Canvas]
    fc_port_entry: tk.Entry | ttk.Combobox
    fc_baud_entry: tk.Entry
    scan_fc_button: tk.Button
    connect_fc_button: tk.Button
    arduino_button: tk.Button
    chart_status: tk.StringVar
    chart_strip_canvas: tk.Canvas
    chart_strip_frame: tk.Frame
    attitude_chart: AttitudeChartPanel


@dataclass
class PulseChartCapture:
    axis: str
    channel_index: int
    target_us: int
    reference_us: int
    command_delta_us: int
    timeout_s: float
    start_monotonic: float
    samples: list[tuple[float, float]] = field(default_factory=list)
    restore_monotonic: float | None = None
    restore_reason: str = ""


def build_main_gui(root: tk.Tk) -> MainUi:
    root.title("PPM Modbus")
    root.resizable(False, False)
    for col in range(6):
        root.grid_columnconfigure(col, weight=1)

    layout_grid = tk.Frame(root)
    layout_grid.grid(row=0, column=0, columnspan=6, padx=6, pady=6, sticky="we")
    layout_grid.grid_columnconfigure(0, weight=3)  # 60%
    layout_grid.grid_columnconfigure(1, weight=2)  # 40%

    main_frame = tk.LabelFrame(layout_grid, text="Main Controls", padx=6, pady=4)
    main_frame.grid(row=0, column=0, padx=(0, 4), pady=(0, 6), sticky="nsew")

    for i, channel_name in enumerate(("Roll", "Pitch", "Throttle", "Yaw"), start=1):
        tk.Label(main_frame, text=channel_name).grid(row=1, column=i, padx=4)

    tk.Label(main_frame, text="Adjust").grid(row=2, column=0, padx=6, pady=(0, 2), sticky="e")
    channel_adjust_canvases: list[tk.Canvas] = []
    for i in range(1, 5):
        width = 52
        height = 18
        canvas = tk.Canvas(main_frame, width=width, height=height, bg="#F0F0F0", highlightthickness=0)
        mid_x = width // 2
        canvas.create_rectangle(1, 1, mid_x, height - 1, fill="#C94B4B", outline="")
        canvas.create_rectangle(mid_x, 1, width - 1, height - 1, fill="#4CAF50", outline="")
        canvas.create_line(mid_x, 1, mid_x, height - 1, fill="white", width=2)
        canvas.create_text(13, height // 2, text="-", fill="white", font=("Segoe UI", 11, "bold"))
        canvas.create_text(width - 13, height // 2, text="+", fill="white", font=("Segoe UI", 11, "bold"))
        canvas.grid(row=2, column=i, padx=4, pady=(0, 2))
        channel_adjust_canvases.append(canvas)

    ch_entries = make_row(main_frame, 3, "Channels", CHANNEL_DEFAULTS)
    off_entries = make_row(main_frame, 4, "Offsets", OFFSET_DEFAULTS)
    tk.Label(main_frame, text="Adjust Tgt").grid(row=5, column=0, padx=6, pady=(0, 2), sticky="e")
    target_adjust_canvases: list[tk.Canvas] = []
    for i in range(1, 5):
        width = 52
        height = 18
        canvas = tk.Canvas(main_frame, width=width, height=height, bg="#F0F0F0", highlightthickness=0)
        mid_x = width // 2
        canvas.create_rectangle(1, 1, mid_x, height - 1, fill="#C94B4B", outline="")
        canvas.create_rectangle(mid_x, 1, width - 1, height - 1, fill="#4CAF50", outline="")
        canvas.create_line(mid_x, 1, mid_x, height - 1, fill="white", width=2)
        canvas.create_text(13, height // 2, text="-", fill="white", font=("Segoe UI", 11, "bold"))
        canvas.create_text(width - 13, height // 2, text="+", fill="white", font=("Segoe UI", 11, "bold"))
        canvas.grid(row=5, column=i, padx=4, pady=(0, 2))
        target_adjust_canvases.append(canvas)

    target_entries = make_row(main_frame, 6, "Targets", PULSE_TARGET_DEFAULTS)
    dur_entries = make_row(main_frame, 7, "Duration", PULSE_DURATION_DEFAULTS)

    tk.Label(main_frame, text="Angle").grid(row=8, column=0, padx=6, pady=(0, 2), sticky="e")
    angle_entries: list[tk.Entry] = []
    for i in range(1, 5):
        entry = tk.Entry(main_frame, width=8)
        entry.insert(0, "0")
        entry.grid(row=8, column=i, padx=4, pady=(0, 2))
        angle_entries.append(entry)

    tk.Label(main_frame, text="Idle").grid(row=9, column=0, padx=6, pady=(0, 2), sticky="e")
    channel_output_canvases: list[tk.Canvas] = []
    channel_output_fill_ids: list[int] = []
    for i in range(1, 5):
        canvas = tk.Canvas(main_frame, width=96, height=14, bg="#F0F0F0", highlightthickness=0)
        canvas.create_rectangle(1, 2, 95, 12, fill="#E6EBF0", outline="#B4BEC8")
        canvas.create_line(48, 2, 48, 12, fill="#8F98A3")
        fill_id = canvas.create_rectangle(48, 3, 48, 11, fill="#94D98F", outline="")
        canvas.grid(row=9, column=i, padx=4, pady=(0, 2))
        channel_output_canvases.append(canvas)
        channel_output_fill_ids.append(fill_id)

    tk.Label(main_frame, text="Pulse").grid(row=10, column=0, padx=6, pady=1, sticky="e")
    hold_send_canvases: list[tk.Canvas] = []
    for i in range(4):
        width = 52
        height = 18
        canvas = tk.Canvas(main_frame, width=width, height=height, bg="#F0F0F0", highlightthickness=0)
        mid_x = width // 2
        canvas.create_rectangle(1, 1, mid_x, height - 1, fill="#C94B4B", outline="")
        canvas.create_rectangle(mid_x, 1, width - 1, height - 1, fill="#4CAF50", outline="")
        canvas.create_line(mid_x, 1, mid_x, height - 1, fill="white", width=2)
        canvas.create_text(13, height // 2, text="-", fill="white", font=("Segoe UI", 11, "bold"))
        canvas.create_text(width - 13, height // 2, text="+", fill="white", font=("Segoe UI", 11, "bold"))
        canvas.grid(row=10, column=i + 1, padx=4, pady=1)
        hold_send_canvases.append(canvas)

    tk.Label(main_frame, text="End").grid(row=11, column=0, padx=6, pady=1, sticky="e")
    hold_end_buttons: list[tk.Button] = []
    for i in range(4):
        button = tk.Button(main_frame, text="End", width=8)
        button.grid(row=11, column=i + 1, pady=1)
        hold_end_buttons.append(button)

    status = tk.StringVar(value="Idle")
    pc_link_box = tk.Label(main_frame, width=18, relief="groove", bd=2)

    charts_frame = tk.LabelFrame(layout_grid, text="Pulse Movement Charts", padx=6, pady=6)
    charts_frame.grid(row=1, column=0, padx=(0, 4), sticky="nsew")
    chart_status = tk.StringVar(
        value="Pulse Roll or Pitch to append charts here. Baseline is derived after neutral restore."
    )
    tk.Label(charts_frame, textvariable=chart_status, justify="left", anchor="w", wraplength=540).pack(
        fill="x", pady=(0, 4)
    )

    chart_shell = tk.Frame(charts_frame)
    chart_shell.pack(fill="both", expand=True)
    chart_strip_canvas = tk.Canvas(
        chart_shell,
        width=560,
        height=268,
        bg="#F4F8FB",
        bd=0,
        highlightthickness=1,
        highlightbackground="#B5C0CB",
    )
    chart_scrollbar = tk.Scrollbar(chart_shell, orient="horizontal", command=chart_strip_canvas.xview)
    chart_strip_frame = tk.Frame(chart_strip_canvas, bg="#F4F8FB")
    chart_strip_window = chart_strip_canvas.create_window((0, 0), window=chart_strip_frame, anchor="nw")
    chart_strip_canvas.configure(xscrollcommand=chart_scrollbar.set)
    chart_strip_canvas.grid(row=0, column=0, sticky="we")
    chart_scrollbar.grid(row=1, column=0, sticky="we")
    chart_shell.grid_columnconfigure(0, weight=1)

    def update_chart_scroll_region(_event: tk.Event | None = None) -> None:
        bbox = chart_strip_canvas.bbox("all")
        if bbox is None:
            chart_strip_canvas.configure(scrollregion=(0, 0, 0, 0))
            return
        chart_strip_canvas.configure(scrollregion=bbox)

    def keep_chart_strip_height(event: tk.Event) -> None:
        chart_strip_canvas.itemconfigure(chart_strip_window, height=max(236, event.height - 14))

    def chart_mousewheel_horizontal(event: tk.Event) -> None:
        if event.delta:
            chart_strip_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")

    chart_strip_frame.bind("<Configure>", update_chart_scroll_region)
    chart_strip_canvas.bind("<Configure>", keep_chart_strip_height)
    chart_strip_canvas.bind("<Shift-MouseWheel>", chart_mousewheel_horizontal)

    fc_frame = tk.LabelFrame(layout_grid, text="FC / INAV", padx=8, pady=8)
    fc_frame.grid(row=0, column=1, padx=(4, 0), pady=(0, 6), sticky="nsew")
    horizon = ArtificialHorizon(fc_frame, size=220)
    roll_text = tk.StringVar(value="Roll: 0.0 deg")
    pitch_text = tk.StringVar(value="Pitch: 0.0 deg")
    pid_ff_labels = ("P", "I", "D", "FF")
    roll_pidff_vars: list[tk.StringVar] = []
    pitch_pidff_vars: list[tk.StringVar] = []
    pid_ff_adjust_canvases: list[tk.Canvas] = []
    left_metrics_frame = tk.Frame(fc_frame, bd=1, relief="groove", padx=6, pady=6)
    left_metrics_frame.grid(row=0, column=0, columnspan=2, sticky="nw", pady=(0, 6))
    tk.Label(left_metrics_frame, textvariable=roll_text, anchor="w", width=12).grid(
        row=0, column=0, columnspan=2, sticky="w", padx=(0, 6), pady=(0, 4)
    )
    tk.Label(left_metrics_frame, textvariable=pitch_text, anchor="w", width=12).grid(
        row=0, column=3, columnspan=2, sticky="w", pady=(0, 4)
    )
    for idx, gain_name in enumerate(pid_ff_labels, start=1):
        roll_var = tk.StringVar(value=f"{gain_name}: --")
        pitch_var = tk.StringVar(value=f"{gain_name}: --")
        tk.Entry(
            left_metrics_frame,
            width=8,
            textvariable=roll_var,
            justify="left",
            state="readonly",
            readonlybackground="#FFFFFF",
        ).grid(row=idx, column=0, padx=(0, 2), pady=1, sticky="w")
        roll_adjust = tk.Canvas(left_metrics_frame, width=32, height=16, bg="#F0F0F0", highlightthickness=0)
        roll_adjust.create_rectangle(1, 1, 16, 15, fill="#C94B4B", outline="")
        roll_adjust.create_rectangle(16, 1, 31, 15, fill="#4CAF50", outline="")
        roll_adjust.create_line(16, 1, 16, 15, fill="white", width=1)
        roll_adjust.create_text(8, 8, text="-", fill="white", font=("Segoe UI", 9, "bold"))
        roll_adjust.create_text(24, 8, text="+", fill="white", font=("Segoe UI", 9, "bold"))
        roll_adjust.grid(row=idx, column=1, padx=(0, 8), pady=1)
        tk.Entry(
            left_metrics_frame,
            width=8,
            textvariable=pitch_var,
            justify="left",
            state="readonly",
            readonlybackground="#FFFFFF",
        ).grid(row=idx, column=3, padx=(0, 2), pady=1, sticky="w")
        pitch_adjust = tk.Canvas(left_metrics_frame, width=32, height=16, bg="#F0F0F0", highlightthickness=0)
        pitch_adjust.create_rectangle(1, 1, 16, 15, fill="#C94B4B", outline="")
        pitch_adjust.create_rectangle(16, 1, 31, 15, fill="#4CAF50", outline="")
        pitch_adjust.create_line(16, 1, 16, 15, fill="white", width=1)
        pitch_adjust.create_text(8, 8, text="-", fill="white", font=("Segoe UI", 9, "bold"))
        pitch_adjust.create_text(24, 8, text="+", fill="white", font=("Segoe UI", 9, "bold"))
        pitch_adjust.grid(row=idx, column=4, pady=1)
        roll_pidff_vars.append(roll_var)
        pitch_pidff_vars.append(pitch_var)
        pid_ff_adjust_canvases.extend((roll_adjust, pitch_adjust))

    horizon.grid(row=0, column=2, rowspan=2, columnspan=2, padx=(10, 0), pady=(0, 4), sticky="n")

    port_fields_frame = tk.Frame(fc_frame)
    port_fields_frame.grid(row=1, column=0, columnspan=2, sticky="nw")
    tk.Label(port_fields_frame, text="Arduino Port").grid(row=0, column=0, sticky="e", padx=(0, 4))
    port_entry = ttk.Combobox(port_fields_frame, width=8, state="normal")
    port_entry.set(PORT_DEFAULT)
    port_entry.grid(row=0, column=1, sticky="w")

    tk.Label(port_fields_frame, text="Baud").grid(row=0, column=2, sticky="e", padx=(8, 4))
    port_baud_entry = tk.Entry(port_fields_frame, width=10)
    port_baud_entry.insert(0, str(BAUDRATE))
    port_baud_entry.config(state="readonly")
    port_baud_entry.grid(row=0, column=3, sticky="w")

    tk.Label(port_fields_frame, text="FC Port").grid(row=1, column=0, sticky="e", padx=(0, 4), pady=(4, 0))
    fc_port_entry = ttk.Combobox(port_fields_frame, width=8, state="normal")
    fc_port_entry.set(FC_PORT_DEFAULT)
    fc_port_entry.grid(row=1, column=1, sticky="w", pady=(4, 0))

    tk.Label(port_fields_frame, text="Baud").grid(row=1, column=2, sticky="e", padx=(8, 4), pady=(4, 0))
    fc_baud_entry = tk.Entry(port_fields_frame, width=10)
    fc_baud_entry.insert(0, str(FC_BAUD_DEFAULT))
    fc_baud_entry.grid(row=1, column=3, sticky="w", pady=(4, 0))

    fc_button_width = 13
    arduino_button_width = 18
    button_row = tk.Frame(fc_frame)
    button_row.grid(row=2, column=0, columnspan=4, sticky="w", pady=(2, 0))
    arduino_button = tk.Button(button_row, text="Connect Arduino", width=arduino_button_width)
    arduino_button.pack(side="left", padx=(0, 2))
    connect_fc_button = tk.Button(button_row, text="Connect FC", width=fc_button_width)
    connect_fc_button.pack(side="left", padx=(0, 2))
    scan_fc_button = tk.Button(button_row, text="Scan Ports", width=10)
    scan_fc_button.pack(side="left", padx=(0, 2))
    level_button = tk.Button(button_row, text="Level", width=6, state="disabled")
    level_button.pack(side="left")

    chart_frame = tk.LabelFrame(layout_grid, text="FC Chart", padx=8, pady=8)
    chart_frame.grid(row=1, column=1, padx=(4, 0), sticky="nsew")
    attitude_chart = AttitudeChartPanel(chart_frame, figure_size=(4.2, 2.8))

    return MainUi(
        port_entry=port_entry,
        channel_adjust_canvases=channel_adjust_canvases,
        target_adjust_canvases=target_adjust_canvases,
        ch_entries=ch_entries,
        off_entries=off_entries,
        target_entries=target_entries,
        dur_entries=dur_entries,
        angle_entries=angle_entries,
        channel_output_canvases=channel_output_canvases,
        channel_output_fill_ids=channel_output_fill_ids,
        hold_send_canvases=hold_send_canvases,
        hold_end_buttons=hold_end_buttons,
        level_button=level_button,
        status=status,
        pc_link_box=pc_link_box,
        horizon=horizon,
        roll_text=roll_text,
        pitch_text=pitch_text,
        roll_pidff_vars=roll_pidff_vars,
        pitch_pidff_vars=pitch_pidff_vars,
        pid_ff_adjust_canvases=pid_ff_adjust_canvases,
        fc_port_entry=fc_port_entry,
        fc_baud_entry=fc_baud_entry,
        scan_fc_button=scan_fc_button,
        connect_fc_button=connect_fc_button,
        arduino_button=arduino_button,
        chart_status=chart_status,
        chart_strip_canvas=chart_strip_canvas,
        chart_strip_frame=chart_strip_frame,
        attitude_chart=attitude_chart,
    )

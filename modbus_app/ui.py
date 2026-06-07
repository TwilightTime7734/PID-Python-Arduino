"""Tkinter UI components and layout builders."""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from collections.abc import Sequence
from dataclasses import dataclass

from .constants import (
    BAUDRATE,
    CHANNEL_DEFAULTS,
    FC_BAUD_DEFAULT,
    FC_PORT_DEFAULT,
    OFFSET_DEFAULTS,
    PORT_DEFAULT,
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


@dataclass
class MainUi:
    port_entry: tk.Entry | ttk.Combobox
    channel_adjust_canvases: list[tk.Canvas]
    ch_entries: list[tk.Entry]
    off_entries: list[tk.Entry]
    channel_output_canvases: list[tk.Canvas]
    channel_output_fill_ids: list[int]
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
    import_blackbox_button: tk.Button
    analyze_blackbox_button: tk.Button
    arduino_button: tk.Button
    auto_session_button: tk.Button
    fly_log_button: tk.Button
    simulation_mode_var: tk.BooleanVar
    simulation_mode_checkbutton: tk.Checkbutton
    pid_progress_button: tk.Button
    auto_report_text: tk.Text
    step_response_button: tk.Button
    pid_tuning_plan_button: tk.Button

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

    ch_entries = make_row(main_frame, 3, "Default", CHANNEL_DEFAULTS)
    off_entries = make_row(main_frame, 4, "Offsets", OFFSET_DEFAULTS)

    tk.Label(main_frame, text="Idle").grid(row=5, column=0, padx=6, pady=(0, 2), sticky="e")
    channel_output_canvases: list[tk.Canvas] = []
    channel_output_fill_ids: list[int] = []
    for i in range(1, 5):
        canvas = tk.Canvas(main_frame, width=96, height=14, bg="#F0F0F0", highlightthickness=0)
        canvas.create_rectangle(1, 2, 95, 12, fill="#E6EBF0", outline="#B4BEC8")
        canvas.create_line(48, 2, 48, 12, fill="#8F98A3")
        fill_id = canvas.create_rectangle(48, 3, 48, 11, fill="#94D98F", outline="")
        canvas.grid(row=5, column=i, padx=4, pady=(0, 2))
        channel_output_canvases.append(canvas)
        channel_output_fill_ids.append(fill_id)

    status = tk.StringVar(value="Idle")
    pc_link_box = tk.Label(main_frame, width=18, relief="groove", bd=2)

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
    import_blackbox_button = tk.Button(button_row, text="Pull MSC Logs", width=12)
    import_blackbox_button.pack(side="left", padx=(0, 2))
    analyze_blackbox_button = tk.Button(button_row, text="Analyze Logs", width=11)
    analyze_blackbox_button.pack(side="left", padx=(0, 2))
    level_button = tk.Button(button_row, text="Level", width=6, state="disabled")
    level_button.pack(side="left", padx=(0, 6))
    simulation_mode_var = tk.BooleanVar(value=False)
    simulation_mode_checkbutton = tk.Checkbutton(
        button_row,
        text="Simulate",
        variable=simulation_mode_var,
    )
    simulation_mode_checkbutton.pack(side="left")

    auto_frame = tk.LabelFrame(layout_grid, text="Auto Tune Session", padx=8, pady=8)
    auto_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(6, 0))
    auto_frame.grid_columnconfigure(0, weight=1)
    auto_frame.grid_rowconfigure(1, weight=1)

    auto_action_frame = tk.Frame(auto_frame)
    auto_action_frame.grid(row=0, column=0, sticky="w", pady=(0, 6))
    auto_session_button = tk.Button(auto_action_frame, text="Start Auto Session", width=18)
    auto_session_button.pack(side="left", padx=(0, 4))
    fly_log_button = tk.Button(auto_action_frame, text="Fly/Log", width=18, state="disabled")
    fly_log_button.pack(side="left", padx=(0, 4))
    pid_progress_button = tk.Button(auto_action_frame, text="Progress", width=10)
    pid_progress_button.pack(side="left", padx=(0, 4))
    step_response_button = tk.Button(auto_action_frame, text="Chart Step Response", width=18)
    step_response_button.pack(side="left", padx=(0, 4))
    pid_tuning_plan_button = tk.Button(auto_action_frame, text="PID Tuning Plan", width=16)
    pid_tuning_plan_button.pack(side="left")

    auto_report_text = tk.Text(auto_frame, width=100, height=12, wrap="word")
    auto_report_text.insert("1.0", "Report summary will appear here after an auto session.")
    auto_report_text.config(state="disabled")
    auto_report_text.grid(row=1, column=0, sticky="we", pady=(8, 0))

    tk.Label(
        root,
        textvariable=status,
        anchor="w",
        justify="left",
        relief="sunken",
        bd=1,
    ).grid(row=1, column=0, columnspan=6, sticky="we", padx=6, pady=(0, 6))

    return MainUi(
        port_entry=port_entry,
        channel_adjust_canvases=channel_adjust_canvases,
        ch_entries=ch_entries,
        off_entries=off_entries,
        channel_output_canvases=channel_output_canvases,
        channel_output_fill_ids=channel_output_fill_ids,
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
        import_blackbox_button=import_blackbox_button,
        analyze_blackbox_button=analyze_blackbox_button,
        arduino_button=arduino_button,
        auto_session_button=auto_session_button,
        fly_log_button=fly_log_button,
        simulation_mode_var=simulation_mode_var,
        simulation_mode_checkbutton=simulation_mode_checkbutton,
        pid_progress_button=pid_progress_button,
        auto_report_text=auto_report_text,
        step_response_button=step_response_button,
        pid_tuning_plan_button=pid_tuning_plan_button,
    )

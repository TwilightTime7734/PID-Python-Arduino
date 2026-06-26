"""Tkinter UI components and layout builders."""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from .constants import (
    BAUDRATE,
    CHANNEL_DEFAULTS,
    FC_BAUD_DEFAULT,
    FC_PORT_DEFAULT,
    OFFSET_DEFAULTS,
    PORT_DEFAULT,
)


EntryWidget = tk.Entry | ttk.Combobox
CHANNEL_VALUE_MIN = 1000
CHANNEL_VALUE_MAX = 2000
CHANNEL_VALUE_STEP = 25
CHANNEL_VALUE_OPTIONS = tuple(str(value) for value in range(CHANNEL_VALUE_MIN, CHANNEL_VALUE_MAX + 1, CHANNEL_VALUE_STEP))
OFFSET_VALUE_MIN = -20
OFFSET_VALUE_MAX = 20
OFFSET_VALUE_OPTIONS = tuple(str(value) for value in range(OFFSET_VALUE_MIN, OFFSET_VALUE_MAX + 1))
BAUD_OPTIONS = ("9600", "115200")
STARTING_VALUES_PENDING_BACKGROUND = "#FFE4E1"
STARTING_VALUES_SELECTED_BACKGROUND = "#90EE90"
STARTING_VALUES_NEUTRAL_BACKGROUND = "#F0F0F0"
STARTING_VALUES_NEUTRAL_FOREGROUND = "#555555"


def normalize_channel_value(value: int | float) -> int:
    clamped = max(CHANNEL_VALUE_MIN, min(CHANNEL_VALUE_MAX, int(round(float(value)))))
    stepped = CHANNEL_VALUE_MIN + (
        ((clamped - CHANNEL_VALUE_MIN) + (CHANNEL_VALUE_STEP // 2)) // CHANNEL_VALUE_STEP
    ) * CHANNEL_VALUE_STEP
    return max(CHANNEL_VALUE_MIN, min(CHANNEL_VALUE_MAX, int(stepped)))


def normalize_offset_value(value: int | float) -> int:
    return max(OFFSET_VALUE_MIN, min(OFFSET_VALUE_MAX, int(round(float(value)))))


def make_row(
    root: tk.Misc,
    row: int,
    label: str,
    defaults: Sequence[int | float],
    *,
    values: Sequence[str] | None = None,
    normalizer: Callable[[int | float], int] | None = None,
) -> list[EntryWidget]:
    tk.Label(root, text=label).grid(row=row, column=0, padx=6, pady=(0, 2), sticky="e")
    out: list[EntryWidget] = []
    for i, value in enumerate(defaults, 1):
        if values is None:
            entry: EntryWidget = tk.Entry(root, width=8)
            entry.insert(0, str(value))
        else:
            entry = ttk.Combobox(root, width=6, values=values, state="readonly")
            entry.set(str(normalizer(value) if normalizer is not None else value))
        entry.grid(row=row, column=i, padx=4, pady=(0, 2))
        out.append(entry)
    return out


def _readonly_table_entry(parent: tk.Misc, row: int, column: int, value: str, width: int = 8) -> tk.Entry:
    entry = tk.Entry(parent, width=width, justify="center", relief="sunken")
    entry.insert(0, value)
    entry.config(state="readonly")
    entry.grid(row=row, column=column, padx=2, pady=2, sticky="we")
    return entry


def _build_interactive_roll_pitch_table(
    parent: tk.Misc,
    title: str,
    rows: Sequence[tuple[str, Sequence[int | str]]],
    headings: Sequence[str],
    pidff_vars: list[tk.StringVar],
) -> tk.LabelFrame:
    """Build a Roll/Pitch table with row-exclusive clickable entries syncing to FC."""
    frame = tk.LabelFrame(parent, text=title, padx=6, pady=5)
    for column in range(len(headings)):
        frame.grid_columnconfigure(column, weight=1)
        tk.Label(frame, text=headings[column], anchor="center").grid(
            row=0, column=column, padx=2, pady=(0, 2), sticky="we"
        )
    
    param_to_index = {"P": 0, "I": 1, "D": 2, "FF": 3}
    # Tracks the currently active green entry widget per row parameter index
    active_selections: dict[int, tk.Entry] = {}
    
    for row_index, (param_name, values) in enumerate(rows, start=1):
        _readonly_table_entry(frame, row_index, 0, param_name, width=10)
        param_idx = param_to_index.get(param_name, -1)
        
        for column_index in range(1, len(headings)):
            value = values[column_index - 1] if column_index - 1 < len(values) else ""
            entry = tk.Entry(frame, width=8, justify="center", relief="sunken")
            entry.insert(0, str(value))
            entry.config(
                state="readonly",
                readonlybackground=STARTING_VALUES_PENDING_BACKGROUND,
                foreground="black"
            )
            entry.grid(row=row_index, column=column_index, padx=2, pady=2, sticky="we")

            def make_click_handler(current_entry, p_idx, p_name):
                def on_click(event):
                    # If this cell is already selected, unselect it and clear variable
                    if active_selections.get(p_idx) == current_entry:
                        current_entry.config(readonlybackground=STARTING_VALUES_PENDING_BACKGROUND)
                        active_selections.pop(p_idx, None)
                        if 0 <= p_idx < len(pidff_vars):
                            pidff_vars[p_idx].set(f"{p_name}: --")
                        return

                    # Clear previous selection in this specific row if it exists
                    if p_idx in active_selections:
                        active_selections[p_idx].config(readonlybackground=STARTING_VALUES_PENDING_BACKGROUND)

                    # Select new cell
                    current_entry.config(readonlybackground=STARTING_VALUES_SELECTED_BACKGROUND)
                    active_selections[p_idx] = current_entry

                    if 0 <= p_idx < len(pidff_vars):
                        pidff_vars[p_idx].set(f"{p_name}: {current_entry.get()}")
                return on_click
            
            if param_idx >= 0:
                entry.bind("<Button-1>", make_click_handler(entry, param_idx, param_name))
    
    return frame

def _build_interactive_pid_value_table(
    parent: tk.Misc,
    title: str,
    rows: Sequence[tuple[str, Sequence[int | str]]],
    headings: Sequence[str],
    roll_pidff_vars: list[tk.StringVar],
    pitch_pidff_vars: list[tk.StringVar],
) -> tk.LabelFrame:
    """Build a readonly PID value table with axis-exclusive clickable cells."""
    frame = tk.LabelFrame(parent, text=title, padx=6, pady=5)
    for column in range(len(headings)):
        frame.grid_columnconfigure(column, weight=1)
        tk.Label(frame, text=headings[column], anchor="center").grid(
            row=0, column=column, padx=2, pady=(0, 2), sticky="we"
        )

    # Track active selection per (axis, parameter) tuple to avoid global desync
    active_selections: dict[tuple[str, str], tk.Entry] = {}
    param_to_index = {"P": 0, "I": 1, "D": 2, "FF": 3}

    for row_index, (label, values) in enumerate(rows, start=1):
        clean_label = label.replace(" start", "").replace(" rec", "")

        axis_type = None
        if "Roll" in clean_label:
            axis_type = "roll"
        elif "Pitch" in clean_label:
            axis_type = "pitch"

        label_entry = _readonly_table_entry(frame, row_index, 0, clean_label, width=10)
        if axis_type is None:
            label_entry.config(
                state="disabled",
                disabledbackground=STARTING_VALUES_NEUTRAL_BACKGROUND,
                disabledforeground=STARTING_VALUES_NEUTRAL_FOREGROUND,
                takefocus=0,
            )

        for column_index in range(1, len(headings)):
            value = values[column_index - 1] if column_index - 1 < len(values) else ""
            entry = tk.Entry(frame, width=8, justify="center", relief="sunken")
            entry.insert(0, str(value))

            if axis_type is None:
                entry.config(
                    state="disabled",
                    disabledbackground=STARTING_VALUES_NEUTRAL_BACKGROUND,
                    disabledforeground=STARTING_VALUES_NEUTRAL_FOREGROUND,
                    takefocus=0,
                )
                entry.grid(row=row_index, column=column_index, padx=2, pady=2, sticky="we")
                continue

            entry.config(
                state="readonly",
                readonlybackground=STARTING_VALUES_PENDING_BACKGROUND,
                foreground="black",
            )
            entry.grid(row=row_index, column=column_index, padx=2, pady=2, sticky="we")

            param_name = str(headings[column_index]).upper()
            param_idx = param_to_index[param_name]
            key = (axis_type, param_name)

            def make_click_handler(current_entry, axis, p_name, p_idx, select_key):
                def on_click(event):
                    vars_list = roll_pidff_vars if axis == "roll" else pitch_pidff_vars

                    # If clicking the already selected cell, turn it off
                    if active_selections.get(select_key) == current_entry:
                        current_entry.config(readonlybackground=STARTING_VALUES_PENDING_BACKGROUND)
                        active_selections.pop(select_key, None)
                        if p_idx < len(vars_list):
                            vars_list[p_idx].set(f"{p_name}: --")
                        return

                    # Un-green the old selection for this parameter/axis variant
                    if select_key in active_selections:
                        active_selections[select_key].config(readonlybackground=STARTING_VALUES_PENDING_BACKGROUND)

                    # Stage the new choice
                    current_entry.config(readonlybackground=STARTING_VALUES_SELECTED_BACKGROUND)
                    active_selections[select_key] = current_entry

                    if p_idx < len(vars_list):
                        vars_list[p_idx].set(f"{p_name}: {current_entry.get()}")
                return on_click

            entry.bind("<Button-1>", make_click_handler(entry, axis_type, param_name, param_idx, key))

    return frame

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
    ch_entries: list[EntryWidget]
    off_entries: list[EntryWidget]
    channel_output_canvases: list[tk.Canvas]
    channel_output_fill_ids: list[int]
    level_button: tk.Button
    status: tk.StringVar
    horizon: ArtificialHorizon
    roll_text: tk.StringVar
    pitch_text: tk.StringVar
    roll_pidff_vars: list[tk.StringVar]
    pitch_pidff_vars: list[tk.StringVar]
    starting_values_table: tk.LabelFrame
    roll_values_table: tk.LabelFrame
    pitch_values_table: tk.LabelFrame
    pid_ff_adjust_canvases: list[tk.Canvas]
    load_pid_ff_button: tk.Button
    save_pid_ff_button: tk.Button
    port_baud_entry: ttk.Combobox
    fc_port_entry: tk.Entry | ttk.Combobox
    fc_baud_entry: ttk.Combobox
    scan_fc_button: tk.Button
    connect_fc_button: tk.Button
    import_blackbox_button: tk.Button
    analyze_blackbox_button: tk.Button
    arduino_button: tk.Button
    fly_log_button: tk.Button
    pulse_axis_combo: ttk.Combobox
    pulse_force_combo: ttk.Combobox
    pulse_time_combo: ttk.Combobox
    pulse_positive_button: tk.Button
    pulse_negative_button: tk.Button
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

    ch_entries = make_row(
        main_frame,
        3,
        "Default",
        CHANNEL_DEFAULTS,
        values=CHANNEL_VALUE_OPTIONS,
        normalizer=normalize_channel_value,
    )
    off_entries = make_row(
        main_frame,
        4,
        "Offsets",
        OFFSET_DEFAULTS,
        values=OFFSET_VALUE_OPTIONS,
        normalizer=normalize_offset_value,
    )

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

    main_bottom_frame = tk.Frame(main_frame)
    main_bottom_frame.grid(row=6, column=0, columnspan=5, sticky="we", pady=(6, 0))
    main_button_grid = tk.Frame(main_bottom_frame)
    main_button_grid.grid(row=0, column=0, sticky="nw")
    for col in range(2):
        main_button_grid.grid_columnconfigure(col, weight=1, uniform="main_buttons")

    main_button_width = 15
    arduino_button = tk.Button(main_button_grid, text="Connect Arduino", width=main_button_width)
    arduino_button.grid(row=0, column=0, padx=2, pady=2, sticky="we")
    connect_fc_button = tk.Button(main_button_grid, text="Connect FC", width=main_button_width)
    connect_fc_button.grid(row=0, column=1, padx=2, pady=2, sticky="we")

    import_blackbox_button = tk.Button(main_button_grid, text="Pull MSC Logs", width=main_button_width)
    import_blackbox_button.grid(row=1, column=0, padx=2, pady=2, sticky="we")
    analyze_blackbox_button = tk.Button(main_button_grid, text="Analyze Logs", width=main_button_width)
    analyze_blackbox_button.grid(row=1, column=1, padx=2, pady=2, sticky="we")

    status = tk.StringVar(value="Idle")

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
        roll_entry = tk.Entry(
            left_metrics_frame,
            width=8,
            textvariable=roll_var,
            justify="left",
            state="readonly",
            readonlybackground="white",
            foreground="black",
        )
        roll_entry.grid(row=idx, column=0, padx=(0, 2), pady=1, sticky="w")
        roll_adjust = tk.Canvas(left_metrics_frame, width=32, height=16, bg="#F0F0F0", highlightthickness=0)
        roll_adjust.create_rectangle(1, 1, 16, 15, fill="#C94B4B", outline="")
        roll_adjust.create_rectangle(16, 1, 31, 15, fill="#4CAF50", outline="")
        roll_adjust.create_line(16, 1, 16, 15, fill="white", width=1)
        roll_adjust.create_text(8, 8, text="-", fill="white", font=("Segoe UI", 9, "bold"))
        roll_adjust.create_text(24, 8, text="+", fill="white", font=("Segoe UI", 9, "bold"))
        roll_adjust.grid(row=idx, column=1, padx=(0, 8), pady=1)
        pitch_entry = tk.Entry(
            left_metrics_frame,
            width=8,
            textvariable=pitch_var,
            justify="left",
            state="readonly",
            readonlybackground="white",
            foreground="black",
        )
        pitch_entry.grid(row=idx, column=3, padx=(0, 2), pady=1, sticky="w")
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

    pid_load_save_frame = tk.Frame(fc_frame)
    pid_load_save_frame.grid(row=0, column=2, rowspan=2, padx=(8, 0), pady=(36, 0), sticky="n")
    load_pid_ff_button = tk.Button(pid_load_save_frame, text="Load", width=8)
    load_pid_ff_button.grid(row=0, column=0, pady=(0, 4), sticky="we")
    save_pid_ff_button = tk.Button(pid_load_save_frame, text="Save", width=8)
    save_pid_ff_button.grid(row=1, column=0, sticky="we")

    horizon.grid(row=0, column=3, rowspan=2, columnspan=2, padx=(10, 0), pady=(0, 4), sticky="n")

    port_fields_frame = tk.Frame(fc_frame)
    port_fields_frame.grid(row=1, column=0, columnspan=2, sticky="nw")
    tk.Label(port_fields_frame, text="Arduino Port").grid(row=0, column=0, sticky="e", padx=(0, 4))
    port_entry = ttk.Combobox(port_fields_frame, width=8, state="normal")
    port_entry.set(PORT_DEFAULT)
    port_entry.grid(row=0, column=1, sticky="w")

    tk.Label(port_fields_frame, text="Baud").grid(row=0, column=2, sticky="e", padx=(8, 4))
    port_baud_entry = ttk.Combobox(port_fields_frame, width=8, values=BAUD_OPTIONS, state="readonly")
    port_baud_entry.set(str(BAUDRATE))
    port_baud_entry.grid(row=0, column=3, sticky="w")

    tk.Label(port_fields_frame, text="FC Port").grid(row=1, column=0, sticky="e", padx=(0, 4), pady=(4, 0))
    fc_port_entry = ttk.Combobox(port_fields_frame, width=8, state="normal")
    fc_port_entry.set(FC_PORT_DEFAULT)
    fc_port_entry.grid(row=1, column=1, sticky="w", pady=(4, 0))

    tk.Label(port_fields_frame, text="Baud").grid(row=1, column=2, sticky="e", padx=(8, 4), pady=(4, 0))
    fc_baud_entry = ttk.Combobox(port_fields_frame, width=8, values=BAUD_OPTIONS, state="readonly")
    fc_baud_entry.set(str(FC_BAUD_DEFAULT))
    fc_baud_entry.grid(row=1, column=3, sticky="w", pady=(4, 0))
    fc_action_frame = tk.Frame(port_fields_frame)
    fc_action_frame.grid(row=2, column=0, columnspan=4, sticky="we", pady=(6, 0))
    fc_action_frame.grid_columnconfigure(0, weight=1, uniform="fc_actions")
    fc_action_frame.grid_columnconfigure(1, weight=1, uniform="fc_actions")
    scan_fc_button = tk.Button(fc_action_frame, text="Scan Ports", width=12)
    scan_fc_button.grid(row=0, column=0, padx=(0, 2), sticky="we")
    level_button = tk.Button(fc_action_frame, text="Level", width=12, state="disabled")
    level_button.grid(row=0, column=1, padx=(2, 0), sticky="we")

    auto_frame = tk.LabelFrame(layout_grid, text="Tune Layout / Progress", padx=8, pady=8)
    auto_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(6, 0))
    auto_frame.grid_columnconfigure(0, weight=1)
    auto_frame.grid_rowconfigure(1, weight=1)

    auto_action_frame = tk.Frame(auto_frame)
    auto_action_frame.grid(row=0, column=0, sticky="w", pady=(0, 6))
    fly_log_button = tk.Button(auto_action_frame, text="Fly / Log", width=18)
    fly_log_button.pack(side="left", padx=(0, 4))
    tk.Label(auto_action_frame, text="Axis").pack(side="left", padx=(4, 2))
    pulse_axis_combo = ttk.Combobox(
        auto_action_frame,
        width=7,
        values=("Roll", "Pitch"),
        state="readonly",
    )
    pulse_axis_combo.set("Roll")
    pulse_axis_combo.pack(side="left", padx=(0, 4))
    tk.Label(auto_action_frame, text="Pulse us").pack(side="left", padx=(4, 2))
    pulse_force_combo = ttk.Combobox(
        auto_action_frame,
        width=5,
        values=("60", "70", "80", "90", "100", "110", "125", "150"),
        state="readonly",
    )
    pulse_force_combo.pack(side="left", padx=(0, 4))
    tk.Label(auto_action_frame, text="Board s").pack(side="left", padx=(2, 2))
    pulse_time_combo = ttk.Combobox(
        auto_action_frame,
        width=5,
        values=("0.10",),
        state="readonly",
    )
    pulse_time_combo.set("0.10")
    pulse_time_combo.pack(side="left", padx=(0, 4))
    pulse_negative_button = tk.Button(
        auto_action_frame,
        text="- Pulse",
        width=8,
        bg="#dc3545",
        fg="white",
        activebackground="#bb2d3b",
        activeforeground="white",
    )
    pulse_negative_button.pack(side="left", padx=(0, 4))
    pulse_positive_button = tk.Button(
        auto_action_frame,
        text="+ Pulse",
        width=8,
        bg="#198754",
        fg="white",
        activebackground="#157347",
        activeforeground="white",
    )
    pulse_positive_button.pack(side="left", padx=(0, 8))
    step_response_button = tk.Button(auto_action_frame, text="Chart Step Response", width=18)
    step_response_button.pack(side="left", padx=(0, 4))
    pid_tuning_plan_button = tk.Button(auto_action_frame, text="PID Tuning Plan", width=16)
    pid_tuning_plan_button.pack(side="left")

    pid_table_frame = tk.Frame(auto_frame)
    pid_table_frame.grid(row=1, column=0, sticky="we", pady=(8, 0))
    for column in range(3):
        pid_table_frame.grid_columnconfigure(column, weight=1, uniform="pid_tables")

    starting_values_table = _build_interactive_pid_value_table(
        pid_table_frame,
        "Starting values",
        (
            ("Roll start", ("00", "00", "00", "00")),
            ("Pitch start", ("00", "00", "00", "00")),
            ("Yaw start", ("00", "00", "00", "00")),
            ("Yaw rec", ("00", "00", "00", "00")),
        ),
        ("Axis", "P", "I", "D", "FF"),
        roll_pidff_vars,
        pitch_pidff_vars,
    )
    starting_values_table.grid(row=0, column=0, padx=(0, 4), sticky="nwe")

    roll_values_table = _build_interactive_roll_pitch_table(
        pid_table_frame,
        "Roll",
        (
            ("P", ("00", "00", "00", "00", "00")),
            ("I", ("00", "00", "00", "00", "00")),
            ("D", ("00", "00", "00", "00", "00")),
            ("FF", ("00", "00", "00", "00", "00")),
        ),
        ("Gain", "1", "2", "3", "4", "5"),
        roll_pidff_vars,
    )
    roll_values_table.grid(row=0, column=1, padx=4, sticky="nwe")

    pitch_values_table = _build_interactive_roll_pitch_table(
        pid_table_frame,
        "Pitch",
        (
            ("P", ("00", "00", "00", "00", "00")),
            ("I", ("00", "00", "00", "00", "00")),
            ("D", ("00", "00", "00", "00", "00")),
            ("FF", ("00", "00", "00", "00", "00")),
        ),
        ("Gain", "1", "2", "3", "4", "5"),
        pitch_pidff_vars,
    )
    pitch_values_table.grid(row=0, column=2, padx=(4, 0), sticky="nwe")

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
        horizon=horizon,
        roll_text=roll_text,
        pitch_text=pitch_text,
        roll_pidff_vars=roll_pidff_vars,
        pitch_pidff_vars=pitch_pidff_vars,
        starting_values_table=starting_values_table,
        roll_values_table=roll_values_table,
        pitch_values_table=pitch_values_table,
        pid_ff_adjust_canvases=pid_ff_adjust_canvases,
        load_pid_ff_button=load_pid_ff_button,
        save_pid_ff_button=save_pid_ff_button,
        port_baud_entry=port_baud_entry,
        fc_port_entry=fc_port_entry,
        fc_baud_entry=fc_baud_entry,
        scan_fc_button=scan_fc_button,
        connect_fc_button=connect_fc_button,
        import_blackbox_button=import_blackbox_button,
        analyze_blackbox_button=analyze_blackbox_button,
        arduino_button=arduino_button,
        fly_log_button=fly_log_button,
        pulse_axis_combo=pulse_axis_combo,
        pulse_force_combo=pulse_force_combo,
        pulse_time_combo=pulse_time_combo,
        pulse_positive_button=pulse_positive_button,
        pulse_negative_button=pulse_negative_button,
        step_response_button=step_response_button,
        pid_tuning_plan_button=pid_tuning_plan_button,
    )

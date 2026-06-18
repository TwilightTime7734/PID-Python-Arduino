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


def _readonly_table_entry(parent: tk.Misc, row: int, column: int, value: str, width: int = 8) -> tk.Entry:
    entry = tk.Entry(parent, width=width, justify="center", relief="sunken")
    entry.insert(0, value)
    entry.config(state="readonly")
    entry.grid(row=row, column=column, padx=2, pady=2, sticky="we")
    return entry


def _build_pid_value_table(
    parent: tk.Misc,
    title: str,
    rows: Sequence[tuple[str, Sequence[int | str]]],
    headings: Sequence[str],
) -> tk.LabelFrame:
    frame = tk.LabelFrame(parent, text=title, padx=6, pady=5)
    for column in range(len(headings)):
        frame.grid_columnconfigure(column, weight=1)
        tk.Label(frame, text=headings[column], anchor="center").grid(
            row=0,
            column=column,
            padx=2,
            pady=(0, 2),
            sticky="we",
        )
    for row_index, (label, values) in enumerate(rows, start=1):
        _readonly_table_entry(frame, row_index, 0, label, width=10)
        for column_index in range(1, len(headings)):
            value = values[column_index - 1] if column_index - 1 < len(values) else ""
            _readonly_table_entry(frame, row_index, column_index, str(value))
    return frame


def _build_interactive_roll_pitch_table(
    parent: tk.Misc,
    title: str,
    rows: Sequence[tuple[str, Sequence[int | str]]],
    headings: Sequence[str],
    pidff_vars: list[tk.StringVar],
) -> tk.LabelFrame:
    """Build a Roll/Pitch table with clickable entries that sync to FC/INAV section."""
    frame = tk.LabelFrame(parent, text=title, padx=6, pady=5)
    for column in range(len(headings)):
        frame.grid_columnconfigure(column, weight=1)
        tk.Label(frame, text=headings[column], anchor="center").grid(
            row=0,
            column=column,
            padx=2,
            pady=(0, 2),
            sticky="we",
        )
    
    # Store entry widget references and their state for color toggling
    entry_states: dict[tk.Entry, dict] = {}
    
    # Map parameter names to their index in pidff_vars
    # pidff_vars is indexed as: P=0, I=1, D=2, FF=3
    param_to_index = {"P": 0, "I": 1, "D": 2, "FF": 3}
    
    for row_index, (param_name, values) in enumerate(rows, start=1):
        _readonly_table_entry(frame, row_index, 0, param_name, width=10)
        param_idx = param_to_index.get(param_name, -1)
        
        for column_index in range(1, len(headings)):
            value = values[column_index - 1] if column_index - 1 < len(values) else ""
            entry = tk.Entry(frame, width=8, justify="center", relief="sunken")
            entry.insert(0, str(value))
            entry.config(state="readonly", readonlybackground="#FFE4E1", foreground="black")
            entry.grid(row=row_index, column=column_index, padx=2, pady=2, sticky="we")
            
            # Initialize state: light red background, not toggled
            entry_states[entry] = {
                "toggled": False,
                "param_idx": param_idx,
                "param_name": param_name,
            }
            
            def make_click_handler(entry_widget, state_dict):
                def on_click(event):
                    # Toggle state
                    state_dict[entry_widget]["toggled"] = not state_dict[entry_widget]["toggled"]
                    
                    if state_dict[entry_widget]["toggled"]:
                        # Light green
                        entry_widget.config(readonlybackground="#90EE90")
                        # Update FC/INAV section
                        value_text = entry_widget.get()
                        param_idx = state_dict[entry_widget]["param_idx"]
                        param_name = state_dict[entry_widget]["param_name"]
                        
                        if param_idx >= 0 and param_idx < len(pidff_vars):
                            pidff_vars[param_idx].set(f"{param_name}: {value_text}")
                    else:
                        # Light red
                        entry_widget.config(readonlybackground="#FFE4E1")
                return on_click
            
            entry.bind("<Button-1>", make_click_handler(entry, entry_states))
    
    return frame


def _build_interactive_pid_value_table(
    parent: tk.Misc,
    title: str,
    rows: Sequence[tuple[str, Sequence[int | str]]],
    headings: Sequence[str],
    roll_pidff_vars: list[tk.StringVar],
    pitch_pidff_vars: list[tk.StringVar],
) -> tk.LabelFrame:
    """Build a PID value table with clickable entries that sync to FC/INAV section."""
    frame = tk.LabelFrame(parent, text=title, padx=6, pady=5)
    for column in range(len(headings)):
        frame.grid_columnconfigure(column, weight=1)
        tk.Label(frame, text=headings[column], anchor="center").grid(
            row=0,
            column=column,
            padx=2,
            pady=(0, 2),
            sticky="we",
        )
    
    # Store entry widget references and their state for color toggling
    entry_states: dict[tk.Entry, dict] = {}
    
    for row_index, (label, values) in enumerate(rows, start=1):
        # Strip "start" and "rec" from the label
        clean_label = label.replace(" start", "").replace(" rec", "")
        _readonly_table_entry(frame, row_index, 0, clean_label, width=10)
        
        # Determine if this is Roll or Pitch row
        axis_type = None
        if "Roll" in clean_label:
            axis_type = "roll"
        elif "Pitch" in clean_label:
            axis_type = "pitch"
        # Yaw rows are ignored
        
        for column_index in range(1, len(headings)):
            value = values[column_index - 1] if column_index - 1 < len(values) else ""
            entry = tk.Entry(frame, width=8, justify="center", relief="sunken")
            entry.insert(0, str(value))
            entry.config(state="readonly", readonlybackground="#FFE4E1", foreground="black")
            entry.grid(row=row_index, column=column_index, padx=2, pady=2, sticky="we")
            
            # Initialize state: light red background, not toggled
            entry_states[entry] = {
                "toggled": False,
                "axis_type": axis_type,
                "param_index": column_index - 1,  # 0-3 for P, D, I, FF
            }
            
            # Add click handler only for Roll and Pitch
            if axis_type is not None:
                def make_click_handler(entry_widget, state_dict):
                    def on_click(event):
                        # Toggle state
                        state_dict[entry_widget]["toggled"] = not state_dict[entry_widget]["toggled"]
                        
                        if state_dict[entry_widget]["toggled"]:
                            # Light green
                            entry_widget.config(readonlybackground="#90EE90")
                            # Update FC/INAV section
                            value_text = entry_widget.get()
                            axis = state_dict[entry_widget]["axis_type"]
                            param_idx = state_dict[entry_widget]["param_index"]
                            
                            if axis == "roll" and param_idx < len(roll_pidff_vars):
                                param_names = ["P", "D", "I", "FF"]
                                roll_pidff_vars[param_idx].set(f"{param_names[param_idx]}: {value_text}")
                            elif axis == "pitch" and param_idx < len(pitch_pidff_vars):
                                param_names = ["P", "D", "I", "FF"]
                                pitch_pidff_vars[param_idx].set(f"{param_names[param_idx]}: {value_text}")
                        else:
                            # Light red
                            entry_widget.config(readonlybackground="#FFE4E1")
                    return on_click
                
                entry.bind("<Button-1>", make_click_handler(entry, entry_states))
    
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
    starting_values_table: tk.LabelFrame
    pid_ff_adjust_canvases: list[tk.Canvas]
    load_pid_ff_button: tk.Button
    save_pid_ff_button: tk.Button
    fc_port_entry: tk.Entry | ttk.Combobox
    fc_baud_entry: tk.Entry
    scan_fc_button: tk.Button
    connect_fc_button: tk.Button
    import_blackbox_button: tk.Button
    analyze_blackbox_button: tk.Button
    arduino_button: tk.Button
    fly_log_button: tk.Button
    simulation_mode_var: tk.BooleanVar
    simulation_mode_checkbutton: tk.Checkbutton
    pid_progress_button: tk.Button
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

    main_button_grid = tk.Frame(main_frame)
    main_button_grid.grid(row=6, column=0, columnspan=5, sticky="we", pady=(6, 0))
    for col in range(4):
        main_button_grid.grid_columnconfigure(col, weight=1, uniform="main_buttons")

    main_button_width = 17
    arduino_button = tk.Button(main_button_grid, text="Connect Arduino", width=main_button_width)
    arduino_button.grid(row=0, column=0, padx=2, pady=2, sticky="we")
    connect_fc_button = tk.Button(main_button_grid, text="Connect FC", width=main_button_width)
    connect_fc_button.grid(row=0, column=1, padx=2, pady=2, sticky="we")
    scan_fc_button = tk.Button(main_button_grid, text="Scan Ports", width=main_button_width)
    scan_fc_button.grid(row=0, column=2, padx=2, pady=2, sticky="we")
    simulation_mode_var = tk.BooleanVar(value=False)
    simulation_mode_checkbutton = tk.Checkbutton(
        main_button_grid,
        text="Simulate",
        variable=simulation_mode_var,
        width=main_button_width,
        anchor="w",
    )
    simulation_mode_checkbutton.grid(row=0, column=3, padx=2, pady=2, sticky="we")

    import_blackbox_button = tk.Button(main_button_grid, text="Pull MSC Logs", width=main_button_width)
    import_blackbox_button.grid(row=1, column=0, padx=2, pady=2, sticky="we")
    analyze_blackbox_button = tk.Button(main_button_grid, text="Analyze Logs", width=main_button_width)
    analyze_blackbox_button.grid(row=1, column=1, padx=2, pady=2, sticky="we")
    level_button = tk.Button(main_button_grid, text="Level", width=main_button_width, state="disabled")
    level_button.grid(row=1, column=2, padx=2, pady=2, sticky="we")

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
        roll_entry = tk.Entry(
            left_metrics_frame,
            width=8,
            textvariable=roll_var,
            justify="left",
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

    auto_frame = tk.LabelFrame(layout_grid, text="Auto Tune Session", padx=8, pady=8)
    auto_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(6, 0))
    auto_frame.grid_columnconfigure(0, weight=1)
    auto_frame.grid_rowconfigure(1, weight=1)

    auto_action_frame = tk.Frame(auto_frame)
    auto_action_frame.grid(row=0, column=0, sticky="w", pady=(0, 6))
    fly_log_button = tk.Button(auto_action_frame, text="Fly/Log", width=18)
    fly_log_button.pack(side="left", padx=(0, 4))
    pid_progress_button = tk.Button(auto_action_frame, text="Progress", width=10)
    pid_progress_button.pack(side="left", padx=(0, 4))
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
        ("Axis", "P", "D", "I", "FF"),
        roll_pidff_vars,
        pitch_pidff_vars,
    )
    starting_values_table.grid(row=0, column=0, padx=(0, 4), sticky="nwe")

    roll_values_table = _build_interactive_roll_pitch_table(
        pid_table_frame,
        "Roll",
        (
            ("P", ("00", "00", "00", "00", "00")),
            ("D", ("00", "00", "00", "00", "00")),
            ("I", ("00", "00", "00", "00", "00")),
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
            ("D", ("00", "00", "00", "00", "00")),
            ("I", ("00", "00", "00", "00", "00")),
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
        pc_link_box=pc_link_box,
        horizon=horizon,
        roll_text=roll_text,
        pitch_text=pitch_text,
        roll_pidff_vars=roll_pidff_vars,
        pitch_pidff_vars=pitch_pidff_vars,
        starting_values_table=starting_values_table,
        pid_ff_adjust_canvases=pid_ff_adjust_canvases,
        load_pid_ff_button=load_pid_ff_button,
        save_pid_ff_button=save_pid_ff_button,
        fc_port_entry=fc_port_entry,
        fc_baud_entry=fc_baud_entry,
        scan_fc_button=scan_fc_button,
        connect_fc_button=connect_fc_button,
        import_blackbox_button=import_blackbox_button,
        analyze_blackbox_button=analyze_blackbox_button,
        arduino_button=arduino_button,
        fly_log_button=fly_log_button,
        simulation_mode_var=simulation_mode_var,
        simulation_mode_checkbutton=simulation_mode_checkbutton,
        pid_progress_button=pid_progress_button,
        step_response_button=step_response_button,
        pid_tuning_plan_button=pid_tuning_plan_button,
    )

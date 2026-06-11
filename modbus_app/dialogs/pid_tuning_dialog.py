"""PID tuning plan input dialog."""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox

from ..pid_tuning_workflow import (
    PAVO_PICO_II_PRESET_INPUTS,
    PStartInputs,
    safe_p_start_information_needed,
)


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


def ask_pid_tuning_inputs(parent: tk.Misc) -> PStartInputs | None:
    dialog = tk.Toplevel(parent)
    dialog.title("PID Tuning Plan")
    dialog.transient(parent)
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

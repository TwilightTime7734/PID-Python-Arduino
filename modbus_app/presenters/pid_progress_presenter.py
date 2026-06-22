"""PID tuning progress window presenter.

This module owns only the PID progress display/window logic. The PID plan state,
plan stepping, and hardware behavior still live in app.py/workflows for now.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any
import tkinter as tk


PidTarget = dict[str, dict[str, int]]
PidPlanStep = tuple[str, str, PidTarget] | None


class PidProgressPresenter:
    """Builds and refreshes the PID tuning progress window."""

    PHASES = (
        ("safe_start", "Safe Start"),
        ("p_sweep", "P Sweep"),
        ("d_sweep", "D Sweep"),
        ("i_sweep", "I Sweep"),
        ("ff_sweep", "FF Sweep"),
        ("final_write", "Final"),
    )
    PHASE_INDEX = {phase: index for index, (phase, _label) in enumerate(PHASES)}

    def __init__(
        self,
        app: Any,
        current_pid_plan_step: Callable[[], PidPlanStep],
        format_pid_values: Callable[[PidTarget], str],
        set_error: Callable[[str, Exception], None],
    ) -> None:
        self.app = app
        self.current_pid_plan_step = current_pid_plan_step
        self.format_pid_values = format_pid_values
        self.set_error = set_error

    def normalize_phase(self, phase: str) -> str:
        if phase in ("d_optional", "d_recheck"):
            return "d_sweep"
        return phase

    def active_phase(self) -> str:
        app = self.app
        if app.pid_plan_phase == "complete":
            return "complete"
        if app.pid_plan_waiting_for_fly_log or app.pid_plan_fly_log_active:
            return self.normalize_phase(app.pid_plan_current_candidate_phase or app.pid_plan_phase)
        return self.normalize_phase(app.pid_plan_phase)

    def target(self) -> PidTarget | None:
        app = self.app
        if app.pid_plan_waiting_for_fly_log or app.pid_plan_fly_log_active:
            return app.pid_plan_current_candidate_target
        if app.pid_plan is None or app.pid_plan_phase in ("idle", "complete"):
            return None
        try:
            step = self.current_pid_plan_step()
        except Exception:
            return None
        if step is None:
            return None
        return step[2]

    def title(self) -> str:
        app = self.app
        if app.pid_plan is None:
            return "No PID tuning plan is active."
        if app.pid_plan_phase == "complete":
            return "PID tuning plan complete."
        if app.pid_plan_waiting_for_fly_log or app.pid_plan_fly_log_active:
            return app.pid_plan_current_candidate_title or "Current candidate"
        if app.pid_plan_phase == "final_write":
            return "Final values"
        try:
            step = self.current_pid_plan_step()
        except Exception:
            return "PID tuning plan"
        if step is None:
            return "Choose the next winner or stage."
        return step[0]

    def action(self) -> str:
        app = self.app
        if app.pid_plan is None:
            return "Start Auto Session to begin the guided PID plan."
        if app.pid_plan_phase == "complete":
            return "Review final values and save in INAV only when you are satisfied."
        if app.pid_plan_fly_log_active:
            return "Fly/Log movement is active. Keep the drone controlled, then land and disarm."
        if app.pid_plan_waiting_for_fly_log:
            return "Arm, press Fly/Log, land, disarm, review the log, then press Next PID Plan Step."
        if app.pid_plan_phase == "final_write":
            return "Choose final values, then use the FC / INAV Save button when ready."
        return "Press Next PID Plan Step and follow the prompt. Values are staged first; Save is the only FC write."

    def selection_text(self) -> str:
        app = self.app
        selected_p = (
            "--"
            if app.pid_plan_selected_p is None
            else f"Roll {app.pid_plan_selected_p['roll']} / Pitch {app.pid_plan_selected_p['pitch']}"
        )
        selected_i = (
            "--"
            if app.pid_plan_selected_i is None
            else f"Roll {app.pid_plan_selected_i['roll']} / Pitch {app.pid_plan_selected_i['pitch']}"
        )
        selected_ff = (
            "--"
            if app.pid_plan_selected_ff is None
            else f"Roll {app.pid_plan_selected_ff['roll']} / Pitch {app.pid_plan_selected_ff['pitch']}"
        )
        return (
            f"Chosen P: {selected_p}\n"
            f"Chosen D: {'--' if app.pid_plan_selected_d is None else app.pid_plan_selected_d}\n"
            f"Chosen I: {selected_i}\n"
            f"Chosen FF: {selected_ff}"
        )

    def set_target_text(self, text: str) -> None:
        app = self.app
        if app.pid_progress_target_text is None:
            return
        app.pid_progress_target_text.config(state="normal")
        app.pid_progress_target_text.delete("1.0", tk.END)
        app.pid_progress_target_text.insert("1.0", text)
        app.pid_progress_target_text.config(state="disabled")

    def update_window(self) -> None:
        app = self.app
        if app.pid_progress_window is None or not app.pid_progress_window.winfo_exists():
            return

        active_phase = self.active_phase()
        active_index = self.PHASE_INDEX.get(active_phase)
        for phase, label_text in self.PHASES:
            label = app.pid_progress_phase_labels.get(phase)
            if label is None:
                continue
            phase_index = self.PHASE_INDEX[phase]
            if app.pid_plan_phase == "complete":
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

        app.pid_progress_current_var.set(self.title())
        app.pid_progress_action_var.set(self.action())
        app.pid_progress_selection_var.set(self.selection_text())
        app.pid_progress_plan_var.set("" if app.pid_plan is None else f"Plan file: {app.pid_plan.text_path}")

        target = self.target()
        target_text = "No target values are staged yet."
        if target:
            target_text = self.format_pid_values(target)
        self.set_target_text(target_text)

    def close_window(self) -> None:
        app = self.app
        if app.pid_progress_window is not None:
            try:
                app.pid_progress_window.destroy()
            except Exception:
                pass
        app.pid_progress_window = None
        app.pid_progress_target_text = None
        app.pid_progress_phase_labels.clear()

    def open_window(self) -> None:
        app = self.app
        if app.pid_progress_window is not None and app.pid_progress_window.winfo_exists():
            app.pid_progress_window.lift()
            self.update_window()
            return

        window = tk.Toplevel(app.root)
        window.withdraw()
        try:
            window.title("PID Tuning Progress")
            window.resizable(False, False)
            window.grid_rowconfigure(0, weight=1)
            window.grid_columnconfigure(0, weight=1)
            window.protocol("WM_DELETE_WINDOW", self.close_window)

            outer = tk.Frame(window, padx=10, pady=10)
            outer.grid(row=0, column=0, sticky="nsew")
            outer.grid_columnconfigure(0, weight=1)

            phase_frame = tk.LabelFrame(outer, text="Flow", padx=6, pady=6)
            phase_frame.grid(row=0, column=0, sticky="we")
            for column, (phase, label_text) in enumerate(self.PHASES):
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
                app.pid_progress_phase_labels[phase] = label

            current_frame = tk.LabelFrame(outer, text="Current Step", padx=8, pady=8)
            current_frame.grid(row=1, column=0, sticky="we", pady=(8, 0))
            current_frame.grid_columnconfigure(0, weight=1)
            tk.Label(
                current_frame,
                textvariable=app.pid_progress_current_var,
                anchor="w",
                justify="left",
                font=("Segoe UI", 10, "bold"),
                width=82,
                wraplength=680,
            ).grid(row=0, column=0, sticky="w")
            tk.Label(
                current_frame,
                textvariable=app.pid_progress_action_var,
                anchor="w",
                justify="left",
                width=82,
                wraplength=680,
            ).grid(row=1, column=0, sticky="w", pady=(6, 0))
            tk.Label(
                current_frame,
                textvariable=app.pid_progress_plan_var,
                anchor="w",
                justify="left",
                width=82,
                wraplength=680,
                fg="#374151",
            ).grid(row=2, column=0, sticky="w", pady=(6, 0))

            target_frame = tk.LabelFrame(outer, text="Target Values", padx=8, pady=8)
            target_frame.grid(row=2, column=0, sticky="we", pady=(8, 0))
            target_frame.grid_columnconfigure(0, weight=1)
            app.pid_progress_target_text = tk.Text(target_frame, width=82, height=5, wrap="none")
            app.pid_progress_target_text.grid(row=0, column=0, sticky="we")
            app.pid_progress_target_text.config(state="disabled")

            selection_frame = tk.LabelFrame(outer, text="Selected Winners", padx=8, pady=8)
            selection_frame.grid(row=3, column=0, sticky="we", pady=(8, 0))
            tk.Label(
                selection_frame,
                textvariable=app.pid_progress_selection_var,
                anchor="w",
                justify="left",
                width=82,
            ).grid(row=0, column=0, sticky="w")

            buttons = tk.Frame(outer)
            buttons.grid(row=4, column=0, sticky="e", pady=(8, 0))
            tk.Button(buttons, text="Refresh", width=10, command=self.update_window).pack(side="right")
            tk.Button(buttons, text="Close", width=10, command=self.close_window).pack(
                side="right", padx=(0, 6)
            )

            app.pid_progress_window = window
            self.update_window()
            window.deiconify()
            window.lift()
            window.update_idletasks()
        except Exception as exc:
            app.pid_progress_window = None
            app.pid_progress_target_text = None
            app.pid_progress_phase_labels.clear()
            try:
                window.destroy()
            except Exception:
                pass
            self.set_error("PID progress error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

"""Guided PID tuning plan workflow.

This module owns the guided PID plan progression: generating/loading a plan,
staging candidate values, asking the user to choose winners, and advancing from
safe start through P/D/I/FF sweeps to the final values.

The live Fly/Log hardware routine still lives in app.py for now. This workflow
only prepares/stages candidates and tells the existing app when Fly/Log is
needed.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any
import tkinter as tk
from tkinter import messagebox, simpledialog

from serialUSB.inav_serial_service import FF_SETTING_NAME, PID_SETTING_NAME

from ..dialogs.pid_tuning_dialog import ask_pid_tuning_inputs
from ..pid_tuning_workflow import (
    LoadedPIDTuningPlan,
    find_latest_pid_tuning_plan,
    generate_pid_tuning_plan_report,
    load_pid_tuning_plan,
    suggest_starting_p,
)

PidValues = dict[str, dict[str, int]]


class PidPlanWorkflow:
    """Controls the guided PID tuning plan state machine."""

    def __init__(
        self,
        app: Any,
        auto_is_running: Callable[[], bool],
        publish_auto_report: Callable[[str], None],
        set_error: Callable[[str, Exception], None],
        refresh_fly_log_button_state: Callable[[], None],
        update_progress_window: Callable[[], None],
        open_progress_window: Callable[[], None],
        ensure_disarmed_before_pid_write: Callable[[], bool],
        stage_pid_ff_var: Callable[[str, str, int], None],
        stop_simulated_auto_session: Callable[..., None],
        set_test_throttle_us: Callable[[int | None, str], int],
    ) -> None:
        self.app = app
        self.auto_is_running = auto_is_running
        self.publish_auto_report = publish_auto_report
        self.set_error = set_error
        self.refresh_fly_log_button_state = refresh_fly_log_button_state
        self.update_progress_window = update_progress_window
        self.open_progress_window = open_progress_window
        self.ensure_disarmed_before_pid_write = ensure_disarmed_before_pid_write
        self.stage_pid_ff_var = stage_pid_ff_var
        self.stop_simulated_auto_session = stop_simulated_auto_session
        self.set_test_throttle_us = set_test_throttle_us

    def generate_plan(self) -> None:
        app = self.app
        try:
            if app.blackbox_import_inflight:
                app.status.set("Blackbox/report task already in progress.")
                return
            if self.auto_is_running():
                app.status.set("Wait for the auto session/pipeline to finish first.")
                return

            inputs = ask_pid_tuning_inputs(app.root)
            if inputs is None:
                app.status.set("PID tuning plan canceled.")
                return

            recommendation = suggest_starting_p(inputs)
            report = generate_pid_tuning_plan_report(app.blackbox_import_dir, recommendation)
            self.set_test_throttle_us(recommendation.throttle_estimate.level_test_throttle_us, "generated PID plan")
            self.publish_auto_report(Path(report.text_path).read_text(encoding="utf-8", errors="replace"))
            app.status.set(
                f"PID tuning plan generated: {report.report_dir}. "
                f"Shared test throttle set to {recommendation.throttle_estimate.level_test_throttle_us}us."
            )
        except Exception as exc:
            self.set_error("PID tuning plan error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def locate_plan_file(self) -> Path:
        latest = find_latest_pid_tuning_plan(self.app.blackbox_import_dir)
        if latest is not None:
            return latest
        raise RuntimeError("No PID tuning plan file was found. Generate a PID Tuning Plan first.")

    def read_fc_pid_ff_values(self, axes: tuple[str, ...] = ("roll", "pitch", "yaw")) -> PidValues:
        app = self.app
        values: PidValues = {}
        for axis in axes:
            values[axis] = {
                "p": int(app.fc_service.get_setting_int(PID_SETTING_NAME[(axis, "p")], timeout_seconds=1.0)),
                "i": int(app.fc_service.get_setting_int(PID_SETTING_NAME[(axis, "i")], timeout_seconds=1.0)),
                "d": int(app.fc_service.get_setting_int(PID_SETTING_NAME[(axis, "d")], timeout_seconds=1.0)),
                "ff": int(app.fc_service.get_setting_int(FF_SETTING_NAME[axis], timeout_seconds=1.0)),
            }
        return values

    @staticmethod
    def format_pid_values(values: PidValues) -> str:
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

    @staticmethod
    def format_pid_target_check(current: PidValues, target: PidValues) -> str:
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

    def set_plan_report_text(
        self,
        plan: LoadedPIDTuningPlan,
        title: str,
        target: PidValues | None = None,
        current: PidValues | None = None,
    ) -> None:
        lines = [title, f"Plan file: {plan.text_path}"]
        if current:
            lines.extend(["", "Current FC PID/FF", self.format_pid_values(current)])
        if target:
            lines.extend(["", "Target for this step", self.format_pid_values(target)])
        lines.extend(["", plan.text])
        self.publish_auto_report("\n".join(lines))

    def stage_pid_ff_values(self, target: PidValues) -> None:
        for axis in ("roll", "pitch"):
            gains = target.get(axis)
            if not gains:
                continue
            for gain in ("p", "i", "d", "ff"):
                value = int(gains[gain])
                if value < 0 or value > 255:
                    raise RuntimeError(f"{axis.title()} {gain.upper()} target {value} is outside 0-255.")
                self.stage_pid_ff_var(axis, gain, value)

    @staticmethod
    def roll_pitch_target(
        roll_p: int,
        pitch_p: int,
        roll_d: int,
        pitch_d: int,
        roll_i: int,
        pitch_i: int,
        roll_ff: int,
        pitch_ff: int,
    ) -> PidValues:
        return {
            "roll": {"p": int(roll_p), "i": int(roll_i), "d": int(roll_d), "ff": int(roll_ff)},
            "pitch": {"p": int(pitch_p), "i": int(pitch_i), "d": int(pitch_d), "ff": int(pitch_ff)},
        }

    def ask_pid_value(self, title: str, prompt: str, initial: int) -> int | None:
        return simpledialog.askinteger(
            title,
            prompt,
            initialvalue=int(initial),
            minvalue=0,
            maxvalue=255,
            parent=self.app.root,
        )

    def ask_pid_pair(self, title: str, gain: str, initial_roll: int, initial_pitch: int) -> dict[str, int] | None:
        roll_value = self.ask_pid_value(title, f"Enter chosen Roll {gain.upper()} value.", initial_roll)
        if roll_value is None:
            return None
        pitch_value = self.ask_pid_value(title, f"Enter chosen Pitch {gain.upper()} value.", initial_pitch)
        if pitch_value is None:
            return None
        return {"roll": int(roll_value), "pitch": int(pitch_value)}

    def ask_candidate_action(self, title: str, prompt: str) -> str:
        """Ask how to handle a PID-plan candidate.

        Returns one of:
        - "stage": stage target values into the GUI boxes
        - "already_saved": enable Fly/Log without staging/saving values
        - "skip": skip this candidate and move on
        - "cancel": stop the guided PID plan
        """
        app = self.app
        dialog = tk.Toplevel(app.root)
        dialog.title("PID Plan Step")
        dialog.transient(app.root)
        dialog.resizable(False, False)

        result = {"action": "cancel"}

        outer = tk.Frame(dialog, padx=14, pady=12)
        outer.pack(fill="both", expand=True)

        tk.Label(outer, text=title, font=("Segoe UI", 10, "bold"), anchor="w", justify="left").pack(fill="x")
        tk.Label(outer, text=prompt, anchor="w", justify="left", wraplength=620).pack(fill="x", pady=(8, 12))

        buttons = tk.Frame(outer)
        buttons.pack(fill="x")

        def choose(action: str) -> None:
            result["action"] = action
            dialog.destroy()

        tk.Button(buttons, text="Stage Values", width=18, command=lambda: choose("stage")).pack(side="left", padx=(0, 6))
        tk.Button(buttons, text="Already Saved - Enable Fly/Log", width=28, command=lambda: choose("already_saved")).pack(side="left", padx=(0, 6))
        tk.Button(buttons, text="Skip Candidate", width=18, command=lambda: choose("skip")).pack(side="left", padx=(0, 6))
        tk.Button(buttons, text="Stop Plan", width=14, command=lambda: choose("cancel")).pack(side="left")

        dialog.protocol("WM_DELETE_WINDOW", lambda: choose("cancel"))
        dialog.bind("<Escape>", lambda _event: choose("cancel"))

        dialog.update_idletasks()
        try:
            x = app.root.winfo_rootx() + max(0, (app.root.winfo_width() - dialog.winfo_width()) // 2)
            y = app.root.winfo_rooty() + max(0, (app.root.winfo_height() - dialog.winfo_height()) // 2)
            dialog.geometry(f"+{x}+{y}")
        except Exception:
            pass

        dialog.grab_set()
        dialog.focus_set()
        app.root.wait_window(dialog)
        return str(result["action"])

    def d_candidates(self) -> tuple[int, ...]:
        app = self.app
        if app.pid_plan is None:
            return ()
        if len(app.pid_plan.d_sweep) <= 1:
            return ()
        return tuple(int(value) for value in app.pid_plan.d_sweep[1:])

    def p_candidates(self) -> tuple[dict[str, int], ...]:
        app = self.app
        if app.pid_plan is None:
            return ()
        rows: list[dict[str, int]] = []
        start_roll = int(app.pid_plan.start_p["roll"])
        start_pitch = int(app.pid_plan.start_p["pitch"])
        for roll, pitch in zip(app.pid_plan.p_sweep.get("roll", ()), app.pid_plan.p_sweep.get("pitch", ())):
            row = {"roll": int(roll), "pitch": int(pitch)}
            # Safe Start already logs the generated starting P with the first D value.
            # Do not ask the user to run that exact same P candidate twice.
            if row["roll"] == start_roll and row["pitch"] == start_pitch:
                continue
            rows.append(row)
        return tuple(rows)

    def d_recheck_candidates(self) -> tuple[int, ...]:
        app = self.app
        if app.pid_plan_selected_d is None:
            return ()
        delta = 5
        values = (app.pid_plan_selected_d - delta, app.pid_plan_selected_d, app.pid_plan_selected_d + delta)
        return tuple(dict.fromkeys(max(0, min(255, int(value))) for value in values))

    def complete(self, message: str) -> None:
        app = self.app
        app.pid_plan_active = False
        app.pid_plan_phase = "complete"
        app.pid_plan_index = 0
        app.pid_plan_waiting_for_fly_log = False
        app.pid_plan_current_candidate_title = ""
        app.pid_plan_current_candidate_phase = ""
        app.pid_plan_current_candidate_target = None
        self.refresh_fly_log_button_state()
        app.status.set(message)
        self.update_progress_window()

    def prepare_next_step(self) -> bool:
        app = self.app
        if app.pid_plan is None:
            raise RuntimeError("PID tuning plan is not loaded.")

        while True:
            if app.pid_plan_phase == "safe_start":
                return True

            if app.pid_plan_phase == "p_sweep" and app.pid_plan_index >= len(self.p_candidates()):
                candidates = self.p_candidates()
                initial = candidates[-1] if candidates else app.pid_plan.start_p
                selected = self.ask_pid_pair("Choose P", "P", initial["roll"], initial["pitch"])
                if selected is None:
                    app.status.set("PID plan paused; P winners are required before D sweep.")
                    return False
                app.pid_plan_selected_p = selected
                app.pid_plan_phase = "d_sweep"
                app.pid_plan_index = 0
                self.update_progress_window()
                continue

            if app.pid_plan_phase == "d_sweep" and app.pid_plan_index >= len(self.d_candidates()):
                optional_d = app.pid_plan.optional_d
                if optional_d is not None and optional_d not in app.pid_plan.d_sweep:
                    if messagebox.askyesno(
                        "Optional D Step",
                        f"The normal D sweep is complete.\n\nRun optional D {optional_d} before choosing D?",
                        parent=app.root,
                    ):
                        app.pid_plan_phase = "d_optional"
                        app.pid_plan_index = 0
                        return True
                chosen = self.ask_pid_value("Choose D", "Enter the best Roll/Pitch D from the D sweep.", app.pid_plan.d_sweep[0])
                if chosen is None:
                    app.status.set("PID plan paused; D winner is required before I sweep.")
                    return False
                app.pid_plan_selected_d = int(chosen)
                app.pid_plan_phase = "i_sweep"
                app.pid_plan_index = 0
                self.update_progress_window()
                continue

            if app.pid_plan_phase == "d_optional" and app.pid_plan_index >= 1:
                initial = app.pid_plan.optional_d if app.pid_plan.optional_d is not None else app.pid_plan.d_sweep[0]
                chosen = self.ask_pid_value("Choose D", "Enter the best Roll/Pitch D from the D sweep.", int(initial))
                if chosen is None:
                    app.status.set("PID plan paused; D winner is required before I sweep.")
                    return False
                app.pid_plan_selected_d = int(chosen)
                app.pid_plan_phase = "i_sweep"
                app.pid_plan_index = 0
                self.update_progress_window()
                continue

            if app.pid_plan_phase == "d_recheck" and app.pid_plan_index >= len(self.d_recheck_candidates()):
                initial = app.pid_plan_selected_d if app.pid_plan_selected_d is not None else app.pid_plan.d_sweep[0]
                chosen = self.ask_pid_value("Choose Final D", "Enter the best Roll/Pitch D after re-check.", int(initial))
                if chosen is None:
                    app.status.set("PID plan paused; final D is required before I sweep.")
                    return False
                app.pid_plan_selected_d = int(chosen)
                app.pid_plan_phase = "i_sweep"
                app.pid_plan_index = 0
                self.update_progress_window()
                continue

            if app.pid_plan_phase == "i_sweep" and app.pid_plan_index >= len(app.pid_plan.i_sweep):
                initial = app.pid_plan.i_sweep[-1] if app.pid_plan.i_sweep else {"roll": 60, "pitch": 65}
                selected = self.ask_pid_pair("Choose I", "I", initial["roll"], initial["pitch"])
                if selected is None:
                    app.status.set("PID plan paused; I winners are required before FF sweep.")
                    return False
                app.pid_plan_selected_i = selected
                app.pid_plan_phase = "ff_sweep"
                app.pid_plan_index = 0
                self.update_progress_window()
                continue

            if app.pid_plan_phase == "ff_sweep" and app.pid_plan_index >= len(app.pid_plan.ff_sweep):
                initial = app.pid_plan.ff_sweep[-1] if app.pid_plan.ff_sweep else {"roll": 86, "pitch": 89}
                selected = self.ask_pid_pair("Choose FF", "FF", initial["roll"], initial["pitch"])
                if selected is None:
                    app.status.set("PID plan paused; FF winners are required before final write.")
                    return False
                app.pid_plan_selected_ff = selected
                app.pid_plan_phase = "final_write"
                app.pid_plan_index = 0
                self.update_progress_window()
                continue

            return True

    def current_step(self) -> tuple[str, str, PidValues] | None:
        app = self.app
        if app.pid_plan is None:
            raise RuntimeError("PID tuning plan is not loaded.")
        start_d = int(app.pid_plan.d_sweep[0]) if app.pid_plan.d_sweep else 17

        if app.pid_plan_phase == "safe_start":
            target = self.roll_pitch_target(
                app.pid_plan.start_p["roll"],
                app.pid_plan.start_p["pitch"],
                start_d,
                start_d,
                app.pid_plan.start_i["roll"],
                app.pid_plan.start_i["pitch"],
                0,
                0,
            )
            return (
                "Safe start / first P log",
                f"This writes the generated starting P values with Roll I {app.pid_plan.start_i['roll']}, Pitch I {app.pid_plan.start_i['pitch']}, FF = 0, and starting D {start_d}.",
                target,
            )

        if app.pid_plan_phase == "d_sweep":
            if app.pid_plan_selected_p is None:
                return None
            candidates = self.d_candidates()
            if app.pid_plan_index >= len(candidates):
                return None
            d_value = candidates[app.pid_plan_index]
            target = self.roll_pitch_target(
                app.pid_plan_selected_p["roll"],
                app.pid_plan_selected_p["pitch"],
                d_value,
                d_value,
                app.pid_plan.start_i["roll"],
                app.pid_plan.start_i["pitch"],
                0,
                0,
            )
            return (
                f"D sweep {app.pid_plan_index + 2}/{len(app.pid_plan.d_sweep)}",
                f"Log Roll/Pitch D {d_value} with chosen P.",
                target,
            )

        if app.pid_plan_phase == "d_optional":
            if app.pid_plan_selected_p is None or app.pid_plan.optional_d is None or app.pid_plan_index >= 1:
                return None
            d_value = int(app.pid_plan.optional_d)
            target = self.roll_pitch_target(
                app.pid_plan_selected_p["roll"],
                app.pid_plan_selected_p["pitch"],
                d_value,
                d_value,
                app.pid_plan.start_i["roll"],
                app.pid_plan.start_i["pitch"],
                0,
                0,
            )
            return ("Optional D sweep", f"Log optional Roll/Pitch D {d_value} with chosen P.", target)

        if app.pid_plan_phase == "p_sweep":
            candidates = self.p_candidates()
            if app.pid_plan_index >= len(candidates):
                return None
            row = candidates[app.pid_plan_index]
            target = self.roll_pitch_target(
                row["roll"],
                row["pitch"],
                start_d,
                start_d,
                app.pid_plan.start_i["roll"],
                app.pid_plan.start_i["pitch"],
                0,
                0,
            )
            return (
                f"P sweep {app.pid_plan_index + 1}/{len(candidates)}",
                f"Log Roll P {row['roll']} and Pitch P {row['pitch']} with starting D {start_d}.",
                target,
            )

        if app.pid_plan_phase == "d_recheck":
            if app.pid_plan_selected_p is None:
                return None
            candidates = self.d_recheck_candidates()
            if app.pid_plan_index >= len(candidates):
                return None
            d_value = candidates[app.pid_plan_index]
            target = self.roll_pitch_target(
                app.pid_plan_selected_p["roll"],
                app.pid_plan_selected_p["pitch"],
                d_value,
                d_value,
                app.pid_plan.start_i["roll"],
                app.pid_plan.start_i["pitch"],
                0,
                0,
            )
            return (
                f"D re-check {app.pid_plan_index + 1}/{len(candidates)}",
                f"Log Roll/Pitch D {d_value} with chosen P.",
                target,
            )

        if app.pid_plan_phase == "i_sweep":
            if app.pid_plan_selected_p is None or app.pid_plan_selected_d is None:
                return None
            if app.pid_plan_index >= len(app.pid_plan.i_sweep):
                return None
            row = app.pid_plan.i_sweep[app.pid_plan_index]
            target = self.roll_pitch_target(
                app.pid_plan_selected_p["roll"],
                app.pid_plan_selected_p["pitch"],
                app.pid_plan_selected_d,
                app.pid_plan_selected_d,
                row["roll"],
                row["pitch"],
                0,
                0,
            )
            return (
                f"I sweep {app.pid_plan_index + 1}/{len(app.pid_plan.i_sweep)}",
                f"Log Roll I {row['roll']} and Pitch I {row['pitch']}.",
                target,
            )

        if app.pid_plan_phase == "ff_sweep":
            if app.pid_plan_selected_p is None or app.pid_plan_selected_d is None or app.pid_plan_selected_i is None:
                return None
            if app.pid_plan_index >= len(app.pid_plan.ff_sweep):
                return None
            row = app.pid_plan.ff_sweep[app.pid_plan_index]
            target = self.roll_pitch_target(
                app.pid_plan_selected_p["roll"],
                app.pid_plan_selected_p["pitch"],
                app.pid_plan_selected_d,
                app.pid_plan_selected_d,
                app.pid_plan_selected_i["roll"],
                app.pid_plan_selected_i["pitch"],
                row["roll"],
                row["pitch"],
            )
            return (
                f"FF sweep {app.pid_plan_index + 1}/{len(app.pid_plan.ff_sweep)}",
                f"Log Roll FF {row['roll']} and Pitch FF {row['pitch']}.",
                target,
            )

        return None

    def advance_after_step(self) -> None:
        app = self.app
        if app.pid_plan_phase == "safe_start":
            app.pid_plan_phase = "p_sweep"
            app.pid_plan_index = 0
            return
        app.pid_plan_index += 1

    def run_final_write(self) -> None:
        app = self.app
        if (
            app.pid_plan is None
            or app.pid_plan_selected_p is None
            or app.pid_plan_selected_d is None
            or app.pid_plan_selected_i is None
            or app.pid_plan_selected_ff is None
        ):
            raise RuntimeError("PID plan final values are incomplete.")

        roll_pitch = self.roll_pitch_target(
            app.pid_plan_selected_p["roll"],
            app.pid_plan_selected_p["pitch"],
            app.pid_plan_selected_d,
            app.pid_plan_selected_d,
            app.pid_plan_selected_i["roll"],
            app.pid_plan_selected_i["pitch"],
            app.pid_plan_selected_ff["roll"],
            app.pid_plan_selected_ff["pitch"],
        )
        yaw = app.pid_plan.yaw_final_pid_ff
        yaw_recommendation = (
            f"Yaw (not swept; set manually in INAV): "
            f"P {yaw['p']}, I {yaw['i']}, D {yaw['d']}, FF {yaw['ff']}\n"
            "Treat yaw as a conservative baseline; revisit only if logs or flight feel show yaw-specific problems."
        )
        current = self.read_fc_pid_ff_values()
        self.set_plan_report_text(app.pid_plan, "PID plan final values", roll_pitch, current)
        prompt = (
            "The roll/pitch sweeps are complete.\n\n"
            "Yes: stage the chosen roll/pitch values in the FC / INAV PID boxes.\n"
            "No: leave the current boxes unchanged and mark the plan complete.\n"
            "Cancel: stop without staging final values.\n\n"
            "Use the FC / INAV Save button to write and save staged roll/pitch values to the flight controller.\n\n"
            f"{yaw_recommendation}\n\n"
            "Current vs target:\n"
            f"{self.format_pid_target_check(current, roll_pitch)}"
        )
        choice = messagebox.askyesnocancel("PID Plan Final Values", prompt, parent=app.root)
        if choice is None:
            self.complete("PID tuning plan stopped before final values were staged.")
            return
        if choice:
            self.stage_pid_ff_values(roll_pitch)
        self.complete("PID tuning plan complete.")
        if choice:
            completion_text = (
                "Final selected roll/pitch values are staged in the PID boxes. "
                "Press Save in the FC / INAV section to write and save them.\n\n"
                f"Conservative yaw recommendation (set manually in INAV):\n"
                f"P {yaw['p']}, I {yaw['i']}, D {yaw['d']}, FF {yaw['ff']}"
            )
        else:
            completion_text = (
                "Plan marked complete without staging roll/pitch.\n\n"
                f"Conservative yaw recommendation (set manually in INAV):\n"
                f"P {yaw['p']}, I {yaw['i']}, D {yaw['d']}, FF {yaw['ff']}"
            )
        messagebox.showinfo("PID Plan Complete", completion_text, parent=app.root)

    def continue_plan(self) -> None:
        app = self.app
        if not app.pid_plan_active:
            return
        if app.pid_plan is None:
            raise RuntimeError("PID tuning plan is not loaded.")
        if app.pid_plan_waiting_for_fly_log:
            self.open_progress_window()
            messagebox.showinfo(
                "Fly/Log Needed",
                f"{app.pid_plan_current_candidate_title or 'The current candidate'} is ready.\n\n"
                "Arm the drone, press Fly/Log, then disarm the drone before pressing Next PID Plan Step.",
                parent=app.root,
            )
            app.status.set("Press Fly/Log for the current candidate before moving to the next step.")
            self.update_progress_window()
            return
        if not self.prepare_next_step():
            self.update_progress_window()
            return
        if app.pid_plan_phase == "final_write":
            self.update_progress_window()
            self.run_final_write()
            return

        step = self.current_step()
        if step is None:
            app.status.set("PID plan is waiting for the next stage choice.")
            self.update_progress_window()
            return
        title, instruction, target = step
        step_phase = app.pid_plan_phase
        self.update_progress_window()
        if app.pid_plan_phase == "safe_start":
            self.set_plan_report_text(app.pid_plan, f"PID plan step: {title}", target)
            current = self.read_fc_pid_ff_values(tuple(target.keys()))
            self.set_plan_report_text(app.pid_plan, f"PID plan step: {title}", target, current)
            if not self.ensure_disarmed_before_pid_write():
                app.status.set("Safe-start canceled; disarm before staging PID/FF values.")
                self.update_progress_window()
                return
            self.stage_pid_ff_values(target)
            self.advance_after_step()
            app.pid_plan_waiting_for_fly_log = True
            app.pid_plan_current_candidate_title = title
            app.pid_plan_current_candidate_phase = step_phase
            app.pid_plan_current_candidate_target = target
            self.refresh_fly_log_button_state()
            app.status.set("Safe-start PID/FF values staged for the first P log. Press Save before Fly/Log.")
            self.update_progress_window()
            messagebox.showinfo(
                "Safe Start Ready",
                "Safe-start PID/FF values are staged in the FC / INAV boxes.\n\n"
                "Press Save while disarmed, then arm the drone, press Fly/Log, and disarm before pressing Next PID Plan Step.",
                parent=app.root,
            )
            return

        current = self.read_fc_pid_ff_values(tuple(target.keys()))
        self.set_plan_report_text(app.pid_plan, f"PID plan step: {title}", target, current)
        prompt = (
            f"{instruction}\n\n"
            "Required sequence for this candidate:\n"
            "1. DISARM the drone.\n"
            "2. Either stage/save the target values, or use Already Saved if the FC already has them.\n"
            "3. Arm, then press Fly/Log for this candidate.\n"
            "4. Land and DISARM before pressing Next PID Plan Step.\n\n"
            "Button choices:\n"
            "Stage Values: copy target values into the FC / INAV boxes.\n"
            "Already Saved - Enable Fly/Log: do not rewrite values; enable Fly/Log for this candidate.\n"
            "Skip Candidate: skip this candidate and move on.\n"
            "Stop Plan: stop the guided PID plan.\n\n"
            "Current vs target:\n"
            f"{self.format_pid_target_check(current, target)}"
        )
        action = self.ask_candidate_action(title, prompt)
        if action == "cancel":
            self.complete("PID tuning plan stopped by user.")
            return
        if action == "stage":
            self.stage_pid_ff_values(target)
            app.pid_plan_waiting_for_fly_log = True
            app.pid_plan_current_candidate_title = title
            app.pid_plan_current_candidate_phase = step_phase
            app.pid_plan_current_candidate_target = target
            app.status.set(f"PID plan step staged: {title}. Press Save before Fly/Log.")
        elif action == "already_saved":
            app.pid_plan_waiting_for_fly_log = True
            app.pid_plan_current_candidate_title = title
            app.pid_plan_current_candidate_phase = step_phase
            app.pid_plan_current_candidate_target = target
            app.status.set(f"PID plan step enabled without staging: {title}. Press Fly/Log when armed.")
        else:
            app.pid_plan_waiting_for_fly_log = False
            app.pid_plan_current_candidate_title = ""
            app.pid_plan_current_candidate_phase = ""
            app.pid_plan_current_candidate_target = None
            app.status.set(f"PID plan step skipped: {title}")
        self.advance_after_step()
        self.refresh_fly_log_button_state()
        self.update_progress_window()
        messagebox.showinfo(
            "PID Plan Step Ready",
            (
                "Values are staged for this candidate.\n\n"
                "Press Save while disarmed, then arm the drone, press Fly/Log, and disarm before pressing Next PID Plan Step."
                if action == "stage"
                else (
                    "Fly/Log is enabled without staging values.\n\n"
                    "Use this only when the FC already has the correct PID/FF values saved. Arm, press Fly/Log, then disarm before pressing Next PID Plan Step."
                    if action == "already_saved"
                    else "This candidate was skipped. Press Next PID Plan Step when ready for the next candidate."
                )
            ),
            parent=app.root,
        )

    def start_session(self) -> None:
        app = self.app
        if app.sim_active or app.sim_fly_log_active or app.sim_waiting_for_fly_log or app.sim_plan is not None:
            self.stop_simulated_auto_session("", restore_display=True, clear_walkthrough=True)
        plan_path = self.locate_plan_file()
        app.pid_plan = load_pid_tuning_plan(plan_path)
        self.set_test_throttle_us(app.pid_plan.level_test_throttle_us, "loaded PID plan")
        app.pid_plan_active = True
        app.pid_plan_phase = "safe_start"
        app.pid_plan_index = 0
        app.pid_plan_selected_d = None
        app.pid_plan_selected_p = None
        app.pid_plan_selected_i = None
        app.pid_plan_selected_ff = None
        app.pid_plan_waiting_for_fly_log = False
        app.pid_plan_current_candidate_title = ""
        app.pid_plan_current_candidate_phase = ""
        app.pid_plan_current_candidate_target = None
        self.set_plan_report_text(app.pid_plan, "PID tuning plan loaded")
        self.refresh_fly_log_button_state()
        app.status.set(
            f"PID tuning plan loaded: {app.pid_plan.text_path}. "
            f"Shared test throttle {getattr(app, 'test_throttle_us', '--')}us."
        )
        self.open_progress_window()
        self.continue_plan()

"""FC PID/FF display, staging, load, and save workflow.

This module keeps the PID/FF widget plumbing and FC read/write actions out of
app.py. It still delegates app-wide concerns, such as disarm confirmation and
error presentation, back to the main runtime through callbacks.
"""

from __future__ import annotations

from typing import Any
import tkinter as tk
from tkinter import messagebox

from serialUSB.inav_serial_service import AxisPidFf, FF_SETTING_NAME, PID_SETTING_NAME

from ..tasks.worker_tasks import read_fc_pid_ff as worker_read_fc_pid_ff

PidValues = dict[str, dict[str, int]]
PID_FF_GAIN_LIMITS = {
    "p": (20, 70),
    "i": (20, 150),
    "d": (10, 60),
    "ff": (0, 180),
}


class FcPidFfWorkflow:
    """Owns the Roll/Pitch PID/FF fields and FC load/save actions."""

    def __init__(
        self,
        app: Any,
        set_error,
        ensure_disarmed_before_pid_write,
        format_pid_values,
    ) -> None:
        self.app = app
        self.set_error = set_error
        self.ensure_disarmed_before_pid_write = ensure_disarmed_before_pid_write
        self.format_pid_values = format_pid_values

    @staticmethod
    def format_value(value: float) -> str:
        rounded = round(value)
        if abs(value - rounded) < 1e-6:
            return str(int(rounded))
        return f"{value:.2f}".rstrip("0").rstrip(".")

    @staticmethod
    def gain_limits(gain: str) -> tuple[int, int]:
        return PID_FF_GAIN_LIMITS[gain.lower()]

    @classmethod
    def clamp_gain_value(cls, gain: str, value: int | float) -> int:
        low, high = cls.gain_limits(gain)
        return max(low, min(high, int(round(float(value)))))

    def clear_displays(self) -> None:
        app = self.app
        for label, var in zip(app.pid_ff_labels, app.roll_pidff_vars):
            var.set(f"{label}: --")
        for label, var in zip(app.pid_ff_labels, app.pitch_pidff_vars):
            var.set(f"{label}: --")

    def set_displays(self, roll_values: AxisPidFf, pitch_values: AxisPidFf) -> None:
        app = self.app
        roll_series = {"p": roll_values.p, "i": roll_values.i, "d": roll_values.d, "ff": roll_values.ff}
        pitch_series = {"p": pitch_values.p, "i": pitch_values.i, "d": pitch_values.d, "ff": pitch_values.ff}
        for gain, value in roll_series.items():
            self.set_var("roll", gain, int(round(float(value))))
        for gain, value in pitch_series.items():
            self.set_var("pitch", gain, int(round(float(value))))

    def var(self, axis: str, gain: str) -> tk.StringVar:
        app = self.app
        index = {"p": 0, "i": 1, "d": 2, "ff": 3}[gain]
        return app.roll_pidff_vars[index] if axis == "roll" else app.pitch_pidff_vars[index]

    def parse_var(self, axis: str, gain: str) -> int:
        raw = self.var(axis, gain).get().strip()
        if ":" in raw:
            raw = raw.split(":", 1)[1].strip()
        if not raw or raw == "--":
            raise RuntimeError(f"{axis.title()} {gain.upper()} is blank. Press Load or enter a value before saving.")
        try:
            value = int(round(float(raw)))
        except ValueError as exc:
            raise RuntimeError(f"{axis.title()} {gain.upper()} must be a number.") from exc
        low, high = self.gain_limits(gain)
        if value < low or value > high:
            raise RuntimeError(f"{axis.title()} {gain.upper()} must be between {low} and {high}.")
        return value

    def set_var(self, axis: str, gain: str, value: int) -> None:
        label = gain.upper()
        self.var(axis, gain).set(f"{label}: {self.clamp_gain_value(gain, value)}")

    def staged_roll_pitch_values(self) -> PidValues:
        return {
            axis: {gain: self.parse_var(axis, gain) for gain in ("p", "i", "d", "ff")}
            for axis in ("roll", "pitch")
        }

    def refresh_from_fc(self, update_status: bool = False) -> bool:
        app = self.app
        if not app.fc_service.is_connected:
            self.clear_displays()
            if update_status:
                app.status.set("FC is disconnected.")
            return False
        try:
            roll_values, pitch_values = app.fc_service.read_roll_pitch_pid_ff(timeout_seconds=1.2)
            self.set_displays(roll_values, pitch_values)
            if update_status:
                app.status.set("PID/FF refreshed from FC.")
            return True
        except Exception as exc:
            self.clear_displays()
            if update_status:
                self.set_error("PID/FF read error", exc)
            return False

    def queue_refresh(self, connected_port: str, connected_baud: int) -> None:
        app = self.app
        if not app.fc_service.is_connected:
            return

        def on_pid_ff_read_done(ok: bool, res: object) -> None:
            if not app.fc_service.is_connected:
                return
            if not ok:
                self.clear_displays()
                app.status.set(f"FC connected: {connected_port} @ {connected_baud}. PID/FF read failed.")
                return
            if not self._is_pid_ff_pair(res):
                self.clear_displays()
                app.status.set(f"FC connected: {connected_port} @ {connected_baud}. PID/FF read failed.")
                return
            self.set_displays(res[0], res[1])
            app.status.set(f"FC connected: {connected_port} @ {connected_baud}. PID/FF loaded.")

        app.worker.submit(worker_read_fc_pid_ff, app.fc_service, callback=on_pid_ff_read_done)

    def load_from_fc(self) -> None:
        app = self.app
        try:
            if not app.fc_service.is_connected:
                app.status.set("Connect FC before loading PID/FF.")
                return
            app.load_pid_ff_button.config(state="disabled")
            app.status.set("Loading PID/FF from FC...")

            def on_pid_ff_load_done(ok: bool, res: object) -> None:
                app.load_pid_ff_button.config(state="normal")
                if not ok:
                    self.set_error("PID/FF load error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not self._is_pid_ff_pair(res):
                    self.set_error("PID/FF load error", RuntimeError("Unexpected PID/FF load result."))
                    return
                self.set_displays(res[0], res[1])
                app.status.set("PID/FF loaded from FC.")

            app.worker.submit(worker_read_fc_pid_ff, app.fc_service, callback=on_pid_ff_load_done)
        except Exception as exc:
            app.load_pid_ff_button.config(state="normal")
            self.set_error("PID/FF load error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def write_staged_values_to_fc(self, target: PidValues) -> None:
        app = self.app
        for axis in ("roll", "pitch"):
            gains = target.get(axis)
            if not gains:
                continue
            for gain in ("p", "i", "d", "ff"):
                value = int(gains[gain])
                setting_name = FF_SETTING_NAME[axis] if gain == "ff" else PID_SETTING_NAME[(axis, gain)]
                confirmed = int(app.fc_service.set_setting_int(setting_name, value, timeout_seconds=1.2))
                if confirmed != value:
                    raise RuntimeError(
                        f"{axis.title()} {gain.upper()} write verified as {confirmed}, expected {value}."
                    )
        app.fc_service.save_settings(timeout_seconds=1.5)

    def save_to_fc(self) -> None:
        app = self.app
        try:
            if not app.fc_service.is_connected:
                app.status.set("Connect FC before saving PID/FF.")
                return
            target = self.staged_roll_pitch_values()
            if not self.ensure_disarmed_before_pid_write():
                app.status.set("PID/FF save canceled; disarm before saving.")
                return
            summary = self.format_pid_values(target)
            prompt = (
                "Write these Roll/Pitch PID/FF values to the FC and save them?\n\n"
                f"{summary}\n\n"
                "The FC may reboot after saving."
            )
            if not messagebox.askyesno("Save PID/FF", prompt, icon="warning", parent=app.root):
                app.status.set("PID/FF save canceled.")
                return

            app.save_pid_ff_button.config(state="disabled")
            app.status.set("Saving PID/FF to FC...")

            def on_pid_ff_save_done(ok: bool, res: object) -> None:
                app.save_pid_ff_button.config(state="normal")
                if not ok:
                    self.set_error("PID/FF save error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                app.status.set("PID/FF written and saved to FC.")

            app.worker.submit(
                lambda _worker_self: self.write_staged_values_to_fc(target),
                callback=on_pid_ff_save_done,
            )
        except Exception as exc:
            app.save_pid_ff_button.config(state="normal")
            self.set_error("PID/FF save error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def adjust_value(self, index: int, delta: int) -> None:
        app = self.app
        if index < 0 or index >= len(app.pid_ff_adjust_fields):
            return
        if delta == 0:
            return
        axis, gain = app.pid_ff_adjust_fields[index]
        try:
            current = self.parse_var(axis, gain)
            target = self.clamp_gain_value(gain, current + delta)
            if target == current:
                return
            self.set_var(axis, gain, target)
            app.status.set(f"{axis.title()} {gain.upper()} staged at {target}. Press Save to write to FC.")
        except Exception as exc:
            self.set_error("PID/FF adjust error", exc)

    @staticmethod
    def _is_pid_ff_pair(value: object) -> bool:
        return (
            isinstance(value, tuple)
            and len(value) == 2
            and isinstance(value[0], AxisPidFf)
            and isinstance(value[1], AxisPidFf)
        )

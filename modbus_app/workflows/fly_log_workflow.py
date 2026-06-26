"""Fly/Log button workflow.

This module owns the small button-level decision tree for the Fly/Log button.
The lower-level hardware/session implementation still lives in app.py for now.
That keeps this first refactor safe and makes the button entry point easier to trace:

    Fly/Log button -> FlyLogWorkflow.toggle()

Later, start_pid_plan_fly_log() and finish_pid_plan_fly_log() can be moved into
this class after the shared auto-session helpers are extracted.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any


class FlyLogWorkflow:
    """Controls the Fly/Log button entry point and enabled/disabled state."""

    READY_LABEL = "Fly / Log"

    def __init__(
        self,
        app: Any,
        start_pid_plan_fly_log: Callable[[], None],
        set_error: Callable[[str, Exception], None],
    ) -> None:
        self.app = app
        self.start_pid_plan_fly_log = start_pid_plan_fly_log
        self.set_error = set_error

    def refresh_button_state(self) -> None:
        """Enable Fly/Log when a real candidate is waiting."""
        app = self.app

        if app.pid_plan_fly_log_active:
            app.fly_log_button.config(text="Fly / Log Active", state="normal")
        elif getattr(app, "fly_log_pid_isolation_restoring", False):
            app.fly_log_button.config(text="Restoring PID/FF", state="disabled")
        elif getattr(app, "fly_log_pid_isolation_snapshot", None) is not None:
            if getattr(app, "fly_log_pid_isolation_run_complete", False):
                app.fly_log_button.config(text="Disarm to Restore", state="disabled")
            else:
                app.fly_log_button.config(text="Start Fly / Log", state="normal")
        elif app.pid_plan_active and app.pid_plan_waiting_for_fly_log:
            app.fly_log_button.config(text="Prepare Fly / Log", state="normal")
        else:
            app.fly_log_button.config(text=self.READY_LABEL, state="normal")

    def toggle(self) -> None:
        """Main click handler for the Fly/Log button."""
        app = self.app

        try:
            if app.pid_plan_fly_log_active:
                app.status.set("Use Cancel Auto Session to stop Fly/Log.")
                return

            self.start_pid_plan_fly_log()
        except Exception as exc:
            self.set_error("Fly/Log error", exc)

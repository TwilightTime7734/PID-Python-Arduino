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

    def __init__(
        self,
        app: Any,
        simulation_mode_enabled: Callable[[], bool],
        start_pid_plan_fly_log: Callable[[], None],
        start_simulated_fly_log: Callable[[], None],
        set_error: Callable[[str, Exception], None],
    ) -> None:
        self.app = app
        self.simulation_mode_enabled = simulation_mode_enabled
        self.start_pid_plan_fly_log = start_pid_plan_fly_log
        self.start_simulated_fly_log = start_simulated_fly_log
        self.set_error = set_error

    def refresh_button_state(self) -> None:
        """Enable Fly/Log only when a real or simulated candidate is waiting."""
        app = self.app

        if app.pid_plan_fly_log_active:
            app.fly_log_button.config(text="Fly/Log Active", state="disabled")
        elif self.simulation_mode_enabled() and app.sim_fly_log_active:
            app.fly_log_button.config(text="Sim Fly/Log Active", state="disabled")
        elif app.pid_plan_active and app.pid_plan_waiting_for_fly_log:
            app.fly_log_button.config(text="Fly/Log", state="normal")
        elif self.simulation_mode_enabled() and app.sim_plan is not None and app.sim_waiting_for_fly_log:
            app.fly_log_button.config(text="Fly/Log", state="normal")
        else:
            app.fly_log_button.config(text="Fly/Log", state="disabled")

    def toggle(self) -> None:
        """Main click handler for the Fly/Log button."""
        app = self.app

        try:
            if app.sim_fly_log_active:
                app.status.set("Use Cancel Auto Session to stop simulated Fly/Log.")
                return

            if app.sim_plan is not None and app.sim_waiting_for_fly_log:
                self.start_simulated_fly_log()
                return

            if app.pid_plan_fly_log_active:
                app.status.set("Use Cancel Auto Session to stop Fly/Log.")
                return

            self.start_pid_plan_fly_log()
        except Exception as exc:
            self.set_error("Fly/Log error", exc)

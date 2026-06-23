"""Outer auto-session button/control workflow."""

from __future__ import annotations

from collections.abc import Callable

from ..adaptive_session import AdaptiveSessionState


class AutoSessionWorkflow:
    """Coordinates the outer Auto Session / Cancel button behavior.

    This intentionally does not own the live pulse/tick engine. The lower-level
    runtime functions stay in app.py for now and are provided as callbacks.
    """

    def __init__(
        self,
        *,
        app,
        start_auto_session: Callable[[], None],
        open_pid_progress_window: Callable[[], None],
        continue_pid_tuning_plan: Callable[[], None],
        complete_auto_session: Callable[..., None],
        refresh_fly_log_button_state: Callable[[], None],
        complete_pid_tuning_plan: Callable[[str], None],
        update_link_indicators: Callable[[], None],
        update_pid_progress_window: Callable[[], None],
        begin_auto_pipeline: Callable[[], None],
        set_error: Callable[[str, Exception], None],
    ) -> None:
        self.app = app
        self.start_auto_session = start_auto_session
        self.open_pid_progress_window = open_pid_progress_window
        self.continue_pid_tuning_plan = continue_pid_tuning_plan
        self.complete_auto_session = complete_auto_session
        self.refresh_fly_log_button_state = refresh_fly_log_button_state
        self.complete_pid_tuning_plan = complete_pid_tuning_plan
        self.update_link_indicators = update_link_indicators
        self.update_pid_progress_window = update_pid_progress_window
        self.begin_auto_pipeline = begin_auto_pipeline
        self.set_error = set_error

    def is_running(self) -> bool:
        return self.app.auto_state in {
            AdaptiveSessionState.adaptive_run,
            AdaptiveSessionState.recovery,
            AdaptiveSessionState.finalize,
            AdaptiveSessionState.import_analyze,
        }

    def abort(self, reason: str, warning: str = "", continue_pipeline: bool = False) -> None:
        self.complete_auto_session(AdaptiveSessionState.aborted, reason, warning, lower_throttle=True)
        self.app.pid_plan_fly_log_active = False
        self.refresh_fly_log_button_state()
        self.app.status.set(f"Auto session aborted: {reason}")
        self.update_pid_progress_window()
        if continue_pipeline:
            self.begin_auto_pipeline()

    def toggle(self) -> None:
        app = self.app
        try:
            if self.is_running():
                app.status.set("Use Cancel Auto Session to stop the active run.")
                return
            if app.auto_state == AdaptiveSessionState.import_analyze:
                app.status.set("Auto pipeline is running; wait for completion.")
                return
            if app.pid_plan_active:
                self.open_pid_progress_window()
                self.continue_pid_tuning_plan()
                return
            self.start_auto_session()
        except Exception as exc:
            self.set_error("Auto session error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def cancel_available(self) -> bool:
        app = self.app
        runtime_cancelable = self.is_running() and app.auto_state != AdaptiveSessionState.import_analyze
        return (
            runtime_cancelable
            or app.pid_plan_active
        )

    def cancel(self) -> None:
        app = self.app
        try:
            canceled = False
            if self.is_running() and app.auto_state != AdaptiveSessionState.import_analyze:
                self.complete_auto_session(AdaptiveSessionState.aborted, "Canceled by user.", lower_throttle=True)
                app.pid_plan_fly_log_active = False
                self.refresh_fly_log_button_state()
                app.status.set("Auto session canceled.")
                canceled = True
            if app.pid_plan_active:
                app.pid_plan_fly_log_active = False
                self.complete_pid_tuning_plan("PID tuning plan canceled by user.")
                canceled = True
            if not canceled:
                app.status.set("No auto session is active.")
            self.update_link_indicators()
            self.update_pid_progress_window()
        except Exception as exc:
            self.set_error("Cancel auto session error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

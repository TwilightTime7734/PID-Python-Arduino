"""Auto-session runtime completion and report pipeline workflow."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from ..adaptive_session import AdaptiveSessionState
from ..auto_tune_report import AutoTuneReport
from ..blackbox_import import BlackboxImportResult


class AutoSessionCompletionWorkflow:
    """Owns the auto-session shutdown/cleanup side and blackbox report pipeline.

    This intentionally does not own the live tick/pulse engine yet. The active
    control loop still stays in app.py and calls into this workflow when it needs
    to stop, clean up, or start the post-run report pipeline.
    """

    def __init__(
        self,
        *,
        app,
        simulation_mode_enabled: Callable[[], bool],
        fc_port: Callable[[], str],
        fc_baud: Callable[[], int],
        ensure_disarmed_before_blackbox_import: Callable[[str, int], bool],
        do_fc_disconnect: Callable[..., None],
        set_auto_state: Callable[..., None],
        publish_auto_report: Callable[[str], None],
        format_blackbox_report: Callable[[BlackboxImportResult], str],
        auto_session_payload: Callable[[], dict[str, object]],
        auto_abort: Callable[..., None],
        cancel_auto_hold_timer: Callable[[], None],
        cancel_fly_log_marker_timer: Callable[[], None],
        restore_auto_original_base_outputs: Callable[[], None],
        lower_throttle_for_abort: Callable[[], None],
        arduino_output_connected: Callable[[], bool],
        restore_base_outputs_after_hold: Callable[..., None],
        worker_enter_msc_and_import_blackbox_logs: Callable[..., object],
        worker_analyze_specific_blackbox_log: Callable[..., object],
        worker_analyze_blackbox_logs: Callable[..., object],
        worker_generate_auto_report: Callable[..., object],
    ) -> None:
        self.app = app
        self.simulation_mode_enabled = simulation_mode_enabled
        self.fc_port = fc_port
        self.fc_baud = fc_baud
        self.ensure_disarmed_before_blackbox_import = ensure_disarmed_before_blackbox_import
        self.do_fc_disconnect = do_fc_disconnect
        self.set_auto_state = set_auto_state
        self.publish_auto_report = publish_auto_report
        self.format_blackbox_report = format_blackbox_report
        self.auto_session_payload = auto_session_payload
        self.auto_abort = auto_abort
        self.cancel_auto_hold_timer = cancel_auto_hold_timer
        self.cancel_fly_log_marker_timer = cancel_fly_log_marker_timer
        self.restore_auto_original_base_outputs = restore_auto_original_base_outputs
        self.lower_throttle_for_abort = lower_throttle_for_abort
        self.arduino_output_connected = arduino_output_connected
        self.restore_base_outputs_after_hold = restore_base_outputs_after_hold
        self.worker_enter_msc_and_import_blackbox_logs = worker_enter_msc_and_import_blackbox_logs
        self.worker_analyze_specific_blackbox_log = worker_analyze_specific_blackbox_log
        self.worker_analyze_blackbox_logs = worker_analyze_blackbox_logs
        self.worker_generate_auto_report = worker_generate_auto_report

    def stop_runtime(self, restore_outputs: bool = True) -> None:
        app = self.app
        if app.auto_tick_after_id is not None:
            try:
                app.root.after_cancel(app.auto_tick_after_id)
            except Exception:
                pass
            app.auto_tick_after_id = None
        self.cancel_auto_hold_timer()
        self.cancel_fly_log_marker_timer()
        if restore_outputs:
            self.restore_auto_original_base_outputs()
        else:
            app.auto_original_base_outputs = None
        app.auto_pulse_inflight = False
        app.auto_hold_end_requested = False
        app.auto_settle_until_s = None
        app.auto_active_command = None
        app.auto_probe_axes_pending = []
        app.auto_event_peak_delta = 0.0
        app.auto_event_abs_peak_delta = 0.0
        app.auto_event_signed_peak_delta = 0.0
        app.auto_recovery_mode = False
        app.auto_stop_after_recovery = False
        app.fly_log_finishing = False

    def complete(
        self,
        next_state: AdaptiveSessionState,
        reason: str,
        warning: str = "",
        lower_throttle: bool = False,
    ) -> None:
        app = self.app
        app.auto_stop_reason = reason
        app.auto_warning = warning
        if lower_throttle:
            self.lower_throttle_for_abort()
        self.stop_runtime(restore_outputs=not lower_throttle)
        if self.arduino_output_connected():
            try:
                self.restore_base_outputs_after_hold()
            except Exception:
                pass
        self.set_auto_state(next_state, warning or reason)

    def begin_pipeline(self) -> None:
        app = self.app
        if self.simulation_mode_enabled():
            app.status.set("Auto blackbox pipeline skipped: simulation mode is enabled.")
            return
        if app.blackbox_import_inflight:
            app.status.set("Blackbox pipeline already in progress.")
            return
        try:
            selected_port = self.fc_port()
            selected_baud = self.fc_baud()
            if not self.ensure_disarmed_before_blackbox_import(selected_port, selected_baud):
                app.status.set("Auto blackbox pipeline canceled; disarm the drone before pulling logs.")
                return
            if app.fc_service.is_connected:
                self.do_fc_disconnect(update_status=False)
            app.blackbox_import_inflight = True
            self.set_auto_state(AdaptiveSessionState.import_analyze, "Import/analyze running")
            app.status.set("Auto session finished. Pulling and analyzing blackbox logs...")

            def on_auto_pull_done(ok: bool, res: object) -> None:
                if not ok:
                    app.blackbox_import_inflight = False
                    self.auto_abort(
                        "Auto pipeline failed while pulling blackbox logs.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                if not isinstance(res, BlackboxImportResult):
                    app.blackbox_import_inflight = False
                    self.auto_abort("Unexpected pull result in auto pipeline.")
                    return
                app.auto_latest_imported_log = ""
                if res.imported_files:
                    newest = max(res.imported_files, key=lambda item: item.modified_epoch_s)
                    app.auto_latest_imported_log = newest.local_path
                elif res.analysis_source:
                    app.auto_latest_imported_log = res.analysis_source
                if app.auto_latest_imported_log:
                    app.worker.submit(
                        self.worker_analyze_specific_blackbox_log,
                        app.auto_latest_imported_log,
                        app.blackbox_import_dir,
                        callback=on_auto_analyze_done,
                    )
                else:
                    app.worker.submit(
                        self.worker_analyze_blackbox_logs,
                        app.blackbox_import_dir,
                        callback=on_auto_analyze_done,
                    )

            def on_auto_analyze_done(ok: bool, res: object) -> None:
                if not ok:
                    app.blackbox_import_inflight = False
                    self.auto_abort(
                        "Auto pipeline failed while analyzing blackbox logs.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                if not isinstance(res, BlackboxImportResult):
                    app.blackbox_import_inflight = False
                    self.auto_abort("Unexpected analyze result in auto pipeline.")
                    return
                app.auto_import_result = res
                self.publish_auto_report(self.format_blackbox_report(res))
                app.worker.submit(
                    self.worker_generate_auto_report,
                    res,
                    self.auto_session_payload(),
                    app.auto_latest_imported_log,
                    app.blackbox_import_dir,
                    callback=on_auto_report_done,
                )

            def on_auto_report_done(ok: bool, res: object) -> None:
                app.blackbox_import_inflight = False
                if not ok:
                    self.auto_abort(
                        "Auto pipeline failed while generating report artifacts.",
                        warning=str(res) if not isinstance(res, Exception) else str(res),
                    )
                    return
                if not isinstance(res, AutoTuneReport):
                    self.auto_abort("Unexpected report generation result.")
                    return
                app.auto_latest_report = res
                try:
                    report_text = Path(res.summary_txt).read_text(encoding="utf-8", errors="replace")
                except Exception:
                    report_text = f"Report generated at {res.report_dir}\nSummary file: {res.summary_txt}"
                self.publish_auto_report(report_text)
                self.set_auto_state(AdaptiveSessionState.report_ready, "Ready")
                app.status.set(f"Auto report ready: {res.report_dir}")

            app.worker.submit(
                self.worker_enter_msc_and_import_blackbox_logs,
                selected_port,
                selected_baud,
                app.blackbox_import_dir,
                app.blackbox_msc_mount_timeout_s,
                app.blackbox_msc_mount_poll_s,
                callback=on_auto_pull_done,
            )
        except Exception as exc:
            app.blackbox_import_inflight = False
            self.auto_abort("Unable to start auto blackbox pipeline.", warning=str(exc))

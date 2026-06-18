"""Blackbox import, analysis, and report workflow.

This module owns the manual Pull MSC Logs, Analyze Logs, and Chart Step
Response button handlers. Keeping these out of app.py makes the UI button flow
much easier to follow while leaving the auto-session hardware loop untouched.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from tkinter import filedialog, messagebox
from typing import Any

from ..auto_tune_report import AutoTuneReport
from ..blackbox_import import BlackboxImportResult
from ..step_response_report import (
    MAX_STEP_RESPONSE_LOGS,
    StepResponseReport,
    format_step_response_report,
)
from ..tasks.worker_tasks import (
    analyze_specific_blackbox_log as worker_analyze_specific_blackbox_log,
    enter_msc_and_import_blackbox_logs as worker_enter_msc_and_import_blackbox_logs,
    generate_auto_report as worker_generate_auto_report,
    generate_step_response_report as worker_generate_step_response_report,
)


class BlackboxWorkflow:
    """Controls manual blackbox import/analyze/report button actions."""

    def __init__(
        self,
        app: Any,
        fc_port: Callable[[], str],
        fc_baud: Callable[[], int],
        simulation_mode_enabled: Callable[[], bool],
        auto_is_running: Callable[[], bool],
        publish_auto_report: Callable[[str], None],
        set_error: Callable[[str, Exception], None],
        disconnect_fc: Callable[..., None],
    ) -> None:
        self.app = app
        self.fc_port = fc_port
        self.fc_baud = fc_baud
        self.simulation_mode_enabled = simulation_mode_enabled
        self.auto_is_running = auto_is_running
        self.publish_auto_report = publish_auto_report
        self.set_error = set_error
        self.disconnect_fc = disconnect_fc

    def read_fc_armed_state_for_import(self, selected_port: str, selected_baud: int) -> bool:
        app = self.app
        if not app.fc_service.is_connected:
            raise RuntimeError("Connect FC manually before pulling Blackbox logs.")
        return app.fc_service.is_armed(timeout_seconds=0.8)

    def ensure_disarmed_before_import(self, selected_port: str, selected_baud: int) -> bool:
        app = self.app
        if not app.fc_service.is_connected:
            messagebox.showwarning(
                "FC Not Connected",
                "Connect FC manually before pulling Blackbox logs. The app will not auto-connect to the FC.",
                parent=app.root,
            )
            return False
        while True:
            try:
                is_armed = self.read_fc_armed_state_for_import(selected_port, selected_baud)
            except Exception as exc:
                prompt = (
                    "Could not verify whether the drone is armed.\n\n"
                    f"{exc}\n\n"
                    "Cancel to stop, or continue only if you have confirmed the drone is disarmed."
                )
                return messagebox.askokcancel("Arm State Unknown", prompt, icon="warning", parent=app.root)

            if not is_armed:
                return True

            retry = messagebox.askretrycancel(
                "Drone Armed",
                "The FC reports the drone is armed.\n\n"
                "Disarm it before pulling Blackbox logs or entering MSC mode, then click Retry.",
                icon="warning",
                parent=app.root,
            )
            if not retry:
                return False

    @staticmethod
    def format_blackbox_report(result: BlackboxImportResult) -> str:
        lines: list[str] = [result.analysis_summary]
        if result.pid_report is not None:
            if result.pid_report.headline:
                lines.append(result.pid_report.headline)
            if result.pid_report.highlights:
                lines.append("PID recommendations:")
                for line in result.pid_report.highlights[:6]:
                    lines.append(f"- {line}")
            if result.pid_report.cli_commands:
                lines.append("Suggested CLI settings:")
                for command in result.pid_report.cli_commands[:10]:
                    lines.append(f"  {command}")
            if result.pid_report.advisory:
                lines.append("Notes:")
                for note in result.pid_report.advisory[:4]:
                    lines.append(f"- {note}")
        if result.analysis_source:
            lines.append(f"Source: {result.analysis_source}")
        if result.scanned_roots:
            lines.append("Scanned: " + ", ".join(result.scanned_roots))
        if result.warnings:
            lines.append("Warnings:")
            for warning in result.warnings[:3]:
                lines.append(f"- {warning}")
        return "\n".join(lines)

    def pull_blackbox_logs(self) -> None:
        app = self.app
        try:
            if self.simulation_mode_enabled():
                app.status.set("Turn off Simulate before pulling Blackbox logs.")
                return
            if app.blackbox_import_inflight:
                app.status.set("Blackbox import already in progress.")
                return

            selected_port = self.fc_port()
            selected_baud = self.fc_baud()
            if not self.ensure_disarmed_before_import(selected_port, selected_baud):
                app.status.set("Blackbox import canceled; disarm the drone before pulling logs.")
                return
            if app.fc_service.is_connected:
                self.disconnect_fc(update_status=False)

            app.blackbox_import_inflight = True
            app.status.set(f"Requesting FC MSC mode on {selected_port} @ {selected_baud}, then scanning mounted volumes...")

            def on_pull_done(ok: bool, res: object) -> None:
                app.blackbox_import_inflight = False
                if not ok:
                    self.set_error("Blackbox import error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, BlackboxImportResult):
                    self.set_error("Blackbox import error", RuntimeError("Unexpected import task result."))
                    return

                imported_count = len(res.imported_files)
                if imported_count == 0:
                    if res.skipped_count > 0:
                        app.status.set(
                            f"No new Blackbox logs were copied ({res.skipped_count} duplicate file(s) skipped)."
                        )
                    else:
                        app.status.set("No new Blackbox logs were imported from MSC volumes.")
                else:
                    if res.skipped_count > 0:
                        app.status.set(
                            f"Imported {imported_count} Blackbox file(s) to {app.blackbox_import_dir} "
                            f"({res.skipped_count} duplicate file(s) skipped)."
                        )
                    else:
                        app.status.set(f"Imported {imported_count} Blackbox file(s) to {app.blackbox_import_dir}.")

                self.publish_auto_report(self.format_blackbox_report(res))

            app.worker.submit(
                worker_enter_msc_and_import_blackbox_logs,
                selected_port,
                selected_baud,
                app.blackbox_import_dir,
                app.blackbox_msc_mount_timeout_s,
                app.blackbox_msc_mount_poll_s,
                callback=on_pull_done,
            )
        except Exception as exc:
            app.blackbox_import_inflight = False
            self.set_error("Blackbox import error", exc)

    def analyze_blackbox_logs(self) -> None:
        app = self.app
        try:
            if app.blackbox_import_inflight:
                app.status.set("Blackbox import already in progress.")
                return

            initial_dir = app.blackbox_import_dir if app.blackbox_import_dir.exists() else Path.cwd()
            selected_log = filedialog.askopenfilename(
                parent=app.root,
                title="Select Blackbox Log to Analyze",
                initialdir=str(initial_dir),
                filetypes=(
                    ("Blackbox logs", "*.bbl *.bfl *.bbs *.txt *.csv"),
                    ("All files", "*.*"),
                ),
            )
            if not selected_log:
                app.status.set("Blackbox analysis canceled.")
                return

            app.blackbox_import_inflight = True
            selected_name = Path(selected_log).name
            app.status.set(f"Analyzing Blackbox log: {selected_name}...")

            def on_analyze_done(ok: bool, res: object) -> None:
                if not ok:
                    app.blackbox_import_inflight = False
                    self.set_error("Blackbox analyze error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, BlackboxImportResult):
                    app.blackbox_import_inflight = False
                    self.set_error("Blackbox analyze error", RuntimeError("Unexpected analysis task result."))
                    return

                summary = res.analysis_summary
                summary_head = summary.split("|", 1)[0].strip()
                if res.pid_report is not None and res.pid_report.headline:
                    summary_head = res.pid_report.headline
                app.status.set(f"Blackbox analysis complete: {summary_head}. Generating report...")
                self.publish_auto_report(self.format_blackbox_report(res))
                app.auto_latest_report = None

                session_payload = {
                    "state": "manual_analyze",
                    "stop_reason": "Manual Analyze Logs run",
                    "warning": "",
                    "elapsed_s": 0.0,
                    "metrics": {},
                }

                def on_report_done(ok2: bool, res2: object) -> None:
                    app.blackbox_import_inflight = False
                    if not ok2:
                        error_text = str(res2) if not isinstance(res2, Exception) else str(res2)
                        if "Could not resolve a primary Blackbox CSV for" in error_text:
                            error_text += (
                                "\nTip: the selected log did not resolve to a usable CSV. "
                                "Try selecting a CSV log directly, or import/decode the raw log first."
                            )
                        elif "No module named" in error_text:
                            error_text += (
                                "\nTip: this Python environment may be missing required packages "
                                "(for the HTML chart viewer: numpy and plotly)."
                            )
                        app.status.set("Blackbox analysis complete, but report generation failed.")
                        self.publish_auto_report(
                            f"{self.format_blackbox_report(res)}\n\nReport generation error: {error_text}"
                        )
                        return
                    if not isinstance(res2, AutoTuneReport):
                        self.set_error("Blackbox report error", RuntimeError("Unexpected report task result."))
                        return

                    app.auto_latest_report = res2
                    try:
                        report_text = Path(res2.summary_txt).read_text(encoding="utf-8", errors="replace")
                    except Exception:
                        report_text = f"Report generated at {res2.report_dir}\nSummary file: {res2.summary_txt}"
                    self.publish_auto_report(report_text)
                    app.status.set(f"Blackbox report generated: {res2.report_dir}")

                app.worker.submit(
                    worker_generate_auto_report,
                    res,
                    session_payload,
                    selected_log,
                    app.blackbox_import_dir,
                    callback=on_report_done,
                )

            app.worker.submit(
                worker_analyze_specific_blackbox_log,
                selected_log,
                app.blackbox_import_dir,
                callback=on_analyze_done,
            )
        except Exception as exc:
            app.blackbox_import_inflight = False
            self.set_error("Blackbox analyze error", exc)

    def generate_step_response_report(self) -> None:
        app = self.app
        try:
            if app.blackbox_import_inflight:
                app.status.set("Blackbox import/analyze already in progress.")
                return
            if self.auto_is_running():
                raise RuntimeError("Wait for the auto session/pipeline to finish first.")

            initial_dir = app.blackbox_import_dir if app.blackbox_import_dir.exists() else Path.cwd()
            selected_logs = filedialog.askopenfilenames(
                parent=app.root,
                title=f"Select Blackbox Logs for Step Response (max {MAX_STEP_RESPONSE_LOGS})",
                initialdir=str(initial_dir),
                filetypes=(
                    ("Blackbox logs", "*.bbl *.bfl *.bbs *.txt *.csv"),
                    ("All files", "*.*"),
                ),
            )
            if not selected_logs:
                app.status.set("Step response canceled.")
                return
            if len(selected_logs) > MAX_STEP_RESPONSE_LOGS:
                raise RuntimeError(f"Select at most {MAX_STEP_RESPONSE_LOGS} Blackbox logs.")

            app.blackbox_import_inflight = True
            app.step_response_button.config(state="disabled")
            count = len(selected_logs)
            app.status.set(f"Generating step response report for {count} log file(s)...")
            self.publish_auto_report(
                f"Step response generation started for {count} log file(s).\n"
                "Raw logs will be decoded with tools/blackbox_decode_INAV.exe."
            )

            def on_step_response_done(ok: bool, res: object) -> None:
                app.blackbox_import_inflight = False
                app.step_response_button.config(state="normal")
                if not ok:
                    self.set_error("Step response error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, StepResponseReport):
                    self.set_error("Step response error", RuntimeError("Unexpected step-response task result."))
                    return

                self.publish_auto_report(format_step_response_report(res))
                app.status.set(f"Step response report generated: {res.report_dir}")

            app.worker.submit(
                worker_generate_step_response_report,
                list(selected_logs),
                app.blackbox_import_dir,
                callback=on_step_response_done,
            )
        except Exception as exc:
            app.blackbox_import_inflight = False
            app.step_response_button.config(state="normal")
            self.set_error("Step response error", exc)

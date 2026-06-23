"""Runtime-state setup for the Tk application.

This keeps ModbusApp.__init__ focused on orchestration while the large
collection of mutable runtime fields lives in one controller module.
"""

from __future__ import annotations

from pathlib import Path
from collections.abc import Callable
import tkinter as tk

import serial

from serialUSB.inav_serial_service import InavSerialService

from ..attitude_service import AttitudeService
from ..adaptive_session import (
    AdaptiveCommand,
    AdaptiveExcitationController,
    AdaptiveSessionConfig,
    AdaptiveSessionState,
)
from ..auto_tune_report import AutoTuneReport
from ..blackbox_import import BlackboxImportResult
from ..constants import CHANNEL_DEFAULTS, PORT_DEFAULT, THROTTLE_CHANNEL_INDEX
from ..hardware_controller import HardwareController
from .timer_registry import TimerRegistry


class RuntimeStateController:
    """Initializes all mutable state used by ModbusApp.

    The rest of the application still accesses these as ``self.<name>`` so the
    existing callback-heavy run loop can be moved out in smaller later passes.
    """

    def __init__(self, app: object) -> None:
        self.app = app

    def initialize(self) -> None:
        self_ = self.app
        self_.start_pending = False
        self_.is_closing = False
        self_.controller = HardwareController()
        self_.run_active = False
        self_.run_port = PORT_DEFAULT
        self_.run_ser: serial.Serial | None = None
        self_.run_quant: int | None = None
        self_.run_max_count: int | None = None
        self_.adjust_repeat_after_id: str | None = None
        self_.adjust_repeat_handler: Callable[[int, int], None] | None = None
        self_.adjust_repeat_index: int | None = None
        self_.adjust_repeat_delta = 0
        self_.base_channel_outputs = CHANNEL_DEFAULTS.copy()
        self_.live_channel_outputs = self_.base_channel_outputs.copy()
        self_.worker = self_.controller.worker
        self_.fc_service = InavSerialService()
        self_.attitude_service = AttitudeService()
        self_.fc_poll_after_id: str | None = None
        self_.attitude_poll_inflight = False

        # Centralized `after_cancel()` bookkeeping.
        self_.timer_registry = TimerRegistry(self_.root.after_cancel)

        self_.level_active = False
        self_.level_after_id: str | None = None
        self_.level_pulse_inflight = False
        self_.level_timeout_deadline_s: float | None = None
        self_.auto_config = AdaptiveSessionConfig()
        self_.auto_controller: AdaptiveExcitationController | None = None
        self_.auto_state = AdaptiveSessionState.idle
        self_.auto_stop_reason = ""
        self_.auto_warning = ""
        self_.auto_session_start_s: float | None = None
        self_.auto_last_tick_s: float | None = None
        self_.auto_last_sample_s: float | None = None
        self_.auto_tick_after_id: str | None = None
        self_.auto_hold_after_id: str | None = None
        self_.fly_log_marker_after_id: str | None = None
        self_.manual_pulse_inflight = False
        self_.auto_pulse_inflight = False
        self_.auto_hold_end_requested = False
        self_.auto_settle_until_s: float | None = None
        self_.auto_recovery_mode = False
        self_.auto_stop_after_recovery = False
        self_.auto_active_command: AdaptiveCommand | None = None
        self_.auto_event_peak_delta = 0.0
        self_.auto_event_abs_peak_delta = 0.0
        self_.auto_event_signed_peak_delta = 0.0
        self_.auto_event_response_delay_s: float | None = None
        self_.auto_event_baseline = 0.0
        self_.auto_event_start_s = 0.0
        self_.auto_axis_output_sign: dict[str, int] = {"roll": 1, "pitch": 1}
        self_.auto_probe_axes_pending: list[str] = []
        self_.auto_original_base_outputs: list[int] | None = None
        self_.auto_start_throttle_us = self_.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        self_.auto_current_throttle_us = self_.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        self_.auto_peak_throttle_us = self_.base_channel_outputs[THROTTLE_CHANNEL_INDEX]
        self_.auto_latest_report: AutoTuneReport | None = None
        self_.auto_import_result: BlackboxImportResult | None = None
        self_.auto_latest_imported_log: str = ""
        self_.blackbox_import_inflight = False
        self_.blackbox_import_dir = (Path(__file__).resolve().parents[2] / "blackbox_imports").resolve()
        self_.blackbox_msc_mount_timeout_s = 12.0
        self_.blackbox_msc_mount_poll_s = 1.0
        self_.pid_plan_active = False
        self_.pid_plan: LoadedPIDTuningPlan | None = None
        self_.pid_plan_phase = "idle"
        self_.pid_plan_index = 0
        self_.pid_plan_selected_d: int | None = None
        self_.pid_plan_selected_p: dict[str, int] | None = None
        self_.pid_plan_selected_i: dict[str, int] | None = None
        self_.pid_plan_selected_ff: dict[str, int] | None = None
        self_.pid_plan_waiting_for_fly_log = False
        self_.pid_plan_current_candidate_title = ""
        self_.pid_plan_current_candidate_phase = ""
        self_.pid_plan_current_candidate_target: dict[str, dict[str, int]] | None = None
        self_.pid_plan_fly_log_active = False
        self_.fly_log_finishing = False
        self_.fly_log_mixer_snapshot_path: Path | None = None
        self_.pid_progress_window: tk.Toplevel | None = None
        self_.pid_progress_phase_labels: dict[str, tk.Label] = {}
        self_.pid_progress_current_var = tk.StringVar(value="No PID tuning plan is active.")
        self_.pid_progress_action_var = tk.StringVar(value="Generate or start a PID tuning plan.")
        self_.pid_progress_selection_var = tk.StringVar(value="")
        self_.pid_progress_plan_var = tk.StringVar(value="")
        self_.pid_progress_target_text: tk.Text | None = None
        self_.pid_ff_labels = ("P", "I", "D", "FF")
        self_.pid_ff_adjust_fields = [
            ("roll", "p"),
            ("pitch", "p"),
            ("roll", "i"),
            ("pitch", "i"),
            ("roll", "d"),
            ("pitch", "d"),
            ("roll", "ff"),
            ("pitch", "ff"),
        ]

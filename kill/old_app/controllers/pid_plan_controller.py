"""PID tuning plan workflow.

This module is a first extraction target from `modbus_app/app.py`.

NOTE: This initial version is intentionally lightweight and keeps UI/dialog
responsibilities inside `ModbusApp` via callbacks.

We will migrate the nested PID-plan functions incrementally, while preserving
exact behavior.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

import tkinter as tk

from modbus_app.pid_tuning_workflow import LoadedPIDTuningPlan



@dataclass
class PidPlanContext:
    plan: LoadedPIDTuningPlan | None = None

    active: bool = False
    phase: str = "idle"  # safe_start, d_sweep, p_sweep, d_recheck, i_sweep, ff_sweep, final_write, complete
    index: int = 0

    selected_d: int | None = None
    selected_p: dict[str, int] | None = None
    selected_i: dict[str, int] | None = None
    selected_ff: dict[str, int] | None = None

    waiting_for_fly_log: bool = False
    current_candidate_title: str = ""
    current_candidate_phase: str = ""
    current_candidate_target: dict[str, dict[str, int]] | None = None

    pid_plan_fly_log_active: bool = False


class PidPlanController:
    """Encapsulates PID plan state transitions.

    The controller delegates all UI interactions (dialogs/messages/staging
    writes) back to callbacks provided by the GUI.
    """

    def __init__(
        self,
        ctx: PidPlanContext,
        *,
        root: tk.Tk,
        get_current_fc_pid_ff: Callable[[tuple[str, ...]], dict[str, dict[str, int]]],
        ensure_disarmed_before_write: Callable[[], bool],
        stage_pid_ff_values: Callable[[dict[str, dict[str, int]]], None],
        read_inputs_for_plan_generation: Callable[[], object],
        load_plan: Callable[[object], LoadedPIDTuningPlan],
        show_info: Callable[[str, str], None],
        show_yes_no_cancel: Callable[[str, str], Optional[bool]],
        # Optional UI update hooks
        on_plan_report: Callable[[str], None] | None = None,
    ) -> None:
        self.ctx = ctx
        self.root = root
        self.get_current_fc_pid_ff = get_current_fc_pid_ff
        self.ensure_disarmed_before_write = ensure_disarmed_before_write
        self.stage_pid_ff_values_cb = stage_pid_ff_values
        self.read_inputs_for_plan_generation = read_inputs_for_plan_generation
        self.load_plan_cb = load_plan
        self.show_info = show_info
        self.show_yes_no_cancel = show_yes_no_cancel
        self.on_plan_report = on_plan_report

    # --- The incremental migration approach ---
    # For now, we only define state initialization/reset helpers.

    def start(self, plan: LoadedPIDTuningPlan) -> None:
        self.ctx.plan = plan
        self.ctx.active = True
        self.ctx.phase = "safe_start"
        self.ctx.index = 0
        self.ctx.selected_d = None
        self.ctx.selected_p = None
        self.ctx.selected_i = None
        self.ctx.selected_ff = None
        self.ctx.waiting_for_fly_log = False
        self.ctx.current_candidate_title = ""
        self.ctx.current_candidate_phase = ""
        self.ctx.current_candidate_target = None
        self.ctx.pid_plan_fly_log_active = False

    def stop(self) -> None:
        self.ctx = PidPlanContext()



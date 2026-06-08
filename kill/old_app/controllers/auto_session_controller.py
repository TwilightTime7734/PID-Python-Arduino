"""Adaptive auto session controller (incremental extraction).

This file is the next extraction target from `modbus_app/app.py`.

This initial version only provides structure/placeholders so we can wire the
GUI safely without changing behavior yet.
"""

from __future__ import annotations

from dataclasses import dataclass

from modbus_app.adaptive_session import (
    AdaptiveCommand,
    AdaptiveExcitationController,
    AdaptiveSessionConfig,
    AdaptiveSessionState,
)



@dataclass
class AutoSessionContext:
    config: AdaptiveSessionConfig | None = None
    controller: AdaptiveExcitationController | None = None

    state: AdaptiveSessionState = AdaptiveSessionState.idle
    stop_reason: str = ""
    warning: str = ""

    active_command: AdaptiveCommand | None = None
    pulse_inflight: bool = False
    hold_end_requested: bool = False


class AutoSessionController:
    def __init__(self, ctx: AutoSessionContext) -> None:
        self.ctx = ctx

    def set_state(self, next_state: AdaptiveSessionState, reason: str = "") -> None:
        self.ctx.state = next_state
        if reason:
            self.ctx.warning = reason


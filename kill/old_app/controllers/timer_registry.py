"""Small helper to centralize Tkinter `after()` timer cancellation.

Tkinter `after()` returns an opaque id that must be canceled with `after_cancel`.
In the current app, many timers are created across multiple workflows.

This registry provides:
- register(id) and cancel_all()
- cancel_one(id)

It is intentionally tiny so it can be used without restructuring large parts
of the UI.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class TimerRegistry:
    after_cancel: Any
    _ids: set[Any] = field(default_factory=set)

    def register(self, timer_id: Any) -> Any:
        self._ids.add(timer_id)
        return timer_id

    def cancel_one(self, timer_id: Any) -> None:
        if timer_id in self._ids:
            try:
                self.after_cancel(timer_id)
            finally:
                self._ids.discard(timer_id)

    def cancel_all(self) -> None:
        ids = list(self._ids)
        self._ids.clear()
        for timer_id in ids:
            try:
                self.after_cancel(timer_id)
            except Exception:
                # Best-effort: GUI close should never crash.
                pass


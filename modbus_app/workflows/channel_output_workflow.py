"""Manual channel output display and live-output helpers."""

from __future__ import annotations

import tkinter as tk
from collections.abc import Callable

from ..constants import (
    ADJUST_REPEAT_INITIAL_MS,
    ADJUST_REPEAT_INTERVAL_MS,
    THROTTLE_CHANNEL_INDEX,
)
from ..ch8_marker import channels_with_pid_test_ch8
from ..ui import normalize_channel_value, normalize_offset_value


class ChannelOutputWorkflow:
    def __init__(self, app, set_error: Callable[[str, Exception], None]) -> None:
        self.app = app
        self.set_error = set_error

    def draw_channel_output(self, index: int, value: int) -> None:
        app = self.app
        clamped = max(1000, min(2000, value))
        canvas = app.channel_output_canvases[index]
        fill_id = app.channel_output_fill_ids[index]

        left = 2.0
        right = 94.0
        center = (left + right) / 2.0
        y1 = 3.0
        y2 = 13.0

        if clamped < 1500:
            ratio = (1500 - clamped) / 500.0
            x = center - (center - left) * ratio
            canvas.coords(fill_id, x, y1, center, y2)
            canvas.itemconfig(fill_id, fill="#E38C8C")
        elif clamped > 1500:
            ratio = (clamped - 1500) / 500.0
            x = center + (right - center) * ratio
            canvas.coords(fill_id, center, y1, x, y2)
            canvas.itemconfig(fill_id, fill="#94D98F")
        else:
            canvas.coords(fill_id, center, y1, center, y2)
            canvas.itemconfig(fill_id, fill="#94D98F")

    def parse_channel_values_with_defaults(self) -> list[int]:
        app = self.app
        return [normalize_channel_value(int(entry.get().strip())) for entry in app.ch_entries]

    def parse_offset_values_with_defaults(self) -> list[int]:
        app = self.app
        return [normalize_offset_value(int(entry.get().strip())) for entry in app.off_entries]

    def adjust_channel_value(self, index: int, delta: int) -> None:
        app = self.app
        current = int(app.ch_entries[index].get().strip())
        updated = normalize_channel_value(current + delta)
        self.set_channel_entry_value(index, updated)
        self.on_output_inputs_changed()

    def get_adjust_delta(self, event: tk.Event, step: int = 5) -> int:
        width = int(event.widget.cget("width"))
        mid_x = width / 2
        return -step if event.x <= mid_x else step

    def cancel_adjust_repeat(self) -> None:
        app = self.app
        if app.adjust_repeat_after_id is not None:
            try:
                app.root.after_cancel(app.adjust_repeat_after_id)
            except Exception:
                pass
            finally:
                app.adjust_repeat_after_id = None
        app.adjust_repeat_handler = None
        app.adjust_repeat_index = None
        app.adjust_repeat_delta = 0

    def schedule_adjust_repeat(self) -> None:
        app = self.app
        if app.adjust_repeat_handler is None or app.adjust_repeat_index is None or app.adjust_repeat_delta == 0:
            app.adjust_repeat_after_id = None
            return
        app.adjust_repeat_handler(app.adjust_repeat_index, app.adjust_repeat_delta)
        app.adjust_repeat_after_id = app.root.after(ADJUST_REPEAT_INTERVAL_MS, self.schedule_adjust_repeat)

    def on_adjust_press(
        self,
        adjust_handler: Callable[[int, int], None],
        index: int,
        event: tk.Event,
        step: int = 5,
    ) -> None:
        app = self.app
        self.cancel_adjust_repeat()
        delta = self.get_adjust_delta(event, step=step)
        adjust_handler(index, delta)
        app.adjust_repeat_handler = adjust_handler
        app.adjust_repeat_index = index
        app.adjust_repeat_delta = delta
        app.adjust_repeat_after_id = app.root.after(ADJUST_REPEAT_INITIAL_MS, self.schedule_adjust_repeat)

    def on_adjust_release(self, _event: tk.Event) -> None:
        self.cancel_adjust_repeat()

    def set_live_channel_outputs(self, values: list[int]) -> None:
        app = self.app
        app.live_channel_outputs = values[: len(app.channel_output_canvases)].copy()
        self.refresh_channel_outputs()

    def arduino_output_connected(self) -> bool:
        return bool(self.app.controller.is_connected)

    def restore_base_outputs_after_hold(self, offsets: list[int] | None = None) -> None:
        app = self.app
        if not self.arduino_output_connected():
            return
        restore_offsets = offsets.copy() if offsets is not None else self.parse_offset_values_with_defaults()
        self.set_live_channel_outputs(app.base_channel_outputs)
        self.queue_live_channel_update(app.base_channel_outputs.copy(), restore_offsets)

    def refresh_channel_outputs(self) -> None:
        app = self.app
        for i, value in enumerate(app.live_channel_outputs[: len(app.channel_output_canvases)]):
            self.draw_channel_output(i, value)

    def queue_live_channel_update(
        self,
        channels: list[int],
        offsets: list[int],
        after_update: Callable[[bool, object], None] | None = None,
    ) -> None:
        app = self.app
        if not self.arduino_output_connected():
            if after_update is not None:
                after_update(False, RuntimeError("Arduino output is disconnected."))
            return

        def on_live_update_done(ok: bool, res: object) -> None:
            if not ok:
                self.set_error("Live update error", res if isinstance(res, Exception) else RuntimeError(res))
            else:
                if (
                    not isinstance(res, tuple)
                    or len(res) != 3
                    or not isinstance(res[0], int)
                    or not isinstance(res[1], int)
                    or not isinstance(res[2], list)
                ):
                    self.set_error("Live update error", RuntimeError("Unexpected worker result from live update task"))
                else:
                    app.run_quant = res[0]
                    app.run_max_count = res[1]
                    sent_channels = [int(v) for v in res[2]]
                    app.base_channel_outputs = sent_channels
                    self.set_live_channel_outputs(sent_channels)

            if after_update is not None:
                try:
                    after_update(ok, res)
                except Exception as exc:
                    self.set_error("Live update callback error", exc)

        app.controller.queue_live_channel_update(
            channels.copy(),
            offsets.copy(),
            callback=on_live_update_done,
        )

    def set_channel_entry_value(self, index: int, value: int) -> None:
        app = self.app
        normalized = normalize_channel_value(value)
        app.ch_entries[index].set(str(normalized))

    def apply_auto_base_outputs(self, channels: list[int], safety_text: str = "", send_update: bool = True) -> None:
        app = self.app
        clamped = [normalize_channel_value(value) for value in channels[: len(app.ch_entries)]]
        app.base_channel_outputs = clamped.copy()
        if app.auto_original_base_outputs is not None:
            app.auto_current_throttle_us = clamped[THROTTLE_CHANNEL_INDEX]
            app.auto_peak_throttle_us = max(app.auto_peak_throttle_us, app.auto_current_throttle_us)
        for index, value in enumerate(clamped[: len(app.ch_entries)]):
            self.set_channel_entry_value(index, value)
        self.set_live_channel_outputs(clamped)
        if send_update and self.arduino_output_connected():
            self.queue_live_channel_update(clamped.copy(), self.parse_offset_values_with_defaults())
        if safety_text:
            app.status.set(safety_text)

    def restore_auto_original_base_outputs(self) -> None:
        app = self.app
        if app.auto_original_base_outputs is None:
            return
        original = app.auto_original_base_outputs
        app.auto_original_base_outputs = None
        self.apply_auto_base_outputs(original, "restored pre-auto outputs")

    def lower_throttle_for_abort(self) -> None:
        app = self.app
        channels = app.base_channel_outputs.copy()
        if len(channels) <= THROTTLE_CHANNEL_INDEX:
            return
        target = max(1000, min(2000, int(app.auto_config.abort_throttle_us)))
        channels[THROTTLE_CHANNEL_INDEX] = min(channels[THROTTLE_CHANNEL_INDEX], target)
        channels = channels_with_pid_test_ch8(channels, active=False)
        self.apply_auto_base_outputs(channels, send_update=False)
        app.auto_original_base_outputs = None
        if self.arduino_output_connected():
            self.queue_live_channel_update(channels.copy(), self.parse_offset_values_with_defaults())

    def on_output_inputs_changed(self) -> None:
        app = self.app
        if not self.arduino_output_connected():
            self.set_live_channel_outputs(self.parse_channel_values_with_defaults())
            return

        channels = self.parse_channel_values_with_defaults()
        offsets = self.parse_offset_values_with_defaults()
        self.set_live_channel_outputs(channels)
        app.base_channel_outputs = channels.copy()
        self.queue_live_channel_update(channels, offsets)

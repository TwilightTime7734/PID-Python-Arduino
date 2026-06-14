"""Hardware output and worker management for the Modbus app."""

from __future__ import annotations

from collections.abc import Callable
import serial

from .constants import PID_TEST_CH8_OFF_US, PPM_OUTPUT_CHANNEL_COUNT, PORT_DEFAULT
from .serial_protocol import (
    open_serial,
    run_ppm_on_serial,
    stop_ppm_on_serial,
)
from .worker import SerialWorker

ControllerCallback = Callable[[bool, object], None]


class HardwareController:
    def __init__(self) -> None:
        self.worker = SerialWorker()
        self.run_active = False
        self.run_port = PORT_DEFAULT
        self.run_ser: serial.Serial | None = None
        self.run_quant: int | None = None
        self.run_max_count: int | None = None
        self.channel_update_inflight = False
        self.pending_channel_update_channels: list[int] | None = None
        self.pending_channel_update_offsets: list[int] | None = None
        self.pending_channel_update_after: ControllerCallback | None = None

    @property
    def is_connected(self) -> bool:
        return self.run_active and self.run_ser is not None

    def start_output(
        self,
        port: str,
        channels: list[int],
        offsets: list[int],
        callback: ControllerCallback | None = None,
    ) -> None:
        def on_start_done(ok: bool, res: object) -> None:
            if not ok:
                if callback is not None:
                    callback(False, res)
                return
            if (
                not isinstance(res, tuple)
                or len(res) != 3
                or not isinstance(res[0], int)
                or not isinstance(res[1], int)
                or (res[2] is not None and not isinstance(res[2], str))
            ):
                if callback is not None:
                    callback(False, RuntimeError("Unexpected worker result from start task"))
                return
            self.run_port = port
            self.run_quant = res[0]
            self.run_max_count = res[1]
            self.run_ser = self.worker.ser
            self.run_active = True
            self.channel_update_inflight = False
            self.pending_channel_update_channels = None
            self.pending_channel_update_offsets = None
            self.pending_channel_update_after = None
            if callback is not None:
                callback(True, res)

        if self.run_ser is None:
            self.worker.submit(
                self._task_open_and_start,
                port,
                channels,
                offsets,
                callback=on_start_done,
            )
            return

        if port != self.run_port:
            if callback is not None:
                callback(False, RuntimeError(f"Output is active on {self.run_port}. Press Disconnect Arduino before switching ports."))
            return

        self.worker.submit(
            self._task_run_ppm_on_existing,
            channels,
            offsets,
            callback=on_start_done,
        )

    def stop_output(self, callback: ControllerCallback | None = None) -> None:
        def on_stop_done(ok: bool, res: object) -> None:
            if not ok:
                if callback is not None:
                    callback(False, res)
                return
            self.run_ser = None
            self.run_quant = None
            self.run_max_count = None
            self.run_active = False
            self.channel_update_inflight = False
            self.pending_channel_update_channels = None
            self.pending_channel_update_offsets = None
            self.pending_channel_update_after = None
            if callback is not None:
                callback(True, res)

        self.worker.submit(self._task_stop, callback=on_stop_done)

    def shutdown(self, callback: ControllerCallback | None = None) -> None:
        def on_shutdown_done(ok: bool, res: object) -> None:
            self.run_ser = None
            self.run_quant = None
            self.run_max_count = None
            self.run_active = False
            self.channel_update_inflight = False
            self.pending_channel_update_channels = None
            self.pending_channel_update_offsets = None
            self.pending_channel_update_after = None
            if callback is not None:
                callback(ok, res)

        self.worker.submit(self._task_shutdown, callback=on_shutdown_done)

    def queue_live_channel_update(
        self,
        channels: list[int],
        offsets: list[int],
        callback: ControllerCallback | None = None,
    ) -> None:
        if not self.is_connected:
            if callback is not None:
                callback(False, RuntimeError("Arduino output is disconnected."))
            return

        if self.channel_update_inflight:
            self.pending_channel_update_channels = channels.copy()
            self.pending_channel_update_offsets = offsets.copy()
            self.pending_channel_update_after = callback
            return

        self.channel_update_inflight = True

        def on_live_update_done(ok: bool, res: object) -> None:
            self.channel_update_inflight = False
            if ok and isinstance(res, tuple) and len(res) == 3 and isinstance(res[0], int) and isinstance(res[1], int) and isinstance(res[2], list):
                self.run_quant = res[0]
                self.run_max_count = res[1]
            if callback is not None:
                try:
                    callback(ok, res)
                except Exception:
                    # Caller is responsible for logging or handling callback failures.
                    pass

            if not self.is_connected:
                self.pending_channel_update_channels = None
                self.pending_channel_update_offsets = None
                self.pending_channel_update_after = None
                return

            if self.pending_channel_update_channels is None or self.pending_channel_update_offsets is None:
                return

            next_channels = self.pending_channel_update_channels
            next_offsets = self.pending_channel_update_offsets
            next_after = self.pending_channel_update_after
            self.pending_channel_update_channels = None
            self.pending_channel_update_offsets = None
            self.pending_channel_update_after = None
            self.queue_live_channel_update(next_channels, next_offsets, callback=next_after)

        self.worker.submit(
            self._task_update_channels,
            channels.copy(),
            offsets.copy(),
            callback=on_live_update_done,
        )

    def _task_open_and_start(
        self,
        worker_self: SerialWorker,
        port: str,
        channels: list[int],
        offsets: list[int],
    ):
        ser = open_serial(port)
        worker_self.ser = ser
        try:
            ppm_channels = self._ppm_channels_for_firmware(channels)
            ppm_offsets = self._ppm_offsets_for_firmware(offsets, len(ppm_channels))
            quant, max_count, version_warning = run_ppm_on_serial(ser, ppm_channels, ppm_offsets)
        except Exception:
            ser.close()
            worker_self.ser = None
            raise
        return (quant, max_count, version_warning)

    def _task_stop(self, worker_self: SerialWorker):
        if worker_self.ser is not None:
            try:
                stop_ppm_on_serial(worker_self.ser)
            finally:
                try:
                    worker_self.ser.close()
                finally:
                    worker_self.ser = None
        return None

    def _task_run_ppm_on_existing(
        self,
        worker_self: SerialWorker,
        channels: list[int],
        offsets: list[int],
    ):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        ppm_channels = self._ppm_channels_for_firmware(channels)
        ppm_offsets = self._ppm_offsets_for_firmware(offsets, len(ppm_channels))
        return run_ppm_on_serial(worker_self.ser, ppm_channels, ppm_offsets)

    def _task_update_channels(
        self,
        worker_self: SerialWorker,
        channels: list[int],
        offsets: list[int],
    ):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        ppm_channels = self._ppm_channels_for_firmware(channels)
        ppm_offsets = self._ppm_offsets_for_firmware(offsets, len(ppm_channels))
        quant, max_count, _ = run_ppm_on_serial(worker_self.ser, ppm_channels, ppm_offsets)
        # Return the actual frame values sent to the Arduino after padding and clamping.
        return (quant, max_count, ppm_channels)

    def _task_shutdown(self, worker_self: SerialWorker):
        if worker_self.ser is None:
            return None
        try:
            stop_ppm_on_serial(worker_self.ser)
        except Exception:
            pass
        finally:
            try:
                worker_self.ser.close()
            finally:
                worker_self.ser = None
        return None

    @staticmethod
    def _ppm_channels_for_firmware(channels: list[int]) -> list[int]:
        count = max(PPM_OUTPUT_CHANNEL_COUNT, len(channels))
        output = [PID_TEST_CH8_OFF_US] * count
        for index, value in enumerate(channels):
            output[index] = max(1000, min(2000, int(value)))
        return output

    @staticmethod
    def _ppm_offsets_for_firmware(offsets: list[int], channel_count: int) -> list[int]:
        output = [0] * channel_count
        for index, value in enumerate(offsets[:channel_count]):
            output[index] = int(value)
        return output

"""Arduino and FC connection workflow."""

from __future__ import annotations

import tkinter as tk
from collections.abc import Callable
from tkinter import messagebox

from serial.tools import list_ports

from ..adaptive_session import AdaptiveSessionState
from ..constants import (
    FC_DEVICE_ID,
    FC_DEVICE_PID,
    FC_DEVICE_VID,
    FC_PORT_DEFAULT,
)
from ..ch8_marker import channels_with_pid_test_ch8


class ConnectionWorkflow:
    def __init__(
        self,
        *,
        app,
        port: Callable[[], str],
        arduino_baud: Callable[[], int],
        fc_port: Callable[[], str],
        fc_baud: Callable[[], int],
        simulation_mode_enabled: Callable[[], bool],
        auto_is_running: Callable[[], bool],
        refresh_level_button_state: Callable[[bool], None],
        refresh_fly_log_button_state: Callable[[], None],
        clear_pid_ff_displays: Callable[[], None],
        queue_fc_pid_ff_refresh: Callable[[str, int], None],
        set_live_channel_outputs: Callable[[list[int]], None],
        parse_channel_values_with_defaults: Callable[[], list[int]],
        parse_offset_values_with_defaults: Callable[[], list[int]],
        set_error: Callable[[str, Exception], None],
        auto_abort: Callable[..., None],
    ) -> None:
        self.app = app
        self.port = port
        self.arduino_baud = arduino_baud
        self.fc_port = fc_port
        self.fc_baud = fc_baud
        self.simulation_mode_enabled = simulation_mode_enabled
        self.auto_is_running = auto_is_running
        self.refresh_level_button_state = refresh_level_button_state
        self.refresh_fly_log_button_state = refresh_fly_log_button_state
        self.clear_pid_ff_displays = clear_pid_ff_displays
        self.queue_fc_pid_ff_refresh = queue_fc_pid_ff_refresh
        self.set_live_channel_outputs = set_live_channel_outputs
        self.parse_channel_values_with_defaults = parse_channel_values_with_defaults
        self.parse_offset_values_with_defaults = parse_offset_values_with_defaults
        self.set_error = set_error
        self.auto_abort = auto_abort

    def select_fc_port(self, port_infos) -> str:
        target_id = FC_DEVICE_ID.upper()
        for info in port_infos:
            vid = getattr(info, "vid", None)
            pid = getattr(info, "pid", None)
            if vid == FC_DEVICE_VID and pid == FC_DEVICE_PID:
                device = str(getattr(info, "device", "") or "").strip()
                if device:
                    return device
            hwid = str(getattr(info, "hwid", "") or "")
            if target_id in hwid.upper():
                device = str(getattr(info, "device", "") or "").strip()
                if device:
                    return device
        return FC_PORT_DEFAULT

    def list_scanned_ports(self, port_infos) -> list[str]:
        ports = [str(getattr(p, "device", "") or "").strip() for p in port_infos]
        return [p for p in ports if p]

    def populate_port_dropdowns(self, ports) -> None:
        app = self.app
        values = tuple(ports)
        app.port_entry.config(values=values)
        app.fc_port_entry.config(values=values)

    def scan_fc_ports(self, update_status: bool = True) -> None:
        app = self.app
        port_infos = sorted(
            list_ports.comports(),
            key=lambda p: str(getattr(p, "device", "") or "").upper(),
        )
        ports = self.list_scanned_ports(port_infos)
        self.populate_port_dropdowns(ports)
        selected_port = self.select_fc_port(port_infos)
        app.fc_port_entry.set(selected_port)
        if update_status:
            if ports:
                app.status.set(f"Detected ports: {', '.join(ports)}. FC port set to {selected_port}.")
            else:
                app.status.set(f"No serial ports detected. FC port set to {selected_port}.")

    def update_link_indicators(self) -> None:
        app = self.app
        sim_mode = self.simulation_mode_enabled()
        arduino_connected = app.controller.is_connected
        fc_connected = app.fc_service.is_connected

        app.port_entry.config(state="disabled" if arduino_connected or app.start_pending else "normal")
        app.port_baud_entry.config(state="disabled" if arduino_connected or app.start_pending else "readonly")
        app.fc_port_entry.config(state="disabled" if fc_connected else "normal")
        app.fc_baud_entry.config(state="disabled" if fc_connected else "readonly")

        if app.controller.is_connected:
            app.pc_link_box.config(text="PC-ARD OPEN", bg="#2E7D32", fg="white")
        else:
            app.pc_link_box.config(text="PC-ARD CLOSED", bg="#8B1E1E", fg="white")

        if fc_connected:
            app.connect_fc_button.config(
                text="Disconnect FC",
                state="normal",
                bg="#BEEAC4",
                activebackground="#A6E1AE",
                fg="#0E2F11",
                activeforeground="#0E2F11",
            )
        else:
            app.connect_fc_button.config(
                text="Connect FC",
                state="normal",
                bg="#F3C1C1",
                activebackground="#ECA8A8",
                fg="#3A1111",
                activeforeground="#3A1111",
            )
        if sim_mode:
            app.connect_fc_button.config(state="disabled")

        pid_save_state = "normal" if fc_connected and not sim_mode else "disabled"
        app.load_pid_ff_button.config(state=pid_save_state)
        app.save_pid_ff_button.config(state=pid_save_state)

        if app.start_pending:
            app.arduino_button.config(
                text="Connecting...",
                state="disabled",
                bg="#F3E6B3",
                activebackground="#EBD997",
                fg="#3F3210",
                activeforeground="#3F3210",
            )
        elif arduino_connected:
            app.arduino_button.config(
                text="Disconnect Arduino",
                state="normal",
                bg="#BEEAC4",
                activebackground="#A6E1AE",
                fg="#0E2F11",
                activeforeground="#0E2F11",
            )
        else:
            app.arduino_button.config(
                text="Connect Arduino",
                state="normal",
                bg="#F3C1C1",
                activebackground="#ECA8A8",
                fg="#3A1111",
                activeforeground="#3A1111",
            )
        if sim_mode:
            app.arduino_button.config(state="disabled")

        simulation_blocked = app.start_pending or arduino_connected or fc_connected
        app.simulation_mode_checkbutton.config(state="normal" if sim_mode or not simulation_blocked else "disabled")
        self.refresh_level_button_state(app.attitude_service.is_connected)
        self.refresh_fly_log_button_state()

    def connect_fc(self) -> None:
        app = self.app
        try:
            if self.simulation_mode_enabled():
                raise RuntimeError("Turn off Simulate before connecting FC.")
            if app.fc_service.is_connected:
                return
            selected_port = self.fc_port()
            selected_baud = self.fc_baud()
            app.fc_service.connect(selected_port, selected_baud)
            self.update_link_indicators()
            app.status.set(f"FC connected: {selected_port} @ {selected_baud}. Loading PID/FF...")
            self.queue_fc_pid_ff_refresh(selected_port, selected_baud)
        except Exception as exc:
            self.set_error("FC connect error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def disconnect_fc(self, update_status: bool = True) -> None:
        app = self.app
        if app.auto_state in (AdaptiveSessionState.adaptive_run, AdaptiveSessionState.recovery):
            self.auto_abort("FC disconnected during adaptive session.", continue_pipeline=False)
        try:
            app.fc_service.disconnect()
        except Exception as exc:
            if not app.is_closing:
                self.set_error("FC disconnect error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))
        finally:
            self.clear_pid_ff_displays()
            self.update_link_indicators()
            if update_status and not app.is_closing:
                app.status.set("FC disconnected.")

    def toggle_fc(self) -> None:
        app = self.app
        if self.simulation_mode_enabled():
            app.status.set("Turn off Simulate before connecting FC.")
            return
        if app.fc_service.is_connected:
            self.disconnect_fc()
        else:
            self.connect_fc()

    def toggle_arduino(self) -> None:
        app = self.app
        if self.simulation_mode_enabled():
            app.status.set("Turn off Simulate before connecting Arduino.")
            return
        if app.start_pending:
            return
        if app.controller.is_connected:
            self.stop_arduino()
        else:
            self.start_arduino()

    def start_arduino(self) -> None:
        app = self.app
        try:
            if self.simulation_mode_enabled():
                raise RuntimeError("Turn off Simulate before connecting Arduino.")
            if app.start_pending:
                raise RuntimeError("Start is already in progress.")
            channels = self.parse_channel_values_with_defaults()
            offsets = self.parse_offset_values_with_defaults()
            selected_port = self.port()
            selected_baud = self.arduino_baud()
            if app.controller.is_connected and (
                selected_port != app.controller.run_port
                or selected_baud != app.controller.run_baudrate
            ):
                raise RuntimeError(
                    f"Output is active on {app.controller.run_port} @ {app.controller.run_baudrate}. "
                    "Press Disconnect Arduino before switching ports or baud."
                )
            channels = channels_with_pid_test_ch8(channels, active=False)

            def on_start_done(ok: bool, res: object) -> None:
                app.start_pending = False
                if not ok:
                    app.attitude_service.disconnect()
                    self.update_link_indicators()
                    self.set_error("Start error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if (
                    not isinstance(res, tuple)
                    or len(res) != 3
                    or not isinstance(res[0], int)
                    or not isinstance(res[1], int)
                    or (res[2] is not None and not isinstance(res[2], str))
                ):
                    self.update_link_indicators()
                    self.set_error("Start error", RuntimeError("Unexpected worker result from start task"))
                    return
                app.base_channel_outputs = channels.copy()
                app.attitude_service.connect()
                self.set_live_channel_outputs(app.base_channel_outputs)
                self.update_link_indicators()
                version_warning = res[2]
                if version_warning:
                    app.status.set(version_warning)
                    messagebox.showwarning("Firmware version", version_warning)
                else:
                    app.status.set(f"PPM output configured and started on {selected_port} @ {selected_baud}.")

            app.start_pending = True
            self.update_link_indicators()
            app.controller.start_output(
                selected_port,
                selected_baud,
                channels,
                offsets,
                callback=on_start_done,
            )

        except Exception as exc:
            app.start_pending = False
            self.update_link_indicators()
            self.set_error("Start error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

    def stop_arduino(self) -> None:
        app = self.app
        try:
            def on_stop_done(ok: bool, res: object) -> None:
                if not ok:
                    self.set_error("Stop error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if res is not None:
                    self.set_error("Stop error", RuntimeError("Unexpected worker result from stop task"))
                    return
                app.attitude_service.disconnect()
                self.set_live_channel_outputs(self.parse_channel_values_with_defaults())
                self.update_link_indicators()
                app.status.set("PPM output stopped.")

            app.controller.stop_output(callback=on_stop_done)
        except Exception as exc:
            self.set_error("Stop error", exc if isinstance(exc, Exception) else RuntimeError(str(exc)))

"""UI widget binding helpers for ModbusApp."""

from __future__ import annotations


class WidgetBindings:
    """Copies widgets from the UI bundle onto the application object.

    This preserves the existing ``self.<widget>`` access pattern while moving
    the long list of assignments out of ``app.py``.
    """

    @staticmethod
    def attach_to(app: object, ui: object) -> None:
        app.port_entry = ui.port_entry
        app.channel_adjust_canvases = ui.channel_adjust_canvases
        app.ch_entries = ui.ch_entries
        app.off_entries = ui.off_entries
        app.channel_output_canvases = ui.channel_output_canvases
        app.channel_output_fill_ids = ui.channel_output_fill_ids
        app.level_button = ui.level_button
        app.status = ui.status
        app.horizon = ui.horizon
        app.roll_text = ui.roll_text
        app.pitch_text = ui.pitch_text
        app.roll_pidff_vars = ui.roll_pidff_vars
        app.pitch_pidff_vars = ui.pitch_pidff_vars
        app.starting_values_table = ui.starting_values_table
        app.roll_values_table = ui.roll_values_table
        app.pitch_values_table = ui.pitch_values_table
        app.pid_ff_adjust_canvases = ui.pid_ff_adjust_canvases
        app.load_pid_ff_button = ui.load_pid_ff_button
        app.save_pid_ff_button = ui.save_pid_ff_button
        app.port_baud_entry = ui.port_baud_entry
        app.fc_port_entry = ui.fc_port_entry
        app.fc_baud_entry = ui.fc_baud_entry
        app.scan_fc_button = ui.scan_fc_button
        app.connect_fc_button = ui.connect_fc_button
        app.import_blackbox_button = ui.import_blackbox_button
        app.analyze_blackbox_button = ui.analyze_blackbox_button
        app.arduino_button = ui.arduino_button
        app.fly_log_button = ui.fly_log_button
        app.pulse_axis_combo = ui.pulse_axis_combo
        app.pulse_force_combo = ui.pulse_force_combo
        app.pulse_time_combo = ui.pulse_time_combo
        app.pulse_positive_button = ui.pulse_positive_button
        app.pulse_negative_button = ui.pulse_negative_button
        app.step_response_button = ui.step_response_button
        app.pid_tuning_plan_button = ui.pid_tuning_plan_button
        app.mixer_debug_button = ui.mixer_debug_button

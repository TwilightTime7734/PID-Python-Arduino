"""Temporarily zero the locked roll/pitch axis plus yaw PID/FF for Fly/Log."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from serialUSB.inav_serial_service import AxisPidFf, FF_SETTING_NAME, PID_SETTING_NAME


@dataclass(frozen=True)
class FlyLogPidIsolationSnapshot:
    test_axis: str
    isolated_axis: str
    isolated_axes: tuple[str, ...]
    roll_values: AxisPidFf
    pitch_values: AxisPidFf
    yaw_values: AxisPidFf


@dataclass(frozen=True)
class FlyLogPidIsolationResult:
    snapshot: FlyLogPidIsolationSnapshot
    roll_values: AxisPidFf
    pitch_values: AxisPidFf


def _zero_axis_values() -> AxisPidFf:
    return AxisPidFf(p=0.0, i=0.0, d=0.0, ff=0.0)


def _axis_values_from_pair(axis: str, roll_values: AxisPidFf, pitch_values: AxisPidFf) -> AxisPidFf:
    return roll_values if axis == "roll" else pitch_values


def _write_axis_values(fc_service: Any, axis: str, values: AxisPidFf, timeout_seconds: float = 1.2) -> None:
    axis_name = axis.strip().lower()
    series = {
        "p": values.p,
        "i": values.i,
        "d": values.d,
        "ff": values.ff,
    }
    for gain, raw_value in series.items():
        value = int(round(float(raw_value)))
        setting_name = FF_SETTING_NAME[axis_name] if gain == "ff" else PID_SETTING_NAME[(axis_name, gain)]
        confirmed = int(fc_service.set_setting_int(setting_name, value, timeout_seconds=timeout_seconds))
        if confirmed != value:
            raise RuntimeError(
                f"{axis_name.title()} {gain.upper()} write verified as {confirmed}, expected {value}."
            )


def _verify_axis_values(actual: AxisPidFf, expected: AxisPidFf, axis: str) -> None:
    for gain in ("p", "i", "d", "ff"):
        actual_value = int(round(float(getattr(actual, gain))))
        expected_value = int(round(float(getattr(expected, gain))))
        if actual_value != expected_value:
            raise RuntimeError(
                f"{axis.title()} {gain.upper()} verified as {actual_value}, expected {expected_value}."
            )


def _normalized_test_axis(test_axis: str) -> str:
    axis = str(test_axis).strip().lower()
    if axis not in {"roll", "pitch"}:
        raise RuntimeError(f"Unsupported Fly/Log axis '{test_axis}'.")
    return axis


def _read_axis_values(fc_service: Any, axis: str, timeout_seconds: float = 1.2) -> AxisPidFf:
    axis_name = axis.strip().lower()
    return AxisPidFf(
        p=float(fc_service.get_setting_int(PID_SETTING_NAME[(axis_name, "p")], timeout_seconds)),
        i=float(fc_service.get_setting_int(PID_SETTING_NAME[(axis_name, "i")], timeout_seconds)),
        d=float(fc_service.get_setting_int(PID_SETTING_NAME[(axis_name, "d")], timeout_seconds)),
        ff=float(fc_service.get_setting_int(FF_SETTING_NAME[axis_name], timeout_seconds)),
    )


def prepare_fly_log_pid_isolation(fc_service: Any, test_axis: str) -> FlyLogPidIsolationResult:
    axis = _normalized_test_axis(test_axis)
    isolated_axis = "pitch" if axis == "roll" else "roll"
    isolated_axes = (isolated_axis, "yaw")
    roll_values, pitch_values = fc_service.read_roll_pitch_pid_ff(timeout_seconds=1.2)
    yaw_values = _read_axis_values(fc_service, "yaw")
    snapshot = FlyLogPidIsolationSnapshot(
        test_axis=axis,
        isolated_axis=isolated_axis,
        isolated_axes=isolated_axes,
        roll_values=roll_values,
        pitch_values=pitch_values,
        yaw_values=yaw_values,
    )
    zero_values = _zero_axis_values()
    for axis_to_zero in isolated_axes:
        _write_axis_values(fc_service, axis_to_zero, zero_values)
    fc_service.save_settings(timeout_seconds=1.5)

    confirmed_roll, confirmed_pitch = fc_service.read_roll_pitch_pid_ff(timeout_seconds=1.2)
    confirmed_yaw = _read_axis_values(fc_service, "yaw")
    _verify_axis_values(
        _axis_values_from_pair(isolated_axis, confirmed_roll, confirmed_pitch),
        zero_values,
        isolated_axis,
    )
    _verify_axis_values(confirmed_yaw, zero_values, "yaw")
    return FlyLogPidIsolationResult(
        snapshot=snapshot,
        roll_values=confirmed_roll,
        pitch_values=confirmed_pitch,
    )


def restore_fly_log_pid_isolation(
    fc_service: Any,
    snapshot: FlyLogPidIsolationSnapshot,
) -> FlyLogPidIsolationResult:
    _write_axis_values(fc_service, "roll", snapshot.roll_values)
    _write_axis_values(fc_service, "pitch", snapshot.pitch_values)
    _write_axis_values(fc_service, "yaw", snapshot.yaw_values)
    fc_service.save_settings(timeout_seconds=1.5)

    confirmed_roll, confirmed_pitch = fc_service.read_roll_pitch_pid_ff(timeout_seconds=1.2)
    confirmed_yaw = _read_axis_values(fc_service, "yaw")
    _verify_axis_values(confirmed_roll, snapshot.roll_values, "roll")
    _verify_axis_values(confirmed_pitch, snapshot.pitch_values, "pitch")
    _verify_axis_values(confirmed_yaw, snapshot.yaw_values, "yaw")
    return FlyLogPidIsolationResult(
        snapshot=snapshot,
        roll_values=confirmed_roll,
        pitch_values=confirmed_pitch,
    )

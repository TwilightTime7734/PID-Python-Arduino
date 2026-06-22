"""Deterministic Fly/Log sample extraction for decoded INAV Blackbox CSV logs.

The current Fly/Log test routine sends repeatable RC pulses:

    selected axis + - -> optional slow center adjust -> repeat

This module finds the configured main pulses from decoded Blackbox CSV files,
ignores the smaller center-adjust pulses for PID sample metrics, and writes a
compact CSV/JSON/text summary that can be used before deeper PIDtoolbox-style
analysis. The CH8 marker window defines the analysis range; no fixed pulse
count is required.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import median
from typing import Iterable

import numpy as np

from .blackbox_trace_viewer import _pick_column, detect_time_column, load_blackbox_csv


MAIN_PULSE_THRESHOLD_US = 60.0
MIN_MAIN_PULSE_S = 0.18
PULSES_PER_GROUP = 2
EXPECTED_SIGNS = (1, -1)

# The slow-center routine uses much smaller and shorter nudges than the main
# test pulses. These are reported, but never counted as PID response samples.
CENTER_PULSE_MIN_DELTA_US = 25.0
CENTER_PULSE_MAX_DELTA_US = MAIN_PULSE_THRESHOLD_US
CENTER_PULSE_MIN_S = 0.025
CENTER_PULSE_MAX_S = 0.18

# Quality bands are intentionally broad. They are for deciding whether a group
# is worth feeding into the response analyzer, not for making PID changes yet.
MIN_USEFUL_RESPONSE_DEG = 8.0
MAX_SAFE_RESPONSE_DEG = 42.0
MIN_EXPECTED_DURATION_S = 0.18
MAX_EXPECTED_DURATION_S = 0.44


@dataclass(frozen=True)
class _MarkerWindow:
    column: str
    start_index: int | None
    end_index: int | None
    start_s: float | None
    end_s: float | None
    warning: str


@dataclass(frozen=True)
class FlyLogPulseMetric:
    log_label: str
    group: int
    axis: str
    pulse_in_axis: int
    pulse_in_log: int
    expected_axis: str
    expected_direction: str
    actual_direction: str
    sequence_ok: bool
    start_s: float
    end_s: float
    duration_s: float
    center_us: float
    command_us: float
    delta_us: float
    baseline_deg: float | None
    end_deg: float | None
    min_deg: float | None
    max_deg: float | None
    response_deg: float | None
    peak_abs_gyro_dps: float | None
    peak_abs_setpoint_dps: float | None
    quality: str
    warning: str


@dataclass(frozen=True)
class FlyLogCenterAdjustMetric:
    log_label: str
    group: int
    axis: str
    start_s: float
    end_s: float
    duration_s: float
    command_us: float
    delta_us: float


@dataclass(frozen=True)
class FlyLogGroupMetric:
    log_label: str
    group: int
    pitch_pulses: int
    roll_pulses: int
    pitch_sequence_ok: bool
    roll_sequence_ok: bool
    center_adjust_pulses: int
    pitch_median_duration_s: float | None
    roll_median_duration_s: float | None
    pitch_median_response_deg: float | None
    roll_median_response_deg: float | None
    quality: str
    warnings: str


@dataclass(frozen=True)
class FlyLogLogSummary:
    log_label: str
    source_csv: str
    marker_column: str
    marker_start_s: float | None
    marker_end_s: float | None
    pulse_count: int
    complete_groups: int
    usable_groups: int
    clean_groups: int
    center_adjust_pulses: int
    center_adjusted_groups: int
    pitch_pulses: int
    roll_pulses: int
    median_pitch_duration_s: float | None
    median_roll_duration_s: float | None
    median_pitch_response_deg: float | None
    median_roll_response_deg: float | None
    warnings: tuple[str, ...]


@dataclass(frozen=True)
class FlyLogSampleReport:
    summary_txt: str
    samples_csv: str
    groups_csv: str
    center_adjust_csv: str
    samples_json: str
    logs: tuple[FlyLogLogSummary, ...]


def generate_flylog_sample_report(
    prepared_logs: Iterable[tuple[Path, Path, str]],
    report_dir: str | Path,
) -> FlyLogSampleReport:
    """Analyze prepared Step Response logs and write deterministic Fly/Log sample files.

    Parameters
    ----------
    prepared_logs:
        Iterable of ``(source_path, decoded_csv_path, label)`` tuples. This is
        the same shape used by ``step_response_report`` after raw-log decoding.
    report_dir:
        Existing report directory where the output files should be written.
    """

    output_dir = Path(report_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    all_metrics: list[FlyLogPulseMetric] = []
    all_groups: list[FlyLogGroupMetric] = []
    all_center_adjusts: list[FlyLogCenterAdjustMetric] = []
    summaries: list[FlyLogLogSummary] = []

    for _source, csv_path, label in prepared_logs:
        metrics, groups, center_adjusts, summary = analyze_flylog_csv(csv_path, label=label)
        all_metrics.extend(metrics)
        all_groups.extend(groups)
        all_center_adjusts.extend(center_adjusts)
        summaries.append(summary)

    samples_csv = output_dir / "flylog_samples.csv"
    groups_csv = output_dir / "flylog_groups.csv"
    center_adjust_csv = output_dir / "flylog_center_adjustments.csv"
    samples_json = output_dir / "flylog_samples.json"
    summary_txt = output_dir / "flylog_samples_summary.txt"

    _write_dataclass_csv(samples_csv, FlyLogPulseMetric, all_metrics)
    _write_dataclass_csv(groups_csv, FlyLogGroupMetric, all_groups)
    _write_dataclass_csv(center_adjust_csv, FlyLogCenterAdjustMetric, all_center_adjusts)
    samples_json.write_text(
        json.dumps(
            {
                "logs": [asdict(item) for item in summaries],
                "groups": [asdict(item) for item in all_groups],
                "pulses": [asdict(item) for item in all_metrics],
                "center_adjustments": [asdict(item) for item in all_center_adjusts],
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    summary_txt.write_text(
        _format_summary_text(summaries, all_groups, all_metrics, all_center_adjusts),
        encoding="utf-8",
    )

    return FlyLogSampleReport(
        summary_txt=str(summary_txt),
        samples_csv=str(samples_csv),
        groups_csv=str(groups_csv),
        center_adjust_csv=str(center_adjust_csv),
        samples_json=str(samples_json),
        logs=tuple(summaries),
    )


def analyze_flylog_csv(
    csv_path: str | Path,
    *,
    label: str | None = None,
) -> tuple[list[FlyLogPulseMetric], list[FlyLogGroupMetric], list[FlyLogCenterAdjustMetric], FlyLogLogSummary]:
    path = Path(csv_path)
    log_label = label or path.stem
    warnings: list[str] = []
    try:
        columns = load_blackbox_csv(path)
    except Exception as exc:
        summary = _empty_summary(log_label, path, warnings=(f"Could not load CSV: {exc}",))
        return [], [], [], summary

    time_col = detect_time_column(columns)
    if time_col is None:
        summary = _empty_summary(log_label, path, warnings=("No time column found; Fly/Log pulse extraction skipped.",))
        return [], [], [], summary

    time_s = _time_seconds(columns[time_col], time_col)
    marker = _detect_marker_window(path, time_s)
    if marker.warning:
        warnings.append(marker.warning)

    roll_rc_col = _pick_column(columns, ["rcData[0]", "rcCommand[0]"])
    pitch_rc_col = _pick_column(columns, ["rcData[1]", "rcCommand[1]"])
    if roll_rc_col is None or pitch_rc_col is None:
        summary = _empty_summary(
            log_label,
            path,
            marker=marker,
            warnings=tuple(warnings + ["Missing roll/pitch rcData columns; Fly/Log pulse extraction skipped."]),
        )
        return [], [], [], summary

    roll_center = _finite_median(columns[roll_rc_col]) or 1500.0
    pitch_center = _finite_median(columns[pitch_rc_col]) or 1500.0

    raw_pulses: list[dict[str, object]] = []
    raw_pulses.extend(
        _detect_axis_pulses(
            "roll",
            columns[roll_rc_col],
            roll_center,
            time_s,
            marker,
            min_delta_us=MAIN_PULSE_THRESHOLD_US,
            max_delta_us=None,
            min_duration_s=MIN_MAIN_PULSE_S,
            max_duration_s=None,
        )
    )
    raw_pulses.extend(
        _detect_axis_pulses(
            "pitch",
            columns[pitch_rc_col],
            pitch_center,
            time_s,
            marker,
            min_delta_us=MAIN_PULSE_THRESHOLD_US,
            max_delta_us=None,
            min_duration_s=MIN_MAIN_PULSE_S,
            max_duration_s=None,
        )
    )
    raw_pulses.sort(key=lambda item: float(item["start_s"]))

    raw_center_adjusts: list[dict[str, object]] = []
    raw_center_adjusts.extend(
        _detect_axis_pulses(
            "roll",
            columns[roll_rc_col],
            roll_center,
            time_s,
            marker,
            min_delta_us=CENTER_PULSE_MIN_DELTA_US,
            max_delta_us=CENTER_PULSE_MAX_DELTA_US,
            min_duration_s=CENTER_PULSE_MIN_S,
            max_duration_s=CENTER_PULSE_MAX_S,
        )
    )
    raw_center_adjusts.extend(
        _detect_axis_pulses(
            "pitch",
            columns[pitch_rc_col],
            pitch_center,
            time_s,
            marker,
            min_delta_us=CENTER_PULSE_MIN_DELTA_US,
            max_delta_us=CENTER_PULSE_MAX_DELTA_US,
            min_duration_s=CENTER_PULSE_MIN_S,
            max_duration_s=CENTER_PULSE_MAX_S,
        )
    )
    raw_center_adjusts.sort(key=lambda item: float(item["start_s"]))

    metrics: list[FlyLogPulseMetric] = []
    for pulse_index, pulse in enumerate(raw_pulses, start=1):
        group_index = ((pulse_index - 1) // PULSES_PER_GROUP) + 1
        pos_in_group = (pulse_index - 1) % PULSES_PER_GROUP
        group_start = ((group_index - 1) * PULSES_PER_GROUP)
        group_axis = str(raw_pulses[group_start]["axis"]) if group_start < len(raw_pulses) else str(pulse["axis"])
        expected_sign = EXPECTED_SIGNS[pos_in_group]
        actual_axis = str(pulse["axis"])
        actual_sign = int(pulse["sign"])
        next_start_s = None
        if pulse_index < len(raw_pulses):
            next_start_s = float(raw_pulses[pulse_index]["start_s"])
        metrics.append(
            _metric_for_pulse(
                log_label=log_label,
                columns=columns,
                time_s=time_s,
                pulse=pulse,
                pulse_index=pulse_index,
                group_index=group_index,
                pulse_in_axis=pos_in_group + 1,
                expected_axis=group_axis,
                expected_sign=expected_sign,
                sequence_ok=(actual_axis == group_axis and actual_sign == expected_sign),
                next_start_s=next_start_s,
            )
        )

    center_adjusts = [_center_metric_for_pulse(log_label, pulse, raw_pulses) for pulse in raw_center_adjusts]
    groups = _build_group_metrics(log_label, metrics, center_adjusts)

    pitch_metrics = [m for m in metrics if m.axis == "pitch"]
    roll_metrics = [m for m in metrics if m.axis == "roll"]
    complete_groups = sum(1 for group in groups if _group_is_complete(group))
    usable_groups = sum(1 for group in groups if group.quality in {"good", "usable"})
    clean_groups = sum(1 for group in groups if group.quality == "good")
    center_adjusted_groups = len({item.group for item in center_adjusts if item.group > 0})
    if any(not item.sequence_ok for item in metrics):
        warnings.append("One or more pulses did not match the expected one-axis + - pair order.")
    if pitch_metrics and roll_metrics:
        warnings.append("Detected main pulses on both roll and pitch inside the marker window.")

    summary = FlyLogLogSummary(
        log_label=log_label,
        source_csv=str(path),
        marker_column=marker.column,
        marker_start_s=marker.start_s,
        marker_end_s=marker.end_s,
        pulse_count=len(metrics),
        complete_groups=complete_groups,
        usable_groups=usable_groups,
        clean_groups=clean_groups,
        center_adjust_pulses=len(center_adjusts),
        center_adjusted_groups=center_adjusted_groups,
        pitch_pulses=len(pitch_metrics),
        roll_pulses=len(roll_metrics),
        median_pitch_duration_s=_median_or_none([m.duration_s for m in pitch_metrics]),
        median_roll_duration_s=_median_or_none([m.duration_s for m in roll_metrics]),
        median_pitch_response_deg=_median_or_none([abs(m.response_deg) for m in pitch_metrics if m.response_deg is not None]),
        median_roll_response_deg=_median_or_none([abs(m.response_deg) for m in roll_metrics if m.response_deg is not None]),
        warnings=tuple(warnings),
    )
    return metrics, groups, center_adjusts, summary


def _empty_summary(
    label: str,
    path: Path,
    *,
    marker: _MarkerWindow | None = None,
    warnings: tuple[str, ...],
) -> FlyLogLogSummary:
    marker = marker or _MarkerWindow("", None, None, None, None, "")
    return FlyLogLogSummary(
        log_label=label,
        source_csv=str(path),
        marker_column=marker.column,
        marker_start_s=marker.start_s,
        marker_end_s=marker.end_s,
        pulse_count=0,
        complete_groups=0,
        usable_groups=0,
        clean_groups=0,
        center_adjust_pulses=0,
        center_adjusted_groups=0,
        pitch_pulses=0,
        roll_pulses=0,
        median_pitch_duration_s=None,
        median_roll_duration_s=None,
        median_pitch_response_deg=None,
        median_roll_response_deg=None,
        warnings=warnings,
    )


def _detect_axis_pulses(
    axis: str,
    rc_values: np.ndarray,
    center_us: float,
    time_s: np.ndarray,
    marker: _MarkerWindow,
    *,
    min_delta_us: float,
    max_delta_us: float | None,
    min_duration_s: float,
    max_duration_s: float | None,
) -> list[dict[str, object]]:
    values = np.asarray(rc_values, dtype=float)
    n = min(values.size, time_s.size)
    if n <= 1:
        return []
    values = values[:n]
    t = time_s[:n]
    deviation = values - float(center_us)
    abs_dev = np.abs(deviation)
    active = np.isfinite(deviation) & (abs_dev >= float(min_delta_us))
    if max_delta_us is not None:
        active &= abs_dev < float(max_delta_us)
    if marker.start_index is not None and marker.end_index is not None and marker.end_index > marker.start_index:
        window = np.zeros(n, dtype=bool)
        start = max(0, min(int(marker.start_index), n))
        end = max(start, min(int(marker.end_index), n))
        window[start:end] = True
        active &= window
    indices = np.flatnonzero(active)
    if indices.size == 0:
        return []

    pulses: list[dict[str, object]] = []
    start = int(indices[0])
    prev = int(indices[0])
    for index in indices[1:]:
        index = int(index)
        if index != prev + 1:
            _append_pulse_if_valid(
                pulses,
                axis,
                start,
                prev + 1,
                values,
                deviation,
                t,
                center_us,
                min_duration_s=min_duration_s,
                max_duration_s=max_duration_s,
            )
            start = index
        prev = index
    _append_pulse_if_valid(
        pulses,
        axis,
        start,
        prev + 1,
        values,
        deviation,
        t,
        center_us,
        min_duration_s=min_duration_s,
        max_duration_s=max_duration_s,
    )
    return pulses


def _append_pulse_if_valid(
    pulses: list[dict[str, object]],
    axis: str,
    start: int,
    end: int,
    values: np.ndarray,
    deviation: np.ndarray,
    time_s: np.ndarray,
    center_us: float,
    *,
    min_duration_s: float,
    max_duration_s: float | None,
) -> None:
    if end <= start or start >= time_s.size:
        return
    end = min(end, time_s.size)
    duration = float(time_s[end - 1] - time_s[start])
    if duration < float(min_duration_s):
        return
    if max_duration_s is not None and duration > float(max_duration_s):
        return
    seg_dev = deviation[start:end]
    seg_values = values[start:end]
    median_dev = _finite_median(seg_dev)
    median_cmd = _finite_median(seg_values)
    if median_dev is None or median_cmd is None:
        return
    sign = 1 if median_dev >= 0 else -1
    pulses.append(
        {
            "axis": axis,
            "start_index": start,
            "end_index": end,
            "start_s": float(time_s[start]),
            "end_s": float(time_s[end - 1]),
            "duration_s": duration,
            "center_us": float(center_us),
            "command_us": float(median_cmd),
            "delta_us": float(median_dev),
            "sign": sign,
        }
    )


def _metric_for_pulse(
    *,
    log_label: str,
    columns: dict[str, np.ndarray],
    time_s: np.ndarray,
    pulse: dict[str, object],
    pulse_index: int,
    group_index: int,
    pulse_in_axis: int,
    expected_axis: str,
    expected_sign: int,
    sequence_ok: bool,
    next_start_s: float | None,
) -> FlyLogPulseMetric:
    axis = str(pulse["axis"])
    sign = int(pulse["sign"])
    start_s = float(pulse["start_s"])
    end_s = float(pulse["end_s"])
    response_end_s = min(end_s + 0.45, next_start_s - 0.02 if next_start_s is not None else end_s + 0.45)
    response_end_s = max(end_s, response_end_s)

    attitude_col = _pick_column(columns, ["attitude[0]"]) if axis == "roll" else _pick_column(columns, ["attitude[1]"])
    gyro_col = _pick_column(columns, ["gyroADC[0]"]) if axis == "roll" else _pick_column(columns, ["gyroADC[1]"])
    setpoint_col = _pick_column(columns, ["axisRate[0]", "setpoint[0]"]) if axis == "roll" else _pick_column(columns, ["axisRate[1]", "setpoint[1]"])

    baseline_deg = end_deg = min_deg = max_deg = response_deg = None
    if attitude_col is not None:
        attitude_deg = np.asarray(columns[attitude_col], dtype=float) / 10.0
        baseline_mask = (time_s >= start_s - 0.12) & (time_s < start_s)
        window_mask = (time_s >= start_s) & (time_s <= response_end_s)
        baseline_deg = _finite_median(attitude_deg[baseline_mask])
        end_deg = _finite_median(attitude_deg[(time_s >= max(start_s, response_end_s - 0.06)) & (time_s <= response_end_s)])
        if np.any(window_mask):
            win = attitude_deg[window_mask]
            min_deg = _finite_min(win)
            max_deg = _finite_max(win)
            if baseline_deg is not None:
                if sign >= 0 and max_deg is not None:
                    response_deg = float(max_deg - baseline_deg)
                elif sign < 0 and min_deg is not None:
                    response_deg = float(baseline_deg - min_deg)

    peak_abs_gyro_dps = _peak_abs_in_window(columns.get(gyro_col, np.asarray([], dtype=float)), time_s, start_s, response_end_s) if gyro_col else None
    peak_abs_setpoint_dps = (
        _peak_abs_in_window(columns.get(setpoint_col, np.asarray([], dtype=float)), time_s, start_s, end_s) if setpoint_col else None
    )
    quality, warning = _pulse_quality(sequence_ok, float(pulse["duration_s"]), response_deg)

    return FlyLogPulseMetric(
        log_label=log_label,
        group=group_index,
        axis=axis,
        pulse_in_axis=pulse_in_axis,
        pulse_in_log=pulse_index,
        expected_axis=expected_axis,
        expected_direction="+" if expected_sign >= 0 else "-",
        actual_direction="+" if sign >= 0 else "-",
        sequence_ok=sequence_ok,
        start_s=start_s,
        end_s=end_s,
        duration_s=float(pulse["duration_s"]),
        center_us=float(pulse["center_us"]),
        command_us=float(pulse["command_us"]),
        delta_us=float(pulse["delta_us"]),
        baseline_deg=baseline_deg,
        end_deg=end_deg,
        min_deg=min_deg,
        max_deg=max_deg,
        response_deg=response_deg,
        peak_abs_gyro_dps=peak_abs_gyro_dps,
        peak_abs_setpoint_dps=peak_abs_setpoint_dps,
        quality=quality,
        warning=warning,
    )


def _pulse_quality(sequence_ok: bool, duration_s: float, response_deg: float | None) -> tuple[str, str]:
    warnings: list[str] = []
    if not sequence_ok:
        warnings.append("sequence mismatch")
    if duration_s < MIN_EXPECTED_DURATION_S or duration_s > MAX_EXPECTED_DURATION_S:
        warnings.append(f"duration {duration_s:.3f}s outside expected range")
    if response_deg is None:
        warnings.append("no attitude response measurement")
    elif abs(response_deg) < MIN_USEFUL_RESPONSE_DEG:
        warnings.append(f"small response {abs(response_deg):.1f} deg")
    elif abs(response_deg) > MAX_SAFE_RESPONSE_DEG:
        warnings.append(f"large response {abs(response_deg):.1f} deg")
    if not sequence_ok:
        return "bad", "; ".join(warnings)
    if warnings:
        return "usable", "; ".join(warnings)
    return "good", ""


def _center_metric_for_pulse(
    log_label: str,
    pulse: dict[str, object],
    main_pulses: list[dict[str, object]],
) -> FlyLogCenterAdjustMetric:
    group = _nearest_group_for_time(float(pulse["start_s"]), main_pulses)
    return FlyLogCenterAdjustMetric(
        log_label=log_label,
        group=group,
        axis=str(pulse["axis"]),
        start_s=float(pulse["start_s"]),
        end_s=float(pulse["end_s"]),
        duration_s=float(pulse["duration_s"]),
        command_us=float(pulse["command_us"]),
        delta_us=float(pulse["delta_us"]),
    )


def _nearest_group_for_time(start_s: float, main_pulses: list[dict[str, object]]) -> int:
    if not main_pulses:
        return 0
    previous_index = 0
    for index, pulse in enumerate(main_pulses, start=1):
        if float(pulse["start_s"]) > start_s:
            break
        previous_index = index
    if previous_index <= 0:
        return 1
    return ((previous_index - 1) // PULSES_PER_GROUP) + 1


def _build_group_metrics(
    log_label: str,
    metrics: list[FlyLogPulseMetric],
    center_adjusts: list[FlyLogCenterAdjustMetric],
) -> list[FlyLogGroupMetric]:
    groups: list[FlyLogGroupMetric] = []
    max_group = max([m.group for m in metrics] + [0])
    for group_index in range(1, max_group + 1):
        group_pulses = [m for m in metrics if m.group == group_index]
        pitch = [m for m in group_pulses if m.axis == "pitch"]
        roll = [m for m in group_pulses if m.axis == "roll"]
        center_count = sum(1 for m in center_adjusts if m.group == group_index)
        pitch_sequence_ok = _axis_sequence_ok(pitch) if pitch else True
        roll_sequence_ok = _axis_sequence_ok(roll) if roll else True
        active_axis = _group_active_axis(pitch, roll)
        group_complete = (
            active_axis is not None
            and len(group_pulses) == PULSES_PER_GROUP
            and ((active_axis == "pitch" and pitch_sequence_ok) or (active_axis == "roll" and roll_sequence_ok))
        )
        warnings: list[str] = []
        if len(group_pulses) != PULSES_PER_GROUP:
            warnings.append(f"pulse count {len(group_pulses)}/{PULSES_PER_GROUP}")
        if pitch and roll:
            warnings.append("mixed roll/pitch pulses in one pair")
        if pitch and not pitch_sequence_ok:
            warnings.append("pitch sequence not + -")
        if roll and not roll_sequence_ok:
            warnings.append("roll sequence not + -")
        for axis_name, pulses in (("pitch", pitch), ("roll", roll)):
            if not pulses:
                continue
            response = _median_or_none([abs(p.response_deg) for p in pulses if p.response_deg is not None])
            if response is None:
                warnings.append(f"{axis_name} response unavailable")
            elif response < MIN_USEFUL_RESPONSE_DEG:
                warnings.append(f"{axis_name} response small ({response:.1f} deg)")
            elif response > MAX_SAFE_RESPONSE_DEG:
                warnings.append(f"{axis_name} response large ({response:.1f} deg)")
        if center_count:
            warnings.append(f"center adjust pulses {center_count}")

        if not group_complete:
            quality = "bad"
        elif any("response large" in item for item in warnings):
            quality = "usable"
        elif warnings:
            quality = "usable"
        else:
            quality = "good"

        groups.append(
            FlyLogGroupMetric(
                log_label=log_label,
                group=group_index,
                pitch_pulses=len(pitch),
                roll_pulses=len(roll),
                pitch_sequence_ok=pitch_sequence_ok,
                roll_sequence_ok=roll_sequence_ok,
                center_adjust_pulses=center_count,
                pitch_median_duration_s=_median_or_none([m.duration_s for m in pitch]),
                roll_median_duration_s=_median_or_none([m.duration_s for m in roll]),
                pitch_median_response_deg=_median_or_none([abs(m.response_deg) for m in pitch if m.response_deg is not None]),
                roll_median_response_deg=_median_or_none([abs(m.response_deg) for m in roll if m.response_deg is not None]),
                quality=quality,
                warnings="; ".join(warnings),
            )
        )
    return groups


def _axis_sequence_ok(metrics: list[FlyLogPulseMetric]) -> bool:
    if len(metrics) != PULSES_PER_GROUP:
        return False
    signs = tuple(1 if m.actual_direction == "+" else -1 for m in sorted(metrics, key=lambda item: item.pulse_in_axis))
    return signs == EXPECTED_SIGNS


def _group_active_axis(pitch: list[FlyLogPulseMetric], roll: list[FlyLogPulseMetric]) -> str | None:
    if len(pitch) == PULSES_PER_GROUP and not roll:
        return "pitch"
    if len(roll) == PULSES_PER_GROUP and not pitch:
        return "roll"
    return None


def _group_is_complete(group: FlyLogGroupMetric) -> bool:
    return (
        (group.pitch_pulses == PULSES_PER_GROUP and group.roll_pulses == 0 and group.pitch_sequence_ok)
        or (group.roll_pulses == PULSES_PER_GROUP and group.pitch_pulses == 0 and group.roll_sequence_ok)
    )


def _detect_marker_window(csv_path: Path, time_s: np.ndarray) -> _MarkerWindow:
    try:
        result = _load_csv_text_column(csv_path, ["flightModeFlags (flags)", "flightModeFlags", "activeFlightModeFlags"])
    except Exception as exc:
        return _MarkerWindow("", None, None, None, None, f"Could not read beeper marker column: {exc}")
    if result is None:
        return _MarkerWindow("", None, None, None, None, "No beeper marker column found; using entire CSV.")
    column_name, values = result
    beeper_indices = [index for index, value in enumerate(values) if _flight_mode_has_beeper_on(value)]
    if not beeper_indices:
        return _MarkerWindow(column_name, None, None, None, None, "No BEEPERON marker found; using entire CSV.")
    start = beeper_indices[0]
    end = beeper_indices[-1] + 1
    return _MarkerWindow(
        column=column_name,
        start_index=start,
        end_index=end,
        start_s=_time_value(time_s, start),
        end_s=_time_value(time_s, end - 1),
        warning="",
    )


def _load_csv_text_column(csv_path: Path, candidates: list[str]) -> tuple[str, list[str]] | None:
    with csv_path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        reader = csv.reader(handle, skipinitialspace=True)
        header: list[str] | None = None
        column_index = -1
        for row in reader:
            if not row:
                continue
            normalized = {_normalize_header(name): index for index, name in enumerate(row)}
            for candidate in candidates:
                index = normalized.get(_normalize_header(candidate))
                if index is not None:
                    header = row
                    column_index = index
                    break
            if header is not None:
                break
        if header is None or column_index < 0:
            return None
        values: list[str] = []
        for row in reader:
            values.append(row[column_index].strip() if column_index < len(row) else "")
        return header[column_index], values


def _normalize_header(header: str) -> str:
    return "".join(ch for ch in str(header).strip().lower() if ch not in " \t\r\n_()-")


def _flight_mode_has_beeper_on(value: str) -> bool:
    tokens = [token.strip().upper() for token in str(value or "").split("|")]
    return "BEEPERON" in tokens


def _time_value(time_s: np.ndarray, index: int) -> float | None:
    if time_s.size == 0:
        return None
    index = max(0, min(int(index), time_s.size - 1))
    value = float(time_s[index])
    return value if math.isfinite(value) else None


def _time_seconds(raw: np.ndarray, time_col: str) -> np.ndarray:
    values = np.asarray(raw, dtype=float)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return np.arange(values.size, dtype=float)
    values = values - float(np.nanmin(finite))
    span = float(np.nanmax(values)) if values.size else 0.0
    normalized = "".join(ch for ch in str(time_col).lower() if ch.isalnum())
    if "us" in normalized or span > 1_000_000.0:
        return values / 1_000_000.0
    if span > 10_000.0:
        return values / 1000.0
    return values


def _finite_values(values: np.ndarray | list[float]) -> list[float]:
    arr = np.asarray(values, dtype=float).ravel()
    return [float(v) for v in arr if math.isfinite(float(v))]


def _finite_median(values: np.ndarray | list[float]) -> float | None:
    finite = _finite_values(values)
    if not finite:
        return None
    return float(median(finite))


def _finite_min(values: np.ndarray | list[float]) -> float | None:
    finite = _finite_values(values)
    if not finite:
        return None
    return float(min(finite))


def _finite_max(values: np.ndarray | list[float]) -> float | None:
    finite = _finite_values(values)
    if not finite:
        return None
    return float(max(finite))


def _median_or_none(values: Iterable[float | None]) -> float | None:
    finite = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not finite:
        return None
    return float(median(finite))


def _peak_abs_in_window(values: np.ndarray, time_s: np.ndarray, start_s: float, end_s: float) -> float | None:
    arr = np.asarray(values, dtype=float)
    n = min(arr.size, time_s.size)
    if n <= 0:
        return None
    mask = (time_s[:n] >= start_s) & (time_s[:n] <= end_s)
    finite = _finite_values(np.abs(arr[:n][mask]))
    if not finite:
        return None
    return float(max(finite))


def _write_dataclass_csv(path: Path, model: type, rows: list[object]) -> None:
    fieldnames = list(model.__dataclass_fields__.keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def _format_summary_text(
    summaries: list[FlyLogLogSummary],
    groups: list[FlyLogGroupMetric],
    metrics: list[FlyLogPulseMetric],
    center_adjusts: list[FlyLogCenterAdjustMetric],
) -> str:
    lines: list[str] = [
        "Deterministic Fly/Log sample extraction",
        "Finds the main configured one-axis pulses inside the CH8 marker window and ignores smaller center-adjust pulses.",
        "The CH8 marker window defines the analysis range; no fixed pulse count is required.",
        "Quality terms: good = complete/no warnings, usable = complete with warnings, bad = missing or wrong sequence.",
        "",
    ]
    if not summaries:
        lines.append("No logs were analyzed.")
        return "\n".join(lines).strip()

    for summary in summaries:
        lines.append(summary.log_label)
        lines.append(f"- decoded CSV: {summary.source_csv}")
        if summary.marker_column and summary.marker_start_s is not None and summary.marker_end_s is not None:
            lines.append(
                f"- marker window: {summary.marker_column}, {summary.marker_start_s:.2f}s to {summary.marker_end_s:.2f}s"
            )
        log_groups = [group for group in groups if group.log_label == summary.log_label]
        lines.append(
            f"- pulse pairs: {summary.complete_groups}/{len(log_groups)} complete, "
            f"{summary.usable_groups} usable, {summary.clean_groups} clean"
        )
        lines.append(
            f"- main pulses: {summary.pulse_count} total, {summary.pitch_pulses} pitch, {summary.roll_pulses} roll"
        )
        lines.append(
            f"- center adjustments: {summary.center_adjust_pulses} pulses across {summary.center_adjusted_groups} group(s)"
        )
        lines.append(
            "- median duration: "
            f"pitch={_fmt(summary.median_pitch_duration_s, 's')}, "
            f"roll={_fmt(summary.median_roll_duration_s, 's')}"
        )
        lines.append(
            "- median response: "
            f"pitch={_fmt(summary.median_pitch_response_deg, 'deg')}, "
            f"roll={_fmt(summary.median_roll_response_deg, 'deg')}"
        )
        for warning in summary.warnings:
            lines.append(f"- warning: {warning}")

        if log_groups:
            lines.append("- group quality:")
            for group in log_groups:
                extra = f" ({group.warnings})" if group.warnings else ""
                lines.append(
                    f"  group {group.group}: {group.quality}, "
                    f"pitch={_fmt(group.pitch_median_response_deg, 'deg')}, "
                    f"roll={_fmt(group.roll_median_response_deg, 'deg')}, "
                    f"center={group.center_adjust_pulses}{extra}"
                )
        lines.append("")

    bad_sequence = [item for item in metrics if not item.sequence_ok]
    if bad_sequence:
        lines.append(f"Sequence warnings: {len(bad_sequence)} pulse(s) did not match expected one-axis + - order.")
        lines.append("")

    lines.append("Files written:")
    lines.append("- flylog_samples.csv: one row per detected main pulse")
    lines.append("- flylog_groups.csv: one row per detected +,- pulse pair")
    lines.append("- flylog_center_adjustments.csv: one row per small center nudge")
    lines.append("- flylog_samples.json: machine-readable version of all Fly/Log sample data")
    lines.append("- flylog_samples_summary.txt: this summary")
    return "\n".join(lines).strip()


def _fmt(value: float | None, suffix: str) -> str:
    if value is None or not math.isfinite(float(value)):
        return "n/a"
    if suffix == "s":
        return f"{float(value):.3f}s"
    if suffix == "deg":
        return f"{float(value):.1f}°"
    return f"{float(value):.3g}{suffix}"

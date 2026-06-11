"""PIDtoolbox-style step-response report generation for Blackbox logs."""

from __future__ import annotations

import csv
import json
import math
import shutil
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from .blackbox_import import CSV_EXTENSIONS, RAW_BLACKBOX_EXTENSIONS, REPO_ROOT
from .blackbox_trace_viewer import _pick_column, detect_time_column, load_blackbox_csv
from .pidtoolbox_step_response import StepResponseResult, compute_pidtoolbox_step_response
from .flylog_sample_report import FlyLogSampleReport, generate_flylog_sample_report


MAX_STEP_RESPONSE_LOGS = 6
STEP_RESPONSE_COLORS = (
    "#8b0000",
    "#2563eb",
    "#16a34a",
    "#7c3aed",
    "#ea580c",
    "#0891b2",
)
STEP_RESPONSE_SETTINGS: dict[str, object] = {
    "smooth_level": 0,
    "y_correction": True,
    "segment_seconds": 2.0,
    "response_ms": 500.0,
    "steady_start_ms": 200.0,
    "peak_window_ms": 150.0,
    "min_input_deg_s": 20.0,
    "subsample_factor": 9,
}


@dataclass(frozen=True)
class StepResponseAxisMetrics:
    axis: str
    setpoint_column: str
    gyro_column: str
    accepted_traces: int
    sample_rate_hz: float
    peak: float | None
    peak_time_ms: float | None
    latency_half_height_ms: float | None
    error: str = ""


@dataclass(frozen=True)
class StepResponseLogMetrics:
    label: str
    color: str
    source_log: str
    decoded_csv: str
    marker_column: str
    marker_start_index: int | None
    marker_end_index: int | None
    marker_start_s: float | None
    marker_end_s: float | None
    marker_samples: int
    marker_warning: str
    axes: dict[str, StepResponseAxisMetrics]


@dataclass(frozen=True)
class StepResponseReport:
    report_dir: str
    html_path: str
    summary_json: str
    decoded_csv_paths: tuple[str, ...]
    source_paths: tuple[str, ...]
    logs: tuple[StepResponseLogMetrics, ...]
    flylog_sample_report: FlyLogSampleReport | None = None


def generate_step_response_report(log_paths: list[str] | tuple[str, ...], output_root: str | Path) -> StepResponseReport:
    sources = [Path(path).resolve() for path in log_paths]
    if not sources:
        raise RuntimeError("Select at least one Blackbox log.")
    if len(sources) > MAX_STEP_RESPONSE_LOGS:
        raise RuntimeError(f"Select at most {MAX_STEP_RESPONSE_LOGS} Blackbox logs.")
    for source in sources:
        if not source.exists() or not source.is_file():
            raise RuntimeError(f"Log file not found: {source}")
        suffix = source.suffix.lower()
        if suffix not in RAW_BLACKBOX_EXTENSIONS and suffix not in CSV_EXTENSIONS:
            raise RuntimeError(f"Unsupported log type: {source.name}")

    output_root_path = _resolve_report_output_root(output_root)
    report_dir = _next_report_dir(output_root_path / "reports", "step_response")
    report_dir.mkdir(parents=True, exist_ok=True)

    prepared = [_prepare_log_source(source, report_dir, index) for index, source in enumerate(sources, start=1)]
    logs: list[StepResponseLogMetrics] = []
    for index, (source, csv_path, label) in enumerate(prepared):
        color = STEP_RESPONSE_COLORS[(index - 1) % len(STEP_RESPONSE_COLORS)]
        logs.append(_analyze_log(source, csv_path, label, color))

    flylog_sample_report = generate_flylog_sample_report(prepared, report_dir)

    html_path = report_dir / "pidtoolbox_step_response_detail.html"
    summary_json = report_dir / "pidtoolbox_step_response_summary.json"
    _write_step_response_html(logs, html_path)
    summary_json.write_text(_summary_json_payload(report_dir, html_path, summary_json, logs), encoding="utf-8")

    return StepResponseReport(
        report_dir=str(report_dir),
        html_path=str(html_path),
        summary_json=str(summary_json),
        decoded_csv_paths=tuple(log.decoded_csv for log in logs),
        source_paths=tuple(str(source) for source in sources),
        logs=tuple(logs),
        flylog_sample_report=flylog_sample_report,
    )


def format_step_response_report(report: StepResponseReport) -> str:
    lines = [
        "PIDtoolbox Step Response",
        f"Report: {report.html_path}",
        "Analysis uses the CH8 beeper marker window (BEEPERON through BEEPEROFF), not a fixed duration.",
        "",
    ]
    for log in report.logs:
        lines.append(log.label)
        if log.marker_column and log.marker_start_s is not None and log.marker_end_s is not None:
            marker_line = (
                f"- beeper marker: {log.marker_column}, "
                f"{log.marker_start_s:.2f}s to {log.marker_end_s:.2f}s"
            )
            if log.marker_warning:
                marker_line = f"{marker_line} ({log.marker_warning})"
            lines.append(marker_line)
        elif log.marker_warning:
            lines.append(f"- beeper marker: {log.marker_warning}")
        for axis in ("roll", "pitch", "yaw"):
            metrics = log.axes.get(axis)
            if metrics is None:
                continue
            if metrics.error:
                lines.append(f"- {axis}: {metrics.error}")
            elif metrics.accepted_traces <= 0:
                lines.append(f"- {axis}: insufficient data")
            else:
                lines.append(
                    f"- {axis}: n={metrics.accepted_traces}, peak={metrics.peak:.3f}, "
                    f"latency={metrics.latency_half_height_ms:.1f} ms"
                )
        lines.append("")
    if report.flylog_sample_report is not None:
        lines.append("Deterministic Fly/Log samples")
        lines.append(f"- summary: {report.flylog_sample_report.summary_txt}")
        lines.append(f"- samples CSV: {report.flylog_sample_report.samples_csv}")
        lines.append(f"- groups CSV: {report.flylog_sample_report.groups_csv}")
        lines.append(f"- center adjustments CSV: {report.flylog_sample_report.center_adjust_csv}")
        lines.append(f"- samples JSON: {report.flylog_sample_report.samples_json}")
        for sample_log in report.flylog_sample_report.logs:
            lines.append(
                f"- {sample_log.log_label}: groups={sample_log.complete_groups}/6 complete, "
                f"usable={sample_log.usable_groups}/6, clean={sample_log.clean_groups}/6, "
                f"pulses={sample_log.pulse_count}, center nudges={sample_log.center_adjust_pulses}, "
                f"pitch response={_format_optional_deg(sample_log.median_pitch_response_deg)}, "
                f"roll response={_format_optional_deg(sample_log.median_roll_response_deg)}"
            )
            for warning in sample_log.warnings[:3]:
                lines.append(f"  warning: {warning}")
        lines.append("")

    return "\n".join(lines).strip()


def _resolve_report_output_root(output_root: str | Path) -> Path:
    """Return the canonical Blackbox report root.

    Runtime state used to point at ``modbus_app/blackbox_imports``.  Step
    Response reports belong in the project-level ``blackbox_imports`` folder,
    so repair that legacy path here as a guardrail too.
    """
    path = Path(output_root).expanduser()
    if not path.is_absolute():
        path = REPO_ROOT / path
    resolved = path.resolve()
    legacy_nested = (REPO_ROOT / "modbus_app" / "blackbox_imports").resolve()
    canonical = (REPO_ROOT / "blackbox_imports").resolve()
    if resolved == legacy_nested:
        return canonical
    return resolved


def _format_optional_deg(value: float | None) -> str:
    if value is None or not math.isfinite(float(value)):
        return "n/a"
    return f"{float(value):.1f} deg"


def _next_report_dir(parent: Path, prefix: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = parent / f"{prefix}_{timestamp}"
    if not base.exists():
        return base
    for index in range(2, 1000):
        candidate = parent / f"{prefix}_{timestamp}_{index:02d}"
        if not candidate.exists():
            return candidate
    raise RuntimeError("Could not allocate a step-response report folder.")


def _prepare_log_source(source: Path, report_dir: Path, index: int) -> tuple[Path, Path, str]:
    label = source.stem
    local_name = f"{index:02d}_{_safe_filename(source.name)}"
    local_source = report_dir / local_name
    shutil.copy2(source, local_source)

    suffix = local_source.suffix.lower()
    if suffix in CSV_EXTENSIONS:
        return local_source, local_source, label
    if suffix not in RAW_BLACKBOX_EXTENSIONS:
        raise RuntimeError(f"Unsupported log type: {source.name}")

    csv_path = _decode_raw_log(local_source, report_dir)
    return local_source, csv_path, label


def _decode_raw_log(raw_path: Path, report_dir: Path) -> Path:
    decoder = _find_tools_decoder()
    before = {path.resolve() for path in report_dir.glob(f"{raw_path.stem}*.csv")}
    completed = subprocess.run(
        [str(decoder), "--unit-rotation", "deg/s", str(raw_path.resolve())],
        cwd=str(report_dir),
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    after = {path.resolve() for path in report_dir.glob(f"{raw_path.stem}*.csv")}
    new_files = sorted(after - before)
    if new_files:
        return Path(new_files[0])
    if completed.returncode == 0 and after:
        return Path(sorted(after)[0])
    stderr = (completed.stderr or completed.stdout or "").strip().splitlines()
    reason = stderr[-1] if stderr else f"exit code {completed.returncode}"
    raise RuntimeError(f"Decode failed for '{raw_path.name}': {reason}")


def _find_tools_decoder() -> Path:
    candidates = (
        REPO_ROOT / "tools" / "blackbox_decode_INAV.exe",
        REPO_ROOT / "tools" / "blackbox_decode_INAV",
        REPO_ROOT / "tools" / "blackbox_decode.exe",
        REPO_ROOT / "tools" / "blackbox_decode",
    )
    for candidate in candidates:
        if candidate.exists() and candidate.is_file():
            return candidate
    raise RuntimeError("No Blackbox decoder was found in the tools folder.")


@dataclass(frozen=True)
class _MarkerWindow:
    column: str
    start_index: int | None
    end_index: int | None
    start_s: float | None
    end_s: float | None
    samples: int
    warning: str


def _analyze_log(source: Path, csv_path: Path, label: str, color: str) -> StepResponseLogMetrics:
    columns = load_blackbox_csv(csv_path)
    time_col = detect_time_column(columns)
    if time_col is None:
        raise RuntimeError(f"Decoded CSV has no time column: {csv_path}")
    marker = _detect_beeper_marker_window(columns, time_col, csv_path)
    analysis_columns = _slice_columns_for_marker(columns, marker)
    time_us = analysis_columns[time_col]

    axes: dict[str, StepResponseAxisMetrics] = {}
    for axis, axis_index in (("roll", 0), ("pitch", 1), ("yaw", 2)):
        setpoint_col = _pick_column(analysis_columns, [f"axisRate[{axis_index}]", f"setpoint[{axis_index}]"])
        gyro_col = _pick_column(analysis_columns, [f"gyroADC[{axis_index}] (deg/s)", f"gyroADC[{axis_index}]"])
        if setpoint_col is None or gyro_col is None:
            axes[axis] = StepResponseAxisMetrics(
                axis=axis,
                setpoint_column=setpoint_col or "",
                gyro_column=gyro_col or "",
                accepted_traces=0,
                sample_rate_hz=0.0,
                peak=None,
                peak_time_ms=None,
                latency_half_height_ms=None,
                error="missing setpoint or gyro columns",
            )
            continue
        try:
            result = compute_pidtoolbox_step_response(
                analysis_columns[setpoint_col],
                analysis_columns[gyro_col],
                time_us=time_us,
                smooth_level=int(STEP_RESPONSE_SETTINGS["smooth_level"]),
                y_correction=bool(STEP_RESPONSE_SETTINGS["y_correction"]),
                segment_seconds=float(STEP_RESPONSE_SETTINGS["segment_seconds"]),
                response_ms=float(STEP_RESPONSE_SETTINGS["response_ms"]),
                steady_start_ms=float(STEP_RESPONSE_SETTINGS["steady_start_ms"]),
                peak_window_ms=float(STEP_RESPONSE_SETTINGS["peak_window_ms"]),
                min_input_deg_s=float(STEP_RESPONSE_SETTINGS["min_input_deg_s"]),
                subsample_factor=int(STEP_RESPONSE_SETTINGS["subsample_factor"]),
            )
            axes[axis] = _axis_metrics_from_result(axis, setpoint_col, gyro_col, result)
        except Exception as exc:
            axes[axis] = StepResponseAxisMetrics(
                axis=axis,
                setpoint_column=setpoint_col,
                gyro_column=gyro_col,
                accepted_traces=0,
                sample_rate_hz=0.0,
                peak=None,
                peak_time_ms=None,
                latency_half_height_ms=None,
                error=str(exc),
            )

    return StepResponseLogMetrics(
        label=label,
        color=color,
        source_log=str(source),
        decoded_csv=str(csv_path),
        marker_column=marker.column,
        marker_start_index=marker.start_index,
        marker_end_index=marker.end_index,
        marker_start_s=marker.start_s,
        marker_end_s=marker.end_s,
        marker_samples=marker.samples,
        marker_warning=marker.warning,
        axes=axes,
    )


def _detect_beeper_marker_window(columns: dict[str, np.ndarray], time_col: str, csv_path: Path) -> _MarkerWindow:
    flag_marker = _detect_beeper_mode_flag_window(csv_path, columns, time_col)
    if flag_marker is not None:
        return flag_marker

    return _MarkerWindow(
        "",
        None,
        None,
        None,
        None,
        0,
        "CH8 beeper marker window (BEEPERON/BEEPEROFF) not found; full log used",
    )


def _detect_beeper_mode_flag_window(
    csv_path: Path,
    columns: dict[str, np.ndarray],
    time_col: str,
) -> _MarkerWindow | None:
    text_column = _load_csv_text_column(csv_path, ["flightModeFlags (flags)", "flightModeFlags"])
    if text_column is None:
        return None
    marker_col, values = text_column
    high = np.asarray([_flight_mode_has_beeper_on(value) for value in values], dtype=bool)
    if not np.any(high):
        return None

    max_samples = len(columns[time_col])
    high_indices = np.flatnonzero(high)
    start = int(high_indices[0])
    if start >= max_samples:
        return None

    after_start = high[start + 1 :]
    off_after_start = np.flatnonzero(~after_start)
    end = start + 1 + int(off_after_start[0]) if off_after_start.size else len(high)
    end = min(end, max_samples)

    times = np.asarray(columns[time_col], dtype=float)
    start_s = _time_value_s(times, start)
    end_s = _time_value_s(times, max(start, end - 1))
    duration_s = max(0.0, end_s - start_s) if start_s is not None and end_s is not None else 0.0
    return _MarkerWindow(
        marker_col,
        start,
        end,
        start_s,
        end_s,
        int(end - start),
        f"using CH8 beeper marker window ({duration_s:.2f}s, BEEPERON flight mode flag)",
    )


def _flight_mode_has_beeper_on(value: str) -> bool:
    tokens = [token.strip().upper() for token in str(value or "").split("|")]
    return "BEEPERON" in tokens


def _load_csv_text_column(csv_path: Path, candidates: list[str]) -> tuple[str, list[str]] | None:
    with csv_path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        reader = csv.reader(handle, skipinitialspace=True)
        header: list[str] | None = None
        column_name = ""
        column_index = -1
        for row in reader:
            if not row:
                continue
            normalized = {_normalize_text_header(name): index for index, name in enumerate(row)}
            for candidate in candidates:
                index = normalized.get(_normalize_text_header(candidate))
                if index is not None:
                    header = row
                    column_name = row[index]
                    column_index = index
                    break
            if header is not None:
                break

        if header is None or column_index < 0:
            return None

        values: list[str] = []
        for row in reader:
            values.append(row[column_index].strip() if column_index < len(row) else "")
        return column_name, values


def _normalize_text_header(header: str) -> str:
    return "".join(ch for ch in header.strip().lower() if ch not in " \t\r\n_()-")


def _time_value_s(times: np.ndarray, index: int) -> float | None:
    if times.size == 0:
        return None
    index = max(0, min(int(index), times.size - 1))
    value = float(times[index])
    if not math.isfinite(value):
        return None
    return value / 1_000_000.0


def _slice_columns_for_marker(columns: dict[str, np.ndarray], marker: _MarkerWindow) -> dict[str, np.ndarray]:
    if marker.start_index is None or marker.end_index is None or marker.end_index <= marker.start_index:
        return columns
    return {name: values[marker.start_index : marker.end_index] for name, values in columns.items()}


def _axis_metrics_from_result(
    axis: str, setpoint_col: str, gyro_col: str, result: StepResponseResult
) -> StepResponseAxisMetrics:
    return StepResponseAxisMetrics(
        axis=axis,
        setpoint_column=setpoint_col,
        gyro_column=gyro_col,
        accepted_traces=result.n_traces,
        sample_rate_hz=result.sample_rate_hz,
        peak=_finite_or_none(result.peak),
        peak_time_ms=_finite_or_none(result.peak_time_ms),
        latency_half_height_ms=_finite_or_none(result.latency_half_height_ms),
    )


def _result_for_plot(log: StepResponseLogMetrics, axis: str) -> StepResponseResult | None:
    columns = load_blackbox_csv(log.decoded_csv)
    time_col = detect_time_column(columns)
    metrics = log.axes.get(axis)
    if time_col is None or metrics is None or metrics.error or not metrics.setpoint_column or not metrics.gyro_column:
        return None
    if log.marker_start_index is not None and log.marker_end_index is not None:
        columns = {
            name: values[log.marker_start_index : log.marker_end_index]
            for name, values in columns.items()
        }
    return compute_pidtoolbox_step_response(
        columns[metrics.setpoint_column],
        columns[metrics.gyro_column],
        time_us=columns[time_col],
        smooth_level=int(STEP_RESPONSE_SETTINGS["smooth_level"]),
        y_correction=bool(STEP_RESPONSE_SETTINGS["y_correction"]),
        segment_seconds=float(STEP_RESPONSE_SETTINGS["segment_seconds"]),
        response_ms=float(STEP_RESPONSE_SETTINGS["response_ms"]),
        steady_start_ms=float(STEP_RESPONSE_SETTINGS["steady_start_ms"]),
        peak_window_ms=float(STEP_RESPONSE_SETTINGS["peak_window_ms"]),
        min_input_deg_s=float(STEP_RESPONSE_SETTINGS["min_input_deg_s"]),
        subsample_factor=int(STEP_RESPONSE_SETTINGS["subsample_factor"]),
    )


def _write_step_response_html(logs: list[StepResponseLogMetrics], html_path: Path) -> None:
    from plotly.subplots import make_subplots
    import plotly.graph_objects as go

    fig = make_subplots(
        rows=3,
        cols=1,
        vertical_spacing=0.085,
        subplot_titles=[
            "Roll Response",
            "Pitch Response",
            "Yaw Response",
        ],
    )
    axis_rows = (("roll", 1), ("pitch", 2), ("yaw", 3))
    for axis, row in axis_rows:
        fig.add_hline(y=1.0, line_dash="dash", line_width=3.0, line_color="#000000", row=row, col=1)
        fig.add_hline(y=0.5, line_dash="dot", line_width=1, line_color="#9ca3af", row=row, col=1)

        for log in logs:
            metrics = log.axes.get(axis)
            result = _result_for_plot(log, axis)
            if result is not None and result.n_traces > 0 and metrics is not None:
                for trace in result.traces:
                    fig.add_trace(
                        go.Scatter(
                            x=result.t_ms,
                            y=trace,
                            mode="lines",
                            line={"color": log.color, "width": 0.55, "simplify": False},
                            opacity=0.10,
                            hoverinfo="skip",
                            showlegend=False,
                        ),
                        row=row,
                        col=1,
                    )
                fig.add_trace(
                    go.Scatter(
                        x=result.t_ms,
                        y=result.mean_response,
                        mode="lines",
                        line={"color": log.color, "width": 2.4, "simplify": False},
                        name=f"{log.label} {axis}",
                        legendgroup=log.label,
                        hovertemplate=f"{log.label}<br>time=%{{x:.1f}} ms<br>response=%{{y:.4f}}<extra></extra>",
                    ),
                    row=row,
                    col=1,
                )

    for row in range(1, 4):
        fig.update_xaxes(range=[0, 500], title_text="Time (ms)", showgrid=True, minor_ticks="inside", row=row, col=1)
        fig.update_yaxes(
            range=[0, 1.75],
            title_text=["Roll Response", "Pitch Response", "Yaw Response"][row - 1],
            showgrid=True,
            minor_ticks="inside",
            row=row,
            col=1,
        )
    _add_metric_annotations(fig, logs)
    fig.update_layout(
        title={"text": "PIDtoolbox-style Step Response Detail", "x": 0.5, "xanchor": "center"},
        template="plotly_white",
        height=1180,
        width=1280,
        hovermode="x unified",
        legend={"orientation": "h", "yanchor": "bottom", "y": 1.02, "xanchor": "left", "x": 0.0},
        margin={"l": 80, "r": 220, "t": 92, "b": 64},
    )
    fig.write_html(str(html_path), include_plotlyjs=True, full_html=True, auto_open=False)


def _add_metric_annotations(fig: Any, logs: list[StepResponseLogMetrics]) -> None:
    for axis, row in (("roll", 1), ("pitch", 2), ("yaw", 3)):
        lines: list[str] = []
        for index, log in enumerate(logs, start=1):
            metrics = log.axes.get(axis)
            if metrics is None:
                continue
            if metrics.error:
                detail = metrics.error
            elif metrics.accepted_traces <= 0:
                detail = "insufficient data"
            else:
                detail = f"n={metrics.accepted_traces}, peak={metrics.peak:.3f}, lat={metrics.latency_half_height_ms:.1f} ms"
            lines.append(f"<span style='color:{log.color}'> {index}) {log.label}: {detail}</span>")
        fig.add_annotation(
            text="<br>".join(lines),
            x=1.01,
            y=0.98,
            xref="x domain",
            yref="y domain",
            xanchor="left",
            yanchor="top",
            align="left",
            showarrow=False,
            font={"size": 11},
            row=row,
            col=1,
        )


def _summary_json_payload(report_dir: Path, html_path: Path, summary_json: Path, logs: list[StepResponseLogMetrics]) -> str:
    payload: dict[str, object] = {
        "report_dir": str(report_dir),
        "html_path": str(html_path),
        "summary_json": str(summary_json),
        "decoder": str(_find_tools_decoder()),
        "decoder_options": ["--unit-rotation", "deg/s"],
        "settings": STEP_RESPONSE_SETTINGS,
        "logs": [
            {
                "label": log.label,
                "color": log.color,
                "source_log": log.source_log,
                "decoded_csv": log.decoded_csv,
                "beeper_marker": {
                    "column": log.marker_column,
                    "start_index": log.marker_start_index,
                    "end_index": log.marker_end_index,
                    "start_s": log.marker_start_s,
                    "end_s": log.marker_end_s,
                    "samples": log.marker_samples,
                    "warning": log.marker_warning,
                },
                "axes": {
                    axis: {
                        "setpoint_column": metrics.setpoint_column,
                        "gyro_column": metrics.gyro_column,
                        "accepted_traces": metrics.accepted_traces,
                        "sample_rate_hz": metrics.sample_rate_hz,
                        "peak": metrics.peak,
                        "peak_time_ms": metrics.peak_time_ms,
                        "latency_half_height_ms": metrics.latency_half_height_ms,
                        "error": metrics.error,
                    }
                    for axis, metrics in log.axes.items()
                },
            }
            for log in logs
        ],
    }
    return json.dumps(payload, indent=2)


def _finite_or_none(value: float) -> float | None:
    if not math.isfinite(float(value)):
        return None
    return float(value)


def _safe_filename(name: str) -> str:
    safe = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in name.strip())
    return safe or "blackbox_log"

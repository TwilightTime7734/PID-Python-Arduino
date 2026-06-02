"""Interactive roll trace viewer generation for Blackbox CSV logs."""

from __future__ import annotations

import csv
import webbrowser
from pathlib import Path
from typing import Any

import numpy as np


def _normalize_header(header: str) -> str:
    return "".join(ch for ch in header.strip().lower() if ch not in " \t\r\n_()-")


def _pick_column(columns: dict[str, np.ndarray], candidates: list[str]) -> str | None:
    normalized_to_raw = {_normalize_header(raw): raw for raw in columns}
    for candidate in candidates:
        raw = normalized_to_raw.get(_normalize_header(candidate))
        if raw is not None:
            return raw
    return None


def _looks_like_blackbox_header(raw_fields: list[str]) -> bool:
    if not raw_fields:
        return False
    fields = {_normalize_header(item) for item in raw_fields if item}
    if not fields:
        return False
    has_time = any(name in fields for name in ("time", "timeus", "loopiteration"))
    has_motion = any(
        name in fields
        for name in ("axisrate[0]", "gyroadc[0]", "gyroraw[0]", "setpoint[0]", "rccommand[0]", "motor[0]")
    )
    return has_time and has_motion


def _detect_csv_header_offset(path: Path, max_probe_lines: int = 500) -> int:
    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        for index, line in enumerate(handle):
            if index >= max_probe_lines:
                break
            if "," not in line:
                continue
            try:
                row = next(csv.reader([line], skipinitialspace=True), [])
            except Exception:
                continue
            if _looks_like_blackbox_header(row):
                return index
    return 0


def load_blackbox_csv(csv_path: str | Path) -> dict[str, np.ndarray]:
    """Load numeric columns from a blackbox CSV into numpy arrays."""
    path = Path(csv_path)
    header_offset = _detect_csv_header_offset(path)

    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        for _ in range(header_offset):
            next(handle, None)
        sample = handle.read(4096)
    dialect = csv.excel
    if sample.strip():
        try:
            dialect = csv.Sniffer().sniff(sample)
        except Exception:
            dialect = csv.excel

    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        for _ in range(header_offset):
            next(handle, None)
        reader = csv.DictReader(handle, dialect=dialect, skipinitialspace=True)
        if reader.fieldnames and (
            (len(reader.fieldnames) <= 1 and "," in sample)
            or any("," in header for header in reader.fieldnames if header)
        ):
            handle.close()
            with path.open("r", encoding="utf-8", errors="replace", newline="") as comma_handle:
                for _ in range(header_offset):
                    next(comma_handle, None)
                reader = csv.DictReader(comma_handle, delimiter=",", skipinitialspace=True)
                if not reader.fieldnames:
                    raise RuntimeError("CSV header row is missing.")
                headers = [header for header in reader.fieldnames if header is not None]
                values: dict[str, list[float]] = {header: [] for header in headers}
                for row in reader:
                    for header in headers:
                        raw = (row.get(header) or "").strip()
                        try:
                            values[header].append(float(raw))
                        except Exception:
                            values[header].append(np.nan)

        else:
            if not reader.fieldnames:
                raise RuntimeError("CSV header row is missing.")
            headers = [header for header in reader.fieldnames if header is not None]
            values = {header: [] for header in headers}
            for row in reader:
                for header in headers:
                    raw = (row.get(header) or "").strip()
                    try:
                        values[header].append(float(raw))
                    except Exception:
                        values[header].append(np.nan)

    return {name: np.asarray(series, dtype=float) for name, series in values.items()}


def detect_time_column(columns: dict[str, np.ndarray]) -> str | None:
    return _pick_column(columns, ["time", "time (us)", "timeUs", "loopIteration"])


def detect_roll_columns(columns: dict[str, np.ndarray]) -> dict[str, str | None]:
    # Reminder (2026-06-01): do not use axisRate as the roll setpoint source here.
    # Keep this viewer on setpoint/RC vs gyro signals.
    return {
        "setpoint": _pick_column(
            columns,
            ["setpoint[0]", "gyroSetpoint[0]", "rcCommand[0]"],
        ),
        "gyro": _pick_column(
            columns,
            ["gyroADC[0]", "gyroADC[roll]", "gyro[0]", "gyroData[0]"],
        ),
        "rc": _pick_column(columns, ["rcCommand[0]", "rcData[0]"]),
        "throttle": _pick_column(columns, ["throttle", "rcCommand[3]"]),
    }


def _time_axis(columns: dict[str, np.ndarray]) -> tuple[np.ndarray, str]:
    time_col = detect_time_column(columns)
    if time_col is None:
        # Fallback to sample index if time is missing.
        first = next(iter(columns.values()))
        return np.arange(first.size, dtype=float), "Sample"

    raw = np.asarray(columns[time_col], dtype=float)
    if raw.size == 0 or not np.any(np.isfinite(raw)):
        first = next(iter(columns.values()))
        return np.arange(first.size, dtype=float), "Sample"
    raw = raw - np.nanmin(raw)
    span = float(np.nanmax(raw)) if raw.size else 0.0
    if span > 1_000_000.0:
        return raw / 1_000_000.0, "Time (s)"
    if span > 10_000.0:
        return raw / 1000.0, "Time (s)"
    if _normalize_header(time_col) == _normalize_header("loopIteration"):
        return raw, "Loop iteration"
    return raw, "Time"


def _replace_invalid(values: np.ndarray) -> np.ndarray:
    return np.nan_to_num(np.asarray(values, dtype=float), nan=0.0, posinf=0.0, neginf=0.0)


def _add_trace(fig: Any, *, row: int, x: np.ndarray, y: np.ndarray, name: str, color: str, width: float) -> None:
    import plotly.graph_objects as go

    fig.add_trace(
        go.Scatter(
            x=x,
            y=y,
            mode="lines",
            name=name,
            line={"width": width, "color": color},
        ),
        row=row,
        col=1,
    )


def build_roll_trace_viewer(csv_path: str | Path, output_dir: str | Path) -> Path:
    """Generate a Blackbox-Explorer-style HTML viewer for roll traces."""
    try:
        from plotly.subplots import make_subplots
    except Exception as exc:
        raise RuntimeError(
            "Plotly is required for roll trace viewer generation. Install with: pip install plotly"
        ) from exc

    csv_path = Path(csv_path)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    columns = load_blackbox_csv(csv_path)
    detected = detect_roll_columns(columns)
    setpoint_col = detected["setpoint"]
    gyro_col = detected["gyro"]

    if setpoint_col is None or gyro_col is None:
        available = ", ".join(columns.keys())
        axisrate_hint = ""
        if setpoint_col is None and _pick_column(columns, ["axisRate[0]"]) is not None:
            axisrate_hint = "\nNote: axisRate[0] is intentionally ignored for setpoint selection."
        raise RuntimeError(
            "Could not find required roll columns.\n"
            f"Detected setpoint: {setpoint_col}\n"
            f"Detected gyro: {gyro_col}\n"
            f"Available columns: {available}{axisrate_hint}"
        )

    x, x_label = _time_axis(columns)
    setpoint = _replace_invalid(columns[setpoint_col])
    gyro = _replace_invalid(columns[gyro_col])
    roll_error = setpoint - gyro

    rc_series: np.ndarray | None = None
    throttle_series: np.ndarray | None = None
    if detected["rc"] is not None:
        rc_series = _replace_invalid(columns[detected["rc"]])
    if detected["throttle"] is not None:
        throttle_series = _replace_invalid(columns[detected["throttle"]])
    else:
        motor_cols = [_pick_column(columns, [f"motor[{i}]"]) for i in range(4)]
        if all(col is not None for col in motor_cols):
            motors = [_replace_invalid(columns[col]) for col in motor_cols if col is not None]
            throttle_series = np.mean(np.vstack(motors), axis=0)

    has_aux = rc_series is not None or throttle_series is not None
    rows = 3 if has_aux else 2
    heights = [0.56, 0.27, 0.17] if has_aux else [0.68, 0.32]
    titles = ("Roll setpoint vs gyro", "Roll error", "RC / Throttle") if has_aux else ("Roll setpoint vs gyro", "Roll error")
    fig = make_subplots(
        rows=rows,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.05,
        row_heights=heights,
        subplot_titles=titles,
    )

    _add_trace(fig, row=1, x=x, y=setpoint, name=f"Setpoint ({setpoint_col})", color="#f59e0b", width=1.0)
    _add_trace(fig, row=1, x=x, y=gyro, name=f"Gyro ({gyro_col})", color="#2563eb", width=1.0)
    _add_trace(fig, row=2, x=x, y=roll_error, name="Roll error", color="#dc2626", width=0.8)

    if has_aux:
        if rc_series is not None and detected["rc"] is not None:
            _add_trace(fig, row=3, x=x, y=rc_series, name=f"RC ({detected['rc']})", color="#0f766e", width=0.9)
        if throttle_series is not None:
            name = f"Throttle ({detected['throttle']})" if detected["throttle"] else "Throttle (motor mean)"
            _add_trace(fig, row=3, x=x, y=throttle_series, name=name, color="#7c3aed", width=0.9)

    fig.update_layout(
        template="plotly_white",
        hovermode="x unified",
        title=f"Roll Trace Viewer: {csv_path.name}",
        height=920 if has_aux else 760,
        legend={"orientation": "h", "yanchor": "bottom", "y": 1.02, "xanchor": "left", "x": 0.01},
        margin={"l": 60, "r": 20, "t": 90, "b": 50},
    )
    fig.update_xaxes(title_text=x_label, row=rows, col=1)
    fig.update_yaxes(title_text="Rate", row=1, col=1)
    fig.update_yaxes(title_text="Error", row=2, col=1)
    if has_aux:
        fig.update_yaxes(title_text="Input", row=3, col=1)

    html_path = output_path / "roll_trace_viewer.html"
    fig.write_html(
        str(html_path),
        include_plotlyjs="cdn",
        full_html=True,
        auto_open=False,
        config={
            "responsive": True,
            "displaylogo": False,
            "scrollZoom": True,
        },
    )

    return html_path


def open_trace_viewer(html_path: Path) -> None:
    """Open a generated roll trace HTML viewer in the default browser."""
    webbrowser.open_new_tab(html_path.resolve().as_uri())

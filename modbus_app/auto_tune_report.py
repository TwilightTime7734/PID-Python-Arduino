"""Auto-tune chart and summary report generation for blackbox sessions."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from .blackbox_import import BlackboxImportResult


_CHART_DPI = 300
_COMBINED_CHART_DPI = 260
_CHART_SIZE_WIDE = (11.5, 4.2)
_CHART_SIZE_SHORT = (11.5, 3.6)
_COMBINED_CHART_SIZE = (20.0, 12.5)
_OSCILLOSCOPE_WINDOW_S = 0.45
_OSCILLOSCOPE_MAX_WINDOWS = 4
_OSCILLOSCOPE_MAX_MARKERS = 220


@dataclass(frozen=True)
class AutoTuneReport:
    report_dir: str
    summary_txt: str
    summary_json: str
    combined_chart_sheet: str
    chart_paths: tuple[str, ...]


def generate_auto_tune_report(
    output_root: str | Path,
    analysis_result: BlackboxImportResult,
    session_payload: dict[str, Any],
    preferred_log_path: str | Path | None = None,
) -> AutoTuneReport:
    output_root_path = Path(output_root)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_dir = output_root_path / "reports" / f"session_{timestamp}"
    report_dir.mkdir(parents=True, exist_ok=True)

    source_hint = Path(preferred_log_path) if preferred_log_path else Path(analysis_result.analysis_source or "")
    csv_path = _resolve_csv_path(source_hint)
    if csv_path is None:
        raise RuntimeError("Could not resolve a primary Blackbox CSV for chart generation.")

    time_s, cols = _read_blackbox_csv(csv_path)
    data = _extract_plot_columns(time_s, cols)

    charts: dict[str, Path] = {}
    charts["roll_setpoint_vs_actual"] = report_dir / "roll_setpoint_vs_actual.png"
    charts["roll_error"] = report_dir / "roll_error.png"
    charts["roll_pid_terms"] = report_dir / "roll_pid_terms.png"
    charts["motor_outputs"] = report_dir / "motor_outputs.png"
    charts["roll_gyro_fft"] = report_dir / "roll_gyro_fft.png"
    charts["extreme_event_zoom"] = report_dir / "extreme_event_zoom.png"
    charts["tracking_oscilloscope"] = report_dir / "tracking_oscilloscope.png"
    combined_chart = report_dir / "combined_chart_sheet.png"

    _render_individual_charts(data, charts)
    _render_combined_chart(data, charts, combined_chart)

    summary_txt = report_dir / "summary.txt"
    summary_json = report_dir / "summary.json"

    summary_payload = _build_summary_payload(
        analysis_result=analysis_result,
        session_payload=session_payload,
        csv_path=csv_path,
        combined_chart=combined_chart,
        charts=charts,
        data=data,
    )
    summary_txt.write_text(_format_summary_text(summary_payload), encoding="utf-8")
    summary_json.write_text(json.dumps(summary_payload, indent=2), encoding="utf-8")

    all_chart_paths_list: list[str] = []
    combined_svg = combined_chart.with_suffix(".svg")
    if combined_svg.exists():
        all_chart_paths_list.append(str(combined_svg))
    for path in charts.values():
        all_chart_paths_list.append(str(path))
        svg_path = path.with_suffix(".svg")
        if svg_path.exists():
            all_chart_paths_list.append(str(svg_path))
    all_chart_paths = tuple(all_chart_paths_list)
    return AutoTuneReport(
        report_dir=str(report_dir),
        summary_txt=str(summary_txt),
        summary_json=str(summary_json),
        combined_chart_sheet=str(combined_chart),
        chart_paths=all_chart_paths,
    )


def _resolve_csv_path(source_hint: Path) -> Path | None:
    if source_hint.exists() and source_hint.is_file() and source_hint.suffix.lower() == ".csv":
        if not _is_auxiliary_csv(source_hint):
            return source_hint

    search_dir = source_hint.parent if source_hint.exists() else source_hint.parent if source_hint.parent.exists() else None
    if search_dir is None or not search_dir.exists():
        return None

    stem = source_hint.stem if source_hint.stem else ""
    candidates: list[Path] = []
    if stem:
        candidates.extend(p for p in search_dir.glob(f"{stem}*.csv") if p.is_file())
    candidates.extend(p for p in search_dir.glob("*.csv") if p.is_file())

    preferred = [p for p in candidates if not _is_auxiliary_csv(p)]
    if not preferred:
        preferred = candidates
    if not preferred:
        return None

    return max(preferred, key=lambda p: p.stat().st_mtime)


def _is_auxiliary_csv(path: Path) -> bool:
    name = path.name.lower()
    return name.endswith(".gps.csv") or name.endswith(".event.csv") or name.endswith(".events.csv")


def _normalize_header(header: str) -> str:
    return "".join(ch for ch in header.strip().lower() if ch not in " \t\r\n")


def _read_blackbox_csv(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        sample = handle.read(4096)
        handle.seek(0)
        dialect = csv.excel
        if sample.strip():
            try:
                dialect = csv.Sniffer().sniff(sample)
            except Exception:
                dialect = csv.excel
        reader = csv.DictReader(handle, dialect=dialect, skipinitialspace=True)
        if reader.fieldnames and (
            (len(reader.fieldnames) <= 1 and "," in sample)
            or any("," in header for header in reader.fieldnames if header)
        ):
            handle.seek(0)
            reader = csv.DictReader(handle, delimiter=",", skipinitialspace=True)
        if not reader.fieldnames:
            raise RuntimeError("CSV header row missing.")

        headers = [h for h in reader.fieldnames if h is not None]
        normalized = {_normalize_header(h): h for h in headers}
        values: dict[str, list[float]] = {key: [] for key in normalized}

        for row in reader:
            for norm_key, raw_key in normalized.items():
                raw = (row.get(raw_key) or "").strip()
                try:
                    values[norm_key].append(float(raw))
                except Exception:
                    values[norm_key].append(0.0)

    time_key = None
    for candidate in ("time(us)", "time", "loopiteration"):
        if candidate in values:
            time_key = candidate
            break
    if time_key is None:
        raise RuntimeError("No recognizable time column in blackbox CSV.")

    raw_time = np.array(values[time_key], dtype=float)
    if raw_time.size < 10:
        raise RuntimeError("Not enough rows in blackbox CSV for reporting.")
    raw_time = raw_time - raw_time.min()
    span = float(raw_time.max())
    if span > 1_000_000.0:
        time_s = raw_time / 1_000_000.0
    elif span > 10_000.0:
        time_s = raw_time / 1000.0
    else:
        time_s = raw_time

    columns = {name: np.array(series, dtype=float) for name, series in values.items()}
    return time_s, columns


def _column(columns: dict[str, np.ndarray], names: list[str]) -> np.ndarray | None:
    for name in names:
        key = _normalize_header(name)
        if key in columns:
            return columns[key]
    return None


def _extract_plot_columns(time_s: np.ndarray, columns: dict[str, np.ndarray]) -> dict[str, Any]:
    roll_setpoint = _column(columns, ["rcCommand[0]", "setpoint[0]"])
    roll_actual = _column(columns, ["gyroADC[0]"])
    if roll_setpoint is None or roll_actual is None:
        raise RuntimeError("Missing roll setpoint/actual columns in CSV.")

    axis_p = _column(columns, ["axisP[0]"])
    axis_i = _column(columns, ["axisI[0]"])
    axis_d = _column(columns, ["axisD[0]"])
    axis_f = _column(columns, ["axisF[0]"])

    motors = []
    for index in range(4):
        series = _column(columns, [f"motor[{index}]"])
        if series is not None:
            motors.append(series)
    if not motors:
        raise RuntimeError("Missing motor output columns in CSV.")

    roll_gyro = _column(columns, ["gyroRaw[0]", "gyroADC[0]"])
    if roll_gyro is None:
        raise RuntimeError("Missing roll gyro column for FFT chart.")

    roll_error = roll_setpoint - roll_actual
    peak_idx = int(np.argmax(np.abs(roll_error)))
    peak_time = float(time_s[peak_idx])
    crossing_idx, crossing_time, crossing_value = _compute_tracking_crossings(time_s, roll_setpoint, roll_actual)

    sample_rate = _estimate_sample_rate(time_s)
    return {
        "time_s": time_s,
        "setpoint": roll_setpoint,
        "actual": roll_actual,
        "error": roll_error,
        "axis_p": axis_p,
        "axis_i": axis_i,
        "axis_d": axis_d,
        "axis_f": axis_f,
        "motors": motors,
        "gyro": roll_gyro,
        "sample_rate": sample_rate,
        "peak_idx": peak_idx,
        "peak_time": peak_time,
        "crossing_idx": crossing_idx,
        "crossing_time": crossing_time,
        "crossing_value": crossing_value,
    }


def _estimate_sample_rate(time_s: np.ndarray) -> float:
    if time_s.size < 3:
        return 1.0
    diffs = np.diff(time_s)
    diffs = diffs[diffs > 0]
    if diffs.size == 0:
        return 1.0
    return float(1.0 / np.median(diffs))


def _configure_matplotlib() -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    # Keep dense traces intact and anti-aliased for high-zoom inspection.
    matplotlib.rcParams["path.simplify"] = False
    matplotlib.rcParams["lines.antialiased"] = True


def _save_chart_figure(fig: Any, path: Path, dpi: int) -> None:
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    bg = fig.get_facecolor()
    fig.savefig(path, dpi=dpi, bbox_inches="tight", facecolor=bg)
    fig.savefig(path.with_suffix(".svg"), format="svg", bbox_inches="tight", facecolor=bg)


def _compute_tracking_crossings(
    time_s: np.ndarray,
    setpoint: np.ndarray,
    actual: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if time_s.size < 2 or setpoint.size < 2 or actual.size < 2:
        return (
            np.array([], dtype=int),
            np.array([], dtype=float),
            np.array([], dtype=float),
        )

    error = setpoint - actual
    e0 = error[:-1]
    e1 = error[1:]
    crossing_mask = (
        (e0 == 0.0)
        | (e1 == 0.0)
        | ((e0 > 0.0) & (e1 < 0.0))
        | ((e0 < 0.0) & (e1 > 0.0))
    )
    idx = np.flatnonzero(crossing_mask)
    if idx.size == 0:
        return (
            np.array([], dtype=int),
            np.array([], dtype=float),
            np.array([], dtype=float),
        )

    t0 = time_s[idx]
    t1 = time_s[idx + 1]
    s0 = setpoint[idx]
    s1 = setpoint[idx + 1]
    a0 = actual[idx]
    a1 = actual[idx + 1]
    denom = np.abs(e0[idx]) + np.abs(e1[idx])
    frac = np.zeros(idx.size, dtype=float)
    good = denom > 1e-12
    frac[good] = np.abs(e0[idx][good]) / denom[good]

    crossing_time = t0 + (t1 - t0) * frac
    s_interp = s0 + (s1 - s0) * frac
    a_interp = a0 + (a1 - a0) * frac
    crossing_value = 0.5 * (s_interp + a_interp)
    return idx.astype(int), crossing_time.astype(float), crossing_value.astype(float)


def _select_evenly_spaced_indices(count: int, target_count: int) -> np.ndarray:
    if count <= 0 or target_count <= 0:
        return np.array([], dtype=int)
    if count <= target_count:
        return np.arange(count, dtype=int)
    values = np.linspace(0, count - 1, num=target_count)
    return np.unique(values.astype(int))


def _marker_subset_indices(count: int, max_markers: int) -> np.ndarray:
    if count <= 0:
        return np.array([], dtype=int)
    if count <= max_markers:
        return np.arange(count, dtype=int)
    step = int(math.ceil(count / max_markers))
    return np.arange(0, count, step, dtype=int)


def _render_tracking_oscilloscope_chart(data: dict[str, Any], output: Path) -> None:
    from matplotlib.figure import Figure
    from matplotlib.ticker import MultipleLocator

    crossing_times = np.asarray(data["crossing_time"], dtype=float)
    crossing_values = np.asarray(data["crossing_value"], dtype=float)
    if crossing_times.size > 0:
        selected = _select_evenly_spaced_indices(crossing_times.size, _OSCILLOSCOPE_MAX_WINDOWS)
        centers = crossing_times[selected]
    else:
        centers = np.array([float(data["peak_time"])], dtype=float)

    rows = int(max(1, centers.size))
    fig_height = max(4.8, rows * 3.2)
    fig = Figure(figsize=(14.0, fig_height), dpi=_CHART_DPI)
    axes = [fig.add_subplot(rows, 1, i + 1) for i in range(rows)]

    time_s = np.asarray(data["time_s"], dtype=float)
    setpoint = np.asarray(data["setpoint"], dtype=float)
    actual = np.asarray(data["actual"], dtype=float)
    half_window = _OSCILLOSCOPE_WINDOW_S / 2.0
    major_step = _OSCILLOSCOPE_WINDOW_S / 9.0
    minor_step = major_step / 5.0

    for i, (axis, center) in enumerate(zip(axes, centers)):
        t0 = center - half_window
        t1 = center + half_window
        mask = (time_s >= t0) & (time_s <= t1)
        if np.count_nonzero(mask) < 4:
            t0 = center - _OSCILLOSCOPE_WINDOW_S
            t1 = center + _OSCILLOSCOPE_WINDOW_S
            mask = (time_s >= t0) & (time_s <= t1)
        if np.count_nonzero(mask) < 2:
            mask = np.ones_like(time_s, dtype=bool)
            t0 = float(time_s.min())
            t1 = float(time_s.max())

        xt = time_s[mask]
        ys = setpoint[mask]
        ya = actual[mask]
        yall = np.concatenate((ys, ya))
        y_min = float(np.min(yall))
        y_max = float(np.max(yall))
        y_span = max(1.0, y_max - y_min)
        y_pad = max(4.0, y_span * 0.18)

        axis.set_facecolor("#060b0f")
        for spine in axis.spines.values():
            spine.set_color("#5a8f60")
            spine.set_linewidth(0.8)
        axis.set_axisbelow(True)
        axis.tick_params(colors="#b8f7bf", labelsize=8, length=4.0, width=0.8)
        axis.grid(which="major", color="#2b5931", linewidth=0.75, alpha=0.95)
        axis.grid(which="minor", color="#19361e", linewidth=0.5, alpha=0.95)
        axis.xaxis.set_major_locator(MultipleLocator(max(major_step, 0.002)))
        axis.xaxis.set_minor_locator(MultipleLocator(max(minor_step, 0.001)))

        axis.plot(xt, ys, color="#ffd166", linewidth=1.35, label="Setpoint")
        axis.plot(xt, ya, color="#00d4ff", linewidth=1.35, label="Actual")
        axis.set_xlim(float(t0), float(t1))
        axis.set_ylim(y_min - y_pad, y_max + y_pad)
        axis.set_ylabel("Rate", color="#b8f7bf")

        crossing_window = (crossing_times >= t0) & (crossing_times <= t1)
        if np.any(crossing_window):
            local_t = crossing_times[crossing_window]
            local_v = crossing_values[crossing_window]
            keep = _marker_subset_indices(local_t.size, _OSCILLOSCOPE_MAX_MARKERS)
            axis.scatter(
                local_t[keep],
                local_v[keep],
                s=18,
                c="#f8fafc",
                edgecolors="#111827",
                linewidths=0.5,
                zorder=5,
                label="Cross",
            )

        axis.axvline(center, color="#f97316", linestyle="--", linewidth=0.95, alpha=0.85)
        axis.set_title(
            f"Tracking scope {i + 1}: center={center:.3f}s, window={t1 - t0:.3f}s",
            color="#c9fbcf",
            fontsize=10,
        )
        if i == 0:
            axis.legend(loc="upper right", fontsize=8, framealpha=0.22)

    axes[-1].set_xlabel("Time (s)", color="#b8f7bf")
    fig.patch.set_facecolor("#05070b")
    fig.tight_layout(pad=1.25)
    _save_chart_figure(fig, output, _CHART_DPI)


def _render_individual_charts(data: dict[str, Any], charts: dict[str, Path]) -> None:
    _configure_matplotlib()
    from matplotlib.figure import Figure

    # 1) Setpoint vs actual
    fig = Figure(figsize=_CHART_SIZE_WIDE, dpi=_CHART_DPI)
    ax = fig.add_subplot(111)
    ax.plot(data["time_s"], data["setpoint"], label="Roll setpoint", linewidth=1.0)
    ax.plot(data["time_s"], data["actual"], label="Roll actual", linewidth=1.0)
    ax.set_title("Roll setpoint vs actual")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rate")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.25)
    crossing_times = np.asarray(data["crossing_time"], dtype=float)
    crossing_values = np.asarray(data["crossing_value"], dtype=float)
    if crossing_times.size > 0:
        keep = _marker_subset_indices(crossing_times.size, _OSCILLOSCOPE_MAX_MARKERS)
        ax.scatter(
            crossing_times[keep],
            crossing_values[keep],
            s=14,
            c="#111827",
            edgecolors="#f8fafc",
            linewidths=0.5,
            alpha=0.9,
            label="Crossings",
            zorder=5,
        )
        ax.text(
            0.01,
            0.97,
            f"Crossings: {crossing_times.size}",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=8,
            color="#0f172a",
            bbox={"facecolor": "#f8fafc", "alpha": 0.65, "pad": 2.2, "edgecolor": "#cbd5e1"},
        )
    _save_chart_figure(fig, charts["roll_setpoint_vs_actual"], _CHART_DPI)

    # 2) Error
    fig = Figure(figsize=_CHART_SIZE_SHORT, dpi=_CHART_DPI)
    ax = fig.add_subplot(111)
    ax.plot(data["time_s"], data["error"], color="#b91c1c", linewidth=1.0)
    ax.axhline(0.0, color="#334155", linestyle="--", linewidth=0.8)
    ax.set_title("Roll tracking error")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Setpoint - actual")
    ax.grid(True, alpha=0.25)
    _save_chart_figure(fig, charts["roll_error"], _CHART_DPI)

    # 3) PID terms
    fig = Figure(figsize=_CHART_SIZE_WIDE, dpi=_CHART_DPI)
    ax = fig.add_subplot(111)
    if data["axis_p"] is not None:
        ax.plot(data["time_s"], data["axis_p"], label="P", linewidth=0.9)
    if data["axis_i"] is not None:
        ax.plot(data["time_s"], data["axis_i"], label="I", linewidth=0.9)
    if data["axis_d"] is not None:
        ax.plot(data["time_s"], data["axis_d"], label="D", linewidth=0.9)
    if data["axis_f"] is not None:
        ax.plot(data["time_s"], data["axis_f"], label="F", linewidth=0.9)
    ax.set_title("Roll PID terms")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("PID term value")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    _save_chart_figure(fig, charts["roll_pid_terms"], _CHART_DPI)

    # 4) Motor outputs
    fig = Figure(figsize=_CHART_SIZE_WIDE, dpi=_CHART_DPI)
    ax = fig.add_subplot(111)
    for i, motor in enumerate(data["motors"]):
        ax.plot(data["time_s"], motor, label=f"M{i + 1}", linewidth=0.85)
    ax.set_title("Motor outputs")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Motor command")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", ncol=2)
    _save_chart_figure(fig, charts["motor_outputs"], _CHART_DPI)

    # 5) FFT
    fig = Figure(figsize=_CHART_SIZE_SHORT, dpi=_CHART_DPI)
    ax = fig.add_subplot(111)
    gyro = data["gyro"] - np.mean(data["gyro"])
    sr = max(1.0, float(data["sample_rate"]))
    if gyro.size > 8:
        spectrum = np.fft.rfft(gyro)
        freqs = np.fft.rfftfreq(gyro.size, d=1.0 / sr)
        power = np.abs(spectrum)
        ax.plot(freqs, power, color="#0369a1", linewidth=0.9)
        ax.set_xlim(0.0, min(500.0, float(freqs.max())))
    ax.set_title("Roll gyro FFT")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Magnitude")
    ax.grid(True, alpha=0.25)
    _save_chart_figure(fig, charts["roll_gyro_fft"], _CHART_DPI)

    # 6) Extreme event zoom
    fig = Figure(figsize=_CHART_SIZE_WIDE, dpi=_CHART_DPI)
    ax = fig.add_subplot(111)
    peak_time = float(data["peak_time"])
    t0 = peak_time - 0.8
    t1 = peak_time + 0.8
    mask = (data["time_s"] >= t0) & (data["time_s"] <= t1)
    if not np.any(mask):
        mask = np.ones_like(data["time_s"], dtype=bool)
    ax.plot(data["time_s"][mask], data["setpoint"][mask], label="Setpoint", linewidth=1.0)
    ax.plot(data["time_s"][mask], data["actual"][mask], label="Actual", linewidth=1.0)
    ax.plot(data["time_s"][mask], data["error"][mask], label="Error", linewidth=0.9)
    ax.axvline(peak_time, color="#ea580c", linestyle="--", linewidth=0.9)
    ax.set_title("Extreme event zoom (max |roll error|)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    _save_chart_figure(fig, charts["extreme_event_zoom"], _CHART_DPI)

    _render_tracking_oscilloscope_chart(data, charts["tracking_oscilloscope"])


def _render_combined_chart(data: dict[str, Any], charts: dict[str, Path], output: Path) -> None:
    _configure_matplotlib()
    from matplotlib.figure import Figure

    fig = Figure(figsize=_COMBINED_CHART_SIZE, dpi=_COMBINED_CHART_DPI)
    axes = [fig.add_subplot(3, 2, i + 1) for i in range(6)]

    # Recreate the same chart content in a compact sheet.
    axes[0].plot(data["time_s"], data["setpoint"], label="Setpoint", linewidth=0.8)
    axes[0].plot(data["time_s"], data["actual"], label="Actual", linewidth=0.8)
    axes[0].set_title("Roll setpoint vs actual")
    axes[0].grid(True, alpha=0.2)
    axes[0].legend(loc="upper right", fontsize=7)

    axes[1].plot(data["time_s"], data["error"], color="#b91c1c", linewidth=0.8)
    axes[1].axhline(0.0, color="#334155", linestyle="--", linewidth=0.7)
    axes[1].set_title("Roll error")
    axes[1].grid(True, alpha=0.2)

    if data["axis_p"] is not None:
        axes[2].plot(data["time_s"], data["axis_p"], label="P", linewidth=0.7)
    if data["axis_i"] is not None:
        axes[2].plot(data["time_s"], data["axis_i"], label="I", linewidth=0.7)
    if data["axis_d"] is not None:
        axes[2].plot(data["time_s"], data["axis_d"], label="D", linewidth=0.7)
    if data["axis_f"] is not None:
        axes[2].plot(data["time_s"], data["axis_f"], label="F", linewidth=0.7)
    axes[2].set_title("Roll PID terms")
    axes[2].grid(True, alpha=0.2)
    axes[2].legend(loc="upper right", fontsize=7)

    for i, motor in enumerate(data["motors"]):
        axes[3].plot(data["time_s"], motor, label=f"M{i + 1}", linewidth=0.7)
    axes[3].set_title("Motor outputs")
    axes[3].grid(True, alpha=0.2)
    axes[3].legend(loc="upper right", fontsize=7, ncol=2)

    gyro = data["gyro"] - np.mean(data["gyro"])
    sr = max(1.0, float(data["sample_rate"]))
    if gyro.size > 8:
        spectrum = np.fft.rfft(gyro)
        freqs = np.fft.rfftfreq(gyro.size, d=1.0 / sr)
        power = np.abs(spectrum)
        axes[4].plot(freqs, power, color="#0369a1", linewidth=0.7)
        axes[4].set_xlim(0.0, min(500.0, float(freqs.max())))
    axes[4].set_title("Roll gyro FFT")
    axes[4].grid(True, alpha=0.2)

    peak_time = float(data["peak_time"])
    t0 = peak_time - 0.8
    t1 = peak_time + 0.8
    mask = (data["time_s"] >= t0) & (data["time_s"] <= t1)
    if not np.any(mask):
        mask = np.ones_like(data["time_s"], dtype=bool)
    axes[5].plot(data["time_s"][mask], data["setpoint"][mask], label="Setpoint", linewidth=0.8)
    axes[5].plot(data["time_s"][mask], data["actual"][mask], label="Actual", linewidth=0.8)
    axes[5].plot(data["time_s"][mask], data["error"][mask], label="Error", linewidth=0.75)
    axes[5].axvline(peak_time, color="#ea580c", linestyle="--", linewidth=0.8)
    axes[5].set_title("Extreme event zoom")
    axes[5].grid(True, alpha=0.2)
    axes[5].legend(loc="upper right", fontsize=7)

    for axis in axes:
        axis.tick_params(labelsize=7)
    fig.tight_layout()
    _save_chart_figure(fig, output, _COMBINED_CHART_DPI)


def _build_summary_payload(
    analysis_result: BlackboxImportResult,
    session_payload: dict[str, Any],
    csv_path: Path,
    combined_chart: Path,
    charts: dict[str, Path],
    data: dict[str, Any],
) -> dict[str, Any]:
    metrics = session_payload.get("metrics", {})
    pid_report = analysis_result.pid_report
    return {
        "generated_at": datetime.now().isoformat(),
        "session": {
            "state": session_payload.get("state", "unknown"),
            "stop_reason": session_payload.get("stop_reason", ""),
            "warning": session_payload.get("warning", ""),
            "elapsed_s": round(float(session_payload.get("elapsed_s", 0.0)), 3),
        },
        "coverage": metrics,
        "analysis": {
            "summary": analysis_result.analysis_summary,
            "source": analysis_result.analysis_source,
            "pid_headline": pid_report.headline if pid_report else "",
            "pid_highlights": list(pid_report.highlights) if pid_report else [],
            "pid_cli": list(pid_report.cli_commands) if pid_report else [],
            "warnings": list(analysis_result.warnings),
        },
        "log": {
            "csv_used": str(csv_path),
            "rows": int(data["time_s"].size),
            "duration_s": round(float(data["time_s"].max() - data["time_s"].min()), 3),
            "sample_rate_hz": round(float(data["sample_rate"]), 2),
            "peak_roll_error": round(float(np.max(np.abs(data["error"]))), 3),
        },
        "artifacts": {
            "combined_chart_sheet": str(combined_chart),
            "combined_chart_sheet_svg": str(combined_chart.with_suffix(".svg")),
            "charts": {name: str(path) for name, path in charts.items()},
            "charts_svg": {name: str(path.with_suffix(".svg")) for name, path in charts.items()},
            "render": {
                "png_dpi": _CHART_DPI,
                "combined_png_dpi": _COMBINED_CHART_DPI,
                "vector_sidecars": True,
            },
        },
    }


def _format_summary_text(payload: dict[str, Any]) -> str:
    lines: list[str] = []
    lines.append("Adaptive Auto Tune Summary")
    lines.append("=" * 28)
    lines.append(f"Generated: {payload.get('generated_at', '')}")
    session = payload.get("session", {})
    lines.append(f"State: {session.get('state', '')}")
    lines.append(f"Elapsed: {session.get('elapsed_s', 0.0)} s")
    if session.get("stop_reason"):
        lines.append(f"Stop reason: {session.get('stop_reason')}")
    if session.get("warning"):
        lines.append(f"Warning: {session.get('warning')}")

    coverage = payload.get("coverage", {})
    axis_conf = coverage.get("axis_confidence", {}) if isinstance(coverage, dict) else {}
    if axis_conf:
        lines.append(f"Axis confidence roll={axis_conf.get('roll', 0.0):.2f}, pitch={axis_conf.get('pitch', 0.0):.2f}")

    analysis = payload.get("analysis", {})
    lines.append("")
    lines.append("Analysis")
    lines.append("-" * 8)
    lines.append(str(analysis.get("summary", "")))
    pid_headline = str(analysis.get("pid_headline", "")).strip()
    if pid_headline:
        lines.append(pid_headline)
    for item in analysis.get("pid_highlights", [])[:8]:
        lines.append(f"- {item}")

    pid_cli = analysis.get("pid_cli", [])
    if pid_cli:
        lines.append("Suggested CLI")
        for command in pid_cli:
            lines.append(f"  {command}")

    warnings = analysis.get("warnings", [])
    if warnings:
        lines.append("Warnings")
        for warning in warnings[:5]:
            lines.append(f"- {warning}")

    artifacts = payload.get("artifacts", {})
    lines.append("")
    lines.append("Artifacts")
    lines.append("-" * 9)
    lines.append(f"Combined: {artifacts.get('combined_chart_sheet', '')}")
    combined_svg = artifacts.get("combined_chart_sheet_svg", "")
    if combined_svg:
        lines.append(f"Combined SVG: {combined_svg}")
    charts = artifacts.get("charts", {})
    if isinstance(charts, dict):
        for name, path in charts.items():
            lines.append(f"{name}: {path}")
    charts_svg = artifacts.get("charts_svg", {})
    if isinstance(charts_svg, dict):
        for name, path in charts_svg.items():
            lines.append(f"{name}_svg: {path}")
    render = artifacts.get("render", {})
    if isinstance(render, dict):
        lines.append(
            f"Render quality: PNG {render.get('png_dpi', '')} DPI, "
            f"combined PNG {render.get('combined_png_dpi', '')} DPI, "
            f"vector sidecars={render.get('vector_sidecars', False)}"
        )

    return "\n".join(lines).strip() + "\n"

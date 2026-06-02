"""Auto-tune summary report generation for blackbox sessions (HTML chart artifacts only)."""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from .blackbox_import import BlackboxImportResult
from .blackbox_trace_viewer import build_roll_trace_viewer


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
        raise RuntimeError("Could not resolve a primary Blackbox CSV for report generation.")

    time_s, cols = _read_blackbox_csv(csv_path)
    data = _extract_plot_columns(time_s, cols)

    trace_dir = report_dir / "charts"
    trace_viewer_html: Path | None = None
    trace_viewer_warning = ""
    try:
        trace_viewer_html = build_roll_trace_viewer(csv_path, trace_dir)
    except Exception as exc:
        trace_viewer_warning = str(exc)

    summary_txt = report_dir / "summary.txt"
    summary_json = report_dir / "summary.json"

    summary_payload = _build_summary_payload(
        analysis_result=analysis_result,
        session_payload=session_payload,
        csv_path=csv_path,
        data=data,
        trace_viewer_html=trace_viewer_html,
        trace_viewer_warning=trace_viewer_warning,
    )
    summary_txt.write_text(_format_summary_text(summary_payload), encoding="utf-8")
    summary_json.write_text(json.dumps(summary_payload, indent=2), encoding="utf-8")

    chart_paths = (str(trace_viewer_html),) if trace_viewer_html is not None else tuple()
    combined_artifact = str(trace_viewer_html) if trace_viewer_html is not None else ""
    return AutoTuneReport(
        report_dir=str(report_dir),
        summary_txt=str(summary_txt),
        summary_json=str(summary_json),
        combined_chart_sheet=combined_artifact,
        chart_paths=chart_paths,
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


def _looks_like_blackbox_header(raw_fields: list[str]) -> bool:
    if not raw_fields:
        return False
    fields = {_normalize_header(item) for item in raw_fields if item}
    if not fields:
        return False
    has_time = any(name in fields for name in ("time", "time(us)", "timeus", "loopiteration"))
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


def _read_blackbox_csv(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
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
        else:
            if not reader.fieldnames:
                raise RuntimeError("CSV header row missing.")
            headers = [h for h in reader.fieldnames if h is not None]
            normalized = {_normalize_header(h): h for h in headers}
            values = {key: [] for key in normalized}
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
    roll_actual = _column(columns, ["gyroADC[0]", "gyroRaw[0]"])
    if roll_setpoint is None or roll_actual is None:
        raise RuntimeError("Missing roll setpoint/actual columns in CSV.")

    roll_error = roll_setpoint - roll_actual
    sample_rate = _estimate_sample_rate(time_s)
    return {
        "time_s": time_s,
        "setpoint": roll_setpoint,
        "actual": roll_actual,
        "error": roll_error,
        "sample_rate": sample_rate,
    }


def _estimate_sample_rate(time_s: np.ndarray) -> float:
    if time_s.size < 3:
        return 1.0
    diffs = np.diff(time_s)
    diffs = diffs[diffs > 0]
    if diffs.size == 0:
        return 1.0
    return float(1.0 / np.median(diffs))


def _build_summary_payload(
    analysis_result: BlackboxImportResult,
    session_payload: dict[str, Any],
    csv_path: Path,
    data: dict[str, Any],
    trace_viewer_html: Path | None,
    trace_viewer_warning: str,
) -> dict[str, Any]:
    metrics = session_payload.get("metrics", {})
    pid_report = analysis_result.pid_report

    viewer_path = str(trace_viewer_html) if trace_viewer_html is not None else ""
    viewer_files = [viewer_path] if viewer_path else []

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
            "combined_chart_sheet": viewer_path,
            "trace_viewer": {
                "html": viewer_path,
                "files": viewer_files,
                "warning": trace_viewer_warning,
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
    trace_viewer = artifacts.get("trace_viewer", {})
    if isinstance(trace_viewer, dict):
        trace_html = str(trace_viewer.get("html", "")).strip()
        if trace_html:
            lines.append(f"Trace viewer: {trace_html}")
        trace_warning = str(trace_viewer.get("warning", "")).strip()
        if trace_warning:
            lines.append(f"Trace viewer warning: {trace_warning}")

    return "\n".join(lines).strip() + "\n"

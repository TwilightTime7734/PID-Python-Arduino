"""Blackbox log import helpers for MSC-mounted FC storage."""

from __future__ import annotations

import csv
import ctypes
import os
import re
import shutil
import string
import subprocess
import sys
import tempfile
import zlib
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path

from .pid_tuning import PIDAnalyzer, PIDRecommendation

WINDOWS_DRIVE_REMOVABLE = 2
WINDOWS_DRIVE_FIXED = 3
RAW_BLACKBOX_EXTENSIONS = {".txt", ".bbl", ".bfl", ".bbs"}
CSV_EXTENSIONS = {".csv"}
TOOLKIT_ANALYZE_EXTENSIONS = RAW_BLACKBOX_EXTENSIONS | CSV_EXTENSIONS
LOG_SEARCH_DIR_NAMES = {"blackbox", "logs", "log", "inav"}
REPO_ROOT = Path(__file__).resolve().parents[1]
LOCAL_DECODER_CANDIDATES = (
    REPO_ROOT / "tools" / "blackbox_decode_INAV.exe",
    REPO_ROOT / "tools" / "blackbox_decode_INAV",
    REPO_ROOT / "tools" / "blackbox_decode.exe",
    REPO_ROOT / "tools" / "blackbox_decode",
)
DECODER_FALLBACK_CANDIDATES = (
    Path(r"C:\Program Files\PIDtoolbox\application\blackbox_decode_INAV.exe"),
    Path(r"C:\Program Files\PIDtoolbox\application\blackbox_decode.exe"),
    Path(r"C:\Program Files (x86)\PIDtoolbox\application\blackbox_decode_INAV.exe"),
    Path(r"C:\Program Files (x86)\PIDtoolbox\application\blackbox_decode.exe"),
)
TOOLKIT_ROOT_CANDIDATES = (
    Path(os.environ.get("INAV_TOOLKIT_ROOT", "")).expanduser(),
    (Path(__file__).resolve().parents[2] / "INAV-Toolkit-main"),
    Path.cwd().parent / "INAV-Toolkit-main",
)
PID_PARAM_PATTERN = re.compile(r"^(roll|pitch|yaw)_(p|i|d|ff)$")
PID_PARAM_TO_SETTING = {
    "roll_p": "mc_p_roll",
    "roll_i": "mc_i_roll",
    "roll_d": "mc_d_roll",
    "roll_ff": "mc_cd_roll",
    "pitch_p": "mc_p_pitch",
    "pitch_i": "mc_i_pitch",
    "pitch_d": "mc_d_pitch",
    "pitch_ff": "mc_cd_pitch",
    "yaw_p": "mc_p_yaw",
    "yaw_i": "mc_i_yaw",
    "yaw_d": "mc_d_yaw",
    "yaw_ff": "mc_cd_yaw",
}
AXIS_ORDER = {"roll": 0, "pitch": 1, "yaw": 2}
GAIN_ORDER = {"p": 0, "i": 1, "d": 2, "ff": 3}


@dataclass(frozen=True)
class ImportedLogFile:
    source_path: str
    local_path: str
    file_size_bytes: int
    modified_epoch_s: float


@dataclass(frozen=True)
class BlackboxAnalysis:
    axis: str
    recommendation: PIDRecommendation


@dataclass(frozen=True)
class BlackboxPidChange:
    axis: str
    gain: str
    param: str
    source_action: str
    deferred: bool
    current_value: int | None = None
    recommended_value: int | None = None
    delta_percent: float | None = None


@dataclass(frozen=True)
class BlackboxPidReport:
    headline: str
    highlights: tuple[str, ...]
    advisory: tuple[str, ...]
    cli_commands: tuple[str, ...]
    changes: tuple[BlackboxPidChange, ...]


@dataclass(frozen=True)
class BlackboxImportResult:
    scanned_roots: tuple[str, ...]
    imported_files: tuple[ImportedLogFile, ...]
    skipped_count: int
    warnings: tuple[str, ...]
    analysis_summary: str
    analysis_source: str
    pid_report: BlackboxPidReport | None = None


def import_blackbox_logs_from_msc(destination_dir: str | Path) -> BlackboxImportResult:
    destination = Path(destination_dir)
    destination.mkdir(parents=True, exist_ok=True)

    roots = _candidate_msc_roots()
    warnings: list[str] = []
    imported: list[ImportedLogFile] = []
    skipped = 0

    if not roots:
        return BlackboxImportResult(
            scanned_roots=tuple(),
            imported_files=tuple(),
            skipped_count=0,
            warnings=("No mounted MSC-style drives were detected.",),
            analysis_summary="No log analysis available.",
            analysis_source="",
        )

    for root in roots:
        candidates = _discover_blackbox_files(root)
        for src in candidates:
            try:
                copied = _copy_with_dedup(src, destination)
                if copied is None:
                    skipped += 1
                    continue
                stat = copied.stat()
                imported.append(
                    ImportedLogFile(
                        source_path=str(src),
                        local_path=str(copied),
                        file_size_bytes=int(stat.st_size),
                        modified_epoch_s=float(stat.st_mtime),
                    )
                )
            except Exception as exc:
                warnings.append(f"Copy failed for '{src}': {exc}")

    analysis_candidates = [Path(item.local_path) for item in imported]
    if not analysis_candidates:
        analysis_candidates = _collect_local_log_candidates(destination)

    analysis_summary = "No parsed CSV logs were found."
    analysis_source = ""
    pid_report: BlackboxPidReport | None = None
    toolkit_summary, toolkit_source, toolkit_warnings, toolkit_report = _summarize_with_inav_toolkit(analysis_candidates)
    warnings.extend(toolkit_warnings)
    if toolkit_summary and toolkit_source:
        analysis_summary = toolkit_summary
        pid_report = toolkit_report
        chart_csv = _resolve_chart_csv_source([Path(toolkit_source), *analysis_candidates], destination, warnings)
        analysis_source = str(chart_csv) if chart_csv is not None else toolkit_source
    else:
        csv_sources = _collect_csv_candidates_from_paths(analysis_candidates)
        if not csv_sources:
            raw_sources = [p for p in analysis_candidates if p.suffix.lower() in RAW_BLACKBOX_EXTENSIONS]
            decoded_csv, decode_warnings = _decode_raw_logs(raw_sources, destination)
            warnings.extend(decode_warnings)
            if decoded_csv:
                csv_sources = decoded_csv
        if csv_sources:
            ranked_csv = sorted(csv_sources, key=lambda p: p.stat().st_mtime if p.exists() else 0.0, reverse=True)
            for candidate_csv in ranked_csv:
                try:
                    analysis_summary, pid_report = analyze_blackbox_csv(candidate_csv)
                    analysis_source = str(candidate_csv)
                    break
                except Exception as exc:
                    warnings.append(f"CSV analysis failed for '{candidate_csv.name}': {exc}")
            if not analysis_source:
                analysis_summary = "CSV log found, but analysis failed."
                analysis_source = str(ranked_csv[0]) if ranked_csv else ""
        else:
            if imported:
                warnings.append(
                    "Raw Blackbox logs were imported. To enable CSV fallback analysis, decode/export to CSV then re-import."
                )
            elif skipped > 0:
                warnings.append(
                    "No new files were copied (duplicates skipped). Existing local logs were checked for analysis."
                )
            else:
                warnings.append("No Blackbox log files were found on detected MSC volumes.")

    return BlackboxImportResult(
        scanned_roots=tuple(str(p) for p in roots),
        imported_files=tuple(imported),
        skipped_count=skipped,
        warnings=tuple(warnings),
        analysis_summary=analysis_summary,
        analysis_source=analysis_source,
        pid_report=pid_report,
    )


def analyze_pulled_blackbox_logs(destination_dir: str | Path) -> BlackboxImportResult:
    destination = Path(destination_dir)
    destination.mkdir(parents=True, exist_ok=True)

    warnings: list[str] = []
    analysis_candidates = _collect_local_log_candidates(destination)
    if not analysis_candidates:
        return BlackboxImportResult(
            scanned_roots=(str(destination),),
            imported_files=tuple(),
            skipped_count=0,
            warnings=(f"No local Blackbox logs were found in {destination}.",),
            analysis_summary="No log analysis available.",
            analysis_source="",
        )

    analysis_summary = "No parsed CSV logs were found."
    analysis_source = ""
    pid_report: BlackboxPidReport | None = None
    toolkit_summary, toolkit_source, toolkit_warnings, toolkit_report = _summarize_with_inav_toolkit(analysis_candidates)
    warnings.extend(toolkit_warnings)
    if toolkit_summary and toolkit_source:
        analysis_summary = toolkit_summary
        pid_report = toolkit_report
        chart_csv = _resolve_chart_csv_source([Path(toolkit_source), *analysis_candidates], destination, warnings)
        analysis_source = str(chart_csv) if chart_csv is not None else toolkit_source
    else:
        csv_sources = _collect_csv_candidates_from_paths(analysis_candidates)
        if not csv_sources:
            raw_sources = [p for p in analysis_candidates if p.suffix.lower() in RAW_BLACKBOX_EXTENSIONS]
            decoded_csv, decode_warnings = _decode_raw_logs(raw_sources, destination)
            warnings.extend(decode_warnings)
            if decoded_csv:
                csv_sources = decoded_csv
        if csv_sources:
            ranked_csv = sorted(csv_sources, key=lambda p: p.stat().st_mtime if p.exists() else 0.0, reverse=True)
            for candidate_csv in ranked_csv:
                try:
                    analysis_summary, pid_report = analyze_blackbox_csv(candidate_csv)
                    analysis_source = str(candidate_csv)
                    break
                except Exception as exc:
                    warnings.append(f"CSV analysis failed for '{candidate_csv.name}': {exc}")
            if not analysis_source:
                analysis_summary = "CSV log found, but analysis failed."
                analysis_source = str(ranked_csv[0]) if ranked_csv else ""
        else:
            warnings.append(f"No analyzable local Blackbox logs were found in {destination}.")

    return BlackboxImportResult(
        scanned_roots=(str(destination),),
        imported_files=tuple(),
        skipped_count=0,
        warnings=tuple(warnings),
        analysis_summary=analysis_summary,
        analysis_source=analysis_source,
        pid_report=pid_report,
    )


def analyze_blackbox_log(log_path: str | Path, decode_destination_dir: str | Path | None = None) -> BlackboxImportResult:
    path = Path(log_path)
    if not path.exists() or not path.is_file():
        return BlackboxImportResult(
            scanned_roots=(str(path.parent),),
            imported_files=tuple(),
            skipped_count=0,
            warnings=(f"Log path does not exist: {path}",),
            analysis_summary="No log analysis available.",
            analysis_source="",
            pid_report=None,
        )

    decode_destination = Path(decode_destination_dir) if decode_destination_dir else path.parent
    decode_destination.mkdir(parents=True, exist_ok=True)

    warnings: list[str] = []
    analysis_summary = "No parsed CSV logs were found."
    analysis_source = ""
    pid_report: BlackboxPidReport | None = None

    toolkit_summary, toolkit_source, toolkit_warnings, toolkit_report = _summarize_with_inav_toolkit([path])
    warnings.extend(toolkit_warnings)
    if toolkit_summary and toolkit_source:
        analysis_summary = toolkit_summary
        pid_report = toolkit_report
        chart_csv = _resolve_chart_csv_source([Path(toolkit_source), path], decode_destination, warnings)
        analysis_source = str(chart_csv) if chart_csv is not None else toolkit_source
    else:
        csv_sources: list[Path] = []
        if path.suffix.lower() in CSV_EXTENSIONS and path.exists():
            csv_sources = [path]
        elif path.suffix.lower() in RAW_BLACKBOX_EXTENSIONS:
            decoded_csv, decode_warnings = _decode_raw_logs([path], decode_destination)
            warnings.extend(decode_warnings)
            csv_sources = decoded_csv

        if csv_sources:
            ranked_csv = sorted(csv_sources, key=lambda p: p.stat().st_mtime if p.exists() else 0.0, reverse=True)
            for candidate_csv in ranked_csv:
                try:
                    analysis_summary, pid_report = analyze_blackbox_csv(candidate_csv)
                    analysis_source = str(candidate_csv)
                    break
                except Exception as exc:
                    warnings.append(f"CSV analysis failed for '{candidate_csv.name}': {exc}")
            if not analysis_source and ranked_csv:
                analysis_summary = "CSV log found, but analysis failed."
                analysis_source = str(ranked_csv[0])
        else:
            warnings.append(f"No analyzable data could be derived from {path}.")

    return BlackboxImportResult(
        scanned_roots=(str(path.parent),),
        imported_files=tuple(),
        skipped_count=0,
        warnings=tuple(warnings),
        analysis_summary=analysis_summary,
        analysis_source=analysis_source,
        pid_report=pid_report,
    )


def _summarize_with_inav_toolkit(
    candidates: list[Path],
) -> tuple[str, str, list[str], BlackboxPidReport | None]:
    analyzer, load_warnings = _load_toolkit_analyzer_with_warnings()
    if analyzer is None:
        return "", "", load_warnings, None

    warnings: list[str] = []
    filtered = [p for p in candidates if p.exists() and p.suffix.lower() in TOOLKIT_ANALYZE_EXTENSIONS]
    if not filtered:
        return "", "", load_warnings, None

    for path in sorted(filtered, key=lambda p: p.stat().st_mtime, reverse=True)[:3]:
        try:
            summary, pid_report = analyze_blackbox_with_toolkit(path, analyzer)
            return summary, str(path), warnings + load_warnings, pid_report
        except Exception as exc:
            warnings.append(f"INAV-Toolkit analysis failed for '{path.name}': {exc}")
    return "", "", warnings + load_warnings, None


def summarize_blackbox_with_toolkit(log_path: str | Path, analyzer) -> str:
    summary, _ = analyze_blackbox_with_toolkit(log_path, analyzer)
    return summary


def analyze_blackbox_with_toolkit(log_path: str | Path, analyzer) -> tuple[str, BlackboxPidReport]:
    path = Path(log_path)
    ext = path.suffix.lower()
    if ext not in TOOLKIT_ANALYZE_EXTENSIONS:
        raise RuntimeError(f"Unsupported extension: {ext}")

    config: dict[str, object] = {}
    if ext in RAW_BLACKBOX_EXTENSIONS:
        raw_params = analyzer.parse_headers_from_bbl(str(path))
        config = analyzer.extract_fc_config(raw_params)
        data = analyzer.decode_blackbox_native(str(path), raw_params, quiet=True)
    else:
        data = analyzer.parse_csv_log(str(path))

    profile = analyzer.get_frame_profile(5, 5, 3)
    sample_rate = float(data.get("sample_rate", 0.0))
    if sample_rate <= 0:
        raise RuntimeError("Decoded log has invalid sample rate.")

    noise_results = [analyzer.analyze_noise(data, ax, f"gyro_{ax.lower()}", sample_rate) for ax in analyzer.AXIS_NAMES]
    pid_results = [analyzer.analyze_pid_response(data, i, sample_rate) for i in range(3)]
    motor_analysis = analyzer.analyze_motors(data, sample_rate, config)
    dterm_results = analyzer.analyze_dterm_noise(data, sample_rate)
    motor_response = analyzer.analyze_motor_response(data, sample_rate)
    hover_osc = None
    if hasattr(analyzer, "detect_hover_oscillation"):
        hover_osc = analyzer.detect_hover_oscillation(data, sample_rate, profile)

    phase_lag = None
    if analyzer.config_has_filters(config):
        signal_freq = (profile["noise_band_mid"][0] + profile["noise_band_mid"][1]) / 4
        phase_lag = analyzer.estimate_total_phase_lag(config, profile, signal_freq)

    plan = analyzer.generate_action_plan(
        noise_results,
        pid_results,
        motor_analysis,
        dterm_results,
        config,
        data,
        profile,
        phase_lag,
        motor_response,
        None,
        None,
        hover_osc,
    )
    pid_report = _build_toolkit_pid_report(plan, config)
    return _format_toolkit_summary(plan, pid_results, pid_report), pid_report


def _load_toolkit_analyzer_with_warnings() -> tuple[object | None, list[str]]:
    warnings: list[str] = []
    try:
        return import_module("inav_toolkit.blackbox_analyzer"), warnings
    except Exception as exc:
        warnings.append(
            f"INAV-Toolkit import failed in '{sys.executable}': {type(exc).__name__}: {exc}"
        )
        pass

    for root in TOOLKIT_ROOT_CANDIDATES:
        if not root:
            continue
        root = root.resolve()
        package_dir = root / "inav_toolkit"
        if not package_dir.exists():
            continue
        root_str = str(root)
        if root_str not in sys.path:
            sys.path.insert(0, root_str)
        try:
            # Local toolkit path resolved successfully; suppress earlier
            # "not installed as package" warnings.
            return import_module("inav_toolkit.blackbox_analyzer"), []
        except Exception as exc:
            warnings.append(
                f"INAV-Toolkit import failed from '{root}': {type(exc).__name__}: {exc}"
            )
            continue

    warnings.append(
        "INAV-Toolkit unavailable; install 'scipy' into the same Python interpreter used by this app."
    )
    return None, warnings


def _format_toolkit_summary(
    plan: dict[str, object],
    pid_results: list[object],
    pid_report: BlackboxPidReport,
) -> str:
    verdict_text = str(plan.get("verdict_text") or plan.get("verdict") or "No verdict")
    scores = plan.get("scores")
    overall = None
    if isinstance(scores, dict):
        overall = scores.get("overall")

    parts: list[str] = []
    if isinstance(overall, (int, float)):
        parts.append(f"INAV-Toolkit: {overall:.0f}/100 ({verdict_text})")
    else:
        parts.append(f"INAV-Toolkit: {verdict_text}")

    axis_parts: list[str] = []
    for result in pid_results:
        if not isinstance(result, dict):
            continue
        axis = result.get("axis")
        overshoot = result.get("avg_overshoot_pct")
        delay = result.get("tracking_delay_ms")
        if not isinstance(axis, str):
            continue
        detail_parts: list[str] = []
        if isinstance(overshoot, (int, float)):
            detail_parts.append(f"OS {overshoot:.0f}%")
        if isinstance(delay, (int, float)):
            detail_parts.append(f"Delay {delay:.0f}ms")
        if detail_parts:
            axis_parts.append(f"{axis}: {', '.join(detail_parts)}")
    if axis_parts:
        parts.append("PID " + "; ".join(axis_parts))

    if pid_report.highlights:
        parts.append("PID plan: " + "; ".join(_compact_text(line) for line in pid_report.highlights[:3]))
    elif pid_report.headline:
        parts.append(_compact_text(pid_report.headline))

    if pid_report.advisory:
        parts.append("Notes: " + "; ".join(_compact_text(line) for line in pid_report.advisory[:2]))

    actions = plan.get("actions")
    if isinstance(actions, list):
        active: list[dict[str, object]] = []
        deferred_count = 0
        for action in actions:
            if not isinstance(action, dict):
                continue
            if action.get("deferred"):
                deferred_count += 1
                continue
            active.append(action)

        if active:
            snippets: list[str] = []
            for action in active[:3]:
                text = str(action.get("action") or "").strip()
                if text:
                    snippets.append(_compact_text(text))
            if snippets:
                parts.append("Top changes: " + "; ".join(snippets))
        elif deferred_count > 0:
            parts.append("No immediate changes; deferred actions remain after re-flight.")

    return " | ".join(parts)


def _build_toolkit_pid_report(plan: dict[str, object], config: dict[str, object]) -> BlackboxPidReport:
    actions = plan.get("actions")
    if not isinstance(actions, list):
        return BlackboxPidReport(
            headline="No actionable PID recommendations were generated.",
            highlights=tuple(),
            advisory=tuple(),
            cli_commands=tuple(),
            changes=tuple(),
        )

    selected_changes: dict[str, tuple[int, bool, BlackboxPidChange]] = {}
    advisory: list[str] = []
    text_only_pid_hints: list[str] = []

    for raw_action in actions:
        if not isinstance(raw_action, dict):
            continue
        action = raw_action
        action_text = str(action.get("action") or "").strip()
        category = str(action.get("category") or "").strip()
        deferred = bool(action.get("deferred"))
        priority_raw = action.get("priority")
        priority = int(priority_raw) if isinstance(priority_raw, (int, float)) else 999

        if category in ("Filter", "Mechanical", "Motor", "Phase Lag") and action_text and not deferred:
            advisory.append(_compact_text(action_text, max_len=120))

        sub_actions = action.get("sub_actions")
        parsed_any = False
        if isinstance(sub_actions, list):
            for raw_sub in sub_actions:
                if not isinstance(raw_sub, dict):
                    continue
                sub = raw_sub
                parsed = _parse_pid_param(str(sub.get("param") or "").strip().lower())
                if parsed is None:
                    continue
                axis, gain = parsed
                param = f"{axis}_{gain}"
                recommended = _coerce_int(sub.get("new"))
                if recommended is None:
                    continue
                current = _coerce_int(sub.get("current"))
                if current is None:
                    current = _coerce_int(config.get(param))
                delta_percent = None
                if current not in (None, 0):
                    delta_percent = ((recommended - current) / current) * 100.0

                candidate = BlackboxPidChange(
                    axis=axis,
                    gain=gain,
                    param=param,
                    source_action=action_text,
                    deferred=deferred,
                    current_value=current,
                    recommended_value=recommended,
                    delta_percent=delta_percent,
                )
                existing = selected_changes.get(param)
                if existing is None or _prefer_pid_change(existing, (priority, deferred, candidate)):
                    selected_changes[param] = (priority, deferred, candidate)
                parsed_any = True

        if not parsed_any and category in ("PID", "Oscillation") and action_text:
            text_only_pid_hints.append(_compact_text(action_text, max_len=120))

    changes = sorted((entry[2] for entry in selected_changes.values()), key=_pid_change_sort_key)
    highlights = [_format_pid_change(change) for change in changes if not change.deferred]
    deferred_highlights = [_format_pid_change(change) for change in changes if change.deferred]

    if not highlights and text_only_pid_hints:
        highlights.extend(text_only_pid_hints[:3])

    if deferred_highlights:
        advisory.append(
            "Some PID changes are deferred; fix filter/mechanical items first, then re-fly and re-analyze."
        )
        advisory.extend(f"Deferred: {line}" for line in deferred_highlights[:3])

    if highlights:
        headline = "Recommended PID updates from Blackbox analysis."
    elif text_only_pid_hints:
        headline = "PID actions were detected, but exact value deltas were unavailable."
    else:
        headline = "No direct PID changes generated from this Blackbox log."

    cli_commands: list[str] = []
    for change in changes:
        if change.deferred:
            continue
        if change.recommended_value is None:
            continue
        setting_name = PID_PARAM_TO_SETTING.get(change.param)
        if not setting_name:
            continue
        cli_commands.append(f"set {setting_name} = {change.recommended_value}")
    if cli_commands:
        cli_commands.append("save")

    return BlackboxPidReport(
        headline=headline,
        highlights=tuple(_dedupe_keep_order(highlights)),
        advisory=tuple(_dedupe_keep_order(advisory)),
        cli_commands=tuple(_dedupe_keep_order(cli_commands)),
        changes=tuple(changes),
    )


def _build_csv_pid_report(axis_summaries: list[BlackboxAnalysis]) -> BlackboxPidReport:
    by_param: dict[str, BlackboxPidChange] = {}
    for entry in axis_summaries:
        axis = entry.axis.lower()
        rec = entry.recommendation
        for gain, delta_percent, reason in _csv_pid_change_hints(rec):
            param = f"{axis}_{gain}"
            change = BlackboxPidChange(
                axis=axis,
                gain=gain,
                param=param,
                source_action=reason,
                deferred=False,
                current_value=None,
                recommended_value=None,
                delta_percent=delta_percent,
            )
            existing = by_param.get(param)
            if existing is None:
                by_param[param] = change
            elif (existing.delta_percent or 0.0) == 0.0 or abs(delta_percent) > abs(existing.delta_percent or 0.0):
                by_param[param] = change

    changes = sorted(by_param.values(), key=_pid_change_sort_key)
    highlights = [_format_pid_change(change) for change in changes]
    advisory: list[str] = []
    if not highlights:
        advisory.append("No strong PID change was inferred from the available stick events in this CSV log.")
        headline = "PID signal is limited in this CSV log; collect stronger stick steps and re-log."
    else:
        headline = "Fallback CSV analysis produced conservative PID percent adjustments."
        advisory.append("Adjust one gain at a time, test-fly, and re-analyze before stacking more changes.")

    return BlackboxPidReport(
        headline=headline,
        highlights=tuple(highlights),
        advisory=tuple(advisory),
        cli_commands=tuple(),
        changes=tuple(changes),
    )


def _csv_pid_change_hints(recommendation: PIDRecommendation) -> list[tuple[str, float, str]]:
    text = recommendation.recommendation.lower()
    if "oscillation is high" in text:
        return [
            ("p", -6.0, "Oscillation and/or overshoot is high in stick response."),
            ("d", 6.0, "Add D after reducing P to damp oscillation."),
        ]
    if "slow to settle" in text:
        return [("p", 6.0, "Response is slow to settle.")]
    if "steady-state error remains" in text:
        return [("i", 6.0, "Steady-state error remains after movement.")]
    if "overshoot is visible" in text:
        return [
            ("d", 6.0, "Overshoot is visible in the response trace."),
            ("p", -3.0, "Trim P slightly while adding D."),
        ]
    if "small p increase" in text:
        return [("p", 3.0, "Response is stable; small P increase can improve tracking speed.")]
    return []


def _prefer_pid_change(
    existing: tuple[int, bool, BlackboxPidChange],
    candidate: tuple[int, bool, BlackboxPidChange],
) -> bool:
    existing_priority, existing_deferred, _ = existing
    candidate_priority, candidate_deferred, _ = candidate

    if existing_deferred and not candidate_deferred:
        return True
    if not existing_deferred and candidate_deferred:
        return False
    if candidate_priority < existing_priority:
        return True
    return False


def _pid_change_sort_key(change: BlackboxPidChange) -> tuple[int, int]:
    return (AXIS_ORDER.get(change.axis, 99), GAIN_ORDER.get(change.gain, 99))


def _format_pid_change(change: BlackboxPidChange) -> str:
    axis_label = change.axis.title()
    gain_label = change.gain.upper()
    prefix = "[DEFERRED] " if change.deferred else ""

    if change.current_value is not None and change.recommended_value is not None:
        base = f"{axis_label} {gain_label} {change.current_value} -> {change.recommended_value}"
    elif change.recommended_value is not None:
        base = f"{axis_label} {gain_label} -> {change.recommended_value}"
    elif change.delta_percent is not None:
        base = f"{axis_label} {gain_label} {change.delta_percent:+.0f}%"
    else:
        base = f"{axis_label} {gain_label}"

    if change.delta_percent is not None and change.recommended_value is not None and change.current_value is not None:
        base += f" ({change.delta_percent:+.0f}%)"
    return prefix + base


def _parse_pid_param(param: str) -> tuple[str, str] | None:
    match = PID_PARAM_PATTERN.fullmatch(param)
    if match is None:
        return None
    return (match.group(1), match.group(2))


def _coerce_int(value: object) -> int | None:
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return int(round(float(value)))
    text = str(value).strip()
    if not text:
        return None
    try:
        return int(round(float(text)))
    except Exception:
        return None


def _dedupe_keep_order(values: list[str]) -> list[str]:
    out: list[str] = []
    seen: set[str] = set()
    for value in values:
        key = value.strip()
        if not key or key in seen:
            continue
        seen.add(key)
        out.append(value)
    return out


def _compact_text(text: str, max_len: int = 96) -> str:
    if len(text) <= max_len:
        return text
    if max_len <= 3:
        return text[:max_len]
    return text[: max_len - 3].rstrip() + "..."


def summarize_blackbox_csv(csv_path: str | Path) -> str:
    summary, _ = analyze_blackbox_csv(csv_path)
    return summary


def analyze_blackbox_csv(csv_path: str | Path) -> tuple[str, BlackboxPidReport]:
    path = Path(csv_path)
    times, columns = _read_numeric_csv(path)
    if len(times) < 30:
        raise RuntimeError("CSV does not contain enough samples.")

    axis_summaries: list[BlackboxAnalysis] = []
    for axis_name, axis_index in (("roll", 0), ("pitch", 1)):
        command = _choose_column(columns, [f"setpoint[{axis_index}]", f"rcCommand[{axis_index}]"])
        measured = _choose_column(columns, [f"gyroADC[{axis_index}]"])
        if command is None or measured is None:
            continue
        if len(command) != len(times) or len(measured) != len(times):
            continue
        recommendation = _analyze_axis(times, command, measured)
        if recommendation is None:
            continue
        axis_summaries.append(BlackboxAnalysis(axis=axis_name, recommendation=recommendation))

    if not axis_summaries:
        raise RuntimeError("CSV is missing expected setpoint/measured columns for roll/pitch.")

    pid_report = _build_csv_pid_report(axis_summaries)
    lines: list[str] = []
    by_axis = {entry.axis: entry.recommendation for entry in axis_summaries}
    for axis in ("roll", "pitch"):
        rec = by_axis.get(axis)
        if rec is None:
            continue
        settle = f"{rec.settling_time:.2f}s" if rec.settling_time is not None else "not settled"
        lines.append(
            f"{axis.title()}: {rec.recommendation} (Osc {rec.oscillation_count}, Settle {settle}, Over {rec.overshoot_ratio:.2f})"
        )

    if "roll" in by_axis and "pitch" in by_axis:
        max_osc = max(by_axis["roll"].oscillation_count, by_axis["pitch"].oscillation_count)
        max_over = max(by_axis["roll"].overshoot_ratio, by_axis["pitch"].overshoot_ratio)
        if max_osc >= 4:
            lines.append("Combined: oscillation risk is elevated across axes; reduce P slightly and re-test.")
        elif max_over > 0.2:
            lines.append("Combined: overshoot is visible; increase D in small steps before other changes.")

    if pid_report.headline:
        lines.append(pid_report.headline)
    if pid_report.advisory:
        lines.append("Notes: " + "; ".join(pid_report.advisory[:2]))

    return " | ".join(lines), pid_report


def _analyze_axis(times_s: list[float], command_raw: list[float], measured_raw: list[float]) -> PIDRecommendation | None:
    if len(times_s) < 30:
        return None

    command = _smooth_series(command_raw, window=3)
    measured = _smooth_series(measured_raw, window=3)
    command_peak = max(abs(v) for v in command) if command else 0.0
    measured_peak = max(abs(v) for v in measured) if measured else 0.0
    if command_peak < 1e-6 or measured_peak < 1e-6:
        return None

    scale = measured_peak / command_peak
    target_series = [value * scale for value in command]

    # Use the strongest stick-rate segment for a first-pass recommendation.
    center = _median(command)
    deviations = [abs(v - center) for v in command]
    peak_index = max(range(len(deviations)), key=deviations.__getitem__)
    if deviations[peak_index] < max(20.0, command_peak * 0.15):
        return None

    window_samples = min(240, max(80, len(times_s) // 6))
    start = max(0, peak_index - window_samples // 4)
    end = min(len(times_s), start + window_samples)
    if end - start < 20:
        return None

    segment_times = times_s[start:end]
    t0 = segment_times[0]
    segment_times = [t - t0 for t in segment_times]
    segment_measured = measured[start:end]
    segment_target = target_series[start:end]
    segment_target_level = _mean(segment_target[-min(15, len(segment_target)) :])

    baseline = _mean(segment_measured[: min(15, len(segment_measured))])
    movement = [value - baseline for value in segment_measured]
    target = segment_target_level - baseline

    if abs(target) < max(8.0, measured_peak * 0.08):
        return None

    analyzer = PIDAnalyzer()
    return analyzer.analyze(segment_times, movement, target)


def _smooth_series(values: list[float], window: int = 3) -> list[float]:
    if window <= 1 or len(values) < window:
        return values[:]
    out: list[float] = []
    half = window // 2
    for i in range(len(values)):
        left = max(0, i - half)
        right = min(len(values), i + half + 1)
        out.append(_mean(values[left:right]))
    return out


def _discover_blackbox_files(root: Path) -> list[Path]:
    out: list[Path] = []
    try:
        entries = list(root.iterdir())
    except Exception:
        return out

    for entry in entries:
        if entry.is_file():
            suffix = entry.suffix.lower()
            if suffix in RAW_BLACKBOX_EXTENSIONS or suffix in CSV_EXTENSIONS:
                out.append(entry)
            continue
        if not entry.is_dir():
            continue
        if entry.name.lower() not in LOG_SEARCH_DIR_NAMES:
            continue
        out.extend(_walk_for_logs(entry))
    return sorted(set(out), key=lambda p: p.name.lower())


def _walk_for_logs(folder: Path) -> list[Path]:
    found: list[Path] = []
    stack: list[tuple[Path, int]] = [(folder, 0)]
    while stack:
        current, depth = stack.pop()
        if depth > 3:
            continue
        try:
            children = list(current.iterdir())
        except Exception:
            continue
        for child in children:
            if child.is_file():
                suffix = child.suffix.lower()
                if suffix in RAW_BLACKBOX_EXTENSIONS or suffix in CSV_EXTENSIONS:
                    found.append(child)
            elif child.is_dir():
                stack.append((child, depth + 1))
    return found


def _copy_with_dedup(src: Path, destination: Path) -> Path | None:
    if not src.exists() or not src.is_file():
        return None

    safe_name = _sanitize_filename(src.name)
    target = destination / safe_name
    src_size = src.stat().st_size

    if target.exists():
        try:
            dst_stat = target.stat()
            if dst_stat.st_size == src_size:
                src_crc32 = _crc32_file(src)
                dst_crc32 = _crc32_file(target)
                if src_crc32 == dst_crc32:
                    return None
        except Exception:
            pass
        target = _next_available_filename(destination, safe_name)

    shutil.copy2(src, target)
    return target


def _collect_csv_candidates_from_paths(paths: list[Path]) -> list[Path]:
    preferred_csv: list[Path] = []
    auxiliary_csv: list[Path] = []
    for local in paths:
        if local.suffix.lower() in CSV_EXTENSIONS and local.exists():
            if _is_auxiliary_csv(local):
                auxiliary_csv.append(local)
            else:
                preferred_csv.append(local)
            continue
        sibling_glob = list(local.parent.glob(f"{local.stem}*.csv"))
        for p in sibling_glob:
            if not p.is_file():
                continue
            if _is_auxiliary_csv(p):
                auxiliary_csv.append(p)
            else:
                preferred_csv.append(p)

    preferred_unique = {p.resolve() for p in preferred_csv if p.exists()}
    auxiliary_unique = {p.resolve() for p in auxiliary_csv if p.exists()}

    ranked_preferred = sorted(preferred_unique, key=lambda p: p.stat().st_mtime if p.exists() else 0.0)
    if ranked_preferred:
        return ranked_preferred
    return sorted(auxiliary_unique, key=lambda p: p.stat().st_mtime if p.exists() else 0.0)


def _resolve_chart_csv_source(
    paths: list[Path],
    decode_destination: Path | None,
    warnings: list[str],
) -> Path | None:
    csv_sources = _collect_csv_candidates_from_paths(paths)
    if csv_sources:
        return max(csv_sources, key=lambda p: p.stat().st_mtime if p.exists() else 0.0)

    if decode_destination is None:
        return None

    raw_sources = [
        candidate
        for candidate in paths
        if candidate.exists() and candidate.is_file() and candidate.suffix.lower() in RAW_BLACKBOX_EXTENSIONS
    ]
    if not raw_sources:
        return None

    decoded_csv, decode_warnings = _decode_raw_logs(raw_sources, decode_destination)
    warnings.extend(decode_warnings)
    if not decoded_csv:
        return None
    return max(decoded_csv, key=lambda p: p.stat().st_mtime if p.exists() else 0.0)


def _is_auxiliary_csv(path: Path) -> bool:
    name = path.name.lower()
    return name.endswith(".gps.csv") or name.endswith(".event.csv") or name.endswith(".events.csv")


def _collect_local_log_candidates(destination: Path) -> list[Path]:
    out: list[Path] = []
    try:
        for candidate in destination.iterdir():
            if not candidate.is_file():
                continue
            if candidate.suffix.lower() in TOOLKIT_ANALYZE_EXTENSIONS:
                out.append(candidate)
    except Exception:
        return []
    return sorted(out, key=lambda p: p.stat().st_mtime if p.exists() else 0.0, reverse=True)


def _decode_raw_logs(raw_paths: list[Path], destination: Path) -> tuple[list[Path], list[str]]:
    if not raw_paths:
        return [], []

    decoder = _find_blackbox_decoder()
    if decoder is None:
        explorer_root = Path(r"C:\Program Files (x86)\INAV\INAV-BlackboxExplorer")
        has_explorer_decoder_js = (explorer_root / "js" / "decoders.js").exists()
        if has_explorer_decoder_js:
            return [], [
                "INAV Blackbox Explorer is installed, but it does not ship blackbox_decode.exe; "
                "raw logs were copied only."
            ]
        return [], [
            "No usable blackbox decoder was found (PATH or PIDtoolbox fallback paths); "
            "raw logs were copied only."
        ]

    warnings: list[str] = []
    decoded: list[Path] = []
    for raw in raw_paths:
        raw = raw.resolve()
        before = {p.resolve() for p in destination.glob(f"{raw.stem}*.csv")}
        try:
            completed = subprocess.run(
                [str(decoder), str(raw)],
                cwd=str(destination),
                capture_output=True,
                text=True,
                timeout=90,
                check=False,
            )
        except Exception as exc:
            warnings.append(f"Decode failed for '{raw.name}': {exc}")
            continue

        after = {p.resolve() for p in destination.glob(f"{raw.stem}*.csv")}
        new_files = sorted(after - before)
        if new_files:
            decoded.extend(Path(p) for p in new_files)
            continue

        if completed.returncode == 0 and after:
            decoded.extend(Path(p) for p in sorted(after))
            continue

        stderr_line = (completed.stderr or "").strip().splitlines()
        err_msg = stderr_line[-1] if stderr_line else f"exit code {completed.returncode}"
        warnings.append(f"Decode failed for '{raw.name}': {err_msg}")

    unique = sorted({p.resolve() for p in decoded}, key=lambda p: p.stat().st_mtime if p.exists() else 0.0)
    return [Path(p) for p in unique], warnings


def _find_blackbox_decoder() -> Path | None:
    for candidate in LOCAL_DECODER_CANDIDATES:
        if candidate.exists():
            return candidate

    which_candidates = (
        "blackbox_decode_INAV.exe",
        "blackbox_decode_INAV",
        "blackbox_decode.exe",
        "blackbox_decode",
    )
    for name in which_candidates:
        resolved = shutil.which(name)
        if resolved:
            path = Path(resolved)
            if path.exists():
                return path

    for candidate in DECODER_FALLBACK_CANDIDATES:
        if candidate.exists():
            return candidate
    return None


def _candidate_msc_roots() -> list[Path]:
    if not hasattr(ctypes, "windll"):
        return []

    kernel32 = ctypes.windll.kernel32  # type: ignore[attr-defined]
    roots: list[Path] = []
    for letter in string.ascii_uppercase:
        drive = Path(f"{letter}:\\")
        if not drive.exists():
            continue
        drive_type = int(kernel32.GetDriveTypeW(ctypes.c_wchar_p(str(drive))))
        if drive_type == WINDOWS_DRIVE_REMOVABLE:
            roots.append(drive)
            continue
        if drive_type == WINDOWS_DRIVE_FIXED and _looks_like_fc_volume(str(drive), kernel32):
            roots.append(drive)
    return roots


def _looks_like_fc_volume(root: str, kernel32) -> bool:
    label_buffer = ctypes.create_unicode_buffer(261)
    filesystem_buffer = ctypes.create_unicode_buffer(261)
    serial = ctypes.c_uint32(0)
    max_component = ctypes.c_uint32(0)
    flags = ctypes.c_uint32(0)
    ok = kernel32.GetVolumeInformationW(
        ctypes.c_wchar_p(root),
        label_buffer,
        len(label_buffer),
        ctypes.byref(serial),
        ctypes.byref(max_component),
        ctypes.byref(flags),
        filesystem_buffer,
        len(filesystem_buffer),
    )
    if not ok:
        return False
    label = label_buffer.value.lower()
    fs_name = filesystem_buffer.value.lower()
    return any(token in label for token in ("inav", "blackbox", "fc", "flight")) or fs_name in ("fat", "fat32", "exfat")


def _read_numeric_csv(path: Path) -> tuple[list[float], dict[str, list[float]]]:
    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        sample = f.read(4096)
        f.seek(0)
        dialect = csv.excel
        if sample.strip():
            try:
                dialect = csv.Sniffer().sniff(sample)
            except Exception:
                dialect = csv.excel
        reader = csv.DictReader(f, dialect=dialect, skipinitialspace=True)
        if reader.fieldnames and (
            (len(reader.fieldnames) <= 1 and "," in sample)
            or any("," in header for header in reader.fieldnames)
        ):
            # Some blackbox CSV exports confuse Sniffer and collapse the full
            # header row or split on the wrong delimiter; force a comma parser.
            f.seek(0)
            reader = csv.DictReader(f, delimiter=",", skipinitialspace=True)
        if not reader.fieldnames:
            raise RuntimeError("CSV header row is missing.")
        headers = [header.strip() for header in reader.fieldnames]
        columns: dict[str, list[float]] = {h: [] for h in headers}
        for row in reader:
            for h in headers:
                raw = (row.get(h) or "").strip()
                if raw == "":
                    columns[h].append(0.0)
                    continue
                try:
                    columns[h].append(float(raw))
                except ValueError:
                    columns[h].append(0.0)

    time_key = _find_time_key(columns)
    if time_key is None:
        raise RuntimeError("CSV does not contain a recognizable time column.")
    times = _normalize_time_seconds(columns[time_key])
    if len(times) < 2:
        raise RuntimeError("Time column does not contain enough points.")
    return times, columns


def _find_time_key(columns: dict[str, list[float]]) -> str | None:
    for key in columns:
        name = key.strip().lower()
        compact = "".join(ch for ch in name if ch.isalnum())
        if compact in ("time", "timeus", "looptime", "loopiteration"):
            return key
    return None


def _normalize_time_seconds(values: list[float]) -> list[float]:
    if not values:
        return []
    minimum = min(values)
    normalized = [v - minimum for v in values]
    span = max(normalized) if normalized else 0.0
    if span > 1_000_000.0:
        return [v / 1_000_000.0 for v in normalized]
    if span > 10_000.0:
        return [v / 1000.0 for v in normalized]
    return normalized


def _choose_column(columns: dict[str, list[float]], names: list[str]) -> list[float] | None:
    lowered = {k.lower(): k for k in columns}
    for name in names:
        actual = lowered.get(name.lower())
        if actual is not None:
            return columns[actual]
    return None


def _sanitize_filename(name: str) -> str:
    invalid = '<>:"/\\|?*'
    out = "".join("_" if ch in invalid else ch for ch in name.strip())
    return out or "blackbox.log"


def _next_available_filename(folder: Path, preferred_name: str) -> Path:
    stem = Path(preferred_name).stem
    suffix = Path(preferred_name).suffix
    for idx in range(1, 10000):
        candidate = folder / f"{stem}_{idx:03d}{suffix}"
        if not candidate.exists():
            return candidate
    fd, tmp = tempfile.mkstemp(prefix=f"{stem}_", suffix=suffix, dir=folder)
    Path(tmp).unlink(missing_ok=True)
    return Path(tmp)


def _crc32_file(path: Path) -> int:
    crc = 0
    with path.open("rb") as f:
        while True:
            chunk = f.read(1024 * 1024)
            if not chunk:
                break
            crc = zlib.crc32(chunk, crc)
    return crc & 0xFFFFFFFF


def _mean(values: list[float]) -> float:
    return sum(values) / max(1, len(values))


def _median(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0

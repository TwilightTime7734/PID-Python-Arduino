"""Background worker task functions used by the Tkinter runtime.

These functions are intentionally kept free of direct Tk widget access so they
can run safely through SerialWorker and report back through callbacks.
"""

from __future__ import annotations

from pathlib import Path
import time

from serialUSB.inav_serial_service import InavSerialService, send_cli_msc_command

from ..attitude_service import AttitudeSample
from ..auto_tune_report import generate_auto_tune_report
from ..blackbox_import import (
    BlackboxImportResult,
    analyze_blackbox_log,
    analyze_pulled_blackbox_logs,
    import_blackbox_logs_from_msc,
)
from ..constants import REG_QUANT
from ..serial_protocol import (
    cancel_active_pulse_on_serial,
    read_pulse_status_on_serial,
    read_regs,
    start_fixed_pulse_on_serial,
)
from ..step_response_report import generate_step_response_report as generate_step_response_report_impl
from ..worker import SerialWorker
from ..workflows.fly_log_pid_isolation_workflow import (
    FlyLogPidIsolationSnapshot,
    prepare_fly_log_pid_isolation as prepare_fly_log_pid_isolation_impl,
    restore_fly_log_pid_isolation as restore_fly_log_pid_isolation_impl,
)


def pulse_channel_force(
    worker_self: SerialWorker,
    channel_index: int,
    force_us: int,
):
    if worker_self.ser is None:
        raise RuntimeError("Serial not open")
    _, max_count = read_regs(worker_self.ser, REG_QUANT, 2)
    start_fixed_pulse_on_serial(worker_self.ser, max_count, channel_index, force_us)
    return read_pulse_status_on_serial(worker_self.ser, max_count)


def cancel_active_pulse(worker_self: SerialWorker):
    if worker_self.ser is None:
        raise RuntimeError("Serial not open")
    _, max_count = read_regs(worker_self.ser, REG_QUANT, 2)
    cancel_active_pulse_on_serial(worker_self.ser, max_count)
    return read_pulse_status_on_serial(worker_self.ser, max_count)


def _as_signed_i16(value: int) -> int:
    if value > 0x7FFF:
        return value - 0x10000
    return value


def read_movement_attitude(worker_self: SerialWorker):
    """Read attitude-board movement registers from the Modbus firmware."""
    if worker_self.ser is None:
        raise RuntimeError("Serial not open")
    regs = read_regs(worker_self.ser, 29, 6)
    if len(regs) < 6:
        return None

    movement_status = int(regs[0])
    if movement_status != 2:
        return None

    movement_seq = int(regs[1]) & 0xFFFF
    roll_deg = float(_as_signed_i16(int(regs[4])))
    pitch_deg = float(_as_signed_i16(int(regs[5])))
    movement_millis = (int(regs[2]) & 0xFFFF) | ((int(regs[3]) & 0xFFFF) << 16)
    return AttitudeSample(
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
        yaw_deg=0.0,
        movement_millis=movement_millis,
        movement_seq=movement_seq,
    )


def read_fc_pid_ff(_worker_self: SerialWorker, fc_service: InavSerialService):
    return fc_service.read_roll_pitch_pid_ff(timeout_seconds=1.2)


def prepare_fly_log_pid_isolation(_worker_self: SerialWorker, fc_service: InavSerialService, test_axis: str):
    return prepare_fly_log_pid_isolation_impl(fc_service, test_axis)


def restore_fly_log_pid_isolation(
    _worker_self: SerialWorker,
    fc_service: InavSerialService,
    snapshot: FlyLogPidIsolationSnapshot,
):
    return restore_fly_log_pid_isolation_impl(fc_service, snapshot)


def enter_msc_and_import_blackbox_logs(
    _worker_self: SerialWorker,
    fc_port_name: str,
    fc_baud_rate: int,
    blackbox_import_dir: Path,
    mount_timeout_s: float,
    mount_poll_s: float,
) -> BlackboxImportResult:
    msc_warnings: list[str] = []
    try:
        send_cli_msc_command(fc_port_name, fc_baud_rate)
    except Exception as exc:
        msc_warnings.append(f"Could not send CLI 'msc' on {fc_port_name}: {exc}")

    deadline = time.monotonic() + mount_timeout_s
    result: BlackboxImportResult | None = None
    while True:
        result = import_blackbox_logs_from_msc(blackbox_import_dir)
        if result.scanned_roots:
            break
        if time.monotonic() >= deadline:
            break
        time.sleep(mount_poll_s)

    if result is None:
        result = import_blackbox_logs_from_msc(blackbox_import_dir)
    if not msc_warnings:
        return result

    merged_warnings: list[str] = []
    for warning in [*msc_warnings, *result.warnings]:
        if warning and warning not in merged_warnings:
            merged_warnings.append(warning)
    return BlackboxImportResult(
        scanned_roots=result.scanned_roots,
        imported_files=result.imported_files,
        skipped_count=result.skipped_count,
        warnings=tuple(merged_warnings),
        analysis_summary=result.analysis_summary,
        analysis_source=result.analysis_source,
        pid_report=result.pid_report,
    )


def analyze_blackbox_logs(_worker_self: SerialWorker, blackbox_import_dir: Path):
    return analyze_pulled_blackbox_logs(blackbox_import_dir)


def analyze_specific_blackbox_log(
    _worker_self: SerialWorker,
    log_path: str,
    blackbox_import_dir: Path,
):
    return analyze_blackbox_log(log_path, decode_destination_dir=blackbox_import_dir)


def is_auxiliary_blackbox_csv(path: Path) -> bool:
    lower = path.name.lower()
    return lower.endswith(".gps.csv") or lower.endswith(".event.csv") or lower.endswith(".events.csv")


def resolve_chart_source_path(
    analysis_result: BlackboxImportResult,
    preferred_log_path: str,
    blackbox_import_dir: Path,
) -> str | None:
    candidates: list[Path] = []
    if preferred_log_path:
        candidates.append(Path(preferred_log_path))
    if analysis_result.analysis_source:
        candidates.append(Path(analysis_result.analysis_source))

    # Prefer any explicit, existing non-aux CSV path first.
    for candidate in candidates:
        if candidate.exists() and candidate.is_file() and candidate.suffix.lower() == ".csv":
            if not is_auxiliary_blackbox_csv(candidate):
                return str(candidate)

    search_dirs: list[Path] = []
    for candidate in candidates:
        parent = candidate.parent
        if parent.exists() and parent not in search_dirs:
            search_dirs.append(parent)
    if blackbox_import_dir.exists() and blackbox_import_dir not in search_dirs:
        search_dirs.append(blackbox_import_dir)

    stems: list[str] = []
    for candidate in candidates:
        stem = candidate.stem.strip()
        if stem and stem not in stems:
            stems.append(stem)

    for search_dir in search_dirs:
        csv_candidates: list[Path] = []
        for stem in stems:
            csv_candidates.extend(p for p in search_dir.glob(f"{stem}*.csv") if p.is_file())
        csv_candidates.extend(p for p in search_dir.glob("*.csv") if p.is_file())
        preferred_csvs = [p for p in csv_candidates if not is_auxiliary_blackbox_csv(p)]
        if preferred_csvs:
            return str(max(preferred_csvs, key=lambda p: p.stat().st_mtime))

    if analysis_result.analysis_source:
        return analysis_result.analysis_source
    return preferred_log_path or None


def generate_auto_report(
    _worker_self: SerialWorker,
    analysis_result: BlackboxImportResult,
    session_payload: dict[str, object],
    preferred_log_path: str,
    blackbox_import_dir: Path,
):
    source_path = resolve_chart_source_path(analysis_result, preferred_log_path, blackbox_import_dir)
    return generate_auto_tune_report(
        blackbox_import_dir,
        analysis_result,
        session_payload,
        source_path,
    )


def generate_step_response_report(
    _worker_self: SerialWorker,
    log_paths: list[str],
    blackbox_import_dir: Path,
):
    return generate_step_response_report_impl(log_paths, blackbox_import_dir)


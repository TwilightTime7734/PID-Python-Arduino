"""Snapshot and restore INAV motor mixer rows for Fly/Log tests."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import re
from pathlib import Path
from typing import Any


_FIELD_NAMES = ("throttle", "roll", "pitch", "yaw")
_ZEROABLE_FIELDS = ("roll", "pitch", "yaw")
_FIELD_PATTERN = re.compile(r"(throttle|roll|pitch|yaw)$")
_INDEX_PATTERN = re.compile(r"\d+")


@dataclass(frozen=True)
class _MixerCell:
    setting_name: str
    raw_value: bytes


@dataclass(frozen=True)
class _MixerRow:
    row_index: int
    cells: dict[str, _MixerCell]


class FlyLogMixerWorkflow:
    """Reads, snapshots, zeroes, and restores INAV motor mixer rows."""

    def __init__(self, app: Any, *, snapshot_dir: Path | None = None) -> None:
        self.app = app
        self.snapshot_dir = (snapshot_dir or (Path(__file__).resolve().parents[2] / "mixer_snapshots")).resolve()
        self.snapshot_path: Path | None = None
        self._rows: list[_MixerRow] = []
        self._axis: str = ""
        self._active = False

    def is_active(self) -> bool:
        return self._active

    def _trace(self, message: str) -> None:
        self.app.status.set(f"[Fly/Log mixer] {message}")

    @staticmethod
    def _normalize_name(name: str) -> str:
        return re.sub(r"[^a-z0-9]+", "_", name.strip().lower()).strip("_")

    def _candidate_names(self, row_index: int, field: str) -> list[str]:
        row_tokens = [row_index, row_index + 1]
        candidates = []
        prefixes = (
            "motor_mixer",
            "motor_mixers",
            "mixer_motor",
            "mixer_motors",
            "mixer_profile_motor_mixer",
            "mixer_profile_motor_mixers",
            "mixer_profiles_0_motor_mixer",
            "mixer_profiles_0_motor_mixers",
        )
        for row in row_tokens:
            for prefix in prefixes:
                candidates.append(f"{prefix}_{row}_{field}")
        return candidates

    def _read_cell(self, setting_name: str) -> bytes:
        return self.app.fc_service.get_setting_bytes(setting_name, timeout_seconds=1.0)

    def _discover_rows_by_candidates(self) -> list[_MixerRow]:
        rows: dict[int, dict[str, _MixerCell]] = {}
        for row_index in range(0, 4):
            for field in _FIELD_NAMES:
                for candidate in self._candidate_names(row_index, field):
                    try:
                        raw = self._read_cell(candidate)
                    except Exception:
                        continue
                    rows.setdefault(row_index, {})[field] = _MixerCell(candidate, raw)
                    break
        complete = [row for row in rows.items() if all(field in row[1] for field in _FIELD_NAMES)]
        if len(complete) >= 4:
            return [_MixerRow(index, cells) for index, cells in sorted(complete)[:4]]
        return []

    def _discover_rows_by_scan(self) -> list[_MixerRow]:
        rows: dict[int, dict[str, _MixerCell]] = {}
        try:
            names = self.app.fc_service.list_setting_names(timeout_seconds=0.25, max_index=4096)
        except Exception:
            return []

        for raw_name in names:
            normalized = self._normalize_name(raw_name)
            if "motor" not in normalized and "mix" not in normalized:
                continue
            if not any(field in normalized for field in _FIELD_NAMES):
                continue
            field_match = _FIELD_PATTERN.search(normalized)
            if field_match is None:
                continue
            field = field_match.group(1)
            numbers = [int(value) for value in _INDEX_PATTERN.findall(normalized)]
            if not numbers:
                continue
            row_index = numbers[-1]
            try:
                raw = self._read_cell(raw_name)
            except Exception:
                continue
            rows.setdefault(row_index, {})[field] = _MixerCell(raw_name, raw)

        complete = [row for row in rows.items() if all(field in row[1] for field in _FIELD_NAMES)]
        if not complete:
            return []
        return [_MixerRow(index, cells) for index, cells in sorted(complete)[:4]]

    def _discover_rows(self) -> list[_MixerRow]:
        rows = self._discover_rows_by_candidates()
        if rows:
            return rows
        rows = self._discover_rows_by_scan()
        if rows:
            return rows
        raise RuntimeError(
            "Unable to locate INAV motor mixer rows. "
            "The FC may not expose them through MSP settings on this firmware build."
        )

    def _snapshot_payload(self, rows: list[_MixerRow], axis: str) -> dict[str, object]:
        return {
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "axis": axis,
            "rows": [
                {
                    "row_index": row.row_index,
                    "cells": {
                        field: {
                            "setting_name": cell.setting_name,
                            "raw_hex": cell.raw_value.hex(),
                        }
                        for field, cell in row.cells.items()
                    },
                }
                for row in rows
            ],
        }

    def prepare_for_test(self, axis: str) -> Path:
        axis_name = str(axis).strip().lower()
        if axis_name not in {"roll", "pitch"}:
            raise RuntimeError(f"Unsupported Fly/Log axis '{axis}'.")
        if self._active:
            raise RuntimeError("A Fly/Log mixer snapshot is already active.")

        rows = self._discover_rows()
        self.snapshot_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        snapshot_path = self.snapshot_dir / f"fly_log_mixer_snapshot_{timestamp}_{axis_name}.json"
        payload = self._snapshot_payload(rows, axis_name)
        snapshot_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")

        zero_fields = [field for field in _ZEROABLE_FIELDS if field != axis_name]
        for row in rows:
            for field in zero_fields:
                cell = row.cells[field]
                self.app.fc_service.set_setting_bytes(cell.setting_name, b"\x00" * len(cell.raw_value), timeout_seconds=1.0)
        self.app.fc_service.save_settings(timeout_seconds=1.5)

        for row in rows:
            for field in zero_fields:
                cell = row.cells[field]
                confirmed = self.app.fc_service.get_setting_bytes(cell.setting_name, timeout_seconds=1.0)
                if confirmed != b"\x00" * len(cell.raw_value):
                    raise RuntimeError(
                        f"INAV did not confirm zeroed mixer field '{cell.setting_name}' after save."
                    )

        self.snapshot_path = snapshot_path
        self._rows = rows
        self._axis = axis_name
        self._active = True
        self._trace(
            f"Snapshot saved to {snapshot_path.name}; zeroed {', '.join(zero_fields)} for the Fly/Log test."
        )
        return snapshot_path

    def restore_after_test(self) -> Path | None:
        if not self._active:
            return self.snapshot_path
        if not self._rows:
            raise RuntimeError("No mixer snapshot is available to restore.")

        for row in self._rows:
            for field in _FIELD_NAMES:
                cell = row.cells[field]
                self.app.fc_service.set_setting_bytes(cell.setting_name, cell.raw_value, timeout_seconds=1.0)
        self.app.fc_service.save_settings(timeout_seconds=1.5)

        for row in self._rows:
            for field in _FIELD_NAMES:
                cell = row.cells[field]
                confirmed = self.app.fc_service.get_setting_bytes(cell.setting_name, timeout_seconds=1.0)
                if confirmed != cell.raw_value:
                    raise RuntimeError(
                        f"INAV did not confirm restored mixer field '{cell.setting_name}' after save."
                    )

        self._trace(
            f"Restored original mixer rows from {self.snapshot_path.name if self.snapshot_path else 'snapshot'}."
        )
        self._active = False
        return self.snapshot_path

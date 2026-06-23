"""Safe mixer debug capture for INAV FC connectivity checks."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any


MIXER_MSP_READ = 0x2010

EXPECTED_MOTOR_MIXER_ROWS = (
    {
        "motor": 1,
        "throttle": 1,
        "roll": -1,
        "pitch": -1,
        "yaw": 1,
    },
    {
        "motor": 2,
        "throttle": 1,
        "roll": 1,
        "pitch": 1,
        "yaw": 1,
    },
    {
        "motor": 3,
        "throttle": 1,
        "roll": 1,
        "pitch": -1,
        "yaw": -1,
    },
    {
        "motor": 4,
        "throttle": 1,
        "roll": -1,
        "pitch": 1,
        "yaw": -1,
    },
)


@dataclass(frozen=True)
class MixerDebugCapture:
    created_at_utc: str
    mixer_payload_hex: str
    mixer_payload_bytes: list[int]
    decoded_mixer_payload: dict[str, object]
    fc_mixer_settings: dict[str, object]
    expected_motor_mixer_rows: tuple[dict[str, int], ...]


class MixerDebugWorkflow:
    """Captures the current FC mixer packet and a known-good reference matrix."""

    def __init__(self, app: Any, *, output_dir: Path | None = None) -> None:
        self.app = app
        self.output_dir = (output_dir or (Path(__file__).resolve().parents[2] / "mixer_snapshots")).resolve()

    def _trace(self, message: str) -> None:
        self.app.status.set(f"[Mixer debug] {message}")

    def _read_mixer_settings(self) -> dict[str, object]:
        settings: dict[str, object] = {}
        service = self.app.fc_service
        for name in (
            "mixer_control_profile_linking",
            "mixer_automated_switch",
            "mixer_switch_trans_timer",
            "transition_pid_mmix_multiplier_roll",
            "transition_pid_mmix_multiplier_pitch",
            "transition_pid_mmix_multiplier_yaw",
            "fpv_mix_degrees",
        ):
            try:
                settings[name] = service.get_setting_int(name, timeout_seconds=1.0)
            except Exception as exc:
                settings[name] = f"ERR: {exc}"
        return settings

    @staticmethod
    def _decode_mixer_payload(payload: bytes) -> dict[str, object]:
        u8_values = list(payload)
        u16_le_values = [
            int.from_bytes(payload[index : index + 2], byteorder="little", signed=False)
            for index in range(0, len(payload) - (len(payload) % 2), 2)
        ]
        u32_le_values = [
            int.from_bytes(payload[index : index + 4], byteorder="little", signed=False)
            for index in range(0, len(payload) - (len(payload) % 4), 4)
        ]
        signed_i8_values = [int.from_bytes(bytes([value]), byteorder="little", signed=True) for value in payload]
        nonzero_indexes = [index for index, value in enumerate(payload) if value != 0]
        return {
            "payload_length": len(payload),
            "u8_values": u8_values,
            "u16_le_values": u16_le_values,
            "u32_le_values": u32_le_values,
            "signed_i8_values": signed_i8_values,
            "nonzero_indexes": nonzero_indexes,
        }

    def capture_reference(self) -> Path:
        if not self.app.fc_service.is_connected:
            raise RuntimeError("Connect the FC before capturing mixer debug data.")

        payload = self.app.fc_service.request_msp_v2(MIXER_MSP_READ, b"", timeout_seconds=1.0)
        capture = MixerDebugCapture(
            created_at_utc=datetime.now(timezone.utc).isoformat(),
            mixer_payload_hex=payload.hex(),
            mixer_payload_bytes=list(payload),
            decoded_mixer_payload=self._decode_mixer_payload(payload),
            fc_mixer_settings=self._read_mixer_settings(),
            expected_motor_mixer_rows=EXPECTED_MOTOR_MIXER_ROWS,
        )

        self.output_dir.mkdir(parents=True, exist_ok=True)
        out_path = self.output_dir / f"mixer_debug_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}.json"
        out_path.write_text(json.dumps(capture.__dict__, indent=2, sort_keys=True), encoding="utf-8")
        self._trace(f"Recorded mixer debug capture to {out_path.name}.")
        return out_path

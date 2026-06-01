"""INAV MSP serial transport with background attitude polling."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from datetime import datetime

import serial

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_ATTITUDE = 108
MSP2_COMMON_SETTING = 0x1003
MSP2_COMMON_SET_SETTING = 0x1004
MSP2_COMMON_SETTING_INFO = 0x1007

PID_SETTING_NAME = {
    ("roll", "p"): "mc_p_roll",
    ("roll", "i"): "mc_i_roll",
    ("roll", "d"): "mc_d_roll",
    ("pitch", "p"): "mc_p_pitch",
    ("pitch", "i"): "mc_i_pitch",
    ("pitch", "d"): "mc_d_pitch",
}
FF_SETTING_NAME = {
    "roll": "mc_cd_roll",
    "pitch": "mc_cd_pitch",
}


@dataclass(frozen=True)
class AttitudeSample:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    timestamp_local: datetime


@dataclass(frozen=True)
class AxisPidFf:
    p: float
    i: float
    d: float
    ff: float


@dataclass
class _MspFrame:
    command_id: int
    payload: bytes
    is_valid: bool


@dataclass
class _PendingRequest:
    command_id: int
    done: threading.Event
    response: bytes | None = None
    error: Exception | None = None


class _MspStreamParser:
    def __init__(self) -> None:
        self._state = "idle"
        self._is_v2 = False
        self._v2_flag = 0
        self._cmd = 0
        self._size = 0
        self._offset = 0
        self._payload = bytearray()

    def push(self, value: int) -> _MspFrame | None:
        b = value & 0xFF
        if self._state == "idle":
            if b == ord("$"):
                self._state = "proto"
            return None

        if self._state == "proto":
            if b == ord("M"):
                self._is_v2 = False
                self._reset_packet()
                self._state = "dir"
            elif b == ord("X"):
                self._is_v2 = True
                self._reset_packet()
                self._state = "dir"
            else:
                self._state = "idle"
            return None

        if self._state == "dir":
            if b not in (ord(">"), ord("!"), ord("<")):
                self._state = "idle"
                return None
            self._state = "v2_flag" if self._is_v2 else "v1_size"
            return None

        return self._advance(b)

    def _reset_packet(self) -> None:
        self._cmd = 0
        self._size = 0
        self._offset = 0
        self._payload = bytearray()
        self._v2_flag = 0

    def _advance(self, b: int) -> _MspFrame | None:
        if self._state == "v1_size":
            self._size = b
            self._payload = bytearray(self._size)
            self._offset = 0
            self._state = "v1_cmd"
            return None

        if self._state == "v1_cmd":
            self._cmd = b
            self._state = "v1_payload" if self._size > 0 else "v1_crc"
            return None

        if self._state == "v1_payload":
            self._payload[self._offset] = b
            self._offset += 1
            if self._offset >= self._size:
                self._state = "v1_crc"
            return None

        if self._state == "v1_crc":
            crc = self._size ^ self._cmd
            for value in self._payload:
                crc ^= value
            frame = _MspFrame(self._cmd, bytes(self._payload), (crc & 0xFF) == b)
            self._state = "idle"
            return frame

        if self._state == "v2_flag":
            self._v2_flag = b
            self._state = "v2_cmd_lo"
            return None

        if self._state == "v2_cmd_lo":
            self._cmd = b
            self._state = "v2_cmd_hi"
            return None

        if self._state == "v2_cmd_hi":
            self._cmd |= b << 8
            self._state = "v2_size_lo"
            return None

        if self._state == "v2_size_lo":
            self._size = b
            self._state = "v2_size_hi"
            return None

        if self._state == "v2_size_hi":
            self._size |= b << 8
            self._payload = bytearray(self._size)
            self._offset = 0
            self._state = "v2_payload" if self._size > 0 else "v2_crc"
            return None

        if self._state == "v2_payload":
            self._payload[self._offset] = b
            self._offset += 1
            if self._offset >= self._size:
                self._state = "v2_crc"
            return None

        if self._state == "v2_crc":
            crc = _crc8_dvb_s2(bytes([self._v2_flag]))
            crc = _crc8_dvb_s2(bytes([self._cmd & 0xFF]), crc)
            crc = _crc8_dvb_s2(bytes([(self._cmd >> 8) & 0xFF]), crc)
            crc = _crc8_dvb_s2(bytes([self._size & 0xFF]), crc)
            crc = _crc8_dvb_s2(bytes([(self._size >> 8) & 0xFF]), crc)
            crc = _crc8_dvb_s2(bytes(self._payload), crc)
            frame = _MspFrame(self._cmd, bytes(self._payload), (crc & 0xFF) == b)
            self._state = "idle"
            return frame

        self._state = "idle"
        return None


def _crc8_dvb_s2(data: bytes, seed: int = 0) -> int:
    crc = seed & 0xFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if (crc & 0x80) != 0:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _build_msp_v2_request(command_id: int, payload: bytes) -> bytes:
    cmd = command_id & 0xFFFF
    size = len(payload) & 0xFFFF
    header = bytes(
        [
            0,
            cmd & 0xFF,
            (cmd >> 8) & 0xFF,
            size & 0xFF,
            (size >> 8) & 0xFF,
        ]
    )
    checksum = _crc8_dvb_s2(header + payload)
    return b"$X<" + header + payload + bytes([checksum])


def _parse_attitude_payload(payload: bytes) -> AttitudeSample:
    if len(payload) < 6:
        raise RuntimeError(f"Invalid MSP_ATTITUDE payload length: {len(payload)}")
    roll_raw = int.from_bytes(payload[0:2], byteorder="little", signed=True)
    pitch_raw = int.from_bytes(payload[2:4], byteorder="little", signed=True)
    yaw_raw = int.from_bytes(payload[4:6], byteorder="little", signed=True)
    return AttitudeSample(
        roll_deg=roll_raw / 10.0,
        pitch_deg=pitch_raw / 10.0,
        yaw_deg=float(yaw_raw),
        timestamp_local=datetime.now(),
    )


def send_cli_msc_command(
    port_name: str,
    baud_rate: int = 115200,
    cli_enter_delay_s: float = 0.25,
    post_command_delay_s: float = 0.2,
) -> None:
    """Enter INAV CLI and issue `msc` so the FC re-enumerates as mass storage."""
    port = str(port_name).strip()
    if not port:
        raise RuntimeError("FC port is empty.")
    baud = int(baud_rate)
    if baud <= 0:
        raise RuntimeError("FC baud must be > 0.")

    try:
        with serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            write_timeout=0.2,
        ) as ser:
            time.sleep(0.15)
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass

            # Match INAV Configurator CLI entry: send '#', then send 'msc\n'.
            ser.write(b"#")
            ser.flush()
            time.sleep(max(0.0, float(cli_enter_delay_s)))
            ser.write(b"msc\n")
            ser.flush()
            time.sleep(max(0.0, float(post_command_delay_s)))
    except serial.SerialException as exc:
        raise RuntimeError(f"Unable to open FC serial port '{port}' @ {baud} baud: {exc}") from exc


class InavSerialService:
    def __init__(self) -> None:
        self._sync = threading.Lock()
        self._write_lock = threading.Lock()
        self._pending_sync = threading.Condition()
        self._pending: _PendingRequest | None = None
        self._attitude_sync = threading.Lock()
        self._latest_attitude: AttitudeSample | None = None
        self._serial: serial.Serial | None = None
        self._stop_event = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._poll_thread: threading.Thread | None = None
        self._setting_index_cache: dict[str, int] = {}
        self._port_name = ""
        self._baud_rate = 115200

    @property
    def is_connected(self) -> bool:
        with self._sync:
            return self._serial is not None and self._serial.is_open

    def connect(self, port_name: str, baud_rate: int = 115200) -> None:
        self.disconnect()
        self._port_name = str(port_name).strip()
        self._baud_rate = int(baud_rate)
        ser = self._open_transport(self._port_name, self._baud_rate)

        self._stop_event.clear()
        with self._sync:
            self._serial = ser
            self._setting_index_cache.clear()
            self._reader_thread = threading.Thread(target=self._read_loop, name="INAV-MSP-Reader", daemon=True)
            self._reader_thread.start()

        try:
            self._probe_inav()
        except Exception:
            # Match Usb2Arduino behavior: recover once from transient USB/FC reset races.
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass
            time.sleep(0.12)
            try:
                self._probe_inav()
            except Exception:
                self.disconnect()
                raise

        with self._sync:
            if self._poll_thread is None:
                self._poll_thread = threading.Thread(
                    target=self._attitude_poll_loop,
                    name="INAV-MSP-AttitudePoll",
                    daemon=True,
                )
                self._poll_thread.start()

    def disconnect(self) -> None:
        with self._sync:
            ser = self._serial
            reader = self._reader_thread
            poller = self._poll_thread
            self._serial = None
            self._reader_thread = None
            self._poll_thread = None

        self._stop_event.set()
        if ser is not None:
            try:
                if ser.is_open:
                    ser.close()
            except Exception:
                pass

        if poller is not None:
            poller.join(timeout=0.5)
        if reader is not None:
            reader.join(timeout=0.5)

        with self._pending_sync:
            pending = self._pending
            self._pending = None
            if pending is not None and not pending.done.is_set():
                pending.error = TimeoutError("MSP transport disconnected.")
                pending.done.set()
            self._pending_sync.notify_all()

        with self._attitude_sync:
            self._latest_attitude = None
        with self._sync:
            self._setting_index_cache.clear()

    def read_attitude(self, timeout_seconds: float = 1.0) -> AttitudeSample:
        with self._attitude_sync:
            cached = self._latest_attitude
            if cached is not None and (datetime.now() - cached.timestamp_local).total_seconds() <= 0.8:
                return cached
        payload = self._request(MSP_ATTITUDE, b"", timeout_seconds)
        sample = _parse_attitude_payload(payload)
        with self._attitude_sync:
            self._latest_attitude = sample
        return sample

    def latest_attitude(self) -> AttitudeSample | None:
        with self._attitude_sync:
            return self._latest_attitude

    def read_roll_pitch_pid_ff(self, timeout_seconds: float = 1.0) -> tuple[AxisPidFf, AxisPidFf]:
        roll = AxisPidFf(
            p=float(self.get_setting_int(PID_SETTING_NAME[("roll", "p")], timeout_seconds)),
            i=float(self.get_setting_int(PID_SETTING_NAME[("roll", "i")], timeout_seconds)),
            d=float(self.get_setting_int(PID_SETTING_NAME[("roll", "d")], timeout_seconds)),
            ff=float(self.get_setting_int(FF_SETTING_NAME["roll"], timeout_seconds)),
        )
        pitch = AxisPidFf(
            p=float(self.get_setting_int(PID_SETTING_NAME[("pitch", "p")], timeout_seconds)),
            i=float(self.get_setting_int(PID_SETTING_NAME[("pitch", "i")], timeout_seconds)),
            d=float(self.get_setting_int(PID_SETTING_NAME[("pitch", "d")], timeout_seconds)),
            ff=float(self.get_setting_int(FF_SETTING_NAME["pitch"], timeout_seconds)),
        )
        return roll, pitch

    def get_setting_int(self, name: str, timeout_seconds: float = 0.8) -> int:
        index = self._setting_index(name, timeout_seconds=max(0.1, timeout_seconds))
        payload = self._request(MSP2_COMMON_SETTING, self._setting_key_payload(index), timeout_seconds=max(0.1, timeout_seconds))
        if not payload:
            raise RuntimeError(f"INAV setting '{name}' returned empty payload.")
        return int.from_bytes(payload, byteorder="little", signed=False)

    def set_setting_int(self, name: str, value: int, timeout_seconds: float = 0.8) -> int:
        setting = name.strip().lower()
        target = int(value)
        timeout_s = max(0.1, timeout_seconds)
        index = self._setting_index(setting, timeout_seconds=timeout_s)
        info = bytearray(self._request(MSP2_COMMON_SETTING_INFO, self._setting_key_payload(index), timeout_s))
        try:
            name_end = info.index(0)
        except ValueError as exc:
            raise RuntimeError(f"Invalid setting info payload for '{setting}'.") from exc
        current_payload = self._request(MSP2_COMMON_SETTING, self._setting_key_payload(index), timeout_s)
        value_size = len(current_payload)
        if value_size not in (1, 2, 4):
            raise RuntimeError(f"Unsupported setting byte width for '{setting}': {value_size}")
        encoded = target.to_bytes(value_size, byteorder="little", signed=False)
        info[name_end + 1 : name_end + 1 + value_size] = encoded
        self._request(MSP2_COMMON_SET_SETTING, bytes(info), timeout_s)
        return self.get_setting_int(setting, timeout_seconds=timeout_s)

    def _setting_key_payload(self, index: int) -> bytes:
        # INAV settings API expects setting index in upper 24 bits.
        return (int(index) << 8).to_bytes(4, byteorder="little", signed=False)

    def _setting_index(self, name: str, timeout_seconds: float) -> int:
        key = name.strip().lower()
        with self._sync:
            cached = self._setting_index_cache.get(key)
        if cached is not None:
            return cached

        setting_info_timeout = min(0.5, max(0.2, timeout_seconds))
        for index in range(0, 4096):
            payload = self._request(
                MSP2_COMMON_SETTING_INFO,
                self._setting_key_payload(index),
                timeout_seconds=setting_info_timeout,
            )
            nul = payload.find(b"\x00")
            if nul <= 0:
                continue
            found = payload[:nul].decode("ascii", errors="ignore").strip().lower()
            if not found:
                continue
            with self._sync:
                if found not in self._setting_index_cache:
                    self._setting_index_cache[found] = index
                if found == key:
                    return index
        raise KeyError(f"INAV setting '{name}' was not found via MSP2_COMMON_SETTING_INFO.")

    def _request(self, command_id: int, payload: bytes, timeout_seconds: float) -> bytes:
        timeout_s = max(0.1, timeout_seconds)
        done = threading.Event()
        request = _PendingRequest(command_id=command_id, done=done)
        deadline = time.monotonic() + timeout_s

        with self._pending_sync:
            while self._pending is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Timed out queueing MSP command {command_id}.")
                self._pending_sync.wait(remaining)
            self._pending = request

        packet = _build_msp_v2_request(command_id, payload)
        ser = self._require_serial()
        try:
            with self._write_lock:
                ser.write(packet)
                ser.flush()
        except Exception as exc:
            with self._pending_sync:
                if self._pending is request:
                    self._pending = None
                    self._pending_sync.notify_all()
            raise RuntimeError(f"Failed writing MSP command {command_id}: {exc}") from exc

        if not request.done.wait(timeout_s + 0.1):
            with self._pending_sync:
                if self._pending is request:
                    self._pending = None
                    self._pending_sync.notify_all()
            raise TimeoutError(f"Timed out waiting for MSP command {command_id}.")

        if request.error is not None:
            raise request.error
        if request.response is None:
            raise TimeoutError(f"MSP command {command_id} returned no payload.")
        return request.response

    def _probe_inav(self) -> None:
        variant_payload = self._request(MSP_FC_VARIANT, b"", 1.2)
        if len(variant_payload) < 4:
            raise RuntimeError("MSP did not return FC variant.")
        variant_text = variant_payload[:4].decode("ascii", errors="ignore").upper()
        if variant_text != "INAV":
            raise RuntimeError(f"MSP variant '{variant_text}' is not INAV.")
        self._request(MSP_API_VERSION, b"", 1.0)
        self._request(MSP_FC_VERSION, b"", 1.0)

    def _attitude_poll_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                payload = self._request(MSP_ATTITUDE, b"", 0.25)
                sample = _parse_attitude_payload(payload)
                with self._attitude_sync:
                    self._latest_attitude = sample
            except Exception:
                pass
            self._stop_event.wait(0.045)

    def _read_loop(self) -> None:
        parser = _MspStreamParser()
        while not self._stop_event.is_set():
            ser = self._require_serial(optional=True)
            if ser is None:
                return
            try:
                b = ser.read(1)
                if not b:
                    continue
                frame = parser.push(b[0])
                if frame is None or not frame.is_valid:
                    continue
                if frame.command_id == MSP_ATTITUDE and len(frame.payload) >= 6:
                    try:
                        sample = _parse_attitude_payload(frame.payload)
                        with self._attitude_sync:
                            self._latest_attitude = sample
                    except Exception:
                        pass
                with self._pending_sync:
                    pending = self._pending
                    if pending is not None and pending.command_id == frame.command_id and not pending.done.is_set():
                        pending.response = frame.payload
                        pending.done.set()
                        self._pending = None
                        self._pending_sync.notify_all()
            except serial.SerialException:
                return
            except OSError:
                return
            except Exception:
                continue

    def _require_serial(self, optional: bool = False) -> serial.Serial | None:
        with self._sync:
            ser = self._serial
        if ser is None or not ser.is_open:
            if optional:
                return None
            raise RuntimeError("FC USB serial transport is not connected.")
        return ser

    def _open_transport(self, port_name: str, baud_rate: int) -> serial.Serial:
        ser = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            write_timeout=0.2,
        )
        time.sleep(0.15)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        return ser

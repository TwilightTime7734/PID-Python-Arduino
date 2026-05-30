"""Low-level Modbus serial protocol helpers."""

from __future__ import annotations

import struct
import time

import serial

from .constants import (
    BAUDRATE,
    BOOT_WAIT,
    EXPECTED_FIRMWARE_VERSION,
    FRAME_US,
    PAUSE_US,
    READ,
    REG_CHANNEL0,
    REG_QUANT,
    REG_STATE,
    RUN_STATE,
    SLAVE_ID,
    TIMEOUT,
    WRITE,
)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def add_crc(payload: bytes) -> bytes:
    return payload + struct.pack("<H", crc16(payload))


def read_exact(ser: serial.Serial, size: int) -> bytes:
    data = ser.read(size)
    if len(data) != size:
        raise RuntimeError(f"Serial timeout ({len(data)}/{size} bytes)")
    return data


def check_frame(frame: bytes, fn: int) -> None:
    if len(frame) < 5:
        raise RuntimeError("Response too short")
    if struct.unpack("<H", frame[-2:])[0] != crc16(frame[:-2]):
        raise RuntimeError("CRC mismatch")
    if frame[0] != SLAVE_ID:
        raise RuntimeError(f"Unexpected slave id {frame[0]}")
    if frame[1] == (fn | 0x80):
        raise RuntimeError(f"Modbus exception 0x{frame[2]:02X}")
    if frame[1] != fn:
        raise RuntimeError(f"Unexpected function 0x{frame[1]:02X}")


def raise_modbus_exception_frame(frame: bytes, fn: int) -> None:
    if len(frame) != 5:
        raise RuntimeError("Exception response has invalid length")
    if struct.unpack("<H", frame[-2:])[0] != crc16(frame[:-2]):
        raise RuntimeError("CRC mismatch")
    if frame[0] != SLAVE_ID:
        raise RuntimeError(f"Unexpected slave id {frame[0]}")
    if frame[1] != (fn | 0x80):
        raise RuntimeError(f"Unexpected function 0x{frame[1]:02X}")
    raise RuntimeError(f"Modbus exception 0x{frame[2]:02X}")


def read_regs(ser: serial.Serial, start: int, count: int) -> list[int]:
    req = add_crc(struct.pack(">BBHH", SLAVE_ID, READ, start, count))
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    head2 = read_exact(ser, 2)
    if head2[1] == (READ | 0x80):
        raise_modbus_exception_frame(head2 + read_exact(ser, 3), READ)
    if head2[1] != READ:
        raise RuntimeError(f"Unexpected function 0x{head2[1]:02X}")

    byte_count = read_exact(ser, 1)[0]
    frame = head2 + bytes([byte_count]) + read_exact(ser, byte_count + 2)
    check_frame(frame, READ)
    if byte_count != count * 2:
        raise RuntimeError("Unexpected read length")
    return list(struct.unpack(f">{count}H", frame[3:-2]))


def write_regs(ser: serial.Serial, start: int, values: list[int]) -> None:
    if not values:
        raise RuntimeError("No values to write")
    payload = struct.pack(">BBHHB", SLAVE_ID, WRITE, start, len(values), len(values) * 2)
    req = add_crc(payload + struct.pack(f">{len(values)}H", *values))
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    head2 = read_exact(ser, 2)
    if head2[1] == (WRITE | 0x80):
        raise_modbus_exception_frame(head2 + read_exact(ser, 3), WRITE)
    if head2[1] != WRITE:
        raise RuntimeError(f"Unexpected function 0x{head2[1]:02X}")

    frame = head2 + read_exact(ser, 6)
    check_frame(frame, WRITE)
    resp_start, resp_count = struct.unpack(">HH", frame[2:6])
    if resp_start != start or resp_count != len(values):
        raise RuntimeError("Write echo mismatch")


def open_serial(port: str) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT,
        write_timeout=TIMEOUT,
    )
    time.sleep(BOOT_WAIT)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def us_to_ticks(us: int, quant: int) -> int:
    return max(1, round(us * quant))


def read_firmware_version_on_serial(ser: serial.Serial, max_count: int) -> str:
    version_reg = REG_CHANNEL0 + max_count + 6
    major, minor, patch = read_regs(ser, version_reg, 3)
    return f"{major}.{minor}.{patch}"


def firmware_version_warning_on_serial(ser: serial.Serial, max_count: int) -> str | None:
    try:
        device_version = read_firmware_version_on_serial(ser, max_count)
    except Exception as exc:
        return f"Firmware version check failed: expected {EXPECTED_FIRMWARE_VERSION}, reason: {exc}"
    if device_version != EXPECTED_FIRMWARE_VERSION:
        return f"Firmware version mismatch: expected {EXPECTED_FIRMWARE_VERSION}, device reports {device_version}."
    return None


def run_ppm_on_serial(ser: serial.Serial, channels: list[int], offsets: list[int]) -> tuple[int, int, str | None]:
    if len(channels) != len(offsets):
        raise RuntimeError("Channel and offset counts must match")
    adjusted = [c - o for c, o in zip(channels, offsets)]

    quant, max_count = read_regs(ser, REG_QUANT, 2)
    if quant <= 0:
        raise RuntimeError("Invalid quant from device")
    if len(channels) > max_count:
        raise RuntimeError(f"Firmware supports only {max_count} channels")
    if any(v <= PAUSE_US for v in adjusted):
        raise RuntimeError("Every adjusted channel must be > pause_us")

    sync_us = FRAME_US - sum(adjusted)
    if sync_us <= 0:
        raise RuntimeError("frame_us must be greater than channel total")

    pause_ticks = us_to_ticks(PAUSE_US, quant)
    sync_ticks = us_to_ticks(sync_us, quant)
    channel_ticks = [us_to_ticks(v, quant) for v in adjusted]
    if pause_ticks > 0xFFFF or any(v > 0xFFFF for v in channel_ticks):
        raise RuntimeError(f"pause/channels must be <= {0xFFFF / quant:.2f} us for quant={quant}")

    values = [RUN_STATE, len(channels), pause_ticks, sync_ticks & 0xFFFF, (sync_ticks >> 16) & 0xFFFF, *channel_ticks]
    write_regs(ser, REG_STATE, values)
    return quant, max_count, firmware_version_warning_on_serial(ser, max_count)


def run_ppm(port: str, channels: list[int], offsets: list[int]) -> None:
    with open_serial(port) as ser:
        run_ppm_on_serial(ser, channels, offsets)


def stop_ppm_on_serial(ser: serial.Serial) -> None:
    write_regs(ser, REG_STATE, [0])


def stop_ppm(port: str) -> None:
    with open_serial(port) as ser:
        stop_ppm_on_serial(ser)


def send_pulse_command_on_serial(ser: serial.Serial, max_count: int, chl: int, value_ticks: int, duration_us: int) -> None:
    pulse_base = REG_CHANNEL0 + max_count
    pulse_seq_reg = pulse_base + 4
    current_seq = read_regs(ser, pulse_seq_reg, 1)[0]
    next_seq = (current_seq + 1) & 0xFFFF
    write_regs(
        ser,
        pulse_base,
        [
            chl,
            value_ticks,
            duration_us & 0xFFFF,
            (duration_us >> 16) & 0xFFFF,
            next_seq,
        ],
    )


def read_pulse_status_on_serial(ser: serial.Serial, max_count: int) -> int:
    pulse_base = REG_CHANNEL0 + max_count
    pulse_status_reg = pulse_base + 5
    return read_regs(ser, pulse_status_reg, 1)[0]


def set_channel_until_stop_on_serial(
    ser: serial.Serial,
    quant: int,
    max_count: int,
    chl: int,
    val_us: int,
    offset_us: int,
    timeout_s: float,
) -> None:
    if chl < 0:
        raise RuntimeError("Channel index must be >= 0")
    if quant <= 0:
        raise RuntimeError("Invalid quant from device")
    if chl >= max_count:
        raise RuntimeError(f"Channel index {chl} out of range (max {max_count - 1})")

    adjusted = val_us - offset_us
    if adjusted <= PAUSE_US:
        raise RuntimeError("Target minus offset must be > pause_us")

    value_ticks = us_to_ticks(adjusted, quant)
    if value_ticks > 0xFFFF:
        raise RuntimeError(f"Target must be <= {0xFFFF / quant:.2f} us for quant={quant}")

    if timeout_s <= 0:
        raise RuntimeError("Duration must be > 0 for hold timeout")
    timeout_us = round(timeout_s * 1_000_000)
    if timeout_us <= 0 or timeout_us > 0xFFFFFFFF:
        raise RuntimeError("Duration is out of supported range")

    # Hold mode uses duration as a default timeout safety for automatic restore.
    send_pulse_command_on_serial(ser, max_count, chl, value_ticks, timeout_us)


def end_hold_on_serial(ser: serial.Serial, max_count: int, chl: int) -> None:
    if chl < 0:
        raise RuntimeError("Channel index must be >= 0")
    if chl >= max_count:
        raise RuntimeError(f"Channel index {chl} out of range (max {max_count - 1})")
    # Duration=0 is a firmware "end hold now" command.
    send_pulse_command_on_serial(ser, max_count, chl, 0, 0)

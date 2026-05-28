"""PPM Modbus desktop controller.
AETR
Roll, pitch, throttle, yaww

"""

from __future__ import annotations

import math
import struct
import time
import tkinter as tk
from dataclasses import dataclass
from collections.abc import Sequence
from tkinter import messagebox

import serial
import threading
import queue
from serial.tools import list_ports

from serialUSB.inav_serial_service import InavSerialService

READ = 0x03
WRITE = 0x10
REG_QUANT = 0
REG_MAX_COUNT = 1
REG_STATE = 2
REG_CHANNEL0 = 7

PORT_DEFAULT = "COM6"
FC_PORT_DEFAULT = "COM5"
FC_DEVICE_ID = "USB\\VID_0483&PID_5740"
FC_DEVICE_VID = 0x0483
FC_DEVICE_PID = 0x5740
FC_BAUD_DEFAULT = 115200
CHANNEL_DEFAULTS = [1500, 1500, 1100, 1500]
OFFSET_DEFAULTS = [-4, -2, -3, 6]
PULSE_TARGET_DEFAULTS = [1500, 1500, 1100, 1500]
PULSE_DURATION_DEFAULTS = [2, 2, 2, 2]

PAUSE_US = 300
FRAME_US = 22500
RUN_STATE = 1

BAUDRATE = 115200
SLAVE_ID = 1
TIMEOUT = 0.5
BOOT_WAIT = 1.5
APP_VERSION = "1.0.1"
EXPECTED_FIRMWARE_VERSION = APP_VERSION

PULSE_STATUS_IDLE = 0
PULSE_STATUS_ACTIVE = 1
PULSE_STATUS_REJECTED = 2
PULSE_STATUS_TIMEOUT_RESTORED = 3
PULSE_STATUS_HOLD_ENDED = 4


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


class SerialWorker:
    def __init__(self) -> None:
        self.tasks: "queue.Queue[tuple]" = queue.Queue()
        self.results: "queue.Queue[tuple]" = queue.Queue()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.ser: serial.Serial | None = None
        self._running = True
        self.thread.start()

    def _loop(self) -> None:
        while self._running:
            try:
                func, args, kwargs, cb = self.tasks.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                res = func(self, *args, **kwargs)
                self.results.put((cb, True, res))
            except Exception as e:
                self.results.put((cb, False, e))

    def submit(self, func, *args, callback=None, **kwargs) -> None:
        self.tasks.put((func, args, kwargs, callback))

    def stop(self) -> None:
        self._running = False
        self.thread.join()



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


def parse_entries(entries: list[tk.Entry], cast, label: str) -> list:
    try:
        return [cast(e.get().strip()) for e in entries]
    except ValueError:
        kind = "integers" if cast is int else "numbers"
        raise RuntimeError(f"{label} values must be {kind}")


def require_range(values: Sequence[int], label: str, min_value: int, max_value: int) -> None:
    for idx, value in enumerate(values, 1):
        if value < min_value or value > max_value:
            raise RuntimeError(f"{label} CH{idx} must be between {min_value} and {max_value}")


def require_duration_range(values: Sequence[float], min_s: float, max_s: float) -> None:
    for idx, value in enumerate(values, 1):
        if value < min_s or value > max_s:
            raise RuntimeError(f"Duration CH{idx} must be between {min_s:.3g}s and {max_s:.3g}s")


def make_row(root: tk.Tk, row: int, label: str, defaults: Sequence[int | float]) -> list[tk.Entry]:
    tk.Label(root, text=label).grid(row=row, column=0, padx=6, pady=(0, 4), sticky="e")
    out: list[tk.Entry] = []
    for i, value in enumerate(defaults, 1):
        e = tk.Entry(root, width=8)
        e.insert(0, str(value))
        e.grid(row=row, column=i, padx=4, pady=(0, 4))
        out.append(e)
    return out


class ArtificialHorizon(tk.Canvas):
    def __init__(self, parent: tk.Misc, size: int = 180) -> None:
        super().__init__(parent, width=size, height=size, bg="#101417", highlightthickness=0)
        self._size = size
        self._cx = size / 2
        self._cy = size / 2
        self._extent = size * 2.0
        self._pitch_px_per_deg = size / 90.0

        self._sky = self.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#4B95D9", outline="")
        self._ground = self.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#8C4F2A", outline="")
        self._horizon_line = self.create_line(0, 0, 0, 0, fill="white", width=2)

        self.create_oval(2, 2, size - 2, size - 2, outline="#C9D1D9", width=2)
        self.create_line(self._cx - 22, self._cy, self._cx + 22, self._cy, fill="#F6D32D", width=3)
        self.create_line(self._cx, self._cy - 8, self._cx, self._cy + 8, fill="#F6D32D", width=2)
        for deg in (-30, -20, -10, 10, 20, 30):
            y = self._cy - deg * self._pitch_px_per_deg
            self.create_line(self._cx - 10, y, self._cx + 10, y, fill="#FFFFFF", width=1)

        self.set_attitude(roll_deg=0.0, pitch_deg=0.0)

    def set_attitude(self, roll_deg: float, pitch_deg: float) -> None:
        roll_rad = math.radians(roll_deg)
        dx = math.cos(roll_rad)
        dy = math.sin(roll_rad)
        nx = -dy
        ny = dx

        pitch_shift = max(-45.0, min(45.0, pitch_deg)) * self._pitch_px_per_deg
        px = self._cx + nx * pitch_shift
        py = self._cy + ny * pitch_shift

        half = self._extent
        x1 = px - dx * half
        y1 = py - dy * half
        x2 = px + dx * half
        y2 = py + dy * half

        sx1 = x1 - nx * half
        sy1 = y1 - ny * half
        sx2 = x2 - nx * half
        sy2 = y2 - ny * half

        gx1 = x1 + nx * half
        gy1 = y1 + ny * half
        gx2 = x2 + nx * half
        gy2 = y2 + ny * half

        self.coords(self._sky, sx1, sy1, sx2, sy2, x2, y2, x1, y1)
        self.coords(self._ground, x1, y1, x2, y2, gx2, gy2, gx1, gy1)
        self.coords(self._horizon_line, x1, y1, x2, y2)

@dataclass
class MainUi:
    port_entry: tk.Entry
    ch_entries: list[tk.Entry]
    off_entries: list[tk.Entry]
    target_entries: list[tk.Entry]
    dur_entries: list[tk.Entry]
    hold_send_buttons: list[tk.Button]
    hold_end_buttons: list[tk.Button]
    start_button: tk.Button
    stop_button: tk.Button
    status: tk.StringVar
    pc_link_box: tk.Label
    horizon: ArtificialHorizon
    attitude_text: tk.StringVar
    fc_port_entry: tk.Entry
    fc_baud_entry: tk.Entry
    fc_link_box: tk.Label
    scan_fc_button: tk.Button
    connect_fc_button: tk.Button
    disconnect_fc_button: tk.Button


def build_main_gui(root: tk.Tk) -> MainUi:
    root.title("PPM Modbus")
    root.resizable(False, False)

    tk.Label(root, text="Port").grid(row=0, column=0, padx=6, pady=6, sticky="e")
    port_entry = tk.Entry(root, width=8)
    port_entry.insert(0, PORT_DEFAULT)
    port_entry.grid(row=0, column=1, padx=4, pady=6, sticky="w")

    for i, channel_name in enumerate(("Roll", "Pitch", "Throttle", "Yaw"), start=1):
        tk.Label(root, text=channel_name).grid(row=1, column=i, padx=4)

    ch_entries = make_row(root, 2, "Channels", CHANNEL_DEFAULTS)
    off_entries = make_row(root, 3, "Offsets", OFFSET_DEFAULTS)
    target_entries = make_row(root, 4, "Targets", PULSE_TARGET_DEFAULTS)
    dur_entries = make_row(root, 5, "Duration", PULSE_DURATION_DEFAULTS)

    tk.Label(root, text="Hold").grid(row=6, column=0, padx=6, pady=2, sticky="e")
    hold_send_buttons: list[tk.Button] = []
    for i in range(4):
        button = tk.Button(root, text="Send", width=8)
        button.grid(row=6, column=i + 1, pady=2)
        hold_send_buttons.append(button)

    tk.Label(root, text="End").grid(row=7, column=0, padx=6, pady=2, sticky="e")
    hold_end_buttons: list[tk.Button] = []
    for i in range(4):
        button = tk.Button(root, text="End", width=8)
        button.grid(row=7, column=i + 1, pady=2)
        hold_end_buttons.append(button)

    start_button = tk.Button(root, text="Start", width=12)
    start_button.grid(row=8, column=1, columnspan=2, pady=4)
    stop_button = tk.Button(root, text="Stop", width=12)
    stop_button.grid(row=8, column=3, columnspan=2, pady=4)

    status = tk.StringVar(value="Idle")
    tk.Label(root, textvariable=status, anchor="w").grid(row=9, column=0, columnspan=5, sticky="we", padx=6, pady=(0, 6))

    tk.Label(root, text="Links").grid(row=10, column=0, padx=6, pady=(0, 6), sticky="e")
    pc_link_box = tk.Label(root, width=18, relief="groove", bd=2)
    pc_link_box.grid(row=10, column=1, columnspan=4, padx=4, pady=(0, 6), sticky="we")

    fc_frame = tk.LabelFrame(root, text="FC / INAV", padx=8, pady=8)
    fc_frame.grid(row=0, column=5, rowspan=11, padx=(12, 6), pady=6, sticky="ns")
    horizon = ArtificialHorizon(fc_frame, size=180)
    horizon.grid(row=0, column=0, columnspan=3, pady=(0, 8))
    attitude_text = tk.StringVar(value="Roll: 0.0 deg  Pitch: 0.0 deg  Yaw: 0")
    tk.Label(fc_frame, textvariable=attitude_text, anchor="w", width=40).grid(
        row=1, column=0, columnspan=3, sticky="w", pady=(0, 8)
    )

    tk.Label(fc_frame, text="FC Port").grid(row=2, column=0, sticky="e", padx=(0, 4))
    fc_port_entry = tk.Entry(fc_frame, width=10)
    fc_port_entry.insert(0, FC_PORT_DEFAULT)
    fc_port_entry.grid(row=2, column=1, sticky="w")

    tk.Label(fc_frame, text="Baud").grid(row=3, column=0, sticky="e", padx=(0, 4), pady=(4, 0))
    fc_baud_entry = tk.Entry(fc_frame, width=10)
    fc_baud_entry.insert(0, str(FC_BAUD_DEFAULT))
    fc_baud_entry.grid(row=3, column=1, sticky="w", pady=(4, 0))

    fc_link_box = tk.Label(fc_frame, width=24, relief="groove", bd=2)
    fc_link_box.grid(row=4, column=0, columnspan=3, pady=(8, 6), sticky="we")

    scan_fc_button = tk.Button(fc_frame, text="Scan Ports", width=10)
    scan_fc_button.grid(row=2, column=2, padx=(6, 0))
    connect_fc_button = tk.Button(fc_frame, text="Connect FC", width=10)
    connect_fc_button.grid(row=5, column=0, pady=(2, 0))
    disconnect_fc_button = tk.Button(fc_frame, text="Disconnect FC", width=12)
    disconnect_fc_button.grid(row=5, column=1, columnspan=2, pady=(2, 0))

    return MainUi(
        port_entry=port_entry,
        ch_entries=ch_entries,
        off_entries=off_entries,
        target_entries=target_entries,
        dur_entries=dur_entries,
        hold_send_buttons=hold_send_buttons,
        hold_end_buttons=hold_end_buttons,
        start_button=start_button,
        stop_button=stop_button,
        status=status,
        pc_link_box=pc_link_box,
        horizon=horizon,
        attitude_text=attitude_text,
        fc_port_entry=fc_port_entry,
        fc_baud_entry=fc_baud_entry,
        fc_link_box=fc_link_box,
        scan_fc_button=scan_fc_button,
        connect_fc_button=connect_fc_button,
        disconnect_fc_button=disconnect_fc_button,
    )


def main() -> None:
    root = tk.Tk()
    ui = build_main_gui(root)
    port_entry = ui.port_entry
    ch_entries = ui.ch_entries
    off_entries = ui.off_entries
    target_entries = ui.target_entries
    dur_entries = ui.dur_entries
    hold_send_buttons = ui.hold_send_buttons
    hold_end_buttons = ui.hold_end_buttons
    start_button = ui.start_button
    stop_button = ui.stop_button
    status = ui.status
    pc_link_box = ui.pc_link_box
    horizon = ui.horizon
    attitude_text = ui.attitude_text
    fc_port_entry = ui.fc_port_entry
    fc_baud_entry = ui.fc_baud_entry
    fc_link_box = ui.fc_link_box
    scan_fc_button = ui.scan_fc_button
    connect_fc_button = ui.connect_fc_button
    disconnect_fc_button = ui.disconnect_fc_button

    run_active = False
    start_pending = False
    is_closing = False
    run_port = PORT_DEFAULT
    run_ser: serial.Serial | None = None
    run_quant: int | None = None
    run_max_count: int | None = None
    hold_timeout_after_id: str | None = None
    worker = SerialWorker()
    fc_service = InavSerialService()
    fc_poll_after_id: str | None = None

    def port() -> str:
        return port_entry.get().strip() or PORT_DEFAULT

    def fc_port() -> str:
        return fc_port_entry.get().strip() or FC_PORT_DEFAULT

    def fc_baud() -> int:
        try:
            value = int(fc_baud_entry.get().strip())
        except ValueError as exc:
            raise RuntimeError("FC baud must be an integer.") from exc
        if value <= 0:
            raise RuntimeError("FC baud must be > 0.")
        return value

    def select_fc_port(port_infos: Sequence[object]) -> str:
        target_id = FC_DEVICE_ID.upper()
        for info in port_infos:
            vid = getattr(info, "vid", None)
            pid = getattr(info, "pid", None)
            if vid == FC_DEVICE_VID and pid == FC_DEVICE_PID:
                device = str(getattr(info, "device", "") or "").strip()
                if device:
                    return device
            hwid = str(getattr(info, "hwid", "") or "")
            if target_id in hwid.upper():
                device = str(getattr(info, "device", "") or "").strip()
                if device:
                    return device
        return FC_PORT_DEFAULT

    def scan_fc_ports() -> None:
        port_infos = sorted(
            list_ports.comports(),
            key=lambda p: str(getattr(p, "device", "") or "").upper(),
        )
        ports = [str(getattr(p, "device", "") or "").strip() for p in port_infos]
        ports = [p for p in ports if p]
        selected_port = select_fc_port(port_infos)
        fc_port_entry.delete(0, tk.END)
        fc_port_entry.insert(0, selected_port)
        if ports:
            status.set(f"Detected ports: {', '.join(ports)}. FC port set to {selected_port}.")
        else:
            status.set(f"No serial ports detected. FC port set to {selected_port}.")

    def set_error(title: str, exc: Exception) -> None:
        if is_closing:
            return
        status.set("Error")
        messagebox.showerror(title, str(exc))

    def update_link_indicators() -> None:
        if run_ser is not None:
            pc_link_box.config(text="PC-ARD OPEN", bg="#2E7D32", fg="white")
        else:
            pc_link_box.config(text="PC-ARD CLOSED", bg="#8B1E1E", fg="white")
        fc_connected = fc_service.is_connected
        if fc_connected:
            fc_link_box.config(text="FC-INAV OPEN", bg="#2E7D32", fg="white")
        else:
            fc_link_box.config(text="FC-INAV CLOSED", bg="#8B1E1E", fg="white")
        connect_fc_button.config(state="disabled" if fc_connected else "normal")
        disconnect_fc_button.config(state="normal" if fc_connected else "disabled")

    def cancel_hold_timeout() -> None:
        nonlocal hold_timeout_after_id
        if hold_timeout_after_id is not None:
            try:
                root.after_cancel(hold_timeout_after_id)
            except Exception:
                pass
            finally:
                hold_timeout_after_id = None

    def close_run_connection() -> None:
        nonlocal run_ser, run_quant, run_max_count, run_active
        if run_ser is not None:
            try:
                run_ser.close()
            except Exception:
                pass
            finally:
                run_ser = None
                run_quant = None
                run_max_count = None
                run_active = False
                update_link_indicators()

    def do_fc_connect() -> None:
        try:
            if fc_service.is_connected:
                return
            selected_port = fc_port()
            selected_baud = fc_baud()
            fc_service.connect(selected_port, selected_baud)
            update_link_indicators()
            status.set(f"FC connected: {selected_port} @ {selected_baud}.")
        except Exception as exc:
            set_error("FC connect error", exc)

    def do_fc_disconnect(update_status: bool = True) -> None:
        try:
            fc_service.disconnect()
        except Exception as exc:
            if not is_closing:
                set_error("FC disconnect error", exc)
        finally:
            horizon.set_attitude(0.0, 0.0)
            attitude_text.set("Roll: 0.0 deg  Pitch: 0.0 deg  Yaw: 0")
            update_link_indicators()
            if update_status and not is_closing:
                status.set("FC disconnected.")

    def poll_fc_attitude() -> None:
        nonlocal fc_poll_after_id
        try:
            sample = fc_service.latest_attitude()
            if sample is not None:
                horizon.set_attitude(sample.roll_deg, sample.pitch_deg)
                attitude_text.set(
                    f"Roll: {sample.roll_deg:6.1f} deg  Pitch: {sample.pitch_deg:6.1f} deg  Yaw: {sample.yaw_deg:6.0f}"
                )
        except Exception:
            pass
        fc_poll_after_id = root.after(60, poll_fc_attitude)

    def _task_open_and_start(worker_self: SerialWorker, port: str, channels: list[int], offsets: list[int]):
        ser = open_serial(port)
        worker_self.ser = ser
        try:
            quant, max_count, version_warning = run_ppm_on_serial(ser, channels, offsets)
        except Exception:
            ser.close()
            worker_self.ser = None
            raise
        return (quant, max_count, version_warning)

    def _task_stop(worker_self: SerialWorker, port: str):
        if worker_self.ser is not None:
            try:
                stop_ppm_on_serial(worker_self.ser)
            finally:
                try:
                    worker_self.ser.close()
                finally:
                    worker_self.ser = None
            return None
        else:
            with open_serial(port) as ser:
                stop_ppm_on_serial(ser)
            return None

    def _task_hold(worker_self: SerialWorker, i: int, target: int, offset: int, timeout_s: float):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        quant, max_count = read_regs(worker_self.ser, REG_QUANT, 2)
        set_channel_until_stop_on_serial(worker_self.ser, quant, max_count, i, target, offset, timeout_s)
        return read_pulse_status_on_serial(worker_self.ser, max_count)

    def _task_hold_end(worker_self: SerialWorker, i: int):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        _, max_count = read_regs(worker_self.ser, REG_QUANT, 2)
        end_hold_on_serial(worker_self.ser, max_count, i)
        return read_pulse_status_on_serial(worker_self.ser, max_count)

    def _task_run_ppm_on_existing(worker_self: SerialWorker, channels: list[int], offsets: list[int]):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        return run_ppm_on_serial(worker_self.ser, channels, offsets)

    def _task_read_pulse_status(worker_self: SerialWorker, max_count: int):
        if worker_self.ser is None:
            raise RuntimeError("Serial not open")
        return read_pulse_status_on_serial(worker_self.ser, max_count)

    def _task_shutdown(worker_self: SerialWorker):
        if worker_self.ser is None:
            return None
        try:
            stop_ppm_on_serial(worker_self.ser)
        except Exception:
            pass
        finally:
            try:
                worker_self.ser.close()
            finally:
                worker_self.ser = None
        return None

    def poll_results() -> None:
        while True:
            try:
                cb, ok, res = worker.results.get_nowait()
            except queue.Empty:
                break
            if cb:
                try:
                    cb(ok, res)
                except Exception as e:
                    set_error("Callback error", e)
        root.after(50, poll_results)

    def do_start() -> None:
        nonlocal run_active, run_port, run_ser, run_quant, run_max_count, start_pending
        try:
            if start_pending:
                raise RuntimeError("Start is already in progress.")
            if hold_timeout_after_id is not None:
                raise RuntimeError("Wait for active Hold to finish or press End/Stop.")
            channels = parse_entries(ch_entries, int, "Channel")
            require_range(channels, "Channel", 1000, 2000)
            offsets = parse_entries(off_entries, int, "Offset")
            selected_port = port()

            def on_start_done(ok: bool, res: object) -> None:
                nonlocal run_active, run_port, run_ser, run_quant, run_max_count, start_pending
                start_pending = False
                if not ok:
                    set_error("Start error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if (
                    not isinstance(res, tuple)
                    or len(res) != 3
                    or not isinstance(res[0], int)
                    or not isinstance(res[1], int)
                    or (res[2] is not None and not isinstance(res[2], str))
                ):
                    set_error("Start error", RuntimeError("Unexpected worker result from start task"))
                    return
                # success
                run_port = selected_port
                run_quant = res[0]
                run_max_count = res[1]
                run_ser = worker.ser
                run_active = True
                update_link_indicators()
                version_warning = res[2]
                if version_warning:
                    status.set(version_warning)
                    messagebox.showwarning("Firmware version", version_warning)
                else:
                    status.set("PPM output configured and started.")

            if run_ser is None:
                start_pending = True
                worker.submit(_task_open_and_start, selected_port, channels, offsets, callback=on_start_done)
            else:
                if selected_port != run_port:
                    raise RuntimeError(f"Output is active on {run_port}. Press Stop before switching ports.")
                start_pending = True
                worker.submit(_task_run_ppm_on_existing, channels, offsets, callback=on_start_done)

        except Exception as exc:
            start_pending = False
            set_error("Start error", exc)

    def do_stop() -> None:
        try:
            cancel_hold_timeout()
            def on_stop_done(ok: bool, res: object) -> None:
                nonlocal run_ser, run_quant, run_max_count, run_active
                if not ok:
                    set_error("Stop error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if res is not None:
                    set_error("Stop error", RuntimeError("Unexpected worker result from stop task"))
                    return
                run_ser = None
                run_quant = None
                run_max_count = None
                run_active = False
                update_link_indicators()
                status.set("PPM output stopped.")

            worker.submit(_task_stop, port(), callback=on_stop_done)
        except Exception as exc:
            set_error("Stop error", exc)

    def do_hold_send(i: int) -> None:
        nonlocal run_max_count, hold_timeout_after_id
        try:
            if not run_active or run_ser is None:
                raise RuntimeError("Press Start before using Hold.")
            if hold_timeout_after_id is not None:
                raise RuntimeError("A hold command is already active. Wait for timeout or press End.")

            offsets = parse_entries(off_entries, int, "Offset")
            targets = parse_entries(target_entries, int, "Target")
            require_range(targets, "Target", 1000, 2000)
            durations = parse_entries(dur_entries, float, "Duration")
            require_duration_range(durations, 0.05, 60.0)
            timeout_s = durations[i]

            def on_hold_done(ok: bool, res: object) -> None:
                nonlocal hold_timeout_after_id, run_max_count
                if not ok:
                    set_error("Hold error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, int):
                    set_error("Hold error", RuntimeError("Unexpected worker result from hold task"))
                    return
                pulse_status = res
                if pulse_status == PULSE_STATUS_REJECTED:
                    set_error("Hold error", RuntimeError("Firmware rejected hold command"))
                    return

                timeout_ms = max(1, round(timeout_s * 1000))
                chan_label = i + 1

                def schedule_check() -> None:
                    def cb(ok2: bool, res2: object) -> None:
                        nonlocal hold_timeout_after_id
                        if not ok2:
                            set_error("Hold timeout error", res2 if isinstance(res2, Exception) else RuntimeError(res2))
                            hold_timeout_after_id = None
                            return
                        if not isinstance(res2, int):
                            set_error("Hold timeout error", RuntimeError("Unexpected worker result from pulse-status task"))
                            hold_timeout_after_id = None
                            return
                        pulse_status_now = res2
                        if pulse_status_now not in (PULSE_STATUS_TIMEOUT_RESTORED, PULSE_STATUS_HOLD_ENDED):
                            hold_timeout_after_id = root.after(20, schedule_check)
                            return
                        hold_timeout_after_id = None
                        if pulse_status_now == PULSE_STATUS_TIMEOUT_RESTORED:
                            status.set(f"CH{chan_label} hold timed out; channel restored.")
                        else:
                            status.set(f"CH{chan_label} hold ended; channel restored.")

                    worker.submit(_task_read_pulse_status, run_max_count or 0, callback=cb)

                hold_timeout_after_id = root.after(timeout_ms, schedule_check)
                status.set(f"CH{chan_label} hold active. Press End for early restore (auto in {timeout_s:.3g}s).")

            worker.submit(_task_hold, i, targets[i], offsets[i], timeout_s, callback=on_hold_done)
        except Exception as exc:
            set_error("Hold error", exc)

    def do_hold_end(i: int) -> None:
        try:
            if not run_active or run_ser is None:
                raise RuntimeError("Press Start before ending Hold.")
            if hold_timeout_after_id is None:
                raise RuntimeError("No active Hold to end.")

            def on_hold_end_done(ok: bool, res: object) -> None:
                if not ok:
                    set_error("Hold end error", res if isinstance(res, Exception) else RuntimeError(res))
                    return
                if not isinstance(res, int):
                    set_error("Hold end error", RuntimeError("Unexpected worker result from hold-end task"))
                    return
                if res == PULSE_STATUS_REJECTED:
                    set_error("Hold end error", RuntimeError("Firmware rejected hold-end command"))
                    return
                cancel_hold_timeout()
                status.set(f"CH{i + 1} hold ended; channel restored.")

            worker.submit(_task_hold_end, i, callback=on_hold_end_done)
        except Exception as exc:
            set_error("Hold end error", exc)

    def on_close() -> None:
        nonlocal is_closing, fc_poll_after_id
        is_closing = True
        cancel_hold_timeout()
        if fc_poll_after_id is not None:
            try:
                root.after_cancel(fc_poll_after_id)
            except Exception:
                pass
            finally:
                fc_poll_after_id = None

        def on_stop_and_close(ok: bool, res: object) -> None:
            do_fc_disconnect(update_status=False)
            try:
                worker.stop()
            except Exception:
                pass
            close_run_connection()
            root.destroy()

        try:
            worker.submit(_task_shutdown, callback=on_stop_and_close)
        except Exception:
            on_stop_and_close(False, None)

    initial_port_infos = sorted(
        list_ports.comports(),
        key=lambda p: str(getattr(p, "device", "") or "").upper(),
    )
    fc_port_entry.delete(0, tk.END)
    fc_port_entry.insert(0, select_fc_port(initial_port_infos))

    scan_fc_button.config(command=scan_fc_ports)
    connect_fc_button.config(command=do_fc_connect)
    disconnect_fc_button.config(command=do_fc_disconnect)
    for i, button in enumerate(hold_send_buttons):
        button.config(command=lambda i=i: do_hold_send(i))
    for i, button in enumerate(hold_end_buttons):
        button.config(command=lambda i=i: do_hold_end(i))
    start_button.config(command=do_start)
    stop_button.config(command=do_stop)
    update_link_indicators()
    root.after(50, poll_results)
    fc_poll_after_id = root.after(60, poll_fc_attitude)
    root.protocol("WM_DELETE_WINDOW", on_close)

    root.mainloop()


if __name__ == "__main__":
    main()

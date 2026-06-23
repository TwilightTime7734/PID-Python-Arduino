import unittest
from unittest.mock import patch

from modbus_app.constants import PULSE_COMMAND_CANCEL, PULSE_COMMAND_START_FIXED, REG_CHANNEL0
from modbus_app.serial_protocol import cancel_active_pulse_on_serial, start_fixed_pulse_on_serial


class SerialPulseCommandTests(unittest.TestCase):
    def test_start_fixed_pulse_writes_channel_command_signed_force_and_seq(self) -> None:
        writes: list[tuple[int, list[int]]] = []

        def fake_read_regs(_ser, start: int, count: int) -> list[int]:
            self.assertEqual(start, REG_CHANNEL0 + 16 + 4)
            self.assertEqual(count, 1)
            return [41]

        def fake_write_regs(_ser, start: int, values: list[int]) -> None:
            writes.append((start, values))

        with (
            patch("modbus_app.serial_protocol.read_regs", side_effect=fake_read_regs),
            patch("modbus_app.serial_protocol.write_regs", side_effect=fake_write_regs),
        ):
            start_fixed_pulse_on_serial(object(), max_count=16, chl=2, force_us=-45)

        self.assertEqual(writes, [(REG_CHANNEL0 + 16, [2, PULSE_COMMAND_START_FIXED, 0xFFD3, 0, 42])])

    def test_cancel_active_pulse_writes_cancel_command_without_force(self) -> None:
        writes: list[tuple[int, list[int]]] = []

        with (
            patch("modbus_app.serial_protocol.read_regs", return_value=[0xFFFF]),
            patch("modbus_app.serial_protocol.write_regs", side_effect=lambda _ser, start, values: writes.append((start, values))),
        ):
            cancel_active_pulse_on_serial(object(), max_count=16)

        self.assertEqual(writes, [(REG_CHANNEL0 + 16, [0, PULSE_COMMAND_CANCEL, 0, 0, 0])])


if __name__ == "__main__":
    unittest.main()

import unittest

from modbus_app.ch8_marker import channels_with_pid_test_ch8
from modbus_app.constants import (
    PID_TEST_CH8_CHANNEL_INDEX,
    PID_TEST_CH8_OFF_US,
    PID_TEST_CH8_ON_US,
    PPM_OUTPUT_CHANNEL_COUNT,
)
from modbus_app.hardware_controller import HardwareController


class HardwareControllerTests(unittest.TestCase):
    def test_ppm_channels_pad_ch8_low_by_default(self) -> None:
        channels = [1500, 1500, 1200, 1500]

        output = HardwareController._ppm_channels_for_firmware(channels)

        self.assertEqual(PPM_OUTPUT_CHANNEL_COUNT, len(output))
        self.assertEqual(channels, output[: len(channels)])
        self.assertEqual(PID_TEST_CH8_OFF_US, output[PID_TEST_CH8_CHANNEL_INDEX])

    def test_pid_test_ch8_helper_sets_channel_value(self) -> None:
        channels = [1500, 1500, 1200, 1500]

        off = channels_with_pid_test_ch8(channels, active=False)
        on = channels_with_pid_test_ch8(channels, active=True)

        self.assertEqual(PID_TEST_CH8_OFF_US, off[PID_TEST_CH8_CHANNEL_INDEX])
        self.assertEqual(PID_TEST_CH8_ON_US, on[PID_TEST_CH8_CHANNEL_INDEX])


if __name__ == "__main__":
    unittest.main()

import unittest

from modbus_app.constants import (
    BEEPER_MARKER_CHANNEL_INDEX,
    BEEPER_MARKER_OFF_US,
    BEEPER_MARKER_ON_US,
    PPM_OUTPUT_CHANNEL_COUNT,
)
from modbus_app.hardware_controller import HardwareController


class HardwareControllerTests(unittest.TestCase):
    def test_ppm_channels_use_ch8_beeper_marker(self) -> None:
        channels = [1500, 1500, 1200, 1500]

        marker_off = HardwareController._ppm_channels_for_firmware(channels, marker_active=False)
        marker_on = HardwareController._ppm_channels_for_firmware(channels, marker_active=True)

        self.assertEqual(PPM_OUTPUT_CHANNEL_COUNT, len(marker_off))
        self.assertEqual(PPM_OUTPUT_CHANNEL_COUNT, len(marker_on))
        self.assertEqual(channels, marker_off[: len(channels)])
        self.assertEqual(channels, marker_on[: len(channels)])
        self.assertEqual(BEEPER_MARKER_OFF_US, marker_off[BEEPER_MARKER_CHANNEL_INDEX])
        self.assertEqual(BEEPER_MARKER_ON_US, marker_on[BEEPER_MARKER_CHANNEL_INDEX])


if __name__ == "__main__":
    unittest.main()

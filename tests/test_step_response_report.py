import tempfile
import unittest
from pathlib import Path

import numpy as np

from modbus_app.step_response_report import (
    _detect_beeper_mode_flag_window,
    _flight_mode_has_beeper_on,
)


class StepResponseMarkerTests(unittest.TestCase):
    def test_flight_mode_detects_beeper_on_only(self) -> None:
        self.assertTrue(_flight_mode_has_beeper_on("ANGLE|BEEPERON"))
        self.assertFalse(_flight_mode_has_beeper_on("ANGLE|BEEPER"))

    def test_marker_window_uses_first_beeperon_bracket(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            csv_path = Path(tmp_dir) / "sample.csv"
            csv_path.write_text(
                "time (us),flightModeFlags (flags),axisRate[0],gyroADC[0] (deg/s)\n"
                "0,ANGLE,\n"
                "100000,ANGLE|BEEPERON,\n"
                "200000,ANGLE|BEEPERON,\n"
                "300000,ANGLE,\n"
                "400000,ANGLE|BEEPERON,\n",
                encoding="utf-8",
            )
            columns = {
                "time (us)": np.asarray([0, 100_000, 200_000, 300_000, 400_000], dtype=float),
                "axisRate[0]": np.zeros(5),
                "gyroADC[0] (deg/s)": np.zeros(5),
            }
            marker = _detect_beeper_mode_flag_window(csv_path, columns, "time (us)")

            self.assertIsNotNone(marker)
            assert marker is not None
            self.assertEqual(1, marker.start_index)
            self.assertEqual(3, marker.end_index)
            self.assertIn("CH8 beeper marker window", marker.warning)


if __name__ == "__main__":
    unittest.main()

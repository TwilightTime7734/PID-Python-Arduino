import unittest

import numpy as np

from modbus_app.blackbox_trace_viewer import detect_pitch_columns, detect_roll_columns


class BlackboxTraceViewerTests(unittest.TestCase):
    def test_detect_roll_columns_does_not_prefer_axisrate_for_setpoint(self) -> None:
        columns = {
            "axisRate[0]": np.array([1.0, 2.0, 3.0], dtype=float),
            "setpoint[0]": np.array([4.0, 5.0, 6.0], dtype=float),
            "gyroADC[0]": np.array([7.0, 8.0, 9.0], dtype=float),
        }

        detected = detect_roll_columns(columns)
        self.assertEqual(detected["setpoint"], "setpoint[0]")
        self.assertEqual(detected["gyro"], "gyroADC[0]")

    def test_detect_roll_columns_ignores_axisrate_when_no_setpoint_columns_exist(self) -> None:
        columns = {
            "axisRate[0]": np.array([1.0, 2.0, 3.0], dtype=float),
            "gyroADC[0]": np.array([7.0, 8.0, 9.0], dtype=float),
        }

        detected = detect_roll_columns(columns)
        self.assertIsNone(detected["setpoint"])
        self.assertEqual(detected["gyro"], "gyroADC[0]")

    def test_detect_pitch_columns_uses_pitch_axis_columns(self) -> None:
        columns = {
            "axisRate[1]": np.array([1.0, 2.0, 3.0], dtype=float),
            "setpoint[1]": np.array([4.0, 5.0, 6.0], dtype=float),
            "gyroADC[1]": np.array([7.0, 8.0, 9.0], dtype=float),
            "setpoint[0]": np.array([10.0, 11.0, 12.0], dtype=float),
            "gyroADC[0]": np.array([13.0, 14.0, 15.0], dtype=float),
        }

        detected = detect_pitch_columns(columns)
        self.assertEqual(detected["setpoint"], "setpoint[1]")
        self.assertEqual(detected["gyro"], "gyroADC[1]")


if __name__ == "__main__":
    unittest.main()

import tempfile
import unittest
from pathlib import Path

from modbus_app.flylog_sample_report import analyze_flylog_csv


class FlyLogSampleReportTests(unittest.TestCase):
    def test_one_axis_analysis_does_not_require_fixed_pulse_count(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            csv_path = Path(tmp_dir) / "one_axis.csv"
            rows = [
                "time (us),flightModeFlags (flags),rcData[0],rcData[1],attitude[0],attitude[1],axisRate[0],gyroADC[0]",
            ]
            time_us = 0
            pulse_values = [1600, 1400, 1600, 1400]
            attitude = 0
            for value in pulse_values:
                for _ in range(5):
                    attitude += 20 if value > 1500 else -20
                    rows.append(f"{time_us},ANGLE|BEEPERON,{value},1500,{attitude},0,100,90")
                    time_us += 50_000
                for _ in range(2):
                    rows.append(f"{time_us},ANGLE|BEEPERON,1500,1500,{attitude},0,0,0")
                    time_us += 50_000
            rows.append(f"{time_us},ANGLE,1500,1500,{attitude},0,0,0")
            csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

            metrics, groups, _center_adjusts, summary = analyze_flylog_csv(csv_path)

            self.assertEqual(4, summary.pulse_count)
            self.assertEqual(2, len(groups))
            self.assertEqual(2, summary.complete_groups)
            self.assertEqual(4, summary.roll_pulses)
            self.assertEqual(0, summary.pitch_pulses)
            self.assertFalse(any("expected" in warning.lower() for warning in summary.warnings))
            self.assertTrue(all(metric.sequence_ok for metric in metrics))


if __name__ == "__main__":
    unittest.main()

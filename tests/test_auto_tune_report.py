import tempfile
import unittest
from pathlib import Path

from modbus_app.auto_tune_report import generate_auto_tune_report
from modbus_app.blackbox_import import BlackboxImportResult


class AutoTuneReportTests(unittest.TestCase):
    def test_report_generation_creates_expected_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            base = Path(tmp_dir)
            csv_path = base / "LOG00001.01.csv"
            self._write_sample_csv(csv_path)

            analysis_result = BlackboxImportResult(
                scanned_roots=(str(base),),
                imported_files=tuple(),
                skipped_count=0,
                warnings=tuple(),
                analysis_summary="ok",
                analysis_source=str(csv_path),
                pid_report=None,
            )
            report = generate_auto_tune_report(
                output_root=base,
                analysis_result=analysis_result,
                session_payload={
                    "state": "report_ready",
                    "stop_reason": "unit test",
                    "warning": "",
                    "elapsed_s": 60.0,
                    "metrics": {},
                },
                preferred_log_path=str(csv_path),
            )

            self.assertTrue(Path(report.summary_txt).exists())
            self.assertTrue(Path(report.summary_json).exists())
            self.assertTrue(Path(report.combined_chart_sheet).exists())
            self.assertEqual(6, len(report.chart_paths))
            for item in report.chart_paths:
                self.assertTrue(Path(item).exists())

    def _write_sample_csv(self, path: Path) -> None:
        header = [
            "loopIteration",
            "time (us)",
            "axisRate[0]",
            "axisP[0]",
            "axisI[0]",
            "axisD[0]",
            "axisF[0]",
            "rcCommand[0]",
            "motor[0]",
            "motor[1]",
            "motor[2]",
            "motor[3]",
            "gyroRaw[0]",
        ]
        rows = [",".join(header)]
        for i in range(1200):
            t_us = i * 1000
            setpoint = 160.0 if (i // 100) % 2 == 0 else -140.0
            actual = setpoint * 0.82
            axp = setpoint * 0.12
            axi = setpoint * 0.03
            axd = setpoint * 0.05
            axf = setpoint * 0.06
            m0 = 1100 + ((i % 40) * 5)
            m1 = 1120 + ((i % 35) * 6)
            m2 = 1110 + ((i % 30) * 7)
            m3 = 1130 + ((i % 28) * 4)
            gyro = actual + (12.0 if (i % 50) < 5 else -9.0)
            row = [
                str(i),
                str(t_us),
                f"{actual:.3f}",
                f"{axp:.3f}",
                f"{axi:.3f}",
                f"{axd:.3f}",
                f"{axf:.3f}",
                f"{setpoint:.3f}",
                f"{m0:.3f}",
                f"{m1:.3f}",
                f"{m2:.3f}",
                f"{m3:.3f}",
                f"{gyro:.3f}",
            ]
            rows.append(",".join(row))
        path.write_text("\n".join(rows) + "\n", encoding="utf-8")


if __name__ == "__main__":
    unittest.main()

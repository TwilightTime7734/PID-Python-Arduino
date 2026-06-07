import json
import tempfile
import unittest
from pathlib import Path

from modbus_app.pid_tuning_workflow import (
    PAVO_PICO_II_PRESET_INPUTS,
    PStartInputs,
    find_latest_pid_tuning_plan,
    format_pid_tuning_plan,
    generate_pid_tuning_plan_report,
    load_pid_tuning_plan,
    suggest_starting_p,
)


class PIDTuningWorkflowTests(unittest.TestCase):
    def test_default_plan_tunes_roll_pitch_and_keeps_yaw_final_only(self) -> None:
        recommendation = suggest_starting_p(PStartInputs())

        self.assertEqual(recommendation.start_p, {"roll": 45, "pitch": 47})
        self.assertEqual(recommendation.start_i, {"roll": 30, "pitch": 35})
        self.assertEqual(recommendation.p_sweep["roll"], (40, 45, 50, 55))
        self.assertEqual(recommendation.p_sweep["pitch"], (42, 47, 52, 57))
        self.assertEqual(recommendation.yaw_final_pid_ff, {"p": 45, "i": 60, "d": 0, "ff": 86})

        plan = format_pid_tuning_plan(recommendation)
        self.assertIn("D tuning, roll/pitch only", plan)
        self.assertIn("- Roll:  P 45, D 17, I 30, FF 0", plan)
        self.assertIn("- Pitch: P 47, D 17, I 35, FF 0", plan)
        self.assertIn("Yaw final recommendation, not tested", plan)
        self.assertNotIn("Yaw P candidates", plan)
        self.assertNotIn("Write values while disarmed", plan)
        self.assertNotIn("Use Y Correction", plan)
        self.assertNotIn("Keep final picks supervised", plan)

    def test_high_risk_inputs_reduce_starting_p(self) -> None:
        recommendation = suggest_starting_p(
            PStartInputs(
                all_up_weight_g=950,
                motor_kv=4500,
                prop_diameter_in=3.0,
                prop_pitch_in=5.5,
                battery_cells=6,
                battery_chemistry="lipo",
                motor_count=4,
            )
        )

        self.assertLess(recommendation.start_p["roll"], 45)
        self.assertLess(recommendation.start_p["pitch"], 47)
        self.assertGreaterEqual(recommendation.start_p["roll"], 15)
        self.assertGreaterEqual(recommendation.start_p["pitch"], 15)

    def test_pavo_pico_ii_preset_uses_stock_betafpv_specs(self) -> None:
        self.assertEqual(PAVO_PICO_II_PRESET_INPUTS.all_up_weight_g, 83)
        self.assertEqual(PAVO_PICO_II_PRESET_INPUTS.motor_kv, 14000)
        self.assertEqual(PAVO_PICO_II_PRESET_INPUTS.prop_diameter_in, 1.77)
        self.assertEqual(PAVO_PICO_II_PRESET_INPUTS.prop_pitch_in, 1.5)
        self.assertEqual(PAVO_PICO_II_PRESET_INPUTS.battery_cells, 2)
        self.assertEqual(PAVO_PICO_II_PRESET_INPUTS.battery_chemistry, "lihv")
        self.assertEqual(PAVO_PICO_II_PRESET_INPUTS.motor_count, 4)

    def test_report_generation_writes_text_and_summary(self) -> None:
        recommendation = suggest_starting_p(PStartInputs())
        with tempfile.TemporaryDirectory() as tmp_dir:
            report = generate_pid_tuning_plan_report(tmp_dir, recommendation)

            self.assertTrue(Path(report.text_path).exists())
            self.assertTrue(Path(report.summary_json).exists())
            self.assertIn("pid_tuning_plan", report.report_dir)
            plan_text = Path(report.text_path).read_text(encoding="utf-8").strip()
            summary = json.loads(Path(report.summary_json).read_text(encoding="utf-8"))
            self.assertEqual(summary["plan"]["format"], "sample")
            self.assertEqual(summary["plan"]["text"], plan_text)
            self.assertIn("Once the 'Fly/Log' button is pressed", summary["plan"]["text"])

    def test_load_generated_pid_tuning_plan_uses_summary_values(self) -> None:
        recommendation = suggest_starting_p(PAVO_PICO_II_PRESET_INPUTS)
        with tempfile.TemporaryDirectory() as tmp_dir:
            report = generate_pid_tuning_plan_report(tmp_dir, recommendation)

            loaded = load_pid_tuning_plan(report.text_path)

            self.assertEqual(loaded.start_p, recommendation.start_p)
            self.assertEqual(loaded.start_i, {"roll": 30, "pitch": 35})
            self.assertEqual(loaded.p_sweep, recommendation.p_sweep)
            self.assertEqual(loaded.d_sweep, (17, 23, 30, 36, 42))
            self.assertIsNone(loaded.optional_d)
            self.assertEqual(loaded.yaw_final_pid_ff, recommendation.yaw_final_pid_ff)

    def test_find_latest_pid_tuning_plan_returns_newest_text_file(self) -> None:
        recommendation = suggest_starting_p(PStartInputs())
        with tempfile.TemporaryDirectory() as tmp_dir:
            first = generate_pid_tuning_plan_report(tmp_dir, recommendation)
            second = generate_pid_tuning_plan_report(tmp_dir, recommendation)
            Path(second.text_path).touch()

            latest = find_latest_pid_tuning_plan(tmp_dir)

            self.assertEqual(latest, Path(second.text_path))


if __name__ == "__main__":
    unittest.main()

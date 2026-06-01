import unittest

from modbus_app.adaptive_session import (
    AdaptiveExcitationController,
    AdaptiveSessionConfig,
    ExcitationEvent,
)


class AdaptiveControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = AdaptiveSessionConfig(min_runtime_s=1.0, max_runtime_s=10.0)
        self.controller = AdaptiveExcitationController(self.cfg)

    def _event(self, axis: str, direction: int, peak: float, settle_success: bool = True) -> ExcitationEvent:
        return ExcitationEvent(
            axis=axis,
            direction=direction,
            force_us=180,
            hold_s=0.3,
            settle_s=0.4,
            baseline_angle_deg=0.0,
            peak_delta_deg=peak,
            settle_success=settle_success,
            response_delay_s=0.12,
            final_error_deg=0.8,
        )

    def test_axis_preference_favors_lower_coverage(self) -> None:
        for _ in range(8):
            self.controller.record_event(self._event("roll", -1, 10.0))
            self.controller.record_event(self._event("roll", +1, 11.0))
        command = self.controller.next_command(roll_deg=0.0, pitch_deg=0.0, recovery_mode=False)
        self.assertIsNotNone(command)
        self.assertEqual("pitch", command.axis)

    def test_direction_bias_moves_toward_center_when_tilted(self) -> None:
        command = self.controller.next_command(roll_deg=24.0, pitch_deg=0.0, recovery_mode=False)
        self.assertIsNotNone(command)
        if command.axis == "roll":
            self.assertEqual(-1, command.direction)

    def test_force_is_clamped_and_reduced_near_limit(self) -> None:
        low_angle = self.controller.next_command(roll_deg=0.0, pitch_deg=0.0, recovery_mode=False)
        high_angle = self.controller.next_command(roll_deg=37.0, pitch_deg=0.0, recovery_mode=False)
        self.assertIsNotNone(low_angle)
        self.assertIsNotNone(high_angle)
        self.assertGreaterEqual(low_angle.force_us, self.cfg.force_min_us)
        self.assertLessEqual(low_angle.force_us, self.cfg.force_max_us)
        if high_angle.axis == "roll" and high_angle.direction > 0:
            self.assertLessEqual(high_angle.force_us, low_angle.force_us)

    def test_recovery_entry_and_exit_rules(self) -> None:
        self.assertTrue(self.controller.should_recover(43.5, 2.0))
        self.assertFalse(self.controller.recovery_complete(21.0, 0.0))
        self.assertTrue(self.controller.recovery_complete(10.0, 15.0))

    def test_stop_rule_needs_min_runtime_then_confidence(self) -> None:
        ready, _, _ = self.controller.stop_ready(0.5)
        self.assertFalse(ready)

        for axis in ("roll", "pitch"):
            for direction in (-1, +1):
                for _ in range(self.cfg.target_valid_events):
                    self.controller.record_event(self._event(axis, direction, 12.0, settle_success=True))

        ready, reason, warning = self.controller.stop_ready(2.0)
        self.assertTrue(ready)
        self.assertIn("Coverage confidence", reason)
        self.assertEqual("", warning)

    def test_abort_on_hard_limit(self) -> None:
        abort, reason = self.controller.should_abort(45.0, 0.0)
        self.assertTrue(abort)
        self.assertIn("Hard safety limit", reason)


if __name__ == "__main__":
    unittest.main()

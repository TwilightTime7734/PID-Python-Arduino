import random
import unittest

from modbus_app.adaptive_session import (
    AdaptiveExcitationController,
    AdaptiveSessionConfig,
    ExcitationEvent,
)


class AdaptiveControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = AdaptiveSessionConfig(max_runtime_s=60.0)
        self.controller = AdaptiveExcitationController(self.cfg, rng=random.Random(7))

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

    def test_random_cycle_queues_roll_and_pitch_commands(self) -> None:
        first = self.controller.next_command(roll_deg=0.0, pitch_deg=0.0, recovery_mode=False)
        second = self.controller.next_command(roll_deg=0.0, pitch_deg=0.0, recovery_mode=False)

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        self.assertEqual({"roll", "pitch"}, {first.axis, second.axis})
        for command in (first, second):
            self.assertGreaterEqual(command.force_us, self.cfg.force_min_us)
            self.assertLessEqual(command.force_us, self.cfg.force_max_us)
            self.assertGreaterEqual(command.hold_s, self.cfg.hold_min_s)
            self.assertLessEqual(command.hold_s, self.cfg.hold_max_s)
            self.assertGreaterEqual(command.target_peak_deg, self.cfg.target_peak_min_deg)

    def test_unsafe_away_from_limit_direction_is_flipped_toward_center(self) -> None:
        command = self.controller._random_command_for_axis("roll", current_angle=42.0, direction=1)

        self.assertIsNotNone(command)
        self.assertEqual(-1, command.direction)

    def test_command_peak_is_bounded_by_remaining_angle_margin(self) -> None:
        command = self.controller._random_command_for_axis("roll", current_angle=30.0, direction=1)

        self.assertIsNotNone(command)
        safe_limit = self.cfg.hard_limit_deg - self.cfg.safety_margin_deg
        self.assertLessEqual(command.target_peak_deg, safe_limit - 30.0)

    def test_stop_rule_ends_at_sixty_seconds(self) -> None:
        ready, _, _ = self.controller.stop_ready(59.9)
        self.assertFalse(ready)

        ready, reason, warning = self.controller.stop_ready(60.0)
        self.assertTrue(ready)
        self.assertIn("60-second", reason)
        self.assertEqual("", warning)

    def test_abort_on_hard_limit(self) -> None:
        abort, reason = self.controller.should_abort(45.0, 0.0)
        self.assertTrue(abort)
        self.assertIn("Hard safety limit", reason)

    def test_recovery_starts_before_hard_limit_and_completes_near_center(self) -> None:
        self.assertTrue(self.controller.should_recover(0.0, -12.0))
        self.assertFalse(self.controller.recovery_complete(0.0, -12.0))
        self.assertTrue(self.controller.recovery_complete(2.0, -3.0))

    def test_recovery_command_points_toward_center_on_largest_axis(self) -> None:
        command = self.controller.next_command(roll_deg=6.0, pitch_deg=30.0, recovery_mode=True)

        self.assertIsNotNone(command)
        self.assertTrue(command.recovery)
        self.assertEqual("pitch", command.axis)
        self.assertEqual(-1, command.direction)
        self.assertLessEqual(command.force_us, self.cfg.recovery_force_us)

    def test_initial_throttle_applies_floor_and_cap(self) -> None:
        self.assertEqual(1260, self.cfg.throttle_start_us)

        target, reason = self.controller.initial_throttle(1200)
        self.assertEqual(self.cfg.throttle_start_us, target)
        self.assertIn("floor", reason)

        target, reason = self.controller.initial_throttle(1700)
        self.assertEqual(self.cfg.throttle_max_us, target)
        self.assertIn("capped", reason)

    def test_throttle_is_not_changed_after_random_events(self) -> None:
        target, reason = self.controller.throttle_after_event(1300, self._event("roll", 1, 1.5))

        self.assertEqual(1300, target)
        self.assertEqual("", reason)

    def test_metrics_record_random_events(self) -> None:
        self.controller.record_event(self._event("roll", 1, 10.0))
        metrics = self.controller.coverage_metrics()

        self.assertEqual(1, metrics.direction["roll_pos"].total_count)
        self.assertGreater(metrics.axis_confidence["roll"], 0.0)


if __name__ == "__main__":
    unittest.main()

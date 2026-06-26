import unittest
from types import SimpleNamespace
from unittest.mock import patch

from modbus_app.attitude_service import AttitudeSample, AttitudeService
from modbus_app.tasks import worker_tasks
from modbus_app.workflows.auto_session_helpers import AutoSessionHelpers


class AttitudeServiceTests(unittest.TestCase):
    def sample(self, *, roll: float, pitch: float, millis: int, seq: int) -> AttitudeSample:
        return AttitudeSample(
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=0.0,
            movement_millis=millis,
            movement_seq=seq,
        )

    def test_duplicate_arduino_sample_is_rejected(self) -> None:
        service = AttitudeService()
        service.REFERENCE_CAPTURE_SECONDS = 0.0
        service.connect()

        first = self.sample(roll=10.0, pitch=5.0, millis=1000, seq=7)
        duplicate = self.sample(roll=20.0, pitch=15.0, millis=1000, seq=7)

        self.assertTrue(service.ingest_sample(first))
        self.assertFalse(service.ingest_sample(duplicate))
        absolute = service.latest_absolute_attitude()
        self.assertIsNotNone(absolute)
        self.assertEqual(10.0, absolute.roll_deg)
        self.assertEqual(5.0, absolute.pitch_deg)

    def test_missing_arduino_timing_is_rejected(self) -> None:
        service = AttitudeService()
        service.connect()
        malformed = AttitudeSample(
            roll_deg=1.0,
            pitch_deg=2.0,
            yaw_deg=0.0,
            movement_millis=None,  # type: ignore[arg-type]
            movement_seq=1,
        )

        with self.assertRaisesRegex(ValueError, "Arduino movement_millis"):
            service.ingest_sample(malformed)

    def test_changed_arduino_sequence_or_timestamp_is_accepted(self) -> None:
        service = AttitudeService()
        service.REFERENCE_CAPTURE_SECONDS = 0.0
        service.connect()

        self.assertTrue(service.ingest_sample(self.sample(roll=1.0, pitch=2.0, millis=1000, seq=7)))
        self.assertTrue(service.ingest_sample(self.sample(roll=3.0, pitch=4.0, millis=1000, seq=8)))
        self.assertTrue(service.ingest_sample(self.sample(roll=5.0, pitch=6.0, millis=1020, seq=8)))

        absolute = service.latest_absolute_attitude()
        self.assertIsNotNone(absolute)
        self.assertEqual(5.0, absolute.roll_deg)
        self.assertEqual(1020, absolute.movement_millis)
        self.assertEqual(8, absolute.movement_seq)

    def test_worker_reads_movement_sequence_and_millis(self) -> None:
        worker = SimpleNamespace(ser=object())
        regs = [
            2,        # movement_status
            42,       # movement_seq
            0x5678,   # movement_millis low
            0x1234,   # movement_millis high
            0xFFFE,   # roll -2
            3,        # pitch +3
        ]

        with patch.object(worker_tasks, "read_regs", return_value=regs):
            sample = worker_tasks.read_movement_attitude(worker)

        self.assertIsNotNone(sample)
        self.assertEqual(42, sample.movement_seq)
        self.assertEqual(0x12345678, sample.movement_millis)
        self.assertEqual(-2.0, sample.roll_deg)
        self.assertEqual(3.0, sample.pitch_deg)

    def test_arduino_elapsed_requires_millis(self) -> None:
        self.assertEqual(0.02, AutoSessionHelpers.arduino_elapsed_s(1000, 1020))
        with self.assertRaisesRegex(ValueError, "Arduino movement_millis"):
            AutoSessionHelpers.arduino_elapsed_s(None, 1020)  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()

import unittest
from dataclasses import replace

from modbus_app.workflows.fly_log_pid_isolation_workflow import (
    prepare_fly_log_pid_isolation,
    restore_fly_log_pid_isolation,
)
from serialUSB.inav_serial_service import AxisPidFf, FF_SETTING_NAME, PID_SETTING_NAME


SETTING_TO_AXIS_GAIN = {
    setting_name: (axis, gain)
    for (axis, gain), setting_name in PID_SETTING_NAME.items()
}
SETTING_TO_AXIS_GAIN.update(
    {
        setting_name: (axis, "ff")
        for axis, setting_name in FF_SETTING_NAME.items()
    }
)


class FakeFcService:
    def __init__(self) -> None:
        self.values = {
            "roll": AxisPidFf(p=44, i=63, d=18, ff=91),
            "pitch": AxisPidFf(p=48, i=67, d=21, ff=95),
            "yaw": AxisPidFf(p=35, i=45, d=0, ff=70),
        }
        self.save_count = 0

    def read_roll_pitch_pid_ff(self, timeout_seconds: float = 1.0) -> tuple[AxisPidFf, AxisPidFf]:
        return self.values["roll"], self.values["pitch"]

    def get_setting_int(self, setting_name: str, timeout_seconds: float = 1.0) -> int:
        axis, gain = SETTING_TO_AXIS_GAIN[setting_name]
        return int(getattr(self.values[axis], gain))

    def set_setting_int(self, setting_name: str, value: int, timeout_seconds: float = 1.0) -> int:
        axis, gain = SETTING_TO_AXIS_GAIN[setting_name]
        self.values[axis] = replace(self.values[axis], **{gain: int(value)})
        return int(value)

    def save_settings(self, timeout_seconds: float = 1.0) -> None:
        self.save_count += 1


class FlyLogPidIsolationWorkflowTests(unittest.TestCase):
    def test_roll_test_zeroes_pitch_and_yaw_pid_ff(self) -> None:
        fc = FakeFcService()
        original_roll = fc.values["roll"]

        result = prepare_fly_log_pid_isolation(fc, "roll")

        self.assertEqual("roll", result.snapshot.test_axis)
        self.assertEqual(("pitch", "yaw"), result.snapshot.isolated_axes)
        self.assertEqual(original_roll, fc.values["roll"])
        self.assertEqual(AxisPidFf(p=0, i=0, d=0, ff=0), fc.values["pitch"])
        self.assertEqual(AxisPidFf(p=0, i=0, d=0, ff=0), fc.values["yaw"])
        self.assertEqual(1, fc.save_count)

    def test_pitch_test_zeroes_roll_and_yaw_pid_ff(self) -> None:
        fc = FakeFcService()
        original_pitch = fc.values["pitch"]

        result = prepare_fly_log_pid_isolation(fc, "pitch")

        self.assertEqual("pitch", result.snapshot.test_axis)
        self.assertEqual(("roll", "yaw"), result.snapshot.isolated_axes)
        self.assertEqual(AxisPidFf(p=0, i=0, d=0, ff=0), fc.values["roll"])
        self.assertEqual(original_pitch, fc.values["pitch"])
        self.assertEqual(AxisPidFf(p=0, i=0, d=0, ff=0), fc.values["yaw"])
        self.assertEqual(1, fc.save_count)

    def test_restore_puts_back_roll_pitch_and_yaw_pid_ff(self) -> None:
        fc = FakeFcService()
        original_values = dict(fc.values)
        prepared = prepare_fly_log_pid_isolation(fc, "roll")

        restored = restore_fly_log_pid_isolation(fc, prepared.snapshot)

        self.assertEqual(original_values, fc.values)
        self.assertEqual(original_values["roll"], restored.roll_values)
        self.assertEqual(original_values["pitch"], restored.pitch_values)
        self.assertEqual(2, fc.save_count)


if __name__ == "__main__":
    unittest.main()

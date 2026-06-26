import unittest

from modbus_app.workflows.fc_pid_ff_workflow import FcPidFfWorkflow
from serialUSB.inav_serial_service import AxisPidFf


class FakeVar:
    def __init__(self) -> None:
        self.value = ""

    def get(self) -> str:
        return self.value

    def set(self, value: str) -> None:
        self.value = value


class FakeApp:
    def __init__(self) -> None:
        self.pid_ff_labels = ("P", "I", "D", "FF")
        self.roll_pidff_vars = [FakeVar() for _ in self.pid_ff_labels]
        self.pitch_pidff_vars = [FakeVar() for _ in self.pid_ff_labels]


class FcPidFfWorkflowTests(unittest.TestCase):
    def test_ff_zero_can_be_staged_and_parsed(self) -> None:
        app = FakeApp()
        workflow = FcPidFfWorkflow(
            app=app,
            set_error=lambda _title, _exc: None,
            ensure_disarmed_before_pid_write=lambda: True,
            format_pid_values=lambda _values: "",
        )

        workflow.set_var("roll", "ff", 0)

        self.assertEqual("FF: 0", app.roll_pidff_vars[3].get())
        self.assertEqual(0, workflow.parse_var("roll", "ff"))

    def test_display_loaded_zero_pid_values_without_clamping(self) -> None:
        app = FakeApp()
        workflow = FcPidFfWorkflow(
            app=app,
            set_error=lambda _title, _exc: None,
            ensure_disarmed_before_pid_write=lambda: True,
            format_pid_values=lambda _values: "",
        )

        workflow.set_displays(
            AxisPidFf(p=0, i=0, d=0, ff=0),
            AxisPidFf(p=48, i=67, d=21, ff=95),
        )

        self.assertEqual(["P: 0", "I: 0", "D: 0", "FF: 0"], [var.get() for var in app.roll_pidff_vars])


if __name__ == "__main__":
    unittest.main()

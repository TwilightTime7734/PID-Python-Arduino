"""Hardware controller-backed state properties."""

from __future__ import annotations

import serial


class HardwareStateMixin:
    """Delegates serial runtime properties to ``self.controller``."""

    @property
    def run_active(self) -> bool:
        return self.controller.run_active

    @run_active.setter
    def run_active(self, value: bool) -> None:
        self.controller.run_active = value

    @property
    def run_port(self) -> str:
        return self.controller.run_port

    @run_port.setter
    def run_port(self, value: str) -> None:
        self.controller.run_port = value

    @property
    def run_ser(self) -> serial.Serial | None:
        return self.controller.run_ser

    @run_ser.setter
    def run_ser(self, value: serial.Serial | None) -> None:
        self.controller.run_ser = value

    @property
    def run_quant(self) -> int | None:
        return self.controller.run_quant

    @run_quant.setter
    def run_quant(self, value: int | None) -> None:
        self.controller.run_quant = value

    @property
    def run_max_count(self) -> int | None:
        return self.controller.run_max_count

    @run_max_count.setter
    def run_max_count(self, value: int | None) -> None:
        self.controller.run_max_count = value


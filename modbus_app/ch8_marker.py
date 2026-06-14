"""Helpers for the PID-test CH8 log marker."""

from __future__ import annotations

from .constants import (
    PID_TEST_CH8_CHANNEL_INDEX,
    PID_TEST_CH8_OFF_US,
    PID_TEST_CH8_ON_US,
)


def channels_with_pid_test_ch8(channels: list[int], *, active: bool) -> list[int]:
    output = channels.copy()
    while len(output) <= PID_TEST_CH8_CHANNEL_INDEX:
        output.append(PID_TEST_CH8_OFF_US)
    output[PID_TEST_CH8_CHANNEL_INDEX] = PID_TEST_CH8_ON_US if active else PID_TEST_CH8_OFF_US
    return output

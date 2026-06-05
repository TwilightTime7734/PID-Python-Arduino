import unittest

from serialUSB.inav_serial_service import _parse_armed_status_payload


class InavSerialServiceTests(unittest.TestCase):
    def test_parse_armed_status_payload_detects_arm_mode_flag(self) -> None:
        payload = bytearray(11)
        payload[6:10] = (1).to_bytes(4, byteorder="little")

        self.assertTrue(_parse_armed_status_payload(bytes(payload)))

    def test_parse_armed_status_payload_detects_disarmed(self) -> None:
        payload = bytearray(11)
        payload[6:10] = (0).to_bytes(4, byteorder="little")

        self.assertFalse(_parse_armed_status_payload(bytes(payload)))

    def test_parse_armed_status_payload_rejects_short_payload(self) -> None:
        with self.assertRaises(RuntimeError):
            _parse_armed_status_payload(b"\x00" * 9)


if __name__ == "__main__":
    unittest.main()

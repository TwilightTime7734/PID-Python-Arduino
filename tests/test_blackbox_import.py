import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from modbus_app import blackbox_import


class BlackboxImportTests(unittest.TestCase):
    def test_local_inav_decoder_is_preferred(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            local_inav = temp_path / "blackbox_decode_INAV.exe"
            path_decoder = temp_path / "blackbox_decode.exe"
            fallback_inav = temp_path / "fallback_blackbox_decode_INAV.exe"
            local_inav.write_text("", encoding="utf-8")
            path_decoder.write_text("", encoding="utf-8")
            fallback_inav.write_text("", encoding="utf-8")

            with (
                patch.object(blackbox_import, "LOCAL_DECODER_CANDIDATES", (local_inav,)),
                patch.object(blackbox_import, "DECODER_FALLBACK_CANDIDATES", (fallback_inav,)),
                patch.object(blackbox_import.shutil, "which", return_value=str(path_decoder)),
            ):
                self.assertEqual(local_inav, blackbox_import._find_blackbox_decoder())


if __name__ == "__main__":
    unittest.main()

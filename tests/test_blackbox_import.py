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

    def test_msc_import_clears_local_logs_and_moves_new_logs(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            source_root = temp_path / "msc"
            destination = temp_path / "blackbox_imports"
            report_dir = destination / "reports"
            source_root.mkdir()
            report_dir.mkdir(parents=True)

            old_raw = destination / "OLD00001.bbl"
            old_csv = destination / "OLD00001.csv"
            keep_note = destination / "notes.md"
            keep_report_csv = report_dir / "previous_report.csv"
            new_log = source_root / "LOG00001.bbl"
            old_raw.write_text("old raw", encoding="utf-8")
            old_csv.write_text("old csv", encoding="utf-8")
            keep_note.write_text("keep", encoding="utf-8")
            keep_report_csv.write_text("keep report", encoding="utf-8")
            new_log.write_text("new raw", encoding="utf-8")

            with (
                patch.object(blackbox_import, "_candidate_msc_roots", return_value=[source_root]),
                patch.object(blackbox_import, "_summarize_with_inav_toolkit", return_value=("", "", [], None)),
                patch.object(blackbox_import, "_decode_raw_logs", return_value=([], [])),
            ):
                result = blackbox_import.import_blackbox_logs_from_msc(destination)

            moved_log = destination / "LOG00001.bbl"
            self.assertFalse(old_raw.exists())
            self.assertFalse(old_csv.exists())
            self.assertTrue(keep_note.exists())
            self.assertTrue(keep_report_csv.exists())
            self.assertFalse(new_log.exists())
            self.assertTrue(moved_log.exists())
            self.assertEqual("new raw", moved_log.read_text(encoding="utf-8"))
            self.assertEqual((str(moved_log),), tuple(item.local_path for item in result.imported_files))


if __name__ == "__main__":
    unittest.main()

import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from replay_data import ReplayData


class ReplayDataTests(unittest.TestCase):
    def test_load_csv_and_read_latest_row(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "sample.csv"
            with path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(
                    [
                        "frame_index", "time_s",
                        "t_raw_a", "t_raw_b",
                        "m_raw_a", "m_raw_b",
                        "final_a", "final_b",
                        "target_a", "target_b",
                        "output_a", "output_b",
                    ]
                )
                writer.writerow([0, 0.0, 0, 0, 0, 0, 1.0, 2.0, 1.5, 2.5, 100, 120])
                writer.writerow([1, 0.1, 0, 0, 0, 0, 1.1, 2.1, 1.5, 2.5, 101, 121])

            replay = ReplayData.load(path)

            self.assertEqual(replay.row_count, 2)
            latest = replay.latest_frame_at_time(0.1)
            self.assertEqual(latest["final_A"], 1.1)
            self.assertEqual(latest["output_B"], 121)
            self.assertEqual(latest["afc_output_A"], 0.0)

    def test_load_csv_with_afc_columns_reads_afc_values(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "sample_with_afc.csv"
            with path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(
                    [
                        "frame_index", "time_s",
                        "t_raw_a", "t_raw_b",
                        "m_raw_a", "m_raw_b",
                        "final_a", "final_b",
                        "target_a", "target_b",
                        "output_a", "output_b",
                        "afc_output_a", "afc_output_b",
                    ]
                )
                writer.writerow([0, 0.0, 0, 0, 0, 0, 1.0, 2.0, 1.5, 2.5, 100, 120, 12.5, 13.5])

            replay = ReplayData.load(path)

            latest = replay.latest_frame_at_time(0.0)
            self.assertEqual(latest["afc_output_A"], 12.5)
            self.assertEqual(latest["afc_output_B"], 13.5)

    def test_missing_columns_raise_value_error(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.csv"
            with path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(["frame_index", "time_s"])

            with self.assertRaises(ValueError):
                ReplayData.load(path)

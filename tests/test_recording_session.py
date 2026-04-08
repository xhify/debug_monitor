import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from protocol import DataFrame
from recording_session import RecordingSession


def make_frame() -> DataFrame:
    return DataFrame(
        t_raw_A=1.0,
        t_raw_B=2.0,
        m_raw_A=3.0,
        m_raw_B=4.0,
        final_A=5.0,
        final_B=6.0,
        target_A=7.0,
        target_B=8.0,
        output_A=9,
        output_B=10,
        afc_output_A=11.5,
        afc_output_B=12.5,
    )


class RecordingSessionTests(unittest.TestCase):
    def test_start_write_finalize_moves_temp_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            session = RecordingSession(base_dir=Path(tmp))
            temp_path = session.start()
            session.write_frame(frame_index=0, time_s=0.0, frame=make_frame())
            final_path = Path(tmp) / "saved.csv"

            session.finalize(final_path)

            self.assertFalse(temp_path.exists())
            self.assertTrue(final_path.exists())
            with final_path.open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.reader(fh))
            self.assertEqual(
                rows[0],
                [
                    "frame_index", "time_s",
                    "t_raw_a", "t_raw_b",
                    "m_raw_a", "m_raw_b",
                    "final_a", "final_b",
                    "target_a", "target_b",
                    "output_a", "output_b",
                    "afc_output_a", "afc_output_b",
                ],
            )
            self.assertEqual(rows[1][0], "0")

    def test_cancel_deletes_temp_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            session = RecordingSession(base_dir=Path(tmp))
            temp_path = session.start()
            session.write_frame(frame_index=0, time_s=0.0, frame=make_frame())

            session.cancel()

            self.assertFalse(temp_path.exists())

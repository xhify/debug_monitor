import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from data_buffer import COL_FINAL_A, DataBuffer
from protocol import DataFrame
from recording_session import RecordingSession


def make_frame(index: int) -> DataFrame:
    return DataFrame(
        t_raw_A=0.0,
        t_raw_B=0.0,
        m_raw_A=0.0,
        m_raw_B=0.0,
        final_A=float(index),
        final_B=float(index) + 1.0,
        target_A=10.0,
        target_B=11.0,
        output_A=index,
        output_B=index + 1,
    )


class DataBufferTests(unittest.TestCase):
    def test_recent_window_returns_last_10_seconds(self) -> None:
        buffer = DataBuffer()
        for index in range(1500):
            buffer.append(make_frame(index))

        time_s, data = buffer.get_recent_window(10.0)

        self.assertEqual(len(time_s), 1000)
        self.assertAlmostEqual(time_s[0], 5.0, places=2)
        self.assertAlmostEqual(data[-1, COL_FINAL_A], 1499.0, places=2)

    def test_append_writes_to_active_recording_session(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            session = RecordingSession(base_dir=Path(tmp))
            session.start()
            buffer = DataBuffer()
            buffer.start_recording(session)

            buffer.append(make_frame(1))

            self.assertEqual(buffer.csv_rows_written, 1)
            stopped = buffer.stop_recording()
            self.assertIs(stopped, session)
            stopped.cancel()

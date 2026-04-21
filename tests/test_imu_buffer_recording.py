import csv
import json
import shutil
import os
import sys
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from imu_buffer import COL_ACC_X, COL_EULER_YAW, ImuBuffer  # noqa: E402
from imu_protocol import CSV_FIELDS, ImuSample  # noqa: E402
from imu_recording import ImuRecordingSession, ImuSessionRecorder, build_aligned_rows  # noqa: E402


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"imu_{uuid.uuid4().hex}"
    path.mkdir()
    try:
        yield str(path)
    finally:
        shutil.rmtree(path, ignore_errors=True)


def make_sample(
    sequence: int = 1,
    host_time: float = 1.0,
    accel: tuple[float, float, float] | None = (1.0, 2.0, 3.0),
    gyro: tuple[float, float, float] | None = (4.0, 5.0, 6.0),
    euler: tuple[float, float, float] | None = (7.0, 8.0, 9.0),
    device_time: int | None = 100,
    sync_time: int | None = 200,
) -> ImuSample:
    return ImuSample(
        host_time=host_time,
        sequence=sequence,
        accel=accel,
        gyro=gyro,
        euler=euler,
        temperature_c=25.0,
        device_time=device_time,
        sync_time=sync_time,
    )


class ImuBufferTests(unittest.TestCase):
    def test_append_snapshot_latest_and_clear(self) -> None:
        buffer = ImuBuffer(capacity=3)

        buffer.append(make_sample(sequence=1, host_time=10.0))
        buffer.append(make_sample(sequence=2, host_time=10.1, accel=(11.0, 12.0, 13.0)))

        time_arr, data = buffer.get_snapshot()
        latest = buffer.get_latest()

        self.assertEqual(time_arr.tolist(), [10.0, 10.1])
        self.assertEqual(data.shape, (2, 9))
        self.assertEqual(data[0, COL_ACC_X], 1.0)
        self.assertEqual(data[1, COL_ACC_X], 11.0)
        self.assertEqual(data[1, COL_EULER_YAW], 9.0)
        self.assertEqual(latest.sequence, 2)
        self.assertEqual(buffer.frame_index, 2)

        buffer.clear()

        time_arr, data = buffer.get_snapshot()
        self.assertEqual(time_arr.tolist(), [])
        self.assertEqual(data.shape, (0, 9))
        self.assertIsNone(buffer.get_latest())
        self.assertEqual(buffer.frame_index, 0)

    def test_snapshot_wraps_in_chronological_order(self) -> None:
        buffer = ImuBuffer(capacity=2)

        buffer.append(make_sample(sequence=1, host_time=1.0, accel=(1.0, 0.0, 0.0)))
        buffer.append(make_sample(sequence=2, host_time=2.0, accel=(2.0, 0.0, 0.0)))
        buffer.append(make_sample(sequence=3, host_time=3.0, accel=(3.0, 0.0, 0.0)))

        time_arr, data = buffer.get_snapshot()

        self.assertEqual(time_arr.tolist(), [2.0, 3.0])
        self.assertEqual(data[:, COL_ACC_X].tolist(), [2.0, 3.0])

    def test_recording_session_receives_appended_samples(self) -> None:
        with temp_dir() as tmp:
            buffer = ImuBuffer(capacity=3)
            session = ImuRecordingSession(base_dir=Path(tmp))
            session.start()
            buffer.start_recording(session)

            buffer.append(make_sample(sequence=1, host_time=1.0))

            self.assertTrue(buffer.recording)
            self.assertEqual(buffer.csv_rows_written, 1)

            stopped = buffer.stop_recording()
            self.assertIs(stopped, session)
            self.assertFalse(buffer.recording)
            session.cancel()


class ImuRecordingSessionTests(unittest.TestCase):
    def test_start_write_finalize_moves_temp_file(self) -> None:
        with temp_dir() as tmp:
            session = ImuRecordingSession(base_dir=Path(tmp))
            temp_path = session.start()
            session.write_sample(make_sample(sequence=9, host_time=1.25))
            final_path = Path(tmp) / "imu_saved.csv"

            session.finalize(final_path)

            self.assertFalse(temp_path.exists())
            self.assertTrue(final_path.exists())
            with final_path.open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.reader(fh))
            self.assertEqual(rows[0], CSV_FIELDS)
            self.assertEqual(rows[1][0], "1.250000")
            self.assertEqual(rows[1][1], "9")

    def test_cancel_deletes_temp_file(self) -> None:
        with temp_dir() as tmp:
            session = ImuRecordingSession(base_dir=Path(tmp))
            temp_path = session.start()
            session.write_sample(make_sample())

            session.cancel()

            self.assertFalse(temp_path.exists())


class ImuSessionRecorderTests(unittest.TestCase):
    def test_session_recorder_writes_two_raw_csvs_metadata_and_alignment(self) -> None:
        with temp_dir() as tmp:
            recorder = ImuSessionRecorder(base_dir=Path(tmp))
            session_dir = recorder.start(
                {
                    "A": {"port": "COM4", "baudrate": 460800},
                    "B": {"port": "COM5", "baudrate": 460800},
                },
                timestamp="20260420_150000",
            )

            recorder.write_sample("A", make_sample(sequence=1, host_time=10.00, device_time=1000, sync_time=5000))
            recorder.write_sample("B", make_sample(sequence=2, host_time=10.01, device_time=1100, sync_time=5100))
            recorder.finalize()

            self.assertEqual(session_dir.name, "imu_session_20260420_150000")
            self.assertTrue((session_dir / "imu_A.csv").exists())
            self.assertTrue((session_dir / "imu_B.csv").exists())
            self.assertTrue((session_dir / "session.json").exists())
            self.assertTrue((session_dir / "merged_aligned.csv").exists())

            with (session_dir / "imu_A.csv").open("r", encoding="utf-8", newline="") as fh:
                rows_a = list(csv.reader(fh))
            self.assertEqual(rows_a[0], CSV_FIELDS)
            self.assertEqual(rows_a[1][1], "1")

            with (session_dir / "merged_aligned.csv").open("r", encoding="utf-8", newline="") as fh:
                merged = list(csv.DictReader(fh))
            self.assertEqual(len(merged), 1)
            self.assertEqual(merged[0]["A_sequence"], "1")
            self.assertEqual(merged[0]["B_sequence"], "2")

    def test_session_recorder_can_write_into_existing_summary_directory(self) -> None:
        with temp_dir() as tmp:
            session_dir = Path(tmp) / "session_20260420_160000"
            session_dir.mkdir()
            recorder = ImuSessionRecorder(base_dir=Path(tmp))

            recorder.start_in_directory(
                session_dir,
                {"A": {"port": "COM4"}, "B": {"port": "COM5"}},
                started_at="20260420_160000",
                metadata_filename="imu_session.json",
                merged_filename="imu_merged_aligned.csv",
                note="直线加速测试",
            )
            recorder.write_sample("A", make_sample(sequence=1, host_time=1.0, sync_time=1000))
            recorder.write_sample("B", make_sample(sequence=2, host_time=1.001, sync_time=1100))
            recorder.finalize()

            self.assertTrue((session_dir / "imu_A.csv").exists())
            self.assertTrue((session_dir / "imu_B.csv").exists())
            self.assertTrue((session_dir / "imu_session.json").exists())
            self.assertTrue((session_dir / "imu_merged_aligned.csv").exists())
            self.assertFalse((session_dir / "session.json").exists())
            self.assertFalse((session_dir / "merged_aligned.csv").exists())
            with (session_dir / "imu_session.json").open("r", encoding="utf-8") as fh:
                metadata = json.load(fh)
            self.assertEqual(metadata["note"], "直线加速测试")

    def test_session_recorder_cancel_removes_session_directory(self) -> None:
        with temp_dir() as tmp:
            recorder = ImuSessionRecorder(base_dir=Path(tmp))
            session_dir = recorder.start({"A": {}, "B": {}}, timestamp="20260420_150001")
            recorder.write_sample("A", make_sample(sequence=1))

            recorder.cancel()

            self.assertFalse(session_dir.exists())

    def test_build_aligned_rows_pairs_samples_with_nearest_sync_time(self) -> None:
        rows = build_aligned_rows(
            [
                make_sample(sequence=1, host_time=0.001, sync_time=1000),
                make_sample(sequence=2, host_time=0.003, sync_time=3000),
            ],
            [
                make_sample(sequence=10, host_time=0.00105, sync_time=1050),
                make_sample(sequence=11, host_time=0.009, sync_time=9000),
            ],
            align_window_seconds=0.001,
        )

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["A_sequence"], "1")
        self.assertEqual(rows[0]["B_sequence"], "10")
        self.assertEqual(rows[0]["time_delta_ms"], "0.050")


if __name__ == "__main__":
    unittest.main()

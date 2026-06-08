import csv
import os
import shutil
import sys
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from recording_clock import RecordingClock
from ros_topic_recorders import (
    ODOMETRY_FIELDNAMES,
    POWER_VOLTAGE_FIELDNAMES,
    extract_header_stamp,
    make_odometry_recorder,
    make_power_voltage_recorder,
    make_ros_imu_recorder,
    RosTopicMonitor,
)


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"ros_topic_recorders_{uuid.uuid4().hex}"
    path.mkdir()
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


class HeaderStampParsingTests(unittest.TestCase):
    def test_extract_header_stamp_supports_secs_nsecs(self) -> None:
        value = extract_header_stamp({"header": {"stamp": {"secs": 123, "nsecs": 456_000_000}}})

        self.assertAlmostEqual(value, 123.456)

    def test_extract_header_stamp_supports_sec_nanosec(self) -> None:
        value = extract_header_stamp({"header": {"stamp": {"sec": 12, "nanosec": 250_000_000}}})

        self.assertAlmostEqual(value, 12.25)

    def test_extract_header_stamp_tolerates_missing_header(self) -> None:
        self.assertEqual(extract_header_stamp({}), 0.0)


class RosTopicMonitorTests(unittest.TestCase):
    def test_monitor_estimates_frequency_and_header_presence(self) -> None:
        monitor = RosTopicMonitor(
            topic="/wheeltec/akm_state",
            expected_type="turn_on_wheeltec_robot/AkmState",
            required_fields=("header.frame_id", "seq_id"),
            warning_hz_below=80.0,
        )
        monitor.observe(
            {
                "header": {"stamp": {"secs": 1, "nsecs": 0}, "frame_id": "base_link"},
                "seq_id": 1,
            },
            message_type="turn_on_wheeltec_robot/AkmState",
            recv_time_epoch_s=10.0,
        )
        monitor.observe(
            {
                "header": {"stamp": {"secs": 1, "nsecs": 10_000_000}, "frame_id": "base_link"},
                "seq_id": 2,
            },
            message_type="turn_on_wheeltec_robot/AkmState",
            recv_time_epoch_s=10.01,
        )
        result = monitor.result()

        self.assertEqual(result.messages_received, 2)
        self.assertAlmostEqual(result.estimated_hz, 100.0, places=3)
        self.assertTrue(result.has_header_stamp)
        self.assertEqual(result.status, "ok")


class RosTopicRecorderTests(unittest.TestCase):
    def test_odometry_recorder_writes_expected_fields(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "fastlio_odometry.csv"
            recorder = make_odometry_recorder(path, RecordingClock(session_id="session_1"))
            recorder.write_message(
                {
                    "header": {"stamp": {"secs": 100, "nsecs": 500_000_000}, "frame_id": "odom"},
                    "child_frame_id": "base_link",
                    "pose": {
                        "pose": {
                            "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.1, "w": 0.99},
                        }
                    },
                    "twist": {
                        "twist": {
                            "linear": {"x": 0.4, "y": 0.5, "z": 0.6},
                            "angular": {"x": 0.1, "y": 0.2, "z": 0.3},
                        }
                    },
                },
                recv_time_epoch_s=200.0,
            )
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual(rows[0]["session_id"], "session_1")
            self.assertEqual(rows[0]["ros_time"], "100.5")
            self.assertEqual(rows[0]["frame_id"], "odom")
            self.assertEqual(rows[0]["child_frame_id"], "base_link")
            self.assertEqual(rows[0]["position_x"], "1.0")
            self.assertEqual(rows[0]["linear_z"], "0.6")
            self.assertEqual(recorder.fieldnames[: len(ODOMETRY_FIELDNAMES)], ODOMETRY_FIELDNAMES)

    def test_imu_recorder_appends_session_fields_after_existing_columns(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "ros_imu.csv"
            recorder = make_ros_imu_recorder(path, RecordingClock(session_id="session_2"))
            recorder.write_message(
                {
                    "header": {"stamp": {"secs": 10, "nsecs": 250_000_000}, "frame_id": "imu_link"},
                    "linear_acceleration": {"x": 1.0, "y": 2.0, "z": 3.0},
                    "angular_velocity": {"x": 4.0, "y": 5.0, "z": 6.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                recv_time_epoch_s=20.0,
            )
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual(rows[0]["frame_id"], "imu_link")
            self.assertEqual(rows[0]["ros_time"], "10.25")
            self.assertEqual(rows[0]["session_id"], "session_2")
            self.assertIn("session_elapsed_s", rows[0])

    def test_power_voltage_recorder_keeps_time_and_appends_session_fields(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "ros_power_voltage.csv"
            recorder = make_power_voltage_recorder(path, RecordingClock(session_id="session_3"))
            recorder.write_message({"data": 24.5}, recv_time_epoch_s=30.0)
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual(rows[0]["voltage"], "24.5")
            self.assertEqual(rows[0]["session_id"], "session_3")
            self.assertEqual(list(rows[0].keys()), POWER_VOLTAGE_FIELDNAMES)


if __name__ == "__main__":
    unittest.main()

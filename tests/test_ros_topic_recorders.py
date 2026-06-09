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
    make_ros_odom_compat_recorder,
    make_ros_imu_compat_recorder,
    make_odometry_recorder,
    make_power_voltage_recorder,
    make_ros_imu_recorder,
    RosTopicMonitor,
    sample_ros_topic,
    sample_ros_topics,
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

    def test_sample_ros_topic_uses_real_subscription_and_unsubscribes(self) -> None:
        class FakeRos:
            def __init__(self, host: str, port: int) -> None:
                self.host = host
                self.port = port
                self.closed = False

            def run(self) -> None:
                pass

            def close(self) -> None:
                self.closed = True

        class FakeTopic:
            instances: list["FakeTopic"] = []

            def __init__(self, _ros, name: str, message_type: str) -> None:
                self.name = name
                self.message_type = message_type
                self.unsubscribed = False
                FakeTopic.instances.append(self)

            def subscribe(self, callback) -> None:
                callback({"header": {"stamp": {"secs": 1, "nsecs": 0}, "frame_id": "odom"}})
                callback({"header": {"stamp": {"secs": 1, "nsecs": 100_000_000}, "frame_id": "odom"}})

            def unsubscribe(self) -> None:
                self.unsubscribed = True

        result = sample_ros_topic(
            host="robot.local",
            port=9090,
            topic="/odom",
            expected_type="nav_msgs/Odometry",
            sample_seconds=0.0,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
            time_fn=(value for value in [10.0, 10.1]).__next__,
            sleep_fn=lambda _seconds: None,
        )

        self.assertEqual(result.messages_received, 2)
        self.assertAlmostEqual(result.estimated_hz, 10.0)
        self.assertTrue(result.has_header_stamp)
        self.assertTrue(FakeTopic.instances[0].unsubscribed)

    def test_sample_ros_topic_reports_offline_when_no_messages_arrive(self) -> None:
        class FakeRos:
            def __init__(self, _host: str, _port: int) -> None:
                pass

            def run(self) -> None:
                pass

            def close(self) -> None:
                pass

        class FakeTopic:
            def __init__(self, *_args) -> None:
                pass

            def subscribe(self, _callback) -> None:
                pass

            def unsubscribe(self) -> None:
                pass

        result = sample_ros_topic(
            host="robot.local",
            port=9090,
            topic="/missing",
            expected_type="std_msgs/String",
            sample_seconds=0.0,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
            sleep_fn=lambda _seconds: None,
        )

        self.assertEqual(result.status, "offline")
        self.assertEqual(result.messages_received, 0)

    def test_sample_ros_topics_subscribes_all_topics_in_one_sampling_window(self) -> None:
        class FakeRos:
            instances: list["FakeRos"] = []

            def __init__(self, host: str, port: int) -> None:
                self.host = host
                self.port = port
                self.closed = False
                self.run_timeout = None
                FakeRos.instances.append(self)

            def run(self, timeout=None) -> None:
                self.run_timeout = timeout

            def close(self) -> None:
                self.closed = True

        class FakeTopic:
            instances: list["FakeTopic"] = []

            def __init__(self, _ros, name: str, message_type: str) -> None:
                self.name = name
                self.message_type = message_type
                self.unsubscribed = False
                FakeTopic.instances.append(self)

            def subscribe(self, callback) -> None:
                callback({"header": {"stamp": {"secs": 1, "nsecs": 0}, "frame_id": self.name}})

            def unsubscribe(self) -> None:
                self.unsubscribed = True

        sleeps: list[float] = []
        results = sample_ros_topics(
            host="robot.local",
            port=9090,
            topic_specs=[
                {"source_id": "odom", "topic": "/odom", "expected_type": "nav_msgs/Odometry"},
                {"source_id": "imu", "topic": "/imu", "expected_type": "sensor_msgs/Imu"},
            ],
            sample_seconds=2.0,
            ros_factory=FakeRos,
            topic_factory=FakeTopic,
            time_fn=(value for value in [10.0, 10.1]).__next__,
            sleep_fn=lambda seconds: sleeps.append(seconds),
            connection_timeout=3.0,
        )

        self.assertEqual(len(FakeRos.instances), 1)
        self.assertEqual(FakeRos.instances[0].run_timeout, 3.0)
        self.assertEqual(sleeps, [2.0])
        self.assertEqual({topic.name for topic in FakeTopic.instances}, {"/odom", "/imu"})
        self.assertTrue(all(topic.unsubscribed for topic in FakeTopic.instances))
        self.assertEqual(results["odom"].messages_received, 1)
        self.assertEqual(results["imu"].messages_received, 1)


class RosTopicRecorderTests(unittest.TestCase):
    def test_recorder_elapsed_uses_recv_time_epoch_when_provided(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "ros_power_voltage.csv"
            clock = RecordingClock(
                session_id="session_recv",
                start_epoch_s=1000.0,
                start_perf_s=1.0,
            )
            recorder = make_power_voltage_recorder(path, clock)
            recorder.write_message({"data": 24.5}, recv_time_epoch_s=1002.5)
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual(rows[0]["recv_time_epoch_s"], "1002.5")
            self.assertEqual(rows[0]["session_elapsed_s"], "2.5")

    def test_ros_odom_compat_recorder_fills_frame_count(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "ros_odom.csv"
            recorder = make_ros_odom_compat_recorder(path, RecordingClock(session_id="session_odom"))
            message = {
                "header": {"stamp": {"secs": 10, "nsecs": 0}, "frame_id": "odom"},
                "pose": {"pose": {"position": {"x": 1.0}, "orientation": {"w": 1.0}}},
                "twist": {"twist": {"linear": {"x": 0.2}, "angular": {"z": 0.3}}},
            }
            recorder.write_message(message, recv_time_epoch_s=20.0)
            recorder.write_message(message, recv_time_epoch_s=20.1)
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual([row["frame_count"] for row in rows], ["1", "2"])

    def test_ros_imu_compat_recorder_fills_euler_angles(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "ros_imu.csv"
            recorder = make_ros_imu_compat_recorder(path, RecordingClock(session_id="session_imu"))
            recorder.write_message(
                {
                    "header": {"stamp": {"secs": 11, "nsecs": 0}, "frame_id": "imu"},
                    "linear_acceleration": {"x": 1.0},
                    "angular_velocity": {"z": 2.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.7071068, "w": 0.7071068},
                },
                recv_time_epoch_s=21.0,
            )
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))

            self.assertNotEqual(rows[0]["roll_deg"], "")
            self.assertNotEqual(rows[0]["pitch_deg"], "")
            self.assertAlmostEqual(float(rows[0]["yaw_deg"]), 90.0, places=2)

    def test_recorder_flushes_by_row_interval(self) -> None:
        flushes: list[int] = []

        class CountingHandle:
            def __init__(self) -> None:
                self.closed = False

            def write(self, _text: str) -> int:
                return 0

            def flush(self) -> None:
                flushes.append(1)

            def close(self) -> None:
                self.closed = True

        with temp_dir() as tmp:
            path = tmp / "ros_power_voltage.csv"
            recorder = make_power_voltage_recorder(path, RecordingClock(session_id="session_flush"))
            handle = CountingHandle()
            recorder._handle.close()
            recorder._handle = handle
            recorder._writer = csv.DictWriter(handle, fieldnames=recorder.fieldnames)

            recorder.write_message({"data": 24.5}, recv_time_epoch_s=30.0)
            recorder.write_message({"data": 24.6}, recv_time_epoch_s=30.1)
            recorder.close()

            self.assertEqual(len(flushes), 1)

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

    def test_ros_odom_compat_recorder_keeps_old_columns_first(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "ros_odom.csv"
            recorder = make_ros_odom_compat_recorder(path, RecordingClock(session_id="session_4"))
            recorder.write_message(
                {
                    "header": {"stamp": {"secs": 10, "nsecs": 0}, "frame_id": "odom"},
                    "pose": {"pose": {"position": {"x": 1.0}, "orientation": {"w": 1.0}}},
                    "twist": {"twist": {"linear": {"x": 0.2}, "angular": {"z": 0.3}}},
                },
                recv_time_epoch_s=20.0,
            )
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                header = next(csv.reader(handle))

            self.assertEqual(header[0], "time_s")
            self.assertEqual(header[-3:], ["ros_time", "recv_time", "frame_id"])

    def test_ros_imu_compat_recorder_keeps_old_columns_first(self) -> None:
        with temp_dir() as tmp:
            path = tmp / "ros_imu.csv"
            recorder = make_ros_imu_compat_recorder(path, RecordingClock(session_id="session_5"))
            recorder.write_message(
                {
                    "header": {"stamp": {"secs": 11, "nsecs": 0}, "frame_id": "imu"},
                    "linear_acceleration": {"x": 1.0},
                    "angular_velocity": {"z": 2.0},
                    "orientation": {"w": 1.0},
                },
                recv_time_epoch_s=21.0,
            )
            recorder.close()

            with path.open("r", encoding="utf-8", newline="") as handle:
                header = next(csv.reader(handle))

            self.assertEqual(header[0], "time_s")
            self.assertEqual(header[-3:], ["ros_time", "recv_time", "frame_id"])


if __name__ == "__main__":
    unittest.main()

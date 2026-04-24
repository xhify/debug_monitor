import csv
import os
import shutil
import sys
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from ros_bridge_worker import RosImuReading, RosSnapshot
from ros_data import (
    ROS_CSV_HEADER,
    ROS_IMU_ALIGNED_HEADER,
    ROS_IMU_RAW_HEADER,
    ROS_SUMMARY_ODOM_HEADER,
    RosCsvRecordingSession,
    RosSummaryRecordingSession,
    RosTimeSeriesBuffer,
)


TEST_TMP_ROOT = Path(__file__).resolve().parents[1] / ".test_tmp"


@contextmanager
def temp_dir():
    TEST_TMP_ROOT.mkdir(exist_ok=True)
    path = TEST_TMP_ROOT / f"ros_data_{uuid.uuid4().hex}"
    path.mkdir()
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


def make_snapshot(index: int) -> RosSnapshot:
    return RosSnapshot(
        frame_count=index,
        linear_x=index * 0.1,
        linear_y=index * 0.2,
        angular_z=-index * 0.01,
        pose_x=index * 1.0,
        pose_y=index * 2.0,
        accel_z=9.8 + index,
        gyro_z=0.05 * index,
        voltage=24.0 + index,
    )


class RosTimeSeriesBufferTests(unittest.TestCase):
    def test_append_returns_chronological_plot_series(self) -> None:
        buffer = RosTimeSeriesBuffer(capacity=2)

        buffer.append(make_snapshot(1), timestamp=10.0)
        buffer.append(make_snapshot(2), timestamp=10.5)
        buffer.append(make_snapshot(3), timestamp=11.0)

        time_arr, data = buffer.snapshot()
        self.assertEqual(time_arr.tolist(), [0.5, 1.0])
        self.assertAlmostEqual(data["linear_x"][0], 0.2)
        self.assertAlmostEqual(data["linear_x"][1], 0.3)
        self.assertAlmostEqual(data["angular_z"][0], -0.02)
        self.assertAlmostEqual(data["angular_z"][1], -0.03)
        self.assertEqual(data["voltage"].tolist(), [26.0, 27.0])


class RosCsvRecordingSessionTests(unittest.TestCase):
    def test_start_write_finalize_moves_ros_csv(self) -> None:
        with temp_dir() as tmp:
            session = RosCsvRecordingSession(base_dir=tmp)
            temp_path = session.start()
            session.write_snapshot(time_s=0.25, snapshot=make_snapshot(4))
            final_path = tmp / "ros.csv"

            session.finalize(final_path)

            self.assertFalse(temp_path.exists())
            self.assertTrue(final_path.exists())
            with final_path.open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.reader(fh))
            self.assertEqual(rows[0], ROS_CSV_HEADER)
            self.assertEqual(rows[0][2:4], ["motor_a_left_speed", "motor_b_right_speed"])
            self.assertEqual(rows[1][0], "0.250")
            self.assertEqual(rows[1][1], "4")
            self.assertEqual(rows[1][2], "0.4")
            self.assertEqual(rows[1][-1], "28.0")

    def test_cancel_deletes_temp_file(self) -> None:
        with temp_dir() as tmp:
            session = RosCsvRecordingSession(base_dir=tmp)
            temp_path = session.start()
            session.write_snapshot(time_s=0.0, snapshot=make_snapshot(1))

            session.cancel()

            self.assertFalse(temp_path.exists())


class RosSummaryRecordingSessionTests(unittest.TestCase):
    def test_writes_odom_and_received_imu_streams_to_separate_summary_files(self) -> None:
        with temp_dir() as tmp:
            session = RosSummaryRecordingSession()
            with patch("ros_data.perf_counter", side_effect=[100.0, 100.001, 100.010, 100.015]):
                session.start_in_directory(tmp, started_at="20260423_101500")
                session.write_snapshot(
                    RosSnapshot(
                        frame_count=3,
                        linear_x=0.42,
                        linear_y=0.01,
                        angular_z=-0.13,
                        pose_x=1.0,
                        pose_y=2.0,
                        pose_z=0.0,
                        orientation_z=0.1,
                        orientation_w=0.99,
                        last_topic="/odom",
                    )
                )
                session.write_snapshot(
                    RosSnapshot(
                        frame_count=4,
                        last_topic="/imu",
                        imu=RosImuReading(
                            frame_count=1,
                            accel_x=1.1,
                            accel_y=1.2,
                            accel_z=9.7,
                            gyro_x=0.01,
                            gyro_y=0.02,
                            gyro_z=0.03,
                            orientation_x=0.0,
                            orientation_y=0.0,
                            orientation_z=0.70710678,
                            orientation_w=0.70710678,
                            yaw_deg=90.0,
                        ),
                    )
                )
                session.write_snapshot(
                    RosSnapshot(
                        frame_count=5,
                        last_topic="/active_imu",
                        active_imu=RosImuReading(
                            frame_count=2,
                            accel_x=-1.1,
                            accel_y=-1.2,
                            accel_z=-9.8,
                            gyro_x=-0.01,
                            gyro_y=-0.02,
                            gyro_z=-0.03,
                            orientation_x=0.0,
                            orientation_y=0.70710678,
                            orientation_z=0.0,
                            orientation_w=0.70710678,
                            pitch_deg=90.0,
                        ),
                    )
                )
            session.finalize()

            with (tmp / "ros_odom.csv").open("r", encoding="utf-8", newline="") as fh:
                odom_rows = list(csv.reader(fh))
            with (tmp / "ros_imu.csv").open("r", encoding="utf-8", newline="") as fh:
                imu_rows = list(csv.reader(fh))
            with (tmp / "ros_active_imu.csv").open("r", encoding="utf-8", newline="") as fh:
                active_rows = list(csv.reader(fh))
            with (tmp / "ros_imu_merged_aligned.csv").open("r", encoding="utf-8", newline="") as fh:
                merged_rows = list(csv.reader(fh))

            self.assertEqual(odom_rows[0], ROS_SUMMARY_ODOM_HEADER)
            self.assertEqual(odom_rows[0][2:4], ["motor_a_left_speed", "motor_b_right_speed"])
            self.assertEqual(odom_rows[1][1], "3")
            self.assertEqual(odom_rows[1][2], "0.42")
            self.assertEqual(odom_rows[1][5], "1.0")
            self.assertEqual(imu_rows[0], ROS_IMU_RAW_HEADER)
            self.assertEqual(len(imu_rows), 2)
            self.assertEqual(imu_rows[1][1], "1")
            self.assertEqual(imu_rows[1][4], "9.7")
            self.assertEqual(imu_rows[1][-1], "90.0")
            self.assertEqual(active_rows[0], ROS_IMU_RAW_HEADER)
            self.assertEqual(len(active_rows), 2)
            self.assertEqual(active_rows[1][1], "2")
            self.assertEqual(active_rows[1][4], "-9.8")
            self.assertEqual(active_rows[1][-2], "90.0")
            self.assertEqual(merged_rows[0], ROS_IMU_ALIGNED_HEADER)
            self.assertEqual(len(merged_rows), 2)
            self.assertEqual(merged_rows[1][0], "0")
            self.assertEqual(merged_rows[1][2], "5.000")
            self.assertEqual(merged_rows[1][4], "1")
            active_start = merged_rows[0].index("active_imu_frame_count")
            self.assertEqual(merged_rows[1][active_start], "2")
            self.assertEqual(session.rows_written_by_stream, {"odom": 1, "imu": 2})

    def test_ros_imu_summary_aligns_nearby_topic_updates_without_repeating_previous_side(self) -> None:
        with temp_dir() as tmp:
            session = RosSummaryRecordingSession()
            with patch("ros_data.perf_counter", side_effect=[100.0, 100.010, 100.015]):
                session.start_in_directory(tmp, started_at="20260423_101502")
                session.write_snapshot(
                    RosSnapshot(
                        frame_count=1,
                        last_topic="/active_imu",
                        active_imu=RosImuReading(frame_count=7, accel_z=-9.8),
                    )
                )
                session.write_snapshot(
                    RosSnapshot(
                        frame_count=2,
                        last_topic="/imu",
                        imu=RosImuReading(frame_count=8, accel_z=9.8),
                        active_imu=RosImuReading(frame_count=7, accel_z=-9.8),
                    )
                )
            session.finalize()

            with (tmp / "ros_imu_merged_aligned.csv").open("r", encoding="utf-8", newline="") as fh:
                imu_rows = list(csv.reader(fh))

            active_start = imu_rows[0].index("active_imu_frame_count")
            self.assertEqual(len(imu_rows), 2)
            self.assertEqual(imu_rows[1][0], "0")
            self.assertEqual(imu_rows[1][2], "5.000")
            self.assertEqual(imu_rows[1][4], "8")
            self.assertEqual(imu_rows[1][7], "9.8")
            self.assertEqual(imu_rows[1][active_start], "7")
            self.assertEqual(imu_rows[1][active_start + 3], "-9.8")
            self.assertEqual(session.rows_written_by_stream, {"odom": 0, "imu": 2})

    def test_ros_imu_summary_does_not_pair_samples_more_than_10ms_apart(self) -> None:
        with temp_dir() as tmp:
            session = RosSummaryRecordingSession()
            with patch("ros_data.perf_counter", side_effect=[100.0, 100.010, 100.021]):
                session.start_in_directory(tmp, started_at="20260423_101503")
                session.write_snapshot(
                    RosSnapshot(
                        frame_count=1,
                        last_topic="/active_imu",
                        active_imu=RosImuReading(frame_count=7, accel_z=-9.8),
                    )
                )
                session.write_snapshot(
                    RosSnapshot(
                        frame_count=2,
                        last_topic="/imu",
                        imu=RosImuReading(frame_count=8, accel_z=9.8),
                    )
                )
            session.finalize()

            with (tmp / "ros_imu_merged_aligned.csv").open("r", encoding="utf-8", newline="") as fh:
                merged_rows = list(csv.reader(fh))

            self.assertEqual(merged_rows, [ROS_IMU_ALIGNED_HEADER])
            self.assertEqual(session.rows_written_by_stream, {"odom": 0, "imu": 2})

    def test_ros_imu_summary_leaves_missing_side_blank_in_wide_rows(self) -> None:
        with temp_dir() as tmp:
            session = RosSummaryRecordingSession()
            session.start_in_directory(tmp, started_at="20260423_101501")

            session.write_snapshot(
                RosSnapshot(
                    frame_count=1,
                    last_topic="/imu",
                    imu=RosImuReading(frame_count=1, accel_z=9.8),
                )
            )
            session.finalize()

            with (tmp / "ros_imu.csv").open("r", encoding="utf-8", newline="") as fh:
                imu_rows = list(csv.reader(fh))
            with (tmp / "ros_active_imu.csv").open("r", encoding="utf-8", newline="") as fh:
                active_rows = list(csv.reader(fh))
            with (tmp / "ros_imu_merged_aligned.csv").open("r", encoding="utf-8", newline="") as fh:
                merged_rows = list(csv.reader(fh))

            self.assertEqual(imu_rows[0], ROS_IMU_RAW_HEADER)
            self.assertEqual(imu_rows[1][1], "1")
            self.assertEqual(imu_rows[1][4], "9.8")
            self.assertEqual(active_rows, [ROS_IMU_RAW_HEADER])
            self.assertEqual(merged_rows, [ROS_IMU_ALIGNED_HEADER])
            self.assertEqual(session.rows_written_by_stream, {"odom": 0, "imu": 1})


if __name__ == "__main__":
    unittest.main()

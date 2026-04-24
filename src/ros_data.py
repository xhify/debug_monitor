"""ROS topic time-series buffering and CSV recording."""

from __future__ import annotations

import csv
import errno
import shutil
from collections import deque
from datetime import datetime
from pathlib import Path
from time import perf_counter

import numpy as np

from ros_bridge_worker import RosSnapshot

ROS_SERIES_KEYS = [
    "linear_x",
    "linear_y",
    "angular_z",
    "pose_x",
    "pose_y",
    "accel_z",
    "gyro_z",
    "voltage",
]

ROS_CSV_HEADER = [
    "time_s",
    "frame_count",
    "motor_a_left_speed",
    "motor_b_right_speed",
    "angular_z",
    "pose_x",
    "pose_y",
    "pose_z",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "voltage",
]

ROS_IMU_DEVICES = ("imu", "active_imu")
ROS_IMU_SERIES_KEYS = (
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
)

ROS_SUMMARY_ODOM_HEADER = [
    "time_s",
    "frame_count",
    "motor_a_left_speed",
    "motor_b_right_speed",
    "angular_z",
    "pose_x",
    "pose_y",
    "pose_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
]

ROS_IMU_RAW_HEADER = [
    "time_s",
    "frame_count",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
]

ROS_IMU_ALIGNED_HEADER = [
    "pair_index",
    "pair_time",
    "time_delta_ms",
    "imu_time_s",
    "imu_frame_count",
    "imu_accel_x",
    "imu_accel_y",
    "imu_accel_z",
    "imu_gyro_x",
    "imu_gyro_y",
    "imu_gyro_z",
    "imu_orientation_x",
    "imu_orientation_y",
    "imu_orientation_z",
    "imu_orientation_w",
    "imu_roll_deg",
    "imu_pitch_deg",
    "imu_yaw_deg",
    "active_imu_time_s",
    "active_imu_frame_count",
    "active_imu_accel_x",
    "active_imu_accel_y",
    "active_imu_accel_z",
    "active_imu_gyro_x",
    "active_imu_gyro_y",
    "active_imu_gyro_z",
    "active_imu_orientation_x",
    "active_imu_orientation_y",
    "active_imu_orientation_z",
    "active_imu_orientation_w",
    "active_imu_roll_deg",
    "active_imu_pitch_deg",
    "active_imu_yaw_deg",
]

ROS_SUMMARY_IMU_HEADER = ROS_IMU_ALIGNED_HEADER


class RosTimeSeriesBuffer:
    """Fixed-size chronological buffer for ROS snapshots."""

    def __init__(self, capacity: int = 3000) -> None:
        self._rows: deque[tuple[float, RosSnapshot]] = deque(maxlen=capacity)
        self._start_timestamp: float | None = None

    def append(self, snapshot: RosSnapshot, timestamp: float | None = None) -> float:
        if timestamp is None:
            timestamp = perf_counter()
        if self._start_timestamp is None:
            self._start_timestamp = timestamp
        time_s = float(timestamp - self._start_timestamp)
        self._rows.append((time_s, snapshot))
        return time_s

    def clear(self) -> None:
        self._rows.clear()
        self._start_timestamp = None

    def snapshot(self) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        times = np.array([row[0] for row in self._rows], dtype=np.float64)
        data = {
            key: np.array([float(getattr(row[1], key)) for row in self._rows], dtype=np.float64)
            for key in ROS_SERIES_KEYS
        }
        return times, data

    @property
    def count(self) -> int:
        return len(self._rows)


class RosDualImuTimeSeriesBuffer:
    """Fixed-size chronological buffer for two ROS IMU readings."""

    def __init__(self, capacity: int = 3000) -> None:
        self._rows: deque[tuple[float, RosSnapshot]] = deque(maxlen=capacity)
        self._start_timestamp: float | None = None

    def append(self, snapshot: RosSnapshot, timestamp: float | None = None) -> float:
        if timestamp is None:
            timestamp = perf_counter()
        if self._start_timestamp is None:
            self._start_timestamp = timestamp
        time_s = float(timestamp - self._start_timestamp)
        self._rows.append((time_s, snapshot.clone()))
        return time_s

    def clear(self) -> None:
        self._rows.clear()
        self._start_timestamp = None

    def snapshot(self) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        times = np.array([row[0] for row in self._rows], dtype=np.float64)
        data: dict[str, np.ndarray] = {}
        for device_key in ROS_IMU_DEVICES:
            for series_key in ROS_IMU_SERIES_KEYS:
                data[f"{device_key}_{series_key}"] = np.array(
                    [
                        float(getattr(getattr(row[1], device_key), series_key))
                        for row in self._rows
                    ],
                    dtype=np.float64,
                )
        return times, data

    @property
    def count(self) -> int:
        return len(self._rows)


class RosCsvRecordingSession:
    """Streaming CSV recording for ROS snapshots."""

    def __init__(self, base_dir: Path) -> None:
        self._base_dir = Path(base_dir)
        self._file = None
        self._writer: csv.writer | None = None
        self._temp_path: Path | None = None
        self.rows_written = 0

    @property
    def temp_path(self) -> Path | None:
        return self._temp_path

    def start(self) -> Path:
        self._base_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        temp_path, handle = self._create_temp_file(timestamp)
        self._file = handle
        self._temp_path = temp_path
        self._writer = csv.writer(handle)
        self._writer.writerow(ROS_CSV_HEADER)
        handle.flush()
        self.rows_written = 0
        return temp_path

    def _create_temp_file(self, timestamp: str):
        for counter in range(1000):
            temp_path = self._base_dir / f"ros_recording_{timestamp}_{counter:03d}.tmp.csv"
            try:
                return temp_path, temp_path.open("x", newline="", encoding="utf-8")
            except FileExistsError:
                continue
        raise RuntimeError("could not create temporary ROS recording file")

    def write_snapshot(self, time_s: float, snapshot: RosSnapshot) -> None:
        if self._file is None or self._writer is None:
            raise RuntimeError("recording not started")
        self._writer.writerow([
            f"{time_s:.3f}",
            snapshot.frame_count,
            snapshot.linear_x,
            snapshot.linear_y,
            snapshot.angular_z,
            snapshot.pose_x,
            snapshot.pose_y,
            snapshot.pose_z,
            snapshot.accel_x,
            snapshot.accel_y,
            snapshot.accel_z,
            snapshot.gyro_x,
            snapshot.gyro_y,
            snapshot.gyro_z,
            snapshot.voltage,
        ])
        self._file.flush()
        self.rows_written += 1

    def finalize(self, final_path: Path) -> None:
        if self._temp_path is None:
            raise RuntimeError("recording not started")
        source = self._temp_path
        self._close()
        final_path = Path(final_path)
        final_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            source.replace(final_path)
        except OSError as exc:
            if exc.errno != errno.EXDEV and getattr(exc, "winerror", None) != 17:
                raise
            shutil.move(str(source), str(final_path))
        self._temp_path = None

    def cancel(self) -> None:
        if self._temp_path is None:
            return
        source = self._temp_path
        self._close()
        if source.exists():
            source.unlink()
        self._temp_path = None

    def _close(self) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None
        self._writer = None


class RosSummaryRecordingSession:
    """Streaming CSV recording for summary sessions using ROS sources."""

    IMU_TOPICS = {
        "/imu": "imu",
        "/active_imu": "active_imu",
    }

    def __init__(self, imu_align_window_seconds: float = 0.01) -> None:
        self._imu_align_window_seconds = float(imu_align_window_seconds)
        self._session_dir: Path | None = None
        self._started_at: float | None = None
        self._odom_file = None
        self._odom_writer: csv.writer | None = None
        self._imu_files: dict[str, object] = {}
        self._imu_writers: dict[str, csv.writer] = {}
        self._imu_samples: dict[str, list[tuple[float, object]]] = {
            "imu": [],
            "active_imu": [],
        }
        self.rows_written_by_stream = {"odom": 0, "imu": 0}

    def start_in_directory(self, session_dir: Path, started_at: str | None = None) -> None:
        del started_at
        self._session_dir = Path(session_dir)
        self._session_dir.mkdir(parents=True, exist_ok=True)
        self._started_at = perf_counter()
        self._odom_file = (self._session_dir / "ros_odom.csv").open("w", newline="", encoding="utf-8")
        self._odom_writer = csv.writer(self._odom_file)
        self._odom_writer.writerow(ROS_SUMMARY_ODOM_HEADER)
        self._open_imu_raw_files()
        self._flush()
        self._imu_samples = {"imu": [], "active_imu": []}
        self.rows_written_by_stream = {"odom": 0, "imu": 0}

    def _open_imu_raw_files(self) -> None:
        if self._session_dir is None:
            raise RuntimeError("session directory not set")
        for device_key, filename in (
            ("imu", "ros_imu.csv"),
            ("active_imu", "ros_active_imu.csv"),
        ):
            handle = (self._session_dir / filename).open("w", newline="", encoding="utf-8")
            writer = csv.writer(handle)
            writer.writerow(ROS_IMU_RAW_HEADER)
            handle.flush()
            self._imu_files[device_key] = handle
            self._imu_writers[device_key] = writer

    def write_snapshot(self, snapshot: RosSnapshot) -> None:
        if self._started_at is None:
            raise RuntimeError("recording not started")
        time_s = perf_counter() - self._started_at
        if snapshot.last_topic == "/odom":
            self._write_odom(time_s, snapshot)
            return
        if snapshot.last_topic in self.IMU_TOPICS:
            self._write_imu_raw(time_s, snapshot)

    def finalize(self) -> None:
        if self._session_dir is None:
            raise RuntimeError("recording not started")
        self._close_imu_raw_files()
        self._write_aligned_imu_rows()
        self._close()
        self._session_dir = None

    def cancel(self) -> None:
        session_dir = self._session_dir
        self._close()
        if session_dir is None:
            return
        for filename in ("ros_odom.csv", "ros_imu.csv", "ros_active_imu.csv", "ros_imu_merged_aligned.csv"):
            path = session_dir / filename
            if path.exists():
                path.unlink()

    def _write_odom(self, time_s: float, snapshot: RosSnapshot) -> None:
        if self._odom_writer is None:
            raise RuntimeError("recording not started")
        self._odom_writer.writerow([
            f"{time_s:.3f}",
            snapshot.frame_count,
            snapshot.linear_x,
            snapshot.linear_y,
            snapshot.angular_z,
            snapshot.pose_x,
            snapshot.pose_y,
            snapshot.pose_z,
            snapshot.orientation_x,
            snapshot.orientation_y,
            snapshot.orientation_z,
            snapshot.orientation_w,
        ])
        self.rows_written_by_stream["odom"] += 1
        self._flush()

    def _write_imu_raw(self, time_s: float, snapshot: RosSnapshot) -> None:
        device_key = self.IMU_TOPICS[snapshot.last_topic]
        reading = getattr(snapshot, device_key)
        if reading.frame_count <= 0:
            return
        writer = self._imu_writers.get(device_key)
        handle = self._imu_files.get(device_key)
        if writer is None or handle is None:
            raise RuntimeError("recording not started")
        writer.writerow([
            f"{time_s:.3f}",
            *self._imu_reading_values(reading),
        ])
        handle.flush()
        self.rows_written_by_stream["imu"] += 1
        self._imu_samples[device_key].append((time_s, reading.clone()))

    def _write_aligned_imu_rows(self) -> None:
        if self._session_dir is None:
            return
        rows = _build_ros_imu_aligned_rows(
            self._imu_samples["imu"],
            self._imu_samples["active_imu"],
            self._imu_align_window_seconds,
        )
        with (self._session_dir / "ros_imu_merged_aligned.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(ROS_IMU_ALIGNED_HEADER)
            writer.writerows(rows)

    @staticmethod
    def _imu_reading_values(reading) -> list[object]:
        if reading is None:
            return [""] * 14
        if reading.frame_count <= 0:
            return [""] * 14
        return [
            reading.frame_count,
            reading.accel_x,
            reading.accel_y,
            reading.accel_z,
            reading.gyro_x,
            reading.gyro_y,
            reading.gyro_z,
            reading.orientation_x,
            reading.orientation_y,
            reading.orientation_z,
            reading.orientation_w,
            reading.roll_deg,
            reading.pitch_deg,
            reading.yaw_deg,
        ]

    def _flush(self) -> None:
        if self._odom_file is not None:
            self._odom_file.flush()
        for handle in self._imu_files.values():
            handle.flush()

    def _close(self) -> None:
        self._close_odom()
        self._close_imu_raw_files()
        self._started_at = None

    def _close_odom(self) -> None:
        if self._odom_file is not None:
            self._odom_file.close()
            self._odom_file = None
        self._odom_writer = None

    def _close_imu_raw_files(self) -> None:
        for handle in self._imu_files.values():
            handle.close()
        self._imu_files = {}
        self._imu_writers = {}


def _build_ros_imu_aligned_rows(
    imu_samples: list[tuple[float, object]],
    active_samples: list[tuple[float, object]],
    align_window_seconds: float,
) -> list[list[object]]:
    rows: list[list[object]] = []
    imu_index = 0
    active_index = 0
    while imu_index < len(imu_samples) and active_index < len(active_samples):
        imu_time, imu_reading = imu_samples[imu_index]
        active_time, active_reading = active_samples[active_index]
        delta = active_time - imu_time
        if abs(delta) <= align_window_seconds + 1e-12:
            rows.append(_ros_imu_aligned_row(len(rows), imu_time, delta, imu_reading, active_time, active_reading))
            imu_index += 1
            active_index += 1
        elif imu_time < active_time:
            imu_index += 1
        else:
            active_index += 1
    return rows


def _ros_imu_aligned_row(
    pair_index: int,
    imu_time: float,
    delta_s: float,
    imu_reading,
    active_time: float,
    active_reading,
) -> list[object]:
    return [
        pair_index,
        f"{imu_time:.6f}",
        f"{abs(delta_s) * 1000.0:.3f}",
        f"{imu_time:.6f}",
        *RosSummaryRecordingSession._imu_reading_values(imu_reading),
        f"{active_time:.6f}",
        *RosSummaryRecordingSession._imu_reading_values(active_reading),
    ]

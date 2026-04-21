"""Thread-safe ring buffer for IMU samples."""

from __future__ import annotations

import threading

import numpy as np

from imu_protocol import ImuSample
from imu_recording import ImuRecordingSession


DEFAULT_IMU_CAPACITY = 3000

COL_ACC_X = 0
COL_ACC_Y = 1
COL_ACC_Z = 2
COL_GYRO_X = 3
COL_GYRO_Y = 4
COL_GYRO_Z = 5
COL_EULER_PITCH = 6
COL_EULER_ROLL = 7
COL_EULER_YAW = 8
IMU_NUM_COLS = 9


class ImuBuffer:
    def __init__(self, capacity: int = DEFAULT_IMU_CAPACITY) -> None:
        self._capacity = int(capacity)
        self._lock = threading.Lock()
        self._time = np.zeros(self._capacity, dtype=np.float64)
        self._data = np.full((self._capacity, IMU_NUM_COLS), np.nan, dtype=np.float64)
        self._write_idx = 0
        self._count = 0
        self._frame_index = 0
        self._latest: ImuSample | None = None
        self._recording_session: ImuRecordingSession | None = None

    def append(self, sample: ImuSample) -> None:
        with self._lock:
            idx = self._write_idx % self._capacity
            self._time[idx] = sample.host_time
            self._data[idx, :] = np.nan
            self._write_vec(idx, (COL_ACC_X, COL_ACC_Y, COL_ACC_Z), sample.accel)
            self._write_vec(idx, (COL_GYRO_X, COL_GYRO_Y, COL_GYRO_Z), sample.gyro)
            self._write_vec(
                idx,
                (COL_EULER_PITCH, COL_EULER_ROLL, COL_EULER_YAW),
                sample.euler,
            )
            self._write_idx += 1
            if self._count < self._capacity:
                self._count += 1
            self._frame_index += 1
            self._latest = sample
            recording_session = self._recording_session
        if recording_session is not None:
            recording_session.write_sample(sample)

    def _write_vec(self, idx: int, cols: tuple[int, int, int], values: tuple[float, float, float] | None) -> None:
        if values is None:
            return
        for col, value in zip(cols, values):
            self._data[idx, col] = value

    def get_snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            n = self._count
            if n == 0:
                return np.array([]), np.zeros((0, IMU_NUM_COLS))

            if n < self._capacity:
                return self._time[:n].copy(), self._data[:n].copy()

            start = self._write_idx % self._capacity
            time = np.concatenate([self._time[start:], self._time[:start]])
            data = np.concatenate([self._data[start:], self._data[:start]], axis=0)
            return time, data

    def get_latest(self) -> ImuSample | None:
        with self._lock:
            return self._latest

    @property
    def frame_index(self) -> int:
        with self._lock:
            return self._frame_index

    def clear(self) -> None:
        with self._lock:
            self._time[:] = 0
            self._data[:, :] = np.nan
            self._write_idx = 0
            self._count = 0
            self._frame_index = 0
            self._latest = None

    def start_recording(self, session: ImuRecordingSession) -> None:
        with self._lock:
            self._recording_session = session

    def stop_recording(self) -> ImuRecordingSession | None:
        with self._lock:
            session = self._recording_session
            self._recording_session = None
            return session

    @property
    def recording(self) -> bool:
        with self._lock:
            return self._recording_session is not None

    @property
    def csv_rows_written(self) -> int:
        with self._lock:
            if self._recording_session is None:
                return 0
            return self._recording_session.rows_written


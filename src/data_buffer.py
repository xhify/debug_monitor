"""
线程安全的 numpy 环形缓冲区。

子线程（SerialWorker）通过 append() 写入数据，
主线程通过 get_snapshot() / get_latest() 读取数据用于绘图和数值更新。
所有公开方法均通过 threading.Lock 保证线程安全。
"""

from __future__ import annotations

import threading

import numpy as np

from protocol import DataFrame
from recording_session import RecordingSession

CAPACITY = 3000

COL_T_RAW_A = 0
COL_T_RAW_B = 1
COL_M_RAW_A = 2
COL_M_RAW_B = 3
COL_FINAL_A = 4
COL_FINAL_B = 5
COL_TGT_A = 6
COL_TGT_B = 7
COL_OUT_A = 8
COL_OUT_B = 9
NUM_COLS = 10


class DataBuffer:
    """线程安全的环形数据缓冲区，并可绑定流式录制会话。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data = np.zeros((CAPACITY, NUM_COLS), dtype=np.float64)
        self._write_idx = 0
        self._count = 0
        self._frame_index = 0
        self._latest: DataFrame | None = None
        self._recording_session: RecordingSession | None = None

    def append(self, frame: DataFrame) -> None:
        """将一帧数据写入环形缓冲区。由子线程调用，加锁保护。"""
        with self._lock:
            idx = self._write_idx % CAPACITY
            self._data[idx, COL_T_RAW_A] = frame.t_raw_A
            self._data[idx, COL_T_RAW_B] = frame.t_raw_B
            self._data[idx, COL_M_RAW_A] = frame.m_raw_A
            self._data[idx, COL_M_RAW_B] = frame.m_raw_B
            self._data[idx, COL_FINAL_A] = frame.final_A
            self._data[idx, COL_FINAL_B] = frame.final_B
            self._data[idx, COL_TGT_A] = frame.target_A
            self._data[idx, COL_TGT_B] = frame.target_B
            self._data[idx, COL_OUT_A] = frame.output_A
            self._data[idx, COL_OUT_B] = frame.output_B

            self._write_idx += 1
            if self._count < CAPACITY:
                self._count += 1
            frame_index = self._frame_index
            self._frame_index += 1
            self._latest = frame
            recording_session = self._recording_session

        if recording_session is not None:
            time_s = frame_index * 0.01
            recording_session.write_frame(frame_index=frame_index, time_s=time_s, frame=frame)

    def get_snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        """获取当前缓冲区的有序快照，返回 (time_array, data_array)。"""
        with self._lock:
            n = self._count
            if n == 0:
                return np.array([]), np.zeros((0, NUM_COLS))

            if n < CAPACITY:
                data = self._data[:n].copy()
            else:
                start = self._write_idx % CAPACITY
                data = np.concatenate([self._data[start:], self._data[:start]], axis=0)

            end_index = self._frame_index
            start_index = end_index - n
            time_array = np.arange(start_index, end_index, dtype=np.float64) * 0.01
            return time_array, data

    def get_recent_window(self, window_s: float) -> tuple[np.ndarray, np.ndarray]:
        """返回最近指定秒数的数据窗口。"""
        time_array, data = self.get_snapshot()
        if time_array.size == 0:
            return time_array, data
        start_time = max(float(time_array[-1] - window_s), float(time_array[0]))
        start_idx = int(np.searchsorted(time_array, start_time, side="right"))
        return time_array[start_idx:], data[start_idx:]

    def get_latest(self) -> DataFrame | None:
        """获取最新一帧数据，用于数值面板更新。"""
        with self._lock:
            return self._latest

    @property
    def frame_index(self) -> int:
        with self._lock:
            return self._frame_index

    def clear(self) -> None:
        """清空缓冲区数据。"""
        with self._lock:
            self._data[:] = 0
            self._write_idx = 0
            self._count = 0
            self._frame_index = 0
            self._latest = None

    def start_recording(self, session: RecordingSession) -> None:
        """绑定一个已启动的流式录制会话。"""
        with self._lock:
            self._recording_session = session

    def stop_recording(self) -> RecordingSession | None:
        """解除当前录制会话并返回它，由调用方决定保存或取消。"""
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
            return 0 if self._recording_session is None else self._recording_session.rows_written

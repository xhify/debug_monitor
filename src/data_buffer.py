"""
线程安全的 numpy 环形缓冲区

子线程（SerialWorker）通过 append() 写入数据，
主线程通过 get_snapshot() / get_latest() 读取数据用于绘图和数值更新。
所有公开方法均通过 threading.Lock 保证线程安全。

CSV 记录：记录期间数据缓存在内存中，停止记录时一次性写入文件。
"""

import csv
import threading
from pathlib import Path

import numpy as np

from protocol import DataFrame

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
COL_AFC_A = 10
COL_AFC_B = 11
NUM_COLS = 12

CSV_HEADER = [
    'frame_index', 'time_s',
    't_raw_a', 't_raw_b',
    'm_raw_a', 'm_raw_b',
    'final_a', 'final_b',
    'target_a', 'target_b',
    'output_a', 'output_b',
    'afc_output_a', 'afc_output_b',
]


class DataBuffer:
    """线程安全的环形数据缓冲区，附带 CSV 内存缓存记录功能。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data = np.zeros((CAPACITY, NUM_COLS), dtype=np.float64)
        self._write_idx: int = 0
        self._count: int = 0
        self._frame_index: int = 0
        self._latest: DataFrame | None = None
        self._recording: bool = False
        self._csv_buffer: list[list] = []

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
            self._data[idx, COL_AFC_A] = frame.afc_output_A
            self._data[idx, COL_AFC_B] = frame.afc_output_B

            self._write_idx += 1
            if self._count < CAPACITY:
                self._count += 1
            self._frame_index += 1
            self._latest = frame

            if self._recording:
                time_s = (self._frame_index - 1) * 0.01
                self._csv_buffer.append([
                    self._frame_index - 1, f'{time_s:.2f}',
                    frame.t_raw_A, frame.t_raw_B,
                    frame.m_raw_A, frame.m_raw_B,
                    frame.final_A, frame.final_B,
                    frame.target_A, frame.target_B,
                    frame.output_A, frame.output_B,
                    frame.afc_output_A, frame.afc_output_B,
                ])

    def get_snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        """
        获取当前缓冲区的有序快照。
        返回 (time_array, data_array)。
        """
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

    def get_latest(self) -> DataFrame | None:
        """获取最新一帧数据，用于数值面板更新。"""
        with self._lock:
            return self._latest

    @property
    def frame_index(self) -> int:
        return self._frame_index

    def clear(self) -> None:
        """清空缓冲区数据。"""
        with self._lock:
            self._data[:] = 0
            self._write_idx = 0
            self._count = 0
            self._frame_index = 0
            self._latest = None

    def start_recording(self) -> None:
        """开始记录，数据缓存在内存中。"""
        with self._lock:
            if self._recording:
                return
            self._csv_buffer.clear()
            self._recording = True

    def stop_recording(self, filepath: str | Path | None = None) -> int:
        """
        停止记录。如果提供了 filepath，将缓存数据一次性写入 CSV 文件。
        filepath 为 None 时丢弃数据。返回记录的行数。
        """
        with self._lock:
            if not self._recording:
                return 0
            self._recording = False
            total = len(self._csv_buffer)
            if filepath and self._csv_buffer:
                with open(filepath, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(CSV_HEADER)
                    writer.writerows(self._csv_buffer)
            self._csv_buffer.clear()
            return total

    @property
    def recording(self) -> bool:
        return self._recording

    @property
    def csv_rows_written(self) -> int:
        return len(self._csv_buffer)

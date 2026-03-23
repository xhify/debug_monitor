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

# 环形缓冲区容量：3000 个采样点 ≈ 30 秒 @100Hz
CAPACITY = 3000

# 数据通道列索引
COL_RAW_A = 0
COL_RAW_B = 1
COL_FILT_A = 2
COL_FILT_B = 3
COL_TGT_A = 4
COL_TGT_B = 5
COL_OUT_A = 6
COL_OUT_B = 7
NUM_COLS = 8

# CSV 表头
CSV_HEADER = [
    'frame_index', 'time_s',
    'raw_speed_a', 'raw_speed_b',
    'filtered_a', 'filtered_b',
    'target_a', 'target_b',
    'output_a', 'output_b',
]


class DataBuffer:
    """线程安全的环形数据缓冲区，附带 CSV 内存缓存记录功能。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data = np.zeros((CAPACITY, NUM_COLS), dtype=np.float64)
        self._write_idx: int = 0    # 下一个写入位置（对 CAPACITY 取模）
        self._count: int = 0        # 已写入的有效样本数（上限 CAPACITY）
        self._frame_index: int = 0  # 全局帧计数器（持续自增，不回绕）
        self._latest: DataFrame | None = None  # 最新一帧原始数据

        # CSV 记录相关（内存缓存模式，停止时一次性写入文件）
        self._recording: bool = False
        self._csv_buffer: list[list] = []

    # ─── 写入（子线程调用）────────────────────────────────

    def append(self, frame: DataFrame) -> None:
        """将一帧数据写入环形缓冲区。由子线程调用，加锁保护。"""
        with self._lock:
            idx = self._write_idx % CAPACITY
            self._data[idx, COL_RAW_A] = frame.raw_speed_A
            self._data[idx, COL_RAW_B] = frame.raw_speed_B
            self._data[idx, COL_FILT_A] = frame.filtered_A
            self._data[idx, COL_FILT_B] = frame.filtered_B
            self._data[idx, COL_TGT_A] = frame.target_A
            self._data[idx, COL_TGT_B] = frame.target_B
            self._data[idx, COL_OUT_A] = frame.output_A
            self._data[idx, COL_OUT_B] = frame.output_B

            self._write_idx += 1
            if self._count < CAPACITY:
                self._count += 1
            self._frame_index += 1
            self._latest = frame

            # CSV 记录（内存缓存）
            if self._recording:
                time_s = (self._frame_index - 1) * 0.01
                self._csv_buffer.append([
                    self._frame_index - 1, f'{time_s:.2f}',
                    frame.raw_speed_A, frame.raw_speed_B,
                    frame.filtered_A, frame.filtered_B,
                    frame.target_A, frame.target_B,
                    frame.output_A, frame.output_B,
                ])

    # ─── 读取（主线程调用）────────────────────────────────

    def get_snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        """
        获取当前缓冲区的有序快照。
        返回 (time_array, data_array)：
          - time_array: shape (n,)，单位秒，基于 frame_index × 0.01
          - data_array: shape (n, 8)，8个数据通道
        """
        with self._lock:
            n = self._count
            if n == 0:
                return np.array([]), np.zeros((0, NUM_COLS))

            if n < CAPACITY:
                # 缓冲区未满，直接切片
                data = self._data[:n].copy()
            else:
                # 缓冲区已满，需重排使最旧数据在前
                start = self._write_idx % CAPACITY
                data = np.concatenate(
                    [self._data[start:], self._data[:start]], axis=0
                )

            # 时间轴：基于全局帧计数推算理论时间
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
        """当前全局帧计数（无锁读取，仅用于状态栏显示）。"""
        return self._frame_index

    # ─── 清空 ─────────────────────────────────────────────

    def clear(self) -> None:
        """清空缓冲区数据。"""
        with self._lock:
            self._data[:] = 0
            self._write_idx = 0
            self._count = 0
            self._frame_index = 0
            self._latest = None

    # ─── CSV 记录（内存缓存模式）──────────────────────────

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
        """当前内存缓存中的记录行数。"""
        return len(self._csv_buffer)

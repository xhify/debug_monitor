"""CSV replay data loading and slicing."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from data_buffer import (
    COL_AFC_A, COL_AFC_B, COL_FINAL_A, COL_FINAL_B, COL_M_RAW_A, COL_M_RAW_B,
    COL_OUT_A, COL_OUT_B, COL_TGT_A, COL_TGT_B, COL_T_RAW_A, COL_T_RAW_B, NUM_COLS,
)

REQUIRED_COLUMNS = [
    "frame_index", "time_s",
    "t_raw_a", "t_raw_b",
    "m_raw_a", "m_raw_b",
    "final_a", "final_b",
    "target_a", "target_b",
    "output_a", "output_b",
]
OPTIONAL_AFC_COLUMNS = ["afc_output_a", "afc_output_b"]


@dataclass(slots=True)
class ReplayData:
    time_s: np.ndarray
    data: np.ndarray

    @classmethod
    def load(cls, path: Path) -> "ReplayData":
        with Path(path).open("r", encoding="utf-8", newline="") as fh:
            reader = csv.DictReader(fh)
            if reader.fieldnames is None:
                raise ValueError("CSV has no header")
            missing = [col for col in REQUIRED_COLUMNS if col not in reader.fieldnames]
            if missing:
                raise ValueError(f"missing columns: {missing}")

            rows = list(reader)
        if not rows:
            raise ValueError("CSV is empty")

        time_values = np.array([float(row["time_s"]) for row in rows], dtype=np.float64)
        data = np.zeros((len(rows), NUM_COLS), dtype=np.float64)
        for idx, row in enumerate(rows):
            data[idx, COL_T_RAW_A] = float(row["t_raw_a"])
            data[idx, COL_T_RAW_B] = float(row["t_raw_b"])
            data[idx, COL_M_RAW_A] = float(row["m_raw_a"])
            data[idx, COL_M_RAW_B] = float(row["m_raw_b"])
            data[idx, COL_FINAL_A] = float(row["final_a"])
            data[idx, COL_FINAL_B] = float(row["final_b"])
            data[idx, COL_TGT_A] = float(row["target_a"])
            data[idx, COL_TGT_B] = float(row["target_b"])
            data[idx, COL_OUT_A] = float(row["output_a"])
            data[idx, COL_OUT_B] = float(row["output_b"])
            data[idx, COL_AFC_A] = float(row.get("afc_output_a", 0.0) or 0.0)
            data[idx, COL_AFC_B] = float(row.get("afc_output_b", 0.0) or 0.0)
        return cls(time_s=time_values, data=data)

    @classmethod
    def from_rows(cls, time_values: list[float], rows: list[dict[str, float]]) -> "ReplayData":
        data = np.zeros((len(rows), NUM_COLS), dtype=np.float64)
        for idx, row in enumerate(rows):
            data[idx, COL_T_RAW_A] = float(row.get("t_raw_a", 0.0))
            data[idx, COL_T_RAW_B] = float(row.get("t_raw_b", 0.0))
            data[idx, COL_M_RAW_A] = float(row.get("m_raw_a", 0.0))
            data[idx, COL_M_RAW_B] = float(row.get("m_raw_b", 0.0))
            data[idx, COL_FINAL_A] = float(row.get("final_a", 0.0))
            data[idx, COL_FINAL_B] = float(row.get("final_b", 0.0))
            data[idx, COL_TGT_A] = float(row.get("target_a", 0.0))
            data[idx, COL_TGT_B] = float(row.get("target_b", 0.0))
            data[idx, COL_OUT_A] = float(row.get("output_a", 0.0))
            data[idx, COL_OUT_B] = float(row.get("output_b", 0.0))
            data[idx, COL_AFC_A] = float(row.get("afc_output_a", 0.0))
            data[idx, COL_AFC_B] = float(row.get("afc_output_b", 0.0))
        return cls(time_s=np.array(time_values, dtype=np.float64), data=data)

    @property
    def row_count(self) -> int:
        return int(self.time_s.size)

    @property
    def duration_s(self) -> float:
        if self.row_count == 0:
            return 0.0
        return float(self.time_s[-1])

    def latest_frame_at_time(self, current_time_s: float) -> dict[str, float]:
        idx = self.index_at_time(current_time_s)
        row = self.data[idx]
        return {
            "t_raw_A": float(row[COL_T_RAW_A]),
            "t_raw_B": float(row[COL_T_RAW_B]),
            "m_raw_A": float(row[COL_M_RAW_A]),
            "m_raw_B": float(row[COL_M_RAW_B]),
            "final_A": float(row[COL_FINAL_A]),
            "final_B": float(row[COL_FINAL_B]),
            "target_A": float(row[COL_TGT_A]),
            "target_B": float(row[COL_TGT_B]),
            "output_A": int(row[COL_OUT_A]),
            "output_B": int(row[COL_OUT_B]),
            "afc_output_A": float(row[COL_AFC_A]),
            "afc_output_B": float(row[COL_AFC_B]),
        }

    def index_at_time(self, current_time_s: float) -> int:
        idx = int(np.searchsorted(self.time_s, current_time_s, side="right") - 1)
        return max(0, min(idx, self.row_count - 1))

    def snapshot_to_time(self, current_time_s: float) -> tuple[np.ndarray, np.ndarray]:
        if self.row_count == 0:
            return np.array([]), np.zeros((0, NUM_COLS))
        idx = self.index_at_time(current_time_s)
        return self.time_s[: idx + 1].copy(), self.data[: idx + 1].copy()

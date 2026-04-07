"""Streaming CSV recording session management."""

from __future__ import annotations

import csv
import tempfile
from datetime import datetime
from pathlib import Path

from protocol import DataFrame

CSV_HEADER = [
    "frame_index", "time_s",
    "t_raw_a", "t_raw_b",
    "m_raw_a", "m_raw_b",
    "final_a", "final_b",
    "target_a", "target_b",
    "output_a", "output_b",
]


class RecordingSession:
    """Manage one streaming recording session backed by a temporary CSV file."""

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
        handle = tempfile.NamedTemporaryFile(
            mode="w",
            newline="",
            encoding="utf-8",
            prefix=f"recording_{timestamp}_",
            suffix=".tmp.csv",
            dir=self._base_dir,
            delete=False,
        )
        self._file = handle
        self._temp_path = Path(handle.name)
        self._writer = csv.writer(handle)
        self._writer.writerow(CSV_HEADER)
        handle.flush()
        self.rows_written = 0
        return self._temp_path

    def write_frame(self, frame_index: int, time_s: float, frame: DataFrame) -> None:
        if self._file is None or self._writer is None:
            raise RuntimeError("recording not started")

        self._writer.writerow([
            frame_index, f"{time_s:.2f}",
            frame.t_raw_A, frame.t_raw_B,
            frame.m_raw_A, frame.m_raw_B,
            frame.final_A, frame.final_B,
            frame.target_A, frame.target_B,
            frame.output_A, frame.output_B,
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
        source.replace(final_path)
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

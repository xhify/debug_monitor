# Streaming Record Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add streaming CSV recording, CSV replay, and shared live/replay analytics to the existing debug monitor.

**Architecture:** Keep the existing single-window application and add focused modules for recording, replay, and analytics. Extend the current refresh loop so plots, numeric panels, and analytics panels all consume one selected data source (`live` or `replay`) while live recording writes rows incrementally to a temporary CSV file.

**Tech Stack:** Python 3.11, PySide6, numpy, csv, unittest

---

### Task 1: Add Recording Session Module

**Files:**
- Create: `src/recording_session.py`
- Create: `tests/test_recording_session.py`

- [ ] **Step 1: Write the failing tests**

```python
import csv
import os
import tempfile
import unittest
from pathlib import Path

from recording_session import RecordingSession
from protocol import DataFrame


def make_frame() -> DataFrame:
    return DataFrame(
        t_raw_A=1.0,
        t_raw_B=2.0,
        m_raw_A=3.0,
        m_raw_B=4.0,
        final_A=5.0,
        final_B=6.0,
        target_A=7.0,
        target_B=8.0,
        output_A=9,
        output_B=10,
    )


class RecordingSessionTests(unittest.TestCase):
    def test_start_write_finalize_moves_temp_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            session = RecordingSession(base_dir=Path(tmp))
            temp_path = session.start()
            session.write_frame(frame_index=0, time_s=0.0, frame=make_frame())
            final_path = Path(tmp) / "saved.csv"

            session.finalize(final_path)

            self.assertFalse(temp_path.exists())
            self.assertTrue(final_path.exists())
            with final_path.open("r", encoding="utf-8", newline="") as fh:
                rows = list(csv.reader(fh))
            self.assertEqual(rows[0], [
                "frame_index", "time_s",
                "t_raw_a", "t_raw_b",
                "m_raw_a", "m_raw_b",
                "final_a", "final_b",
                "target_a", "target_b",
                "output_a", "output_b",
            ])
            self.assertEqual(rows[1][0], "0")

    def test_cancel_deletes_temp_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            session = RecordingSession(base_dir=Path(tmp))
            temp_path = session.start()
            session.write_frame(frame_index=0, time_s=0.0, frame=make_frame())

            session.cancel()

            self.assertFalse(temp_path.exists())
```

- [ ] **Step 2: Run test to verify it fails**

Run: `D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_recording_session -v`  
Expected: `ModuleNotFoundError` for `recording_session`

- [ ] **Step 3: Write minimal implementation**

```python
import csv
from datetime import datetime
from pathlib import Path
import tempfile

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
    def __init__(self, base_dir: Path) -> None:
        self._base_dir = Path(base_dir)
        self._file = None
        self._writer = None
        self._temp_path: Path | None = None
        self.rows_written = 0

    @property
    def temp_path(self) -> Path | None:
        return self._temp_path

    def start(self) -> Path:
        self._base_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fd, temp_name = tempfile.mkstemp(
            prefix=f"recording_{timestamp}_",
            suffix=".tmp.csv",
            dir=str(self._base_dir),
        )
        Path(temp_name).touch(exist_ok=True)
        self._temp_path = Path(temp_name)
        self._file = open(fd, "w", newline="", encoding="utf-8", closefd=False)
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_HEADER)
        self._file.flush()
        self.rows_written = 0
        return self._temp_path

    def write_frame(self, frame_index: int, time_s: float, frame: DataFrame) -> None:
        if self._writer is None or self._file is None:
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
        self._close()
        final_path = Path(final_path)
        final_path.parent.mkdir(parents=True, exist_ok=True)
        self._temp_path.replace(final_path)
        self._temp_path = None

    def cancel(self) -> None:
        if self._temp_path is None:
            return
        temp_path = self._temp_path
        self._close()
        if temp_path.exists():
            temp_path.unlink()
        self._temp_path = None

    def _close(self) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None
        self._writer = None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_recording_session -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add src/recording_session.py tests/test_recording_session.py
git commit -m "feat: add streaming recording session"
```

### Task 2: Add Replay Data Loader

**Files:**
- Create: `src/replay_data.py`
- Create: `tests/test_replay_data.py`

- [ ] **Step 1: Write the failing tests**

```python
import csv
import tempfile
import unittest
from pathlib import Path

from replay_data import ReplayData


class ReplayDataTests(unittest.TestCase):
    def test_load_csv_and_read_latest_row(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "sample.csv"
            with path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow([
                    "frame_index", "time_s",
                    "t_raw_a", "t_raw_b",
                    "m_raw_a", "m_raw_b",
                    "final_a", "final_b",
                    "target_a", "target_b",
                    "output_a", "output_b",
                ])
                writer.writerow([0, 0.0, 0, 0, 0, 0, 1.0, 2.0, 1.5, 2.5, 100, 120])
                writer.writerow([1, 0.1, 0, 0, 0, 0, 1.1, 2.1, 1.5, 2.5, 101, 121])

            replay = ReplayData.load(path)

            self.assertEqual(replay.row_count, 2)
            latest = replay.latest_frame_at_time(0.1)
            self.assertEqual(latest["final_a"], 1.1)
            self.assertEqual(latest["output_b"], 121)

    def test_missing_columns_raise_value_error(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.csv"
            with path.open("w", encoding="utf-8", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(["frame_index", "time_s"])

            with self.assertRaises(ValueError):
                ReplayData.load(path)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_replay_data -v`  
Expected: `ModuleNotFoundError` for `replay_data`

- [ ] **Step 3: Write minimal implementation**

```python
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


REQUIRED_COLUMNS = [
    "frame_index", "time_s",
    "t_raw_a", "t_raw_b",
    "m_raw_a", "m_raw_b",
    "final_a", "final_b",
    "target_a", "target_b",
    "output_a", "output_b",
]


@dataclass(slots=True)
class ReplayData:
    frame_index: np.ndarray
    time_s: np.ndarray
    values: dict[str, np.ndarray]

    @classmethod
    def load(cls, path: Path) -> "ReplayData":
        df = pd.read_csv(path)
        missing = [col for col in REQUIRED_COLUMNS if col not in df.columns]
        if missing:
            raise ValueError(f"missing columns: {missing}")
        values = {col: df[col].to_numpy() for col in REQUIRED_COLUMNS if col not in ("frame_index", "time_s")}
        return cls(
            frame_index=df["frame_index"].to_numpy(),
            time_s=df["time_s"].to_numpy(),
            values=values,
        )

    @property
    def row_count(self) -> int:
        return int(self.time_s.size)

    def latest_frame_at_time(self, t: float) -> dict[str, float]:
        idx = int(np.searchsorted(self.time_s, t, side="right") - 1)
        idx = max(0, min(idx, self.row_count - 1))
        result = {"frame_index": int(self.frame_index[idx]), "time_s": float(self.time_s[idx])}
        for key, values in self.values.items():
            result[key] = float(values[idx])
        return result
```

- [ ] **Step 4: Run test to verify it passes**

Run: `D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_replay_data -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add src/replay_data.py tests/test_replay_data.py
git commit -m "feat: add replay data loader"
```

### Task 3: Add Analytics Module

**Files:**
- Create: `src/analytics.py`
- Create: `tests/test_analytics.py`

- [ ] **Step 1: Write the failing tests**

```python
import unittest

import numpy as np

from analytics import compute_channel_metrics


class AnalyticsTests(unittest.TestCase):
    def test_compute_basic_and_error_metrics(self) -> None:
        time_s = np.array([0.0, 0.1, 0.2, 0.3])
        target = np.array([1.0, 1.0, 1.0, 1.0])
        final = np.array([0.5, 0.8, 1.0, 1.0])

        metrics = compute_channel_metrics(time_s, target, final)

        self.assertAlmostEqual(metrics["mean"], 0.825, places=3)
        self.assertAlmostEqual(metrics["max_abs_error"], 0.5, places=3)
        self.assertEqual(metrics["overshoot_pct"], 0.0)

    def test_return_none_like_values_when_samples_insufficient(self) -> None:
        time_s = np.array([0.0])
        target = np.array([1.0])
        final = np.array([1.0])

        metrics = compute_channel_metrics(time_s, target, final)

        self.assertIsNone(metrics["rise_time_s"])
        self.assertIsNone(metrics["settling_time_s"])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_analytics -v`  
Expected: `ModuleNotFoundError` for `analytics`

- [ ] **Step 3: Write minimal implementation**

```python
import numpy as np


def compute_channel_metrics(time_s: np.ndarray, target: np.ndarray, final: np.ndarray) -> dict[str, float | None]:
    if time_s.size == 0 or target.size == 0 or final.size == 0:
        return _empty_metrics()

    error = target - final
    metrics = {
        "mean": float(np.mean(final)),
        "std": float(np.std(final)),
        "min": float(np.min(final)),
        "max": float(np.max(final)),
        "peak_to_peak": float(np.max(final) - np.min(final)),
        "mean_error": float(np.mean(error)),
        "max_abs_error": float(np.max(np.abs(error))),
        "steady_state_error": _steady_state_error(error),
        "rise_time_s": None,
        "settling_time_s": None,
        "overshoot_pct": _overshoot_pct(target, final),
    }
    rise_time_s, settling_time_s = _step_response_metrics(time_s, target, final)
    metrics["rise_time_s"] = rise_time_s
    metrics["settling_time_s"] = settling_time_s
    return metrics


def _empty_metrics() -> dict[str, float | None]:
    return {
        "mean": None,
        "std": None,
        "min": None,
        "max": None,
        "peak_to_peak": None,
        "mean_error": None,
        "max_abs_error": None,
        "steady_state_error": None,
        "rise_time_s": None,
        "settling_time_s": None,
        "overshoot_pct": None,
    }


def _steady_state_error(error: np.ndarray) -> float | None:
    if error.size < 3:
        return None
    tail_size = max(1, int(np.ceil(error.size * 0.2)))
    return float(np.mean(error[-tail_size:]))


def _overshoot_pct(target: np.ndarray, final: np.ndarray) -> float | None:
    if target.size < 2:
        return None
    step_indices = np.flatnonzero(np.abs(np.diff(target)) > 1e-6)
    if step_indices.size == 0:
        return 0.0
    start = int(step_indices[-1] + 1)
    initial = float(target[start - 1])
    goal = float(target[start])
    amplitude = goal - initial
    if abs(amplitude) < 1e-6:
        return None
    response = final[start:]
    peak = float(np.max(response)) if amplitude > 0 else float(np.min(response))
    overshoot = max(0.0, peak - goal) if amplitude > 0 else max(0.0, goal - peak)
    return float((overshoot / abs(amplitude)) * 100.0)


def _step_response_metrics(time_s: np.ndarray, target: np.ndarray, final: np.ndarray) -> tuple[float | None, float | None]:
    if time_s.size < 3:
        return None, None
    step_indices = np.flatnonzero(np.abs(np.diff(target)) > 1e-6)
    if step_indices.size == 0:
        return None, None
    start = int(step_indices[-1] + 1)
    start_time = float(time_s[start])
    initial = float(target[start - 1])
    goal = float(target[start])
    amplitude = goal - initial
    if abs(amplitude) < 1e-6:
        return None, None
    low = initial + amplitude * 0.1
    high = initial + amplitude * 0.9
    response = final[start:]
    response_time = time_s[start:]
    low_hits = np.flatnonzero(response >= min(low, high)) if amplitude > 0 else np.flatnonzero(response <= max(low, high))
    high_hits = np.flatnonzero(response >= max(low, high)) if amplitude > 0 else np.flatnonzero(response <= min(low, high))
    rise_time = None
    if low_hits.size and high_hits.size:
        rise_time = float(response_time[high_hits[0]] - response_time[low_hits[0]])
    tolerance = abs(goal) * 0.05
    if tolerance < 1e-6:
        return rise_time, None
    band = np.abs(response - goal) <= tolerance
    settling_time = None
    for idx in range(band.size):
        if np.all(band[idx:]):
            settling_time = float(response_time[idx] - start_time)
            break
    return rise_time, settling_time
```

- [ ] **Step 4: Run test to verify it passes**

Run: `D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_analytics -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add src/analytics.py tests/test_analytics.py
git commit -m "feat: add live and replay analytics"
```

### Task 4: Extend Data Buffer for Streaming and Analytics Windows

**Files:**
- Modify: `src/data_buffer.py`
- Create: `tests/test_data_buffer.py`

- [ ] **Step 1: Write the failing tests**

```python
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from data_buffer import DataBuffer
from protocol import DataFrame
from recording_session import RecordingSession


def make_frame(index: int) -> DataFrame:
    return DataFrame(
        t_raw_A=0.0,
        t_raw_B=0.0,
        m_raw_A=0.0,
        m_raw_B=0.0,
        final_A=float(index),
        final_B=float(index) + 1.0,
        target_A=10.0,
        target_B=11.0,
        output_A=index,
        output_B=index + 1,
    )


class DataBufferTests(unittest.TestCase):
    def test_recent_window_returns_last_10_seconds(self) -> None:
        buffer = DataBuffer()
        for index in range(1500):
            buffer.append(make_frame(index))

        time_s, data = buffer.get_recent_window(10.0)

        self.assertEqual(len(time_s), 1000)
        self.assertAlmostEqual(time_s[0], 5.0, places=2)
        self.assertAlmostEqual(data[-1, 4], 1499.0, places=2)

    def test_append_writes_to_active_recording_session(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            session = RecordingSession(base_dir=Path(tmp))
            session.start()
            buffer = DataBuffer()
            buffer.start_recording(session)

            buffer.append(make_frame(1))

            self.assertEqual(buffer.csv_rows_written, 1)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_data_buffer -v`  
Expected: missing `get_recent_window` or `start_recording(session)` signature mismatch

- [ ] **Step 3: Write minimal implementation**

```python
# Update DataBuffer to:
# - remove in-memory _csv_buffer
# - store active recording session in self._recording_session
# - add get_recent_window(window_s: float)
# - add start_recording(session: RecordingSession)
# - add stop_recording() returning active session

def get_recent_window(self, window_s: float) -> tuple[np.ndarray, np.ndarray]:
    time_array, data = self.get_snapshot()
    if len(time_array) == 0:
        return time_array, data
    start_time = max(float(time_array[-1] - window_s), float(time_array[0]))
    start_idx = int(np.searchsorted(time_array, start_time, side="left"))
    return time_array[start_idx:], data[start_idx:]

def start_recording(self, session: RecordingSession) -> None:
    with self._lock:
        self._recording_session = session

def stop_recording(self) -> RecordingSession | None:
    with self._lock:
        session = self._recording_session
        self._recording_session = None
        return session

# In append():
if self._recording_session is not None:
    time_s = (self._frame_index - 1) * 0.01
    self._recording_session.write_frame(
        frame_index=self._frame_index - 1,
        time_s=time_s,
        frame=frame,
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_data_buffer -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add src/data_buffer.py tests/test_data_buffer.py
git commit -m "refactor: support streaming recording windows"
```

### Task 5: Add Analysis Panel Widget

**Files:**
- Create: `src/widgets/analysis_panel.py`
- Create: `tests/test_analysis_panel.py`

- [ ] **Step 1: Write the failing test**

```python
import os
import sys
import unittest

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from widgets.analysis_panel import AnalysisPanel


class AnalysisPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_update_metrics_renders_values(self) -> None:
        panel = AnalysisPanel()
        panel.update_metrics(
            mode_label="最近 10 秒",
            metrics_a={"mean": 1.23, "std": 0.1},
            metrics_b={"mean": 2.34, "std": 0.2},
        )

        self.assertIn("1.23", panel.metric_text("mean", "A"))
        self.assertIn("2.34", panel.metric_text("mean", "B"))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `set QT_QPA_PLATFORM=offscreen && D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_analysis_panel -v`  
Expected: `ModuleNotFoundError` for `widgets.analysis_panel`

- [ ] **Step 3: Write minimal implementation**

```python
from PySide6.QtWidgets import QFormLayout, QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget


class AnalysisPanel(QWidget):
    METRICS = [
        ("mean", "均值"),
        ("std", "标准差"),
        ("min", "最小值"),
        ("max", "最大值"),
        ("peak_to_peak", "峰峰值"),
        ("mean_error", "误差均值"),
        ("max_abs_error", "最大误差"),
        ("steady_state_error", "稳态误差"),
        ("rise_time_s", "上升时间"),
        ("settling_time_s", "调节时间"),
        ("overshoot_pct", "超调量"),
    ]

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._labels: dict[tuple[str, str], QLabel] = {}
        layout = QVBoxLayout(self)
        self._title = QLabel("统计分析")
        layout.addWidget(self._title)
        group = QGroupBox("电机指标")
        grid = QGridLayout()
        grid.addWidget(QLabel("指标"), 0, 0)
        grid.addWidget(QLabel("电机 A"), 0, 1)
        grid.addWidget(QLabel("电机 B"), 0, 2)
        for row, (key, text) in enumerate(self.METRICS, start=1):
            grid.addWidget(QLabel(text), row, 0)
            for motor, col in (("A", 1), ("B", 2)):
                label = QLabel("--")
                grid.addWidget(label, row, col)
                self._labels[(key, motor)] = label
        group.setLayout(grid)
        layout.addWidget(group)

    def update_metrics(self, mode_label: str, metrics_a: dict, metrics_b: dict) -> None:
        self._title.setText(f"统计分析 - {mode_label}")
        for key, _ in self.METRICS:
            self._labels[(key, "A")].setText(self._format(metrics_a.get(key)))
            self._labels[(key, "B")].setText(self._format(metrics_b.get(key)))

    def metric_text(self, key: str, motor: str) -> str:
        return self._labels[(key, motor)].text()

    @staticmethod
    def _format(value) -> str:
        if value is None:
            return "--"
        return f"{value:.3f}"
```

- [ ] **Step 4: Run test to verify it passes**

Run: `set QT_QPA_PLATFORM=offscreen && D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_analysis_panel -v`  
Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add src/widgets/analysis_panel.py tests/test_analysis_panel.py
git commit -m "feat: add analytics panel widget"
```

### Task 6: Add Replay Controls and Main Window Integration

**Files:**
- Modify: `src/main_window.py`
- Modify: `src/widgets/plot_panel.py`
- Modify: `src/widgets/data_panel.py`
- Modify: `src/main.py`

- [ ] **Step 1: Write the failing integration test**

```python
import os
import sys
import tempfile
import unittest
from pathlib import Path

from PySide6.QtWidgets import QApplication

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from main_window import MainWindow


class MainWindowReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_switching_to_replay_enables_replay_mode(self) -> None:
        window = MainWindow()
        window._set_replay_loaded_for_test(
            time_values=[0.0, 0.1],
            rows=[
                {"final_a": 1.0, "final_b": 2.0, "target_a": 1.0, "target_b": 2.0, "output_a": 10, "output_b": 20},
                {"final_a": 1.1, "final_b": 2.1, "target_a": 1.0, "target_b": 2.0, "output_a": 11, "output_b": 21},
            ],
        )
        window._set_data_mode_for_test("replay")

        self.assertEqual(window.current_data_mode(), "replay")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `set QT_QPA_PLATFORM=offscreen && D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_main_window_replay -v`  
Expected: missing replay test helpers or mode support

- [ ] **Step 3: Write minimal implementation**

```python
# Update MainWindow to:
# - create AnalysisPanel
# - replace old record toggle flow with RecordingSession start/finalize/cancel
# - add data source widgets (combo/radios, load button, play button, slider, speed combo)
# - track self._data_mode, self._replay_data, self._replay_current_time
# - use live buffer in live mode and replay slices in replay mode
# - compute analytics in _on_refresh using analytics.compute_channel_metrics

# Update PlotPanel/DataPanel to accept direct arrays/dicts instead of only DataBuffer:
# PlotPanel.refresh_series(time_arr, data)
# DataPanel.refresh_frame(frame_dict)

# Add small test-only helpers:
def current_data_mode(self) -> str:
    return self._data_mode

def _set_replay_loaded_for_test(self, time_values, rows) -> None:
    self._replay_data = ReplayData.from_rows(time_values, rows)
    self._replay_controls_enabled(True)

def _set_data_mode_for_test(self, mode: str) -> None:
    self._data_mode = mode
```

- [ ] **Step 4: Run the focused integration test**

Run: `set QT_QPA_PLATFORM=offscreen && D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest tests.test_main_window_replay -v`  
Expected: `OK`

- [ ] **Step 5: Run broader regression tests**

Run: `set QT_QPA_PLATFORM=offscreen && D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest discover -s tests -v`  
Expected: all tests pass

- [ ] **Step 6: Commit**

```bash
git add src/main_window.py src/widgets/plot_panel.py src/widgets/data_panel.py src/main.py tests/test_main_window_replay.py
git commit -m "feat: integrate replay and analytics into main window"
```

### Task 7: Update Docs and Manual Verification Notes

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Update usage docs**

```markdown
- 说明流式录制在开始时自动创建临时文件
- 说明停止录制后才命名保存
- 说明取消保存时临时文件会自动删除
- 新增 CSV 回放操作说明
- 新增统计分析面板说明
```

- [ ] **Step 2: Run tests after docs-affecting code is already green**

Run: `set QT_QPA_PLATFORM=offscreen && D:\radar\car\debug_monitor\.venv\Scripts\python.exe -m unittest discover -s tests -v`  
Expected: all tests pass

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: describe streaming recording and replay analytics"
```

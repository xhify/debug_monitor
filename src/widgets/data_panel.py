"""数值显示面板：展示当前最新一帧的数据，可用于实时或回放模式。"""

from __future__ import annotations

from collections import deque
from time import perf_counter

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

from data_buffer import DataBuffer


class DataPanel(QWidget):
    """数值显示面板。"""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._labels: dict[str, QLabel] = {}
        self._fps_samples: deque[tuple[float, int]] = deque()
        self._group: QGroupBox | None = None
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._group = QGroupBox("实时数据")
        grid = QGridLayout()

        grid.addWidget(QLabel("<b></b>"), 0, 0)
        grid.addWidget(QLabel("<b>电机 A</b>"), 0, 1, Qt.AlignCenter)
        grid.addWidget(QLabel("<b>电机 B</b>"), 0, 2, Qt.AlignCenter)

        rows = [
            ("T法原始速度", "t_raw_A", "t_raw_B"),
            ("M法原始速度", "m_raw_A", "m_raw_B"),
            ("融合反馈速度", "final_A", "final_B"),
            ("目标速度", "target_A", "target_B"),
            ("PWM 输出", "output_A", "output_B"),
        ]

        for row_idx, (label, key_a, key_b) in enumerate(rows, start=1):
            grid.addWidget(QLabel(label), row_idx, 0)

            lbl_a = QLabel("---")
            lbl_a.setAlignment(Qt.AlignCenter)
            lbl_a.setMinimumWidth(90)
            grid.addWidget(lbl_a, row_idx, 1)
            self._labels[key_a] = lbl_a

            lbl_b = QLabel("---")
            lbl_b.setAlignment(Qt.AlignCenter)
            lbl_b.setMinimumWidth(90)
            grid.addWidget(lbl_b, row_idx, 2)
            self._labels[key_b] = lbl_b

        grid.addWidget(QLabel("帧率/回放"), 6, 0)
        self._fps_label = QLabel("---")
        self._fps_label.setAlignment(Qt.AlignCenter)
        grid.addWidget(self._fps_label, 6, 1, 1, 2)

        self._group.setLayout(grid)
        layout.addWidget(self._group)

    def refresh(self, buffer: DataBuffer) -> None:
        frame = buffer.get_latest()
        if frame is None:
            return

        self.refresh_frame(
            {
                "t_raw_A": frame.t_raw_A,
                "t_raw_B": frame.t_raw_B,
                "m_raw_A": frame.m_raw_A,
                "m_raw_B": frame.m_raw_B,
                "final_A": frame.final_A,
                "final_B": frame.final_B,
                "target_A": frame.target_A,
                "target_B": frame.target_B,
                "output_A": frame.output_A,
                "output_B": frame.output_B,
            },
            title="实时数据",
            footer_text=self._live_fps_text(buffer.frame_index),
        )

    def refresh_frame(self, frame: dict[str, float] | None, title: str = "实时数据", footer_text: str = "---") -> None:
        self._group.setTitle(title)
        if frame is None:
            for label in self._labels.values():
                label.setText("---")
            self._fps_label.setText(footer_text)
            return

        for key in (
            "t_raw_A", "t_raw_B",
            "m_raw_A", "m_raw_B",
            "final_A", "final_B",
            "target_A", "target_B",
        ):
            self._labels[key].setText(f"{float(frame.get(key, 0.0)):.4f}")
        self._labels["output_A"].setText(str(int(frame.get("output_A", 0))))
        self._labels["output_B"].setText(str(int(frame.get("output_B", 0))))
        self._fps_label.setText(footer_text)

    def _live_fps_text(self, current_index: int) -> str:
        now = perf_counter()
        self._fps_samples.append((now, current_index))
        cutoff = now - 1.0
        while len(self._fps_samples) > 1 and self._fps_samples[0][0] < cutoff:
            self._fps_samples.popleft()

        if len(self._fps_samples) >= 2:
            start_time, start_index = self._fps_samples[0]
            elapsed = now - start_time
            if elapsed > 0:
                fps = (current_index - start_index) / elapsed
                return f"{fps:.0f} Hz"
        return "---"

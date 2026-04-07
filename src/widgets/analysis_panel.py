"""Statistics display panel for live and replay analysis."""

from __future__ import annotations

from PySide6.QtWidgets import QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget


class AnalysisPanel(QWidget):
    """Show per-motor analytics metrics."""

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
        self._title = QLabel("统计分析")
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
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
            self._labels[(key, "A")].setText(self._format(key, metrics_a.get(key)))
            self._labels[(key, "B")].setText(self._format(key, metrics_b.get(key)))

    def metric_text(self, key: str, motor: str) -> str:
        return self._labels[(key, motor)].text()

    @staticmethod
    def _format(key: str, value: float | None) -> str:
        if value is None:
            return "--"
        suffix = "%" if key == "overshoot_pct" else ""
        return f"{value:.3f}{suffix}"

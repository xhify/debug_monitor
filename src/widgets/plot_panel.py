"""实时绘图面板：速度图（8条曲线）+ PWM/AFC 图（4条曲线），垂直堆叠，X轴联动。"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QCheckBox, QHBoxLayout, QSplitter, QVBoxLayout, QWidget
import pyqtgraph as pg

from data_buffer import (
    COL_AFC_A, COL_AFC_B, COL_FINAL_A, COL_FINAL_B, COL_M_RAW_A, COL_M_RAW_B,
    COL_OUT_A, COL_OUT_B, COL_TGT_A, COL_TGT_B, COL_T_RAW_A, COL_T_RAW_B,
)

REPLAY_WINDOW_S = 10.0


class PlotPanel(QWidget):
    """实时/回放绘图面板，包含速度图和 PWM 图。"""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._paused = False
        self._curves: dict[str, pg.PlotDataItem] = {}
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        splitter = QSplitter(Qt.Vertical)

        self._speed_plot = pg.PlotWidget(title="速度 (m/s)")
        self._speed_plot.setBackground("w")
        self._speed_plot.showGrid(x=True, y=True, alpha=0.3)
        self._speed_plot.setLabel("left", "速度", units="m/s")
        self._speed_plot.setLabel("bottom", "时间", units="s")
        self._speed_plot.addLegend(offset=(10, 10))
        self._apply_perf_opts(self._speed_plot)

        self._curves["t_raw_A"] = self._speed_plot.plot(pen=pg.mkPen("#c0392b", width=1), name="T法A")
        self._curves["t_raw_B"] = self._speed_plot.plot(pen=pg.mkPen("#2980b9", width=1), name="T法B")
        self._curves["m_raw_A"] = self._speed_plot.plot(pen=pg.mkPen("#e67e22", width=1, style=Qt.DashLine), name="M法A")
        self._curves["m_raw_B"] = self._speed_plot.plot(pen=pg.mkPen("#00acc1", width=1, style=Qt.DashLine), name="M法B")
        self._curves["final_A"] = self._speed_plot.plot(pen=pg.mkPen("#f1c40f", width=2), name="融合A")
        self._curves["final_B"] = self._speed_plot.plot(pen=pg.mkPen("#1abc9c", width=2), name="融合B")
        self._curves["tgt_A"] = self._speed_plot.plot(pen=pg.mkPen("#2ecc71", width=1.5, style=Qt.DashLine), name="目标A")
        self._curves["tgt_B"] = self._speed_plot.plot(pen=pg.mkPen("#9b59b6", width=1.5, style=Qt.DotLine), name="目标B")

        splitter.addWidget(self._speed_plot)

        self._pwm_plot = pg.PlotWidget(title="PWM 输出")
        self._pwm_plot.setBackground("w")
        self._pwm_plot.showGrid(x=True, y=True, alpha=0.3)
        self._pwm_plot.setLabel("left", "PWM")
        self._pwm_plot.setLabel("bottom", "时间", units="s")
        self._pwm_plot.addLegend(offset=(10, 10))
        self._pwm_plot.setXLink(self._speed_plot)
        self._apply_perf_opts(self._pwm_plot)

        self._curves["out_A"] = self._pwm_plot.plot(pen=pg.mkPen("#ff7043", width=2), name="PWM A")
        self._curves["out_B"] = self._pwm_plot.plot(pen=pg.mkPen("#7e57c2", width=2), name="PWM B")
        self._curves["afc_A"] = self._pwm_plot.plot(
            pen=pg.mkPen("#8e44ad", width=1.5, style=Qt.DashLine), name="AFC A"
        )
        self._curves["afc_B"] = self._pwm_plot.plot(
            pen=pg.mkPen("#16a085", width=1.5, style=Qt.DashLine), name="AFC B"
        )

        splitter.addWidget(self._pwm_plot)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)
        layout.addWidget(splitter, stretch=1)

        self._setup_legend_toggle(self._speed_plot)
        self._setup_legend_toggle(self._pwm_plot)

        ctrl_layout = QHBoxLayout()
        self._pause_cb = QCheckBox("暂停绘图")
        self._pause_cb.toggled.connect(self._on_pause_toggled)
        ctrl_layout.addWidget(self._pause_cb)
        ctrl_layout.addStretch()
        layout.addLayout(ctrl_layout)

    def _setup_legend_toggle(self, plot_widget: pg.PlotWidget) -> None:
        legend = plot_widget.getPlotItem().legend
        for sample, label in legend.items:
            curve = sample.item
            sample.setCursor(Qt.PointingHandCursor)
            label.setCursor(Qt.PointingHandCursor)
            sample.mouseClickEvent = self._make_toggle_handler(sample, label, curve)
            label.mouseClickEvent = self._make_toggle_handler(sample, label, curve)

    def _make_toggle_handler(self, sample, label, curve):
        def handler(event):
            vis = not curve.isVisible()
            curve.setVisible(vis)
            sample.setOpacity(1.0 if vis else 0.3)
            label.setOpacity(1.0 if vis else 0.3)

        return handler

    @staticmethod
    def _apply_perf_opts(plot_widget: pg.PlotWidget) -> None:
        plot_item = plot_widget.getPlotItem()
        plot_item.setDownsampling(mode="peak")
        plot_item.setClipToView(True)

    def refresh_series(self, time_arr, data) -> None:
        if self._paused:
            return
        if len(time_arr) == 0:
            self.reset()
            return

        self._curves["t_raw_A"].setData(time_arr, data[:, COL_T_RAW_A])
        self._curves["t_raw_B"].setData(time_arr, data[:, COL_T_RAW_B])
        self._curves["m_raw_A"].setData(time_arr, data[:, COL_M_RAW_A])
        self._curves["m_raw_B"].setData(time_arr, data[:, COL_M_RAW_B])
        self._curves["final_A"].setData(time_arr, data[:, COL_FINAL_A])
        self._curves["final_B"].setData(time_arr, data[:, COL_FINAL_B])
        self._curves["tgt_A"].setData(time_arr, data[:, COL_TGT_A])
        self._curves["tgt_B"].setData(time_arr, data[:, COL_TGT_B])
        self._curves["out_A"].setData(time_arr, data[:, COL_OUT_A])
        self._curves["out_B"].setData(time_arr, data[:, COL_OUT_B])
        self._curves["afc_A"].setData(time_arr, data[:, COL_AFC_A])
        self._curves["afc_B"].setData(time_arr, data[:, COL_AFC_B])

    def follow_time_cursor(self, current_time: float) -> None:
        """回放模式固定显示最近 10 秒窗口。"""
        end = max(REPLAY_WINDOW_S, float(current_time))
        start = max(0.0, end - REPLAY_WINDOW_S)
        self._speed_plot.setXRange(start, end, padding=0.0)

    def reset(self) -> None:
        for curve in self._curves.values():
            curve.setData([], [])
        self._speed_plot.setXRange(0, 1)
        self._speed_plot.setYRange(-1, 1)
        self._pwm_plot.setXRange(0, 1)
        self._pwm_plot.setYRange(-18000, 18000)
        self._speed_plot.enableAutoRange()
        self._pwm_plot.enableAutoRange()

    @property
    def paused(self) -> bool:
        return self._paused

    def _on_pause_toggled(self, checked: bool) -> None:
        self._paused = checked

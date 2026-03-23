"""
实时绘图面板：速度图（6条曲线）+ PWM图（2条曲线），垂直堆叠，X轴联动。

性能优化：
- setDownsampling(mode='peak')：缩放到全局时自动降采样
- setClipToView(True)：仅渲染可见区域的数据点
"""

from PySide6.QtWidgets import QWidget, QVBoxLayout, QCheckBox, QHBoxLayout, QSplitter
from PySide6.QtCore import Qt
import pyqtgraph as pg
import numpy as np

from data_buffer import DataBuffer, NUM_COLS
from data_buffer import (
    COL_RAW_A, COL_RAW_B, COL_FILT_A, COL_FILT_B,
    COL_TGT_A, COL_TGT_B, COL_OUT_A, COL_OUT_B,
)


class PlotPanel(QWidget):
    """实时绘图面板，包含速度图和 PWM 图。"""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._paused = False
        self._curves: dict[str, pg.PlotDataItem] = {}
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # ── 绘图区（垂直分割）──────────────────────────
        splitter = QSplitter(Qt.Vertical)

        # 速度图
        self._speed_plot = pg.PlotWidget(title="速度 (m/s)")
        self._speed_plot.setBackground('w')
        self._speed_plot.showGrid(x=True, y=True, alpha=0.3)
        self._speed_plot.setLabel('left', '速度', units='m/s')
        self._speed_plot.setLabel('bottom', '时间', units='s')
        self._speed_plot.addLegend(offset=(10, 10))
        self._apply_perf_opts(self._speed_plot)

        # 速度曲线：原始(实线细) / 滤波(实线粗) / 目标(虚线/点线，防遮挡)
        # A=暖色 B=冷色，不同色相对比，对红绿色弱友好
        self._curves['raw_A'] = self._speed_plot.plot(
            pen=pg.mkPen('#e74c3c', width=1), name='原始A')
        self._curves['raw_B'] = self._speed_plot.plot(
            pen=pg.mkPen('#3498db', width=1), name='原始B')
        self._curves['filt_A'] = self._speed_plot.plot(
            pen=pg.mkPen('#e67e22', width=2), name='滤波A')
        self._curves['filt_B'] = self._speed_plot.plot(
            pen=pg.mkPen('#00bcd4', width=2), name='滤波B')
        self._curves['tgt_A'] = self._speed_plot.plot(
            pen=pg.mkPen('#2ecc71', width=1.5, style=Qt.DashLine), name='目标A')
        self._curves['tgt_B'] = self._speed_plot.plot(
            pen=pg.mkPen('#9b59b6', width=1.5, style=Qt.DotLine), name='目标B')

        splitter.addWidget(self._speed_plot)

        # PWM 图
        self._pwm_plot = pg.PlotWidget(title="PWM 输出")
        self._pwm_plot.setBackground('w')
        self._pwm_plot.showGrid(x=True, y=True, alpha=0.3)
        self._pwm_plot.setLabel('left', 'PWM')
        self._pwm_plot.setLabel('bottom', '时间', units='s')
        self._pwm_plot.addLegend(offset=(10, 10))
        self._pwm_plot.setXLink(self._speed_plot)  # X轴联动
        self._apply_perf_opts(self._pwm_plot)

        self._curves['out_A'] = self._pwm_plot.plot(
            pen=pg.mkPen('#ff7043', width=2), name='PWM A')
        self._curves['out_B'] = self._pwm_plot.plot(
            pen=pg.mkPen('#7e57c2', width=2), name='PWM B')

        splitter.addWidget(self._pwm_plot)

        # 速度图占更大比例
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter, stretch=1)

        # 图例点击切换曲线显隐
        self._setup_legend_toggle(self._speed_plot)
        self._setup_legend_toggle(self._pwm_plot)

        # ── 控制栏 ─────────────────────────────────────
        ctrl_layout = QHBoxLayout()
        self._pause_cb = QCheckBox("暂停绘图")
        self._pause_cb.toggled.connect(self._on_pause_toggled)
        ctrl_layout.addWidget(self._pause_cb)
        ctrl_layout.addStretch()
        layout.addLayout(ctrl_layout)

    def _setup_legend_toggle(self, plot_widget: pg.PlotWidget) -> None:
        """为图例条目添加点击切换曲线显隐功能。"""
        legend = plot_widget.getPlotItem().legend
        for sample, label in legend.items:
            curve = sample.item  # pyqtgraph ItemSample 存储了关联的 PlotDataItem
            sample.setCursor(Qt.PointingHandCursor)
            label.setCursor(Qt.PointingHandCursor)
            # 工厂函数创建独立闭包，规避循环变量迟绑定
            sample.mouseClickEvent = self._make_toggle_handler(sample, label, curve)
            label.mouseClickEvent = self._make_toggle_handler(sample, label, curve)

    def _make_toggle_handler(self, sample, label, curve):
        """为单个图例条目创建独立的点击回调。"""
        def handler(event):
            vis = not curve.isVisible()
            curve.setVisible(vis)
            sample.setOpacity(1.0 if vis else 0.3)
            label.setOpacity(1.0 if vis else 0.3)
        return handler

    @staticmethod
    def _apply_perf_opts(plot_widget: pg.PlotWidget) -> None:
        """应用 pyqtgraph 性能优化参数。"""
        plot_item = plot_widget.getPlotItem()
        plot_item.setDownsampling(mode='peak')
        plot_item.setClipToView(True)

    def refresh(self, buffer: DataBuffer) -> None:
        """从 DataBuffer 读取快照并更新曲线。由主窗口定时器调用。"""
        if self._paused:
            return

        time_arr, data = buffer.get_snapshot()
        if len(time_arr) == 0:
            return

        # 速度曲线
        self._curves['raw_A'].setData(time_arr, data[:, COL_RAW_A])
        self._curves['raw_B'].setData(time_arr, data[:, COL_RAW_B])
        self._curves['filt_A'].setData(time_arr, data[:, COL_FILT_A])
        self._curves['filt_B'].setData(time_arr, data[:, COL_FILT_B])
        self._curves['tgt_A'].setData(time_arr, data[:, COL_TGT_A])
        self._curves['tgt_B'].setData(time_arr, data[:, COL_TGT_B])

        # PWM 曲线
        self._curves['out_A'].setData(time_arr, data[:, COL_OUT_A])
        self._curves['out_B'].setData(time_arr, data[:, COL_OUT_B])

    def reset(self) -> None:
        """清空曲线数据并重置坐标轴到初始范围。"""
        for curve in self._curves.values():
            curve.setData([], [])
        # 空数据时 enableAutoRange 无法推算范围，需显式设定初始值
        self._speed_plot.setXRange(0, 1)
        self._speed_plot.setYRange(-1, 1)
        self._pwm_plot.setXRange(0, 1)
        self._pwm_plot.setYRange(-100, 100)
        # 启用 autoRange，后续有数据到达时自动跟随
        self._speed_plot.enableAutoRange()
        self._pwm_plot.enableAutoRange()

    @property
    def paused(self) -> bool:
        return self._paused

    def _on_pause_toggled(self, checked: bool) -> None:
        self._paused = checked

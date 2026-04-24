"""ROS dual-IMU monitor panel for /imu and /active_imu."""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)
import numpy as np
import pyqtgraph as pg

from ros_bridge_worker import RosImuReading, RosSnapshot
from ros_data import RosDualImuTimeSeriesBuffer


ROS_IMU_DISPLAY_NAMES = {
    "imu": "IMU",
    "active_imu": "活动 IMU",
}

_PLOT_LAYOUT: tuple[tuple[str, str, str], ...] = (
    ("Acc X", "m/s²", "accel_x"),
    ("Acc Y", "m/s²", "accel_y"),
    ("Acc Z", "m/s²", "accel_z"),
    ("Gyro X", "rad/s", "gyro_x"),
    ("Gyro Y", "rad/s", "gyro_y"),
    ("Gyro Z", "rad/s", "gyro_z"),
    ("Roll", "deg", "roll_deg"),
    ("Pitch", "deg", "pitch_deg"),
    ("Yaw", "deg", "yaw_deg"),
)

_DEVICE_STYLES: tuple[tuple[str, Qt.PenStyle, str], ...] = (
    ("imu", Qt.SolidLine, "#2c3e50"),
    ("active_imu", Qt.DashLine, "#e67e22"),
)


class RosImuPanel(QWidget):
    """Display two ROS sensor_msgs/Imu topics side by side."""

    _FOLLOW_WINDOW_SECONDS = 10.0

    connect_requested = Signal(str, int)
    disconnect_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._labels: dict[str, dict[str, QLabel]] = {
            "imu": {},
            "active_imu": {},
        }
        self._curves: dict[str, pg.PlotDataItem] = {}
        self._raw_curves: dict[str, pg.PlotDataItem] = {}
        self._buffer = RosDualImuTimeSeriesBuffer()
        self._paused = False
        self._last_imu_frame_counts = (0, 0)
        self._pending_snapshot: RosSnapshot | None = None
        self._plot_dirty = False
        self._setup_ui()
        self._setup_refresh_timer()
        self.set_connected(False)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        layout.addWidget(self._build_connection_group())

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_plot_widget())
        splitter.addWidget(self._build_value_widget())
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        layout.addWidget(splitter, stretch=1)

        control_row = QHBoxLayout()
        self._clear_btn = QPushButton("清空数据")
        self._clear_btn.clicked.connect(self._clear_data)
        control_row.addWidget(self._clear_btn)

        self._pause_cb = QCheckBox("暂停绘图")
        self._pause_cb.toggled.connect(self._on_pause_toggled)
        control_row.addWidget(self._pause_cb)

        self._show_raw_cb = QCheckBox("显示原始数据")
        self._show_raw_cb.toggled.connect(self._on_display_option_changed)
        control_row.addWidget(self._show_raw_cb)

        control_row.addWidget(QLabel("平滑:"))
        self._smoothing_window_combo = QComboBox()
        self._smoothing_window_combo.addItem("关", 0.0)
        self._smoothing_window_combo.addItem("50 ms", 0.05)
        self._smoothing_window_combo.addItem("100 ms", 0.1)
        self._smoothing_window_combo.addItem("200 ms", 0.2)
        self._smoothing_window_combo.addItem("500 ms", 0.5)
        self._smoothing_window_combo.setCurrentIndex(3)
        self._smoothing_window_combo.currentIndexChanged.connect(self._on_display_option_changed)
        control_row.addWidget(self._smoothing_window_combo)
        control_row.addStretch()
        layout.addLayout(control_row)

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("rosbridge 连接")
        row = QHBoxLayout(group)

        row.addWidget(QLabel("主机:"))
        self._host_edit = QLineEdit("192.168.0.14")
        self._host_edit.setMinimumWidth(160)
        row.addWidget(self._host_edit)

        row.addWidget(QLabel("端口:"))
        self._port_spin = QSpinBox()
        self._port_spin.setRange(1, 65535)
        self._port_spin.setValue(9090)
        row.addWidget(self._port_spin)

        self._connect_btn = QPushButton("连接")
        self._connect_btn.clicked.connect(self._on_connect)
        row.addWidget(self._connect_btn)

        self._disconnect_btn = QPushButton("断开")
        self._disconnect_btn.clicked.connect(self.disconnect_requested.emit)
        row.addWidget(self._disconnect_btn)

        self._status_label = QLabel("未连接")
        row.addWidget(self._status_label)
        row.addStretch()
        return group

    def _build_plot_widget(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        layout.addWidget(self._build_legend_strip())

        grid_host = QWidget()
        grid = QGridLayout(grid_host)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(4)

        self._plots: dict[str, pg.PlotWidget] = {}
        for idx, (title, unit, data_key) in enumerate(_PLOT_LAYOUT):
            row, col = divmod(idx, 3)
            plot = self._make_small_plot(title, unit, show_x_label=(row == 2))
            grid.addWidget(plot, row, col)
            self._plots[data_key] = plot

        for row in range(3):
            grid.setRowStretch(row, 1)
        for col in range(3):
            grid.setColumnStretch(col, 1)

        self._acc_plot = self._plots["accel_x"]
        for data_key, plot in self._plots.items():
            if plot is not self._acc_plot:
                plot.setXLink(self._acc_plot)

        for data_key, plot in self._plots.items():
            for device_key, style, color in _DEVICE_STYLES:
                curve_key = f"{device_key}_{data_key}"
                self._raw_curves[curve_key] = plot.plot(
                    pen=pg.mkPen(color, width=0.7, style=style),
                )
                self._raw_curves[curve_key].setVisible(False)
                self._curves[f"{device_key}_{data_key}"] = plot.plot(
                    pen=pg.mkPen(color, width=1.8, style=style),
                )

        layout.addWidget(grid_host, stretch=1)
        return widget

    def _build_legend_strip(self) -> QWidget:
        strip = QWidget()
        row = QHBoxLayout(strip)
        row.setContentsMargins(4, 0, 4, 0)
        row.setSpacing(16)
        row.addStretch()
        for device_key, _style, color in _DEVICE_STYLES:
            marker = QLabel()
            marker.setFixedSize(28, 2)
            dashed = device_key == "active_imu"
            border = "dashed" if dashed else "solid"
            marker.setStyleSheet(
                f"background-color: transparent; border-top: 2px {border} {color};"
            )
            row.addWidget(marker, alignment=Qt.AlignVCenter)
            text = QLabel(ROS_IMU_DISPLAY_NAMES[device_key])
            text.setStyleSheet(f"color: {color};")
            row.addWidget(text)
        row.addStretch()
        return strip

    def _make_small_plot(self, title: str, unit: str, show_x_label: bool) -> pg.PlotWidget:
        plot = pg.PlotWidget()
        plot.setBackground("w")
        plot.setTitle(
            f"<span style='color:#2c3e50; font-size:9pt'>{title} ({unit})</span>"
        )
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.getPlotItem().getAxis("left").setWidth(38)
        if show_x_label:
            plot.setLabel("bottom", "时间", units="s")
        plot.getPlotItem().setDownsampling(mode="peak")
        plot.getPlotItem().setClipToView(True)
        return plot

    def _build_value_widget(self) -> QGroupBox:
        group = QGroupBox("ROS IMU 当前数据")
        grid = QGridLayout(group)
        value_rows = [
            ("帧数", "frame_count", "int"),
            ("Acc X", "accel_x", "float4"),
            ("Acc Y", "accel_y", "float4"),
            ("Acc Z", "accel_z", "float4"),
            ("Gyro X", "gyro_x", "float4"),
            ("Gyro Y", "gyro_y", "float4"),
            ("Gyro Z", "gyro_z", "float4"),
            ("Roll", "roll_deg", "angle"),
            ("Pitch", "pitch_deg", "angle"),
            ("Yaw", "yaw_deg", "angle"),
        ]

        grid.addWidget(QLabel("<b>字段</b>"), 0, 0)
        for col, device_key in enumerate(("imu", "active_imu"), start=1):
            title_label = QLabel(ROS_IMU_DISPLAY_NAMES[device_key])
            title_label.setAlignment(Qt.AlignCenter)
            grid.addWidget(title_label, 0, col)
            self._labels[device_key]["title"] = title_label

        for row, (title, key, _fmt) in enumerate(value_rows, start=1):
            grid.addWidget(QLabel(title), row, 0)
            for col, device_key in enumerate(("imu", "active_imu"), start=1):
                label = QLabel("---")
                label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                label.setMinimumWidth(86)
                grid.addWidget(label, row, col)
                self._labels[device_key][key] = label
        return group

    def set_connected(self, connected: bool) -> None:
        self._connect_btn.setEnabled(not connected)
        self._disconnect_btn.setEnabled(connected)
        self._host_edit.setEnabled(not connected)
        self._port_spin.setEnabled(not connected)
        self._status_label.setText("已连接" if connected else "未连接")
        self._status_label.setStyleSheet("color: green;" if connected else "color: red;")

    def update_snapshot(self, snapshot: RosSnapshot) -> None:
        current_counts = (snapshot.imu.frame_count, snapshot.active_imu.frame_count)
        imu_changed = current_counts != self._last_imu_frame_counts and any(
            count > 0 for count in current_counts
        )
        self._last_imu_frame_counts = current_counts
        if imu_changed:
            self._buffer.append(snapshot)
            self._plot_dirty = True
        self._pending_snapshot = snapshot

    def shutdown(self) -> None:
        self._refresh_timer.stop()
        self._clear_data()

    def _setup_refresh_timer(self) -> None:
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._flush_pending_snapshot)
        self._refresh_timer.start(50)

    def _flush_pending_snapshot(self) -> None:
        snapshot = self._pending_snapshot
        if snapshot is None:
            return
        self._refresh_values("imu", snapshot.imu)
        self._refresh_values("active_imu", snapshot.active_imu)
        if self._plot_dirty and not self._paused:
            self._refresh_plot()
            self._plot_dirty = False

    def _on_connect(self) -> None:
        host = self._host_edit.text().strip()
        if not host:
            self._status_label.setText("请输入主机")
            return
        self.connect_requested.emit(host, self._port_spin.value())

    def _refresh_values(self, device_key: str, reading: RosImuReading) -> None:
        labels = self._labels[device_key]
        labels["frame_count"].setText(str(reading.frame_count))
        for key in ("accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"):
            labels[key].setText(f"{getattr(reading, key):.4f}")
        for key in ("roll_deg", "pitch_deg", "yaw_deg"):
            labels[key].setText(f"{getattr(reading, key):.2f}")

    def _refresh_plot(self) -> None:
        time_arr, data = self._buffer.snapshot()
        if len(time_arr) == 0:
            for curve in [*self._curves.values(), *self._raw_curves.values()]:
                curve.setData([], [])
            return
        smoothing_window_s = float(self._smoothing_window_combo.currentData() or 0.0)
        show_raw = self._show_raw_cb.isChecked()
        for key, curve in self._curves.items():
            raw_values = data[key]
            curve.setData(time_arr, self._smooth_series(time_arr, raw_values, smoothing_window_s))
            raw_curve = self._raw_curves[key]
            raw_curve.setData(time_arr, raw_values)
            raw_curve.setVisible(show_raw)
        self._follow_latest_time(time_arr)

    @staticmethod
    def _smooth_series(time_arr: np.ndarray, values: np.ndarray, window_s: float) -> np.ndarray:
        if window_s <= 0.0 or values.size < 2:
            return values

        smoothed = np.empty_like(values, dtype=np.float64)
        cumulative = np.concatenate(([0.0], np.cumsum(values, dtype=np.float64)))
        start_indices = np.searchsorted(time_arr, time_arr - window_s - 1e-12, side="left")
        end_indices = np.arange(1, values.size + 1)
        counts = end_indices - start_indices
        smoothed[:] = (cumulative[end_indices] - cumulative[start_indices]) / counts
        return smoothed

    def _follow_latest_time(self, time_arr) -> None:
        latest_time = float(time_arr[-1])
        x_max = max(self._FOLLOW_WINDOW_SECONDS, latest_time)
        x_min = max(0.0, x_max - self._FOLLOW_WINDOW_SECONDS)
        self._acc_plot.setXRange(x_min, x_max, padding=0.0)

    def _clear_data(self) -> None:
        self._buffer.clear()
        self._pending_snapshot = None
        self._plot_dirty = False
        self._last_imu_frame_counts = (0, 0)
        self._refresh_plot()
        for device_key in ("imu", "active_imu"):
            labels = self._labels[device_key]
            for key, label in labels.items():
                if key != "title":
                    label.setText("---")

    def _on_pause_toggled(self, checked: bool) -> None:
        self._paused = checked

    def _on_display_option_changed(self) -> None:
        self._plot_dirty = True
        if not self._paused:
            self._refresh_plot()
            self._plot_dirty = False

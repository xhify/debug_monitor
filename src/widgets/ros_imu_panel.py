"""ROS dual-IMU monitor panel for /imu and /active_imu."""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
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
import pyqtgraph as pg

from ros_bridge_worker import RosImuReading, RosSnapshot
from ros_data import RosDualImuTimeSeriesBuffer


ROS_IMU_DISPLAY_NAMES = {
    "imu": "IMU",
    "active_imu": "活动 IMU",
}


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

        splitter = QSplitter(Qt.Vertical)
        self._acc_plot = self._make_plot("加速度", "m/s^2")
        self._gyro_plot = self._make_plot("角速度", "rad/s")
        self._euler_plot = self._make_plot("姿态角", "deg")
        self._gyro_plot.setXLink(self._acc_plot)
        self._euler_plot.setXLink(self._acc_plot)

        for device_key, style in (("imu", Qt.SolidLine), ("active_imu", Qt.DashLine)):
            self._add_device_curves(device_key, style)

        splitter.addWidget(self._acc_plot)
        splitter.addWidget(self._gyro_plot)
        splitter.addWidget(self._euler_plot)
        layout.addWidget(splitter, stretch=1)
        return widget

    def _make_plot(self, title: str, unit: str) -> pg.PlotWidget:
        plot = pg.PlotWidget(title=title)
        plot.setBackground("w")
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.setLabel("left", title, units=unit)
        plot.setLabel("bottom", "时间", units="s")
        plot.addLegend(offset=(10, 10))
        plot.getPlotItem().setDownsampling(mode="peak")
        plot.getPlotItem().setClipToView(True)
        return plot

    def _add_device_curves(self, device_key: str, style: Qt.PenStyle) -> None:
        name = ROS_IMU_DISPLAY_NAMES[device_key]
        palette = {
            "x": "#c0392b" if device_key == "imu" else "#ff7043",
            "y": "#2980b9" if device_key == "imu" else "#5dade2",
            "z": "#27ae60" if device_key == "imu" else "#58d68d",
        }
        for axis in ("x", "y", "z"):
            self._curves[f"{device_key}_accel_{axis}"] = self._acc_plot.plot(
                pen=pg.mkPen(palette[axis], width=1.4, style=style),
                name=f"{name} Acc {axis.upper()}",
            )
            self._curves[f"{device_key}_gyro_{axis}"] = self._gyro_plot.plot(
                pen=pg.mkPen(palette[axis], width=1.4, style=style),
                name=f"{name} Gyro {axis.upper()}",
            )

        self._curves[f"{device_key}_roll_deg"] = self._euler_plot.plot(
            pen=pg.mkPen("#34495e" if device_key == "imu" else "#7f8c8d", width=1.4, style=style),
            name=f"{name} Roll",
        )
        self._curves[f"{device_key}_pitch_deg"] = self._euler_plot.plot(
            pen=pg.mkPen("#e67e22" if device_key == "imu" else "#f5b041", width=1.4, style=style),
            name=f"{name} Pitch",
        )
        self._curves[f"{device_key}_yaw_deg"] = self._euler_plot.plot(
            pen=pg.mkPen("#2c7fb8" if device_key == "imu" else "#85c1e9", width=1.4, style=style),
            name=f"{name} Yaw",
        )

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
            for curve in self._curves.values():
                curve.setData([], [])
            return
        for key, curve in self._curves.items():
            curve.setData(time_arr, data[key])
        self._follow_latest_time(time_arr)

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

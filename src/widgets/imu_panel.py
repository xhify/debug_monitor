"""Independent dual-IMU acquisition module."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from imu_buffer import (
    COL_ACC_X,
    COL_ACC_Y,
    COL_ACC_Z,
    COL_EULER_PITCH,
    COL_EULER_ROLL,
    COL_EULER_YAW,
    COL_GYRO_X,
    COL_GYRO_Y,
    COL_GYRO_Z,
    ImuBuffer,
)
from imu_protocol import ImuSample
from imu_recording import ImuSessionRecorder
from imu_serial_worker import COMMON_IMU_BAUDS, ImuSerialWorker


@dataclass(slots=True)
class ImuDeviceUi:
    key: str
    buffer: ImuBuffer
    worker: ImuSerialWorker
    group: QGroupBox
    port_combo: QComboBox
    refresh_btn: QPushButton
    baud_combo: QComboBox
    connect_btn: QPushButton
    disconnect_btn: QPushButton
    status_label: QLabel
    labels: dict[str, QLabel]


class ImuPanel(QWidget):
    """IMU module page with two independent serial inputs and one session recorder."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._devices: dict[str, ImuDeviceUi] = {}
        self._curves: dict[str, pg.PlotDataItem] = {}
        self._session_recorder: ImuSessionRecorder | None = None
        self._paused = False

        self._setup_ui()
        self._setup_timers()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        device_layout = QVBoxLayout()
        device_layout.setSpacing(4)
        device_layout.addWidget(self._build_device_group("A"))
        device_layout.addWidget(self._build_device_group("B"))
        layout.addLayout(device_layout)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_plot_widget())
        splitter.addWidget(self._build_value_widget())
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        layout.addWidget(splitter, stretch=1)

        layout.addLayout(self._build_control_row())

    def _build_device_group(self, device_key: str) -> QGroupBox:
        group = QGroupBox(f"IMU {device_key}")
        row = QHBoxLayout(group)

        row.addWidget(QLabel("端口:"))
        port_combo = QComboBox()
        port_combo.setEditable(True)
        port_combo.setMinimumWidth(170)
        row.addWidget(port_combo)

        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(lambda: self._refresh_ports(device_key))
        row.addWidget(refresh_btn)

        row.addWidget(QLabel("波特率:"))
        baud_combo = QComboBox()
        for baud in COMMON_IMU_BAUDS:
            baud_combo.addItem(str(baud), baud)
        row.addWidget(baud_combo)

        connect_btn = QPushButton("连接")
        connect_btn.clicked.connect(lambda: self._on_connect(device_key))
        row.addWidget(connect_btn)

        disconnect_btn = QPushButton("断开")
        disconnect_btn.setEnabled(False)
        disconnect_btn.clicked.connect(lambda: self._on_disconnect(device_key))
        row.addWidget(disconnect_btn)

        status_label = QLabel("未连接")
        status_label.setStyleSheet("color: red;")
        row.addWidget(status_label)
        row.addStretch()

        buffer = ImuBuffer()
        worker = ImuSerialWorker(buffer)
        worker.connection_changed.connect(
            lambda connected, key=device_key: self._on_connection_changed(key, connected)
        )
        worker.error_occurred.connect(
            lambda message, key=device_key: self._on_error(key, message)
        )
        worker.sample_received.connect(
            lambda sample, key=device_key: self._on_sample(key, sample)
        )

        self._devices[device_key] = ImuDeviceUi(
            key=device_key,
            buffer=buffer,
            worker=worker,
            group=group,
            port_combo=port_combo,
            refresh_btn=refresh_btn,
            baud_combo=baud_combo,
            connect_btn=connect_btn,
            disconnect_btn=disconnect_btn,
            status_label=status_label,
            labels={},
        )
        self._refresh_ports(device_key)
        return group

    def _build_plot_widget(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        splitter = QSplitter(Qt.Vertical)
        self._acc_plot = self._make_plot("加速度", "m/s^2")
        self._gyro_plot = self._make_plot("角速度", "deg/s")
        self._euler_plot = self._make_plot("欧拉角", "deg")
        self._gyro_plot.setXLink(self._acc_plot)
        self._euler_plot.setXLink(self._acc_plot)

        for device_key, style in (("A", Qt.SolidLine), ("B", Qt.DashLine)):
            self._add_device_curves(device_key, style)

        splitter.addWidget(self._acc_plot)
        splitter.addWidget(self._gyro_plot)
        splitter.addWidget(self._euler_plot)
        layout.addWidget(splitter, stretch=1)
        return widget

    def _add_device_curves(self, device_key: str, style: Qt.PenStyle) -> None:
        palette = {
            "x": "#c0392b" if device_key == "A" else "#ff7043",
            "y": "#2980b9" if device_key == "A" else "#5dade2",
            "z": "#27ae60" if device_key == "A" else "#58d68d",
        }
        self._curves[f"{device_key}_acc_x"] = self._acc_plot.plot(
            pen=pg.mkPen(palette["x"], width=1.4, style=style), name=f"{device_key} Acc X"
        )
        self._curves[f"{device_key}_acc_y"] = self._acc_plot.plot(
            pen=pg.mkPen(palette["y"], width=1.4, style=style), name=f"{device_key} Acc Y"
        )
        self._curves[f"{device_key}_acc_z"] = self._acc_plot.plot(
            pen=pg.mkPen(palette["z"], width=1.4, style=style), name=f"{device_key} Acc Z"
        )
        self._curves[f"{device_key}_gyro_x"] = self._gyro_plot.plot(
            pen=pg.mkPen(palette["x"], width=1.4, style=style), name=f"{device_key} Gyro X"
        )
        self._curves[f"{device_key}_gyro_y"] = self._gyro_plot.plot(
            pen=pg.mkPen(palette["y"], width=1.4, style=style), name=f"{device_key} Gyro Y"
        )
        self._curves[f"{device_key}_gyro_z"] = self._gyro_plot.plot(
            pen=pg.mkPen(palette["z"], width=1.4, style=style), name=f"{device_key} Gyro Z"
        )
        self._curves[f"{device_key}_pitch"] = self._euler_plot.plot(
            pen=pg.mkPen("#34495e" if device_key == "A" else "#7f8c8d", width=1.4, style=style),
            name=f"{device_key} Pitch",
        )
        self._curves[f"{device_key}_roll"] = self._euler_plot.plot(
            pen=pg.mkPen("#e67e22" if device_key == "A" else "#f5b041", width=1.4, style=style),
            name=f"{device_key} Roll",
        )
        self._curves[f"{device_key}_yaw"] = self._euler_plot.plot(
            pen=pg.mkPen("#2c7fb8" if device_key == "A" else "#85c1e9", width=1.4, style=style),
            name=f"{device_key} Yaw",
        )

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

    def _build_value_widget(self) -> QWidget:
        group = QGroupBox("IMU当前数据")
        grid = QGridLayout(group)
        value_rows = [
            ("序号", "sequence"),
            ("温度 ℃", "temperature_c"),
            ("设备时间 us", "device_time"),
            ("同步时间 us", "sync_time"),
            ("Acc X", "acc_x"),
            ("Acc Y", "acc_y"),
            ("Acc Z", "acc_z"),
            ("Gyro X", "gyro_x"),
            ("Gyro Y", "gyro_y"),
            ("Gyro Z", "gyro_z"),
            ("Pitch", "pitch"),
            ("Roll", "roll"),
            ("Yaw", "yaw"),
            ("帧数", "frame_count"),
        ]
        grid.addWidget(QLabel("<b>字段</b>"), 0, 0)
        grid.addWidget(QLabel("<b>IMU A</b>"), 0, 1, Qt.AlignCenter)
        grid.addWidget(QLabel("<b>IMU B</b>"), 0, 2, Qt.AlignCenter)
        for row, (title, key) in enumerate(value_rows, start=1):
            grid.addWidget(QLabel(title), row, 0)
            for col, device_key in enumerate(("A", "B"), start=1):
                label = QLabel("---")
                label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                label.setMinimumWidth(86)
                grid.addWidget(label, row, col)
                self._devices[device_key].labels[key] = label
        return group

    def _build_control_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self._clear_btn = QPushButton("清空数据")
        self._clear_btn.clicked.connect(self._clear_data)
        row.addWidget(self._clear_btn)

        self._record_btn = QPushButton("开始记录")
        self._record_btn.clicked.connect(self._toggle_record)
        row.addWidget(self._record_btn)

        self._pause_cb = QCheckBox("暂停绘图")
        self._pause_cb.toggled.connect(self._on_pause_toggled)
        row.addWidget(self._pause_cb)

        self._session_status_label = QLabel("")
        row.addWidget(self._session_status_label)
        row.addStretch()
        return row

    def _setup_timers(self) -> None:
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._on_refresh)
        self._refresh_timer.start(33)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(1000)

    def _refresh_ports(self, device_key: str) -> None:
        device = self._devices[device_key]
        current_text = device.port_combo.currentText()
        device.port_combo.clear()
        for port, description in ImuSerialWorker.list_ports():
            device.port_combo.addItem(f"{port} - {description}", port)
        if current_text and device.port_combo.findText(current_text) < 0:
            device.port_combo.setEditText(current_text)

    def _on_connect(self, device_key: str) -> None:
        device = self._devices[device_key]
        port = device.port_combo.currentData()
        if not port:
            text = device.port_combo.currentText().strip()
            port = text.split(" - ")[0] if " - " in text else text
        if not port:
            device.status_label.setText("请选择串口")
            return
        device.buffer.clear()
        device.worker.open_port(port, int(device.baud_combo.currentData()))

    def _on_disconnect(self, device_key: str) -> None:
        device = self._devices[device_key]
        if self._session_recorder is not None:
            self._stop_recording(save=False)
        device.worker.close_port()

    def _on_connection_changed(self, device_key: str, connected: bool) -> None:
        device = self._devices[device_key]
        device.connect_btn.setEnabled(not connected)
        device.disconnect_btn.setEnabled(connected)
        device.port_combo.setEnabled(not connected)
        device.baud_combo.setEnabled(not connected)
        device.refresh_btn.setEnabled(not connected)
        if connected:
            port = device.port_combo.currentData() or device.port_combo.currentText()
            baud = device.baud_combo.currentText()
            device.status_label.setText(f"已连接: {port} @ {baud}")
            device.status_label.setStyleSheet("color: green;")
        else:
            device.status_label.setText("未连接")
            device.status_label.setStyleSheet("color: red;")

    def _on_error(self, device_key: str, message: str) -> None:
        device = self._devices[device_key]
        device.status_label.setText(f"错误: {message}")
        device.status_label.setStyleSheet("color: red;")

    def _on_sample(self, device_key: str, sample: ImuSample) -> None:
        if self._session_recorder is not None:
            self._session_recorder.write_sample(device_key, sample)

    def _on_refresh(self) -> None:
        if not self._paused:
            for device_key, device in self._devices.items():
                time_arr, data = device.buffer.get_snapshot()
                self._refresh_device_plots(device_key, time_arr, data)
        for device_key, device in self._devices.items():
            self._refresh_values(device_key, device.buffer.get_latest())
        self._update_status()

    def _refresh_device_plots(self, device_key: str, time_arr, data) -> None:
        if len(time_arr) == 0:
            for suffix in ("acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "pitch", "roll", "yaw"):
                self._curves[f"{device_key}_{suffix}"].setData([], [])
            return

        display_time = time_arr - time_arr[0]
        self._curves[f"{device_key}_acc_x"].setData(display_time, data[:, COL_ACC_X])
        self._curves[f"{device_key}_acc_y"].setData(display_time, data[:, COL_ACC_Y])
        self._curves[f"{device_key}_acc_z"].setData(display_time, data[:, COL_ACC_Z])
        self._curves[f"{device_key}_gyro_x"].setData(display_time, data[:, COL_GYRO_X])
        self._curves[f"{device_key}_gyro_y"].setData(display_time, data[:, COL_GYRO_Y])
        self._curves[f"{device_key}_gyro_z"].setData(display_time, data[:, COL_GYRO_Z])
        self._curves[f"{device_key}_pitch"].setData(display_time, data[:, COL_EULER_PITCH])
        self._curves[f"{device_key}_roll"].setData(display_time, data[:, COL_EULER_ROLL])
        self._curves[f"{device_key}_yaw"].setData(display_time, data[:, COL_EULER_YAW])

    def _refresh_values(self, device_key: str, sample: ImuSample | None) -> None:
        labels = self._devices[device_key].labels
        if sample is None:
            for label in labels.values():
                label.setText("---")
            return

        labels["sequence"].setText(str(sample.sequence))
        labels["temperature_c"].setText(_fmt_optional(sample.temperature_c))
        labels["device_time"].setText(_fmt_int_optional(sample.device_time))
        labels["sync_time"].setText(_fmt_int_optional(sample.sync_time))
        _set_vec_labels(labels, ("acc_x", "acc_y", "acc_z"), sample.accel)
        _set_vec_labels(labels, ("gyro_x", "gyro_y", "gyro_z"), sample.gyro)
        _set_vec_labels(labels, ("pitch", "roll", "yaw"), sample.euler)

    def _update_status(self) -> None:
        for device in self._devices.values():
            if "frame_count" in device.labels:
                device.labels["frame_count"].setText(str(device.buffer.frame_index))
        if self._session_recorder is not None:
            rows = self._session_recorder.rows_written_by_device
            self._record_btn.setText(f"停止记录 (A:{rows['A']} B:{rows['B']})")

    def _clear_data(self) -> None:
        if self._session_recorder is not None:
            self._stop_recording(save=False)
        for device in self._devices.values():
            device.buffer.clear()

    def _toggle_record(self) -> None:
        if self._session_recorder is None:
            self._start_recording()
            return
        self._stop_recording(save=True)

    def _start_recording(self) -> None:
        recorder = ImuSessionRecorder(base_dir=Path.cwd() / "recordings")
        session_dir = recorder.start(self._device_configs())
        self._activate_recorder(recorder, session_dir)

    def start_recording_in_directory(self, session_dir: Path, started_at: str, note: str = "") -> None:
        recorder = ImuSessionRecorder(base_dir=session_dir.parent)
        recorder.start_in_directory(
            session_dir,
            self._device_configs(),
            started_at=started_at,
            metadata_filename="imu_session.json",
            merged_filename="imu_merged_aligned.csv",
            note=note,
        )
        self._activate_recorder(recorder, session_dir)

    def _activate_recorder(self, recorder: ImuSessionRecorder, session_dir: Path) -> None:
        self._session_recorder = recorder
        self._record_btn.setText("停止记录 (A:0 B:0)")
        self._record_btn.setStyleSheet("background-color: #e74c3c; color: white;")
        self._session_status_label.setText(f"记录中: {session_dir.name}")

    def _device_configs(self) -> dict[str, dict[str, object]]:
        configs: dict[str, dict[str, object]] = {}
        for device_key, device in self._devices.items():
            port = device.port_combo.currentData() or device.port_combo.currentText()
            configs[device_key] = {
                "port": str(port),
                "baudrate": int(device.baud_combo.currentData()),
            }
        return configs

    def stop_recording(self, save: bool) -> Path | None:
        return self._stop_recording(save=save)

    def _stop_recording(self, save: bool) -> Path | None:
        recorder = self._session_recorder
        self._session_recorder = None
        self._record_btn.setText("开始记录")
        self._record_btn.setStyleSheet("")
        if recorder is None:
            return None
        if save:
            session_dir = recorder.finalize()
            self._session_status_label.setText(f"已保存: {session_dir}")
            return session_dir
        else:
            recorder.cancel()
            self._session_status_label.setText("记录已取消")
            return None

    def is_recording(self) -> bool:
        return self._session_recorder is not None

    def recording_rows_text(self) -> str:
        if self._session_recorder is None:
            return ""
        rows = self._session_recorder.rows_written_by_device
        return f"A:{rows['A']} B:{rows['B']}"

    def _on_pause_toggled(self, checked: bool) -> None:
        self._paused = checked

    def shutdown(self) -> None:
        if self._session_recorder is not None:
            self._stop_recording(save=False)
        self._refresh_timer.stop()
        self._status_timer.stop()
        for device in self._devices.values():
            device.worker.close_port()


def _fmt_optional(value: float | None) -> str:
    return "---" if value is None else f"{value:.4f}"


def _fmt_int_optional(value: int | None) -> str:
    return "---" if value is None else str(value)


def _set_vec_labels(
    labels: dict[str, QLabel],
    keys: tuple[str, ...],
    values: tuple[float, ...] | None,
) -> None:
    for index, key in enumerate(keys):
        labels[key].setText("---" if values is None else f"{values[index]:.4f}")

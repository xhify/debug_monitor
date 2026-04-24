"""ROS bridge panel for standard topic monitoring and /cmd_vel publishing."""

from __future__ import annotations

import tempfile
from pathlib import Path

from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ros_bridge_worker import RosSnapshot
from ros_data import RosCsvRecordingSession, RosTimeSeriesBuffer


class RosPanel(QWidget):
    """ROS monitor and command panel backed by rosbridge websocket."""

    connect_requested = Signal(str, int)
    disconnect_requested = Signal()
    cmd_vel_requested = Signal(float, float)
    pid_control_requested = Signal(float, bool, bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._labels: dict[str, QLabel] = {}
        self._buffer = RosTimeSeriesBuffer()
        self._recording_session: RosCsvRecordingSession | None = None
        self._pending_snapshot: RosSnapshot | None = None
        self._setup_ui()
        self._setup_refresh_timer()
        self.set_connected(False)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        connection_group = QGroupBox("rosbridge 连接")
        connection_layout = QHBoxLayout(connection_group)
        connection_layout.addWidget(QLabel("主机:"))
        self._host_edit = QLineEdit("192.168.0.14")
        self._host_edit.setMinimumWidth(160)
        connection_layout.addWidget(self._host_edit)

        connection_layout.addWidget(QLabel("端口:"))
        self._port_spin = QSpinBox()
        self._port_spin.setRange(1, 65535)
        self._port_spin.setValue(9090)
        connection_layout.addWidget(self._port_spin)

        self._connect_btn = QPushButton("连接")
        self._connect_btn.clicked.connect(self._on_connect)
        connection_layout.addWidget(self._connect_btn)

        self._disconnect_btn = QPushButton("断开")
        self._disconnect_btn.clicked.connect(self.disconnect_requested.emit)
        connection_layout.addWidget(self._disconnect_btn)

        self._status_label = QLabel("未连接")
        connection_layout.addWidget(self._status_label)
        connection_layout.addStretch()
        layout.addWidget(connection_group)

        content_row = QHBoxLayout()
        content_row.setSpacing(8)

        self._data_group = QGroupBox("ROS 数据")
        data_group = self._data_group
        data_grid = QGridLayout(data_group)
        rows = [
            ("帧数", "frame_count", ""),
            ("线速度 X", "linear_x", "m/s"),
            ("线速度 Y", "linear_y", "m/s"),
            ("角速度 Z", "angular_z", "rad/s"),
            ("位置 X", "pose_x", "m"),
            ("位置 Y", "pose_y", "m"),
            ("加速度 Z", "accel_z", "m/s^2"),
            ("角速度 IMU Z", "gyro_z", "rad/s"),
            ("电压", "voltage", "V"),
        ]
        for row, (label, key, unit) in enumerate(rows):
            data_grid.addWidget(QLabel(label), row, 0)
            value_label = QLabel("---")
            value_label.setMinimumWidth(100)
            data_grid.addWidget(value_label, row, 1)
            data_grid.addWidget(QLabel(unit), row, 2)
            self._labels[key] = value_label
        data_grid.setColumnStretch(3, 1)
        content_row.addWidget(data_group, stretch=2)

        control_column = QVBoxLayout()
        control_column.setSpacing(8)

        self._cmd_group = QGroupBox("/cmd_vel")
        cmd_group = self._cmd_group
        cmd_form = QFormLayout(cmd_group)
        self._linear_x_spin = QDoubleSpinBox()
        self._linear_x_spin.setRange(-3.0, 3.0)
        self._linear_x_spin.setDecimals(3)
        self._linear_x_spin.setSingleStep(0.05)
        cmd_form.addRow("linear.x:", self._linear_x_spin)

        self._angular_z_spin = QDoubleSpinBox()
        self._angular_z_spin.setRange(-6.0, 6.0)
        self._angular_z_spin.setDecimals(3)
        self._angular_z_spin.setSingleStep(0.05)
        cmd_form.addRow("angular.z:", self._angular_z_spin)

        button_row = QHBoxLayout()
        self._send_cmd_vel_btn = QPushButton("发送")
        self._send_cmd_vel_btn.clicked.connect(self._on_send_cmd_vel)
        button_row.addWidget(self._send_cmd_vel_btn)
        self._stop_cmd_vel_btn = QPushButton("停止")
        self._stop_cmd_vel_btn.clicked.connect(lambda: self.cmd_vel_requested.emit(0.0, 0.0))
        button_row.addWidget(self._stop_cmd_vel_btn)
        button_row.addStretch()
        cmd_form.addRow(button_row)
        control_column.addWidget(cmd_group)

        self._pid_group = QGroupBox("PID 直行控制 /line_follow_control")
        pid_group = self._pid_group
        pid_form = QFormLayout(pid_group)
        self._pid_linear_x_spin = QDoubleSpinBox()
        self._pid_linear_x_spin.setRange(0.0, 3.0)
        self._pid_linear_x_spin.setDecimals(3)
        self._pid_linear_x_spin.setSingleStep(0.05)
        self._pid_linear_x_spin.setValue(0.2)
        pid_form.addRow("linear.x:", self._pid_linear_x_spin)

        pid_button_row = QHBoxLayout()
        self._pid_forward_btn = QPushButton("PID 前进")
        self._pid_forward_btn.clicked.connect(
            lambda: self.pid_control_requested.emit(self._pid_linear_x_spin.value(), True, False)
        )
        pid_button_row.addWidget(self._pid_forward_btn)

        self._pid_backward_btn = QPushButton("PID 后退")
        self._pid_backward_btn.clicked.connect(
            lambda: self.pid_control_requested.emit(self._pid_linear_x_spin.value(), False, True)
        )
        pid_button_row.addWidget(self._pid_backward_btn)

        self._pid_stop_btn = QPushButton("PID 停止")
        self._pid_stop_btn.clicked.connect(lambda: self.pid_control_requested.emit(0.0, False, False))
        pid_button_row.addWidget(self._pid_stop_btn)
        pid_button_row.addStretch()
        pid_form.addRow(pid_button_row)
        control_column.addWidget(pid_group)
        control_column.addStretch()
        content_row.addLayout(control_column, stretch=3)
        layout.addLayout(content_row, stretch=1)

        record_group = QGroupBox("ROS CSV 记录")
        record_layout = QHBoxLayout(record_group)
        self._record_btn = QPushButton("开始记录")
        self._record_btn.clicked.connect(self._toggle_recording)
        record_layout.addWidget(self._record_btn)
        self._record_status_label = QLabel("")
        record_layout.addWidget(self._record_status_label, stretch=1)
        layout.addWidget(record_group)
        layout.addStretch()

    def set_connected(self, connected: bool) -> None:
        self._connect_btn.setEnabled(not connected)
        self._disconnect_btn.setEnabled(connected)
        self._host_edit.setEnabled(not connected)
        self._port_spin.setEnabled(not connected)
        self._send_cmd_vel_btn.setEnabled(connected)
        self._stop_cmd_vel_btn.setEnabled(connected)
        self._pid_forward_btn.setEnabled(connected)
        self._pid_backward_btn.setEnabled(connected)
        self._pid_stop_btn.setEnabled(connected)
        self._status_label.setText("已连接" if connected else "未连接")
        self._status_label.setStyleSheet("color: green;" if connected else "color: red;")

    def update_snapshot(self, snapshot: RosSnapshot) -> None:
        time_s = self._buffer.append(snapshot)
        self._pending_snapshot = snapshot
        if self._recording_session is not None:
            self._recording_session.write_snapshot(time_s=time_s, snapshot=snapshot)

    def start_recording_for_test(self, base_dir: Path) -> None:
        self._start_recording(base_dir=base_dir)

    def stop_recording_for_test(self, final_path: Path) -> None:
        self._stop_recording(final_path=final_path)

    def shutdown(self) -> None:
        self._refresh_timer.stop()
        if self._recording_session is not None:
            self._cancel_recording()

    def is_recording(self) -> bool:
        return self._recording_session is not None

    def _setup_refresh_timer(self) -> None:
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._flush_pending_snapshot)
        self._refresh_timer.start(50)

    def _flush_pending_snapshot(self) -> None:
        snapshot = self._pending_snapshot
        if snapshot is None:
            return
        self._labels["frame_count"].setText(str(snapshot.frame_count))
        for key in ("linear_x", "linear_y", "angular_z", "pose_x", "pose_y", "accel_z", "gyro_z"):
            self._labels[key].setText(f"{getattr(snapshot, key):.4f}")
        self._labels["voltage"].setText(f"{snapshot.voltage:.2f}")
        if self._recording_session is not None:
            self._record_status_label.setText(f"记录中: {self._recording_session.rows_written} 行")

    def _on_connect(self) -> None:
        host = self._host_edit.text().strip()
        if not host:
            self._status_label.setText("请输入主机")
            return
        self.connect_requested.emit(host, self._port_spin.value())

    def _on_send_cmd_vel(self) -> None:
        self.cmd_vel_requested.emit(self._linear_x_spin.value(), self._angular_z_spin.value())

    def _toggle_recording(self) -> None:
        if self._recording_session is None:
            self._start_recording(base_dir=Path(tempfile.gettempdir()))
            return
        default_name = f"ros_data_{Path.cwd().name}.csv"
        filepath, _ = QFileDialog.getSaveFileName(self, "保存 ROS 记录数据", default_name, "CSV 文件 (*.csv)")
        if filepath:
            self._stop_recording(final_path=Path(filepath))
        else:
            self._cancel_recording()

    def _start_recording(self, base_dir: Path) -> None:
        session = RosCsvRecordingSession(base_dir=base_dir)
        session.start()
        self._recording_session = session
        self._record_btn.setText("停止记录")
        self._record_btn.setStyleSheet("background-color: #e74c3c; color: white;")
        self._record_status_label.setText("记录中: 0 行")

    def _stop_recording(self, final_path: Path) -> None:
        session = self._recording_session
        self._recording_session = None
        self._record_btn.setText("开始记录")
        self._record_btn.setStyleSheet("")
        if session is None:
            return
        session.finalize(final_path)
        self._record_status_label.setText(f"已保存: {final_path}")

    def _cancel_recording(self) -> None:
        session = self._recording_session
        self._recording_session = None
        self._record_btn.setText("开始记录")
        self._record_btn.setStyleSheet("")
        if session is not None:
            rows = session.rows_written
            session.cancel()
            self._record_status_label.setText(f"记录已丢弃（{rows} 行）")

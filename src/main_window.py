"""
主窗口：布局编排、信号连接、定时器管理。
"""

from __future__ import annotations

import tempfile
import json
from datetime import datetime
from pathlib import Path
from time import perf_counter

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QStackedWidget,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from analytics import compute_channel_metrics
from data_buffer import COL_FINAL_A, COL_FINAL_B, COL_TGT_A, COL_TGT_B, DataBuffer
from recording_session import RecordingSession
from radar_scpi import RadarScpiClient
from replay_data import ReplayData
from ros_bridge_worker import RosBridgeWorker
from ros_data import RosSummaryRecordingSession
from serial_worker import SerialWorker
from widgets.analysis_panel import AnalysisPanel
from widgets.command_panel import CommandPanel
from widgets.data_panel import DataPanel
from widgets.imu_panel import ImuPanel
from widgets.param_panel import ParamPanel
from widgets.plot_panel import PlotPanel
from widgets.ros_imu_panel import RosImuPanel
from widgets.ros_panel import RosPanel
from widgets.serial_panel import SerialPanel


class MainWindow(QMainWindow):
    """调试监视器主窗口。"""

    _DEFAULT_WINDOW_WIDTH = 1400
    _DEFAULT_WINDOW_HEIGHT = 760

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("WHEELTEC C50X 调试监视器")
        self._apply_initial_window_size()

        self._buffer = DataBuffer()
        self._worker = SerialWorker(self._buffer)
        self._worker.param_received.connect(self._on_param)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._ros_worker = RosBridgeWorker()
        self._ros_worker.snapshot_received.connect(self._on_ros_snapshot)
        self._ros_worker.error_occurred.connect(self._on_error)
        self._ros_worker.connection_changed.connect(self._on_ros_connection_changed)

        self._data_mode = "live"
        self._replay_data: ReplayData | None = None
        self._replay_current_time = 0.0
        self._replay_speed = 1.0
        self._initial_screen_fit_done = False
        self._summary_rows: dict[str, dict[str, object]] = {}
        self._summary_last_counts: dict[str, tuple[float, int]] = {}
        self._summary_encoder_session: RecordingSession | None = None
        self._summary_ros_session: RosSummaryRecordingSession | None = None
        self._summary_session_dir: Path | None = None
        self._summary_radar_recording = False
        self._summary_radar_filename = ""
        self._latest_ros_snapshot = None
        self._ros_connected = False
        self._radar_client = RadarScpiClient()

        self._setup_ui()
        self._setup_timers()
        self._center_on_screen()

    def _apply_initial_window_size(self) -> None:
        screen = self.screen() or QGuiApplication.primaryScreen()
        if screen is None:
            self.resize(self._DEFAULT_WINDOW_WIDTH, self._DEFAULT_WINDOW_HEIGHT)
            return

        available = screen.availableGeometry()
        self.resize(
            min(self._DEFAULT_WINDOW_WIDTH, available.width()),
            min(self._DEFAULT_WINDOW_HEIGHT, available.height()),
        )

    def _center_on_screen(self) -> None:
        screen = self.screen() or QGuiApplication.primaryScreen()
        if screen is None:
            return

        frame_geometry = self.frameGeometry()
        frame_geometry.moveCenter(screen.availableGeometry().center())
        self.move(frame_geometry.topLeft())

    def _setup_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(4, 4, 4, 4)
        root_layout.setSpacing(4)

        self._setup_module_switcher(root_layout)

        self._module_stack = QStackedWidget()
        root_layout.addWidget(self._module_stack, stretch=1)

        self._encoder_page = QWidget()
        main_layout = QVBoxLayout(self._encoder_page)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(4)

        self._serial_panel = SerialPanel()
        self._serial_panel.connect_requested.connect(self._on_connect)
        self._serial_panel.disconnect_requested.connect(self._on_disconnect)
        main_layout.addWidget(self._serial_panel)

        mid_splitter = QSplitter(Qt.Horizontal)

        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)

        self._plot_panel = PlotPanel()
        left_layout.addWidget(self._plot_panel, stretch=1)

        control_row = QHBoxLayout()
        self._clear_btn = QPushButton("清空数据")
        self._clear_btn.clicked.connect(self._clear_data)
        control_row.addWidget(self._clear_btn)

        self._record_btn = QPushButton("开始记录")
        self._record_btn.clicked.connect(self._toggle_record)
        control_row.addWidget(self._record_btn)

        self._mode_combo = QComboBox()
        self._mode_combo.addItem("实时", "live")
        self._mode_combo.addItem("回放", "replay")
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        control_row.addWidget(self._mode_combo)

        self._load_btn = QPushButton("加载 CSV")
        self._load_btn.clicked.connect(self._load_replay_csv)
        control_row.addWidget(self._load_btn)

        self._play_btn = QPushButton("播放")
        self._play_btn.clicked.connect(self._toggle_replay_playback)
        control_row.addWidget(self._play_btn)

        self._progress_slider = QSlider(Qt.Horizontal)
        self._progress_slider.setRange(0, 1000)
        self._progress_slider.sliderMoved.connect(self._on_replay_slider_changed)
        control_row.addWidget(self._progress_slider, stretch=1)

        self._speed_combo = QComboBox()
        self._speed_combo.addItem("0.5x", 0.5)
        self._speed_combo.addItem("1x", 1.0)
        self._speed_combo.addItem("2x", 2.0)
        self._speed_combo.setCurrentIndex(1)
        self._speed_combo.currentIndexChanged.connect(self._on_speed_changed)
        control_row.addWidget(self._speed_combo)

        left_layout.addLayout(control_row)
        mid_splitter.addWidget(left_widget)

        self._right_sidebar = QWidget()
        right_layout = QVBoxLayout(self._right_sidebar)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(6)

        self._command_panel = CommandPanel()
        self._command_panel.command_ready.connect(self._worker.send_command)
        right_layout.addWidget(self._command_panel)

        self._data_panel = DataPanel()
        self._param_panel = ParamPanel()
        self._analysis_panel = AnalysisPanel()
        self._monitor_tabs = QTabWidget()
        self._monitor_tabs.addTab(self._param_panel, "固件参数")
        self._monitor_tabs.addTab(self._data_panel, "当前数据")
        self._monitor_tabs.addTab(self._analysis_panel, "统计分析")
        self._monitor_tabs.setCurrentWidget(self._param_panel)
        right_layout.addWidget(self._monitor_tabs, stretch=1)

        self._right_sidebar_scroll = QScrollArea()
        self._right_sidebar_scroll.setWidgetResizable(True)
        self._right_sidebar_scroll.setFrameShape(QScrollArea.NoFrame)
        self._right_sidebar_scroll.setWidget(self._right_sidebar)

        mid_splitter.addWidget(self._right_sidebar_scroll)
        mid_splitter.setStretchFactor(0, 7)
        mid_splitter.setStretchFactor(1, 3)
        main_layout.addWidget(mid_splitter, stretch=1)

        self._imu_panel = ImuPanel()
        self._ros_panel = RosPanel()
        self._ros_imu_panel = RosImuPanel()
        self._ros_panel.connect_requested.connect(self._ros_worker.open_bridge)
        self._ros_panel.disconnect_requested.connect(self._ros_worker.close_bridge)
        self._ros_panel.cmd_vel_requested.connect(self._ros_worker.publish_cmd_vel)
        self._ros_panel.pid_control_requested.connect(self._ros_worker.publish_line_follow_control)
        self._ros_imu_panel.connect_requested.connect(self._ros_worker.open_bridge)
        self._ros_imu_panel.disconnect_requested.connect(self._ros_worker.close_bridge)
        self._summary_page = self._build_summary_page()
        self._module_stack.addWidget(self._summary_page)
        self._module_stack.addWidget(self._encoder_page)
        self._module_stack.addWidget(self._imu_panel)
        self._module_stack.addWidget(self._ros_panel)
        self._module_stack.addWidget(self._ros_imu_panel)
        self._module_stack.setCurrentWidget(self._encoder_page)

        self._status_label = QLabel("就绪")
        self.statusBar().addWidget(self._status_label, stretch=1)
        self._frame_label = QLabel("帧: 0")
        self.statusBar().addPermanentWidget(self._frame_label)
        self._error_label = QLabel("错误: 0")
        self.statusBar().addPermanentWidget(self._error_label)
        self._record_label = QLabel("")
        self.statusBar().addPermanentWidget(self._record_label)

        self._replay_controls_enabled(False)

    def _setup_module_switcher(self, root_layout: QVBoxLayout) -> None:
        switch_row = QHBoxLayout()
        switch_row.setContentsMargins(0, 0, 0, 0)

        self._module_button_group = QButtonGroup(self)
        self._module_button_group.setExclusive(True)

        self._summary_module_btn = QPushButton("汇总")
        self._summary_module_btn.setCheckable(True)
        self._summary_module_btn.clicked.connect(lambda: self._switch_module("summary"))
        self._module_button_group.addButton(self._summary_module_btn)
        switch_row.addWidget(self._summary_module_btn)

        self._encoder_module_btn = QPushButton("编码器")
        self._encoder_module_btn.setCheckable(True)
        self._encoder_module_btn.setChecked(True)
        self._encoder_module_btn.clicked.connect(lambda: self._switch_module("encoder"))
        self._module_button_group.addButton(self._encoder_module_btn)
        switch_row.addWidget(self._encoder_module_btn)

        self._imu_module_btn = QPushButton("IMU")
        self._imu_module_btn.setCheckable(True)
        self._imu_module_btn.clicked.connect(lambda: self._switch_module("imu"))
        self._module_button_group.addButton(self._imu_module_btn)
        switch_row.addWidget(self._imu_module_btn)

        self._ros_module_btn = QPushButton("ROS")
        self._ros_module_btn.setCheckable(True)
        self._ros_module_btn.clicked.connect(lambda: self._switch_module("ros"))
        self._module_button_group.addButton(self._ros_module_btn)
        switch_row.addWidget(self._ros_module_btn)

        self._ros_imu_module_btn = QPushButton("ROS IMU")
        self._ros_imu_module_btn.setCheckable(True)
        self._ros_imu_module_btn.clicked.connect(lambda: self._switch_module("ros_imu"))
        self._module_button_group.addButton(self._ros_imu_module_btn)
        switch_row.addWidget(self._ros_imu_module_btn)

        switch_row.addStretch()
        root_layout.addLayout(switch_row)

    def _switch_module(self, module: str) -> None:
        if module == "summary":
            self._module_stack.setCurrentWidget(self._summary_page)
            self._status_label.setText("汇总模块")
            return
        if module == "imu":
            self._module_stack.setCurrentWidget(self._imu_panel)
            self._status_label.setText("IMU 模块")
            return
        if module == "ros":
            self._module_stack.setCurrentWidget(self._ros_panel)
            self._status_label.setText("ROS 模块")
            return
        if module == "ros_imu":
            self._module_stack.setCurrentWidget(self._ros_imu_panel)
            self._status_label.setText("ROS IMU 模块")
            return
        self._module_stack.setCurrentWidget(self._encoder_page)
        self._status_label.setText("编码器模块")

    def _build_summary_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        layout.addWidget(self._build_summary_device_group("encoder", "编码器", [115200, 9600, 19200, 38400, 57600, 230400, 460800]))
        layout.addWidget(self._build_summary_device_group("imu_A", "IMU A", [460800, 230400, 115200, 57600, 38400, 19200, 9600, 921600]))
        layout.addWidget(self._build_summary_device_group("imu_B", "IMU B", [460800, 230400, 115200, 57600, 38400, 19200, 9600, 921600]))

        record_group = QGroupBox("同步记录")
        record_layout = QVBoxLayout(record_group)
        note_row = QHBoxLayout()
        note_row.addWidget(QLabel("备注:"))
        self._summary_note_edit = QPlainTextEdit()
        self._summary_note_edit.setPlaceholderText("说明本次记录内容，例如路面、速度、控制参数、实验目的")
        self._summary_note_edit.setFixedHeight(72)
        note_row.addWidget(self._summary_note_edit, stretch=1)
        record_layout.addLayout(note_row)

        radar_row = QHBoxLayout()
        self._summary_radar_test_btn = QPushButton("测试雷达连接")
        self._summary_radar_test_btn.clicked.connect(self._test_summary_radar_connection)
        radar_row.addWidget(self._summary_radar_test_btn)

        self._summary_radar_sync_cb = QCheckBox("同步雷达录制")
        self._summary_radar_sync_cb.setEnabled(False)
        radar_row.addWidget(self._summary_radar_sync_cb)

        self._summary_radar_status_label = QLabel("雷达未测试")
        radar_row.addWidget(self._summary_radar_status_label, stretch=1)
        record_layout.addLayout(radar_row)

        button_row = QHBoxLayout()
        self._summary_record_btn = QPushButton("全部开始记录")
        self._summary_record_btn.clicked.connect(self._toggle_summary_recording)
        button_row.addWidget(self._summary_record_btn)
        self._summary_record_status_label = QLabel("")
        button_row.addWidget(self._summary_record_status_label, stretch=1)
        record_layout.addLayout(button_row)
        layout.addWidget(record_group)
        layout.addStretch()

        for key in ("encoder", "imu_A", "imu_B"):
            self._refresh_summary_ports(key)
        return page

    def _build_summary_device_group(self, key: str, title: str, bauds: list[int]) -> QGroupBox:
        group = QGroupBox(title)
        grid = QGridLayout(group)

        source_combo = QComboBox()
        source_combo.addItem("串口", "serial")
        if key == "encoder":
            source_combo.addItem("ROS /odom", "ros_odom")
        else:
            source_combo.addItem("ROS IMU", "ros_imu")
        source_combo.currentIndexChanged.connect(lambda: self._on_summary_source_changed(key))

        port_combo = QComboBox()
        port_combo.setEditable(True)
        port_combo.setMinimumWidth(120)
        baud_combo = QComboBox()
        baud_combo.setMinimumWidth(80)
        for baud in bauds:
            baud_combo.addItem(str(baud), baud)

        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(lambda: self._refresh_summary_ports(key))
        connect_btn = QPushButton("连接")
        connect_btn.clicked.connect(lambda: self._connect_summary_device(key))
        disconnect_btn = QPushButton("断开")
        disconnect_btn.clicked.connect(lambda: self._disconnect_summary_device(key))
        disconnect_btn.setEnabled(False)

        status_label = QLabel("未连接")
        frame_label = QLabel("0")
        error_label = QLabel("0")
        hz_label = QLabel("---")

        grid.addWidget(QLabel("来源"), 0, 0)
        grid.addWidget(source_combo, 0, 1)
        grid.addWidget(QLabel("端口"), 0, 2)
        grid.addWidget(port_combo, 0, 3)
        grid.addWidget(QLabel("波特率"), 0, 4)
        grid.addWidget(baud_combo, 0, 5)
        grid.addWidget(refresh_btn, 0, 6)
        grid.addWidget(connect_btn, 0, 7)
        grid.addWidget(disconnect_btn, 0, 8)
        grid.addWidget(QLabel("状态"), 1, 0)
        grid.addWidget(status_label, 1, 1)
        grid.addWidget(QLabel("帧数"), 1, 2)
        grid.addWidget(frame_label, 1, 3)
        grid.addWidget(QLabel("错误"), 1, 4)
        grid.addWidget(error_label, 1, 5)
        grid.addWidget(QLabel("频率"), 1, 6)
        grid.addWidget(hz_label, 1, 7)

        self._summary_rows[key] = {
            "source_combo": source_combo,
            "port_combo": port_combo,
            "baud_combo": baud_combo,
            "refresh_btn": refresh_btn,
            "connect_btn": connect_btn,
            "disconnect_btn": disconnect_btn,
            "status_label": status_label,
            "frame_label": frame_label,
            "error_label": error_label,
            "hz_label": hz_label,
        }
        self._on_summary_source_changed(key)
        return group

    def _refresh_summary_ports(self, key: str) -> None:
        row = self._summary_rows[key]
        combo: QComboBox = row["port_combo"]
        current_text = combo.currentText()
        combo.clear()
        ports = SerialWorker.list_ports()
        for device, description in ports:
            combo.addItem(f"{device} - {description}", device)
        if current_text and combo.findText(current_text) < 0:
            combo.setEditText(current_text)

    def _summary_port(self, key: str) -> str:
        combo: QComboBox = self._summary_rows[key]["port_combo"]
        port = combo.currentData()
        if port:
            return str(port)
        text = combo.currentText().strip()
        return text.split(" - ")[0] if " - " in text else text

    def _summary_baudrate(self, key: str) -> int:
        combo: QComboBox = self._summary_rows[key]["baud_combo"]
        return int(combo.currentData())

    def _summary_source(self, key: str) -> str:
        combo: QComboBox = self._summary_rows[key]["source_combo"]
        return str(combo.currentData() or "serial")

    def _on_summary_source_changed(self, key: str) -> None:
        row = self._summary_rows[key]
        serial_source = self._summary_source(key) == "serial"
        row["port_combo"].setEnabled(serial_source)
        row["baud_combo"].setEnabled(serial_source)
        row["refresh_btn"].setEnabled(serial_source)
        if serial_source:
            row["status_label"].setText("未连接")
        else:
            row["status_label"].setText("使用 ROS")

    def _connect_summary_device(self, key: str) -> None:
        if self._summary_source(key) != "serial":
            self._ros_worker.open_bridge(self._ros_panel._host_edit.text().strip(), self._ros_panel._port_spin.value())
            return
        port = self._summary_port(key)
        if not port:
            self._summary_rows[key]["status_label"].setText("请选择串口")
            return
        baudrate = self._summary_baudrate(key)
        if key == "encoder":
            self._set_combo_value(self._serial_panel._port_combo, port)
            self._set_combo_value(self._serial_panel._baud_combo, str(baudrate))
            self._on_connect(port, baudrate)
            return

        device_key = "A" if key == "imu_A" else "B"
        device = self._imu_panel._devices[device_key]
        self._set_combo_value(device.port_combo, port)
        self._set_combo_value(device.baud_combo, str(baudrate))
        self._imu_panel._on_connect(device_key)

    def _disconnect_summary_device(self, key: str) -> None:
        if self._summary_source(key) != "serial":
            self._ros_worker.close_bridge()
            return
        if key == "encoder":
            self._on_disconnect()
            return
        device_key = "A" if key == "imu_A" else "B"
        self._imu_panel._on_disconnect(device_key)

    @staticmethod
    def _set_combo_value(combo: QComboBox, value: str) -> None:
        index = combo.findData(value)
        if index < 0:
            index = combo.findText(value)
        if index >= 0:
            combo.setCurrentIndex(index)
        elif combo.isEditable():
            combo.setEditText(value)

    def _toggle_summary_recording(self) -> None:
        if not self._is_summary_recording():
            try:
                self._start_summary_recording()
            except Exception as exc:
                self._summary_record_status_label.setText(f"启动失败: {exc}")
                self._status_label.setText(f"同步记录启动失败: {exc}")
            return
        self._stop_summary_recording(save=True)

    def _is_summary_recording(self) -> bool:
        return self._summary_session_dir is not None

    def _test_summary_radar_connection(self) -> None:
        try:
            response = self._radar_client.identify()
        except Exception as exc:
            self._summary_radar_sync_cb.setChecked(False)
            self._summary_radar_sync_cb.setEnabled(False)
            self._summary_radar_status_label.setText(f"雷达连接失败: {exc}")
            self._status_label.setText(f"雷达连接失败: {exc}")
            return
        self._summary_radar_sync_cb.setEnabled(True)
        self._summary_radar_status_label.setText(f"雷达已连接: {response}")
        self._status_label.setText("雷达连接测试通过")

    def _summary_should_record_radar(self) -> bool:
        return self._summary_radar_sync_cb.isEnabled() and self._summary_radar_sync_cb.isChecked()

    def _start_summary_recording(
        self,
        base_dir: Path | None = None,
        timestamp: str | None = None,
    ) -> Path:
        if timestamp is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if base_dir is None:
            base_dir = Path.cwd() / "recordings"
        session_dir = self._create_summary_session_dir(Path(base_dir), timestamp)

        try:
            if self._summary_should_record_radar():
                self._summary_radar_filename = self._radar_client.start_recording(timestamp)
                self._summary_radar_recording = True
                self._summary_radar_status_label.setText(f"雷达记录中: {self._summary_radar_filename}")
        except Exception:
            session_dir.rmdir()
            raise

        try:
            self._summary_session_dir = session_dir
            if self._summary_source("encoder") == "serial":
                encoder_session = RecordingSession(base_dir=session_dir)
                encoder_session.start()
                self._buffer.start_recording(encoder_session)
                self._summary_encoder_session = encoder_session

            if self._summary_uses_ros():
                ros_session = RosSummaryRecordingSession()
                ros_session.start_in_directory(session_dir, started_at=timestamp)
                self._summary_ros_session = ros_session

            note = self._summary_note_edit.toPlainText().strip()
            if self._summary_uses_serial_imu():
                self._imu_panel.start_recording_in_directory(session_dir, started_at=timestamp, note=note)
            self._write_summary_metadata(session_dir=session_dir, started_at=timestamp, note=note)
        except Exception:
            self._stop_summary_recording(save=False)
            if session_dir.exists():
                session_dir.rmdir()
            raise

        self._record_btn.setText("停止记录")
        self._record_btn.setStyleSheet("background-color: #e74c3c; color: white;")
        self._summary_record_btn.setText("全部停止记录")
        self._summary_record_btn.setStyleSheet("background-color: #e74c3c; color: white;")
        self._summary_record_status_label.setText(f"记录中: {session_dir}")
        self._status_label.setText("正在同步记录汇总数据...")
        return session_dir

    def _create_summary_session_dir(self, base_dir: Path, timestamp: str) -> Path:
        base_dir.mkdir(parents=True, exist_ok=True)
        for counter in range(1000):
            suffix = "" if counter == 0 else f"_{counter:03d}"
            path = base_dir / f"session_{timestamp}{suffix}"
            try:
                path.mkdir()
                return path
            except FileExistsError:
                continue
        raise RuntimeError("无法创建同步记录目录")

    def _write_summary_metadata(self, session_dir: Path, started_at: str, note: str) -> None:
        metadata = {
            "started_at": started_at,
            "note": note,
            "devices": {
                "encoder": self._summary_device_metadata("encoder"),
                "imu_A": self._summary_device_metadata("imu_A"),
                "imu_B": self._summary_device_metadata("imu_B"),
                "radar": self._summary_radar_metadata(),
            },
            "files": self._summary_files_metadata(),
        }
        with (session_dir / "session.json").open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)

    def _summary_uses_ros(self) -> bool:
        return self._summary_source("encoder") == "ros_odom" or any(
            self._summary_source(key) == "ros_imu" for key in ("imu_A", "imu_B")
        )

    def _summary_uses_serial_imu(self) -> bool:
        return any(self._summary_source(key) == "serial" for key in ("imu_A", "imu_B"))

    def _summary_device_metadata(self, key: str) -> dict[str, object]:
        source = self._summary_source(key)
        metadata: dict[str, object] = {"source": source}
        if source == "serial":
            metadata["port"] = self._summary_port(key)
            metadata["baudrate"] = self._summary_baudrate(key)
            return metadata
        metadata["topic"] = "/odom" if source == "ros_odom" else self._summary_imu_topic(key)
        return metadata

    def _summary_radar_metadata(self) -> dict[str, object]:
        return {
            "enabled": self._summary_should_record_radar(),
            "host": getattr(self._radar_client, "host", "127.0.0.1"),
            "port": getattr(self._radar_client, "port", 5026),
            "filename": self._summary_radar_filename,
        }

    def _summary_files_metadata(self) -> dict[str, str]:
        files: dict[str, str] = {}
        if self._summary_source("encoder") == "serial":
            files["encoder"] = "encoder.csv"
        else:
            files["ros_odom"] = "ros_odom.csv"
        if self._summary_uses_serial_imu():
            files.update({
                "imu_A": "imu_A.csv",
                "imu_B": "imu_B.csv",
                "imu_metadata": "imu_session.json",
                "imu_aligned": "imu_merged_aligned.csv",
            })
        if any(self._summary_source(key) == "ros_imu" for key in ("imu_A", "imu_B")):
            files["ros_imu"] = "ros_imu.csv"
        return files

    @staticmethod
    def _summary_imu_topic(key: str) -> str:
        return "/imu" if key == "imu_A" else "/active_imu"

    def _stop_summary_recording(self, save: bool) -> Path | None:
        encoder_session = self._summary_encoder_session
        ros_session = self._summary_ros_session
        session_dir = self._summary_session_dir
        radar_was_recording = self._summary_radar_recording
        self._summary_encoder_session = None
        self._summary_ros_session = None
        self._summary_session_dir = None
        self._summary_radar_recording = False

        self._record_btn.setText("开始记录")
        self._record_btn.setStyleSheet("")
        self._summary_record_btn.setText("全部开始记录")
        self._summary_record_btn.setStyleSheet("")

        radar_stop_error = ""
        if radar_was_recording:
            try:
                self._radar_client.stop_recording()
                self._summary_radar_status_label.setText("雷达记录已停止")
            except Exception as exc:
                radar_stop_error = f"雷达停止失败: {exc}"
                self._summary_radar_status_label.setText(radar_stop_error)

        stopped_encoder_session = self._buffer.stop_recording()
        if encoder_session is None:
            encoder_session = stopped_encoder_session

        if save and session_dir is not None:
            if encoder_session is not None:
                encoder_session.finalize(session_dir / "encoder.csv")
            if self._imu_panel.is_recording():
                self._imu_panel.stop_recording(save=True)
            if ros_session is not None:
                ros_session.finalize()
            self._summary_record_status_label.setText(f"已保存: {session_dir}")
            if radar_stop_error:
                self._status_label.setText(f"同步记录已保存，但{radar_stop_error}")
            else:
                self._status_label.setText(f"同步记录已保存: {session_dir}")
            return session_dir

        if encoder_session is not None:
            encoder_session.cancel()
        if self._imu_panel.is_recording():
            self._imu_panel.stop_recording(save=False)
        if ros_session is not None:
            ros_session.cancel()
        self._summary_record_status_label.setText("同步记录已取消")
        if radar_stop_error:
            self._status_label.setText(f"同步记录已取消，但{radar_stop_error}")
        else:
            self._status_label.setText("同步记录已取消")
        return None

    def _setup_timers(self) -> None:
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._on_refresh)
        self._refresh_timer.start(33)

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(1000)

        self._replay_timer = QTimer(self)
        self._replay_timer.timeout.connect(self._advance_replay)
        self._replay_timer.start(33)

    def _on_connect(self, port: str, baudrate: int) -> None:
        self._buffer.clear()
        self._worker.open_port(port, baudrate)

    def _on_disconnect(self) -> None:
        if self._is_summary_recording():
            self._stop_summary_recording(save=False)
        elif self._buffer.recording:
            self._stop_recording_session(save=False)
        self._worker.close_port()

    def _on_connection_changed(self, connected: bool) -> None:
        self._serial_panel.set_connected(connected)
        self._status_label.setText("已连接，等待数据..." if connected else "已断开")

    def _on_param(self, frame) -> None:
        self._param_panel.update_params(frame)
        self._command_panel.fill_params(frame)
        self._status_label.setText("已收到参数帧")

    def _on_error(self, msg: str) -> None:
        self._status_label.setText(f"错误: {msg}")

    def _on_ros_connection_changed(self, connected: bool) -> None:
        self._ros_connected = connected
        self._ros_panel.set_connected(connected)
        self._ros_imu_panel.set_connected(connected)
        self._status_label.setText("ROS 已连接" if connected else "ROS 已断开")

    def _on_ros_snapshot(self, snapshot) -> None:
        self._latest_ros_snapshot = snapshot
        if self._summary_ros_session is not None:
            self._summary_ros_session.write_snapshot(snapshot)
        current_page = self._module_stack.currentWidget()
        if current_page is self._ros_panel or self._ros_panel.is_recording():
            self._ros_panel.update_snapshot(snapshot)
        if current_page is self._ros_imu_panel:
            self._ros_imu_panel.update_snapshot(snapshot)

    def _on_refresh(self) -> None:
        time_arr, data, frame, title, footer = self._current_source_payload()
        self._plot_panel.refresh_series(time_arr, data)
        if self._data_mode == "replay" and self._replay_data is not None and len(time_arr) > 0:
            self._plot_panel.follow_time_cursor(self._replay_current_time)
        self._data_panel.refresh_frame(frame, title=title, footer_text=footer)
        self._update_analysis(time_arr, data)
        self._sync_replay_slider()

    def _current_source_payload(self):
        if self._data_mode == "replay" and self._replay_data is not None:
            time_arr, data = self._replay_data.snapshot_to_time(self._replay_current_time)
            frame = self._replay_data.latest_frame_at_time(self._replay_current_time)
            footer = f"t = {self._replay_current_time:.2f} s"
            return time_arr, data, frame, "回放数据", footer

        time_arr, data = self._buffer.get_snapshot()
        live_frame = self._buffer.get_latest()
        if live_frame is None:
            return time_arr, data, None, "实时数据", "---"
        frame = {
            "t_raw_A": live_frame.t_raw_A,
            "t_raw_B": live_frame.t_raw_B,
            "m_raw_A": live_frame.m_raw_A,
            "m_raw_B": live_frame.m_raw_B,
            "final_A": live_frame.final_A,
            "final_B": live_frame.final_B,
            "target_A": live_frame.target_A,
            "target_B": live_frame.target_B,
            "output_A": live_frame.output_A,
            "output_B": live_frame.output_B,
            "afc_output_A": live_frame.afc_output_A,
            "afc_output_B": live_frame.afc_output_B,
        }
        footer = self._data_panel._live_fps_text(self._buffer.frame_index)
        return time_arr, data, frame, "实时数据", footer

    def _update_analysis(self, time_arr, data) -> None:
        if len(time_arr) == 0 or len(data) == 0:
            empty = compute_channel_metrics(time_arr, time_arr, time_arr)
            self._analysis_panel.update_metrics("无数据", empty, empty)
            return

        if self._data_mode == "live":
            time_arr, data = self._buffer.get_recent_window(10.0)
            label = "最近 10 秒"
        else:
            label = "当前回放范围"

        metrics_a = compute_channel_metrics(time_arr, data[:, COL_TGT_A], data[:, COL_FINAL_A])
        metrics_b = compute_channel_metrics(time_arr, data[:, COL_TGT_B], data[:, COL_FINAL_B])
        self._analysis_panel.update_metrics(label, metrics_a, metrics_b)

    def _update_status(self) -> None:
        self._frame_label.setText(f"帧: {self._buffer.frame_index}")
        self._error_label.setText(f"错误: {self._worker.error_count}")
        if self._buffer.recording:
            self._record_label.setText(f"记录中: {self._buffer.csv_rows_written} 行")
        else:
            self._record_label.setText("")
        self._update_summary_status()

    def _update_summary_status(self) -> None:
        if self._summary_source("encoder") == "serial":
            self._update_summary_row(
                key="encoder",
                connected=self._serial_connected(self._worker),
                frame_count=self._buffer.frame_index,
                error_count=self._worker.error_count,
            )
        else:
            self._update_summary_row(
                key="encoder",
                connected=self._ros_connected,
                frame_count=0 if self._latest_ros_snapshot is None else self._latest_ros_snapshot.frame_count,
                error_count=self._ros_worker.error_count,
            )
        for summary_key, device_key in (("imu_A", "A"), ("imu_B", "B")):
            if self._summary_source(summary_key) == "serial":
                device = self._imu_panel._devices[device_key]
                self._update_summary_row(
                    key=summary_key,
                    connected=self._serial_connected(device.worker),
                    frame_count=device.buffer.frame_index,
                    error_count=device.worker.error_count,
                )
            else:
                self._update_summary_row(
                    key=summary_key,
                    connected=self._ros_connected,
                    frame_count=self._ros_imu_frame_count(summary_key),
                    error_count=self._ros_worker.error_count,
                )
        if self._is_summary_recording():
            self._summary_record_status_label.setText(self._summary_recording_status_text())

    def _update_summary_row(self, key: str, connected: bool, frame_count: int, error_count: int) -> None:
        row = self._summary_rows[key]
        row["status_label"].setText("正常接收" if connected and frame_count > 0 else ("已连接" if connected else "未连接"))
        row["status_label"].setStyleSheet("color: green;" if connected else "color: red;")
        row["frame_label"].setText(str(frame_count))
        row["error_label"].setText(str(error_count))
        row["hz_label"].setText(self._summary_hz_text(key, frame_count))
        row["connect_btn"].setEnabled(not connected)
        row["disconnect_btn"].setEnabled(connected)
        serial_source = self._summary_source(key) == "serial"
        row["port_combo"].setEnabled(serial_source and not connected)
        row["baud_combo"].setEnabled(serial_source and not connected)
        row["refresh_btn"].setEnabled(serial_source and not connected)

    def _summary_hz_text(self, key: str, frame_count: int) -> str:
        now = perf_counter()
        previous = self._summary_last_counts.get(key)
        self._summary_last_counts[key] = (now, frame_count)
        if previous is None:
            return "---"
        previous_time, previous_count = previous
        elapsed = now - previous_time
        if elapsed <= 0:
            return "---"
        hz = (frame_count - previous_count) / elapsed
        return f"{max(0.0, hz):.0f} Hz"

    @staticmethod
    def _serial_connected(worker) -> bool:
        serial_port = getattr(worker, "_serial", None)
        return bool(serial_port is not None and serial_port.is_open)

    def _ros_imu_frame_count(self, key: str) -> int:
        if self._latest_ros_snapshot is None:
            return 0
        reading = self._latest_ros_snapshot.imu if key == "imu_A" else self._latest_ros_snapshot.active_imu
        return reading.frame_count

    def _summary_recording_status_text(self) -> str:
        parts: list[str] = []
        if self._summary_encoder_session is not None:
            parts.append(f"编码器 {self._buffer.csv_rows_written} 行")
        if self._imu_panel.is_recording():
            parts.append(f"串口 IMU {self._imu_panel.recording_rows_text()}")
        if self._summary_ros_session is not None:
            rows = self._summary_ros_session.rows_written_by_stream
            parts.append(f"ROS odom {rows['odom']} 行")
            parts.append(f"ROS IMU {rows['imu']} 行")
        return "记录中: " + ", ".join(parts)

    def _toggle_record(self) -> None:
        if not self._buffer.recording:
            self._start_recording_session()
            return
        self._stop_recording_session(save=True)

    def _start_recording_session(self) -> None:
        session = RecordingSession(base_dir=Path(tempfile.gettempdir()))
        session.start()
        self._buffer.start_recording(session)
        self._record_btn.setText("停止记录")
        self._record_btn.setStyleSheet("background-color: #e74c3c; color: white;")
        self._status_label.setText("正在流式记录...")

    def _stop_recording_session(self, save: bool) -> None:
        session = self._buffer.stop_recording()
        self._record_btn.setText("开始记录")
        self._record_btn.setStyleSheet("")
        if session is None:
            return

        if not save:
            session.cancel()
            self._status_label.setText("记录已取消")
            return

        default_name = self._recording_default_name()
        filepath, _ = QFileDialog.getSaveFileName(self, "保存记录数据", default_name, "CSV 文件 (*.csv)")
        if filepath:
            session.finalize(Path(filepath))
            self._status_label.setText(f"记录已保存，共 {session.rows_written} 行")
        else:
            session.cancel()
            self._status_label.setText(f"记录已丢弃（{session.rows_written} 行）")

    def _recording_default_name(self, timestamp: str | None = None) -> str:
        if timestamp is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        (a_kp, a_ki, a_kd), (b_kp, b_ki, b_kd) = self._command_panel.current_motor_pid_values_int()
        return (
            f"debug_data_Akp{a_kp}_Aki{a_ki}_Akd{a_kd}_"
            f"Bkp{b_kp}_Bki{b_ki}_Bkd{b_kd}_{timestamp}.csv"
        )

    def _load_replay_csv(self) -> None:
        filepath, _ = QFileDialog.getOpenFileName(self, "加载 CSV 回放", "", "CSV 文件 (*.csv)")
        if not filepath:
            return
        try:
            self._replay_data = ReplayData.load(Path(filepath))
        except Exception as exc:
            self._status_label.setText(f"加载回放失败: {exc}")
            return
        self._replay_current_time = 0.0
        self._replay_controls_enabled(True)
        self._set_data_mode("replay")
        self._status_label.setText(f"已加载回放: {Path(filepath).name}")

    def _replay_controls_enabled(self, enabled: bool) -> None:
        self._play_btn.setEnabled(enabled)
        self._progress_slider.setEnabled(enabled)
        self._speed_combo.setEnabled(enabled)

    def _on_mode_changed(self) -> None:
        self._set_data_mode(self._mode_combo.currentData())

    def _set_data_mode(self, mode: str) -> None:
        if mode == "replay" and self._replay_data is None:
            self._mode_combo.setCurrentIndex(0)
            self._status_label.setText("请先加载 CSV")
            self._data_mode = "live"
            return
        self._data_mode = mode
        index = 0 if mode == "live" else 1
        if self._mode_combo.currentIndex() != index:
            self._mode_combo.setCurrentIndex(index)
        if mode == "live":
            self._play_btn.setText("播放")

    def _toggle_replay_playback(self) -> None:
        if self._data_mode != "replay" or self._replay_data is None:
            return
        if self._replay_timer.isActive() and self._play_btn.text() == "暂停":
            self._play_btn.setText("播放")
            return
        self._play_btn.setText("暂停")

    def _advance_replay(self) -> None:
        if self._data_mode != "replay" or self._replay_data is None:
            return
        if self._play_btn.text() != "暂停":
            return
        self._replay_current_time += 0.033 * self._replay_speed
        if self._replay_current_time >= self._replay_data.duration_s:
            self._replay_current_time = self._replay_data.duration_s
            self._play_btn.setText("播放")

    def _on_replay_slider_changed(self, value: int) -> None:
        if self._replay_data is None:
            return
        ratio = value / 1000.0
        self._replay_current_time = self._replay_data.duration_s * ratio

    def _sync_replay_slider(self) -> None:
        if self._replay_data is None or self._replay_data.duration_s <= 0:
            return
        ratio = self._replay_current_time / self._replay_data.duration_s
        self._progress_slider.blockSignals(True)
        self._progress_slider.setValue(int(max(0.0, min(ratio, 1.0)) * 1000))
        self._progress_slider.blockSignals(False)

    def _on_speed_changed(self) -> None:
        self._replay_speed = float(self._speed_combo.currentData())

    def _clear_data(self) -> None:
        if self._is_summary_recording():
            self._stop_summary_recording(save=False)
        elif self._buffer.recording:
            self._stop_recording_session(save=False)
        self._buffer.clear()
        if self._data_mode == "replay":
            self._replay_current_time = 0.0
        self._plot_panel.reset()
        self._status_label.setText("数据已清空")

    def current_data_mode(self) -> str:
        return self._data_mode

    def _set_replay_loaded_for_test(self, time_values, rows) -> None:
        self._replay_data = ReplayData.from_rows(time_values, rows)
        self._replay_current_time = float(time_values[-1]) if time_values else 0.0
        self._replay_controls_enabled(True)

    def _set_data_mode_for_test(self, mode: str) -> None:
        self._set_data_mode(mode)

    def showEvent(self, event) -> None:
        super().showEvent(event)
        if self._initial_screen_fit_done:
            return
        self._fit_window_to_screen()
        self._center_on_screen()
        self._initial_screen_fit_done = True

    def _fit_window_to_screen(self) -> None:
        screen = self.screen() or QGuiApplication.primaryScreen()
        if screen is None:
            return

        available = screen.availableGeometry()
        frame = self.frameGeometry()
        geometry = self.geometry()
        frame_extra_width = max(0, frame.width() - geometry.width())
        frame_extra_height = max(0, frame.height() - geometry.height())
        target_width = max(100, available.width() - frame_extra_width)
        target_height = max(100, available.height() - frame_extra_height)
        self.resize(
            min(self.width(), target_width),
            min(self.height(), target_height),
        )

    def closeEvent(self, event) -> None:
        if self._is_summary_recording():
            self._stop_summary_recording(save=False)
        elif self._buffer.recording:
            self._stop_recording_session(save=False)
        self._refresh_timer.stop()
        self._status_timer.stop()
        self._replay_timer.stop()
        self._worker.close_port()
        self._ros_worker.close_bridge()
        self._ros_panel.shutdown()
        self._ros_imu_panel.shutdown()
        self._imu_panel.shutdown()
        super().closeEvent(event)

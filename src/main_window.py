"""
主窗口：布局编排、信号连接、定时器管理。
"""

from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime
from pathlib import Path
from time import perf_counter, time
from dataclasses import asdict

from PySide6.QtCore import QObject, Qt, QThread, QTimer, Signal, Slot
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
    QLineEdit,
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

from app_config import DEFAULT_FASTLIO_ODOM_TOPIC, DEFAULT_RECORDINGS_DIR, DEFAULT_ROSBAG_REMOTE_DIR
from akm_topic_recorders import AkmStateRecorder, ChassisDiagnosticsRecorder, ControlDebugRecorder
from analytics import compute_channel_metrics
from data_buffer import COL_FINAL_A, COL_FINAL_B, COL_TGT_A, COL_TGT_B, DataBuffer
from hr23_radar_client import Hr23RadarClient, Hr23RadarError
from recording_session import RecordingSession
from radar_bin_parser import parse_radar_recording
from radar_scpi import RadarScpiClient
from replay_data import ReplayData
from ros_bridge_worker import RosBridgeWorker
from rosbag_models import (
    RemoteRosbagSession,
    RosbagLibraryState,
    RosbagRecordingStatus,
    extract_rosbag_protocol_error,
    parse_rosbag_library_state,
    parse_rosbag_recording_status,
    rosbag_protocol_data,
)
from rosbag_sync_worker import RosbagSyncWorker
from ros_topic_recorders import (
    make_odometry_recorder,
    make_power_voltage_recorder,
    make_ros_imu_compat_recorder,
    make_ros_odom_compat_recorder,
    sample_ros_topic,
    sample_ros_topics,
)
from recording_clock import RecordingClock
from serial_worker import SerialWorker
from summary_package import build_summary_package
from widgets.analysis_panel import AnalysisPanel
from widgets.command_panel import CommandPanel
from widgets.data_panel import DataPanel
from widgets.imu_panel import ImuPanel
from widgets.localization_panel import LocalizationPanel
from widgets.param_panel import ParamPanel
from widgets.plot_panel import PlotPanel
from widgets.ros_imu_panel import RosImuPanel
from widgets.rosbag_panel import RosbagPanel
from widgets.ros_panel import RosPanel
from widgets.serial_panel import SerialPanel


class _FunctionWorker(QObject):
    finished = Signal(object)
    error = Signal(str)

    def __init__(self, function) -> None:
        super().__init__()
        self._function = function

    @Slot()
    def run(self) -> None:
        try:
            self.finished.emit(self._function())
        except Exception as exc:
            self.error.emit(str(exc))


class MainWindow(QMainWindow):
    """调试监视器主窗口。"""

    _DEFAULT_WINDOW_WIDTH = 1400
    _DEFAULT_WINDOW_HEIGHT = 760
    _SUMMARY_CHECK_TIMEOUT_MS = 20_000

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("WHEELTEC C50X 调试监视器")
        self.setMinimumSize(100, 100)
        self._apply_initial_window_size()

        self._buffer = DataBuffer()
        self._worker = SerialWorker(self._buffer)
        self._worker.param_received.connect(self._on_param)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._ros_worker = RosBridgeWorker()
        self._ros_worker.snapshot_received.connect(self._on_ros_snapshot)
        self._ros_worker.message_received.connect(self._on_ros_message)
        self._ros_worker.launch_manager_status_received.connect(self._on_launch_manager_status)
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
        self._summary_ros_session = None
        self._summary_session_dir: Path | None = None
        self._summary_radar_recording = False
        self._summary_radar_filename = ""
        self._summary_radar_started_epoch_s: float | None = None
        self._summary_radar_started_session_elapsed_s = 0.0
        self._summary_clock: RecordingClock | None = None
        self._summary_topic_recorders: dict[str, list[object]] = {}
        self._summary_last_check_results: dict[str, dict[str, object]] = {}
        self._summary_last_check_epoch_s: float | None = None
        self._summary_last_warnings: list[str] = []
        self._summary_last_errors: list[str] = []
        self._summary_check_generation = 0
        self._summary_recording_state = "IDLE"
        self._summary_recording_gate_enabled = False
        self._summary_recording_start_epoch_s: float | None = None
        self._summary_recording_start_perf_s: float | None = None
        self._summary_recording_stop_epoch_s: float | None = None
        self._summary_recording_stop_perf_s: float | None = None
        self._summary_dropped_pre_start_ros_messages = 0
        self._summary_dropped_post_stop_ros_messages = 0
        self._summary_background_threads: list[QThread] = []
        self._summary_background_workers: list[_FunctionWorker] = []
        self._rosbag_sync_threads: list[QThread] = []
        self._rosbag_sync_workers: list[RosbagSyncWorker] = []
        self._latest_ros_snapshot = None
        self._latest_rosbag_status = RosbagRecordingStatus()
        self._latest_rosbag_library = RosbagLibraryState()
        self._summary_rosbag_session_id = ""
        self._summary_rosbag_config: dict[str, object] = {}
        self._summary_rosbag_start_sent = False
        self._summary_rosbag_stop_sent = False
        self._summary_latest_rosbag_status: dict[str, object] = {}
        self._ros_topic_frame_counts: dict[str, int] = {}
        self._ros_connected = False
        self._radar_client = RadarScpiClient()
        self._hr23_radar_client_factory = Hr23RadarClient
        self._summary_hr23_session_client = None
        self._summary_hr23_active = False
        self._summary_hr23_session_metadata: dict[str, object] = {}

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
        self._rosbag_panel = RosbagPanel()
        self._localization_panel = LocalizationPanel()
        self._ros_panel.connect_requested.connect(self._on_ros_connect)
        self._ros_panel.disconnect_requested.connect(self._ros_worker.close_bridge)
        self._ros_panel.data_subscriptions_changed.connect(self._ros_worker.update_data_subscriptions)
        self._ros_panel.cmd_vel_requested.connect(self._ros_worker.publish_cmd_vel)
        self._ros_panel.pid_control_requested.connect(self._ros_worker.publish_line_follow_control)
        self._ros_panel.launch_manager_command_requested.connect(self._publish_launch_manager_command)
        self._ros_imu_panel.connect_requested.connect(self._ros_worker.open_bridge)
        self._ros_imu_panel.disconnect_requested.connect(self._ros_worker.close_bridge)
        self._connect_rosbag_panel_signals()
        self._summary_page = self._build_summary_page()
        self._module_stack.addWidget(self._summary_page)
        self._module_stack.addWidget(self._encoder_page)
        self._module_stack.addWidget(self._imu_panel)
        self._module_stack.addWidget(self._ros_panel)
        self._module_stack.addWidget(self._ros_imu_panel)
        self._module_stack.addWidget(self._rosbag_panel)
        self._module_stack.addWidget(self._localization_panel)
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

        self._rosbag_module_btn = QPushButton("ROSBag")
        self._rosbag_module_btn.setCheckable(True)
        self._rosbag_module_btn.clicked.connect(lambda: self._switch_module("rosbag"))
        self._module_button_group.addButton(self._rosbag_module_btn)
        switch_row.addWidget(self._rosbag_module_btn)

        self._localization_module_btn = QPushButton("定位精度")
        self._localization_module_btn.setCheckable(True)
        self._localization_module_btn.clicked.connect(lambda: self._switch_module("localization"))
        self._module_button_group.addButton(self._localization_module_btn)
        switch_row.addWidget(self._localization_module_btn)

        switch_row.addStretch()
        root_layout.addLayout(switch_row)

    def _switch_module(self, module: str) -> None:
        if module == "localization":
            self._module_stack.setCurrentWidget(self._localization_panel)
            self._status_label.setText("FAST-LIO2 定位精度测试")
            return
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
        if module == "rosbag":
            self._module_stack.setCurrentWidget(self._rosbag_panel)
            self._status_label.setText("ROSBag 管理")
            return
        self._module_stack.setCurrentWidget(self._encoder_page)
        self._status_label.setText("编码器模块")

    def _connect_rosbag_panel_signals(self) -> None:
        self._rosbag_panel.start_requested.connect(self._ros_worker.request_rosbag_start)
        self._rosbag_panel.stop_requested.connect(self._ros_worker.request_rosbag_stop)
        self._rosbag_panel.list_requested.connect(self._ros_worker.request_rosbag_list)
        self._rosbag_panel.inspect_requested.connect(self._ros_worker.request_rosbag_inspect)
        self._rosbag_panel.trash_requested.connect(self._ros_worker.request_rosbag_trash)
        self._rosbag_panel.delete_requested.connect(self._on_rosbag_delete_requested)
        self._rosbag_panel.query_status_requested.connect(self._ros_worker.request_launch_manager_status)
        self._rosbag_panel.sync_requested.connect(self._start_rosbag_sync)

    def _on_rosbag_delete_requested(self, session_id: str, confirm: str) -> None:
        self._ros_worker.request_rosbag_delete(session_id, confirm)
        self._status_label.setText("已发送永久删除请求")

    def _build_summary_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        layout.addWidget(self._build_summary_save_group())
        layout.addWidget(self._build_summary_trajectory_group())
        layout.addWidget(self._build_summary_source_group())
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

        rosbag_row = QHBoxLayout()
        self._summary_rosbag_sync_cb = QCheckBox("同步车端 rosbag 录制")
        self._summary_rosbag_sync_cb.setChecked(True)
        rosbag_row.addWidget(self._summary_rosbag_sync_cb)
        self._summary_rosbag_status_label = QLabel("ROSbridge 未连接")
        rosbag_row.addWidget(self._summary_rosbag_status_label, stretch=1)
        record_layout.addLayout(rosbag_row)

        radar_path_row = QGridLayout()
        self._summary_radar_source_dir_edit = QLineEdit("")
        self._summary_radar_source_dir_edit.setPlaceholderText("雷达软件输出 .bin 的目录")
        self._summary_radar_xml_path_edit = QLineEdit("")
        self._summary_radar_xml_path_edit.setPlaceholderText("radar_config.xml 路径")
        radar_path_row.addWidget(QLabel("雷达目录"), 0, 0)
        radar_path_row.addWidget(self._summary_radar_source_dir_edit, 0, 1)
        radar_path_row.addWidget(QLabel("XML"), 1, 0)
        radar_path_row.addWidget(self._summary_radar_xml_path_edit, 1, 1)
        record_layout.addLayout(radar_path_row)

        button_row = QHBoxLayout()
        self._summary_check_btn = QPushButton("检查可记录数据源")
        self._summary_check_btn.clicked.connect(self._start_summary_source_check)
        button_row.addWidget(self._summary_check_btn)
        self._summary_record_btn = QPushButton("全部开始记录")
        self._summary_record_btn.clicked.connect(self._toggle_summary_recording)
        button_row.addWidget(self._summary_record_btn)
        self._summary_cancel_btn = QPushButton("取消本次记录")
        self._summary_cancel_btn.clicked.connect(self._cancel_summary_recording)
        self._summary_cancel_btn.setEnabled(False)
        button_row.addWidget(self._summary_cancel_btn)
        self._summary_record_status_label = QLabel("")
        button_row.addWidget(self._summary_record_status_label, stretch=1)
        record_layout.addLayout(button_row)

        self._summary_check_status_label = QLabel("")
        record_layout.addWidget(self._summary_check_status_label)
        layout.addWidget(record_group)
        layout.addStretch()

        for key in ("encoder", "imu_A", "imu_B"):
            self._refresh_summary_ports(key)
        return page

    def _build_summary_save_group(self) -> QGroupBox:
        group = QGroupBox("保存目录")
        layout = QGridLayout(group)
        self._summary_save_dir_edit = QLineEdit(DEFAULT_RECORDINGS_DIR)
        self._summary_session_name_edit = QLineEdit(self._summary_default_session_name())
        choose_btn = QPushButton("选择目录")
        choose_btn.clicked.connect(self._choose_summary_save_dir)
        self._summary_open_save_dir_btn = QPushButton("打开目录")
        self._summary_open_save_dir_btn.clicked.connect(self._open_summary_save_dir)

        layout.addWidget(QLabel("目录"), 0, 0)
        layout.addWidget(self._summary_save_dir_edit, 0, 1)
        layout.addWidget(choose_btn, 0, 2)
        layout.addWidget(self._summary_open_save_dir_btn, 0, 3)
        layout.addWidget(QLabel("Session"), 1, 0)
        layout.addWidget(self._summary_session_name_edit, 1, 1, 1, 3)
        return group

    def _build_summary_trajectory_group(self) -> QGroupBox:
        group = QGroupBox("轨迹主话题")
        layout = QGridLayout(group)
        self._trajectory_topic_combo = QComboBox()
        self._trajectory_topic_combo.addItem("FAST-LIO /Odometry", DEFAULT_FASTLIO_ODOM_TOPIC)
        self._trajectory_topic_combo.addItem("Legacy /odom", "/odom")
        self._trajectory_topic_combo.addItem("自定义 nav_msgs/Odometry", "__custom__")
        self._trajectory_topic_combo.currentIndexChanged.connect(self._on_trajectory_topic_changed)
        self._trajectory_topic_custom_edit = QLineEdit()
        self._trajectory_topic_custom_edit.setPlaceholderText("/custom_odometry")
        self._trajectory_topic_custom_edit.setEnabled(False)

        layout.addWidget(QLabel("选择"), 0, 0)
        layout.addWidget(self._trajectory_topic_combo, 0, 1)
        layout.addWidget(QLabel("自定义"), 1, 0)
        layout.addWidget(self._trajectory_topic_custom_edit, 1, 1)
        return group

    def _build_summary_source_group(self) -> QGroupBox:
        group = QGroupBox("可记录数据源")
        layout = QGridLayout(group)
        source_items = [
            ("fastlio_odometry", "/Odometry"),
            ("ros_odom", "/odom"),
            ("ros_imu", "/imu"),
            ("ros_active_imu", "/active_imu"),
            ("ros_power_voltage", "/PowerVoltage"),
            ("akm_state", "/wheeltec/akm_state"),
            ("control_debug", "/wheeltec/control_debug"),
            ("chassis_diagnostics", "/wheeltec/chassis_diagnostics"),
            ("serial_encoder", "串口编码器 / STM32 debug UART"),
            ("imu_A", "串口 IMU A"),
            ("imu_B", "串口 IMU B"),
            ("radar_bin", "谐波雷达 .bin"),
        ]
        self._summary_source_checks = {}
        for index, (source_id, label) in enumerate(source_items):
            checkbox = QCheckBox(label)
            checkbox.setChecked(True)
            self._summary_source_checks[source_id] = checkbox
            layout.addWidget(checkbox, index // 2, index % 2)
        hr23_row = (len(source_items) + 1) // 2
        hr23_checkbox = QCheckBox("新谐波雷达 HR2.3")
        hr23_checkbox.setChecked(False)
        self._summary_source_checks["hr23_radar"] = hr23_checkbox
        layout.addWidget(hr23_checkbox, hr23_row, 0)
        layout.addWidget(self._build_summary_hr23_controls(), hr23_row, 1)
        return group

    def _build_summary_hr23_controls(self) -> QWidget:
        controls = QWidget()
        layout = QHBoxLayout(controls)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        self._summary_hr23_state_label = QLabel("state: ---")
        self._summary_hr23_packets_label = QLabel("packets: 0")
        self._summary_hr23_bytes_label = QLabel("bytes: 0")
        self._summary_hr23_last_packet_label = QLabel("lastPacketUtc: ---")
        self._summary_hr23_host_edit = QLineEdit("127.0.0.1")
        self._summary_hr23_host_edit.setMaximumWidth(120)
        self._summary_hr23_port_edit = QLineEdit("7070")
        self._summary_hr23_port_edit.setMaximumWidth(64)
        self._summary_hr23_test_btn = QPushButton("测试")
        self._summary_hr23_test_btn.clicked.connect(self._test_summary_hr23_connection)

        for widget in (
            self._summary_hr23_state_label,
            self._summary_hr23_packets_label,
            self._summary_hr23_bytes_label,
            self._summary_hr23_last_packet_label,
        ):
            layout.addWidget(widget)
        layout.addWidget(QLabel("host:"))
        layout.addWidget(self._summary_hr23_host_edit)
        layout.addWidget(QLabel("port:"))
        layout.addWidget(self._summary_hr23_port_edit)
        layout.addWidget(self._summary_hr23_test_btn)
        return controls

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
            self._ros_worker.open_bridge(
                self._ros_panel._host_edit.text().strip(),
                self._ros_panel._port_spin.value(),
                self._ros_panel.selected_data_topics(),
            )
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
            if not self._summary_has_fresh_check_results():
                self._start_summary_source_check()
                self._status_label.setText("请等待数据源检查完成后再次开始记录")
                return
            try:
                self._start_summary_recording()
            except Exception as exc:
                self._summary_record_status_label.setText(f"启动失败: {exc}")
                self._status_label.setText(f"同步记录启动失败: {exc}")
            return
        self._stop_summary_recording_async(save=True)

    def _is_summary_recording(self) -> bool:
        return self._summary_session_dir is not None

    def _summary_has_fresh_check_results(self, ttl_s: float = 30.0) -> bool:
        if not self._summary_last_check_results or self._summary_last_check_epoch_s is None:
            return False
        return (time() - self._summary_last_check_epoch_s) <= ttl_s

    def _set_summary_recording_state(self, state: str, detail: str = "") -> None:
        self._summary_recording_state = state
        if hasattr(self, "_summary_check_btn"):
            self._summary_check_btn.setEnabled(state in {"IDLE", "READY", "DONE", "ERROR"})
        if hasattr(self, "_summary_record_btn"):
            self._summary_record_btn.setEnabled(state not in {"CHECKING", "STOPPING", "PACKAGING"})
        if hasattr(self, "_summary_cancel_btn"):
            self._summary_cancel_btn.setEnabled(state == "RECORDING")
        if hasattr(self, "_summary_record_status_label"):
            label = detail or state
            self._summary_record_status_label.setText(label)

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

    def _make_hr23_radar_client(self):
        host = self._summary_hr23_host_edit.text().strip() or "127.0.0.1"
        try:
            port = int(self._summary_hr23_port_edit.text().strip())
        except ValueError as exc:
            raise Hr23RadarError("cmd=status state= error=invalid port message=port must be an integer") from exc
        if not 1 <= port <= 65535:
            raise Hr23RadarError("cmd=status state= error=invalid port message=port must be 1..65535")
        return self._hr23_radar_client_factory(host=host, port=port, timeout=2.0)

    def _test_summary_hr23_connection(self) -> None:
        try:
            response = self._make_hr23_radar_client().status()
        except Exception as exc:
            self._update_summary_hr23_status(error=str(exc))
            self._status_label.setText(f"HR2.3 连接失败: {exc}")
            return
        self._update_summary_hr23_status(response)
        self._status_label.setText("HR2.3 连接测试通过")

    def _update_summary_hr23_status(
        self,
        response: dict[str, object] | None = None,
        error: str = "",
    ) -> None:
        response = response or {}
        state = "error" if error else str(response.get("state", "---"))
        self._summary_hr23_state_label.setText(f"state: {state}")
        self._summary_hr23_packets_label.setText(f"packets: {response.get('packetCount', 0)}")
        self._summary_hr23_bytes_label.setText(f"bytes: {response.get('totalBytes', 0)}")
        last_packet = response.get("lastPacketUtc", "---")
        self._summary_hr23_last_packet_label.setText(f"lastPacketUtc: {last_packet}")
        self._summary_hr23_state_label.setToolTip(error)

    def _new_hr23_session_metadata(self, session_id: str, enabled: bool) -> dict[str, object]:
        try:
            port = int(self._summary_hr23_port_edit.text().strip())
        except ValueError:
            port = 7070
        return {
            "enabled": enabled,
            "host": self._summary_hr23_host_edit.text().strip() or "127.0.0.1",
            "port": port,
            "session_id": session_id,
            "capture_dir": "raw/hr23_radar",
            "files": {
                "raw": "raw/hr23_radar/raw.dat",
                "packets": "raw/hr23_radar/packets.csv",
                "events": "raw/hr23_radar/events.csv",
                "metadata": "raw/hr23_radar/metadata.json",
            },
            "error": "",
            "stop_error": "",
        }

    @staticmethod
    def _merge_hr23_response(metadata: dict[str, object], response: dict[str, object]) -> None:
        for key in (
            "packetCount",
            "totalBytes",
            "firstPacketUtc",
            "lastPacketUtc",
            "rawFileClosedUtc",
            "time",
        ):
            if key in response:
                metadata[key] = response[key]

    def _summary_should_record_radar(self) -> bool:
        return (
            self._summary_source_enabled("radar_bin")
            and self._summary_radar_sync_cb.isEnabled()
            and self._summary_radar_sync_cb.isChecked()
        )

    def _start_summary_recording(
        self,
        base_dir: Path | None = None,
        timestamp: str | None = None,
    ) -> Path:
        check_results = (
            self._summary_last_check_results
            if self._summary_has_fresh_check_results()
            else self._check_summary_sources()
        )
        blockers = [
            source_id
            for source_id, result in check_results.items()
            if result.get("status") in {"offline", "error"}
        ]
        if blockers:
            names = ", ".join(blockers)
            raise RuntimeError(f"数据源未就绪，无法开始: {names}")
        if timestamp is None:
            timestamp = self._summary_session_timestamp()
        if base_dir is None:
            base_dir = Path(self._summary_save_dir_edit.text().strip() or DEFAULT_RECORDINGS_DIR)
        self._summary_clock = RecordingClock(session_id=f"session_{timestamp}")
        self._summary_recording_stop_epoch_s = None
        self._summary_recording_stop_perf_s = None
        self._summary_dropped_pre_start_ros_messages = 0
        self._summary_dropped_post_stop_ros_messages = 0
        self._summary_recording_gate_enabled = True
        self._set_summary_recording_state("STARTING", "STARTING")
        session_dir = self._create_summary_session_dir(Path(base_dir), timestamp)
        self._summary_clock.session_id = session_dir.name
        self._summary_recording_start_epoch_s = self._summary_clock.start_epoch_s
        self._summary_recording_start_perf_s = self._summary_clock.start_perf_s
        self._summary_rosbag_session_id = ""
        self._summary_rosbag_config = {}
        self._summary_rosbag_start_sent = False
        self._summary_rosbag_stop_sent = False
        self._summary_latest_rosbag_status = {}

        if self._summary_source_enabled("rosbag_raw"):
            self._start_summary_rosbag(session_dir.name)
        session_id = session_dir.name
        note = self._summary_note_edit.toPlainText().strip()
        hr23_enabled = self._summary_source_enabled("hr23_radar")
        self._summary_hr23_session_metadata = self._new_hr23_session_metadata(session_id, hr23_enabled)
        self._summary_hr23_session_client = None
        self._summary_hr23_active = False

        if hr23_enabled:
            hr23_output_dir = session_dir / "raw" / "hr23_radar"
            hr23_output_dir.mkdir(parents=True)
            try:
                hr23_client = self._make_hr23_radar_client()
                self._summary_hr23_session_client = hr23_client
                prepare_cmd_send_epoch_s = time()
                prepare_cmd_send_perf_s = perf_counter()
                self._summary_hr23_session_metadata["prepare_cmd_send_epoch_s"] = prepare_cmd_send_epoch_s
                self._summary_hr23_session_metadata["prepare_cmd_send_perf_s"] = prepare_cmd_send_perf_s
                prepare_response = hr23_client.prepare(
                    session_id=session_id,
                    output_dir=hr23_output_dir,
                    prepare_cmd_send_epoch_s=prepare_cmd_send_epoch_s,
                    prepare_cmd_send_perf_s=prepare_cmd_send_perf_s,
                    metadata={"experimentNote": note, "operator": ""},
                    recording_start_epoch_s=self._summary_recording_start_epoch_s,
                    recording_start_perf_s=self._summary_recording_start_perf_s,
                )
                self._summary_hr23_session_metadata["prepare_ack_recv_epoch_s"] = time()
                self._merge_hr23_response(self._summary_hr23_session_metadata, prepare_response)
                self._summary_hr23_active = True
                self._update_summary_hr23_status(prepare_response)
            except Exception as exc:
                self._summary_hr23_session_metadata["error"] = str(exc)
                self._summary_recording_gate_enabled = False
                self._summary_clock = None
                self._summary_hr23_session_client = None
                self._summary_hr23_active = False
                self._update_summary_hr23_status(error=str(exc))
                try:
                    hr23_output_dir.rmdir()
                    hr23_output_dir.parent.rmdir()
                    session_dir.rmdir()
                except OSError:
                    pass
                self._set_summary_recording_state("ERROR", f"HR2.3 prepare 失败: {exc}")
                raise RuntimeError(f"HR2.3 prepare 失败: {exc}") from exc

        self._summary_session_dir = session_dir
        try:
            if self._summary_should_record_radar():
                self._summary_radar_started_epoch_s = self._summary_clock.now_epoch_s()
                self._summary_radar_started_session_elapsed_s = self._summary_clock.elapsed_s()
                self._summary_radar_filename = self._radar_client.start_recording(timestamp)
                self._summary_radar_recording = True
                self._summary_radar_status_label.setText(f"雷达记录中: {self._summary_radar_filename}")
        except Exception:
            self._stop_summary_rosbag()
            self._summary_recording_gate_enabled = False
            try:
                self._stop_summary_recording(save=False)
            except Exception:
                pass
            try:
                session_dir.rmdir()
            except OSError:
                pass
            self._set_summary_recording_state("ERROR", "启动失败")
            raise

        try:
            self._summary_topic_recorders = self._create_summary_topic_recorders(session_dir)
            if self._summary_source("encoder") == "serial" and self._summary_source_enabled("serial_encoder"):
                encoder_session = RecordingSession(base_dir=session_dir)
                encoder_session.start()
                self._buffer.start_recording(encoder_session)
                self._summary_encoder_session = encoder_session

            if self._summary_uses_serial_imu():
                self._imu_panel.start_recording_in_directory(session_dir, started_at=timestamp, note=note)
            if self._summary_hr23_active:
                self._summary_hr23_session_metadata["start_cmd_send_epoch_s"] = time()
                start_response = self._summary_hr23_session_client.start()
                self._summary_hr23_session_metadata["start_ack_recv_epoch_s"] = time()
                self._merge_hr23_response(self._summary_hr23_session_metadata, start_response)
                self._update_summary_hr23_status(start_response)
            self._write_summary_metadata(
                session_dir=session_dir,
                started_at=timestamp,
                note=note,
                check_results=check_results,
            )
        except Exception:
            self._stop_summary_rosbag()
            self._summary_recording_gate_enabled = False
            try:
                self._stop_summary_recording(save=False)
            except Exception:
                pass
            if session_dir.exists():
                try:
                    session_dir.rmdir()
                except OSError:
                    pass
            self._set_summary_recording_state("ERROR", "启动失败")
            raise

        self._record_btn.setText("停止记录")
        self._record_btn.setStyleSheet("background-color: #e74c3c; color: white;")
        self._summary_record_btn.setText("全部停止记录")
        self._summary_record_btn.setStyleSheet("background-color: #e74c3c; color: white;")
        self._set_summary_recording_state("RECORDING", f"RECORDING: {session_dir}")
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

    def _write_summary_metadata(
        self,
        session_dir: Path,
        started_at: str,
        note: str,
        check_results: dict[str, dict[str, object]],
    ) -> None:
        estimated_topic_hz = {
            source_id: result.get("estimated_hz", 0.0)
            for source_id, result in check_results.items()
        }
        session_id = self._summary_clock.session_id if self._summary_clock is not None else f"session_{started_at}"
        metadata = {
            "session_id": session_id,
            "started_at": started_at,
            "started_at_iso": datetime.now().astimezone().isoformat(timespec="seconds"),
            "recording_start_epoch_s": self._summary_recording_start_epoch_s,
            "recording_start_perf_s": self._summary_recording_start_perf_s,
            "recording_gate_enabled_at_start": self._summary_recording_gate_enabled,
            "dropped_pre_start_ros_messages": self._summary_dropped_pre_start_ros_messages,
            "dropped_post_stop_ros_messages": self._summary_dropped_post_stop_ros_messages,
            "note": note,
            "selected_sources": self._selected_summary_sources(),
            "trajectory_main_topic": self._summary_trajectory_topic(),
            "rosbridge": {
                "host": self._summary_rosbridge_host(),
                "port": self._summary_rosbridge_port(),
            },
            "source_check_results": check_results,
            "estimated_topic_hz": estimated_topic_hz,
            "warnings": list(self._summary_last_warnings),
            "errors": list(self._summary_last_errors),
            "devices": {
                "encoder": self._summary_device_metadata("encoder"),
                "imu_A": self._summary_device_metadata("imu_A"),
                "imu_B": self._summary_device_metadata("imu_B"),
                "radar": self._summary_radar_metadata(),
                "hr23_radar": dict(self._summary_hr23_session_metadata),
            },
            "files": self._summary_files_metadata(),
        }
        if self._summary_source_enabled("rosbag_raw"):
            metadata["rosbag"] = self._summary_rosbag_metadata()
        with (session_dir / "session.json").open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)

    def _update_summary_stop_metadata(
        self,
        session_dir: Path,
        files_metadata: dict[str, str] | None = None,
        rosbag_metadata: dict[str, object] | None = None,
        hr23_metadata: dict[str, object] | None = None,
    ) -> None:
        path = session_dir / "session.json"
        if not path.exists():
            return
        with path.open("r", encoding="utf-8") as handle:
            metadata = json.load(handle)
        metadata["stopped_at_iso"] = datetime.now().astimezone().isoformat(timespec="seconds")
        metadata["recording_stop_epoch_s"] = self._summary_recording_stop_epoch_s
        metadata["recording_stop_perf_s"] = self._summary_recording_stop_perf_s
        metadata["recording_gate_enabled_at_stop"] = self._summary_recording_gate_enabled
        if self._summary_recording_start_epoch_s is not None and self._summary_recording_stop_epoch_s is not None:
            metadata["duration_s"] = self._summary_recording_stop_epoch_s - self._summary_recording_start_epoch_s
        metadata["dropped_pre_start_ros_messages"] = self._summary_dropped_pre_start_ros_messages
        metadata["dropped_post_stop_ros_messages"] = self._summary_dropped_post_stop_ros_messages
        metadata["files"] = files_metadata if files_metadata is not None else self._summary_files_metadata()
        if rosbag_metadata is not None:
            metadata["rosbag"] = rosbag_metadata
        elif metadata.get("rosbag"):
            metadata["rosbag"] = self._summary_rosbag_metadata()
        if hr23_metadata is not None:
            metadata.setdefault("devices", {})["hr23_radar"] = hr23_metadata
        with path.open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)

    def _start_summary_rosbag(self, session_id: str) -> None:
        config = self._rosbag_panel.current_config()
        config["session_id"] = session_id
        config["action"] = "start_rosbag"
        self._summary_rosbag_session_id = session_id
        self._summary_rosbag_config = dict(config)
        try:
            self._ros_worker.request_rosbag_start(config)
            self._summary_rosbag_start_sent = True
            self._summary_rosbag_status_label.setText(f"rosbag 已发送开始: {session_id}")
        except Exception as exc:
            self._summary_last_warnings.append(f"rosbag_raw: start command failed: {exc}")
            self._summary_rosbag_status_label.setText("rosbag 开始失败")

    def _stop_summary_rosbag(self) -> None:
        if not self._summary_rosbag_session_id or self._summary_rosbag_stop_sent:
            return
        try:
            self._ros_worker.request_rosbag_stop(self._summary_rosbag_session_id)
            self._summary_rosbag_stop_sent = True
            self._summary_rosbag_status_label.setText(f"rosbag 已发送停止: {self._summary_rosbag_session_id}")
        except Exception as exc:
            self._summary_last_warnings.append(f"rosbag_raw: stop command failed: {exc}")
            self._summary_rosbag_status_label.setText("rosbag 停止失败")

    def _summary_rosbag_metadata(self) -> dict[str, object]:
        config = dict(self._summary_rosbag_config)
        latest_status = dict(self._summary_latest_rosbag_status)
        return {
            "enabled": self._summary_source_enabled("rosbag_raw") or bool(config),
            "session_id": self._summary_rosbag_session_id or str(config.get("session_id", "")),
            "bag_dir": config.get("bag_dir", DEFAULT_ROSBAG_REMOTE_DIR),
            "remote_dir": latest_status.get("remote_dir", ""),
            "topics": config.get("topics", []),
            "compression": config.get("compression", ""),
            "split_size_mb": config.get("split_size_mb", 0),
            "start_command_sent": self._summary_rosbag_start_sent,
            "stop_command_sent": self._summary_rosbag_stop_sent,
            "latest_status": latest_status,
            "remote_files": latest_status.get("bag_files", []),
            "duration_s": latest_status.get("duration_s", 0.0),
            "size_bytes": latest_status.get("current_size_bytes", 0),
            "downloaded": False,
            "local_dir": "",
        }

    def _summary_rosbag_metadata_from_context(self, context: dict[str, object]) -> dict[str, object] | None:
        config = dict(context.get("rosbag_config", {}))
        if not config:
            return None
        latest_status = dict(context.get("rosbag_latest_status", {}))
        return {
            "enabled": True,
            "session_id": str(context.get("rosbag_session_id", "") or config.get("session_id", "")),
            "bag_dir": config.get("bag_dir", DEFAULT_ROSBAG_REMOTE_DIR),
            "remote_dir": latest_status.get("remote_dir", ""),
            "topics": config.get("topics", []),
            "compression": config.get("compression", ""),
            "split_size_mb": config.get("split_size_mb", 0),
            "start_command_sent": bool(context.get("rosbag_start_sent", False)),
            "stop_command_sent": bool(context.get("rosbag_stop_sent", False)),
            "latest_status": latest_status,
            "remote_files": latest_status.get("bag_files", []),
            "duration_s": latest_status.get("duration_s", 0.0),
            "size_bytes": latest_status.get("current_size_bytes", 0),
            "downloaded": False,
            "local_dir": "",
        }

    def _summary_uses_ros(self) -> bool:
        return (
            self._summary_source_enabled("ros_odom")
            and self._summary_source("encoder") == "ros_odom"
        ) or any(
            self._summary_source_enabled("ros_imu" if key == "imu_A" else "ros_active_imu")
            and self._summary_source(key) == "ros_imu"
            for key in ("imu_A", "imu_B")
        )

    def _summary_uses_serial_imu(self) -> bool:
        return any(
            self._summary_source(key) == "serial" and self._summary_source_enabled(key)
            for key in ("imu_A", "imu_B")
        )

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
            "source_dir": self._summary_radar_source_dir_edit.text().strip(),
            "xml_path": self._summary_radar_xml_path_edit.text().strip(),
        }

    def _summary_files_metadata(self) -> dict[str, str]:
        files: dict[str, str] = {}
        if self._summary_source("encoder") == "serial" and self._summary_source_enabled("serial_encoder"):
            files["encoder"] = "encoder.csv"
        if self._summary_source_enabled("ros_odom"):
            files["ros_odom"] = "ros_odom.csv"
        if self._summary_source_enabled("fastlio_odometry"):
            files["fastlio_odometry"] = "fastlio_odometry.csv"
            files["trajectory_odometry"] = "trajectory_odometry.csv"
        if self._summary_uses_serial_imu():
            files.update({
                "imu_A": "imu_A.csv",
                "imu_B": "imu_B.csv",
                "imu_metadata": "imu_session.json",
                "imu_aligned": "imu_merged_aligned.csv",
            })
        if self._summary_source_enabled("ros_imu"):
            files["ros_imu"] = "ros_imu.csv"
        if self._summary_source_enabled("ros_active_imu"):
            files["ros_active_imu"] = "ros_active_imu.csv"
        if self._summary_source_enabled("ros_power_voltage"):
            files["ros_power_voltage"] = "ros_power_voltage.csv"
        if self._summary_source_enabled("akm_state"):
            files["akm_state"] = "akm_state.csv"
        if self._summary_source_enabled("control_debug"):
            files["control_debug"] = "control_debug.csv"
        if self._summary_source_enabled("chassis_diagnostics"):
            files["chassis_diagnostics"] = "chassis_diagnostics.csv"
        if self._summary_source_enabled("rosbag_raw"):
            files["rosbag_manifest"] = "raw/rosbag_manifest.json"
        if self._summary_source_enabled("radar_bin"):
            files["radar_dir"] = "raw/radar"
        if self._summary_source_enabled("hr23_radar"):
            files["hr23_radar_dir"] = "raw/hr23_radar"
        return files

    @staticmethod
    def _summary_imu_topic(key: str) -> str:
        return "/imu" if key == "imu_A" else "/active_imu"

    def _stop_summary_recording(self, save: bool, update_ui: bool = True) -> Path | None:
        context = self._detach_summary_stop_context(update_ui=update_ui)
        return self._finalize_summary_stop_context(context, save=save, update_ui=update_ui)

    def _detach_summary_stop_context(self, update_ui: bool = True) -> dict[str, object]:
        encoder_session = self._summary_encoder_session
        ros_session = self._summary_ros_session
        session_dir = self._summary_session_dir
        if session_dir is not None and self._summary_recording_stop_epoch_s is None:
            self._summary_recording_stop_epoch_s = time()
            self._summary_recording_stop_perf_s = perf_counter()
        self._summary_recording_gate_enabled = False
        if update_ui:
            self._set_summary_recording_state("STOPPING", "STOPPING")
        radar_was_recording = self._summary_radar_recording
        radar_started_epoch_s = self._summary_radar_started_epoch_s
        radar_started_session_elapsed_s = self._summary_radar_started_session_elapsed_s
        radar_stopped_session_elapsed_s = (
            self._summary_clock.elapsed_from_epoch(self._summary_recording_stop_epoch_s)
            if self._summary_clock is not None and self._summary_recording_stop_epoch_s is not None
            else None
        )
        topic_recorders = {
            topic: list(recorders)
            for topic, recorders in self._summary_topic_recorders.items()
        }
        radar_filename = self._summary_radar_filename
        radar_source_dir_text = self._summary_radar_source_dir_edit.text().strip()
        radar_xml_path_text = self._summary_radar_xml_path_edit.text().strip()
        self._stop_summary_rosbag()
        files_metadata = self._summary_files_metadata()
        hr23_active = self._summary_hr23_active
        hr23_client = self._summary_hr23_session_client
        hr23_metadata = dict(self._summary_hr23_session_metadata)
        stopped_encoder_session = self._buffer.stop_recording()
        if encoder_session is None:
            encoder_session = stopped_encoder_session
        imu_recorder = self._imu_panel.detach_recording_session()
        self._summary_encoder_session = None
        self._summary_ros_session = None
        self._summary_session_dir = None
        self._summary_radar_recording = False
        self._summary_radar_started_epoch_s = None
        self._summary_radar_started_session_elapsed_s = 0.0
        self._summary_radar_filename = ""
        rosbag_session_id = self._summary_rosbag_session_id
        rosbag_config = dict(self._summary_rosbag_config)
        rosbag_start_sent = self._summary_rosbag_start_sent
        rosbag_stop_sent = self._summary_rosbag_stop_sent
        rosbag_latest_status = dict(self._summary_latest_rosbag_status)
        self._summary_rosbag_session_id = ""
        self._summary_rosbag_config = {}
        self._summary_rosbag_start_sent = False
        self._summary_rosbag_stop_sent = False
        self._summary_latest_rosbag_status = {}
        self._summary_hr23_active = False
        self._summary_hr23_session_client = None
        self._summary_hr23_session_metadata = {}
        self._summary_clock = None
        self._summary_topic_recorders = {}

        if update_ui:
            self._record_btn.setText("开始记录")
            self._record_btn.setStyleSheet("")
            self._summary_record_btn.setText("全部开始记录")
            self._summary_record_btn.setStyleSheet("")

        return {
            "encoder_session": encoder_session,
            "ros_session": ros_session,
            "session_dir": session_dir,
            "radar_was_recording": radar_was_recording,
            "radar_started_epoch_s": radar_started_epoch_s,
            "radar_started_session_elapsed_s": radar_started_session_elapsed_s,
            "radar_stopped_session_elapsed_s": radar_stopped_session_elapsed_s,
            "radar_filename": radar_filename,
            "radar_source_dir_text": radar_source_dir_text,
            "radar_xml_path_text": radar_xml_path_text,
            "hr23_active": hr23_active,
            "hr23_client": hr23_client,
            "hr23_metadata": hr23_metadata,
            "topic_recorders": topic_recorders,
            "imu_recorder": imu_recorder,
            "files_metadata": files_metadata,
            "rosbag_session_id": rosbag_session_id,
            "rosbag_config": rosbag_config,
            "rosbag_start_sent": rosbag_start_sent,
            "rosbag_stop_sent": rosbag_stop_sent,
            "rosbag_latest_status": rosbag_latest_status,
        }

    def _finalize_summary_stop_context(
        self,
        context: dict[str, object],
        save: bool,
        update_ui: bool = True,
    ) -> Path | None:
        encoder_session = context["encoder_session"]
        ros_session = context["ros_session"]
        session_dir = context["session_dir"]
        radar_was_recording = bool(context["radar_was_recording"])
        radar_started_epoch_s = context["radar_started_epoch_s"]
        radar_started_session_elapsed_s = float(context["radar_started_session_elapsed_s"])
        radar_stopped_session_elapsed_s = context["radar_stopped_session_elapsed_s"]
        radar_filename = str(context["radar_filename"])
        radar_source_dir_text = str(context["radar_source_dir_text"])
        radar_xml_path_text = str(context["radar_xml_path_text"])
        topic_recorders = context["topic_recorders"]
        imu_recorder = context["imu_recorder"]
        files_metadata = context["files_metadata"]
        rosbag_metadata = self._summary_rosbag_metadata_from_context(context)
        hr23_active = bool(context["hr23_active"])
        hr23_client = context["hr23_client"]
        hr23_metadata = dict(context["hr23_metadata"])
        radar_stop_error = ""
        if radar_was_recording:
            try:
                self._radar_client.stop_recording()
                if update_ui:
                    self._summary_radar_status_label.setText("雷达记录已停止")
            except Exception as exc:
                radar_stop_error = f"雷达停止失败: {exc}"
                if update_ui:
                    self._summary_radar_status_label.setText(radar_stop_error)

        hr23_stop_error = ""
        if hr23_active:
            hr23_metadata["stop_cmd_send_epoch_s"] = time()
            try:
                stop_response = hr23_client.stop()
                hr23_metadata["stop_ack_recv_epoch_s"] = time()
                if stop_response.get("state") != "stopped":
                    raise Hr23RadarError(
                        "cmd=stop state="
                        f"{stop_response.get('state', '')} error=invalid state message=expected stopped"
                    )
                self._merge_hr23_response(hr23_metadata, stop_response)
                hr23_metadata["stop_response"] = dict(stop_response)
                self._update_summary_hr23_status(stop_response)
            except Exception as exc:
                hr23_stop_error = str(exc)
                hr23_metadata["stop_error"] = hr23_stop_error
                hr23_metadata["error"] = hr23_stop_error
                self._update_summary_hr23_status(error=hr23_stop_error)

        if save and session_dir is not None:
            if encoder_session is not None:
                encoder_session.finalize(session_dir / "encoder.csv")
            if imu_recorder is not None:
                imu_recorder.finalize()
            if ros_session is not None:
                ros_session.finalize()
            for recorders in topic_recorders.values():
                for recorder in recorders:
                    recorder.close()
            self._update_summary_stop_metadata(
                session_dir,
                files_metadata=files_metadata,
                rosbag_metadata=rosbag_metadata,
                hr23_metadata=hr23_metadata,
            )
            if hr23_stop_error:
                if update_ui:
                    self._set_summary_recording_state("ERROR", "HR2.3 stop 失败")
                    self._status_label.setText(f"HR2.3 stop 失败: {hr23_stop_error}")
                raise RuntimeError(f"HR2.3 stop 失败: {hr23_stop_error}")
            if update_ui:
                self._set_summary_recording_state("PACKAGING", "PACKAGING")
            if radar_was_recording:
                self._parse_summary_radar_outputs(
                    session_dir=session_dir,
                    host_start_epoch_s=radar_started_epoch_s,
                    radar_start_session_elapsed_s=radar_started_session_elapsed_s,
                    radar_stop_session_elapsed_s=radar_stopped_session_elapsed_s,
                    radar_filename=radar_filename,
                    source_dir_text=radar_source_dir_text,
                    xml_path_text=radar_xml_path_text,
                    update_ui=update_ui,
                )
            build_summary_package(session_dir)
            if update_ui:
                self._set_summary_recording_state("DONE", f"已保存: {session_dir}")
                if radar_stop_error:
                    self._status_label.setText(f"同步记录已保存，但{radar_stop_error}")
                else:
                    self._status_label.setText(f"同步记录已保存: {session_dir}")
            return session_dir

        if encoder_session is not None:
            encoder_session.cancel()
        if imu_recorder is not None:
            imu_recorder.cancel()
        if ros_session is not None:
            ros_session.cancel()
        for recorders in topic_recorders.values():
            for recorder in recorders:
                recorder.close()
        if update_ui:
            self._set_summary_recording_state("DONE", "同步记录已取消")
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

    def _publish_launch_manager_command(self, command: str) -> None:
        self._ros_worker.publish_launch_manager_command(command)
        self._status_label.setText(f"launch_manager 命令已发送: {command}")

    def _on_ros_connect(self, host: str, port: int, enabled_data_topics: list[str]) -> None:
        self._ros_worker.open_bridge(host, port, enabled_data_topics)

    def _on_ros_connection_changed(self, connected: bool) -> None:
        self._ros_connected = connected
        self._ros_topic_frame_counts.clear()
        self._summary_last_counts.pop("encoder", None)
        self._ros_panel.set_connected(connected)
        self._ros_imu_panel.set_connected(connected)
        self._summary_rosbag_status_label.setText("ROSbridge 已连接" if connected else "ROSbridge 未连接")
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

    def _on_ros_message(self, event) -> None:
        topic = event.get("topic", "")
        if topic:
            self._ros_topic_frame_counts[topic] = self._ros_topic_frame_counts.get(topic, 0) + 1
        if not self._is_summary_recording():
            return
        recorders = self._summary_topic_recorders.get(topic)
        if not recorders:
            return
        recv_time_epoch_s = event.get("recv_time_epoch_s")
        if recv_time_epoch_s is None:
            recv_time_epoch_s = time()
        recv_time_epoch_s = float(recv_time_epoch_s)
        if self._summary_recording_start_epoch_s is not None and recv_time_epoch_s < self._summary_recording_start_epoch_s:
            self._summary_dropped_pre_start_ros_messages += 1
            return
        if self._summary_recording_stop_epoch_s is not None and recv_time_epoch_s > self._summary_recording_stop_epoch_s:
            self._summary_dropped_post_stop_ros_messages += 1
            return
        if not self._summary_recording_gate_enabled or self._summary_recording_state not in {"STARTING", "RECORDING"}:
            return
        for recorder in recorders:
            recorder.write_message(
                event.get("message", {}),
                recv_time_epoch_s=recv_time_epoch_s,
            )

    def _on_launch_manager_status(self, payload) -> None:
        if not isinstance(payload, dict):
            return
        data = rosbag_protocol_data(payload)
        protocol_error = extract_rosbag_protocol_error(payload)
        if protocol_error:
            self._status_label.setText(f"rosbag 错误: {protocol_error}")
            self._rosbag_panel.append_log(protocol_error)
        if isinstance(data.get("rosbag"), dict):
            status = parse_rosbag_recording_status(payload)
            self._latest_rosbag_status = status
            self._rosbag_panel.update_recording_status(status)
            if self._summary_rosbag_session_id and status.session_id == self._summary_rosbag_session_id:
                self._summary_latest_rosbag_status = asdict(status)
            if status.last_error:
                self._status_label.setText(f"rosbag 错误: {status.last_error}")
        if isinstance(data.get("rosbag_library"), dict):
            library = parse_rosbag_library_state(payload)
            self._latest_rosbag_library = library
            self._rosbag_panel.update_library_state(library)

    def _start_rosbag_sync(self, session: RemoteRosbagSession) -> None:
        if not session.remote_dir:
            self._status_label.setText("rosbag 同步失败: 缺少远程目录")
            return
        if self._summary_session_dir is not None:
            local_dir = self._summary_session_dir / "raw" / "rosbag" / session.session_id
        else:
            base = Path(DEFAULT_RECORDINGS_DIR) / "rosbags"
            local_dir = base / session.session_id
        worker = RosbagSyncWorker(
            host=self._summary_rosbridge_host(),
            remote_dir=session.remote_dir,
            local_dir=local_dir,
        )
        thread = QThread(self)
        worker.moveToThread(thread)
        self._rosbag_sync_threads.append(thread)
        self._rosbag_sync_workers.append(worker)
        thread.started.connect(worker.run)
        worker.progress.connect(self._on_rosbag_sync_progress)
        worker.finished.connect(lambda result, session=session: self._on_rosbag_sync_finished(session, result))
        worker.error.connect(self._on_rosbag_sync_error)
        worker.finished.connect(thread.quit)
        worker.error.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        worker.error.connect(worker.deleteLater)
        thread.finished.connect(lambda: self._cleanup_rosbag_sync(thread, worker))
        thread.finished.connect(thread.deleteLater)
        thread.start()

    def _on_rosbag_sync_progress(self, message: str) -> None:
        self._rosbag_panel.append_log(message)
        self._status_label.setText(message)

    def _on_rosbag_sync_finished(self, session: RemoteRosbagSession, result: object) -> None:
        data = dict(result)
        self._rosbag_panel.append_log(f"同步完成: {data.get('method')} {data.get('local_dir')}")
        self._status_label.setText("rosbag 同步完成")
        self._update_rosbag_sync_metadata(session, data)
        self._ros_worker.request_rosbag_list(DEFAULT_ROSBAG_REMOTE_DIR)

    def _on_rosbag_sync_error(self, message: str) -> None:
        self._rosbag_panel.append_log(f"同步失败: {message}")
        self._status_label.setText(f"rosbag 同步失败: {message}")

    def _cleanup_rosbag_sync(self, thread: QThread, worker: RosbagSyncWorker) -> None:
        if thread in self._rosbag_sync_threads:
            self._rosbag_sync_threads.remove(thread)
        if worker in self._rosbag_sync_workers:
            self._rosbag_sync_workers.remove(worker)

    def _update_rosbag_sync_metadata(self, session: RemoteRosbagSession, result: dict[str, object]) -> None:
        session_dir = self._summary_session_dir
        if session_dir is None:
            return
        path = session_dir / "session.json"
        if not path.exists():
            return
        with path.open("r", encoding="utf-8") as handle:
            metadata = json.load(handle)
        rosbag = metadata.setdefault("rosbag", {})
        if isinstance(rosbag, dict):
            rosbag["downloaded"] = True
            rosbag["local_dir"] = str(result.get("local_dir", ""))
            rosbag["downloaded_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
            rosbag["sync_method"] = str(result.get("method", ""))
            rosbag["session_id"] = session.session_id
        with path.open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)

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
                frame_count=self._ros_topic_frame_counts.get("/odom", 0),
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
        for topic, recorders in self._summary_topic_recorders.items():
            rows = sum(int(getattr(recorder, "rows_written", 0)) for recorder in recorders)
            if rows:
                parts.append(f"{topic} {rows} 行")
        if self._summary_rosbag_session_id:
            parts.append(f"rosbag {self._summary_rosbag_session_id}")
        return "记录中: " + ", ".join(parts)

    def _summary_default_session_name(self) -> str:
        return f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    def _summary_session_timestamp(self) -> str:
        session_name = self._summary_session_name_edit.text().strip()
        if not session_name:
            return datetime.now().strftime("%Y%m%d_%H%M%S")
        if session_name.startswith("session_"):
            return session_name[len("session_"):]
        return session_name

    def _choose_summary_save_dir(self) -> None:
        current = self._summary_save_dir_edit.text().strip() or DEFAULT_RECORDINGS_DIR
        directory = QFileDialog.getExistingDirectory(self, "选择保存目录", current)
        if directory:
            self._summary_save_dir_edit.setText(directory)

    def _open_summary_save_dir(self) -> None:
        directory = Path(self._summary_save_dir_edit.text().strip() or DEFAULT_RECORDINGS_DIR)
        self._open_path(directory)

    def _open_path(self, path: Path) -> None:
        path.mkdir(parents=True, exist_ok=True)
        os.startfile(path)  # type: ignore[attr-defined]

    def _cancel_summary_recording(self) -> None:
        if self._is_summary_recording():
            self._stop_summary_recording_async(save=False)

    def _stop_summary_recording_async(self, save: bool) -> None:
        if self._summary_session_dir is not None and self._summary_recording_stop_epoch_s is None:
            self._summary_recording_stop_epoch_s = time()
            self._summary_recording_stop_perf_s = perf_counter()
        self._summary_recording_gate_enabled = False
        self._set_summary_recording_state("STOPPING", "STOPPING")
        context = self._detach_summary_stop_context(update_ui=True)
        self._run_background_task(
            lambda: self._finalize_summary_stop_context(context, save=save, update_ui=False),
            self._on_summary_stop_finished,
            self._on_summary_stop_error,
        )

    def _on_summary_stop_finished(self, result: object) -> None:
        if result:
            self._set_summary_recording_state("DONE", f"已保存: {result}")
        else:
            self._set_summary_recording_state("DONE", "同步记录已取消")

    def _on_summary_stop_error(self, message: str) -> None:
        self._set_summary_recording_state("ERROR", f"ERROR: {message}")
        self._status_label.setText(f"同步记录停止失败: {message}")

    def _on_trajectory_topic_changed(self) -> None:
        custom = self._trajectory_topic_combo.currentData() == "__custom__"
        self._trajectory_topic_custom_edit.setEnabled(custom)

    def _summary_trajectory_topic(self) -> str:
        current = self._trajectory_topic_combo.currentData()
        if current == "__custom__":
            return self._trajectory_topic_custom_edit.text().strip() or DEFAULT_FASTLIO_ODOM_TOPIC
        return str(current or DEFAULT_FASTLIO_ODOM_TOPIC)

    def _summary_source_enabled(self, source_id: str) -> bool:
        if source_id == "rosbag_raw":
            return self._summary_rosbag_sync_cb.isChecked()
        checkbox = getattr(self, "_summary_source_checks", {}).get(source_id)
        return True if checkbox is None else checkbox.isChecked()

    def _selected_summary_sources(self) -> list[str]:
        selected = [
            source_id
            for source_id, checkbox in getattr(self, "_summary_source_checks", {}).items()
            if checkbox.isChecked()
        ]
        if self._summary_source_enabled("rosbag_raw"):
            selected.append("rosbag_raw")
        return selected

    def _start_summary_source_check(self) -> None:
        self._summary_check_generation += 1
        generation = self._summary_check_generation
        self._set_summary_recording_state("CHECKING", "CHECKING: 正在检查 topic/设备...")
        self._summary_check_status_label.setText("正在检查 topic/设备...")
        QTimer.singleShot(
            self._SUMMARY_CHECK_TIMEOUT_MS,
            lambda generation=generation: self._on_summary_source_check_timeout(generation),
        )
        self._run_background_task(
            lambda: self._check_summary_sources(update_ui=False),
            lambda results, generation=generation: self._on_summary_source_check_finished(results, generation),
            lambda message, generation=generation: self._on_summary_source_check_error(message, generation),
        )

    def _on_summary_source_check_finished(self, results: object, generation: int | None = None) -> None:
        if generation is not None and generation != self._summary_check_generation:
            return
        if self._summary_recording_state != "CHECKING":
            return
        self._apply_summary_check_results(dict(results))
        self._set_summary_recording_state("READY", "READY")

    def _on_summary_source_check_error(self, message: str, generation: int | None = None) -> None:
        if generation is not None and generation != self._summary_check_generation:
            return
        if self._summary_recording_state != "CHECKING":
            return
        self._summary_check_status_label.setText(f"检查失败: {message}")
        self._set_summary_recording_state("ERROR", f"ERROR: {message}")

    def _on_summary_source_check_timeout(self, generation: int) -> None:
        if generation != self._summary_check_generation:
            return
        if self._summary_recording_state != "CHECKING":
            return
        self._summary_check_status_label.setText("检查超时: 请确认 ROSbridge/设备连接，或减少勾选的数据源后重试")
        self._set_summary_recording_state("ERROR", "ERROR: 检查超时")

    def _run_background_task(self, function, on_finished, on_error) -> None:
        thread = QThread(self)
        worker = _FunctionWorker(function)
        worker.moveToThread(thread)
        self._summary_background_threads.append(thread)
        self._summary_background_workers.append(worker)
        thread.started.connect(worker.run)
        worker.finished.connect(on_finished)
        worker.error.connect(on_error)
        worker.finished.connect(thread.quit)
        worker.error.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        worker.error.connect(worker.deleteLater)
        thread.finished.connect(lambda: self._cleanup_background_task(thread, worker))
        thread.finished.connect(thread.deleteLater)
        self._summary_background_thread = thread
        thread.start()

    def _cleanup_background_task(self, thread: QThread, worker: _FunctionWorker) -> None:
        if thread in self._summary_background_threads:
            self._summary_background_threads.remove(thread)
        if worker in self._summary_background_workers:
            self._summary_background_workers.remove(worker)

    def _check_summary_sources(self, update_ui: bool = True) -> dict[str, dict[str, object]]:
        results: dict[str, dict[str, object]] = {}
        ros_specs: list[dict[str, object]] = []
        if self._summary_source_enabled("serial_encoder"):
            results["serial_encoder"] = self._check_serial_source("encoder")
        if self._summary_source_enabled("imu_A"):
            results["imu_A"] = self._check_serial_source("imu_A")
        if self._summary_source_enabled("imu_B"):
            results["imu_B"] = self._check_serial_source("imu_B")
        if self._summary_source_enabled("fastlio_odometry"):
            ros_specs.append({
                "source_id": "fastlio_odometry",
                "topic": self._summary_trajectory_topic(),
                "expected_type": "nav_msgs/Odometry",
                "required_fields": ("pose.pose.position", "pose.pose.orientation"),
            })
        if self._summary_source_enabled("ros_odom"):
            ros_specs.append({"source_id": "ros_odom", "topic": "/odom", "expected_type": "nav_msgs/Odometry"})
        if self._summary_source_enabled("ros_imu"):
            ros_specs.append({"source_id": "ros_imu", "topic": "/imu", "expected_type": "sensor_msgs/Imu"})
        if self._summary_source_enabled("ros_active_imu"):
            ros_specs.append({"source_id": "ros_active_imu", "topic": "/active_imu", "expected_type": "sensor_msgs/Imu"})
        if self._summary_source_enabled("ros_power_voltage"):
            ros_specs.append({"source_id": "ros_power_voltage", "topic": "/PowerVoltage", "expected_type": "std_msgs/Float32"})
        if self._summary_source_enabled("akm_state"):
            ros_specs.append({
                "source_id": "akm_state",
                "topic": "/wheeltec/akm_state",
                "expected_type": "turn_on_wheeltec_robot/AkmState",
                "required_fields": ("header.frame_id",),
                "warning_hz_below": 80.0,
            })
        if self._summary_source_enabled("control_debug"):
            ros_specs.append({
                "source_id": "control_debug",
                "topic": "/wheeltec/control_debug",
                "expected_type": "turn_on_wheeltec_robot/ControlDebug",
                "required_fields": ("header.frame_id",),
                "warning_hz_below": 80.0,
            })
        if self._summary_source_enabled("chassis_diagnostics"):
            ros_specs.append({
                "source_id": "chassis_diagnostics",
                "topic": "/wheeltec/chassis_diagnostics",
                "expected_type": "turn_on_wheeltec_robot/ChassisDiagnostics",
                "required_fields": ("header.frame_id",),
                "warning_hz_below": 80.0,
            })
        if ros_specs:
            results.update(self._check_ros_sources(ros_specs))
        if self._summary_source_enabled("rosbag_raw"):
            results["rosbag_raw"] = {
                "status": "ok" if self._ros_connected else "offline",
                "estimated_hz": 0.0,
                "has_header_stamp": False,
                "messages_received": 0,
                "notes": "ROSbridge 已连接，可发送车端 rosbag 命令" if self._ros_connected else "ROS 未连接",
            }
        if self._summary_source_enabled("radar_bin"):
            results["radar_bin"] = self._check_radar_source()
        if self._summary_source_enabled("hr23_radar"):
            results["hr23_radar"] = self._check_hr23_radar_source()

        if update_ui:
            self._apply_summary_check_results(results)
        return results

    def _apply_summary_check_results(self, results: dict[str, dict[str, object]]) -> None:
        self._summary_last_check_results = results
        self._summary_last_check_epoch_s = time()
        self._summary_last_warnings = [
            f"{source_id}: {result.get('notes', '')}"
            for source_id, result in results.items()
            if result.get("status") == "warning"
        ]
        self._summary_last_errors = [
            f"{source_id}: {result.get('notes', '')}"
            for source_id, result in results.items()
            if result.get("status") == "error"
        ]
        offline = sum(1 for item in results.values() if item["status"] == "offline")
        skipped = sum(1 for item in results.values() if item["status"] == "skipped")
        warnings = sum(1 for item in results.values() if item["status"] == "warning")
        errors = sum(1 for item in results.values() if item["status"] == "error")
        self._summary_check_status_label.setText(
            f"检查完成: {len(results)} 项，offline {offline}，skipped {skipped}，warning {warnings}，error {errors}"
        )

    def _check_serial_source(self, key: str) -> dict[str, object]:
        port = self._summary_port(key).strip()
        connected = False
        frame_count = 0
        if key == "encoder":
            connected = self._serial_connected(self._worker)
            frame_count = self._buffer.frame_index
        else:
            device_key = "A" if key == "imu_A" else "B"
            device = self._imu_panel._devices[device_key]
            connected = self._serial_connected(device.worker)
            frame_count = device.buffer.frame_index
        status = "ok" if connected and frame_count > 0 else "offline"
        notes = "串口在线" if status == "ok" else ("未选择串口" if not port else "串口未在线")
        return {
            "status": status,
            "estimated_hz": 0.0,
            "has_header_stamp": False,
            "messages_received": frame_count,
            "notes": notes,
        }

    def _check_ros_source(
        self,
        topic: str,
        expected_type: str,
        required_fields: tuple[str, ...] = (),
        warning_hz_below: float | None = None,
    ) -> dict[str, object]:
        result = self._sample_ros_topic(
            topic=topic,
            expected_type=expected_type,
            required_fields=required_fields,
            warning_hz_below=warning_hz_below,
        )
        return {
            "status": result.status,
            "estimated_hz": result.estimated_hz,
            "has_header_stamp": result.has_header_stamp,
            "messages_received": result.messages_received,
            "notes": result.notes,
        }

    def _check_ros_sources(self, specs: list[dict[str, object]]) -> dict[str, dict[str, object]]:
        if not hasattr(self._sample_ros_topic, "__func__"):
            return {
                str(spec["source_id"]): self._check_ros_source(
                    topic=str(spec["topic"]),
                    expected_type=str(spec["expected_type"]),
                    required_fields=tuple(spec.get("required_fields", ())),
                    warning_hz_below=spec.get("warning_hz_below"),
                )
                for spec in specs
            }
        sampled = self._sample_ros_topics(specs)
        return {
            source_id: {
                "status": result.status,
                "estimated_hz": result.estimated_hz,
                "has_header_stamp": result.has_header_stamp,
                "messages_received": result.messages_received,
                "notes": result.notes,
            }
            for source_id, result in sampled.items()
        }

    def _sample_ros_topic(
        self,
        topic: str,
        expected_type: str,
        required_fields: tuple[str, ...] = (),
        warning_hz_below: float | None = None,
    ):
        return sample_ros_topic(
            topic=topic,
            expected_type=expected_type,
            host=self._summary_rosbridge_host(),
            port=self._summary_rosbridge_port(),
            required_fields=required_fields,
            warning_hz_below=warning_hz_below,
        )

    def _sample_ros_topics(self, specs: list[dict[str, object]]):
        return sample_ros_topics(
            host=self._summary_rosbridge_host(),
            port=self._summary_rosbridge_port(),
            topic_specs=specs,
        )

    def _summary_rosbridge_host(self) -> str:
        return str(getattr(self._ros_worker, "_host", "127.0.0.1"))

    def _summary_rosbridge_port(self) -> int:
        return int(getattr(self._ros_worker, "_port", 9090))

    def _check_radar_source(self) -> dict[str, object]:
        if not self._summary_radar_sync_cb.isChecked():
            return {
                "status": "skipped",
                "estimated_hz": 0.0,
                "has_header_stamp": False,
                "messages_received": 0,
                "notes": "未启用雷达同步",
            }
        source_dir_text = self._summary_radar_source_dir_edit.text().strip()
        xml_path_text = self._summary_radar_xml_path_edit.text().strip()
        try:
            response = self._radar_client.identify()
        except Exception as exc:
            return {
                "status": "offline",
                "estimated_hz": 0.0,
                "has_header_stamp": False,
                "messages_received": 0,
                "notes": str(exc),
            }
        notes = [response]
        status = "ok"
        if not source_dir_text or not Path(source_dir_text).exists():
            status = "warning"
            notes.append("未配置有效雷达输出目录")
        if not xml_path_text or not Path(xml_path_text).exists():
            status = "warning"
            notes.append("未配置有效 XML")
        return {
            "status": status,
            "estimated_hz": 0.0,
            "has_header_stamp": False,
            "messages_received": 0,
            "notes": "; ".join(notes),
        }

    def _check_hr23_radar_source(self) -> dict[str, object]:
        try:
            response = self._make_hr23_radar_client().status()
        except Exception as exc:
            self._update_summary_hr23_status(error=str(exc))
            return {
                "status": "offline",
                "estimated_hz": 0.0,
                "has_header_stamp": False,
                "messages_received": 0,
                "notes": str(exc),
            }
        self._update_summary_hr23_status(response)
        state = str(response.get("state", ""))
        status = "error" if state in {"recording", "stopping"} else "ok"
        packet_count = int(response.get("packetCount", 0) or 0)
        total_bytes = int(response.get("totalBytes", 0) or 0)
        return {
            "status": status,
            "estimated_hz": 0.0,
            "has_header_stamp": False,
            "messages_received": packet_count,
            "notes": f"state={state}, packetCount={packet_count}, totalBytes={total_bytes}",
        }

    def _create_summary_topic_recorders(self, session_dir: Path) -> dict[str, list[object]]:
        if self._summary_clock is None:
            return {}
        recorders: dict[str, list[object]] = {}
        if self._summary_source_enabled("fastlio_odometry"):
            trajectory_topic = self._summary_trajectory_topic()
            self._add_summary_topic_recorder(
                recorders,
                trajectory_topic,
                make_odometry_recorder(session_dir / "trajectory_odometry.csv", self._summary_clock),
            )
            if trajectory_topic == DEFAULT_FASTLIO_ODOM_TOPIC:
                self._add_summary_topic_recorder(
                    recorders,
                    trajectory_topic,
                    make_odometry_recorder(session_dir / "fastlio_odometry.csv", self._summary_clock),
                )
        if self._summary_source_enabled("ros_odom"):
            self._add_summary_topic_recorder(
                recorders,
                "/odom",
                make_ros_odom_compat_recorder(session_dir / "ros_odom.csv", self._summary_clock),
            )
        if self._summary_source_enabled("ros_imu"):
            self._add_summary_topic_recorder(
                recorders,
                "/imu",
                make_ros_imu_compat_recorder(session_dir / "ros_imu.csv", self._summary_clock),
            )
        if self._summary_source_enabled("ros_active_imu"):
            self._add_summary_topic_recorder(
                recorders,
                "/active_imu",
                make_ros_imu_compat_recorder(session_dir / "ros_active_imu.csv", self._summary_clock),
            )
        if self._summary_source_enabled("ros_power_voltage"):
            self._add_summary_topic_recorder(
                recorders,
                "/PowerVoltage",
                make_power_voltage_recorder(session_dir / "ros_power_voltage.csv", self._summary_clock),
            )
        if self._summary_source_enabled("akm_state"):
            self._add_summary_topic_recorder(
                recorders,
                "/wheeltec/akm_state",
                AkmStateRecorder(session_dir / "akm_state.csv", self._summary_clock),
            )
        if self._summary_source_enabled("control_debug"):
            self._add_summary_topic_recorder(
                recorders,
                "/wheeltec/control_debug",
                ControlDebugRecorder(session_dir / "control_debug.csv", self._summary_clock),
            )
        if self._summary_source_enabled("chassis_diagnostics"):
            self._add_summary_topic_recorder(
                recorders,
                "/wheeltec/chassis_diagnostics",
                ChassisDiagnosticsRecorder(session_dir / "chassis_diagnostics.csv", self._summary_clock),
            )
        return recorders

    @staticmethod
    def _add_summary_topic_recorder(
        recorders: dict[str, list[object]],
        topic: str,
        recorder: object,
    ) -> None:
        recorders.setdefault(topic, []).append(recorder)

    def _parse_summary_radar_outputs(
        self,
        session_dir: Path,
        host_start_epoch_s: float | None,
        radar_start_session_elapsed_s: float,
        radar_stop_session_elapsed_s: float | None,
        radar_filename: str | None = None,
        source_dir_text: str | None = None,
        xml_path_text: str | None = None,
        update_ui: bool = True,
    ) -> None:
        radar_filename = radar_filename if radar_filename is not None else self._summary_radar_filename
        if not radar_filename:
            return
        source_dir_text = (
            source_dir_text
            if source_dir_text is not None
            else self._summary_radar_source_dir_edit.text().strip()
        )
        xml_path_text = (
            xml_path_text
            if xml_path_text is not None
            else self._summary_radar_xml_path_edit.text().strip()
        )
        if not source_dir_text or not xml_path_text:
            return
        bin_path = Path(source_dir_text) / radar_filename
        xml_path = Path(xml_path_text)
        if not bin_path.exists() or not xml_path.exists():
            if update_ui:
                self._summary_radar_status_label.setText("雷达文件未找到，跳过解析")
            return
        output_dir = session_dir / "raw" / "radar"
        host_stop_epoch_s = datetime.now().timestamp()
        parse_radar_recording(
            bin_path=bin_path,
            xml_path=xml_path,
            output_dir=output_dir,
            session_id=session_dir.name,
            radar_start_session_elapsed_s=radar_start_session_elapsed_s,
            radar_stop_session_elapsed_s=radar_stop_session_elapsed_s,
            host_start_epoch_s=host_start_epoch_s if host_start_epoch_s is not None else host_stop_epoch_s,
            host_stop_epoch_s=host_stop_epoch_s,
        )
        if update_ui:
            self._summary_radar_status_label.setText(f"雷达已解析: {output_dir}")

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
        QTimer.singleShot(0, self._finalize_initial_screen_fit)
        self._initial_screen_fit_done = True

    def _finalize_initial_screen_fit(self) -> None:
        self._fit_window_to_screen()
        self._center_on_screen()

    def _fit_window_to_screen(self) -> None:
        screen = self.screen() or QGuiApplication.primaryScreen()
        if screen is None:
            return

        available = screen.availableGeometry()
        frame = self.frameGeometry()
        geometry = self.geometry()
        frame_extra_width = max(0, frame.width() - geometry.width())
        frame_extra_height = max(0, frame.height() - geometry.height())
        safety_margin = 8
        target_width = max(100, available.width() - frame_extra_width - safety_margin)
        target_height = max(100, available.height() - frame_extra_height - safety_margin)
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
        self._localization_panel.shutdown()
        self._imu_panel.shutdown()
        super().closeEvent(event)

"""
主窗口：布局编排、信号连接、定时器管理。
"""

from __future__ import annotations

import tempfile
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from analytics import compute_channel_metrics
from data_buffer import COL_FINAL_A, COL_FINAL_B, COL_TGT_A, COL_TGT_B, DataBuffer
from recording_session import RecordingSession
from replay_data import ReplayData
from serial_worker import SerialWorker
from widgets.analysis_panel import AnalysisPanel
from widgets.command_panel import CommandPanel
from widgets.data_panel import DataPanel
from widgets.param_panel import ParamPanel
from widgets.plot_panel import PlotPanel
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

        self._data_mode = "live"
        self._replay_data: ReplayData | None = None
        self._replay_current_time = 0.0
        self._replay_speed = 1.0
        self._initial_screen_fit_done = False

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
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
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

        self._status_label = QLabel("就绪")
        self.statusBar().addWidget(self._status_label, stretch=1)
        self._frame_label = QLabel("帧: 0")
        self.statusBar().addPermanentWidget(self._frame_label)
        self._error_label = QLabel("错误: 0")
        self.statusBar().addPermanentWidget(self._error_label)
        self._record_label = QLabel("")
        self.statusBar().addPermanentWidget(self._record_label)

        self._replay_controls_enabled(False)

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
        if self._buffer.recording:
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
        if self._buffer.recording:
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
        if self._buffer.recording:
            self._stop_recording_session(save=False)
        self._refresh_timer.stop()
        self._status_timer.stop()
        self._replay_timer.stop()
        self._worker.close_port()
        super().closeEvent(event)

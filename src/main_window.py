"""
主窗口：布局编排、信号连接、定时器管理。

数据流：
- SerialWorker (子线程) → DataBuffer (共享, Lock)
- QTimer 33ms (主线程) → 读 DataBuffer → 更新 PlotPanel + DataPanel
- CommandPanel → command_ready → SerialWorker.send_command
- SerialWorker.param_received → ParamPanel + CommandPanel
"""

from datetime import datetime
from pathlib import Path

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QSplitter,
    QPushButton, QHBoxLayout, QLabel, QFileDialog,
)
from PySide6.QtCore import Qt, QTimer

from data_buffer import DataBuffer
from serial_worker import SerialWorker
from widgets.serial_panel import SerialPanel
from widgets.plot_panel import PlotPanel
from widgets.data_panel import DataPanel
from widgets.command_panel import CommandPanel
from widgets.param_panel import ParamPanel


class MainWindow(QMainWindow):
    """调试监视器主窗口。"""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("WHEELTEC C50X 调试监视器")
        self.resize(1400, 850)

        # 共享数据缓冲区
        self._buffer = DataBuffer()

        # 串口工作线程（持有 buffer 引用）
        self._worker = SerialWorker(self._buffer)
        self._worker.param_received.connect(self._on_param)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.connection_changed.connect(self._on_connection_changed)

        self._setup_ui()
        self._setup_timers()

    def _setup_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        # ── 顶部：串口连接面板 ────────────────────────────
        self._serial_panel = SerialPanel()
        self._serial_panel.connect_requested.connect(self._on_connect)
        self._serial_panel.disconnect_requested.connect(self._on_disconnect)
        main_layout.addWidget(self._serial_panel)

        # ── 中部：水平 Splitter（绘图 | 数据+参数）─────────
        mid_splitter = QSplitter(Qt.Horizontal)

        # 左侧：绘图 + 记录控制
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)

        self._plot_panel = PlotPanel()
        left_layout.addWidget(self._plot_panel, stretch=1)

        # 记录 + 清空按钮行
        record_row = QHBoxLayout()
        record_row.addStretch()
        self._clear_btn = QPushButton("清空数据")
        self._clear_btn.clicked.connect(self._clear_data)
        record_row.addWidget(self._clear_btn)
        self._record_btn = QPushButton("开始记录")
        self._record_btn.clicked.connect(self._toggle_record)
        record_row.addWidget(self._record_btn)
        left_layout.addLayout(record_row)

        mid_splitter.addWidget(left_widget)

        # 右侧：数据面板 + 参数面板
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self._data_panel = DataPanel()
        right_layout.addWidget(self._data_panel)

        self._param_panel = ParamPanel()
        right_layout.addWidget(self._param_panel)

        right_layout.addStretch()

        mid_splitter.addWidget(right_widget)

        # 左侧占 70%，右侧占 30%
        mid_splitter.setStretchFactor(0, 7)
        mid_splitter.setStretchFactor(1, 3)

        main_layout.addWidget(mid_splitter, stretch=1)

        # ── 底部：命令面板 ────────────────────────────────
        self._command_panel = CommandPanel()
        self._command_panel.command_ready.connect(self._worker.send_command)
        main_layout.addWidget(self._command_panel)

        # ── 状态栏 ────────────────────────────────────────
        self._status_label = QLabel("就绪")
        self.statusBar().addWidget(self._status_label, stretch=1)
        self._frame_label = QLabel("帧: 0")
        self.statusBar().addPermanentWidget(self._frame_label)
        self._error_label = QLabel("错误: 0")
        self.statusBar().addPermanentWidget(self._error_label)
        self._record_label = QLabel("")
        self.statusBar().addPermanentWidget(self._record_label)

    def _setup_timers(self) -> None:
        """设置定时器。"""
        # 绘图 + 数值更新定时器（~30fps）
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._on_refresh)
        self._refresh_timer.start(33)

        # 状态栏更新定时器（1秒）
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(1000)

    # ─── 槽函数 ───────────────────────────────────────────

    def _on_connect(self, port: str, baudrate: int) -> None:
        self._buffer.clear()
        self._worker.open_port(port, baudrate)

    def _on_disconnect(self) -> None:
        if self._buffer.recording:
            self._toggle_record()  # 断开时自动停止记录（弹出保存对话框）
        self._worker.close_port()

    def _on_connection_changed(self, connected: bool) -> None:
        self._serial_panel.set_connected(connected)
        if connected:
            self._status_label.setText("已连接，等待数据...")
        else:
            self._status_label.setText("已断开")

    def _on_param(self, frame) -> None:
        """参数帧到达（低频信号，经 Qt Signal 传递）。"""
        self._param_panel.update_params(frame)
        self._command_panel.fill_params(frame)
        self._status_label.setText("已收到参数帧")

    def _on_error(self, msg: str) -> None:
        self._status_label.setText(f"错误: {msg}")

    def _on_refresh(self) -> None:
        """33ms 定时器回调：刷新绘图和数值面板。"""
        self._plot_panel.refresh(self._buffer)
        self._data_panel.refresh(self._buffer)

    def _update_status(self) -> None:
        """1秒定时器回调：更新状态栏计数。"""
        self._frame_label.setText(f"帧: {self._buffer.frame_index}")
        self._error_label.setText(f"错误: {self._worker.error_count}")
        if self._buffer.recording:
            self._record_label.setText(f"记录中: {self._buffer.csv_rows_written} 行")
        else:
            self._record_label.setText("")

    def _toggle_record(self) -> None:
        """切换 CSV 记录状态。开始时缓存到内存，停止时选择文件保存。"""
        if not self._buffer.recording:
            # 开始记录（仅内存缓存，不弹文件对话框）
            self._buffer.start_recording()
            self._record_btn.setText("停止记录")
            self._record_btn.setStyleSheet("background-color: #e74c3c; color: white;")
            self._status_label.setText("正在记录...")
        else:
            # 停止记录，弹出保存对话框
            default_name = f"debug_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            filepath, _ = QFileDialog.getSaveFileName(
                self, "保存记录数据", default_name, "CSV 文件 (*.csv)"
            )
            total = self._buffer.stop_recording(filepath if filepath else None)
            self._record_btn.setText("开始记录")
            self._record_btn.setStyleSheet("")
            if filepath:
                self._status_label.setText(f"记录已保存，共 {total} 行")
            else:
                self._status_label.setText(f"记录已丢弃（{total} 行）")

    def _clear_data(self) -> None:
        """清空缓冲区数据和图表。"""
        if self._buffer.recording:
            self._buffer.stop_recording(None)  # 丢弃记录中的数据
            self._record_btn.setText("开始记录")
            self._record_btn.setStyleSheet("")
        self._buffer.clear()
        self._plot_panel.reset()
        self._status_label.setText("数据已清空")

    # ─── 关闭事件 ─────────────────────────────────────────

    def closeEvent(self, event) -> None:
        """关闭窗口时清理资源。"""
        if self._buffer.recording:
            self._buffer.stop_recording(None)
        self._refresh_timer.stop()
        self._status_timer.stop()
        self._worker.close_port()
        super().closeEvent(event)

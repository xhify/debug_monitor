"""车端 ROSBag 管理面板。"""

from __future__ import annotations

import re

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from app_config import (
    DEFAULT_ROSBAG_COMPRESSION,
    DEFAULT_ROSBAG_REMOTE_DIR,
    DEFAULT_ROSBAG_SPLIT_SIZE_MB,
    ROSBAG_TOPIC_PRESETS,
)
from rosbag_models import (
    RemoteRosbagSession,
    RosbagLibraryState,
    RosbagRecordingStatus,
    format_bytes,
    format_duration,
)


class RosbagPanel(QWidget):
    start_requested = Signal(object)
    stop_requested = Signal(str)
    list_requested = Signal(str)
    inspect_requested = Signal(str)
    sync_requested = Signal(object)
    trash_requested = Signal(str)
    delete_requested = Signal(str, str)
    query_status_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._sessions: list[RemoteRosbagSession] = []
        self._recording_status = RosbagRecordingStatus()
        self._status_values: dict[str, QLabel] = {}
        self._setup_ui()
        self.update_recording_status(self._recording_status)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        layout.addWidget(self._build_status_group())
        layout.addWidget(self._build_config_group())
        layout.addWidget(self._build_action_group())
        layout.addWidget(self._build_table_group(), stretch=1)

        self._log_label = QLabel("")
        layout.addWidget(self._log_label)

    def _build_status_group(self) -> QGroupBox:
        group = QGroupBox("当前车端 rosbag")
        grid = QGridLayout(group)
        rows = [
            ("状态", "state"),
            ("session_id", "session_id"),
            ("远程目录", "remote_dir"),
            ("时长", "duration_s"),
            ("当前大小", "current_size_bytes"),
            ("磁盘剩余", "disk_free_gb"),
            ("文件数", "file_count"),
            ("topic 数", "topic_count"),
            ("错误", "last_error"),
        ]
        for row, (label, key) in enumerate(rows):
            grid.addWidget(QLabel(label), row, 0)
            value = QLabel("---")
            value.setTextInteractionFlags(value.textInteractionFlags())
            self._status_values[key] = value
            grid.addWidget(value, row, 1)
        return group

    def _build_config_group(self) -> QGroupBox:
        group = QGroupBox("录制配置")
        grid = QGridLayout(group)
        self._mode_combo = QComboBox()
        for mode in ("control", "fastlio", "trajectory_environment", "fallback_no_fastlio", "full", "custom"):
            self._mode_combo.addItem(mode, mode)
        self._mode_combo.setCurrentText("fastlio")
        self._mode_combo.currentTextChanged.connect(self._on_mode_changed)

        self._remote_dir_edit = QLineEdit(DEFAULT_ROSBAG_REMOTE_DIR)
        self._prefix_edit = QLineEdit("fastlio")
        self._split_size_spin = QSpinBox()
        self._split_size_spin.setRange(1, 1024 * 1024)
        self._split_size_spin.setValue(DEFAULT_ROSBAG_SPLIT_SIZE_MB)
        self._compression_combo = QComboBox()
        self._compression_combo.addItems(["lz4", "none"])
        self._compression_combo.setCurrentText(DEFAULT_ROSBAG_COMPRESSION)
        self._topic_count_label = QLabel("0 个 topic")
        self._topic_order = self._all_preset_topics()
        self._topic_checks: dict[str, QCheckBox] = {}
        topic_grid = QGridLayout()
        for index, topic in enumerate(self._topic_order):
            checkbox = QCheckBox(topic)
            checkbox.stateChanged.connect(self._update_topic_count_label)
            self._topic_checks[topic] = checkbox
            topic_grid.addWidget(checkbox, index // 3, index % 3)
        self._custom_topics_edit = QLineEdit()
        self._custom_topics_edit.setPlaceholderText("/custom_topic_a, /custom_topic_b")
        self._custom_topics_edit.textChanged.connect(self._update_topic_count_label)

        grid.addWidget(QLabel("模式"), 0, 0)
        grid.addWidget(self._mode_combo, 0, 1)
        grid.addWidget(QLabel("车端目录"), 0, 2)
        grid.addWidget(self._remote_dir_edit, 0, 3)
        grid.addWidget(QLabel("prefix"), 1, 0)
        grid.addWidget(self._prefix_edit, 1, 1)
        grid.addWidget(QLabel("split MB"), 1, 2)
        grid.addWidget(self._split_size_spin, 1, 3)
        grid.addWidget(QLabel("compression"), 2, 0)
        grid.addWidget(self._compression_combo, 2, 1)
        grid.addWidget(QLabel("topics"), 3, 0)
        grid.addWidget(self._topic_count_label, 3, 1, 1, 3)
        grid.addLayout(topic_grid, 4, 0, 1, 4)
        grid.addWidget(QLabel("自定义 topic"), 5, 0)
        grid.addWidget(self._custom_topics_edit, 5, 1, 1, 3)
        self._on_mode_changed("fastlio")
        return group

    def _build_action_group(self) -> QGroupBox:
        group = QGroupBox("操作")
        row = QHBoxLayout(group)
        self._start_btn = QPushButton("开始车端 rosbag")
        self._start_btn.clicked.connect(lambda: self.start_requested.emit(self.current_config()))
        self._stop_btn = QPushButton("停止车端 rosbag")
        self._stop_btn.clicked.connect(lambda: self.stop_requested.emit(self._recording_status.session_id))
        self._list_btn = QPushButton("刷新列表")
        self._list_btn.clicked.connect(lambda: self.list_requested.emit(self._remote_dir_edit.text().strip()))
        self._query_btn = QPushButton("查询状态")
        self._query_btn.clicked.connect(self.query_status_requested.emit)
        for button in (self._start_btn, self._stop_btn, self._list_btn, self._query_btn):
            row.addWidget(button)
        row.addStretch()
        return group

    def _build_table_group(self) -> QGroupBox:
        group = QGroupBox("车端 rosbag 列表")
        layout = QVBoxLayout(group)
        self._session_table = QTableWidget(0, 9)
        self._session_table.setHorizontalHeaderLabels(
            ["session_id", "状态", "时长", "大小", "文件数", "topic数", "创建时间", "已同步", "远程目录"]
        )
        self._session_table.setSelectionBehavior(QTableWidget.SelectRows)
        self._session_table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self._session_table)

        row = QHBoxLayout()
        buttons = [
            ("查看详情", self._inspect_selected),
            ("同步到 Windows", self._sync_selected),
            ("移到车端回收站", self._trash_selected),
            ("永久删除", self._delete_selected),
        ]
        for text, slot in buttons:
            button = QPushButton(text)
            button.clicked.connect(slot)
            row.addWidget(button)
        row.addStretch()
        layout.addLayout(row)
        return group

    def current_config(self) -> dict[str, object]:
        return {
            "action": "start_rosbag",
            "bag_dir": self._remote_dir_edit.text().strip() or DEFAULT_ROSBAG_REMOTE_DIR,
            "prefix": self._prefix_edit.text().strip() or "fastlio",
            "topics": self._topics(),
            "split_size_mb": self._split_size_spin.value(),
            "compression": self._compression_combo.currentText(),
        }

    def update_recording_status(self, status: RosbagRecordingStatus) -> None:
        self._recording_status = status
        self._status_values["state"].setText("录制中" if status.active else (status.state or "空闲"))
        self._status_values["session_id"].setText(status.session_id or "---")
        self._status_values["remote_dir"].setText(status.remote_dir or "---")
        self._status_values["duration_s"].setText(format_duration(status.duration_s))
        self._status_values["current_size_bytes"].setText(format_bytes(status.current_size_bytes))
        self._status_values["disk_free_gb"].setText(f"{status.disk_free_gb:.1f} GB")
        self._status_values["file_count"].setText(str(len(status.bag_files)))
        self._status_values["topic_count"].setText(str(len(status.topics)))
        self._status_values["last_error"].setText(status.last_error or "")

    def update_library_state(self, library: RosbagLibraryState) -> None:
        self._sessions = list(library.sessions)
        self._session_table.setRowCount(len(self._sessions))
        for row, session in enumerate(self._sessions):
            values = [
                session.session_id,
                session.status,
                format_duration(session.duration_s),
                format_bytes(session.size_bytes),
                str(session.file_count),
                str(session.topic_count),
                session.created_at,
                "是" if session.downloaded else "否",
                session.remote_dir,
            ]
            for col, value in enumerate(values):
                self._session_table.setItem(row, col, QTableWidgetItem(value))

    def append_log(self, text: str) -> None:
        self._log_label.setText(text)

    def request_delete_for_test(self, session: RemoteRosbagSession, confirm: str) -> bool:
        return self._request_delete(session, confirm)

    def _topics(self) -> list[str]:
        mode = str(self._mode_combo.currentData() or "")
        preset_order = ROSBAG_TOPIC_PRESETS.get(mode, [])
        topics = [topic for topic in preset_order if self._topic_checks.get(topic) and self._topic_checks[topic].isChecked()]
        for topic in self._topic_order:
            if topic not in topics and self._topic_checks[topic].isChecked():
                topics.append(topic)
        for topic in self._custom_topics():
            if topic not in topics:
                topics.append(topic)
        return topics

    def _custom_topics(self) -> list[str]:
        return [
            token
            for token in re.split(r"[,\s]+", self._custom_topics_edit.text().strip())
            if token
        ]

    def _update_topic_count_label(self) -> None:
        count = len(self._topics())
        self._topic_count_label.setText(f"{count} 个 topic")

    def _on_mode_changed(self, mode: str) -> None:
        if mode == "custom":
            return
        topics = ROSBAG_TOPIC_PRESETS.get(mode, [])
        for topic, checkbox in self._topic_checks.items():
            checkbox.setChecked(topic in topics)
        self._update_topic_count_label()
        if mode:
            self._prefix_edit.setText(mode)

    @staticmethod
    def _all_preset_topics() -> list[str]:
        topics: list[str] = []
        for preset_topics in ROSBAG_TOPIC_PRESETS.values():
            for topic in preset_topics:
                if topic not in topics:
                    topics.append(topic)
        return topics

    def _selected_session(self) -> RemoteRosbagSession | None:
        row = self._session_table.currentRow()
        if row < 0 or row >= len(self._sessions):
            return None
        return self._sessions[row]

    def _inspect_selected(self) -> None:
        session = self._selected_session()
        if session is not None:
            self.inspect_requested.emit(session.session_id)

    def _sync_selected(self) -> None:
        session = self._selected_session()
        if session is not None:
            self.sync_requested.emit(session)

    def _trash_selected(self) -> None:
        session = self._selected_session()
        if session is None:
            return
        if QMessageBox.question(self, "移到车端回收站", f"将 {session.session_id} 移到车端回收站？") == QMessageBox.Yes:
            self.trash_requested.emit(session.session_id)

    def _delete_selected(self) -> None:
        session = self._selected_session()
        if session is None:
            return
        text, ok = QInputDialog.getText(
            self,
            "永久删除 rosbag",
            f"永久删除不可恢复，未同步的数据会丢失。\n请输入完整 session_id 确认:\n{session.session_id}",
        )
        if ok and self._request_delete(session, text):
            self.append_log("已发送永久删除请求")

    def _request_delete(self, session: RemoteRosbagSession, confirm: str) -> bool:
        if self._recording_status.active and self._recording_status.session_id == session.session_id:
            self.append_log("正在录制的 session 不允许删除")
            return False
        if session.status == "recording":
            self.append_log("正在录制的 session 不允许删除")
            return False
        if confirm != session.session_id:
            self.append_log("确认 session_id 不匹配")
            return False
        self.delete_requested.emit(session.session_id, confirm)
        return True

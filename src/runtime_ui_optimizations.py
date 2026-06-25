"""Runtime UI optimizations for the recording summary workflow.

This module keeps the existing application structure intact and patches the
widgets at startup so the requested UI changes are isolated from the core data
recording logic.
"""

from __future__ import annotations

from time import time
from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from app_config import (
    DEFAULT_MAP_TOPIC,
    DEFAULT_MAP_UPDATE_PARAM,
    DEFAULT_ROSBRIDGE_HOST,
    DEFAULT_ROSBRIDGE_PORT,
)
from ros_odometry_client import parse_odometry_message
from widgets.localization_panel import LocalizationPanel

_SERIAL_SOURCE_KEYS = {"serial_encoder", "imu_A", "imu_B"}
_SERIAL_DEVICE_ROWS = {
    "serial_encoder": ("encoder", "编码器", [115200, 9600, 19200, 38400, 57600, 230400, 460800]),
    "imu_A": ("imu_A", "IMU A", [460800, 230400, 115200, 57600, 38400, 19200, 9600, 921600]),
    "imu_B": ("imu_B", "IMU B", [460800, 230400, 115200, 57600, 38400, 19200, 9600, 921600]),
}
_TOPIC_BY_SOURCE = {
    "fastlio_odometry": "/Odometry",
    "ros_odom": "/odom",
    "ros_imu": "/imu",
    "ros_active_imu": "/active_imu",
    "ros_power_voltage": "/PowerVoltage",
    "akm_state": "/wheeltec/akm_state",
    "control_debug": "/wheeltec/control_debug",
    "chassis_diagnostics": "/wheeltec/chassis_diagnostics",
}
_SOURCE_BY_TOPIC = {topic: source_id for source_id, topic in _TOPIC_BY_SOURCE.items()}


def apply_runtime_ui_optimizations(main_window_cls: type) -> None:
    """Install startup patches before ``MainWindow`` is instantiated."""

    _patch_localization_panel()
    _patch_main_window(main_window_cls)


def _patch_localization_panel() -> None:
    if getattr(LocalizationPanel, "_optimized_no_error_stats", False):
        return

    def _build_side_panel_without_error_stats(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setMinimumWidth(420)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        panel = QWidget()
        panel.setMinimumWidth(400)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(self._build_pose_group())
        layout.addWidget(self._build_control_group())
        _ensure_removed_stats_labels(self)
        layout.addWidget(self._build_feedback_group())
        layout.addStretch()
        scroll.setWidget(panel)
        return scroll

    LocalizationPanel._build_side_panel = _build_side_panel_without_error_stats
    LocalizationPanel._build_control_group = _build_localization_control_group
    LocalizationPanel._optimized_no_error_stats = True


def _build_localization_control_group(self) -> QGroupBox:
    group = QGroupBox("测试与建图控制")
    group.setMinimumHeight(300)
    layout = QVBoxLayout(group)
    layout.setContentsMargins(10, 14, 10, 10)
    layout.setSpacing(8)

    def add_button_row(button_specs: tuple[tuple[str, str, Any], ...]) -> None:
        row = QHBoxLayout()
        row.setSpacing(8)
        for attr, text, handler in button_specs:
            button = QPushButton(text)
            button.clicked.connect(handler)
            setattr(self, attr, button)
            row.addWidget(button)
        layout.addLayout(row)

    add_button_row(
        (
            ("_set_origin_btn", "设置当前为起点", self._set_current_origin),
            ("_clear_btn", "清空轨迹", self._clear),
        )
    )
    add_button_row(
        (
            ("_record_btn", "开始定位记录", self._toggle_recording),
            ("_report_btn", "生成测试摘要", self._write_report_dialog),
        )
    )
    add_button_row(
        (
            ("_mapping_freeze_btn", "冻结建图", self._toggle_mapping_freeze),
            ("_save_map_trajectory_btn", "保存地图与轨迹数据", self._write_frozen_package_dialog),
        )
    )
    add_button_row(
        (
            ("_fastlio_launch_start_btn", "启动 FAST-LIO", self._start_fastlio_launch),
            ("_fastlio_launch_stop_btn", "停止 FAST-LIO", self._stop_fastlio_launch),
        )
    )

    self._fastlio_launch_label = QLabel("")
    layout.addWidget(self._fastlio_launch_label)

    add_button_row(
        (
            ("_lidar_launch_start_btn", "启动雷达节点", self._start_lidar_launch),
            ("_lidar_launch_stop_btn", "停止雷达节点", self._stop_lidar_launch),
        )
    )

    self._lidar_launch_label = QLabel("")
    layout.addWidget(self._lidar_launch_label)

    config_grid = QGridLayout()
    config_grid.setContentsMargins(0, 2, 0, 0)
    config_grid.setHorizontalSpacing(10)
    config_grid.setVerticalSpacing(6)
    config_grid.setColumnMinimumWidth(0, 112)
    config_grid.setColumnStretch(1, 1)

    self._map_update_param_edit = QLineEdit(DEFAULT_MAP_UPDATE_PARAM)
    self._map_topic_edit = QLineEdit(DEFAULT_MAP_TOPIC)
    self._local_map_path_edit = QLineEdit("")
    for row_index, (title, edit) in enumerate(
        (
            ("建图参数:", self._map_update_param_edit),
            ("地图 topic:", self._map_topic_edit),
            ("本地地图路径:", self._local_map_path_edit),
        )
    ):
        label = QLabel(title)
        label.setMinimumWidth(112)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        edit.setMinimumWidth(280)
        edit.setMinimumHeight(24)
        edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        config_grid.addWidget(label, row_index, 0)
        config_grid.addWidget(edit, row_index, 1)
    layout.addLayout(config_grid)

    self._map_label = QLabel("建图未冻结")
    layout.addWidget(self._map_label)
    self._record_label = QLabel("")
    layout.addWidget(self._record_label)
    return group


def _ensure_removed_stats_labels(panel: Any) -> None:
    for key in (
        "lateral_error_current",
        "lateral_error_rms",
        "lateral_error_max",
        "endpoint_lateral_error",
        "endpoint_distance",
        "stats_trajectory_length",
        "yaw_rms",
        "estimated_speed_mean",
        "estimated_speed_std",
    ):
        panel._labels.setdefault(key, QLabel("---"))


def _patch_main_window(cls: type) -> None:
    if getattr(cls, "_optimized_summary_ui", False):
        return

    original_setup_ui = cls._setup_ui
    original_on_ros_message = cls._on_ros_message
    original_on_ros_connection_changed = cls._on_ros_connection_changed
    def _setup_ui(self):
        original_setup_ui(self)
        _unify_rosbridge_controls(self)
        _set_ros_related_refresh_intervals(self)
        _connect_ros_latency_signal(self)
        _update_rosbridge_status(self, connected=getattr(self, "_ros_connected", False))

    def _build_summary_source_group(self):
        group = QGroupBox("可记录数据源")
        layout = QGridLayout(group)
        self._summary_source_checks = {}
        self._summary_source_rate_labels = {}
        self._summary_source_detail_labels = {}

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
            ("hr23_radar", "新谐波雷达 HR2.3"),
        ]

        for row_index, (source_id, label) in enumerate(source_items):
            checkbox = self._make_summary_source_checkbox(source_id, label)
            if source_id == "hr23_radar":
                self._summary_source_checks[source_id] = checkbox
                layout.addWidget(checkbox, row_index, 0)
                layout.addWidget(self._build_summary_hr23_controls(), row_index, 1, 1, 2)
                continue
            rate_label = QLabel("---")
            detail_label = QLabel("")
            detail_label.setStyleSheet("color: #666;")
            self._summary_source_checks[source_id] = checkbox
            self._summary_source_rate_labels[source_id] = rate_label
            self._summary_source_detail_labels[source_id] = detail_label

            layout.addWidget(checkbox, row_index, 0)
            layout.addWidget(rate_label, row_index, 1)

            if source_id in _SERIAL_DEVICE_ROWS:
                key, _title, bauds = _SERIAL_DEVICE_ROWS[source_id]
                controls = _create_inline_serial_controls(self, key, bauds)
                layout.addWidget(controls, row_index, 2)
            else:
                layout.addWidget(detail_label, row_index, 2)

        self._summary_rosbridge_status_label = QLabel("")
        self._summary_rosbridge_status_label.setStyleSheet("color: #555;")
        layout.addWidget(self._summary_rosbridge_status_label, len(source_items), 0, 1, 3)
        return group

    def _make_summary_source_checkbox(self, source_id: str, label: str):
        from PySide6.QtWidgets import QCheckBox

        checkbox = QCheckBox(label)
        checkbox.setChecked(False)
        return checkbox

    def _build_summary_device_group(self, key: str, title: str, bauds: list[int]):
        if key not in self._summary_rows:
            _ensure_summary_row(self, key, bauds)
        group = QGroupBox(title)
        group.setVisible(False)
        group.setMaximumHeight(0)
        return group

    def _connect_summary_device(self, key: str):
        if self._summary_source(key) != "serial":
            _open_unified_rosbridge(self)
            return
        return _connect_serial_summary_device(self, key)

    def _disconnect_summary_device(self, key: str):
        if self._summary_source(key) != "serial":
            self._ros_worker.close_bridge()
            return
        return _disconnect_serial_summary_device(self, key)

    def _on_ros_connection_changed(self, connected: bool):
        original_on_ros_connection_changed(self, connected)
        _set_localization_connected_state(self, connected)
        _update_rosbridge_status(self, connected=connected)

    def _on_ros_message(self, event):
        original_on_ros_message(self, event)
        topic = event.get("topic", "") if isinstance(event, dict) else ""
        if topic:
            _update_topic_rate_label_for_event(self, event)
            _update_localization_from_ros_message(self, event)
        _update_ros_latency(self, event)

    def _update_summary_row(self, key: str, connected: bool, frame_count: int, error_count: int):
        row = self._summary_rows[key]
        hz_text = self._summary_hz_text(key, frame_count)
        row["status_label"].setText("正常接收" if connected and frame_count > 0 else ("已连接" if connected else "未连接"))
        row["frame_label"].setText(str(frame_count))
        row["error_label"].setText(str(error_count))
        row["hz_label"].setText(hz_text)
        row["connect_btn"].setEnabled(not connected)
        row["disconnect_btn"].setEnabled(connected)
        serial_source = self._summary_source(key) == "serial"
        row["port_combo"].setEnabled(serial_source and not connected)
        row["baud_combo"].setEnabled(serial_source and not connected)
        row["refresh_btn"].setEnabled(serial_source and not connected)
        source_id = {"encoder": "serial_encoder", "imu_A": "imu_A", "imu_B": "imu_B"}.get(key)
        rate_labels = getattr(self, "_summary_source_rate_labels", {})
        detail_labels = getattr(self, "_summary_source_detail_labels", {})
        if source_id and source_id in rate_labels:
            rate_labels[source_id].setText(hz_text)
        if source_id and source_id in detail_labels:
            detail_labels[source_id].setText(
                f"帧 {frame_count} / 错误 {error_count} / {row['status_label'].text()}"
            )

    def _apply_summary_check_results(self, results: dict[str, dict[str, object]]):
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
        for source_id, result in results.items():
            rate_label = getattr(self, "_summary_source_rate_labels", {}).get(source_id)
            detail_label = getattr(self, "_summary_source_detail_labels", {}).get(source_id)
            hz = float(result.get("estimated_hz", 0.0) or 0.0)
            status = str(result.get("status", ""))
            notes = str(result.get("notes", ""))
            messages = int(result.get("messages_received", 0) or 0)
            if rate_label is not None:
                rate_label.setText(f"{hz:.1f} Hz" if hz > 0 else "---")
            if detail_label is not None:
                detail_label.setText(f"{status} / {messages} 条" + (f" / {notes}" if notes else ""))
        offline = sum(1 for item in results.values() if item["status"] == "offline")
        skipped = sum(1 for item in results.values() if item["status"] == "skipped")
        warnings = sum(1 for item in results.values() if item["status"] == "warning")
        errors = sum(1 for item in results.values() if item["status"] == "error")
        self._summary_check_status_label.setText(
            f"检查完成: {len(results)} 项，offline {offline}，skipped {skipped}，warning {warnings}，error {errors}"
        )

    cls._setup_ui = _setup_ui
    cls._build_summary_source_group = _build_summary_source_group
    cls._make_summary_source_checkbox = _make_summary_source_checkbox
    cls._build_summary_device_group = _build_summary_device_group
    cls._connect_summary_device = _connect_summary_device
    cls._disconnect_summary_device = _disconnect_summary_device
    cls._on_ros_connection_changed = _on_ros_connection_changed
    cls._on_ros_message = _on_ros_message
    cls._update_summary_row = _update_summary_row
    cls._apply_summary_check_results = _apply_summary_check_results
    cls._optimized_summary_ui = True


def _ensure_summary_row(window: Any, key: str, bauds: list[int]) -> dict[str, object]:
    if key in window._summary_rows:
        return window._summary_rows[key]

    source_combo = QComboBox()
    source_combo.addItem("串口", "serial")
    if key == "encoder":
        source_combo.addItem("ROS /odom", "ros_odom")
    else:
        source_combo.addItem("ROS IMU", "ros_imu")

    port_combo = QComboBox()
    port_combo.setEditable(True)
    port_combo.setMinimumWidth(125)
    baud_combo = QComboBox()
    baud_combo.setMinimumWidth(85)
    for baud in bauds:
        baud_combo.addItem(str(baud), baud)

    refresh_btn = QPushButton("刷新")
    refresh_btn.clicked.connect(lambda _checked=False, row_key=key: window._refresh_summary_ports(row_key))
    connect_btn = QPushButton("连接")
    connect_btn.clicked.connect(lambda _checked=False, row_key=key: window._connect_summary_device(row_key))
    disconnect_btn = QPushButton("断开")
    disconnect_btn.clicked.connect(lambda _checked=False, row_key=key: window._disconnect_summary_device(row_key))
    disconnect_btn.setEnabled(False)

    row = {
        "source_combo": source_combo,
        "port_combo": port_combo,
        "baud_combo": baud_combo,
        "refresh_btn": refresh_btn,
        "connect_btn": connect_btn,
        "disconnect_btn": disconnect_btn,
        "status_label": QLabel("未连接"),
        "frame_label": QLabel("0"),
        "error_label": QLabel("0"),
        "hz_label": QLabel("---"),
    }
    window._summary_rows[key] = row
    return row


def _create_inline_serial_controls(window: Any, key: str, bauds: list[int]) -> QWidget:
    row = _ensure_summary_row(window, key, bauds)
    widget = QWidget()
    layout = QHBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(4)
    for label_text, control_name in (("COM", "port_combo"), ("波特率", "baud_combo")):
        layout.addWidget(QLabel(label_text))
        layout.addWidget(row[control_name])
    layout.addWidget(row["refresh_btn"])
    layout.addWidget(row["connect_btn"])
    layout.addWidget(row["disconnect_btn"])
    layout.addWidget(row["status_label"])
    layout.addWidget(row["frame_label"])
    layout.addWidget(row["error_label"])
    layout.addWidget(row["hz_label"])
    return widget


def _connect_serial_summary_device(window: Any, key: str) -> None:
    port = window._summary_port(key)
    if not port:
        window._summary_rows[key]["status_label"].setText("请选择串口")
        return
    baudrate = window._summary_baudrate(key)
    if key == "encoder":
        window._set_combo_value(window._serial_panel._port_combo, port)
        window._set_combo_value(window._serial_panel._baud_combo, str(baudrate))
        window._on_connect(port, baudrate)
        return
    device_key = "A" if key == "imu_A" else "B"
    device = window._imu_panel._devices[device_key]
    window._set_combo_value(device.port_combo, port)
    window._set_combo_value(device.baud_combo, str(baudrate))
    window._imu_panel._on_connect(device_key)


def _disconnect_serial_summary_device(window: Any, key: str) -> None:
    if key == "encoder":
        window._on_disconnect()
        return
    device_key = "A" if key == "imu_A" else "B"
    window._imu_panel._on_disconnect(device_key)


def _unify_rosbridge_controls(window: Any) -> None:
    window._ros_panel._host_edit.setText(getattr(window._ros_panel, "_host_edit").text().strip() or DEFAULT_ROSBRIDGE_HOST)
    host_edit = window._ros_panel._host_edit
    port_spin = window._ros_panel._port_spin

    # Keep ROS IMU and localization host/port controls visually synchronized with
    # the single shared ROSbridge endpoint used by the ROS worker.
    for panel in (getattr(window, "_ros_imu_panel", None), getattr(window, "_localization_panel", None)):
        if panel is None:
            continue
        if hasattr(panel, "_host_edit"):
            panel._host_edit.setText(host_edit.text())
            panel._host_edit.setEnabled(False)
        if hasattr(panel, "_port_spin"):
            panel._port_spin.setValue(port_spin.value())
            panel._port_spin.setEnabled(False)
        if hasattr(panel, "_connect_btn"):
            if panel is getattr(window, "_localization_panel", None):
                panel._connect_btn.setEnabled(False)
                panel._connect_btn.setToolTip("定位页使用 ROS 页的共享 ROSbridge 连接")
            else:
                try:
                    panel._connect_btn.clicked.disconnect()
                except Exception:
                    pass
                panel._connect_btn.clicked.connect(lambda _checked=False, w=window: _open_unified_rosbridge(w))
        if hasattr(panel, "_disconnect_btn"):
            if panel is getattr(window, "_localization_panel", None):
                panel._disconnect_btn.setEnabled(False)
                panel._disconnect_btn.setToolTip("请在 ROS 页断开共享 ROSbridge 连接")
            else:
                try:
                    panel._disconnect_btn.clicked.disconnect()
                except Exception:
                    pass
                panel._disconnect_btn.clicked.connect(window._ros_worker.close_bridge)
        if panel is getattr(window, "_localization_panel", None):
            _wire_localization_to_unified_rosbridge(window, panel)

    host_edit.textChanged.connect(lambda text: _sync_unified_rosbridge_endpoint(window))
    port_spin.valueChanged.connect(lambda _value: _sync_unified_rosbridge_endpoint(window))
    _set_localization_connected_state(window, getattr(window, "_ros_connected", False))


def _wire_localization_to_unified_rosbridge(window: Any, panel: Any) -> None:
    if hasattr(panel, "_connection_label"):
        panel._connection_label.setMinimumWidth(300)
        panel._connection_label.setWordWrap(True)
    for edit_name in ("_map_update_param_edit", "_map_topic_edit", "_local_map_path_edit"):
        edit = getattr(panel, edit_name, None)
        if edit is not None:
            edit.setMinimumWidth(280)
            edit.setMinimumHeight(max(edit.minimumHeight(), 24))
            edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            group = _ancestor_group(edit)
            if group is not None:
                group.setMinimumHeight(max(group.minimumHeight(), 300, group.sizeHint().height()))
    for form in panel.findChildren(QFormLayout):
        form.setHorizontalSpacing(12)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        if _form_contains_widgets(
            form,
            {
                getattr(panel, "_map_update_param_edit", None),
                getattr(panel, "_map_topic_edit", None),
                getattr(panel, "_local_map_path_edit", None),
            },
        ):
            form.setRowWrapPolicy(QFormLayout.DontWrapRows)
            form.setVerticalSpacing(6)
            form.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
            _style_form_labels(form, minimum_width=112)
    for label in panel.findChildren(QLabel):
        if label.text() in {"建图参数:", "地图 topic:", "本地地图路径:"}:
            label.setMinimumWidth(112)
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
def _form_contains_widgets(form: QFormLayout, widgets: set[Any]) -> bool:
    widgets.discard(None)
    for row in range(form.rowCount()):
        item = form.itemAt(row, QFormLayout.FieldRole)
        if item is not None and item.widget() in widgets:
            return True
    return False


def _style_form_labels(form: QFormLayout, minimum_width: int) -> None:
    for row in range(form.rowCount()):
        item = form.itemAt(row, QFormLayout.LabelRole)
        label = item.widget() if item is not None else None
        if label is not None:
            label.setMinimumWidth(minimum_width)
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)


def _ancestor_group(widget: Any) -> QGroupBox | None:
    parent = widget.parent()
    while parent is not None:
        if isinstance(parent, QGroupBox):
            return parent
        parent = parent.parent()
    return None


def _set_localization_connected_state(window: Any, connected: bool) -> None:
    panel = getattr(window, "_localization_panel", None)
    if panel is None or not hasattr(panel, "_set_connected"):
        return
    panel._set_connected(connected)
    if hasattr(panel, "_host_edit"):
        panel._host_edit.setEnabled(False)
    if hasattr(panel, "_port_spin"):
        panel._port_spin.setEnabled(False)


def _set_ros_related_refresh_intervals(window: Any) -> None:
    for panel in (
        getattr(window, "_ros_panel", None),
        getattr(window, "_ros_imu_panel", None),
        getattr(window, "_localization_panel", None),
    ):
        timer = getattr(panel, "_refresh_timer", None)
        if timer is not None:
            timer.setInterval(1000)


def _connect_ros_latency_signal(window: Any) -> None:
    signal = getattr(getattr(window, "_ros_worker", None), "network_latency_measured", None)
    if signal is None:
        return
    signal.connect(lambda latency_ms, w=window: _on_network_latency_measured(w, latency_ms))


def _on_network_latency_measured(window: Any, latency_ms: float) -> None:
    try:
        value = max(0.0, float(latency_ms))
    except (TypeError, ValueError):
        return
    window._ros_network_latency_text = f"网络延迟: {value:.1f} ms"
    _update_rosbridge_status(window, connected=getattr(window, "_ros_connected", False))


def _sync_unified_rosbridge_endpoint(window: Any) -> None:
    host = window._ros_panel._host_edit.text().strip() or DEFAULT_ROSBRIDGE_HOST
    port = window._ros_panel._port_spin.value()
    for panel in (getattr(window, "_ros_imu_panel", None), getattr(window, "_localization_panel", None)):
        if panel is None:
            continue
        if hasattr(panel, "_host_edit"):
            panel._host_edit.setText(host)
        if hasattr(panel, "_port_spin"):
            panel._port_spin.setValue(port)
    _update_rosbridge_status(window, connected=getattr(window, "_ros_connected", False))


def _open_unified_rosbridge(window: Any) -> None:
    host = window._ros_panel._host_edit.text().strip() or DEFAULT_ROSBRIDGE_HOST
    port = window._ros_panel._port_spin.value()
    selected_topics = (
        window._ros_panel.selected_data_topics()
        if hasattr(window._ros_panel, "selected_data_topics")
        else []
    )
    window._ros_worker.open_bridge(host, port, selected_topics)
    _update_rosbridge_status(window, connected=getattr(window, "_ros_connected", False))


def _update_rosbridge_status(window: Any, connected: bool) -> None:
    host = window._ros_panel._host_edit.text().strip() if hasattr(window, "_ros_panel") else DEFAULT_ROSBRIDGE_HOST
    port = window._ros_panel._port_spin.value() if hasattr(window, "_ros_panel") else DEFAULT_ROSBRIDGE_PORT
    network_latency = getattr(window, "_ros_network_latency_text", "网络延迟: ---")
    message_timing = getattr(window, "_ros_message_timing_text", "")
    timing = f"{network_latency} / {message_timing}" if message_timing else network_latency
    text = f"ROSbridge: {'已连接' if connected else '未连接'} {host}:{port} / {timing} / 错误 {window._ros_worker.error_count}"
    label = getattr(window, "_summary_rosbridge_status_label", None)
    if label is not None:
        label.setText(text)
    for panel in (getattr(window, "_ros_panel", None), getattr(window, "_ros_imu_panel", None), getattr(window, "_localization_panel", None)):
        _set_panel_connection_text(panel, text)


def _set_panel_connection_text(panel: Any, text: str) -> None:
    if panel is None:
        return
    label = getattr(panel, "_connection_label", None) or getattr(panel, "_status_label", None)
    if label is not None:
        label.setText(text)


def _update_ros_latency(window: Any, event: Any) -> None:
    if not isinstance(event, dict):
        return
    message = event.get("message", {})
    header = message.get("header", {}) if isinstance(message, dict) else {}
    stamp = header.get("stamp", {}) if isinstance(header, dict) else {}
    if not isinstance(stamp, dict):
        return
    secs = stamp.get("secs", stamp.get("sec", None))
    nsecs = stamp.get("nsecs", stamp.get("nanosec", 0))
    try:
        ros_stamp = float(secs) + float(nsecs) / 1_000_000_000.0
    except (TypeError, ValueError):
        return
    if ros_stamp <= 0:
        return
    recv_time = float(event.get("recv_time_epoch_s", time()))
    latency_ms = (recv_time - ros_stamp) * 1000.0
    topic = str(event.get("topic", ""))
    if abs(latency_ms) > 5000.0 and getattr(window, "_ros_message_timing_text", ""):
        _remember_ros_timestamp_outlier(window, topic, latency_ms)
        return
    if latency_ms < 0.0 or latency_ms > 5000.0:
        sign = "+" if latency_ms > 0.0 else ""
        text = f"时钟差: {sign}{latency_ms:.1f} ms"
    else:
        text = f"消息时差: {latency_ms:.1f} ms"
    last_update = getattr(window, "_optimized_latency_ui_time", None)
    if last_update is not None and recv_time - last_update < 1.0:
        return
    window._optimized_latency_ui_time = recv_time
    window._ros_message_timing_text = text
    _update_rosbridge_status(window, connected=getattr(window, "_ros_connected", False))


def _remember_ros_timestamp_outlier(window: Any, topic: str, latency_ms: float) -> None:
    outliers = getattr(window, "_ros_timestamp_outliers", None)
    if outliers is None:
        outliers = {}
        window._ros_timestamp_outliers = outliers
    outliers[topic or "<unknown>"] = float(latency_ms)


def _update_topic_rate_label_for_event(window: Any, event: Any) -> None:
    if not isinstance(event, dict):
        return
    topic = str(event.get("topic", ""))
    source_id = _SOURCE_BY_TOPIC.get(topic)
    if not source_id:
        return
    labels = getattr(window, "_summary_source_rate_labels", {})
    label = labels.get(source_id)
    if label is None:
        return
    try:
        recv_time = float(event.get("recv_time_epoch_s", time()))
    except (TypeError, ValueError):
        recv_time = time()
    count = int(window._ros_topic_frame_counts.get(topic, 0))
    state = getattr(window, "_optimized_topic_rate_state", None)
    if state is None:
        state = {}
        window._optimized_topic_rate_state = state
    previous = state.get(topic)
    if previous is None:
        state[topic] = (recv_time, 0, "0 Hz")
        label.setText("0 Hz")
        return
    previous_time, previous_count, previous_text = previous
    elapsed = recv_time - previous_time
    if count <= previous_count or elapsed < 1.0:
        label.setText(previous_text)
        return
    hz = (count - previous_count) / elapsed
    text = f"{max(0.0, hz):.0f} Hz"
    state[topic] = (recv_time, count, text)
    label.setText(text)


def _update_localization_from_ros_message(window: Any, event: Any) -> None:
    if not isinstance(event, dict):
        return
    if event.get("localization_sample_emitted"):
        return
    if event.get("topic") != window._summary_trajectory_topic():
        return
    panel = getattr(window, "_localization_panel", None)
    if panel is None:
        return
    message = event.get("message", {})
    if not isinstance(message, dict):
        return
    sample = parse_odometry_message(
        message,
        source=str(event.get("topic", "")),
        recv_time=float(event.get("recv_time_epoch_s", time())),
    )
    panel._on_sample(sample)

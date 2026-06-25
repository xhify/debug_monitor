"""FAST-LIO2 localization stability and frozen-map fusion panel."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import tempfile

from PySide6.QtCore import QSignalBlocker, QSize, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
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

from localization_buffer import LocalizationBuffer, LocalizationSample
from localization_fusion import (
    MapPoint,
    export_frozen_map_trajectory_zip,
)
from map_fetch_client import MapFetchClient, MapFetchConfig
from mapping_update_client import (
    MappingUpdateConfig,
    MappingUpdateClient,
)
from app_config import (
    DEFAULT_MAP_TOPIC,
    DEFAULT_MAP_UPDATE_PARAM,
    DEFAULT_ROSBRIDGE_HOST,
    DEFAULT_ROSBRIDGE_PORT,
)
from ros_bridge_worker import normalize_ros_topic


class LocalizationPanel(QWidget):
    """Read-only FAST-LIO2 odometry monitor plus frozen map fusion export."""

    status_query_requested = Signal()
    fastlio_topic_changed = Signal(str)
    fastlio_subscription_pending_changed = Signal(bool)
    fastlio_subscription_apply_requested = Signal(bool)
    launch_manager_command_requested = Signal(str)
    line_follow_control_requested = Signal(float, bool, bool)

    def __init__(
        self,
        parent=None,
        *,
        mapping_update_client: MappingUpdateClient | None = None,
        map_fetch_client: MapFetchClient | None = None,
    ) -> None:
        super().__init__(parent)
        self._buffer = LocalizationBuffer(max_points=5000)
        self._labels: dict[str, QLabel] = {}
        self._recording_started = False
        self._mapping_update_client = mapping_update_client
        self._map_fetch_client = map_fetch_client
        self._owns_mapping_client = mapping_update_client is None
        self._owns_fetch_client = map_fetch_client is None
        self._map_update_enabled = True
        self._map_frozen = False
        self._frozen_map_points: list[MapPoint] = []
        self._frozen_map_path: Path | None = None
        self._map_fetch_metadata: dict[str, object] = {}
        self._launch_buttons_connected = False
        self._fastlio_running = False
        self._fastlio_stop_pending = False
        self._lidar_running = False
        self._lidar_stop_pending = False
        self._calibration_running = False
        self._calibration_motion_active = False
        self._calibration_start_pending = False
        self._calibration_stop_pending = False
        self._setup_ui()
        self._setup_timer()
        self._set_connected(False)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)
        layout.addWidget(self._build_connection_group())

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_plot_group())
        splitter.addWidget(self._build_side_panel())
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter, stretch=1)

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("FAST-LIO2 rosbridge 连接")
        row = QHBoxLayout(group)
        row.addWidget(QLabel("host:"))
        self._host_edit = QLineEdit(DEFAULT_ROSBRIDGE_HOST)
        self._host_edit.setMinimumWidth(160)
        row.addWidget(self._host_edit)
        row.addWidget(QLabel("port:"))
        self._port_spin = QSpinBox()
        self._port_spin.setRange(1, 65535)
        self._port_spin.setValue(DEFAULT_ROSBRIDGE_PORT)
        row.addWidget(self._port_spin)
        row.addWidget(QLabel("topic:"))
        self._topic_edit = QLineEdit("/Odometry")
        self._topic_edit.setMinimumWidth(120)
        self._topic_edit.editingFinished.connect(self._on_fastlio_topic_edited)
        row.addWidget(self._topic_edit)
        self._fastlio_subscription_cb = QCheckBox("FAST-LIO 里程计")
        self._fastlio_subscription_cb.toggled.connect(
            self.fastlio_subscription_pending_changed.emit
        )
        row.addWidget(self._fastlio_subscription_cb)
        self._apply_fastlio_subscription_btn = QPushButton("应用订阅")
        self._apply_fastlio_subscription_btn.clicked.connect(
            self._on_apply_fastlio_subscription
        )
        row.addWidget(self._apply_fastlio_subscription_btn)
        self._connect_btn = QPushButton("请在 ROS 页连接")
        self._connect_btn.setEnabled(False)
        self._connect_btn.setToolTip("定位页使用 ROS 页的共享 ROSbridge 连接")
        row.addWidget(self._connect_btn)
        self._disconnect_btn = QPushButton("断开")
        self._disconnect_btn.setEnabled(False)
        self._disconnect_btn.setToolTip("请在 ROS 页断开共享 ROSbridge 连接")
        row.addWidget(self._disconnect_btn)
        self._connection_label = QLabel("请在 ROS 页面连接共享 ROSbridge")
        row.addWidget(self._connection_label)
        self._online_label = QLabel("/Odometry: ---")
        row.addWidget(self._online_label)
        row.addStretch()
        return group

    def _build_plot_group(self) -> QGroupBox:
        group = QGroupBox("轨迹 / 冻结地图俯视融合")
        layout = QVBoxLayout(group)
        self._trajectory_plot = pg.PlotWidget()
        self._trajectory_plot.setBackground("w")
        self._trajectory_plot.showGrid(x=True, y=True, alpha=0.3)
        self._trajectory_plot.setLabel("bottom", "trajectory x / x0_aligned", units="m")
        self._trajectory_plot.setLabel("left", "trajectory y / y0_aligned", units="m")
        self._trajectory_plot.addLegend(offset=(10, 10))
        plot_item = self._trajectory_plot.getPlotItem()
        plot_item.setDownsampling(mode="peak")
        plot_item.setClipToView(True)
        plot_item.getViewBox().setAspectLocked(True, ratio=1.0)
        self._map_curve = self._trajectory_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=2,
            symbolBrush=pg.mkBrush("#94a3b8"),
            symbolPen=None,
            name="冻结点云地图",
        )
        self._trajectory_curve = self._trajectory_plot.plot(
            pen=pg.mkPen("#1565c0", width=2),
            name="FAST-LIO2 轨迹",
        )
        self._start_curve = self._trajectory_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=9,
            symbolBrush=pg.mkBrush("#0284c7"),
            symbolPen=None,
            name="起点",
        )
        self._current_curve = self._trajectory_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=9,
            symbolBrush=pg.mkBrush("#ea580c"),
            symbolPen=None,
            name="当前点",
        )
        self._reference_curve = self._trajectory_plot.plot(
            [0.0, 10.0],
            [0.0, 0.0],
            pen=pg.mkPen("#666666", width=1.2, style=Qt.DashLine),
            name="参考直线 y=0",
        )
        layout.addWidget(self._trajectory_plot, stretch=1)
        return group

    def _build_side_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(self._build_pose_group())
        layout.addWidget(self._build_control_group())
        layout.addWidget(self._build_stats_group())
        layout.addWidget(self._build_feedback_group())
        layout.addStretch()
        return panel

    def _build_pose_group(self) -> QGroupBox:
        group = QGroupBox("实时位姿")
        form = QFormLayout(group)
        for title, key in (
            ("frame_id", "frame_id"),
            ("child_frame_id", "child_frame_id"),
            ("x", "x"),
            ("y", "y"),
            ("z", "z"),
            ("yaw_deg", "yaw_deg"),
            ("x0_aligned", "x0_aligned"),
            ("y0_aligned", "y0_aligned"),
            ("yaw0_aligned", "yaw0_aligned"),
            ("speed_estimated", "speed_estimated"),
            ("trajectory_length", "trajectory_length"),
            ("最近时间戳", "last_time"),
        ):
            self._add_value_row(form, title, key)
        return group

    def _build_control_group(self) -> QGroupBox:
        group = QGroupBox("测试与建图控制")
        layout = QVBoxLayout(group)
        row1 = QHBoxLayout()
        self._set_origin_btn = QPushButton("设置当前为起点")
        self._set_origin_btn.clicked.connect(self._set_current_origin)
        row1.addWidget(self._set_origin_btn)
        self._clear_btn = QPushButton("清空轨迹")
        self._clear_btn.clicked.connect(self._clear)
        row1.addWidget(self._clear_btn)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        self._record_btn = QPushButton("开始定位记录")
        self._record_btn.clicked.connect(self._toggle_recording)
        row2.addWidget(self._record_btn)
        self._report_btn = QPushButton("生成测试摘要")
        self._report_btn.clicked.connect(self._write_report_dialog)
        row2.addWidget(self._report_btn)
        layout.addLayout(row2)

        row3 = QHBoxLayout()
        self._mapping_freeze_btn = QPushButton("冻结建图")
        self._mapping_freeze_btn.clicked.connect(self._toggle_mapping_freeze)
        row3.addWidget(self._mapping_freeze_btn)
        self._save_map_trajectory_btn = QPushButton("保存地图与轨迹数据")
        self._save_map_trajectory_btn.clicked.connect(self._write_frozen_package_dialog)
        row3.addWidget(self._save_map_trajectory_btn)
        layout.addLayout(row3)

        row4 = QHBoxLayout()
        self._fastlio_launch_start_btn = QPushButton("启动 FAST-LIO")
        self._fastlio_launch_start_btn.clicked.connect(self._start_fastlio_launch)
        row4.addWidget(self._fastlio_launch_start_btn)
        self._fastlio_launch_stop_btn = QPushButton("停止 FAST-LIO")
        self._fastlio_launch_stop_btn.clicked.connect(self._stop_fastlio_launch)
        row4.addWidget(self._fastlio_launch_stop_btn)
        layout.addLayout(row4)

        self._fastlio_launch_label = QLabel("")
        layout.addWidget(self._fastlio_launch_label)

        row5 = QHBoxLayout()
        self._lidar_launch_start_btn = QPushButton("启动雷达节点")
        self._lidar_launch_start_btn.clicked.connect(self._start_lidar_launch)
        row5.addWidget(self._lidar_launch_start_btn)
        self._lidar_launch_stop_btn = QPushButton("停止雷达节点")
        self._lidar_launch_stop_btn.clicked.connect(self._stop_lidar_launch)
        row5.addWidget(self._lidar_launch_stop_btn)
        layout.addLayout(row5)

        self._lidar_launch_label = QLabel("")
        layout.addWidget(self._lidar_launch_label)

        config_form = QFormLayout()
        self._map_update_param_edit = QLineEdit(DEFAULT_MAP_UPDATE_PARAM)
        self._map_topic_edit = QLineEdit(DEFAULT_MAP_TOPIC)
        self._local_map_path_edit = QLineEdit("")
        for title, widget in (
            ("建图参数:", self._map_update_param_edit),
            ("地图 topic:", self._map_topic_edit),
            ("本地地图路径:", self._local_map_path_edit),
        ):
            config_form.addRow(title, widget)
        layout.addLayout(config_form)

        self._map_label = QLabel("建图未冻结")
        layout.addWidget(self._map_label)
        self._record_label = QLabel("")
        layout.addWidget(self._record_label)
        return group

    def _build_stats_group(self) -> QGroupBox:
        group = QGroupBox("误差统计")
        form = QFormLayout(group)
        for title, key in (
            ("lateral_error_current", "lateral_error_current"),
            ("lateral_error_rms", "lateral_error_rms"),
            ("lateral_error_max", "lateral_error_max"),
            ("endpoint_lateral_error", "endpoint_lateral_error"),
            ("endpoint_distance", "endpoint_distance"),
            ("trajectory_length", "stats_trajectory_length"),
            ("yaw_rms", "yaw_rms"),
            ("estimated_speed_mean", "estimated_speed_mean"),
            ("estimated_speed_std", "estimated_speed_std"),
        ):
            self._add_value_row(form, title, key)
        return group

    def _build_feedback_group(self) -> QGroupBox:
        group = QGroupBox("雷达直线校准")
        layout = QVBoxLayout(group)

        launch_row = QHBoxLayout()
        self._calibration_launch_start_btn = QPushButton("启动校准节点")
        self._calibration_launch_start_btn.clicked.connect(self._start_calibration_launch)
        launch_row.addWidget(self._calibration_launch_start_btn)
        self._calibration_launch_stop_btn = QPushButton("停止校准节点")
        self._calibration_launch_stop_btn.clicked.connect(self._stop_calibration_launch)
        launch_row.addWidget(self._calibration_launch_stop_btn)
        layout.addLayout(launch_row)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("目标速度:"))
        self._calibration_speed_spin = self._make_double_spin(0.0, 3.0, 0.05)
        self._calibration_speed_spin.setValue(0.2)
        speed_row.addWidget(self._calibration_speed_spin)
        speed_row.addWidget(QLabel("m/s"))
        speed_row.addStretch()
        layout.addLayout(speed_row)

        motion_row = QHBoxLayout()
        self._calibration_forward_btn = QPushButton("前进")
        self._calibration_forward_btn.clicked.connect(self._calibration_forward)
        motion_row.addWidget(self._calibration_forward_btn)
        self._calibration_backward_btn = QPushButton("后退")
        self._calibration_backward_btn.clicked.connect(self._calibration_backward)
        motion_row.addWidget(self._calibration_backward_btn)
        self._calibration_stop_btn = QPushButton("停止")
        self._calibration_stop_btn.clicked.connect(self._calibration_stop)
        motion_row.addWidget(self._calibration_stop_btn)
        layout.addLayout(motion_row)

        self._calibration_status_label = QLabel("校准节点未运行")
        layout.addWidget(self._calibration_status_label)
        return group

    def _add_value_row(self, form: QFormLayout, title: str, key: str) -> None:
        label = QLabel("---")
        label.setMinimumWidth(120)
        form.addRow(f"{title}:", label)
        self._labels[key] = label

    def _setup_timer(self) -> None:
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_view)
        self._refresh_timer.start(100)

    def _on_fastlio_topic_edited(self) -> None:
        topic = self.fastlio_odometry_topic()
        self.set_fastlio_odometry_topic(topic)
        self.fastlio_topic_changed.emit(topic)

    def fastlio_odometry_topic(self) -> str:
        return normalize_ros_topic(self._topic_edit.text())

    def set_fastlio_odometry_topic(self, topic: str) -> None:
        blocker = QSignalBlocker(self._topic_edit)
        self._topic_edit.setText(normalize_ros_topic(topic))
        del blocker

    def fastlio_subscription_enabled(self) -> bool:
        return self._fastlio_subscription_cb.isChecked()

    def set_fastlio_subscription_enabled(self, enabled: bool) -> None:
        blocker = QSignalBlocker(self._fastlio_subscription_cb)
        self._fastlio_subscription_cb.setChecked(bool(enabled))
        del blocker

    def _on_apply_fastlio_subscription(self) -> None:
        self.fastlio_subscription_apply_requested.emit(
            self.fastlio_subscription_enabled()
        )

    def set_fastlio_subscription_applied(self, enabled: bool) -> None:
        state = "等待数据" if enabled else "已停用"
        self._online_label.setText(f"{self.fastlio_odometry_topic()}: {state}")

    def set_shared_ros_connected(self, connected: bool) -> None:
        self._set_connected(connected)

    def accept_localization_sample(self, sample: LocalizationSample) -> None:
        self._on_sample(sample)

    def _set_connected(self, connected: bool) -> None:
        self._launch_buttons_connected = connected
        if not connected:
            if self._calibration_motion_active:
                self.line_follow_control_requested.emit(0.0, False, False)
            self._fastlio_running = False
            self._fastlio_stop_pending = False
            self._lidar_running = False
            self._lidar_stop_pending = False
            self._calibration_running = False
            self._calibration_motion_active = False
            self._calibration_start_pending = False
            self._calibration_stop_pending = False
        self._connect_btn.setEnabled(False)
        self._disconnect_btn.setEnabled(False)
        self._host_edit.setEnabled(False)
        self._port_spin.setEnabled(False)
        self._topic_edit.setEnabled(True)
        self._update_launch_buttons()
        self._update_calibration_buttons()
        self._connection_label.setStyleSheet("color: green;" if connected else "color: red;")

    def _on_sample(self, sample: LocalizationSample) -> None:
        enriched = self._buffer.append(sample)
        self._online_label.setText(f"{sample.source}: 在线")
        self._labels["last_time"].setText(f"{enriched.ros_time:.3f}")

    def _on_error(self, message: str) -> None:
        self._connection_label.setText(message)
        self._connection_label.setStyleSheet("color: red;")

    def _start_fastlio_launch(self) -> None:
        self._fastlio_running = True
        self._fastlio_stop_pending = False
        self._update_launch_buttons()
        self.launch_manager_command_requested.emit(
            "start fastlio fast_lio mapping_c16.launch"
        )
        self.set_fastlio_launch_status("FAST-LIO 启动命令已发送")
        self.status_query_requested.emit()

    def _stop_fastlio_launch(self) -> None:
        self._fastlio_stop_pending = True
        self._update_launch_buttons()
        self.launch_manager_command_requested.emit("stop fastlio")
        self.set_fastlio_launch_status("FAST-LIO 停止命令已发送")
        self.status_query_requested.emit()

    def set_fastlio_launch_status(self, text: str, *, error: bool = False) -> None:
        self._fastlio_launch_label.setText(text)
        self._fastlio_launch_label.setStyleSheet("color: red;" if error else "color: green;")

    def _start_lidar_launch(self) -> None:
        self._lidar_running = True
        self._lidar_stop_pending = False
        self._update_launch_buttons()
        self.launch_manager_command_requested.emit(
            "start lidar turn_on_wheeltec_robot wheeltec_lidar.launch"
        )
        self.set_lidar_launch_status("雷达节点启动命令已发送")
        self.status_query_requested.emit()

    def _stop_lidar_launch(self) -> None:
        self._lidar_stop_pending = True
        self._update_launch_buttons()
        self.launch_manager_command_requested.emit("stop lidar")
        self.set_lidar_launch_status("雷达节点停止命令已发送")
        self.status_query_requested.emit()

    def set_lidar_launch_status(self, text: str, *, error: bool = False) -> None:
        self._lidar_launch_label.setText(text)
        self._lidar_launch_label.setStyleSheet("color: red;" if error else "color: green;")

    def _start_calibration_launch(self) -> None:
        self._calibration_start_pending = True
        self._calibration_stop_pending = False
        self._update_calibration_buttons()
        command = (
            "restart pid_control simple_follower pid_control_lidar_assisted.launch "
            f"imu_topic:=/active_imu lidar_odom_topic:={self.fastlio_odometry_topic()}"
        )
        self.launch_manager_command_requested.emit(command)
        self._calibration_status_label.setText("校准节点启动命令已发送")
        self.status_query_requested.emit()

    def _stop_calibration_launch(self) -> None:
        self._calibration_stop()
        self._calibration_stop_pending = True
        self._calibration_start_pending = False
        self._update_calibration_buttons()
        self.launch_manager_command_requested.emit("stop pid_control")
        self._calibration_status_label.setText("校准节点停止命令已发送")
        self.status_query_requested.emit()

    def _calibration_forward(self) -> None:
        if not self._calibration_running:
            return
        self._calibration_motion_active = True
        self.line_follow_control_requested.emit(
            self._calibration_speed_spin.value(),
            True,
            False,
        )

    def _calibration_backward(self) -> None:
        if not self._calibration_running:
            return
        self._calibration_motion_active = True
        self.line_follow_control_requested.emit(
            self._calibration_speed_spin.value(),
            False,
            True,
        )

    def _calibration_stop(self) -> None:
        if not self._calibration_running and not self._calibration_motion_active:
            return
        self._calibration_motion_active = False
        self.line_follow_control_requested.emit(0.0, False, False)

    def update_launch_manager_status(self, status) -> None:
        data = status.get("data", status) if isinstance(status, dict) else {}
        running = set(data.get("running") or []) if isinstance(data, dict) else set()
        raw_detail = data.get("detail", {}) if isinstance(data, dict) else {}
        detail = raw_detail if isinstance(raw_detail, dict) else {}

        self._fastlio_running = "fastlio" in running
        self._fastlio_stop_pending = False
        self._lidar_running = "lidar" in running
        self._lidar_stop_pending = False
        pid_detail = detail.get("pid_control", {})
        self._calibration_running = (
            "pid_control" in running
            and "pid_control_lidar_assisted.launch" in self._launch_detail_text(pid_detail)
        )
        self._calibration_start_pending = False
        self._calibration_stop_pending = False
        if not self._calibration_running:
            self._calibration_motion_active = False
        self._update_launch_buttons()
        self._update_calibration_buttons()
        self.set_fastlio_launch_status(
            self._launch_status_text("FAST-LIO", self._fastlio_running, detail.get("fastlio", {})),
            error=not self._fastlio_running,
        )
        self.set_lidar_launch_status(
            self._launch_status_text("雷达节点", self._lidar_running, detail.get("lidar", {})),
            error=not self._lidar_running,
        )
        calibration_state = "运行中" if self._calibration_running else "未运行"
        calibration_detail = self._launch_detail_text(pid_detail)
        suffix = f": {calibration_detail}" if calibration_detail else ""
        self._calibration_status_label.setText(f"校准节点{calibration_state}{suffix}")

    def _update_launch_buttons(self) -> None:
        connected = self._launch_buttons_connected
        self._fastlio_launch_start_btn.setEnabled(
            connected and not self._fastlio_running and not self._fastlio_stop_pending
        )
        self._fastlio_launch_stop_btn.setEnabled(
            connected and self._fastlio_running and not self._fastlio_stop_pending
        )
        self._lidar_launch_start_btn.setEnabled(
            connected and not self._lidar_running and not self._lidar_stop_pending
        )
        self._lidar_launch_stop_btn.setEnabled(
            connected and self._lidar_running and not self._lidar_stop_pending
        )

    def _update_calibration_buttons(self) -> None:
        connected = self._launch_buttons_connected
        pending = self._calibration_start_pending or self._calibration_stop_pending
        self._calibration_launch_start_btn.setEnabled(
            connected and not self._calibration_running and not pending
        )
        self._calibration_launch_stop_btn.setEnabled(
            connected and self._calibration_running and not pending
        )
        motion_enabled = connected and self._calibration_running and not pending
        self._calibration_forward_btn.setEnabled(motion_enabled)
        self._calibration_backward_btn.setEnabled(motion_enabled)
        self._calibration_stop_btn.setEnabled(motion_enabled)

    def _launch_status_text(self, label: str, running: bool, detail) -> str:
        state = "运行中" if running else "未运行"
        detail_text = self._launch_detail_text(detail)
        if detail_text:
            return f"{label} {state}: {detail_text}"
        return f"{label} {state}"

    def _launch_detail_text(self, detail) -> str:
        if not isinstance(detail, dict):
            return ""
        parts: list[str] = []
        if detail.get("package"):
            parts.append(str(detail["package"]))
        if detail.get("launch"):
            parts.append(str(detail["launch"]))
        return " ".join(parts)

    def _refresh_view(self) -> None:
        latest = self._buffer.latest()
        stats = self._buffer.stats()
        if latest is not None:
            for key in ("frame_id", "child_frame_id"):
                self._labels[key].setText(str(getattr(latest, key)))
            for key in (
                "x", "y", "z", "yaw_deg", "x0_aligned", "y0_aligned",
                "yaw0_aligned", "speed_estimated", "trajectory_length",
            ):
                self._labels[key].setText(f"{float(getattr(latest, key)):.4f}")
        stats_map = {
            "lateral_error_current": stats.lateral_error_current,
            "lateral_error_rms": stats.lateral_error_rms,
            "lateral_error_max": stats.lateral_error_max,
            "endpoint_lateral_error": stats.endpoint_lateral_error,
            "endpoint_distance": stats.endpoint_distance,
            "stats_trajectory_length": stats.trajectory_length,
            "yaw_rms": stats.yaw_rms,
            "estimated_speed_mean": stats.estimated_speed_mean,
            "estimated_speed_std": stats.estimated_speed_std,
        }
        for key, value in stats_map.items():
            self._labels[key].setText(f"{value:.4f}")
        xs, ys = self._trajectory_plot_xy()
        self._trajectory_curve.setData(xs, ys)
        if xs:
            self._start_curve.setData([xs[0]], [ys[0]])
            self._current_curve.setData([xs[-1]], [ys[-1]])
        else:
            self._start_curve.setData([], [])
            self._current_curve.setData([], [])
        end_x = max(10.0, max(xs) if xs else 10.0)
        self._reference_curve.setData([0.0, end_x], [0.0, 0.0])

    def _trajectory_plot_xy(self) -> tuple[list[float], list[float]]:
        if self._map_frozen and self._frozen_map_points:
            rows = self._buffer.rows()
            return [row.x for row in rows], [row.y for row in rows]
        return self._buffer.plot_xy()

    def _set_current_origin(self) -> None:
        self._buffer.set_current_pose_as_origin()
        self._refresh_view()

    def _clear(self) -> None:
        self._buffer.clear()
        self._trajectory_curve.setData([], [])
        self._start_curve.setData([], [])
        self._current_curve.setData([], [])
        for label in self._labels.values():
            label.setText("---")

    def _toggle_mapping_freeze(self) -> None:
        if self._map_update_enabled:
            self._freeze_mapping_and_fetch_map()
            return
        self._resume_mapping()

    def _freeze_mapping_and_fetch_map(self) -> None:
        try:
            mapping = self._current_mapping_update_client()
            freeze_result = mapping.set_map_update_enabled(False)
        except Exception as exc:
            self._record_label.setText(f"建图冻结失败: {exc}")
            return

        self._map_update_enabled = False
        self._map_frozen = True
        self._mapping_freeze_btn.setText("恢复建图")
        self._record_label.setText("建图已冻结，正在获取冻结地图...")
        try:
            fetcher = self._current_map_fetch_client()
            result = fetcher.fetch_once(self._map_cache_dir())
            points = fetcher.read_points(result.local_path)
        except Exception as exc:
            self._record_label.setText(f"建图已冻结，但地图获取失败: {exc}")
            self._map_label.setText("冻结地图获取失败")
            self._frozen_map_points = []
            self._frozen_map_path = None
            return

        self._frozen_map_points = points
        self._frozen_map_path = result.local_path
        self._map_fetch_metadata = {
            "map_source": result.source,
            "map_freeze_method": freeze_result.get("command", freeze_result.get("method", "")),
            "raw_map_file": result.raw_file_name,
        }
        self._map_curve.setData([point.x for point in points], [point.y for point in points])
        self._map_label.setText(f"冻结地图: {result.raw_file_name} ({len(points)} 点)")
        self._record_label.setText(f"建图已冻结，已获取 {len(points)} 个地图点")

    def _resume_mapping(self) -> None:
        try:
            self._current_mapping_update_client().set_map_update_enabled(True)
        except Exception as exc:
            self._record_label.setText(f"恢复建图失败: {exc}")
            return
        self._map_update_enabled = True
        self._map_frozen = False
        self._mapping_freeze_btn.setText("冻结建图")
        self._map_curve.setData([], [])
        self._map_label.setText("建图已恢复，当前只显示实时轨迹")
        self._record_label.setText("建图更新已恢复")

    def _write_frozen_package_dialog(self) -> None:
        default_path = self._default_output_path("frozen_map_trajectory", ".zip")
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "保存地图与轨迹数据",
            str(default_path),
            "ZIP 数据包 (*.zip)",
        )
        if filepath:
            self._save_frozen_package(Path(filepath))

    def _save_frozen_package(self, path: Path) -> Path | None:
        if not self._map_frozen or not self._frozen_map_points:
            self._record_label.setText("请先冻结建图并获取地图")
            return None
        rows = self._buffer.rows()
        if not rows:
            self._record_label.setText("当前没有轨迹数据")
            return None
        summary = export_frozen_map_trajectory_zip(
            Path(path),
            map_points=self._frozen_map_points,
            trajectory_rows=rows,
            metadata={
                "coordinate_frame": "camera_init x-y top-down; map and trajectory both use FAST-LIO raw x/y",
                "odometry_topic": self._topic_edit.text().strip() or "/Odometry",
                "map_source": self._map_fetch_metadata.get("map_source", ""),
                "map_freeze_method": self._map_fetch_metadata.get("map_freeze_method", ""),
                "use_aligned_xy": False,
            },
            raw_map_path=self._frozen_map_path,
        )
        saved = Path(str(summary["output"]))
        self._record_label.setText(
            f"已保存 {saved}，地图 {summary['map_points']} 点，轨迹 {summary['trajectory_points']} 点"
        )
        return saved

    def _toggle_recording(self) -> None:
        if not self._buffer.recording:
            self._buffer.start_recording()
            self._record_btn.setText("停止并保存 CSV")
            self._record_btn.setStyleSheet("background-color: #e74c3c; color: white;")
            self._record_label.setText("定位记录中...")
            return
        default_path = self._default_output_path("localization_test", ".csv")
        filepath, _ = QFileDialog.getSaveFileName(self, "保存定位 CSV", str(default_path), "CSV 文件 (*.csv)")
        if filepath:
            saved = self._buffer.stop_recording(Path(filepath))
            self._record_label.setText(f"已保存 {saved}")
        else:
            self._buffer.cancel_recording()
            self._record_label.setText("记录已停止")
        self._record_btn.setText("开始定位记录")
        self._record_btn.setStyleSheet("")

    def _write_report_dialog(self) -> None:
        default_path = self._default_output_path("localization_test_report", ".md")
        filepath, _ = QFileDialog.getSaveFileName(self, "保存测试报告", str(default_path), "Markdown 文件 (*.md)")
        if not filepath:
            return
        saved = self._buffer.write_report(Path(filepath))
        self._record_label.setText(f"报告已保存 {saved}")

    def _sync_control_state(self) -> None:
        self._buffer.set_control_state(
            enabled=self._control_enabled_cb.isChecked(),
            mode=str(self._control_mode_combo.currentData() or "monitor_only"),
            target_speed=self._target_speed_spin.value(),
            target_yaw=self._target_yaw_spin.value() * 3.141592653589793 / 180.0,
            correction_vx=self._correction_vx_spin.value(),
            correction_vz=self._correction_vz_spin.value(),
            safety_state=self._safety_state_edit.text().strip() or "monitor_only",
        )

    def _current_mapping_update_client(self) -> MappingUpdateClient:
        if self._mapping_update_client is None or self._owns_mapping_client:
            self._mapping_update_client = MappingUpdateClient(
                MappingUpdateConfig(
                    host=self._host_edit.text().strip() or "localhost",
                    port=self._port_spin.value(),
                    parameter=self._map_update_param_edit.text().strip() or DEFAULT_MAP_UPDATE_PARAM,
                )
            )
        return self._mapping_update_client

    def _current_map_fetch_client(self) -> MapFetchClient:
        if self._map_fetch_client is None or self._owns_fetch_client:
            self._map_fetch_client = MapFetchClient(
                MapFetchConfig(
                    host=self._host_edit.text().strip() or "localhost",
                    port=self._port_spin.value(),
                    map_topic=self._map_topic_edit.text().strip(),
                    local_map_path=self._local_map_path_edit.text().strip(),
                )
            )
        return self._map_fetch_client

    @staticmethod
    def _map_cache_dir() -> Path:
        return Path(tempfile.gettempdir()) / "debug_monitor_frozen_maps"

    def shutdown(self) -> None:
        self._calibration_stop()
        self._refresh_timer.stop()

    def sizeHint(self) -> QSize:
        return QSize(1000, 560)

    def minimumSizeHint(self) -> QSize:
        return QSize(760, 420)

    @staticmethod
    def _make_double_spin(minimum: float, maximum: float, step: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(4)
        spin.setSingleStep(step)
        spin.setMaximumWidth(120)
        return spin

    @staticmethod
    def _default_output_path(prefix: str, suffix: str) -> Path:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return Path(tempfile.gettempdir()) / f"{prefix}_{timestamp}{suffix}"

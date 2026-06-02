"""FAST-LIO2 localization stability test panel."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import tempfile

from PySide6.QtCore import QSize, Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
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
    QSplitter,
    QVBoxLayout,
    QWidget,
)
import pyqtgraph as pg

from localization_fusion import MapPoint, read_ascii_ply_xy, save_fused_map_trajectory
from localization_buffer import LocalizationBuffer, LocalizationSample
from ros_odometry_client import RosOdometryWorker


class LocalizationPanel(QWidget):
    """Read-only FAST-LIO2 odometry monitor, recorder, and straight-line evaluator."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._buffer = LocalizationBuffer(max_points=5000)
        self._worker = RosOdometryWorker()
        self._worker.sample_received.connect(self._on_sample)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.connection_changed.connect(self._on_connection_changed)
        self._labels: dict[str, QLabel] = {}
        self._recording_started = False
        self._map_path: Path | None = None
        self._map_points: list[MapPoint] = []
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
        self._host_edit = QLineEdit("192.168.0.14")
        self._host_edit.setMinimumWidth(160)
        row.addWidget(self._host_edit)
        row.addWidget(QLabel("port:"))
        self._port_spin = QSpinBox()
        self._port_spin.setRange(1, 65535)
        self._port_spin.setValue(9090)
        row.addWidget(self._port_spin)
        row.addWidget(QLabel("topic:"))
        self._topic_edit = QLineEdit("/Odometry")
        self._topic_edit.setMinimumWidth(120)
        row.addWidget(self._topic_edit)
        self._connect_btn = QPushButton("连接")
        self._connect_btn.clicked.connect(self._connect)
        row.addWidget(self._connect_btn)
        self._disconnect_btn = QPushButton("断开")
        self._disconnect_btn.clicked.connect(self._worker.close_bridge)
        row.addWidget(self._disconnect_btn)
        self._connection_label = QLabel("未连接")
        row.addWidget(self._connection_label)
        self._online_label = QLabel("/Odometry: ---")
        row.addWidget(self._online_label)
        row.addStretch()
        return group

    def _build_plot_group(self) -> QGroupBox:
        group = QGroupBox("轨迹")
        layout = QVBoxLayout(group)
        self._trajectory_plot = pg.PlotWidget()
        self._trajectory_plot.setBackground("w")
        self._trajectory_plot.showGrid(x=True, y=True, alpha=0.3)
        self._trajectory_plot.setLabel("bottom", "x0_aligned", units="m")
        self._trajectory_plot.setLabel("left", "y0_aligned", units="m")
        self._trajectory_plot.addLegend(offset=(10, 10))
        plot_item = self._trajectory_plot.getPlotItem()
        plot_item.setDownsampling(mode="peak")
        plot_item.setClipToView(True)
        self._map_curve = self._trajectory_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=2,
            symbolBrush=pg.mkBrush("#94a3b8"),
            symbolPen=None,
            name="FAST-LIO 建图",
        )
        self._trajectory_curve = self._trajectory_plot.plot(
            pen=pg.mkPen("#1565c0", width=2),
            name="FAST-LIO2 轨迹",
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
        group = QGroupBox("测试控制")
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
        self._select_map_btn = QPushButton("选择建图 PLY")
        self._select_map_btn.clicked.connect(self._select_map_dialog)
        row3.addWidget(self._select_map_btn)
        self._fused_save_btn = QPushButton("融合显示并保存")
        self._fused_save_btn.clicked.connect(self._write_fused_map_dialog)
        row3.addWidget(self._fused_save_btn)
        self._map_label = QLabel("未选择建图")
        row3.addWidget(self._map_label, stretch=1)
        layout.addLayout(row3)

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
        group = QGroupBox("控制反馈接口预留")
        layout = QVBoxLayout(group)
        note = QLabel("控制反馈接口预留，默认不发送到底盘")
        note.setStyleSheet("color: #b00020; font-weight: bold;")
        layout.addWidget(note)
        form = QFormLayout()
        self._control_enabled_cb = QCheckBox("control_enabled")
        self._control_enabled_cb.setChecked(False)
        self._control_enabled_cb.toggled.connect(self._sync_control_state)
        form.addRow("enable:", self._control_enabled_cb)
        self._control_mode_combo = QComboBox()
        for mode in ("monitor_only", "heading_assist", "lateral_assist", "radar_guided_assist"):
            self._control_mode_combo.addItem(mode, mode)
        self._control_mode_combo.currentIndexChanged.connect(self._sync_control_state)
        form.addRow("control_mode:", self._control_mode_combo)
        self._target_speed_spin = self._make_double_spin(-3.0, 3.0, 0.01)
        self._target_yaw_spin = self._make_double_spin(-180.0, 180.0, 1.0)
        self._correction_vx_spin = self._make_double_spin(-1.0, 1.0, 0.01)
        self._correction_vz_spin = self._make_double_spin(-3.0, 3.0, 0.01)
        for title, spin in (
            ("target_speed:", self._target_speed_spin),
            ("target_yaw_deg:", self._target_yaw_spin),
            ("correction_vx:", self._correction_vx_spin),
            ("correction_vz:", self._correction_vz_spin),
        ):
            spin.valueChanged.connect(self._sync_control_state)
            form.addRow(title, spin)
        self._safety_state_edit = QLineEdit("monitor_only")
        self._safety_state_edit.textChanged.connect(self._sync_control_state)
        form.addRow("safety_state:", self._safety_state_edit)
        layout.addLayout(form)
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

    def _connect(self) -> None:
        self._worker.open_bridge(
            self._host_edit.text().strip() or "localhost",
            self._port_spin.value(),
            self._topic_edit.text().strip() or "/Odometry",
        )

    def _on_connection_changed(self, connected: bool) -> None:
        self._set_connected(connected)

    def _set_connected(self, connected: bool) -> None:
        self._connect_btn.setEnabled(not connected)
        self._disconnect_btn.setEnabled(connected)
        self._host_edit.setEnabled(not connected)
        self._port_spin.setEnabled(not connected)
        self._topic_edit.setEnabled(not connected)
        self._connection_label.setText("已连接" if connected else "未连接")
        self._connection_label.setStyleSheet("color: green;" if connected else "color: red;")

    def _on_sample(self, sample: LocalizationSample) -> None:
        enriched = self._buffer.append(sample)
        self._online_label.setText(f"{sample.source}: 在线")
        self._labels["last_time"].setText(f"{enriched.ros_time:.3f}")

    def _on_error(self, message: str) -> None:
        self._connection_label.setText(message)
        self._connection_label.setStyleSheet("color: red;")

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
        xs, ys = self._buffer.plot_xy()
        self._trajectory_curve.setData(xs, ys)
        end_x = max(10.0, max(xs) if xs else 10.0)
        self._reference_curve.setData([0.0, end_x], [0.0, 0.0])

    def _set_current_origin(self) -> None:
        self._buffer.set_current_pose_as_origin()
        self._refresh_view()

    def _clear(self) -> None:
        self._buffer.clear()
        self._trajectory_curve.setData([], [])
        for label in self._labels.values():
            label.setText("---")

    def _select_map_dialog(self) -> None:
        filepath, _ = QFileDialog.getOpenFileName(self, "选择 FAST-LIO 建图 PLY", "", "PLY 文件 (*.ply)")
        if filepath:
            self.load_map_for_test(Path(filepath))

    def load_map_for_test(self, path: Path) -> None:
        try:
            points = read_ascii_ply_xy(Path(path), max_points=50000)
        except Exception as exc:
            self._map_label.setText(f"建图加载失败: {exc}")
            self._record_label.setText(f"建图加载失败: {exc}")
            return
        self._map_path = Path(path)
        self._map_points = points
        self._map_curve.setData(
            [point.x for point in points],
            [point.y for point in points],
        )
        self._map_label.setText(f"建图: {self._map_path.name} ({len(points)} 点)")

    def _write_fused_map_dialog(self) -> None:
        if self._map_path is None:
            self._select_map_dialog()
            if self._map_path is None:
                return
        default_path = self._default_output_path("fastlio_map_trajectory_fused", ".svg")
        filepath, _ = QFileDialog.getSaveFileName(self, "保存建图轨迹融合 SVG", str(default_path), "SVG 文件 (*.svg)")
        if not filepath:
            return
        self._save_fused_map(Path(filepath))

    def _save_fused_map(self, path: Path) -> Path | None:
        if self._map_path is None:
            self._record_label.setText("请先选择建图 PLY")
            return None
        xs, ys = self._buffer.plot_xy()
        if not xs:
            self._record_label.setText("当前没有 FAST-LIO 轨迹")
            return None
        summary = save_fused_map_trajectory(
            self._map_path,
            list(zip(xs, ys)),
            Path(path),
        )
        saved = Path(str(summary["output"]))
        self._record_label.setText(
            f"融合图已保存 {saved}，地图 {summary['map_points']} 点，轨迹 {summary['trajectory_points']} 点"
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

    def shutdown(self) -> None:
        self._refresh_timer.stop()
        self._worker.close_bridge()

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

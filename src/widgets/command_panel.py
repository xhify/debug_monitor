"""命令发送面板：PID 设置、速度参数设置、参数查询。"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QRadioButton, QButtonGroup, QPushButton,
    QDoubleSpinBox, QFormLayout, QFrame, QSpinBox, QLabel,
)
from PySide6.QtCore import Signal

from pid_settings import load_pid_settings, save_pid_settings
from protocol import (
    CMD_SET_PID_BOTH, CMD_SET_PID_A, CMD_SET_PID_B,
    CMD_SET_RC_SPEED, CMD_SET_MAX_SPEED, CMD_SET_SMOOTH_STEP,
    CMD_SET_TARGET_SPEED_AB, CMD_SET_TARGET_PWM_AB,
    ParamFrame,
    build_pid_command, build_float_command, build_dual_float_command,
    build_dual_int16_command, build_query_command,
)


class CommandPanel(QWidget):
    """命令发送面板。"""

    command_ready = Signal(bytes)

    def __init__(self, parent=None, settings_path: Path | None = None) -> None:
        super().__init__(parent)
        self._settings_path = settings_path
        self._device_pid = {
            "motor_a": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
            "motor_b": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
        }
        self._setup_ui()
        self._apply_pid_settings(load_pid_settings(self._settings_path))
        self._update_pid_mode_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        pid_group = QGroupBox("PID 设置")
        pid_layout = QVBoxLayout()
        pid_layout.setSpacing(4)

        mode_row = QHBoxLayout()
        mode_row.setSpacing(4)
        self._pid_mode_group = QButtonGroup(self)
        self._pid_mode_sync = QRadioButton("两轮同步")
        self._pid_mode_independent = QRadioButton("独立配置")
        self._pid_mode_group.addButton(self._pid_mode_sync)
        self._pid_mode_group.addButton(self._pid_mode_independent)
        mode_row.addWidget(self._pid_mode_sync)
        mode_row.addWidget(self._pid_mode_independent)
        mode_row.addStretch()
        pid_layout.addLayout(mode_row)

        pid_inputs_row = QHBoxLayout()
        pid_inputs_row.setSpacing(8)

        a_group = QGroupBox("电机 A")
        a_form = QFormLayout()
        a_form.setHorizontalSpacing(6)
        a_form.setVerticalSpacing(4)
        self._a_kp_spin = self._make_spin(0, 50000, 4, 0.1)
        self._a_ki_spin = self._make_spin(0, 50000, 4, 0.1)
        self._a_kd_spin = self._make_spin(0, 50000, 4, 0.1)
        a_form.addRow("Kp:", self._a_kp_spin)
        a_form.addRow("Ki:", self._a_ki_spin)
        a_form.addRow("Kd:", self._a_kd_spin)
        a_group.setLayout(a_form)
        pid_inputs_row.addWidget(a_group)

        b_group = QGroupBox("电机 B")
        b_form = QFormLayout()
        b_form.setHorizontalSpacing(6)
        b_form.setVerticalSpacing(4)
        self._b_kp_spin = self._make_spin(0, 50000, 4, 0.1)
        self._b_ki_spin = self._make_spin(0, 50000, 4, 0.1)
        self._b_kd_spin = self._make_spin(0, 50000, 4, 0.1)
        b_form.addRow("Kp:", self._b_kp_spin)
        b_form.addRow("Ki:", self._b_ki_spin)
        b_form.addRow("Kd:", self._b_kd_spin)
        b_group.setLayout(b_form)
        pid_inputs_row.addWidget(b_group)

        pid_layout.addLayout(pid_inputs_row)

        send_row = QHBoxLayout()
        send_row.setSpacing(4)
        self._send_pid_both_btn = QPushButton("同步设置 A/B")
        self._send_pid_both_btn.clicked.connect(self._send_pid_both_from_a)
        send_row.addWidget(self._send_pid_both_btn)
        self._send_pid_a_btn = QPushButton("设置 A")
        self._send_pid_a_btn.clicked.connect(self._send_pid_a)
        send_row.addWidget(self._send_pid_a_btn)
        self._send_pid_b_btn = QPushButton("设置 B")
        self._send_pid_b_btn.clicked.connect(self._send_pid_b)
        send_row.addWidget(self._send_pid_b_btn)
        pid_layout.addLayout(send_row)

        load_row = QHBoxLayout()
        load_row.setSpacing(4)
        load_row.addWidget(QLabel("载入设备当前值:"))
        self._load_device_a_btn = QPushButton("载入设备 A")
        self._load_device_a_btn.clicked.connect(self._load_device_pid_a)
        self._load_device_a_btn.setEnabled(False)
        load_row.addWidget(self._load_device_a_btn)
        self._load_device_b_btn = QPushButton("载入设备 B")
        self._load_device_b_btn.clicked.connect(self._load_device_pid_b)
        self._load_device_b_btn.setEnabled(False)
        load_row.addWidget(self._load_device_b_btn)
        load_row.addStretch()
        pid_layout.addLayout(load_row)

        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)

        speed_group = QGroupBox("速度参数")
        speed_layout = QVBoxLayout()
        speed_layout.setSpacing(4)
        speed_form = QFormLayout()
        speed_form.setHorizontalSpacing(6)
        speed_form.setVerticalSpacing(4)

        rc_row = QHBoxLayout()
        rc_row.setSpacing(4)
        self._rc_speed_spin = self._make_spin(0.01, 10000, 2, 10)
        self._rc_speed_spin.setValue(100)
        rc_row.addWidget(self._rc_speed_spin)
        rc_btn = QPushButton("设置")
        rc_btn.clicked.connect(
            lambda: self._send_float(CMD_SET_RC_SPEED, self._rc_speed_spin.value())
        )
        rc_row.addWidget(rc_btn)
        speed_form.addRow("遥控速度:", rc_row)

        ms_row = QHBoxLayout()
        ms_row.setSpacing(4)
        self._max_speed_spin = self._make_spin(0.01, 10, 3, 0.1)
        self._max_speed_spin.setValue(1.0)
        ms_row.addWidget(self._max_speed_spin)
        ms_btn = QPushButton("设置")
        ms_btn.clicked.connect(
            lambda: self._send_float(CMD_SET_MAX_SPEED, self._max_speed_spin.value())
        )
        ms_row.addWidget(ms_btn)
        speed_form.addRow("最大速度:", ms_row)

        sm_row = QHBoxLayout()
        sm_row.setSpacing(4)
        self._smooth_spin = self._make_spin(0.001, 1.0, 4, 0.01)
        self._smooth_spin.setValue(0.01)
        sm_row.addWidget(self._smooth_spin)
        sm_btn = QPushButton("设置")
        sm_btn.clicked.connect(
            lambda: self._send_float(CMD_SET_SMOOTH_STEP, self._smooth_spin.value())
        )
        sm_row.addWidget(sm_btn)
        speed_form.addRow("平滑步进:", sm_row)

        speed_layout.addLayout(speed_form)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        speed_layout.addWidget(line)

        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        target_group = QGroupBox("USART1 目标控制")
        target_layout = QVBoxLayout()
        target_layout.setSpacing(4)
        target_form = QFormLayout()
        target_form.setHorizontalSpacing(6)
        target_form.setVerticalSpacing(4)

        speed_ab_row = QHBoxLayout()
        speed_ab_row.setSpacing(4)
        self._target_speed_a_spin = self._make_spin(0.0, 10.0, 3, 0.1)
        self._target_speed_b_spin = self._make_spin(0.0, 10.0, 3, 0.1)
        speed_ab_row.addWidget(self._target_speed_a_spin)
        speed_ab_row.addWidget(self._target_speed_b_spin)
        speed_ab_btn = QPushButton("发送速度")
        speed_ab_btn.clicked.connect(self._send_target_speed)
        speed_ab_row.addWidget(speed_ab_btn)
        target_form.addRow("A/B 速度:", speed_ab_row)

        pwm_ab_row = QHBoxLayout()
        pwm_ab_row.setSpacing(4)
        self._target_pwm_a_spin = self._make_int_spin(0, 16800, 100)
        self._target_pwm_b_spin = self._make_int_spin(0, 16800, 100)
        pwm_ab_row.addWidget(self._target_pwm_a_spin)
        pwm_ab_row.addWidget(self._target_pwm_b_spin)
        pwm_ab_btn = QPushButton("发送 PWM")
        pwm_ab_btn.clicked.connect(self._send_target_pwm)
        pwm_ab_row.addWidget(pwm_ab_btn)
        target_form.addRow("A/B PWM:", pwm_ab_row)

        target_layout.addLayout(target_form)
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)

        self._query_btn = QPushButton("查询参数")
        self._query_btn.clicked.connect(self._query_params)
        layout.addWidget(self._query_btn)
        layout.addStretch()

        for spin in (
            self._a_kp_spin, self._a_ki_spin, self._a_kd_spin,
            self._b_kp_spin, self._b_ki_spin, self._b_kd_spin,
        ):
            spin.valueChanged.connect(self._save_current_pid_settings)
        self._pid_mode_sync.toggled.connect(self._on_pid_mode_toggled)
        self._pid_mode_independent.toggled.connect(self._on_pid_mode_toggled)

    def fill_params(self, frame: ParamFrame) -> None:
        """收到参数帧后更新设备当前值与非 PID 参数。"""
        self._device_pid = {
            "motor_a": {"kp": frame.A_kp, "ki": frame.A_ki, "kd": frame.A_kd},
            "motor_b": {"kp": frame.B_kp, "ki": frame.B_ki, "kd": frame.B_kd},
        }
        self._load_device_a_btn.setEnabled(True)
        self._load_device_b_btn.setEnabled(True)
        self._rc_speed_spin.setValue(frame.rc_speed)
        self._max_speed_spin.setValue(frame.limt_max_speed)
        self._smooth_spin.setValue(frame.smooth_MotorStep)

    def _send_pid_both_from_a(self) -> None:
        self.command_ready.emit(
            build_pid_command(
                CMD_SET_PID_BOTH,
                self._a_kp_spin.value(),
                self._a_ki_spin.value(),
                self._a_kd_spin.value(),
            )
        )

    def _send_pid_a(self) -> None:
        self.command_ready.emit(
            build_pid_command(
                CMD_SET_PID_A,
                self._a_kp_spin.value(),
                self._a_ki_spin.value(),
                self._a_kd_spin.value(),
            )
        )

    def _send_pid_b(self) -> None:
        self.command_ready.emit(
            build_pid_command(
                CMD_SET_PID_B,
                self._b_kp_spin.value(),
                self._b_ki_spin.value(),
                self._b_kd_spin.value(),
            )
        )

    def _send_float(self, cmd_id: int, value: float) -> None:
        self.command_ready.emit(build_float_command(cmd_id, value))

    def _query_params(self) -> None:
        self.command_ready.emit(build_query_command())

    def _send_target_speed(self) -> None:
        self.command_ready.emit(
            build_dual_float_command(
                CMD_SET_TARGET_SPEED_AB,
                self._target_speed_a_spin.value(),
                self._target_speed_b_spin.value(),
            )
        )

    def _send_target_pwm(self) -> None:
        self.command_ready.emit(
            build_dual_int16_command(
                CMD_SET_TARGET_PWM_AB,
                self._target_pwm_a_spin.value(),
                self._target_pwm_b_spin.value(),
            )
        )

    def current_pid_values_int(self) -> tuple[int, int, int]:
        """返回 A 组 PID 值，按整数位导出用于文件名。"""
        return (
            int(self._a_kp_spin.value()),
            int(self._a_ki_spin.value()),
            int(self._a_kd_spin.value()),
        )

    def current_motor_pid_values_int(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        """返回 A/B 两组 PID 值，按整数位导出用于文件名。"""
        return (
            (
                int(self._a_kp_spin.value()),
                int(self._a_ki_spin.value()),
                int(self._a_kd_spin.value()),
            ),
            (
                int(self._b_kp_spin.value()),
                int(self._b_ki_spin.value()),
                int(self._b_kd_spin.value()),
            ),
        )

    def _on_pid_mode_toggled(self) -> None:
        self._update_pid_mode_ui()
        self._save_current_pid_settings()

    def _update_pid_mode_ui(self) -> None:
        sync_mode = self._pid_mode_sync.isChecked()
        for widget in (self._b_kp_spin, self._b_ki_spin, self._b_kd_spin):
            widget.setEnabled(not sync_mode)
        self._send_pid_both_btn.setVisible(sync_mode)
        self._send_pid_a_btn.setVisible(not sync_mode)
        self._send_pid_b_btn.setVisible(not sync_mode)

    def _save_current_pid_settings(self) -> None:
        save_pid_settings(self._collect_pid_settings(), self._settings_path)

    def _collect_pid_settings(self) -> dict:
        return {
            "pid_mode": "sync" if self._pid_mode_sync.isChecked() else "independent",
            "motor_a": {
                "kp": self._a_kp_spin.value(),
                "ki": self._a_ki_spin.value(),
                "kd": self._a_kd_spin.value(),
            },
            "motor_b": {
                "kp": self._b_kp_spin.value(),
                "ki": self._b_ki_spin.value(),
                "kd": self._b_kd_spin.value(),
            },
        }

    def _apply_pid_settings(self, settings: dict) -> None:
        all_widgets = [
            self._pid_mode_sync,
            self._pid_mode_independent,
            self._a_kp_spin, self._a_ki_spin, self._a_kd_spin,
            self._b_kp_spin, self._b_ki_spin, self._b_kd_spin,
        ]
        for widget in all_widgets:
            widget.blockSignals(True)

        self._pid_mode_sync.setChecked(settings.get("pid_mode") != "independent")
        self._pid_mode_independent.setChecked(settings.get("pid_mode") == "independent")
        self._a_kp_spin.setValue(settings["motor_a"]["kp"])
        self._a_ki_spin.setValue(settings["motor_a"]["ki"])
        self._a_kd_spin.setValue(settings["motor_a"]["kd"])
        self._b_kp_spin.setValue(settings["motor_b"]["kp"])
        self._b_ki_spin.setValue(settings["motor_b"]["ki"])
        self._b_kd_spin.setValue(settings["motor_b"]["kd"])

        for widget in all_widgets:
            widget.blockSignals(False)

    def _load_device_pid_a(self) -> None:
        self._set_motor_pid_values("motor_a", self._device_pid["motor_a"])

    def _load_device_pid_b(self) -> None:
        self._set_motor_pid_values("motor_b", self._device_pid["motor_b"])

    def _set_motor_pid_values(self, motor_key: str, values: dict[str, float]) -> None:
        if motor_key == "motor_a":
            spins = (self._a_kp_spin, self._a_ki_spin, self._a_kd_spin)
        else:
            spins = (self._b_kp_spin, self._b_ki_spin, self._b_kd_spin)
        for spin, field in zip(spins, ("kp", "ki", "kd")):
            spin.setValue(values[field])

    @staticmethod
    def _make_spin(min_val: float, max_val: float, decimals: int, step: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setMaximumWidth(120)
        return spin

    @staticmethod
    def _make_int_spin(min_val: int, max_val: int, step: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(min_val, max_val)
        spin.setSingleStep(step)
        spin.setMaximumWidth(120)
        return spin

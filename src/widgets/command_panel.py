"""命令发送面板：PID 设置、速度参数设置、参数查询。"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QRadioButton, QButtonGroup, QPushButton,
    QDoubleSpinBox, QFormLayout, QFrame, QSpinBox,
)
from PySide6.QtCore import Signal

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

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        pid_group = QGroupBox("PID 设置")
        pid_layout = QVBoxLayout()

        target_row = QHBoxLayout()
        self._pid_target = QButtonGroup(self)
        for text, cmd_id in [
            ("两轮同步", CMD_SET_PID_BOTH),
            ("电机 A", CMD_SET_PID_A),
            ("电机 B", CMD_SET_PID_B),
        ]:
            rb = QRadioButton(text)
            self._pid_target.addButton(rb, cmd_id)
            target_row.addWidget(rb)
        self._pid_target.button(CMD_SET_PID_BOTH).setChecked(True)
        pid_layout.addLayout(target_row)

        pid_form = QFormLayout()
        self._kp_spin = self._make_spin(0, 50000, 4, 0.1)
        self._ki_spin = self._make_spin(0, 50000, 4, 0.1)
        self._kd_spin = self._make_spin(0, 50000, 4, 0.1)
        pid_form.addRow("Kp:", self._kp_spin)
        pid_form.addRow("Ki:", self._ki_spin)
        pid_form.addRow("Kd:", self._kd_spin)
        pid_layout.addLayout(pid_form)

        self._send_pid_btn = QPushButton("设置 PID")
        self._send_pid_btn.clicked.connect(self._send_pid)
        pid_layout.addWidget(self._send_pid_btn)

        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)

        speed_group = QGroupBox("速度参数")
        speed_layout = QVBoxLayout()
        speed_form = QFormLayout()

        rc_row = QHBoxLayout()
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

        self._query_btn = QPushButton("查询参数")
        self._query_btn.clicked.connect(self._query_params)
        speed_layout.addWidget(self._query_btn)

        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        target_group = QGroupBox("USART1 目标控制")
        target_layout = QVBoxLayout()
        target_form = QFormLayout()

        speed_ab_row = QHBoxLayout()
        self._target_speed_a_spin = self._make_spin(0.0, 10.0, 3, 0.1)
        self._target_speed_b_spin = self._make_spin(0.0, 10.0, 3, 0.1)
        speed_ab_row.addWidget(self._target_speed_a_spin)
        speed_ab_row.addWidget(self._target_speed_b_spin)
        speed_ab_btn = QPushButton("发送速度")
        speed_ab_btn.clicked.connect(self._send_target_speed)
        speed_ab_row.addWidget(speed_ab_btn)
        target_form.addRow("A/B 速度:", speed_ab_row)

        pwm_ab_row = QHBoxLayout()
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

    def fill_params(self, frame: ParamFrame) -> None:
        """收到参数帧后回填当前值到 SpinBox，方便用户微调。"""
        self._kp_spin.setValue(frame.A_kp)
        self._ki_spin.setValue(frame.A_ki)
        self._kd_spin.setValue(frame.A_kd)
        self._rc_speed_spin.setValue(frame.rc_speed)
        self._max_speed_spin.setValue(frame.limt_max_speed)
        self._smooth_spin.setValue(frame.smooth_MotorStep)

    def _send_pid(self) -> None:
        cmd_id = self._pid_target.checkedId()
        data = build_pid_command(
            cmd_id,
            self._kp_spin.value(),
            self._ki_spin.value(),
            self._kd_spin.value(),
        )
        self.command_ready.emit(data)

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
        """返回当前面板中的 PID 值，按整数位导出用于文件名。"""
        return (
            int(self._kp_spin.value()),
            int(self._ki_spin.value()),
            int(self._kd_spin.value()),
        )

    @staticmethod
    def _make_spin(min_val: float, max_val: float, decimals: int, step: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        return spin

    @staticmethod
    def _make_int_spin(min_val: int, max_val: int, step: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(min_val, max_val)
        spin.setSingleStep(step)
        return spin

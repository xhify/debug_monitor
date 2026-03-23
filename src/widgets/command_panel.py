"""命令发送面板：PID 设置、速度参数设置、参数查询。"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QRadioButton, QButtonGroup, QPushButton,
    QDoubleSpinBox, QFormLayout, QLabel, QFrame,
)
from PySide6.QtCore import Signal

from protocol import (
    CMD_SET_PID_BOTH, CMD_SET_PID_A, CMD_SET_PID_B,
    CMD_SET_RC_SPEED, CMD_SET_MAX_SPEED, CMD_SET_SMOOTH_STEP,
    ParamFrame,
    build_pid_command, build_float_command, build_query_command,
)


class CommandPanel(QWidget):
    """命令发送面板。"""

    command_ready = Signal(bytes)  # 组装完成的命令帧

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # ── PID 设置区 ──────────────────────────────────
        pid_group = QGroupBox("PID 设置")
        pid_layout = QVBoxLayout()

        # 目标选择：两轮 / A / B
        target_row = QHBoxLayout()
        self._pid_target = QButtonGroup(self)
        for text, cmd_id in [("两轮同步", CMD_SET_PID_BOTH),
                              ("电机 A", CMD_SET_PID_A),
                              ("电机 B", CMD_SET_PID_B)]:
            rb = QRadioButton(text)
            self._pid_target.addButton(rb, cmd_id)
            target_row.addWidget(rb)
        self._pid_target.button(CMD_SET_PID_BOTH).setChecked(True)
        pid_layout.addLayout(target_row)

        # Kp / Ki / Kd 输入
        pid_form = QFormLayout()
        self._kp_spin = self._make_spin(0, 10000, 4, 0.1)
        self._ki_spin = self._make_spin(0, 10000, 4, 0.1)
        self._kd_spin = self._make_spin(0, 10000, 4, 0.1)
        pid_form.addRow("Kp:", self._kp_spin)
        pid_form.addRow("Ki:", self._ki_spin)
        pid_form.addRow("Kd:", self._kd_spin)
        pid_layout.addLayout(pid_form)

        # 发送按钮
        self._send_pid_btn = QPushButton("设置 PID")
        self._send_pid_btn.clicked.connect(self._send_pid)
        pid_layout.addWidget(self._send_pid_btn)

        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)

        # ── 速度参数区（当前固件不支持从调试串口下发速度命令，控件已禁用）──
        speed_group = QGroupBox("速度参数（不可用）")
        speed_layout = QVBoxLayout()

        speed_form = QFormLayout()

        # 遥控速度限制
        rc_row = QHBoxLayout()
        self._rc_speed_spin = self._make_spin(0.01, 10000, 2, 10)
        self._rc_speed_spin.setValue(100)
        self._rc_speed_spin.setEnabled(False)
        rc_row.addWidget(self._rc_speed_spin)
        rc_btn = QPushButton("设置")
        rc_btn.setEnabled(False)
        rc_row.addWidget(rc_btn)
        speed_form.addRow("遥控速度:", rc_row)

        # 最大速度限制
        ms_row = QHBoxLayout()
        self._max_speed_spin = self._make_spin(0.01, 10, 3, 0.1)
        self._max_speed_spin.setValue(1.0)
        self._max_speed_spin.setEnabled(False)
        ms_row.addWidget(self._max_speed_spin)
        ms_btn = QPushButton("设置")
        ms_btn.setEnabled(False)
        ms_row.addWidget(ms_btn)
        speed_form.addRow("最大速度:", ms_row)

        # 平滑步进
        sm_row = QHBoxLayout()
        self._smooth_spin = self._make_spin(0.001, 1.0, 4, 0.01)
        self._smooth_spin.setValue(0.01)
        self._smooth_spin.setEnabled(False)
        sm_row.addWidget(self._smooth_spin)
        sm_btn = QPushButton("设置")
        sm_btn.setEnabled(False)
        sm_row.addWidget(sm_btn)
        speed_form.addRow("平滑步进:", sm_row)

        speed_layout.addLayout(speed_form)

        # 分隔线 + 查询按钮（查询仍可用）
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        speed_layout.addWidget(line)

        self._query_btn = QPushButton("查询参数")
        self._query_btn.clicked.connect(self._query_params)
        speed_layout.addWidget(self._query_btn)

        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

    def fill_params(self, frame: ParamFrame) -> None:
        """收到参数帧后回填当前值到 SpinBox，方便用户微调。"""
        self._kp_spin.setValue(frame.A_kp)
        self._ki_spin.setValue(frame.A_ki)
        self._kd_spin.setValue(frame.A_kd)
        # 速度参数控件已禁用，不回填

    # ─── 内部方法 ─────────────────────────────────────────

    def _send_pid(self) -> None:
        cmd_id = self._pid_target.checkedId()
        data = build_pid_command(cmd_id, self._kp_spin.value(),
                                 self._ki_spin.value(), self._kd_spin.value())
        self.command_ready.emit(data)

    def _send_float(self, cmd_id: int, value: float) -> None:
        data = build_float_command(cmd_id, value)
        self.command_ready.emit(data)

    def _query_params(self) -> None:
        data = build_query_command()
        self.command_ready.emit(data)

    @staticmethod
    def _make_spin(min_val: float, max_val: float, decimals: int, step: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        return spin

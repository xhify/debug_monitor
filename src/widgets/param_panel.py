"""固件参数显示面板：展示由参数帧 (0x02) 回传的当前固件参数值。"""

from PySide6.QtWidgets import QWidget, QFormLayout, QGroupBox, QLabel, QVBoxLayout

from protocol import ParamFrame


class ParamPanel(QWidget):
    """固件参数显示面板。"""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._labels: dict[str, QLabel] = {}
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox("固件参数")
        form = QFormLayout()

        # 参数字段定义：(显示名, 字段key)
        fields = [
            ("A Kp:", "A_kp"),
            ("A Ki:", "A_ki"),
            ("A Kd:", "A_kd"),
            ("B Kp:", "B_kp"),
            ("B Ki:", "B_ki"),
            ("B Kd:", "B_kd"),
            ("遥控速度限制:", "rc_speed"),
            ("最大速度:", "limt_max_speed"),
            ("平滑步进:", "smooth_MotorStep"),
        ]

        for label_text, key in fields:
            lbl = QLabel("---")
            lbl.setMinimumWidth(100)
            form.addRow(label_text, lbl)
            self._labels[key] = lbl

        group.setLayout(form)
        layout.addWidget(group)

    def update_params(self, frame: ParamFrame) -> None:
        """收到参数帧后更新所有标签。"""
        for key, lbl in self._labels.items():
            value = getattr(frame, key)
            lbl.setText(f"{value:.6f}")

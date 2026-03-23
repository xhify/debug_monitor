"""串口连接面板：端口选择、波特率、刷新/连接/断开按钮。"""

from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QComboBox, QPushButton, QLabel,
)
from PySide6.QtCore import Signal

from serial_worker import SerialWorker


class SerialPanel(QWidget):
    """串口连接控制面板。"""

    connect_requested = Signal(str, int)    # (端口, 波特率)
    disconnect_requested = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # 端口选择（可编辑，支持手动输入 socket://host:port 等 URL）
        layout.addWidget(QLabel("端口:"))
        self._port_combo = QComboBox()
        self._port_combo.setEditable(True)
        self._port_combo.setMinimumWidth(200)
        layout.addWidget(self._port_combo)

        # 刷新按钮
        self._refresh_btn = QPushButton("刷新")
        self._refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(self._refresh_btn)

        # 波特率选择
        layout.addWidget(QLabel("波特率:"))
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(['115200', '9600', '19200', '38400', '57600', '230400', '460800'])
        layout.addWidget(self._baud_combo)

        # 连接/断开按钮
        self._connect_btn = QPushButton("连接")
        self._connect_btn.clicked.connect(self._on_connect)
        layout.addWidget(self._connect_btn)

        self._disconnect_btn = QPushButton("断开")
        self._disconnect_btn.setEnabled(False)
        self._disconnect_btn.clicked.connect(self._on_disconnect)
        layout.addWidget(self._disconnect_btn)

        # 状态标签
        self._status_label = QLabel("未连接")
        layout.addWidget(self._status_label)

        layout.addStretch()

        # 首次扫描端口
        self.refresh_ports()

    def refresh_ports(self) -> None:
        """刷新可用串口列表。"""
        self._port_combo.clear()
        ports = SerialWorker.list_ports()
        for device, description in ports:
            self._port_combo.addItem(f"{device} - {description}", device)

    def set_connected(self, connected: bool) -> None:
        """更新 UI 状态。"""
        self._connect_btn.setEnabled(not connected)
        self._disconnect_btn.setEnabled(connected)
        self._port_combo.setEnabled(not connected)
        self._baud_combo.setEnabled(not connected)
        self._refresh_btn.setEnabled(not connected)
        if connected:
            port = self._port_combo.currentData() or self._port_combo.currentText()
            baud = self._baud_combo.currentText()
            self._status_label.setText(f"已连接: {port} @ {baud}")
            self._status_label.setStyleSheet("color: green;")
        else:
            self._status_label.setText("未连接")
            self._status_label.setStyleSheet("color: red;")

    def _on_connect(self) -> None:
        port = self._port_combo.currentData()
        if not port:
            # 支持手动输入的端口或 URL（如 socket://localhost:9999）
            text = self._port_combo.currentText().strip()
            port = text.split(' - ')[0] if ' - ' in text else text
        if not port:
            self._status_label.setText("请选择串口")
            return
        baudrate = int(self._baud_combo.currentText())
        self.connect_requested.emit(port, baudrate)

    def _on_disconnect(self) -> None:
        self.disconnect_requested.emit()

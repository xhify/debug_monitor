"""QThread serial reader for YESENSE IMU streams."""

from __future__ import annotations

import time

import serial
import serial.tools.list_ports
from PySide6.QtCore import QThread, Signal

from imu_buffer import ImuBuffer
from imu_protocol import YesenseParser, assign_batch_host_times


COMMON_IMU_BAUDS = [460800, 230400, 115200, 57600, 38400, 19200, 9600, 921600]
SERIAL_READ_SIZE = 1024
SERIAL_READ_TIMEOUT_SECONDS = 0.02


class ImuSerialWorker(QThread):
    sample_received = Signal(object)
    error_occurred = Signal(str)
    connection_changed = Signal(bool)

    def __init__(self, buffer: ImuBuffer, parent=None) -> None:
        super().__init__(parent)
        self._buffer = buffer
        self._serial: serial.Serial | None = None
        self._running = False
        self._error_count = 0

    @property
    def error_count(self) -> int:
        return self._error_count

    def open_port(self, port: str, baudrate: int = 460800) -> None:
        try:
            self._serial = serial.serial_for_url(
                port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_READ_TIMEOUT_SECONDS,
            )
            self._running = True
            self._error_count = 0
            self.connection_changed.emit(True)
            self.start()
        except Exception as exc:
            self.error_occurred.emit(f"IMU串口连接失败: {exc}")

    def close_port(self) -> None:
        self._running = False
        self.wait(2000)
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self.connection_changed.emit(False)

    def run(self) -> None:
        parser = YesenseParser()
        while self._running:
            try:
                if not self._serial or not self._serial.is_open:
                    break

                waiting = self._serial.in_waiting
                batch_start_time = time.time()
                chunk = self._serial.read(max(1, min(max(1, waiting), SERIAL_READ_SIZE)))
                batch_end_time = time.time()
                if not chunk:
                    continue

                samples = parser.feed(chunk)
                assign_batch_host_times(samples, batch_start_time, batch_end_time)
                for sample in samples:
                    self._buffer.append(sample)
                    self.sample_received.emit(sample)
            except Exception as exc:
                if self._running:
                    self._error_count += 1
                    self.error_occurred.emit(f"IMU读取错误: {exc}")
                break

    @staticmethod
    def list_ports() -> list[tuple[str, str]]:
        return [
            (port.device, port.description)
            for port in serial.tools.list_ports.comports()
        ]


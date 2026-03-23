"""
串口通信工作线程

在 QThread 中运行阻塞式串口读取循环，使用 bytearray 缓冲区 + 状态机从字节流中
解析完整帧。数据帧直接写入共享 DataBuffer（通过 Lock 保证线程安全），参数帧通过
Qt Signal 发送到主线程。
"""

import serial
import serial.tools.list_ports
from PySide6.QtCore import QThread, Signal

from protocol import (
    HEADER1, HEADER2,
    FRAME_ID_DATA, FRAME_ID_PARAM,
    DATA_FRAME_LEN, PARAM_FRAME_LEN,
    parse_data_frame, parse_param_frame,
    ParamFrame,
)
from data_buffer import DataBuffer


class SerialWorker(QThread):
    """串口通信线程：读取字节流、解帧、写入共享缓冲区。"""

    # 参数帧为低频信号（按需），安全使用 Qt Signal
    param_received = Signal(object)
    error_occurred = Signal(str)
    connection_changed = Signal(bool)

    def __init__(self, buffer: DataBuffer, parent=None) -> None:
        super().__init__(parent)
        self._buffer = buffer
        self._serial: serial.Serial | None = None
        self._running = False
        self._error_count = 0

    @property
    def error_count(self) -> int:
        return self._error_count

    # ─── 连接控制 ─────────────────────────────────────────

    def open_port(self, port: str, baudrate: int = 115200) -> None:
        """打开串口并启动读取线程。支持真实 COM 口和 URL（如 socket://host:port）。"""
        try:
            self._serial = serial.serial_for_url(
                port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,  # 50ms 超时，平衡响应速度与 CPU 占用
            )
            self._running = True
            self._error_count = 0
            self.connection_changed.emit(True)
            self.start()
        except Exception as e:
            self.error_occurred.emit(f"串口连接失败: {e}")

    def close_port(self) -> None:
        """停止读取线程并关闭串口。"""
        self._running = False
        self.wait(2000)  # 等待线程结束，最多 2 秒
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self.connection_changed.emit(False)

    def send_command(self, data: bytes) -> None:
        """
        从主线程发送命令到串口。
        pyserial 的 write 和 read 可以从不同线程并发调用，无需额外加锁。
        """
        try:
            if self._serial and self._serial.is_open:
                self._serial.write(data)
        except Exception as e:
            self.error_occurred.emit(f"发送失败: {e}")

    # ─── 主循环 ───────────────────────────────────────────

    def run(self) -> None:
        """工作线程主循环：缓冲区式读取 + 状态机解帧。"""
        rx_buf = bytearray()

        while self._running:
            try:
                if not self._serial or not self._serial.is_open:
                    break

                # 读取所有可用字节（至少读 1 字节，阻塞到 timeout）
                waiting = self._serial.in_waiting
                chunk = self._serial.read(max(1, waiting))
                if not chunk:
                    continue

                rx_buf.extend(chunk)

                # 在缓冲区上运行解帧状态机
                self._parse_buffer(rx_buf)

            except Exception as e:
                if self._running:
                    self.error_occurred.emit(f"读取错误: {e}")
                break

    def _parse_buffer(self, buf: bytearray) -> None:
        """
        在 bytearray 缓冲区上扫描并解析完整帧。
        解析成功后从缓冲区头部移除已处理的字节。
        """
        while True:
            # 搜索帧头 0xAA 0x55
            header_pos = self._find_header(buf)
            if header_pos < 0:
                # 没有找到帧头，清空缓冲区（保留最后 1 字节，可能是 0xAA）
                if len(buf) > 1:
                    if buf[-1] == HEADER1:
                        del buf[:-1]
                    else:
                        buf.clear()
                return

            # 丢弃帧头之前的垃圾字节
            if header_pos > 0:
                del buf[:header_pos]

            # 至少需要 3 字节才能确定帧类型（header1 + header2 + frame_id）
            if len(buf) < 3:
                return

            frame_id = buf[2]

            # 根据帧类型确定预期长度
            if frame_id == FRAME_ID_DATA:
                expected_len = DATA_FRAME_LEN
            elif frame_id == FRAME_ID_PARAM:
                expected_len = PARAM_FRAME_LEN
            else:
                # 未知帧类型，跳过这个帧头，继续搜索
                del buf[:2]
                self._error_count += 1
                continue

            # 检查缓冲区是否已积累足够字节
            if len(buf) < expected_len:
                return  # 等待更多数据

            # 提取帧并解析
            raw_frame = bytes(buf[:expected_len])
            del buf[:expected_len]

            self._dispatch_frame(frame_id, raw_frame)

    def _find_header(self, buf: bytearray) -> int:
        """在缓冲区中查找 0xAA 0x55 帧头，返回起始位置，未找到返回 -1。"""
        pos = 0
        while pos < len(buf) - 1:
            idx = buf.find(HEADER1, pos)
            if idx < 0 or idx + 1 >= len(buf):
                return -1
            if buf[idx + 1] == HEADER2:
                return idx
            pos = idx + 1
        return -1

    def _dispatch_frame(self, frame_id: int, raw: bytes) -> None:
        """分发解析后的帧到相应处理逻辑。"""
        if frame_id == FRAME_ID_DATA:
            frame = parse_data_frame(raw)
            if frame:
                # 直接写入共享缓冲区（DataBuffer 内部加锁）
                self._buffer.append(frame)
            else:
                self._error_count += 1
        elif frame_id == FRAME_ID_PARAM:
            frame = parse_param_frame(raw)
            if frame:
                self.param_received.emit(frame)
            else:
                self._error_count += 1

    # ─── 工具方法 ─────────────────────────────────────────

    @staticmethod
    def list_ports() -> list[tuple[str, str]]:
        """
        列出可用串口。
        返回 [(设备名, 描述), ...] 列表，例如 [('COM3', 'USB Serial Port')]。
        """
        return [
            (p.device, p.description)
            for p in serial.tools.list_ports.comports()
        ]

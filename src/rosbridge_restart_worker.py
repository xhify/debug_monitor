"""Background helper for restarting only the remote ROSbridge websocket node."""

from __future__ import annotations

from dataclasses import dataclass
import socket
import subprocess
import time
from typing import Callable

from PySide6.QtCore import QObject, Signal, Slot

from app_config import (
    DEFAULT_ROS_SSH_USER,
    DEFAULT_SSH_EXECUTABLE,
    ROSBRIDGE_RESTART_PROBE_INTERVAL_S,
    ROSBRIDGE_RESTART_TIMEOUT_S,
)


@dataclass(frozen=True, slots=True)
class RosbridgeRestartConfig:
    host: str
    port: int
    username: str = DEFAULT_ROS_SSH_USER
    ssh_executable: str = DEFAULT_SSH_EXECUTABLE
    timeout_s: float = ROSBRIDGE_RESTART_TIMEOUT_S
    probe_interval_s: float = ROSBRIDGE_RESTART_PROBE_INTERVAL_S


class RosbridgeRestartWorker(QObject):
    """Restart ROSbridge over passwordless SSH and wait for its TCP port."""

    progress = Signal(str)
    finished = Signal(object)
    error = Signal(str)

    def __init__(
        self,
        config: RosbridgeRestartConfig,
        *,
        process_runner: Callable[..., object] | None = None,
        port_probe: Callable[[str, int, float], bool] | None = None,
        monotonic_clock: Callable[[], float] | None = None,
        sleep: Callable[[float], None] | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.config = config
        self._process_runner = process_runner or subprocess.run
        self._port_probe = port_probe or self._default_port_probe
        self._monotonic_clock = monotonic_clock or time.monotonic
        self._sleep = sleep or time.sleep

    @Slot()
    def run(self) -> None:
        try:
            result = self.restart()
        except Exception as exc:
            self.error.emit(str(exc))
        else:
            self.finished.emit(result)

    def restart(self) -> dict[str, object]:
        self.progress.emit("正在通过 SSH 重启 ROSbridge")
        result = self._process_runner(
            self._ssh_args(),
            capture_output=True,
            text=True,
            timeout=10,
        )
        if int(getattr(result, "returncode", 1)) != 0:
            detail = str(getattr(result, "stderr", "")).strip()
            raise RuntimeError(detail or "SSH 重启 ROSbridge 失败")

        self.progress.emit("ROSbridge 启动命令已发送，正在等待端口")
        self._wait_for_port()
        return {
            "ok": True,
            "host": self.config.host,
            "port": int(self.config.port),
        }

    def _ssh_args(self) -> list[str]:
        remote_command = (
            "source /opt/ros/noetic/setup.bash; "
            "rosnode kill /rosbridge_websocket >/dev/null 2>&1 || true; "
            "nohup roslaunch rosbridge_server rosbridge_websocket.launch "
            f"port:={int(self.config.port)} address:=0.0.0.0 "
            ">/tmp/debug_monitor_rosbridge.log 2>&1 </dev/null &"
        )
        return [
            self.config.ssh_executable,
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            f"{self.config.username}@{self.config.host}",
            remote_command,
        ]

    def _wait_for_port(self) -> None:
        deadline = self._monotonic_clock() + max(0.0, self.config.timeout_s)
        while True:
            if self._port_probe(self.config.host, int(self.config.port), 0.5):
                return
            if self._monotonic_clock() >= deadline:
                raise TimeoutError(
                    f"等待 ROSbridge 端口 {self.config.host}:{self.config.port} 超时"
                )
            self._sleep(max(0.0, self.config.probe_interval_s))

    @staticmethod
    def _default_port_probe(host: str, port: int, timeout: float) -> bool:
        try:
            with socket.create_connection((host, int(port)), timeout=timeout):
                return True
        except OSError:
            return False

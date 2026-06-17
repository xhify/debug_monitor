"""后台同步车端 rosbag 到本地目录。"""

from __future__ import annotations

from pathlib import Path
import subprocess
from typing import Callable

from PySide6.QtCore import QObject, Signal, Slot


class RosbagSyncWorker(QObject):
    progress = Signal(str)
    finished = Signal(object)
    error = Signal(str)

    def __init__(
        self,
        *,
        host: str,
        remote_dir: str,
        local_dir: Path,
        username: str = "wheeltec",
        process_runner: Callable[..., object] | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.host = host
        self.username = username
        self.remote_dir = remote_dir.rstrip("/")
        self.local_dir = Path(local_dir)
        self._process_runner = process_runner or subprocess.run

    @Slot()
    def run(self) -> None:
        self.local_dir.mkdir(parents=True, exist_ok=True)
        remote = f"{self.username}@{self.host}:{self.remote_dir}/"
        rsync_cmd = ["rsync", "-avP", remote, str(self.local_dir)]
        rsync_result = self._run_command(rsync_cmd, "rsync")
        if rsync_result is not None and getattr(rsync_result, "returncode", 1) == 0:
            self.finished.emit(self._result("rsync", rsync_result))
            return

        self.progress.emit("rsync 失败，正在降级为 scp")
        scp_remote = f"{self.username}@{self.host}:{self.remote_dir}/."
        scp_cmd = ["scp", "-r", scp_remote, str(self.local_dir)]
        scp_result = self._run_command(scp_cmd, "scp")
        if scp_result is not None and getattr(scp_result, "returncode", 1) == 0:
            self.finished.emit(self._result("scp", scp_result))
            return

        stderr = "" if scp_result is None else str(getattr(scp_result, "stderr", ""))
        self.error.emit(stderr or "scp 同步失败")

    def _run_command(self, args: list[str], method: str):
        self.progress.emit(f"开始 {method}: {' '.join(args)}")
        try:
            result = self._process_runner(args, capture_output=True, text=True)
        except FileNotFoundError as exc:
            self.progress.emit(f"{method} 不可用: {exc}")
            return None
        output = "\n".join(
            part.strip()
            for part in (getattr(result, "stdout", ""), getattr(result, "stderr", ""))
            if str(part).strip()
        )
        if output:
            self.progress.emit(output[-1000:])
        return result

    def _result(self, method: str, result: object) -> dict[str, object]:
        return {
            "method": method,
            "local_dir": str(self.local_dir),
            "remote_dir": self.remote_dir,
            "returncode": int(getattr(result, "returncode", 0)),
        }

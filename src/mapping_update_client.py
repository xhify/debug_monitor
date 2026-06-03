"""Remote ROS map update enable/disable control."""

from __future__ import annotations

import subprocess
from typing import Callable


DEFAULT_MAPPING_SSH_HOST = "wheeltec14"
DEFAULT_TIMEOUT_SECONDS = 8.0
ROS_SETUP_COMMAND = "source /opt/ros/noetic/setup.bash"
MAP_UPDATE_PARAM = "/mapping/map_update_enable"


class MappingUpdateError(RuntimeError):
    """Raised when the remote ROS map update parameter cannot be changed."""


class MappingUpdateClient:
    """Toggle FAST-LIO mapping updates through a configurable remote command."""

    def __init__(
        self,
        ssh_host: str = DEFAULT_MAPPING_SSH_HOST,
        freeze_command: str | None = None,
        resume_command: str | None = None,
        timeout: float = DEFAULT_TIMEOUT_SECONDS,
        runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
    ) -> None:
        self.ssh_host = ssh_host
        self.freeze_command = freeze_command
        self.resume_command = resume_command
        self.timeout = float(timeout)
        self._runner = runner

    def set_map_update_enabled(self, enabled: bool) -> dict[str, object]:
        remote_command = self._command_for_enabled(enabled)
        args = ["ssh", self.ssh_host, remote_command]
        try:
            result = self._runner(
                args,
                capture_output=True,
                text=True,
                check=False,
                timeout=self.timeout,
            )
        except OSError as exc:
            raise MappingUpdateError(f"建图冻结命令执行失败: {exc}") from exc
        except subprocess.TimeoutExpired as exc:
            raise MappingUpdateError(f"建图冻结命令超时: {MAP_UPDATE_PARAM}") from exc

        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "").strip()
            if detail:
                raise MappingUpdateError(f"{MAP_UPDATE_PARAM} 设置失败: {detail}")
            raise MappingUpdateError(f"{MAP_UPDATE_PARAM} 设置失败: returncode={result.returncode}")

        return {
            "enabled": bool(enabled),
            "method": "custom" if self._uses_custom_command(enabled) else "rosparam",
            "command": " ".join(args),
            "stdout": (result.stdout or "").strip(),
            "stderr": (result.stderr or "").strip(),
        }

    def _command_for_enabled(self, enabled: bool) -> str:
        custom_command = self.resume_command if enabled else self.freeze_command
        if custom_command:
            return custom_command
        value = "true" if enabled else "false"
        return f"{ROS_SETUP_COMMAND} && rosparam set {MAP_UPDATE_PARAM} {value}"

    def _uses_custom_command(self, enabled: bool) -> bool:
        return bool(self.resume_command if enabled else self.freeze_command)

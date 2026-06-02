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
    """Set the FAST-LIO mapping accumulator update parameter over SSH."""

    def __init__(
        self,
        ssh_host: str = DEFAULT_MAPPING_SSH_HOST,
        timeout: float = DEFAULT_TIMEOUT_SECONDS,
        runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
    ) -> None:
        self.ssh_host = ssh_host
        self.timeout = float(timeout)
        self._runner = runner

    def set_map_update_enabled(self, enabled: bool) -> dict[str, object]:
        value = "true" if enabled else "false"
        remote_command = f"{ROS_SETUP_COMMAND} && rosparam set {MAP_UPDATE_PARAM} {value}"
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
            "command": " ".join(args),
            "stdout": (result.stdout or "").strip(),
            "stderr": (result.stderr or "").strip(),
        }

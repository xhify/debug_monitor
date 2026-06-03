"""One-shot frozen map fetch helpers for the FAST-LIO localization panel."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
from typing import Callable

from localization_fusion import MapPoint, read_map_points


DEFAULT_TIMEOUT_SECONDS = 12.0


class MapFetchError(RuntimeError):
    """Raised when a frozen map snapshot cannot be fetched."""


@dataclass(frozen=True, slots=True)
class MapFetchConfig:
    ssh_host: str = "wheeltec14"
    remote_map_path: str = ""
    snapshot_command: str = ""
    local_map_path: str = ""
    timeout: float = DEFAULT_TIMEOUT_SECONDS


@dataclass(frozen=True, slots=True)
class MapFetchResult:
    local_path: Path
    source: str
    method: str
    raw_file_name: str


class MapFetchClient:
    """Fetch exactly one frozen map snapshot after mapping has been frozen."""

    def __init__(
        self,
        config: MapFetchConfig | None = None,
        runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
    ) -> None:
        self.config = config or MapFetchConfig()
        self._runner = runner

    def fetch_once(self, cache_dir: Path) -> MapFetchResult:
        config = self.config
        if config.local_map_path:
            local_path = Path(config.local_map_path)
            return MapFetchResult(
                local_path=local_path,
                source=str(local_path),
                method="local_file",
                raw_file_name=local_path.name,
            )
        if not config.remote_map_path:
            raise MapFetchError("请配置冻结地图路径")
        if not config.ssh_host:
            raise MapFetchError("请配置远程主机")

        cache_dir = Path(cache_dir)
        cache_dir.mkdir(parents=True, exist_ok=True)
        local_path = cache_dir / Path(config.remote_map_path).name

        if config.snapshot_command:
            self._run(["ssh", config.ssh_host, config.snapshot_command], config.timeout)
        remote_source = f"{config.ssh_host}:{config.remote_map_path}"
        self._run(["scp", remote_source, str(local_path)], config.timeout)
        return MapFetchResult(
            local_path=local_path,
            source=remote_source,
            method="remote_file",
            raw_file_name=local_path.name,
        )

    def read_points(self, path: Path, max_points: int | None = 50000) -> list[MapPoint]:
        return read_map_points(path, max_points=max_points)

    def _run(self, args: list[str], timeout: float) -> subprocess.CompletedProcess:
        try:
            result = self._runner(
                args,
                capture_output=True,
                text=True,
                check=False,
                timeout=timeout,
            )
        except OSError as exc:
            raise MapFetchError(f"地图获取命令执行失败: {exc}") from exc
        except subprocess.TimeoutExpired as exc:
            raise MapFetchError("地图获取命令超时") from exc

        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "").strip()
            raise MapFetchError(detail or f"地图获取命令失败: returncode={result.returncode}")
        return result

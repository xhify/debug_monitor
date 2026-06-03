"""One-shot frozen map fetch helpers for the FAST-LIO localization panel."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import subprocess
from typing import Callable

from localization_fusion import MapPoint, read_map_points


DEFAULT_TIMEOUT_SECONDS = 12.0
ROS_PYTHONPATH = "/opt/ros/noetic/lib/python3/dist-packages"


class MapFetchError(RuntimeError):
    """Raised when a frozen map snapshot cannot be fetched."""


@dataclass(frozen=True, slots=True)
class MapFetchConfig:
    ssh_host: str = "wheeltec14"
    map_topic: str = ""
    remote_snapshot_path: str = ""
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
        if config.map_topic:
            return self._fetch_ros_topic_snapshot(cache_dir)
        if not config.remote_map_path:
            raise MapFetchError("请配置冻结地图来源")
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

    def _fetch_ros_topic_snapshot(self, cache_dir: Path) -> MapFetchResult:
        config = self.config
        if not config.ssh_host:
            raise MapFetchError("请配置远程主机")

        remote_path = config.remote_snapshot_path or self._default_remote_snapshot_path(config.map_topic)
        cache_dir = Path(cache_dir)
        cache_dir.mkdir(parents=True, exist_ok=True)
        local_path = cache_dir / Path(remote_path).name
        script = _pointcloud2_snapshot_script(
            topic=config.map_topic,
            output_path=remote_path,
            timeout=config.timeout,
        )

        self._run(
            [
                "ssh",
                config.ssh_host,
                f"env PYTHONPATH={ROS_PYTHONPATH} python3 -",
            ],
            config.timeout,
            input_text=script,
        )
        self._run(["scp", f"{config.ssh_host}:{remote_path}", str(local_path)], config.timeout)
        return MapFetchResult(
            local_path=local_path,
            source=config.map_topic,
            method="ros_topic_snapshot",
            raw_file_name=local_path.name,
        )

    @staticmethod
    def _default_remote_snapshot_path(topic: str) -> str:
        safe_topic = topic.strip("/").replace("/", "_") or "map"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"/tmp/debug_monitor_{safe_topic}_{timestamp}.csv"

    def _run(
        self,
        args: list[str],
        timeout: float,
        input_text: str | None = None,
    ) -> subprocess.CompletedProcess:
        try:
            result = self._runner(
                args,
                capture_output=True,
                text=True,
                check=False,
                timeout=timeout,
                input=input_text,
            )
        except OSError as exc:
            raise MapFetchError(f"地图获取命令执行失败: {exc}") from exc
        except subprocess.TimeoutExpired as exc:
            raise MapFetchError("地图获取命令超时") from exc

        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "").strip()
            raise MapFetchError(detail or f"地图获取命令失败: returncode={result.returncode}")
        return result


def _pointcloud2_snapshot_script(topic: str, output_path: str, timeout: float) -> str:
    return f'''\
import csv
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

TOPIC = "{topic}"
OUTPUT = "{output_path}"
TIMEOUT = {float(timeout)!r}

rospy.init_node("debug_monitor_map_snapshot", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message(TOPIC, PointCloud2, timeout=TIMEOUT)
field_names = [field.name for field in msg.fields]
if "x" not in field_names or "y" not in field_names:
    raise RuntimeError("PointCloud2 map message must contain x/y fields")
read_fields = [name for name in ("x", "y", "z", "intensity") if name in field_names]
with open(OUTPUT, "w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=["x", "y", "z", "intensity"])
    writer.writeheader()
    for point in pc2.read_points(msg, field_names=read_fields, skip_nans=True):
        values = dict(zip(read_fields, point))
        writer.writerow({{
            "x": values.get("x", 0.0),
            "y": values.get("y", 0.0),
            "z": values.get("z", 0.0),
            "intensity": values.get("intensity", 0.0),
        }})
print("saved {{}} points from {{}} to {{}}".format(msg.width * msg.height, TOPIC, OUTPUT))
'''

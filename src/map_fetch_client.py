"""One-shot frozen map fetch helpers for the FAST-LIO localization panel."""

from __future__ import annotations

import base64
import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import struct
import threading
from typing import Any, Callable, Iterable

from app_config import DEFAULT_MAP_TOPIC, DEFAULT_ROSBRIDGE_HOST, DEFAULT_ROSBRIDGE_PORT
from localization_fusion import MapPoint, read_map_points


DEFAULT_TIMEOUT_SECONDS = 12.0


class MapFetchError(RuntimeError):
    """Raised when a frozen map snapshot cannot be fetched."""


@dataclass(frozen=True, slots=True)
class MapFetchConfig:
    host: str = DEFAULT_ROSBRIDGE_HOST
    port: int = DEFAULT_ROSBRIDGE_PORT
    map_topic: str = DEFAULT_MAP_TOPIC
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
        *,
        ros_factory: Callable[[str, int], Any] | None = None,
        topic_factory: Callable[[Any, str, str], Any] | None = None,
        runner: Callable[..., Any] | None = None,
    ) -> None:
        self.config = config or MapFetchConfig()
        self._ros_factory = ros_factory
        self._topic_factory = topic_factory
        self._runner = runner

    def fetch_once(self, cache_dir: Path) -> MapFetchResult:
        config = self.config
        if config.local_map_path:
            local_path = Path(config.local_map_path)
            if not local_path.exists():
                raise MapFetchError(f"Local map file does not exist: {local_path}")
            return MapFetchResult(
                local_path=local_path,
                source=str(local_path),
                method="local_file",
                raw_file_name=local_path.name,
            )
        if config.map_topic:
            return self._fetch_rosbridge_topic_snapshot(cache_dir)
        raise MapFetchError("Configure a map topic or a local map file path.")

    def read_points(self, path: Path, max_points: int | None = 50000) -> list[MapPoint]:
        return read_map_points(path, max_points=max_points)

    def _fetch_rosbridge_topic_snapshot(self, cache_dir: Path) -> MapFetchResult:
        config = self.config
        self._load_default_factories()
        cache_dir = Path(cache_dir)
        cache_dir.mkdir(parents=True, exist_ok=True)
        local_path = cache_dir / self._default_local_snapshot_name(config.map_topic)
        message = self._wait_for_pointcloud2_message()
        try:
            rows = pointcloud2_to_rows(message)
        except Exception as exc:
            raise MapFetchError(f"PointCloud2 parse failed for {config.map_topic}: {exc}") from exc
        _write_rows_csv(local_path, rows)
        return MapFetchResult(
            local_path=local_path,
            source=config.map_topic,
            method="rosbridge_topic_snapshot",
            raw_file_name=local_path.name,
        )

    def _wait_for_pointcloud2_message(self) -> dict:
        ros = self._ros_factory(self.config.host, int(self.config.port))
        topic = None
        event = threading.Event()
        payload: dict[str, Any] = {}
        error: list[Exception] = []

        def on_message(message: dict) -> None:
            payload["message"] = message
            event.set()

        try:
            ros.run()
            topic = self._topic_factory(ros, self.config.map_topic, "sensor_msgs/PointCloud2")
            topic.subscribe(on_message)
            if not event.wait(float(self.config.timeout)):
                raise MapFetchError(f"Map topic timed out: {self.config.map_topic}")
            message = payload.get("message")
            if not isinstance(message, dict):
                raise MapFetchError(f"Map topic returned an invalid message: {self.config.map_topic}")
            return message
        except MapFetchError:
            raise
        except Exception as exc:
            error.append(exc)
            raise MapFetchError(
                f"ROSbridge connection failed for {self.config.host}:{self.config.port}: {exc}"
            ) from exc
        finally:
            if topic is not None:
                unsubscribe = getattr(topic, "unsubscribe", None)
                if callable(unsubscribe):
                    unsubscribe()
            close = getattr(ros, "close", None)
            if callable(close):
                close()

    @staticmethod
    def _default_local_snapshot_name(topic: str) -> str:
        safe_topic = topic.strip("/").replace("/", "_") or "map"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"debug_monitor_{safe_topic}_{timestamp}.csv"

    def _load_default_factories(self) -> None:
        if self._ros_factory is not None and self._topic_factory is not None:
            return
        try:
            import roslibpy
        except ImportError as exc:
            raise MapFetchError("Missing dependency roslibpy. Install requirements.txt first.") from exc
        if self._ros_factory is None:
            self._ros_factory = roslibpy.Ros
        if self._topic_factory is None:
            self._topic_factory = roslibpy.Topic


def pointcloud2_to_rows(message: dict) -> list[tuple[float, float, float, float]]:
    fields = {field.get("name"): field for field in message.get("fields", [])}
    if "x" not in fields or "y" not in fields:
        raise MapFetchError("PointCloud2 map message must contain x/y fields.")

    width = int(message.get("width", 0))
    height = int(message.get("height", 1) or 1)
    point_step = int(message.get("point_step", 0))
    row_step = int(message.get("row_step", point_step * width))
    if width <= 0 or point_step <= 0:
        raise MapFetchError("PointCloud2 width and point_step must be positive.")
    data = _decode_pointcloud2_data(message.get("data", b""))
    endian = ">" if bool(message.get("is_bigendian", False)) else "<"

    rows: list[tuple[float, float, float, float]] = []
    for row in range(height):
        for col in range(width):
            base = row * row_step + col * point_step
            x = _read_field(data, base, fields["x"], endian)
            y = _read_field(data, base, fields["y"], endian)
            z = _read_field(data, base, fields.get("z"), endian, default=0.0)
            intensity = _read_field(data, base, fields.get("intensity"), endian, default=0.0)
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue
            rows.append((x, y, z, intensity))
    return rows


def _decode_pointcloud2_data(data: Any) -> bytes:
    if isinstance(data, bytes):
        return data
    if isinstance(data, bytearray):
        return bytes(data)
    if isinstance(data, str):
        return base64.b64decode(data)
    if isinstance(data, Iterable):
        return bytes(int(value) & 0xFF for value in data)
    raise MapFetchError("Unsupported PointCloud2 data payload.")


def _read_field(
    data: bytes,
    base: int,
    field: dict | None,
    endian: str,
    *,
    default: float = 0.0,
) -> float:
    if field is None:
        return default
    offset = base + int(field.get("offset", 0))
    datatype = int(field.get("datatype", 0))
    fmt = _POINT_FIELD_FORMATS.get(datatype)
    if fmt is None:
        raise MapFetchError(f"Unsupported PointCloud2 datatype: {datatype}")
    size = struct.calcsize(fmt)
    if offset + size > len(data):
        raise MapFetchError("PointCloud2 data is shorter than field offsets require.")
    return float(struct.unpack_from(endian + fmt, data, offset)[0])


def _write_rows_csv(path: Path, rows: Iterable[tuple[float, float, float, float]]) -> None:
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["x", "y", "z", "intensity"])
        for x, y, z, intensity in rows:
            writer.writerow([x, y, z, intensity])


_POINT_FIELD_FORMATS = {
    1: "b",
    2: "B",
    3: "h",
    4: "H",
    5: "i",
    6: "I",
    7: "f",
    8: "d",
}

"""车端 rosbag 状态模型与容错解析。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class RosbagRecordingStatus:
    active: bool = False
    state: str = "idle"
    session_id: str = ""
    duration_s: float = 0.0
    remote_dir: str = ""
    current_size_bytes: int = 0
    disk_free_gb: float = 0.0
    topics: list[str] = field(default_factory=list)
    bag_files: list[str] = field(default_factory=list)
    last_error: str = ""


@dataclass
class RemoteRosbagSession:
    session_id: str = ""
    status: str = "unknown"
    remote_dir: str = ""
    size_bytes: int = 0
    duration_s: float = 0.0
    file_count: int = 0
    topic_count: int = 0
    created_at: str = ""
    bag_files: list[str] = field(default_factory=list)
    downloaded: bool = False
    local_dir: str = ""


@dataclass
class RosbagLibraryState:
    bag_dir: str = ""
    disk_free_gb: float = 0.0
    sessions: list[RemoteRosbagSession] = field(default_factory=list)


def parse_rosbag_recording_status(payload: dict[str, Any]) -> RosbagRecordingStatus:
    data = payload.get("rosbag", payload)
    if not isinstance(data, dict):
        data = {}
    return RosbagRecordingStatus(
        active=data.get("active") is True,
        state=_str(data.get("state"), "idle"),
        session_id=_str(data.get("session_id")),
        duration_s=_float(data.get("duration_s")),
        remote_dir=_str(data.get("remote_dir")),
        current_size_bytes=_int(data.get("current_size_bytes")),
        disk_free_gb=_float(data.get("disk_free_gb")),
        topics=_str_list(data.get("topics")),
        bag_files=_str_list(data.get("bag_files")),
        last_error=_str(data.get("last_error")),
    )


def parse_rosbag_library_state(payload: dict[str, Any]) -> RosbagLibraryState:
    data = payload.get("rosbag_library", payload)
    if not isinstance(data, dict):
        data = {}
    sessions: list[RemoteRosbagSession] = []
    raw_sessions = data.get("sessions", [])
    if isinstance(raw_sessions, list):
        for item in raw_sessions:
            if not isinstance(item, dict):
                continue
            sessions.append(
                RemoteRosbagSession(
                    session_id=_str(item.get("session_id")),
                    status=_str(item.get("status"), "unknown"),
                    remote_dir=_str(item.get("remote_dir")),
                    size_bytes=_int(item.get("size_bytes")),
                    duration_s=_float(item.get("duration_s")),
                    file_count=_int(item.get("file_count")),
                    topic_count=_int(item.get("topic_count")),
                    created_at=_str(item.get("created_at")),
                    bag_files=_str_list(item.get("bag_files")),
                    downloaded=item.get("downloaded") is True,
                    local_dir=_str(item.get("local_dir")),
                )
            )
    return RosbagLibraryState(
        bag_dir=_str(data.get("bag_dir")),
        disk_free_gb=_float(data.get("disk_free_gb")),
        sessions=sessions,
    )


def format_bytes(size_bytes: int) -> str:
    size = max(0.0, float(_int(size_bytes)))
    units = ("B", "KB", "MB", "GB", "TB")
    unit = units[0]
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            break
        size /= 1024.0
    if unit == "B":
        return f"{int(size)} B"
    return f"{size:.1f} {unit}"


def format_duration(duration_s: float) -> str:
    seconds = max(0, int(_float(duration_s)))
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def _str(value: Any, default: str = "") -> str:
    if value is None:
        return default
    try:
        return str(value)
    except Exception:
        return default


def _int(value: Any) -> int:
    try:
        return max(0, int(float(value)))
    except (TypeError, ValueError):
        return 0


def _float(value: Any) -> float:
    try:
        return max(0.0, float(value))
    except (TypeError, ValueError):
        return 0.0


def _str_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [_str(item) for item in value]

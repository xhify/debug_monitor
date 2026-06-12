"""Centralized environment-backed defaults for ROSbridge integrations."""

from __future__ import annotations

import os


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None or not value.strip():
        return default
    try:
        return int(value)
    except ValueError:
        return default


DEFAULT_ROSBRIDGE_HOST = os.getenv("DEBUG_MONITOR_ROSBRIDGE_HOST", "192.168.0.100")
DEFAULT_ROSBRIDGE_PORT = _env_int("DEBUG_MONITOR_ROSBRIDGE_PORT", 9090)
DEFAULT_FASTLIO_ODOM_TOPIC = os.getenv("DEBUG_MONITOR_FASTLIO_ODOM_TOPIC", "/Odometry")
DEFAULT_MAP_TOPIC = os.getenv("DEBUG_MONITOR_MAP_TOPIC", "/Laser_map")
DEFAULT_MAP_UPDATE_PARAM = os.getenv("DEBUG_MONITOR_MAP_UPDATE_PARAM", "/mapping/map_update_enable")
DEFAULT_RECORDINGS_DIR = os.getenv("DEBUG_MONITOR_RECORDINGS_DIR", r"D:\debug_monitor\recordings")

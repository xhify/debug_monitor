"""PID 本地配置的 JSON 读写辅助。"""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path

DEFAULT_PID_SETTINGS = {
    "pid_mode": "sync",
    "motor_a": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
    "motor_b": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
}


def default_pid_settings() -> dict:
    """返回一份新的默认 PID 配置。"""
    return deepcopy(DEFAULT_PID_SETTINGS)


def load_pid_settings(path: Path | None = None) -> dict:
    """从 JSON 文件加载 PID 配置，不存在或异常时返回默认值。"""
    settings_path = path or default_settings_path()
    if not settings_path.exists():
        return default_pid_settings()

    try:
        raw = json.loads(settings_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return default_pid_settings()

    result = default_pid_settings()
    if raw.get("pid_mode") in {"sync", "independent"}:
        result["pid_mode"] = raw["pid_mode"]

    for motor_key in ("motor_a", "motor_b"):
        motor_raw = raw.get(motor_key, {})
        for field in ("kp", "ki", "kd"):
            value = motor_raw.get(field)
            if isinstance(value, (int, float)):
                result[motor_key][field] = float(value)

    return result


def save_pid_settings(settings: dict, path: Path | None = None) -> None:
    """保存 PID 配置到 JSON 文件。"""
    settings_path = path or default_settings_path()
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    settings_path.write_text(
        json.dumps(settings, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def default_settings_path() -> Path:
    """返回默认 PID 配置文件路径。"""
    return Path.cwd() / "settings" / "pid_config.json"

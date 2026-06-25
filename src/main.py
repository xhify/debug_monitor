"""
WHEELTEC C50X 调试监视器 — 入口点

用法：
    python main.py
"""

from __future__ import annotations

from dataclasses import asdict
import json
import sys

from PySide6.QtWidgets import QApplication

from main_window import MainWindow
from rosbag_models import (
    extract_rosbag_protocol_error,
    parse_rosbag_library_state,
    parse_rosbag_recording_status,
    rosbag_protocol_data,
)
from runtime_ui_optimizations import apply_runtime_ui_optimizations


def _launch_state_key(data: dict[str, object]) -> str:
    launch_state = {
        "running": data.get("running", []),
        "detail": data.get("detail", {}),
    }
    try:
        return json.dumps(launch_state, sort_keys=True, ensure_ascii=False)
    except TypeError:
        return repr(launch_state)


def _install_launch_manager_status_filter(main_window_cls: type) -> None:
    """Avoid treating every /launch_manager/status message as a launch-state change."""

    if getattr(main_window_cls, "_launch_manager_status_filter_installed", False):
        return

    def _on_launch_manager_status(self, payload) -> None:
        if not isinstance(payload, dict):
            return

        data = rosbag_protocol_data(payload)
        protocol_error = extract_rosbag_protocol_error(payload)
        level = str(payload.get("level", ""))

        # /launch_manager/status carries both full periodic state broadcasts and
        # shorter command/query responses.  Only the full state envelope should
        # drive launch-node UI state; otherwise level=ok rosbag responses can
        # repeatedly overwrite PID/FAST-LIO/lidar button labels.
        has_launch_state = isinstance(data, dict) and (
            "running" in data or "detail" in data
        )
        if level == "state" and has_launch_state:
            launch_key = _launch_state_key(data)
            if getattr(self, "_last_launch_manager_launch_state_key", None) != launch_key:
                self._last_launch_manager_launch_state_key = launch_key
                self._ros_panel.update_launch_manager_status(data)
                self._localization_panel.update_launch_manager_status(data)

        if protocol_error:
            self._status_label.setText(f"rosbag 错误: {protocol_error}")
            self._rosbag_panel.append_log(protocol_error)

        if isinstance(data.get("rosbag"), dict):
            status = parse_rosbag_recording_status(payload)
            self._latest_rosbag_status = status
            self._rosbag_panel.update_recording_status(status)
            if self._summary_rosbag_session_id and status.session_id == self._summary_rosbag_session_id:
                self._summary_latest_rosbag_status = asdict(status)
            if status.last_error:
                self._status_label.setText(f"rosbag 错误: {status.last_error}")

        if isinstance(data.get("rosbag_library"), dict):
            library = parse_rosbag_library_state(payload)
            self._latest_rosbag_library = library
            self._rosbag_panel.update_library_state(library)

    main_window_cls._on_launch_manager_status = _on_launch_manager_status
    main_window_cls._launch_manager_status_filter_installed = True


apply_runtime_ui_optimizations(MainWindow)
_install_launch_manager_status_filter(MainWindow)


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("WHEELTEC C50X Debug Monitor")
    app.setStyle("Fusion")

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()

"""
WHEELTEC C50X 调试监视器 — 入口点

用法：
    python main.py
"""

import sys
from pathlib import Path

from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication

from main_window import MainWindow
from runtime_ui_optimizations import apply_runtime_ui_optimizations


apply_runtime_ui_optimizations(MainWindow)


def resource_path(relative_path: str) -> Path:
    """返回源码运行或 PyInstaller 打包后的资源路径。"""
    base_path = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    return base_path / relative_path


def app_icon_path() -> Path:
    return resource_path("assets/app_icon.png")


def load_app_icon() -> QIcon:
    return QIcon(str(app_icon_path()))


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("WHEELTEC C50X Debug Monitor")
    app.setStyle('Fusion')
    icon = load_app_icon()
    if not icon.isNull():
        app.setWindowIcon(icon)

    window = MainWindow()
    if not icon.isNull():
        window.setWindowIcon(icon)
    window.show()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
